"""Replace SPH_Project's rigid integrator while keeping its DFSPH kernels.

Both engines use metres, Y-up and world-space COM wrenches.  Prescribed
boundaries are sampled at the fluid-step midpoint, then committed to the
exact end pose.  The adapter also exposes reversible Newton state so a
fluid step which violates the moving-boundary CFL can be retried safely.
"""
import importlib
import json
import math
import sys
from pathlib import Path

import numpy as np
import taichi as ti
import trimesh
import newton
import warp as wp
from scipy.spatial.transform import Rotation

from .dfsph_reference import filled_lattice_alpha_reference, redistribute_short_frame_tail
from .volume_maps import build_or_load_volume_map
from .pressure_projection import divergence_pressure_update, pair_pressure_acceleration, projection_retry_decision


class NewtonRigidSolver:
    def __init__(self, container, gravity, dt):
        self.container=container; self.dt=float(dt); self.total_time=0.
        self.bodies=container.cfg.get_rigid_bodies()
        self.present_rigid_object=[]; self.bindings={}; self.drives={}
        self.body_samples={}
        self.volume_map_grids=[]
        self.last_wrenches={}; self.last_applied_wrenches={}
        self.last_prescribed_power={};self.cumulative_prescribed_work={}
        self.cumulative_fluid_impulse={}
        self.collision_geometry=[]
        sdf_options=dict(target_voxel_size=.002,narrow_band_range=(-.02,.02),
            cache_dir=str(Path(__file__).resolve().parents[2]/'output/coupled_scenes/gpu_newton_sdf_cache'))
        sdf_options.update(getattr(container,'rigid_sdf_options',{}))
        builder=newton.ModelBuilder(up_axis=newton.Axis.Y)
        if gravity[0]!=0 or gravity[2]!=0:raise ValueError('This Y-up adapter requires vertical gravity')
        builder.gravity=float(gravity[1])
        for body in self.bodies:
            oid=body['objectId']; mesh=body['mesh']
            quat=body.get('quaternion_xyzw',[0,0,0,1])
            transform=wp.transform(wp.vec3(*body['translation']),wp.quat(*quat))
            moving=body['isDynamic']; prescribed=body.get('prescribed',False)
            index=builder.add_body(xform=transform,is_kinematic=prescribed,label=str(oid)) if moving else -1
            shape=newton.Mesh(np.asarray(mesh.vertices,np.float32),np.asarray(mesh.faces,np.int32).reshape(-1),compute_inertia=moving)
            shape.build_sdf(**sdf_options)
            sdf_data=shape.sdf.to_texture_kernel_data()
            if sdf_data is None:raise RuntimeError('Rigid mesh SDF has no GPU texture data')
            if getattr(container,'volume_maps_enabled',False):
                volume_map=build_or_load_volume_map(object_id=oid,
                    vertices=np.asarray(mesh.vertices,np.float32),triangles=np.asarray(mesh.faces,np.int32),
                    texture_sdf=sdf_data,support_radius=float(container.dh),
                    voxel_size=float(container.volume_map_voxel_size),
                    cache_dir=container.volume_map_cache_dir,device='cuda:0')
                volume_map['name']=body['name']
                volume_map['dynamic']=bool(moving)
                volume_map['prescribed']=bool(prescribed)
                self.volume_map_grids.append(volume_map)
            builder.add_shape_mesh(index,mesh=shape,xform=transform if index==-1 else None,
                cfg=newton.ModelBuilder.ShapeConfig(density=float(body['density']),mu=.05,restitution=0.))
            self.collision_geometry.append(dict(object_id=oid,representation='precomputed_sdf',
                target_voxel_size_m=sdf_options['target_voxel_size'],
                actual_voxel_size_m=list(sdf_data.voxel_size),
                narrow_band_range_m=list(sdf_options['narrow_band_range']),cache_dir=str(sdf_options['cache_dir'])))
            if moving:self.bindings[oid]=(index,prescribed)
            samples=container.imported_body_samples.get(oid,np.empty((0,3),np.float32))
            self.body_samples[oid]=np.asarray(samples,np.float32).reshape(-1,3)
        self.model=builder.finalize()
        sdf_indices=self.model.shape_sdf_index.numpy()
        if np.any(sdf_indices<0):raise RuntimeError('Rigid mesh missing its precomputed SDF binding')
        for item,index in zip(self.collision_geometry,sdf_indices):item['sdf_index']=int(index)
        self.state=self.model.state();self.next_state=self.model.state()
        self.control=self.model.control();self.contacts=self.model.contacts()
        self.solver=newton.solvers.SolverXPBD(self.model,iterations=8,angular_damping=0.)
        self.com=self.model.body_com.numpy()
        self.body_axis_radii={}
        for oid,(index,_) in self.bindings.items():
            samples=self.body_samples[oid]
            arms=samples-self.com[index]
            self.body_axis_radii[oid]=np.asarray([
                np.max(np.linalg.norm(arms[:,[1,2]],axis=1)),
                np.max(np.linalg.norm(arms[:,[0,2]],axis=1)),
                np.max(np.linalg.norm(arms[:,[0,1]],axis=1))],np.float64) if len(samples) else np.zeros(3)
        velocities=self.state.body_qd.numpy()
        for body in self.bodies:
            if body['objectId'] in self.bindings:
                index,_=self.bindings[body['objectId']]
                velocities[index,:3]=body['velocity']
        if len(velocities):self.state.body_qd.assign(velocities)
        for oid,(_,prescribed) in self.bindings.items():
            if prescribed:
                self.last_prescribed_power[oid]=0.
                self.cumulative_prescribed_work[oid]=0.
                self.cumulative_fluid_impulse[oid]=np.zeros(6,np.float64)

    def set_time_step(self,dt):
        if not np.isfinite(dt) or dt<=0:raise ValueError('Newton time step must be finite and positive')
        self.dt=float(dt)

    def snapshot(self):
        return dict(q=self.state.body_q.numpy().copy(),qd=self.state.body_qd.numpy().copy(),
            f=self.state.body_f.numpy().copy(),total_time=float(self.total_time),
            last_wrenches={oid:value.copy() for oid,value in self.last_wrenches.items()},
            last_applied_wrenches={oid:value.copy() for oid,value in self.last_applied_wrenches.items()},
            last_prescribed_power=dict(self.last_prescribed_power),
            cumulative_prescribed_work=dict(self.cumulative_prescribed_work),
            cumulative_fluid_impulse={oid:value.copy() for oid,value in self.cumulative_fluid_impulse.items()})

    def restore(self,snapshot):
        self.state.body_q.assign(snapshot['q']);self.state.body_qd.assign(snapshot['qd'])
        self.state.body_f.assign(snapshot['f']);self.total_time=snapshot['total_time']
        self.last_wrenches={oid:value.copy() for oid,value in snapshot['last_wrenches'].items()}
        self.last_applied_wrenches={oid:value.copy() for oid,value in snapshot['last_applied_wrenches'].items()}
        self.last_prescribed_power=dict(snapshot['last_prescribed_power'])
        self.cumulative_prescribed_work=dict(snapshot['cumulative_prescribed_work'])
        self.cumulative_fluid_impulse={oid:value.copy() for oid,value in snapshot['cumulative_fluid_impulse'].items()}
        self.publish()

    def prepare_fluid_step(self,t):
        """Publish prescribed bodies at the time layer used by fluid kernels."""
        self.apply_drives(t);self.publish()

    def max_surface_speed(self,t=None):
        """Conservative rigid surface-speed bound including angular velocity."""
        q=self.state.body_q.numpy();qd=self.state.body_qd.numpy();maximum=0.
        for oid,(index,_) in self.bindings.items():
            body_q=q[index];body_qd=qd[index]
            if t is not None and oid in self.drives:
                body_q,body_qd=self.drives[oid](t,self.com[index])
            local_omega=Rotation.from_quat(body_q[3:]).inv().apply(body_qd[3:])
            angular_bound=np.dot(np.abs(local_omega),self.body_axis_radii[oid])
            bound=np.linalg.norm(body_qd[:3])+angular_bound
            maximum=max(maximum,float(bound))
        return maximum

    def insert_rigid_object(self):
        # All assets are present at startup; no hidden Bullet body or step.
        self.present_rigid_object=[b['objectId'] for b in self.bodies]
        self.publish()

    def publish(self):
        q=self.state.body_q.numpy();qd=self.state.body_qd.numpy();c=self.container
        if not np.isfinite(q).all() or not np.isfinite(qd).all():
            raise RuntimeError('Nonfinite Newton state')
        for body in self.bodies:
            oid=body['objectId']
            if oid in self.bindings:
                index,_=self.bindings[oid]
                origin=q[index,:3]
                rotation=Rotation.from_quat(q[index,3:]).as_matrix()
                c.rigid_body_original_centers_of_mass[oid]=self.com[index]
                c.rigid_body_centers_of_mass[oid]=origin+rotation@self.com[index]
                c.rigid_body_rotations[oid]=rotation
                c.rigid_body_velocities[oid]=qd[index,:3]
                c.rigid_body_angular_velocities[oid]=qd[index,3:]
            else:
                origin=np.asarray(body['translation'],np.float32)
                rotation=Rotation.from_quat(body.get('quaternion_xyzw',[0,0,0,1])).as_matrix()
                c.rigid_body_velocities[oid]=np.zeros(3,np.float32)
                c.rigid_body_angular_velocities[oid]=np.zeros(3,np.float32)
            c.volume_map_origins[oid]=origin
            c.volume_map_rotations[oid]=rotation

    def set_drives(self, drives):
        """Callbacks return mesh-origin q(xyz,xyzw), velocity(linear COM,angular)."""
        if any(oid not in self.bindings or not self.bindings[oid][1] for oid in drives):
            raise ValueError('A drive requires an explicitly prescribed Newton body')
        self.drives=dict(drives);self.apply_drives(0.);self.publish()

    def apply_drives(self,t):
        if not self.drives:return
        q=self.state.body_q.numpy();qd=self.state.body_qd.numpy()
        for oid,callback in self.drives.items():
            index,_=self.bindings[oid];q[index],qd[index]=callback(t,self.com[index])
        self.state.body_q.assign(q);self.state.body_qd.assign(qd)

    def step(self):
        forces=self.container.rigid_body_forces.to_numpy()
        torques=self.container.rigid_body_torques.to_numpy()
        body_velocities=self.state.body_qd.numpy()
        # Preserve external forces already supplied by the caller.
        body_forces=self.state.body_f.numpy()
        for oid,(index,prescribed) in self.bindings.items():
            wrench=np.r_[forces[oid],torques[oid]].astype(np.float32)
            if not np.isfinite(wrench).all():raise RuntimeError('Nonfinite liquid wrench')
            self.last_wrenches[oid]=wrench.copy()
            self.last_applied_wrenches[oid]=np.zeros(6,np.float32) if prescribed else wrench.copy()
            if prescribed:
                # wrench is liquid-on-rigid, so its negative is motor-on-liquid.
                power=-float(np.dot(wrench.astype(np.float64),body_velocities[index].astype(np.float64)))
                self.last_prescribed_power[oid]=power
                self.cumulative_prescribed_work[oid]+=power*self.dt
                self.cumulative_fluid_impulse[oid]-=wrench.astype(np.float64)*self.dt
            else:body_forces[index]+=wrench
        if len(body_forces):self.state.body_f.assign(body_forces)
        self.container.rigid_body_forces.fill(0.)
        self.container.rigid_body_torques.fill(0.)
        if self.bindings:
            self.model.collide(self.state,self.contacts)
            self.solver.step(self.state,self.next_state,self.control,self.contacts,self.dt)
            self.state,self.next_state=self.next_state,self.state
            self.state.clear_forces()
        self.apply_drives(self.total_time+self.dt)
        self.publish()


def create_backend(scene, fluid_positions, fluid_velocities, bodies, spacing, dt, upstream,
                   akinci_coefficient=0., minimum_density_iterations=2,
                   minimum_divergence_iterations=1, minimum_dt_divisor=64,
                   diagnose_minimum_dt_failure=False, impact_diagnostic_window=None,
                   local_residual_boundary_scope='moving',
                   density_boundary_projection_enabled=True,
                   divergence_boundary_projection_enabled=True,
                   boundary_model='sampled_particles', pressure_convergence_retries=0,
                   solver_verification_window=None, local_residual_tolerance=None):
    """bodies: local mesh/samples, world pose, density, static/free/prescribed mode."""
    if local_residual_boundary_scope not in ('moving','all'):
        raise ValueError('Local residual boundary scope must be moving or all')
    if boundary_model not in ('sampled_particles','volume_maps_bender2019'):
        raise ValueError('Boundary model must be sampled_particles or volume_maps_bender2019')
    if not 0<=pressure_convergence_retries<=4:
        raise ValueError('Pressure convergence retries must be between zero and four')
    if pressure_convergence_retries and boundary_model!='volume_maps_bender2019':
        raise ValueError('Convergence retry support currently requires Volume Maps')
    sys.path.insert(0,str(Path(upstream).resolve()))
    from SPH.utils import SimConfig
    from SPH.containers import DFSPHContainer
    from SPH.fluid_solvers import DFSPHSolver
    base_module=importlib.import_module('SPH.fluid_solvers.base_solver')
    wp.init();wp.set_device('cuda:0')
    ti.init(arch=ti.cuda,enable_fallback=False,device_memory_GB=4,offline_cache=True)
    if ti.lang.impl.current_cfg().arch!=ti.cuda:raise RuntimeError('CUDA liquid backend unavailable')
    body_lookup={b['objectId']:b for b in bodies}

    @ti.data_oriented
    class ImportedContainer(DFSPHContainer):
        def __init__(self,cfg):
            super().__init__(cfg,GGUI=False)
            self.stable_ids=ti.field(ti.i32,shape=self.particle_max_num)
            self.stable_ids_buffer=ti.field(ti.i32,shape=self.particle_max_num)
            self.snapshot_positions=ti.Vector.field(3,dtype=ti.f32,shape=self.particle_max_num)
            self.snapshot_velocities=ti.Vector.field(3,dtype=ti.f32,shape=self.particle_max_num)
            self.volume_map_origins=ti.Vector.field(3,dtype=ti.f32,shape=self.max_num_object)
            self.volume_map_rotations=ti.Matrix.field(3,3,dtype=ti.f32,shape=self.max_num_object)
            self.stable_ids.from_numpy(np.arange(self.particle_max_num,dtype=np.int32))

        def load_fluid_body(self,body,pitch=None):
            return fluid_positions.copy()

        def load_rigid_body(self,body,pitch=None):
            source=body_lookup[body['objectId']]
            mesh=trimesh.Trimesh(source['vertices'],source['triangles'],process=False)
            body['mesh']=mesh;body['restPosition']=np.asarray(mesh.vertices).copy()
            body['restCenterOfMass']=np.zeros(3)
            samples=np.asarray(source['samples'],np.float32)
            if not body['isDynamic']:
                samples=Rotation.from_quat(body['quaternion_xyzw']).apply(samples)+body['translation']
            return np.asarray(samples,np.float32)

        @ti.kernel
        def reorder_stable_ids(self):
            for i in range(self.particle_num[None]):
                self.stable_ids_buffer[self.grid_ids_new[i]]=self.stable_ids[i]
            for i in range(self.particle_num[None]):
                self.stable_ids[i]=self.stable_ids_buffer[i]

        def prepare_neighborhood_search(self):
            super().prepare_neighborhood_search()
            self.reorder_stable_ids()

        @ti.kernel
        def restore_velocities(self,values:ti.types.ndarray(dtype=ti.f32,ndim=2)):
            for i in range(self.particle_num[None]):
                if self.particle_materials[i]==1:
                    for axis in ti.static(range(3)):
                        self.particle_velocities[i][axis]=values[self.stable_ids[i],axis]

        @ti.kernel
        def save_fluid_state(self):
            for i in range(self.particle_num[None]):
                if self.particle_materials[i]==self.material_fluid:
                    stable_id=self.stable_ids[i]
                    self.snapshot_positions[stable_id]=self.particle_positions[i]
                    self.snapshot_velocities[stable_id]=self.particle_velocities[i]

        @ti.kernel
        def restore_fluid_state(self):
            for i in range(self.particle_num[None]):
                if self.particle_materials[i]==self.material_fluid:
                    stable_id=self.stable_ids[i]
                    self.particle_positions[i]=self.snapshot_positions[stable_id]
                    self.particle_velocities[i]=self.snapshot_velocities[stable_id]

    # The numerical settings match the previously unified GPU comparison.
    spec=json.loads(Path(scene).read_text())
    c=ImportedContainer(SimConfig(str(scene)))
    c.imported_body_samples={oid:np.asarray(body['samples'],np.float32) for oid,body in body_lookup.items()}
    c.rigid_sdf_options=dict(target_voxel_size=spacing/2)
    c.volume_maps_enabled=boundary_model=='volume_maps_bender2019'
    c.volume_map_voxel_size=float(spacing)/2.
    c.volume_map_cache_dir=str(Path(__file__).resolve().parents[2]/
        'output/coupled_scenes/gpu_newton_volume_map_cache')

    class CoupledSolver(DFSPHSolver):
        def __init__(self,container):
            super().__init__(container)
            self.spacing=float(spacing)
            self.volume_maps_enabled=bool(container.volume_maps_enabled)
            self.volume_map_count=len(self.rigid_solver.volume_map_grids)
            if self.volume_maps_enabled and self.volume_map_count!=len(bodies):
                raise RuntimeError('Every rigid body must have one Volume Map')
            if self.volume_maps_enabled:
                grids=self.rigid_solver.volume_map_grids
                offsets=[];total=0
                for grid in grids:
                    offsets.append(total);total+=int(np.prod(grid['dims'],dtype=np.int64))
                self.volume_map_distances=ti.field(ti.f32,shape=total)
                self.volume_map_volumes=ti.field(ti.f32,shape=total)
                self.volume_map_distances.from_numpy(np.concatenate(
                    [grid['distances'].reshape(-1) for grid in grids]).astype(np.float32))
                self.volume_map_volumes.from_numpy(np.concatenate(
                    [grid['volumes'].reshape(-1) for grid in grids]).astype(np.float32))
                self.volume_map_offsets=ti.field(ti.i32,shape=self.volume_map_count)
                self.volume_map_dims=ti.Vector.field(3,dtype=ti.i32,shape=self.volume_map_count)
                self.volume_map_lower=ti.Vector.field(3,dtype=ti.f32,shape=self.volume_map_count)
                self.volume_map_inv_voxel=ti.field(ti.f32,shape=self.volume_map_count)
                self.volume_map_object_ids=ti.field(ti.i32,shape=self.volume_map_count)
                self.volume_map_offsets.from_numpy(np.asarray(offsets,np.int32))
                self.volume_map_dims.from_numpy(np.asarray([grid['dims'] for grid in grids],np.int32))
                self.volume_map_lower.from_numpy(np.asarray([grid['lower'] for grid in grids],np.float32))
                self.volume_map_inv_voxel.from_numpy(np.asarray(
                    [1./grid['voxel_size'] for grid in grids],np.float32))
                self.volume_map_object_ids.from_numpy(np.asarray(
                    [grid['object_id'] for grid in grids],np.int32))
                map_shape=(self.container.particle_max_num,self.volume_map_count)
                self.volume_boundary_volumes=ti.field(ti.f32,shape=map_shape)
                self.volume_boundary_points=ti.Vector.field(3,dtype=ti.f32,shape=map_shape)
                self.volume_boundary_velocities=ti.Vector.field(3,dtype=ti.f32,shape=map_shape)
                self.volume_boundary_distances=ti.field(ti.f32,shape=map_shape)
                self.volume_boundary_normals=ti.Vector.field(3,dtype=ti.f32,shape=map_shape)
                # Full reference-style DFSPH pressure solve state.  Pressure is
                # accumulated while predicted velocities remain unchanged; the
                # converged acceleration is applied exactly once.
                particle_shape=self.container.particle_max_num
                self.volume_pressure=ti.field(ti.f32,shape=particle_shape)
                self.volume_pressure_acceleration=ti.Vector.field(
                    self.container.dim,dtype=ti.f32,shape=particle_shape)
                self.volume_pressure_fluid_acceleration=ti.Vector.field(
                    self.container.dim,dtype=ti.f32,shape=particle_shape)
                self.volume_pressure_boundary_acceleration=ti.Vector.field(
                    self.container.dim,dtype=ti.f32,shape=particle_shape)
                self.volume_projection_residual=ti.field(ti.f32,shape=particle_shape)
                self.volume_density_adv_raw=ti.field(ti.f32,shape=particle_shape)
                self.volume_density_change_raw=ti.field(ti.f32,shape=particle_shape)
                self.volume_divergence_active=ti.field(ti.i32,shape=particle_shape)
                self.volume_fluid_neighbor_count=ti.field(ti.i32,shape=particle_shape)
                self.volume_projection_all_boundary_error=ti.field(ti.f32,shape=())
                self.volume_projection_deficient_compressed=ti.field(ti.i32,shape=())
                self.volume_projection_global_error=ti.field(ti.f32,shape=())
                self.volume_projection_local_error=ti.field(ti.f32,shape=())
                self.volume_projection_max_pressure=ti.field(ti.f32,shape=2)
                self.volume_projection_max_acceleration=ti.field(ti.f32,shape=2)
                self.volume_projection_max_fluid_acceleration=ti.field(ti.f32,shape=2)
                self.volume_projection_max_boundary_acceleration=ti.field(ti.f32,shape=2)
                self.volume_projection_max_velocity_increment=ti.field(ti.f32,shape=2)
                self.volume_projection_max_residual=ti.field(ti.f32,shape=2)
                self.volume_map_metadata=[dict(grid['metadata'],name=grid['name'],dynamic=grid['dynamic'],
                    prescribed=grid['prescribed'],cache_path=grid['cache_path'],cache_hit=grid['cache_hit'])
                    for grid in grids]
                # Dense arrays have been copied to Taichi and need not remain duplicated in host memory.
                for grid in grids:
                    grid['distances']=None;grid['volumes']=None
            else:
                self.volume_map_metadata=[]
            self.filled_lattice_alpha_reference=filled_lattice_alpha_reference(
                self.spacing,float(container.dh),float(container.V0))
            self.base_dt=float(dt)
            self.adaptive_dt_cap=float(dt)
            self.cfl_fraction=.25
            self.max_retries=8
            self.minimum_dt=float(dt)/minimum_dt_divisor
            self.metric_max_speed_sq=ti.field(ti.f32,shape=())
            self.metric_max_displacement_sq=ti.field(ti.f32,shape=())
            self.metric_invalid=ti.field(ti.i32,shape=())
            self.metric_near_density=ti.field(ti.f32,shape=())
            self.metric_near_divergence=ti.field(ti.f32,shape=())
            self.near_moving_boundary_mask=ti.field(ti.i32,shape=self.container.particle_max_num)
            self.local_residual_all_rigid=local_residual_boundary_scope=='all'
            self.density_boundary_projection_enabled=ti.field(ti.i32,shape=())
            self.divergence_boundary_projection_enabled=ti.field(ti.i32,shape=())
            self.density_boundary_projection_enabled[None]=int(density_boundary_projection_enabled)
            self.divergence_boundary_projection_enabled[None]=int(divergence_boundary_projection_enabled)
            self.akinci_normals=ti.Vector.field(self.container.dim,dtype=ti.f32,
                shape=self.container.particle_max_num)
            # Stored by stable fluid ID so the frame diagnostic survives the
            # neighborhood sort performed later in the accepted substep.
            self.akinci_accelerations=ti.Vector.field(self.container.dim,dtype=ti.f32,
                shape=self.container.particle_max_num)
            self.akinci_coefficient=ti.field(ti.f32,shape=())
            self.akinci_enabled=False
            self.diagnose_minimum_dt_failure=bool(diagnose_minimum_dt_failure)
            self.diagnostic_stage_velocities=ti.Vector.field(self.container.dim,dtype=ti.f32,
                shape=(self.container.particle_max_num,5))
            self.failure_diagnostics=None
            self.last_rejected_step_diagnostics=None
            self.impact_diagnostic_window=impact_diagnostic_window
            self.impact_diagnostics_enabled=impact_diagnostic_window is not None
            self.impact_density_fluid_dv=ti.Vector.field(self.container.dim,dtype=ti.f32,
                shape=self.container.particle_max_num)
            self.impact_density_boundary_dv=ti.Vector.field(self.container.dim,dtype=ti.f32,
                shape=self.container.particle_max_num)
            self.impact_divergence_fluid_dv=ti.Vector.field(self.container.dim,dtype=ti.f32,
                shape=self.container.particle_max_num)
            self.impact_divergence_boundary_dv=ti.Vector.field(self.container.dim,dtype=ti.f32,
                shape=self.container.particle_max_num)
            self.impact_fluid_neighbors=ti.field(ti.i32,shape=self.container.particle_max_num)
            self.impact_rigid_neighbors=ti.field(ti.i32,shape=self.container.particle_max_num)
            self.impact_nearest_rigid_distance=ti.field(ti.f32,shape=self.container.particle_max_num)
            self.impact_nearest_rigid_object=ti.field(ti.i32,shape=self.container.particle_max_num)
            self.impact_nearest_rigid_offset=ti.Vector.field(self.container.dim,dtype=ti.f32,
                shape=self.container.particle_max_num)
            self.impact_nearest_rigid_velocity=ti.Vector.field(self.container.dim,dtype=ti.f32,
                shape=self.container.particle_max_num)
            self.impact_category_count=9
            peak_shape=(self.container.particle_max_num,self.impact_category_count)
            self.impact_peak_norm_sq=ti.field(ti.f32,shape=peak_shape)
            self.impact_peak_vector=ti.Vector.field(self.container.dim,dtype=ti.f32,shape=peak_shape)
            self.impact_peak_velocity_before=ti.Vector.field(self.container.dim,dtype=ti.f32,shape=peak_shape)
            self.impact_peak_velocity_after=ti.Vector.field(self.container.dim,dtype=ti.f32,shape=peak_shape)
            self.impact_peak_position=ti.Vector.field(self.container.dim,dtype=ti.f32,shape=peak_shape)
            self.impact_peak_time=ti.field(ti.f32,shape=peak_shape)
            self.impact_peak_dt=ti.field(ti.f32,shape=peak_shape)
            self.impact_peak_density=ti.field(ti.f32,shape=peak_shape)
            self.impact_peak_alpha=ti.field(ti.f32,shape=peak_shape)
            self.impact_peak_fluid_neighbors=ti.field(ti.i32,shape=peak_shape)
            self.impact_peak_rigid_neighbors=ti.field(ti.i32,shape=peak_shape)
            self.impact_peak_nearest_rigid_distance=ti.field(ti.f32,shape=peak_shape)
            self.impact_peak_nearest_rigid_object=ti.field(ti.i32,shape=peak_shape)
            self.impact_peak_nearest_rigid_offset=ti.Vector.field(self.container.dim,dtype=ti.f32,shape=peak_shape)
            self.impact_peak_nearest_rigid_velocity=ti.Vector.field(self.container.dim,dtype=ti.f32,shape=peak_shape)
            self.impact_density_boundary_single_iteration_norm_sq=ti.field(ti.f32,
                shape=self.container.particle_max_num)
            self.impact_density_boundary_single_iteration_vector=ti.Vector.field(self.container.dim,dtype=ti.f32,
                shape=self.container.particle_max_num)
            self.impact_density_boundary_single_iteration_index=ti.field(ti.i32,
                shape=self.container.particle_max_num)
            self.impact_divergence_boundary_single_iteration_norm_sq=ti.field(ti.f32,
                shape=self.container.particle_max_num)
            self.impact_divergence_boundary_single_iteration_vector=ti.Vector.field(self.container.dim,dtype=ti.f32,
                shape=self.container.particle_max_num)
            self.impact_divergence_boundary_single_iteration_index=ti.field(ti.i32,
                shape=self.container.particle_max_num)
            self.impact_peak_single_iteration_norm_sq=ti.field(ti.f32,shape=peak_shape)
            self.impact_peak_single_iteration_vector=ti.Vector.field(self.container.dim,dtype=ti.f32,
                shape=peak_shape)
            self.impact_peak_single_iteration_index=ti.field(ti.i32,shape=peak_shape)
            iteration_detail_shape=(self.container.particle_max_num,2)
            self.impact_projection_iteration_velocity_before=ti.Vector.field(self.container.dim,dtype=ti.f32,
                shape=self.container.particle_max_num)
            self.impact_iteration_fluid_dv=ti.Vector.field(self.container.dim,dtype=ti.f32,
                shape=iteration_detail_shape)
            self.impact_iteration_velocity_before=ti.Vector.field(self.container.dim,dtype=ti.f32,
                shape=iteration_detail_shape)
            self.impact_iteration_position=ti.Vector.field(self.container.dim,dtype=ti.f32,
                shape=iteration_detail_shape)
            self.impact_iteration_constraint=ti.field(ti.f32,shape=iteration_detail_shape)
            self.impact_iteration_kappa=ti.field(ti.f32,shape=iteration_detail_shape)
            self.impact_iteration_density=ti.field(ti.f32,shape=iteration_detail_shape)
            self.impact_iteration_alpha=ti.field(ti.f32,shape=iteration_detail_shape)
            self.impact_iteration_fluid_gradient_square_sum=ti.field(ti.f32,shape=iteration_detail_shape)
            self.impact_iteration_fluid_gradient_sum=ti.Vector.field(self.container.dim,dtype=ti.f32,
                shape=iteration_detail_shape)
            self.impact_iteration_boundary_gradient_sum=ti.Vector.field(self.container.dim,dtype=ti.f32,
                shape=iteration_detail_shape)
            self.impact_iteration_boundary_volume_sum=ti.field(ti.f32,shape=iteration_detail_shape)
            self.impact_iteration_fluid_neighbors=ti.field(ti.i32,shape=iteration_detail_shape)
            self.impact_iteration_rigid_neighbors=ti.field(ti.i32,shape=iteration_detail_shape)
            self.impact_iteration_nearest_rigid_distance=ti.field(ti.f32,shape=iteration_detail_shape)
            self.impact_iteration_nearest_rigid_object=ti.field(ti.i32,shape=iteration_detail_shape)
            self.impact_iteration_nearest_rigid_offset=ti.Vector.field(self.container.dim,dtype=ti.f32,
                shape=iteration_detail_shape)
            self.impact_iteration_nearest_rigid_velocity=ti.Vector.field(self.container.dim,dtype=ti.f32,
                shape=iteration_detail_shape)
            self.impact_peak_iteration_fluid_dv=ti.Vector.field(self.container.dim,dtype=ti.f32,
                shape=iteration_detail_shape)
            self.impact_peak_iteration_velocity_before=ti.Vector.field(self.container.dim,dtype=ti.f32,
                shape=iteration_detail_shape)
            self.impact_peak_iteration_position=ti.Vector.field(self.container.dim,dtype=ti.f32,
                shape=iteration_detail_shape)
            self.impact_peak_iteration_constraint=ti.field(ti.f32,shape=iteration_detail_shape)
            self.impact_peak_iteration_kappa=ti.field(ti.f32,shape=iteration_detail_shape)
            self.impact_peak_iteration_density=ti.field(ti.f32,shape=iteration_detail_shape)
            self.impact_peak_iteration_alpha=ti.field(ti.f32,shape=iteration_detail_shape)
            self.impact_peak_iteration_fluid_gradient_square_sum=ti.field(ti.f32,shape=iteration_detail_shape)
            self.impact_peak_iteration_fluid_gradient_sum=ti.Vector.field(self.container.dim,dtype=ti.f32,
                shape=iteration_detail_shape)
            self.impact_peak_iteration_boundary_gradient_sum=ti.Vector.field(self.container.dim,dtype=ti.f32,
                shape=iteration_detail_shape)
            self.impact_peak_iteration_boundary_volume_sum=ti.field(ti.f32,shape=iteration_detail_shape)
            self.impact_peak_iteration_fluid_neighbors=ti.field(ti.i32,shape=iteration_detail_shape)
            self.impact_peak_iteration_rigid_neighbors=ti.field(ti.i32,shape=iteration_detail_shape)
            self.impact_peak_iteration_nearest_rigid_distance=ti.field(ti.f32,shape=iteration_detail_shape)
            self.impact_peak_iteration_nearest_rigid_object=ti.field(ti.i32,shape=iteration_detail_shape)
            self.impact_peak_iteration_nearest_rigid_offset=ti.Vector.field(self.container.dim,dtype=ti.f32,
                shape=iteration_detail_shape)
            self.impact_peak_iteration_nearest_rigid_velocity=ti.Vector.field(self.container.dim,dtype=ti.f32,
                shape=iteration_detail_shape)
            self.current_projection_iteration=ti.field(ti.i32,shape=())
            self.minimum_density_iterations=int(minimum_density_iterations)
            self.minimum_divergence_iterations=int(minimum_divergence_iterations)
            self.last_density_iterations=0
            self.last_divergence_iterations=0
            self.accepted_substeps=0
            self.rejected_substeps=0
            self.frame_tail_redistributions=0
            self.minimum_accepted_dt=float('inf')
            self.maximum_accepted_dt=0.
            self.last_step_metrics={}
            self.pressure_convergence_retries=int(pressure_convergence_retries)
            self.projection_status={}
            self.pressure_rejected_substeps=0
            self.unconverged_accepted_substeps=0
            self.rollback_verified_substeps=0
            self.solver_verification_window=solver_verification_window
            self.solver_verification_enabled=solver_verification_window is not None
            self.verification_receiver_map=next((index for index,body in enumerate(bodies)
                if body['name']=='receiver'),-1)
            if self.solver_verification_enabled and not self.volume_maps_enabled:
                raise ValueError('Solver verification requires Volume Maps')
            if self.solver_verification_enabled:
                # Receiver encounter history is indexed by stable fluid ID.
                self.verification_contact_before=ti.field(ti.i32,shape=self.container.particle_max_num)
                self.verification_inbound_before=ti.field(ti.f32,shape=self.container.particle_max_num)
                self.verification_inbound_history=ti.field(ti.f32,shape=self.container.particle_max_num)
                self.verification_previous_normal_velocity=ti.field(ti.f32,shape=self.container.particle_max_num)
                self.verification_last_contact=ti.field(ti.i32,shape=self.container.particle_max_num)
                self.verification_event_inbound=ti.field(ti.f32,shape=self.container.particle_max_num)
                self.verification_event_outbound=ti.field(ti.f32,shape=self.container.particle_max_num)
                self.verification_event_time=ti.field(ti.f32,shape=self.container.particle_max_num)
                self.verification_event_count=ti.field(ti.i32,shape=self.container.particle_max_num)
                self.verification_max_components=ti.field(ti.f32,shape=4)
                self.verification_boundary_dominant=ti.field(ti.i32,shape=())
                self.verification_fluid_dominant=ti.field(ti.i32,shape=())
                self.verification_deficient_count=ti.field(ti.i32,shape=())
                self.verification_receiver_contact_count=ti.field(ti.i32,shape=())
                self.verification_component_closure_error=ti.field(ti.f32,shape=2)
                self.verification_rollback_mismatch=ti.field(ti.i32,shape=())

        @ti.kernel
        def reduce_fluid_speed(self):
            self.metric_max_speed_sq[None]=0.
            self.metric_invalid[None]=0
            for i in range(self.container.particle_num[None]):
                if self.container.particle_materials[i]==self.container.material_fluid:
                    value=self.container.particle_velocities[i].norm_sqr()
                    if ti.math.isnan(value) or ti.math.isinf(value):
                        self.metric_invalid[None]=1
                    else:
                        ti.atomic_max(self.metric_max_speed_sq[None],value)

        @ti.kernel
        def reduce_step_displacement(self):
            self.metric_max_displacement_sq[None]=0.
            self.metric_invalid[None]=0
            for i in range(self.container.particle_num[None]):
                if self.container.particle_materials[i]==self.container.material_fluid:
                    delta=self.container.particle_positions[i]-self.container.snapshot_positions[self.container.stable_ids[i]]
                    value=delta.norm_sqr()
                    if ti.math.isnan(value) or ti.math.isinf(value):
                        self.metric_invalid[None]=1
                    else:
                        ti.atomic_max(self.metric_max_displacement_sq[None],value)

        @ti.func
        def akinci_cohesion_kernel(self,distance):
            """Paper-corrected 3-D cohesion kernel C(r), support radius h."""
            value=ti.cast(0.,ti.f32)
            h=ti.cast(self.container.dh,ti.f32)
            if distance>0. and distance<=h:
                q=distance/h
                polynomial=(1.-q)*(1.-q)*(1.-q)*q*q*q
                if q<=.5:
                    polynomial=2.*polynomial-1./64.
                value=32./(np.pi*h*h*h)*polynomial
            return value

        @ti.func
        def accumulate_akinci_normal(self,p_i,p_j,normal:ti.template()):
            if self.container.particle_materials[p_j]==self.container.material_fluid:
                density_j=self.container.particle_densities[p_j]
                if density_j>0.:
                    displacement=self.container.particle_positions[p_i]-self.container.particle_positions[p_j]
                    normal+=self.container.particle_masses[p_j]/density_j*self.kernel_gradient(displacement)

        @ti.kernel
        def compute_akinci_normals(self):
            self.akinci_normals.fill(0.)
            for p_i in range(self.container.particle_num[None]):
                if self.container.particle_materials[p_i]==self.container.material_fluid:
                    normal=ti.Vector([0.0 for _ in range(self.container.dim)])
                    self.container.for_all_neighbors(p_i,self.accumulate_akinci_normal,normal)
                    # This is the area-weighted normal from the paper, not a unit normal.
                    self.akinci_normals[p_i]=self.container.dh*normal

        @ti.func
        def accumulate_akinci_acceleration(self,p_i,p_j,acceleration:ti.template()):
            if self.container.particle_materials[p_j]==self.container.material_fluid:
                displacement=self.container.particle_positions[p_i]-self.container.particle_positions[p_j]
                distance=displacement.norm()
                density_sum=(self.container.particle_densities[p_i]
                    +self.container.particle_densities[p_j])
                if distance>0. and density_sum>0.:
                    density_correction=2.*self.density_0/density_sum
                    cohesion=(self.container.particle_masses[p_j]
                        *self.akinci_cohesion_kernel(distance)*displacement/distance)
                    curvature=self.akinci_normals[p_i]-self.akinci_normals[p_j]
                    acceleration-=self.akinci_coefficient[None]*density_correction*(cohesion+curvature)

        @ti.kernel
        def add_akinci_surface_acceleration(self):
            for p_i in range(self.container.particle_num[None]):
                if self.container.particle_materials[p_i]==self.container.material_fluid:
                    acceleration=ti.Vector([0.0 for _ in range(self.container.dim)])
                    self.container.for_all_neighbors(p_i,self.accumulate_akinci_acceleration,acceleration)
                    self.container.particle_accelerations[p_i]+=acceleration
                    self.akinci_accelerations[self.container.stable_ids[p_i]]=acceleration

        @ti.kernel
        def reduce_akinci_acceleration(self)->float:
            maximum=0.
            for p_i in range(self.container.particle_num[None]):
                if self.container.particle_materials[p_i]==self.container.material_fluid:
                    stable_id=self.container.stable_ids[p_i]
                    ti.atomic_max(maximum,self.akinci_accelerations[stable_id].norm_sqr())
            return ti.sqrt(maximum)

        def compute_surface_tension_acceleration(self):
            """Replace SPH_Project's pair attraction with full Akinci 2013."""
            if not self.akinci_enabled:
                return
            self.compute_akinci_normals()
            self.add_akinci_surface_acceleration()

        @ti.func
        def sample_volume_map(self,map_index,local_position):
            result=ti.Struct(valid=0,distance=ti.cast(1.e10,ti.f32),
                gradient=ti.Vector.zero(ti.f32,3),volume=ti.cast(0.,ti.f32))
            dims=self.volume_map_dims[map_index]
            coordinates=(local_position-self.volume_map_lower[map_index])*self.volume_map_inv_voxel[map_index]
            base=ti.cast(ti.floor(coordinates),ti.i32)
            if (base[0]>=0 and base[1]>=0 and base[2]>=0
                    and base[0]<dims[0]-1 and base[1]<dims[1]-1 and base[2]<dims[2]-1):
                fraction=coordinates-ti.cast(base,ti.f32)
                offset=self.volume_map_offsets[map_index]
                i000=offset+(base[0]*dims[1]+base[1])*dims[2]+base[2]
                stride_x=dims[1]*dims[2];stride_y=dims[2]
                d000=self.volume_map_distances[i000]
                d100=self.volume_map_distances[i000+stride_x]
                d010=self.volume_map_distances[i000+stride_y]
                d110=self.volume_map_distances[i000+stride_x+stride_y]
                d001=self.volume_map_distances[i000+1]
                d101=self.volume_map_distances[i000+stride_x+1]
                d011=self.volume_map_distances[i000+stride_y+1]
                d111=self.volume_map_distances[i000+stride_x+stride_y+1]
                tx=fraction[0];ty=fraction[1];tz=fraction[2]
                one_x=1.-tx;one_y=1.-ty;one_z=1.-tz
                result.distance=(d000*one_x*one_y*one_z+d100*tx*one_y*one_z
                    +d010*one_x*ty*one_z+d110*tx*ty*one_z
                    +d001*one_x*one_y*tz+d101*tx*one_y*tz
                    +d011*one_x*ty*tz+d111*tx*ty*tz)
                inv_voxel=self.volume_map_inv_voxel[map_index]
                result.gradient=inv_voxel*ti.Vector([
                    (d100-d000)*one_y*one_z+(d110-d010)*ty*one_z
                        +(d101-d001)*one_y*tz+(d111-d011)*ty*tz,
                    (d010-d000)*one_x*one_z+(d110-d100)*tx*one_z
                        +(d011-d001)*one_x*tz+(d111-d101)*tx*tz,
                    (d001-d000)*one_x*one_y+(d101-d100)*tx*one_y
                        +(d011-d010)*one_x*ty+(d111-d110)*tx*ty])
                v000=self.volume_map_volumes[i000]
                v100=self.volume_map_volumes[i000+stride_x]
                v010=self.volume_map_volumes[i000+stride_y]
                v110=self.volume_map_volumes[i000+stride_x+stride_y]
                v001=self.volume_map_volumes[i000+1]
                v101=self.volume_map_volumes[i000+stride_x+1]
                v011=self.volume_map_volumes[i000+stride_y+1]
                v111=self.volume_map_volumes[i000+stride_x+stride_y+1]
                result.volume=(v000*one_x*one_y*one_z+v100*tx*one_y*one_z
                    +v010*one_x*ty*one_z+v110*tx*ty*one_z
                    +v001*one_x*one_y*tz+v101*tx*one_y*tz
                    +v011*one_x*ty*tz+v111*tx*ty*tz)
                result.valid=1
            return result

        @ti.kernel
        def compute_volume_boundary_samples(self):
            for i in range(self.container.particle_num[None]):
                if self.container.particle_materials[i]==self.container.material_fluid:
                    position=self.container.particle_positions[i]
                    for map_index in ti.static(range(self.volume_map_count)):
                        self.volume_boundary_volumes[i,map_index]=0.
                        self.volume_boundary_points[i,map_index]=ti.Vector.zero(ti.f32,3)
                        self.volume_boundary_velocities[i,map_index]=ti.Vector.zero(ti.f32,3)
                        object_id=self.volume_map_object_ids[map_index]
                        rotation=self.container.volume_map_rotations[object_id]
                        local_position=rotation.transpose()@(
                            position-self.container.volume_map_origins[object_id])
                        sample=self.sample_volume_map(map_index,local_position)
                        gradient_norm=sample.gradient.norm()
                        self.volume_boundary_distances[i,map_index]=sample.distance
                        self.volume_boundary_normals[i,map_index]=ti.Vector.zero(ti.f32,3)
                        if sample.valid!=0 and gradient_norm>1.e-6:
                            self.volume_boundary_normals[i,map_index]=rotation@(sample.gradient/gradient_norm)
                        if (sample.valid!=0 and sample.distance>0.
                                and sample.distance<self.container.dh
                                and sample.volume>0. and gradient_norm>1.e-6):
                            world_normal=rotation@(sample.gradient/gradient_norm)
                            # Match the authors' reference implementation: x* is shifted
                            # half a particle radius into the solid and kept at least one
                            # particle spacing from the fluid sample.
                            proxy_distance=ti.max(sample.distance+.25*self.spacing,self.spacing)
                            boundary_point=position-proxy_distance*world_normal
                            boundary_velocity=ti.Vector.zero(ti.f32,3)
                            if self.container.rigid_body_is_dynamic[object_id]:
                                center=self.container.rigid_body_centers_of_mass[object_id]
                                boundary_velocity=self.container.rigid_body_velocities[object_id]+ti.math.cross(
                                    self.container.rigid_body_angular_velocities[object_id],boundary_point-center)
                            self.volume_boundary_volumes[i,map_index]=sample.volume
                            self.volume_boundary_points[i,map_index]=boundary_point
                            self.volume_boundary_velocities[i,map_index]=boundary_velocity

        def refresh_volume_boundary_samples(self):
            if self.volume_maps_enabled:
                self.compute_volume_boundary_samples()

        @ti.func
        def compute_density_task(self,p_i,p_j,ret:ti.template()):
            material=self.container.particle_materials[p_j]
            if material==self.container.material_fluid or (ti.static(not self.volume_maps_enabled)
                    and material==self.container.material_rigid):
                displacement=self.container.particle_positions[p_i]-self.container.particle_positions[p_j]
                ret+=self.container.particle_rest_volumes[p_j]*self.kernel_W(displacement.norm())

        @ti.kernel
        def compute_density(self):
            for p_i in range(self.container.particle_num[None]):
                if self.container.particle_materials[p_i]==self.container.material_fluid:
                    density=self.container.particle_rest_volumes[p_i]*self.kernel_W(0.)
                    self.container.for_all_neighbors(p_i,self.compute_density_task,density)
                    if ti.static(self.volume_maps_enabled):
                        position=self.container.particle_positions[p_i]
                        for map_index in ti.static(range(self.volume_map_count)):
                            volume=self.volume_boundary_volumes[p_i,map_index]
                            if volume>0.:
                                density+=volume*self.kernel_W((position-
                                    self.volume_boundary_points[p_i,map_index]).norm())
                    self.container.particle_densities[p_i]=density*self.density_0

        @ti.func
        def compute_alpha_task(self,p_i,p_j,ret:ti.template()):
            material=self.container.particle_materials[p_j]
            if material==self.container.material_fluid:
                grad_p_j=-self.container.particle_rest_volumes[p_j]*self.kernel_gradient(
                    self.container.particle_positions[p_i]-self.container.particle_positions[p_j])
                ret[self.container.dim]+=grad_p_j.norm_sqr()
                for axis in ti.static(range(self.container.dim)):
                    ret[axis]+=grad_p_j[axis]
            elif ti.static(not self.volume_maps_enabled) and material==self.container.material_rigid:
                grad_p_j=-self.container.particle_rest_volumes[p_j]*self.kernel_gradient(
                    self.container.particle_positions[p_i]-self.container.particle_positions[p_j])
                for axis in ti.static(range(self.container.dim)):
                    ret[axis]+=grad_p_j[axis]

        @ti.kernel
        def compute_alpha(self):
            for p_i in range(self.container.particle_num[None]):
                if self.container.particle_materials[p_i]==self.container.material_fluid:
                    ret=ti.Vector.zero(ti.f32,self.container.dim+1)
                    self.container.for_all_neighbors(p_i,self.compute_alpha_task,ret)
                    grad_p_i=ti.Vector([ret[axis] for axis in ti.static(range(self.container.dim))])
                    if ti.static(self.volume_maps_enabled):
                        position=self.container.particle_positions[p_i]
                        for map_index in ti.static(range(self.volume_map_count)):
                            volume=self.volume_boundary_volumes[p_i,map_index]
                            if volume>0.:
                                grad_p_i-=volume*self.kernel_gradient(
                                    position-self.volume_boundary_points[p_i,map_index])
                    denominator=ret[self.container.dim]+grad_p_i.norm_sqr()
                    self.container.particle_dfsph_alphas[p_i]=(1./denominator if denominator>1.e-5 else 0.)

        @ti.func
        def compute_density_derivative_task(self,p_i,p_j,ret:ti.template()):
            material=self.container.particle_materials[p_j]
            if material==self.container.material_fluid or (ti.static(not self.volume_maps_enabled)
                    and material==self.container.material_rigid):
                ret.density_adv+=self.container.particle_rest_volumes[p_j]*ti.math.dot(
                    self.container.particle_velocities[p_i]-self.container.particle_velocities[p_j],
                    self.kernel_gradient(self.container.particle_positions[p_i]-
                        self.container.particle_positions[p_j]))
                ret.num_neighbors+=1

        @ti.kernel
        def compute_density_derivative(self):
            for p_i in range(self.container.particle_num[None]):
                if self.container.particle_materials[p_i]==self.container.material_fluid:
                    ret=ti.Struct(density_adv=ti.cast(0.,ti.f32),num_neighbors=0)
                    self.container.for_all_neighbors(p_i,self.compute_density_derivative_task,ret)
                    if ti.static(self.volume_maps_enabled):
                        position=self.container.particle_positions[p_i]
                        velocity=self.container.particle_velocities[p_i]
                        for map_index in ti.static(range(self.volume_map_count)):
                            volume=self.volume_boundary_volumes[p_i,map_index]
                            if volume>0.:
                                ret.density_adv+=volume*ti.math.dot(velocity-
                                    self.volume_boundary_velocities[p_i,map_index],self.kernel_gradient(
                                        position-self.volume_boundary_points[p_i,map_index]))
                    density_adv=ti.max(ret.density_adv,0.)
                    if ti.static(self.container.dim==3):
                        if ret.num_neighbors<20:density_adv=0.
                    else:
                        if ret.num_neighbors<7:density_adv=0.
                    self.container.particle_densities_derivatives[p_i]=density_adv

        @ti.func
        def compute_density_star_task(self,p_i,p_j,ret:ti.template()):
            material=self.container.particle_materials[p_j]
            if material==self.container.material_fluid or (ti.static(not self.volume_maps_enabled)
                    and material==self.container.material_rigid):
                ret+=self.container.particle_rest_volumes[p_j]*ti.math.dot(
                    self.container.particle_velocities[p_i]-self.container.particle_velocities[p_j],
                    self.kernel_gradient(self.container.particle_positions[p_i]-
                        self.container.particle_positions[p_j]))

        @ti.kernel
        def compute_density_star(self):
            for p_i in range(self.container.particle_num[None]):
                if self.container.particle_materials[p_i]==self.container.material_fluid:
                    delta=ti.cast(0.,ti.f32)
                    self.container.for_all_neighbors(p_i,self.compute_density_star_task,delta)
                    if ti.static(self.volume_maps_enabled):
                        position=self.container.particle_positions[p_i]
                        velocity=self.container.particle_velocities[p_i]
                        for map_index in ti.static(range(self.volume_map_count)):
                            volume=self.volume_boundary_volumes[p_i,map_index]
                            if volume>0.:
                                delta+=volume*ti.math.dot(velocity-
                                    self.volume_boundary_velocities[p_i,map_index],self.kernel_gradient(
                                        position-self.volume_boundary_points[p_i,map_index]))
                    density_adv=self.container.particle_densities[p_i]/self.density_0+self.dt[None]*delta
                    self.container.particle_densities_star[p_i]=ti.max(density_adv,1.)

        @ti.func
        def count_moving_boundary_neighbor(self,p_i,p_j,ret:ti.template()):
            if self.container.particle_materials[p_j]==self.container.material_rigid:
                if ti.static(self.local_residual_all_rigid):
                    ret+=1
                elif self.container.particle_is_dynamic[p_j]:
                    ret+=1

        @ti.func
        def correct_density_error_task(self,p_i,p_j,k_i:ti.template()):
            if self.container.particle_materials[p_j]==self.container.material_fluid:
                k_j=self.container.particle_dfsph_kappa[p_j]
                if ti.abs(k_i+k_j)>self.m_eps*self.dt[None]:
                    grad=self.container.particle_rest_volumes[p_j]*self.kernel_gradient(
                        self.container.particle_positions[p_i]-self.container.particle_positions[p_j])
                    self.container.particle_velocities[p_i]-=grad*(k_i/self.container.particle_densities[p_i]
                        +k_j/self.container.particle_densities[p_j])*self.density_0
            elif (ti.static(not self.volume_maps_enabled)
                    and self.container.particle_materials[p_j]==self.container.material_rigid
                    and self.density_boundary_projection_enabled[None]!=0):
                den_i=self.container.particle_densities[p_i]
                if ti.abs(k_i)>self.m_eps*self.dt[None]:
                    grad=self.container.particle_rest_volumes[p_j]*self.kernel_gradient(
                        self.container.particle_positions[p_i]-self.container.particle_positions[p_j])
                    self.container.particle_velocities[p_i]-=grad*(k_i/den_i)*self.density_0
                    if self.container.particle_is_dynamic[p_j]:
                        object_j=self.container.particle_object_ids[p_j]
                        center=self.container.rigid_body_centers_of_mass[object_j]
                        force=grad*(k_i/den_i)*self.density_0/self.dt[None]*(
                            self.container.particle_rest_volumes[p_i]*self.density_0)
                        self.container.rigid_body_forces[object_j]+=force
                        self.container.rigid_body_torques[object_j]+=ti.math.cross(
                            self.container.particle_positions[p_j]-center,force)

        @ti.func
        def correct_divergence_task(self,p_i,p_j,ret:ti.template()):
            if self.container.particle_materials[p_j]==self.container.material_fluid:
                k_j=self.container.particle_dfsph_kappa_v[p_j]
                k_sum=ret.k_i+k_j
                if ti.abs(k_sum)>self.m_eps*self.dt[None]:
                    grad=self.container.particle_rest_volumes[p_j]*self.kernel_gradient(
                        self.container.particle_positions[p_i]-self.container.particle_positions[p_j])
                    ret.dv-=grad*(ret.k_i/self.container.particle_densities[p_i]
                        +k_j/self.container.particle_densities[p_j])*self.density_0
            elif (ti.static(not self.volume_maps_enabled)
                    and self.container.particle_materials[p_j]==self.container.material_rigid
                    and self.divergence_boundary_projection_enabled[None]!=0):
                den_i=self.container.particle_densities[p_i]
                if ti.abs(ret.k_i)>self.m_eps*self.dt[None]:
                    grad=self.container.particle_rest_volumes[p_j]*self.kernel_gradient(
                        self.container.particle_positions[p_i]-self.container.particle_positions[p_j])
                    ret.dv-=grad*(ret.k_i/den_i)*self.density_0
                    if self.container.particle_is_dynamic[p_j]:
                        object_j=self.container.particle_object_ids[p_j]
                        center=self.container.rigid_body_centers_of_mass[object_j]
                        force=grad*(ret.k_i/den_i)*self.density_0/self.dt[None]*(
                            self.container.particle_rest_volumes[p_i]*self.density_0)
                        self.container.rigid_body_forces[object_j]+=force
                        self.container.rigid_body_torques[object_j]+=ti.math.cross(
                            self.container.particle_positions[p_j]-center,force)

        @ti.func
        def apply_volume_density_correction(self,p_i,k_i):
            if self.density_boundary_projection_enabled[None]!=0:
                if ti.abs(k_i)>self.m_eps*self.dt[None]:
                    density_i=self.container.particle_densities[p_i]
                    position=self.container.particle_positions[p_i]
                    for map_index in ti.static(range(self.volume_map_count)):
                        volume=self.volume_boundary_volumes[p_i,map_index]
                        if volume>0.:
                            boundary_point=self.volume_boundary_points[p_i,map_index]
                            grad=volume*self.kernel_gradient(position-boundary_point)
                            self.container.particle_velocities[p_i]-=grad*(k_i/density_i)*self.density_0
                            object_id=self.volume_map_object_ids[map_index]
                            if self.container.rigid_body_is_dynamic[object_id]:
                                force=grad*(k_i/density_i)*self.density_0/self.dt[None]*(
                                    self.container.particle_rest_volumes[p_i]*self.density_0)
                                center=self.container.rigid_body_centers_of_mass[object_id]
                                self.container.rigid_body_forces[object_id]+=force
                                self.container.rigid_body_torques[object_id]+=ti.math.cross(
                                    boundary_point-center,force)

        @ti.kernel
        def correct_density_error_step(self):
            for p_i in range(self.container.particle_num[None]):
                if self.container.particle_materials[p_i]==self.container.material_fluid:
                    k_i=self.container.particle_dfsph_kappa[p_i]
                    self.container.for_all_neighbors(p_i,self.correct_density_error_task,k_i)
                    if ti.static(self.volume_maps_enabled):
                        self.apply_volume_density_correction(p_i,k_i)

        @ti.func
        def accumulate_volume_divergence_correction(self,p_i,ret:ti.template()):
            if self.divergence_boundary_projection_enabled[None]!=0:
                if ti.abs(ret.k_i)>self.m_eps*self.dt[None]:
                    density_i=self.container.particle_densities[p_i]
                    position=self.container.particle_positions[p_i]
                    for map_index in ti.static(range(self.volume_map_count)):
                        volume=self.volume_boundary_volumes[p_i,map_index]
                        if volume>0.:
                            boundary_point=self.volume_boundary_points[p_i,map_index]
                            grad=volume*self.kernel_gradient(position-boundary_point)
                            ret.dv-=grad*(ret.k_i/density_i)*self.density_0
                            object_id=self.volume_map_object_ids[map_index]
                            if self.container.rigid_body_is_dynamic[object_id]:
                                force=grad*(ret.k_i/density_i)*self.density_0/self.dt[None]*(
                                    self.container.particle_rest_volumes[p_i]*self.density_0)
                                center=self.container.rigid_body_centers_of_mass[object_id]
                                self.container.rigid_body_forces[object_id]+=force
                                self.container.rigid_body_torques[object_id]+=ti.math.cross(
                                    boundary_point-center,force)

        @ti.kernel
        def correct_divergence_step(self):
            for p_i in range(self.container.particle_num[None]):
                if self.container.particle_materials[p_i]==self.container.material_fluid:
                    ret=ti.Struct(dv=ti.Vector.zero(ti.f32,self.container.dim),
                        k_i=self.container.particle_dfsph_kappa_v[p_i])
                    self.container.for_all_neighbors(p_i,self.correct_divergence_task,ret)
                    if ti.static(self.volume_maps_enabled):
                        self.accumulate_volume_divergence_correction(p_i,ret)
                    self.container.particle_velocities[p_i]+=ret.dv

        @ti.func
        def accumulate_volume_pressure_acceleration_task(self,p_i,p_j,ret:ti.template()):
            if self.container.particle_materials[p_j]==self.container.material_fluid:
                grad=self.container.particle_rest_volumes[p_j]*self.kernel_gradient(
                    self.container.particle_positions[p_i]-self.container.particle_positions[p_j])
                ret.fluid+=pair_pressure_acceleration(grad,self.volume_pressure[p_i],self.volume_pressure[p_j])

        @ti.kernel
        def compute_volume_pressure_acceleration(self,boundary_enabled:ti.i32):
            for i in range(self.container.particle_num[None]):
                if self.container.particle_materials[i]==self.container.material_fluid:
                    ret=ti.Struct(fluid=ti.Vector.zero(ti.f32,self.container.dim))
                    self.container.for_all_neighbors(i,self.accumulate_volume_pressure_acceleration_task,ret)
                    boundary=ti.Vector.zero(ti.f32,self.container.dim)
                    if boundary_enabled!=0 and ti.abs(self.volume_pressure[i])>self.m_eps:
                        position=self.container.particle_positions[i]
                        for map_index in ti.static(range(self.volume_map_count)):
                            volume=self.volume_boundary_volumes[i,map_index]
                            if volume>0.:
                                boundary-=volume*self.kernel_gradient(
                                    position-self.volume_boundary_points[i,map_index])*self.volume_pressure[i]
                    self.volume_pressure_fluid_acceleration[i]=ret.fluid
                    self.volume_pressure_boundary_acceleration[i]=boundary
                    self.volume_pressure_acceleration[i]=ret.fluid+boundary

        @ti.func
        def accumulate_volume_pressure_ap_task(self,p_i,p_j,ret:ti.template()):
            if self.container.particle_materials[p_j]==self.container.material_fluid:
                grad=self.container.particle_rest_volumes[p_j]*self.kernel_gradient(
                    self.container.particle_positions[p_i]-self.container.particle_positions[p_j])
                ret.ap+=ti.math.dot(self.volume_pressure_acceleration[p_i]-
                    self.volume_pressure_acceleration[p_j],grad)

        @ti.func
        def accumulate_volume_density_rate_task(self,p_i,p_j,ret:ti.template()):
            if self.container.particle_materials[p_j]==self.container.material_fluid:
                ret.rate+=self.container.particle_rest_volumes[p_j]*ti.math.dot(
                    self.container.particle_velocities[p_i]-self.container.particle_velocities[p_j],
                    self.kernel_gradient(self.container.particle_positions[p_i]-
                        self.container.particle_positions[p_j]))
                ret.neighbors+=1

        @ti.kernel
        def compute_volume_raw_constraints(self):
            for i in range(self.container.particle_num[None]):
                if self.container.particle_materials[i]==self.container.material_fluid:
                    ret=ti.Struct(rate=ti.cast(0.,ti.f32),neighbors=0)
                    self.container.for_all_neighbors(i,self.accumulate_volume_density_rate_task,ret)
                    position=self.container.particle_positions[i]
                    velocity=self.container.particle_velocities[i]
                    for map_index in ti.static(range(self.volume_map_count)):
                        volume=self.volume_boundary_volumes[i,map_index]
                        if volume>0.:
                            ret.rate+=volume*ti.math.dot(velocity-
                                self.volume_boundary_velocities[i,map_index],self.kernel_gradient(
                                    position-self.volume_boundary_points[i,map_index]))
                    self.volume_density_change_raw[i]=ret.rate
                    self.volume_density_adv_raw[i]=(self.container.particle_densities[i]/self.density_0+
                        self.dt[None]*ret.rate)
                    active=1
                    if ti.static(self.container.dim==3):
                        if ret.neighbors<20:active=0
                    else:
                        if ret.neighbors<7:active=0
                    self.volume_divergence_active[i]=active
                    self.volume_fluid_neighbor_count[i]=ret.neighbors

        @ti.kernel
        def initialize_volume_density_pressure(self):
            inv_dt2=1./(self.dt[None]*self.dt[None])
            self.volume_projection_deficient_compressed[None]=0
            for i in range(self.container.particle_num[None]):
                if self.container.particle_materials[i]==self.container.material_fluid:
                    error=ti.max(self.volume_density_adv_raw[i]-1.,0.)
                    self.volume_pressure[i]=error*self.container.particle_dfsph_alphas[i]*inv_dt2
                    self.volume_projection_residual[i]=error
                    if self.volume_divergence_active[i]==0 and error>0.:
                        ti.atomic_add(self.volume_projection_deficient_compressed[None],1)

        @ti.kernel
        def update_volume_density_pressure(self,boundary_enabled:ti.i32):
            self.volume_projection_global_error[None]=0.
            self.volume_projection_local_error[None]=0.
            inv_dt2=1./(self.dt[None]*self.dt[None])
            particle_count=ti.cast(self.container.particle_num[None],ti.f32)
            for i in range(self.container.particle_num[None]):
                if self.container.particle_materials[i]==self.container.material_fluid:
                    ret=ti.Struct(ap=ti.cast(0.,ti.f32))
                    self.container.for_all_neighbors(i,self.accumulate_volume_pressure_ap_task,ret)
                    if boundary_enabled!=0:
                        position=self.container.particle_positions[i]
                        acceleration=self.volume_pressure_acceleration[i]
                        for map_index in ti.static(range(self.volume_map_count)):
                            volume=self.volume_boundary_volumes[i,map_index]
                            if volume>0.:
                                ret.ap+=volume*ti.math.dot(acceleration,self.kernel_gradient(
                                    position-self.volume_boundary_points[i,map_index]))
                    source=1.-self.volume_density_adv_raw[i]
                    equation_residual=source-self.dt[None]*self.dt[None]*ret.ap
                    error=-ti.min(equation_residual,0.)
                    self.volume_projection_residual[i]=error
                    self.volume_pressure[i]=ti.max(self.volume_pressure[i]-.5*equation_residual*
                        self.container.particle_dfsph_alphas[i]*inv_dt2,0.)
                    ti.atomic_add(self.volume_projection_global_error[None],error/particle_count)
                    if self.near_moving_boundary_mask[i]:
                        ti.atomic_max(self.volume_projection_local_error[None],error)

        @ti.kernel
        def initialize_volume_divergence_pressure(self):
            inv_dt=1./self.dt[None]
            for i in range(self.container.particle_num[None]):
                if self.container.particle_materials[i]==self.container.material_fluid:
                    derivative=ti.max(self.volume_density_change_raw[i],0.)
                    if self.volume_divergence_active[i]==0:derivative=0.
                    self.volume_pressure[i]=derivative*self.container.particle_dfsph_alphas[i]*inv_dt
                    self.volume_projection_residual[i]=derivative

        @ti.kernel
        def update_volume_divergence_pressure(self,boundary_enabled:ti.i32):
            self.volume_projection_global_error[None]=0.
            self.volume_projection_local_error[None]=0.
            inv_dt=1./self.dt[None]
            particle_count=ti.cast(self.container.particle_num[None],ti.f32)
            for i in range(self.container.particle_num[None]):
                if self.container.particle_materials[i]==self.container.material_fluid:
                    ret=ti.Struct(ap=ti.cast(0.,ti.f32))
                    self.container.for_all_neighbors(i,self.accumulate_volume_pressure_ap_task,ret)
                    if boundary_enabled!=0:
                        position=self.container.particle_positions[i]
                        acceleration=self.volume_pressure_acceleration[i]
                        for map_index in ti.static(range(self.volume_map_count)):
                            volume=self.volume_boundary_volumes[i,map_index]
                            if volume>0.:
                                ret.ap+=volume*ti.math.dot(acceleration,self.kernel_gradient(
                                    position-self.volume_boundary_points[i,map_index]))
                    source=-self.volume_density_change_raw[i]
                    equation_residual=source-self.dt[None]*ret.ap
                    pressure,error=divergence_pressure_update(self.volume_pressure[i],source,
                        self.dt[None]*ret.ap,self.container.particle_dfsph_alphas[i],
                        self.dt[None],self.volume_divergence_active[i])
                    self.volume_projection_residual[i]=error
                    self.volume_pressure[i]=pressure
                    ti.atomic_add(self.volume_projection_global_error[None],error/particle_count)
                    if self.near_moving_boundary_mask[i]:
                        ti.atomic_max(self.volume_projection_local_error[None],error)

        @ti.kernel
        def evaluate_volume_pressure_residual(self,boundary_enabled:ti.i32,projection_index:ti.i32):
            # Evaluate the FINAL pressure vector without another Jacobi update.
            self.volume_projection_global_error[None]=0.
            self.volume_projection_local_error[None]=0.
            self.volume_projection_all_boundary_error[None]=0.
            particle_count=ti.cast(self.container.particle_num[None],ti.f32)
            for i in range(self.container.particle_num[None]):
                if self.container.particle_materials[i]==self.container.material_fluid:
                    ret=ti.Struct(ap=ti.cast(0.,ti.f32))
                    self.container.for_all_neighbors(i,self.accumulate_volume_pressure_ap_task,ret)
                    contact=0
                    position=self.container.particle_positions[i]
                    for map_index in ti.static(range(self.volume_map_count)):
                        volume=self.volume_boundary_volumes[i,map_index]
                        if volume>0.:
                            contact=1
                            if boundary_enabled!=0:
                                ret.ap+=volume*ti.math.dot(self.volume_pressure_acceleration[i],
                                    self.kernel_gradient(position-self.volume_boundary_points[i,map_index]))
                    error=ti.max(self.volume_density_adv_raw[i]-1.+self.dt[None]*self.dt[None]*ret.ap,0.)
                    if projection_index==1:
                        error=ti.max(self.volume_density_change_raw[i]+self.dt[None]*ret.ap,0.)
                        if self.volume_divergence_active[i]==0:error=0.
                    self.volume_projection_residual[i]=error
                    ti.atomic_add(self.volume_projection_global_error[None],error/particle_count)
                    if self.near_moving_boundary_mask[i]:
                        ti.atomic_max(self.volume_projection_local_error[None],error)
                    if contact!=0:
                        ti.atomic_max(self.volume_projection_all_boundary_error[None],error)

        @ti.kernel
        def apply_volume_pressure_solution(self,boundary_enabled:ti.i32):
            for i in range(self.container.particle_num[None]):
                if self.container.particle_materials[i]==self.container.material_fluid:
                    self.container.particle_velocities[i]+=self.dt[None]*self.volume_pressure_acceleration[i]
                    if boundary_enabled!=0 and ti.abs(self.volume_pressure[i])>self.m_eps:
                        position=self.container.particle_positions[i]
                        mass_i=self.container.particle_rest_volumes[i]*self.density_0
                        for map_index in ti.static(range(self.volume_map_count)):
                            volume=self.volume_boundary_volumes[i,map_index]
                            if volume>0.:
                                boundary_point=self.volume_boundary_points[i,map_index]
                                grad=volume*self.kernel_gradient(position-boundary_point)
                                object_id=self.volume_map_object_ids[map_index]
                                if self.container.rigid_body_is_dynamic[object_id]:
                                    force=grad*self.volume_pressure[i]*mass_i
                                    center=self.container.rigid_body_centers_of_mass[object_id]
                                    self.container.rigid_body_forces[object_id]+=force
                                    self.container.rigid_body_torques[object_id]+=ti.math.cross(
                                        boundary_point-center,force)

        @ti.kernel
        def capture_volume_projection_statistics(self,projection_index:ti.i32):
            self.volume_projection_max_pressure[projection_index]=0.
            self.volume_projection_max_acceleration[projection_index]=0.
            self.volume_projection_max_fluid_acceleration[projection_index]=0.
            self.volume_projection_max_boundary_acceleration[projection_index]=0.
            self.volume_projection_max_velocity_increment[projection_index]=0.
            self.volume_projection_max_residual[projection_index]=0.
            for i in range(self.container.particle_num[None]):
                if self.container.particle_materials[i]==self.container.material_fluid:
                    ti.atomic_max(self.volume_projection_max_pressure[projection_index],
                        self.volume_pressure[i])
                    ti.atomic_max(self.volume_projection_max_acceleration[projection_index],
                        self.volume_pressure_acceleration[i].norm())
                    ti.atomic_max(self.volume_projection_max_fluid_acceleration[projection_index],
                        self.volume_pressure_fluid_acceleration[i].norm())
                    ti.atomic_max(self.volume_projection_max_boundary_acceleration[projection_index],
                        self.volume_pressure_boundary_acceleration[i].norm())
                    ti.atomic_max(self.volume_projection_max_velocity_increment[projection_index],
                        self.dt[None]*self.volume_pressure_acceleration[i].norm())
                    ti.atomic_max(self.volume_projection_max_residual[projection_index],
                        self.volume_projection_residual[i])

        @ti.kernel
        def mark_near_moving_boundary(self):
            self.near_moving_boundary_mask.fill(0)
            for i in range(self.container.particle_num[None]):
                if self.container.particle_materials[i]==self.container.material_fluid:
                    if ti.static(self.volume_maps_enabled):
                        for map_index in ti.static(range(self.volume_map_count)):
                            object_id=self.volume_map_object_ids[map_index]
                            if (self.volume_boundary_volumes[i,map_index]>0.
                                    and (ti.static(self.local_residual_all_rigid)
                                        or self.container.rigid_body_is_dynamic[object_id])):
                                self.near_moving_boundary_mask[i]=1
                    else:
                        moving_neighbors=0
                        self.container.for_all_neighbors(i,self.count_moving_boundary_neighbor,moving_neighbors)
                        if moving_neighbors>0:self.near_moving_boundary_mask[i]=1

        @ti.kernel
        def reduce_near_density_star(self)->float:
            maximum=0.
            for i in range(self.container.particle_num[None]):
                if self.near_moving_boundary_mask[i]:
                    ti.atomic_max(maximum,ti.max(0.,self.container.particle_densities_star[i]-1.))
            return maximum

        @ti.kernel
        def reduce_near_divergence(self)->float:
            maximum=0.
            for i in range(self.container.particle_num[None]):
                if self.near_moving_boundary_mask[i]:
                    ti.atomic_max(maximum,self.container.particle_densities_derivatives[i])
            return maximum

        @ti.kernel
        def reduce_near_boundary_residuals(self):
            self.metric_near_density[None]=0.
            self.metric_near_divergence[None]=0.
            for i in range(self.container.particle_num[None]):
                if self.near_moving_boundary_mask[i]:
                    ti.atomic_max(self.metric_near_density[None],ti.max(0.,self.container.particle_densities[i]/self.density_0-1.))
                    ti.atomic_max(self.metric_near_divergence[None],self.container.particle_densities_derivatives[i])

        @ti.kernel
        def snapshot_projection_iteration_velocity(self):
            for i in range(self.container.particle_num[None]):
                if self.container.particle_materials[i]==self.container.material_fluid:
                    self.impact_projection_iteration_velocity_before[self.container.stable_ids[i]]=self.container.particle_velocities[i]

        def correct_density_error(self):
            if self.volume_maps_enabled:
                return self.correct_volume_density_error()
            self.compute_density_star()
            for iteration in range(self.m_max_iterations):
                self.compute_kappa()
                if self.impact_diagnostics_enabled:
                    self.current_projection_iteration[None]=iteration+1
                    self.snapshot_projection_iteration_velocity()
                    self.accumulate_density_projection_components()
                self.correct_density_error_step();self.compute_density_star()
                global_error=self.compute_density_error()
                local_error=self.reduce_near_density_star()
                if (global_error<=self.max_error and local_error<=self.max_error
                        and iteration+1>=self.minimum_density_iterations):break
            self.last_density_iterations=iteration+1
            self.last_local_density_projection_error=float(local_error)

        def correct_divergence_error(self):
            if self.volume_maps_enabled:
                return self.correct_volume_divergence_error()
            self.compute_density_derivative()
            eta=self.max_error_V*self.density_0/self.dt[None]
            local_eta=self.max_error_V/self.dt[None]
            for iteration in range(self.m_max_iterations_v):
                self.compute_kappa_v()
                if self.impact_diagnostics_enabled:
                    self.current_projection_iteration[None]=iteration+1
                    self.snapshot_projection_iteration_velocity()
                    self.accumulate_divergence_projection_components()
                self.correct_divergence_step();self.compute_density_derivative()
                global_error=self.compute_density_derivative_error()
                local_error=self.reduce_near_divergence()
                if (global_error<=eta and local_error<=local_eta
                        and iteration+1>=self.minimum_divergence_iterations):break
            self.last_divergence_iterations=iteration+1
            self.last_local_divergence_projection_error=float(local_error)

        def correct_volume_density_error(self):
            self.compute_volume_raw_constraints()
            self.initialize_volume_density_pressure()
            boundary_enabled=int(self.density_boundary_projection_enabled[None])
            global_error=float('inf');local_error=float('inf')
            for iteration in range(self.m_max_iterations):
                self.compute_volume_pressure_acceleration(boundary_enabled)
                self.update_volume_density_pressure(boundary_enabled)
                global_error=float(self.volume_projection_global_error[None])
                local_error=float(self.volume_projection_local_error[None])
                if (global_error<=self.max_error and local_error<=self.local_max_error
                        and iteration+1>=self.minimum_density_iterations):break
            self.compute_volume_pressure_acceleration(boundary_enabled)
            self.evaluate_volume_pressure_residual(boundary_enabled,0)
            self.capture_volume_projection_statistics(0)
            self.record_volume_projection_status('density',iteration+1,self.max_error,self.local_max_error)
            if self.solver_verification_enabled:
                self.capture_volume_pressure_components(0)
            self.apply_volume_pressure_solution(boundary_enabled)
            self.compute_density_star()
            self.last_density_iterations=iteration+1
            self.last_local_density_projection_error=float(self.reduce_near_density_star())

        def correct_volume_divergence_error(self):
            self.compute_volume_raw_constraints()
            self.initialize_volume_divergence_pressure()
            boundary_enabled=int(self.divergence_boundary_projection_enabled[None])
            eta=self.max_error_V/self.dt[None]
            local_eta=self.local_max_error_V/self.dt[None]
            global_error=float('inf');local_error=float('inf')
            for iteration in range(self.m_max_iterations_v):
                self.compute_volume_pressure_acceleration(boundary_enabled)
                self.update_volume_divergence_pressure(boundary_enabled)
                global_error=float(self.volume_projection_global_error[None])
                local_error=float(self.volume_projection_local_error[None])
                if (global_error<=eta and local_error<=local_eta
                        and iteration+1>=self.minimum_divergence_iterations):break
            self.compute_volume_pressure_acceleration(boundary_enabled)
            self.evaluate_volume_pressure_residual(boundary_enabled,1)
            self.capture_volume_projection_statistics(1)
            self.record_volume_projection_status('divergence',iteration+1,eta,local_eta)
            if self.solver_verification_enabled:
                self.capture_volume_pressure_components(1)
            self.apply_volume_pressure_solution(boundary_enabled)
            self.compute_density_derivative()
            self.last_divergence_iterations=iteration+1
            self.last_local_divergence_projection_error=float(self.reduce_near_divergence())

        def record_volume_projection_status(self,name,iterations,tolerance,local_tolerance):
            global_error=float(self.volume_projection_global_error[None])
            local_error=float(self.volume_projection_local_error[None])
            all_boundary_error=float(self.volume_projection_all_boundary_error[None])
            index=0 if name=='density' else 1
            finite=bool(np.isfinite([global_error,local_error,all_boundary_error,
                float(self.volume_projection_max_pressure[index]),
                float(self.volume_projection_max_acceleration[index]),
                float(self.volume_projection_max_residual[index])]).all())
            self.projection_status[name]=dict(iterations=iterations,tolerance=tolerance,
                local_tolerance=local_tolerance,
                final_global_compression_residual=global_error,
                final_local_compression_residual=local_error,
                final_all_boundary_compression_residual=all_boundary_error,finite=finite,
                converged=bool(finite and global_error<=tolerance and local_error<=local_tolerance),
                reached_iteration_limit=iterations>=(self.m_max_iterations if name=='density'
                    else self.m_max_iterations_v))
            if name=='density':
                self.projection_status[name]['deficient_particles_with_positive_density_source']=int(
                    self.volume_projection_deficient_compressed[None])

        @ti.kernel
        def capture_volume_pressure_components(self,projection_index:ti.i32):
            # Final applied solution, rather than a sum of Jacobi iterates.
            for i in range(self.container.particle_num[None]):
                if self.container.particle_materials[i]==self.container.material_fluid:
                    stable_id=self.container.stable_ids[i]
                    fluid=self.dt[None]*self.volume_pressure_fluid_acceleration[i]
                    boundary=self.dt[None]*self.volume_pressure_boundary_acceleration[i]
                    if projection_index==0:
                        self.impact_density_fluid_dv[stable_id]=fluid
                        self.impact_density_boundary_dv[stable_id]=boundary
                    else:
                        self.impact_divergence_fluid_dv[stable_id]=fluid
                        self.impact_divergence_boundary_dv[stable_id]=boundary

        @ti.kernel
        def prepare_receiver_verification(self):
            for i in range(self.container.particle_num[None]):
                if self.container.particle_materials[i]==self.container.material_fluid:
                    stable_id=self.container.stable_ids[i]
                    self.verification_contact_before[stable_id]=0
                    self.verification_inbound_before[stable_id]=0.
                    if ti.static(self.verification_receiver_map>=0):
                        index=ti.static(self.verification_receiver_map)
                        if self.volume_boundary_volumes[i,index]>0.:
                            normal=self.volume_boundary_normals[i,index]
                            relative=self.container.particle_velocities[i]-self.volume_boundary_velocities[i,index]
                            self.verification_contact_before[stable_id]=1
                            self.verification_inbound_before[stable_id]=ti.math.dot(relative,normal)

        @ti.kernel
        def collect_accepted_verification(self,event_time:ti.f32):
            self.verification_max_components.fill(0.)
            self.verification_boundary_dominant[None]=0
            self.verification_fluid_dominant[None]=0
            self.verification_deficient_count[None]=0
            self.verification_receiver_contact_count[None]=0
            self.verification_component_closure_error.fill(0.)
            for i in range(self.container.particle_num[None]):
                if self.container.particle_materials[i]==self.container.material_fluid:
                    stable_id=self.container.stable_ids[i]
                    density_delta=(self.diagnostic_stage_velocities[stable_id,2]-
                        self.diagnostic_stage_velocities[stable_id,1])
                    divergence_delta=(self.diagnostic_stage_velocities[stable_id,4]-
                        self.diagnostic_stage_velocities[stable_id,3])
                    ti.atomic_max(self.verification_component_closure_error[0],
                        (density_delta-self.impact_density_fluid_dv[stable_id]-self.impact_density_boundary_dv[stable_id]).norm())
                    ti.atomic_max(self.verification_component_closure_error[1],
                        (divergence_delta-self.impact_divergence_fluid_dv[stable_id]-self.impact_divergence_boundary_dv[stable_id]).norm())
                    if self.volume_divergence_active[i]==0:
                        ti.atomic_add(self.verification_deficient_count[None],1)
                    if ti.static(self.verification_receiver_map>=0):
                        index=ti.static(self.verification_receiver_map)
                        contact=self.volume_boundary_volumes[i,index]>0.
                        if contact:
                            ti.atomic_add(self.verification_receiver_contact_count[None],1)
                            normal=self.volume_boundary_normals[i,index]
                            relative=self.container.particle_velocities[i]-self.volume_boundary_velocities[i,index]
                            vn=ti.math.dot(relative,normal)
                            inbound=ti.max(-vn,0.)
                            if self.verification_contact_before[stable_id]!=0:
                                inbound=ti.max(inbound,-self.verification_inbound_before[stable_id])
                            if self.verification_last_contact[stable_id]==0:
                                self.verification_inbound_history[stable_id]=0.
                            self.verification_inbound_history[stable_id]=ti.max(
                                self.verification_inbound_history[stable_id],inbound)
                            crossed=(self.verification_previous_normal_velocity[stable_id]<0. and vn>0.)
                            if self.verification_contact_before[stable_id]!=0 and self.verification_inbound_before[stable_id]<0. and vn>0.:
                                crossed=True
                            if crossed and self.verification_inbound_history[stable_id]>0.:
                                if self.verification_event_count[stable_id]==0:
                                    self.verification_event_inbound[stable_id]=self.verification_inbound_history[stable_id]
                                    self.verification_event_outbound[stable_id]=vn
                                    self.verification_event_time[stable_id]=event_time
                                self.verification_event_count[stable_id]+=1
                            self.verification_previous_normal_velocity[stable_id]=vn
                            components=ti.Vector([
                                ti.abs(ti.math.dot(self.impact_density_fluid_dv[stable_id],normal)),
                                ti.abs(ti.math.dot(self.impact_density_boundary_dv[stable_id],normal)),
                                ti.abs(ti.math.dot(self.impact_divergence_fluid_dv[stable_id],normal)),
                                ti.abs(ti.math.dot(self.impact_divergence_boundary_dv[stable_id],normal))])
                            for category in ti.static(range(4)):
                                ti.atomic_max(self.verification_max_components[category],components[category])
                            largest_fluid=ti.max(components[0],components[2])
                            largest_boundary=ti.max(components[1],components[3])
                            if largest_boundary>largest_fluid:
                                ti.atomic_add(self.verification_boundary_dominant[None],1)
                            elif largest_fluid>0.:
                                ti.atomic_add(self.verification_fluid_dominant[None],1)
                        else:
                            self.verification_previous_normal_velocity[stable_id]=0.
                            self.verification_inbound_history[stable_id]=0.
                        self.verification_last_contact[stable_id]=ti.cast(contact,ti.i32)

        def collect_wall_density_statistics(self):
            """First 4-mm fluid layer, using each actual solid SDF and normal."""
            n=int(self.container.particle_num[None])
            fluid=self.container.particle_materials.to_numpy()[:n]==self.container.material_fluid
            density=self.container.particle_densities.to_numpy()[:n][fluid].astype(np.float64)/self.density_0
            distances=self.volume_boundary_distances.to_numpy()[:n][fluid]
            normals=self.volume_boundary_normals.to_numpy()[:n][fluid]
            counts=self.volume_fluid_neighbor_count.to_numpy()[:n][fluid]
            result=dict(layer_width_m=self.spacing,fluid_neighbors_below_20=int(np.count_nonzero(counts<20)),regions={})
            for index,metadata in enumerate(self.volume_map_metadata):
                near=(distances[:,index]>0.) & (distances[:,index]<self.spacing)
                normal=normals[:,index]
                bottom=normal[:,1]>np.maximum(np.abs(normal[:,0]),np.abs(normal[:,2]))
                selections={'bottom':near & bottom,'side':near & ~bottom}
                for label,selected in selections.items():
                    values=density[selected]
                    result['regions'][metadata['name']+'_'+label]=dict(count=int(len(values)),
                        density_ratio_mean=float(values.mean()) if len(values) else None,
                        density_ratio_p95=float(np.percentile(values,95)) if len(values) else None)
                result['regions'][metadata['name']+'_penetrated']=dict(
                    count=int(np.count_nonzero((distances[:,index]<=0.) & (distances[:,index]>-self.spacing))))
            return result

        def collect_rebound_events(self):
            counts=self.verification_event_count.to_numpy()
            selected=np.flatnonzero(counts>0)
            inbound=self.verification_event_inbound.to_numpy()[selected].astype(np.float64)
            outbound=self.verification_event_outbound.to_numpy()[selected].astype(np.float64)
            times=self.verification_event_time.to_numpy()[selected]
            ratios=outbound/inbound if len(selected) else np.empty(0)
            return dict(definition='first inward-to-outward normal-velocity crossing per stable ID within receiver Volume Map support; relative to wall velocity',
                limitations='discrete substep samples; flow redirection, pressure work between particles, and small incident normal speeds can produce ratios above one without proving numerical energy injection',
                particle_count=int(len(selected)),event_count=int(counts.sum()),
                ratio_median=float(np.median(ratios)) if len(ratios) else None,
                ratio_p95=float(np.percentile(ratios,95)) if len(ratios) else None,
                ratio_max=float(ratios.max()) if len(ratios) else None,
                events=[dict(stable_id=int(sid),time_s=float(t),incoming_normal_speed_m_s=float(vin),
                    outgoing_normal_speed_m_s=float(vout),ratio=float(ratio))
                    for sid,t,vin,vout,ratio in zip(selected,times,inbound,outbound,ratios)])

        @ti.kernel
        def reset_impact_substep_diagnostics(self):
            for i in range(self.container.particle_num[None]):
                if self.container.particle_materials[i]==self.container.material_fluid:
                    stable_id=self.container.stable_ids[i]
                    self.impact_density_fluid_dv[stable_id]=ti.Vector.zero(ti.f32,self.container.dim)
                    self.impact_density_boundary_dv[stable_id]=ti.Vector.zero(ti.f32,self.container.dim)
                    self.impact_divergence_fluid_dv[stable_id]=ti.Vector.zero(ti.f32,self.container.dim)
                    self.impact_divergence_boundary_dv[stable_id]=ti.Vector.zero(ti.f32,self.container.dim)
                    self.impact_density_boundary_single_iteration_norm_sq[stable_id]=0.
                    self.impact_density_boundary_single_iteration_vector[stable_id]=ti.Vector.zero(ti.f32,self.container.dim)
                    self.impact_density_boundary_single_iteration_index[stable_id]=0
                    self.impact_divergence_boundary_single_iteration_norm_sq[stable_id]=0.
                    self.impact_divergence_boundary_single_iteration_vector[stable_id]=ti.Vector.zero(ti.f32,self.container.dim)
                    self.impact_divergence_boundary_single_iteration_index[stable_id]=0

        @ti.func
        def accumulate_density_projection_component_task(self,p_i,p_j,ret:ti.template()):
            if self.container.particle_materials[p_j]==self.container.material_fluid:
                k_j=self.container.particle_dfsph_kappa[p_j]
                k_sum=ret.k_i+k_j
                grad=self.container.particle_rest_volumes[p_j]*self.kernel_gradient(
                    self.container.particle_positions[p_i]-self.container.particle_positions[p_j])
                alpha_grad=-grad
                ret.fluid_gradient_square_sum+=alpha_grad.norm_sqr()
                ret.fluid_gradient_sum+=alpha_grad
                if ti.abs(k_sum)>self.m_eps*self.dt[None]:
                    ret.fluid-=grad*(ret.k_i/self.container.particle_densities[p_i]
                        +k_j/self.container.particle_densities[p_j])*self.density_0
                ret.fluid_neighbors+=1
            elif (ti.static(not self.volume_maps_enabled)
                    and self.container.particle_materials[p_j]==self.container.material_rigid):
                grad=self.container.particle_rest_volumes[p_j]*self.kernel_gradient(
                    self.container.particle_positions[p_i]-self.container.particle_positions[p_j])
                ret.boundary_gradient_sum-=grad
                ret.boundary_volume_sum+=self.container.particle_rest_volumes[p_j]
                if (self.density_boundary_projection_enabled[None]!=0
                        and ti.abs(ret.k_i)>self.m_eps*self.dt[None]):
                    ret.boundary-=grad*(ret.k_i/self.container.particle_densities[p_i])*self.density_0
                ret.rigid_neighbors+=1
                offset=self.container.particle_positions[p_i]-self.container.particle_positions[p_j]
                distance=offset.norm()
                if distance<ret.nearest_distance:
                    ret.nearest_distance=distance
                    ret.nearest_object=self.container.particle_object_ids[p_j]
                    ret.nearest_offset=offset
                    ret.nearest_velocity=self.container.particle_velocities[p_j]

        @ti.kernel
        def accumulate_density_projection_components(self):
            for i in range(self.container.particle_num[None]):
                if self.container.particle_materials[i]==self.container.material_fluid:
                    ret=ti.Struct(fluid=ti.Vector.zero(ti.f32,self.container.dim),
                        boundary=ti.Vector.zero(ti.f32,self.container.dim),
                        k_i=self.container.particle_dfsph_kappa[i],fluid_neighbors=0,rigid_neighbors=0,
                        nearest_distance=ti.cast(1.e10,ti.f32),nearest_object=-1,
                        nearest_offset=ti.Vector.zero(ti.f32,self.container.dim),
                        nearest_velocity=ti.Vector.zero(ti.f32,self.container.dim),
                        fluid_gradient_square_sum=0.,
                        fluid_gradient_sum=ti.Vector.zero(ti.f32,self.container.dim),
                        boundary_gradient_sum=ti.Vector.zero(ti.f32,self.container.dim),
                        boundary_volume_sum=0.)
                    self.container.for_all_neighbors(i,self.accumulate_density_projection_component_task,ret)
                    if ti.static(self.volume_maps_enabled):
                        position=self.container.particle_positions[i]
                        for map_index in ti.static(range(self.volume_map_count)):
                            volume=self.volume_boundary_volumes[i,map_index]
                            if volume>0.:
                                boundary_point=self.volume_boundary_points[i,map_index]
                                grad=volume*self.kernel_gradient(position-boundary_point)
                                ret.boundary_gradient_sum-=grad
                                ret.boundary_volume_sum+=volume
                                if (self.density_boundary_projection_enabled[None]!=0
                                        and ti.abs(ret.k_i)>self.m_eps*self.dt[None]):
                                    ret.boundary-=grad*(ret.k_i/self.container.particle_densities[i])*self.density_0
                                ret.rigid_neighbors+=1
                                offset=position-boundary_point
                                distance=offset.norm()
                                if distance<ret.nearest_distance:
                                    ret.nearest_distance=distance
                                    ret.nearest_object=self.volume_map_object_ids[map_index]
                                    ret.nearest_offset=offset
                                    ret.nearest_velocity=self.volume_boundary_velocities[i,map_index]
                    stable_id=self.container.stable_ids[i]
                    self.impact_density_fluid_dv[stable_id]+=ret.fluid
                    self.impact_density_boundary_dv[stable_id]+=ret.boundary
                    iteration_norm_sq=ret.boundary.norm_sqr()
                    if iteration_norm_sq>self.impact_density_boundary_single_iteration_norm_sq[stable_id]:
                        self.impact_density_boundary_single_iteration_norm_sq[stable_id]=iteration_norm_sq
                        self.impact_density_boundary_single_iteration_vector[stable_id]=ret.boundary
                        self.impact_density_boundary_single_iteration_index[stable_id]=self.current_projection_iteration[None]
                        self.impact_iteration_fluid_dv[stable_id,0]=ret.fluid
                        self.impact_iteration_velocity_before[stable_id,0]=self.impact_projection_iteration_velocity_before[stable_id]
                        self.impact_iteration_position[stable_id,0]=self.container.particle_positions[i]
                        self.impact_iteration_constraint[stable_id,0]=self.container.particle_densities_star[i]
                        self.impact_iteration_kappa[stable_id,0]=ret.k_i
                        self.impact_iteration_density[stable_id,0]=self.container.particle_densities[i]
                        self.impact_iteration_alpha[stable_id,0]=self.container.particle_dfsph_alphas[i]
                        self.impact_iteration_fluid_gradient_square_sum[stable_id,0]=ret.fluid_gradient_square_sum
                        self.impact_iteration_fluid_gradient_sum[stable_id,0]=ret.fluid_gradient_sum
                        self.impact_iteration_boundary_gradient_sum[stable_id,0]=ret.boundary_gradient_sum
                        self.impact_iteration_boundary_volume_sum[stable_id,0]=ret.boundary_volume_sum
                        self.impact_iteration_fluid_neighbors[stable_id,0]=ret.fluid_neighbors
                        self.impact_iteration_rigid_neighbors[stable_id,0]=ret.rigid_neighbors
                        self.impact_iteration_nearest_rigid_distance[stable_id,0]=ret.nearest_distance
                        self.impact_iteration_nearest_rigid_object[stable_id,0]=ret.nearest_object
                        self.impact_iteration_nearest_rigid_offset[stable_id,0]=ret.nearest_offset
                        self.impact_iteration_nearest_rigid_velocity[stable_id,0]=ret.nearest_velocity

        @ti.func
        def accumulate_divergence_projection_component_task(self,p_i,p_j,ret:ti.template()):
            if self.container.particle_materials[p_j]==self.container.material_fluid:
                k_j=self.container.particle_dfsph_kappa_v[p_j]
                k_sum=ret.k_i+k_j
                grad=self.container.particle_rest_volumes[p_j]*self.kernel_gradient(
                    self.container.particle_positions[p_i]-self.container.particle_positions[p_j])
                alpha_grad=-grad
                ret.fluid_gradient_square_sum+=alpha_grad.norm_sqr()
                ret.fluid_gradient_sum+=alpha_grad
                if ti.abs(k_sum)>self.m_eps*self.dt[None]:
                    ret.fluid-=grad*(ret.k_i/self.container.particle_densities[p_i]
                        +k_j/self.container.particle_densities[p_j])*self.density_0
                ret.fluid_neighbors+=1
            elif (ti.static(not self.volume_maps_enabled)
                    and self.container.particle_materials[p_j]==self.container.material_rigid):
                grad=self.container.particle_rest_volumes[p_j]*self.kernel_gradient(
                    self.container.particle_positions[p_i]-self.container.particle_positions[p_j])
                ret.boundary_gradient_sum-=grad
                ret.boundary_volume_sum+=self.container.particle_rest_volumes[p_j]
                if (self.divergence_boundary_projection_enabled[None]!=0
                        and ti.abs(ret.k_i)>self.m_eps*self.dt[None]):
                    ret.boundary-=grad*(ret.k_i/self.container.particle_densities[p_i])*self.density_0
                ret.rigid_neighbors+=1
                offset=self.container.particle_positions[p_i]-self.container.particle_positions[p_j]
                distance=offset.norm()
                if distance<ret.nearest_distance:
                    ret.nearest_distance=distance
                    ret.nearest_object=self.container.particle_object_ids[p_j]
                    ret.nearest_offset=offset
                    ret.nearest_velocity=self.container.particle_velocities[p_j]

        @ti.kernel
        def accumulate_divergence_projection_components(self):
            for i in range(self.container.particle_num[None]):
                if self.container.particle_materials[i]==self.container.material_fluid:
                    ret=ti.Struct(fluid=ti.Vector.zero(ti.f32,self.container.dim),
                        boundary=ti.Vector.zero(ti.f32,self.container.dim),
                        k_i=self.container.particle_dfsph_kappa_v[i],fluid_neighbors=0,rigid_neighbors=0,
                        nearest_distance=ti.cast(1.e10,ti.f32),nearest_object=-1,
                        nearest_offset=ti.Vector.zero(ti.f32,self.container.dim),
                        nearest_velocity=ti.Vector.zero(ti.f32,self.container.dim),
                        fluid_gradient_square_sum=0.,
                        fluid_gradient_sum=ti.Vector.zero(ti.f32,self.container.dim),
                        boundary_gradient_sum=ti.Vector.zero(ti.f32,self.container.dim),
                        boundary_volume_sum=0.)
                    self.container.for_all_neighbors(i,self.accumulate_divergence_projection_component_task,ret)
                    if ti.static(self.volume_maps_enabled):
                        position=self.container.particle_positions[i]
                        for map_index in ti.static(range(self.volume_map_count)):
                            volume=self.volume_boundary_volumes[i,map_index]
                            if volume>0.:
                                boundary_point=self.volume_boundary_points[i,map_index]
                                grad=volume*self.kernel_gradient(position-boundary_point)
                                ret.boundary_gradient_sum-=grad
                                ret.boundary_volume_sum+=volume
                                if (self.divergence_boundary_projection_enabled[None]!=0
                                        and ti.abs(ret.k_i)>self.m_eps*self.dt[None]):
                                    ret.boundary-=grad*(ret.k_i/self.container.particle_densities[i])*self.density_0
                                ret.rigid_neighbors+=1
                                offset=position-boundary_point
                                distance=offset.norm()
                                if distance<ret.nearest_distance:
                                    ret.nearest_distance=distance
                                    ret.nearest_object=self.volume_map_object_ids[map_index]
                                    ret.nearest_offset=offset
                                    ret.nearest_velocity=self.volume_boundary_velocities[i,map_index]
                    stable_id=self.container.stable_ids[i]
                    self.impact_divergence_fluid_dv[stable_id]+=ret.fluid
                    self.impact_divergence_boundary_dv[stable_id]+=ret.boundary
                    iteration_norm_sq=ret.boundary.norm_sqr()
                    if iteration_norm_sq>self.impact_divergence_boundary_single_iteration_norm_sq[stable_id]:
                        self.impact_divergence_boundary_single_iteration_norm_sq[stable_id]=iteration_norm_sq
                        self.impact_divergence_boundary_single_iteration_vector[stable_id]=ret.boundary
                        self.impact_divergence_boundary_single_iteration_index[stable_id]=self.current_projection_iteration[None]
                        self.impact_iteration_fluid_dv[stable_id,1]=ret.fluid
                        self.impact_iteration_velocity_before[stable_id,1]=self.impact_projection_iteration_velocity_before[stable_id]
                        self.impact_iteration_position[stable_id,1]=self.container.particle_positions[i]
                        self.impact_iteration_constraint[stable_id,1]=self.container.particle_densities_derivatives[i]
                        self.impact_iteration_kappa[stable_id,1]=ret.k_i
                        self.impact_iteration_density[stable_id,1]=self.container.particle_densities[i]
                        self.impact_iteration_alpha[stable_id,1]=self.container.particle_dfsph_alphas[i]
                        self.impact_iteration_fluid_gradient_square_sum[stable_id,1]=ret.fluid_gradient_square_sum
                        self.impact_iteration_fluid_gradient_sum[stable_id,1]=ret.fluid_gradient_sum
                        self.impact_iteration_boundary_gradient_sum[stable_id,1]=ret.boundary_gradient_sum
                        self.impact_iteration_boundary_volume_sum[stable_id,1]=ret.boundary_volume_sum
                        self.impact_iteration_fluid_neighbors[stable_id,1]=ret.fluid_neighbors
                        self.impact_iteration_rigid_neighbors[stable_id,1]=ret.rigid_neighbors
                        self.impact_iteration_nearest_rigid_distance[stable_id,1]=ret.nearest_distance
                        self.impact_iteration_nearest_rigid_object[stable_id,1]=ret.nearest_object
                        self.impact_iteration_nearest_rigid_offset[stable_id,1]=ret.nearest_offset
                        self.impact_iteration_nearest_rigid_velocity[stable_id,1]=ret.nearest_velocity
                    self.impact_fluid_neighbors[stable_id]=ret.fluid_neighbors
                    self.impact_rigid_neighbors[stable_id]=ret.rigid_neighbors
                    self.impact_nearest_rigid_distance[stable_id]=ret.nearest_distance
                    self.impact_nearest_rigid_object[stable_id]=ret.nearest_object
                    self.impact_nearest_rigid_offset[stable_id]=ret.nearest_offset
                    self.impact_nearest_rigid_velocity[stable_id]=ret.nearest_velocity

        @ti.func
        def update_impact_peak(self,stable_id,category,value,before,after,particle_index,event_time):
            norm_sq=value.norm_sqr()
            if norm_sq>self.impact_peak_norm_sq[stable_id,category]:
                self.impact_peak_norm_sq[stable_id,category]=norm_sq
                self.impact_peak_vector[stable_id,category]=value
                self.impact_peak_velocity_before[stable_id,category]=before
                self.impact_peak_velocity_after[stable_id,category]=after
                self.impact_peak_position[stable_id,category]=self.container.particle_positions[particle_index]
                self.impact_peak_time[stable_id,category]=event_time
                self.impact_peak_dt[stable_id,category]=self.dt[None]
                self.impact_peak_density[stable_id,category]=self.container.particle_densities[particle_index]
                self.impact_peak_alpha[stable_id,category]=self.container.particle_dfsph_alphas[particle_index]
                self.impact_peak_fluid_neighbors[stable_id,category]=self.impact_fluid_neighbors[stable_id]
                self.impact_peak_rigid_neighbors[stable_id,category]=self.impact_rigid_neighbors[stable_id]
                self.impact_peak_nearest_rigid_distance[stable_id,category]=self.impact_nearest_rigid_distance[stable_id]
                self.impact_peak_nearest_rigid_object[stable_id,category]=self.impact_nearest_rigid_object[stable_id]
                self.impact_peak_nearest_rigid_offset[stable_id,category]=self.impact_nearest_rigid_offset[stable_id]
                self.impact_peak_nearest_rigid_velocity[stable_id,category]=self.impact_nearest_rigid_velocity[stable_id]
                if category==3:
                    self.impact_peak_single_iteration_norm_sq[stable_id,category]=self.impact_density_boundary_single_iteration_norm_sq[stable_id]
                    self.impact_peak_single_iteration_vector[stable_id,category]=self.impact_density_boundary_single_iteration_vector[stable_id]
                    self.impact_peak_single_iteration_index[stable_id,category]=self.impact_density_boundary_single_iteration_index[stable_id]
                elif category==6:
                    self.impact_peak_single_iteration_norm_sq[stable_id,category]=self.impact_divergence_boundary_single_iteration_norm_sq[stable_id]
                    self.impact_peak_single_iteration_vector[stable_id,category]=self.impact_divergence_boundary_single_iteration_vector[stable_id]
                    self.impact_peak_single_iteration_index[stable_id,category]=self.impact_divergence_boundary_single_iteration_index[stable_id]
                if category==3 or category==6:
                    detail_index=ti.cast(0,ti.i32)
                    if category==6:
                        detail_index=1
                    self.impact_peak_iteration_fluid_dv[stable_id,detail_index]=self.impact_iteration_fluid_dv[stable_id,detail_index]
                    self.impact_peak_iteration_velocity_before[stable_id,detail_index]=self.impact_iteration_velocity_before[stable_id,detail_index]
                    self.impact_peak_iteration_position[stable_id,detail_index]=self.impact_iteration_position[stable_id,detail_index]
                    self.impact_peak_iteration_constraint[stable_id,detail_index]=self.impact_iteration_constraint[stable_id,detail_index]
                    self.impact_peak_iteration_kappa[stable_id,detail_index]=self.impact_iteration_kappa[stable_id,detail_index]
                    self.impact_peak_iteration_density[stable_id,detail_index]=self.impact_iteration_density[stable_id,detail_index]
                    self.impact_peak_iteration_alpha[stable_id,detail_index]=self.impact_iteration_alpha[stable_id,detail_index]
                    self.impact_peak_iteration_fluid_gradient_square_sum[stable_id,detail_index]=self.impact_iteration_fluid_gradient_square_sum[stable_id,detail_index]
                    self.impact_peak_iteration_fluid_gradient_sum[stable_id,detail_index]=self.impact_iteration_fluid_gradient_sum[stable_id,detail_index]
                    self.impact_peak_iteration_boundary_gradient_sum[stable_id,detail_index]=self.impact_iteration_boundary_gradient_sum[stable_id,detail_index]
                    self.impact_peak_iteration_boundary_volume_sum[stable_id,detail_index]=self.impact_iteration_boundary_volume_sum[stable_id,detail_index]
                    self.impact_peak_iteration_fluid_neighbors[stable_id,detail_index]=self.impact_iteration_fluid_neighbors[stable_id,detail_index]
                    self.impact_peak_iteration_rigid_neighbors[stable_id,detail_index]=self.impact_iteration_rigid_neighbors[stable_id,detail_index]
                    self.impact_peak_iteration_nearest_rigid_distance[stable_id,detail_index]=self.impact_iteration_nearest_rigid_distance[stable_id,detail_index]
                    self.impact_peak_iteration_nearest_rigid_object[stable_id,detail_index]=self.impact_iteration_nearest_rigid_object[stable_id,detail_index]
                    self.impact_peak_iteration_nearest_rigid_offset[stable_id,detail_index]=self.impact_iteration_nearest_rigid_offset[stable_id,detail_index]
                    self.impact_peak_iteration_nearest_rigid_velocity[stable_id,detail_index]=self.impact_iteration_nearest_rigid_velocity[stable_id,detail_index]

        @ti.kernel
        def update_impact_peaks(self,event_time:ti.f32):
            for i in range(self.container.particle_num[None]):
                if self.container.particle_materials[i]==self.container.material_fluid:
                    stable_id=self.container.stable_ids[i]
                    akinci_dv=self.akinci_accelerations[stable_id]*self.dt[None]
                    self.update_impact_peak(stable_id,0,akinci_dv,self.diagnostic_stage_velocities[stable_id,0],
                        self.diagnostic_stage_velocities[stable_id,0]+akinci_dv,i,event_time)
                    self.update_impact_peak(stable_id,1,self.diagnostic_stage_velocities[stable_id,1]-self.diagnostic_stage_velocities[stable_id,0],
                        self.diagnostic_stage_velocities[stable_id,0],self.diagnostic_stage_velocities[stable_id,1],i,event_time)
                    self.update_impact_peak(stable_id,2,self.impact_density_fluid_dv[stable_id],
                        self.diagnostic_stage_velocities[stable_id,1],self.diagnostic_stage_velocities[stable_id,2],i,event_time)
                    self.update_impact_peak(stable_id,3,self.impact_density_boundary_dv[stable_id],
                        self.diagnostic_stage_velocities[stable_id,1],self.diagnostic_stage_velocities[stable_id,2],i,event_time)
                    self.update_impact_peak(stable_id,4,self.diagnostic_stage_velocities[stable_id,3]-self.diagnostic_stage_velocities[stable_id,2],
                        self.diagnostic_stage_velocities[stable_id,2],self.diagnostic_stage_velocities[stable_id,3],i,event_time)
                    self.update_impact_peak(stable_id,5,self.impact_divergence_fluid_dv[stable_id],
                        self.diagnostic_stage_velocities[stable_id,3],self.diagnostic_stage_velocities[stable_id,4],i,event_time)
                    self.update_impact_peak(stable_id,6,self.impact_divergence_boundary_dv[stable_id],
                        self.diagnostic_stage_velocities[stable_id,3],self.diagnostic_stage_velocities[stable_id,4],i,event_time)
                    self.update_impact_peak(stable_id,7,self.diagnostic_stage_velocities[stable_id,4],self.diagnostic_stage_velocities[stable_id,0],
                        self.diagnostic_stage_velocities[stable_id,4],i,event_time)
                    self.update_impact_peak(stable_id,8,self.diagnostic_stage_velocities[stable_id,4]-self.diagnostic_stage_velocities[stable_id,0],
                        self.diagnostic_stage_velocities[stable_id,0],self.diagnostic_stage_velocities[stable_id,4],i,event_time)

        def collect_impact_diagnostics(self,top_count=8):
            names=('akinci_dv','nonpressure_total_dv','density_fluid_dv','density_boundary_dv',
                'domain_boundary_dv','divergence_fluid_dv','divergence_boundary_dv',
                'final_speed','total_substep_dv')
            norm_sq=self.impact_peak_norm_sq.to_numpy()
            vectors=self.impact_peak_vector.to_numpy();before=self.impact_peak_velocity_before.to_numpy()
            after=self.impact_peak_velocity_after.to_numpy();positions=self.impact_peak_position.to_numpy()
            times=self.impact_peak_time.to_numpy();dts=self.impact_peak_dt.to_numpy()
            densities=self.impact_peak_density.to_numpy();alphas=self.impact_peak_alpha.to_numpy()
            fluid_n=self.impact_peak_fluid_neighbors.to_numpy();rigid_n=self.impact_peak_rigid_neighbors.to_numpy()
            nearest_d=self.impact_peak_nearest_rigid_distance.to_numpy()
            nearest_o=self.impact_peak_nearest_rigid_object.to_numpy()
            nearest_offset=self.impact_peak_nearest_rigid_offset.to_numpy()
            nearest_velocity=self.impact_peak_nearest_rigid_velocity.to_numpy()
            single_iteration_norm_sq=self.impact_peak_single_iteration_norm_sq.to_numpy()
            single_iteration_vector=self.impact_peak_single_iteration_vector.to_numpy()
            single_iteration_index=self.impact_peak_single_iteration_index.to_numpy()
            iteration_fluid_dv=self.impact_peak_iteration_fluid_dv.to_numpy()
            iteration_velocity_before=self.impact_peak_iteration_velocity_before.to_numpy()
            iteration_position=self.impact_peak_iteration_position.to_numpy()
            iteration_constraint=self.impact_peak_iteration_constraint.to_numpy()
            iteration_kappa=self.impact_peak_iteration_kappa.to_numpy()
            iteration_density=self.impact_peak_iteration_density.to_numpy()
            iteration_alpha=self.impact_peak_iteration_alpha.to_numpy()
            iteration_fluid_gradient_square_sum=self.impact_peak_iteration_fluid_gradient_square_sum.to_numpy()
            iteration_fluid_gradient_sum=self.impact_peak_iteration_fluid_gradient_sum.to_numpy()
            iteration_boundary_gradient_sum=self.impact_peak_iteration_boundary_gradient_sum.to_numpy()
            iteration_boundary_volume_sum=self.impact_peak_iteration_boundary_volume_sum.to_numpy()
            iteration_fluid_neighbors=self.impact_peak_iteration_fluid_neighbors.to_numpy()
            iteration_rigid_neighbors=self.impact_peak_iteration_rigid_neighbors.to_numpy()
            iteration_nearest_rigid_distance=self.impact_peak_iteration_nearest_rigid_distance.to_numpy()
            iteration_nearest_rigid_object=self.impact_peak_iteration_nearest_rigid_object.to_numpy()
            iteration_nearest_rigid_offset=self.impact_peak_iteration_nearest_rigid_offset.to_numpy()
            iteration_nearest_rigid_velocity=self.impact_peak_iteration_nearest_rigid_velocity.to_numpy()
            result=dict(window_seconds=list(self.impact_diagnostic_window),
                filled_lattice_alpha_reference=self.filled_lattice_alpha_reference,
                categories={})
            for category,name in enumerate(names):
                order=np.argsort(norm_sq[:,category])[-top_count:][::-1]
                rows=[]
                for stable_id in order:
                    if norm_sq[stable_id,category]<=0.:continue
                    offset=nearest_offset[stable_id,category].astype(np.float64)
                    offset_norm=float(np.linalg.norm(offset))
                    normal=offset/offset_norm if offset_norm>0. else np.zeros(self.container.dim)
                    relative_normal=float(np.dot(before[stable_id,category].astype(np.float64)
                        -nearest_velocity[stable_id,category].astype(np.float64),normal))
                    row=dict(stable_id=int(stable_id),time_s=float(times[stable_id,category]),
                        dt_s=float(dts[stable_id,category]),norm=float(np.sqrt(norm_sq[stable_id,category])),
                        vector=vectors[stable_id,category].astype(np.float64).tolist(),
                        velocity_before_m_s=before[stable_id,category].astype(np.float64).tolist(),
                        velocity_after_m_s=after[stable_id,category].astype(np.float64).tolist(),
                        position_m=positions[stable_id,category].astype(np.float64).tolist(),
                        density_kg_m3=float(densities[stable_id,category]),alpha=float(alphas[stable_id,category]),
                        fluid_neighbors=int(fluid_n[stable_id,category]),rigid_neighbors=int(rigid_n[stable_id,category]),
                        nearest_rigid_distance_m=float(nearest_d[stable_id,category]),
                        nearest_rigid_object_id=int(nearest_o[stable_id,category]),
                        nearest_rigid_offset_m=offset.tolist(),relative_normal_velocity_before_m_s=relative_normal)
                    if category in (3,6):
                        detail_index=0 if category==3 else 1
                        fluid_gradient_sum=iteration_fluid_gradient_sum[stable_id,detail_index].astype(np.float64)
                        boundary_gradient_sum=iteration_boundary_gradient_sum[stable_id,detail_index].astype(np.float64)
                        denominator=float(iteration_fluid_gradient_square_sum[stable_id,detail_index]
                            +np.dot(fluid_gradient_sum+boundary_gradient_sum,
                                fluid_gradient_sum+boundary_gradient_sum))
                        detail_offset=iteration_nearest_rigid_offset[stable_id,detail_index].astype(np.float64)
                        detail_offset_norm=float(np.linalg.norm(detail_offset))
                        detail_normal=(detail_offset/detail_offset_norm
                            if detail_offset_norm>0. else np.zeros(self.container.dim))
                        detail_velocity=iteration_velocity_before[stable_id,detail_index].astype(np.float64)
                        detail_rigid_velocity=iteration_nearest_rigid_velocity[stable_id,detail_index].astype(np.float64)
                        row.update(maximum_single_iteration_norm=float(np.sqrt(
                                single_iteration_norm_sq[stable_id,category])),
                            maximum_single_iteration_vector=single_iteration_vector[stable_id,category].astype(np.float64).tolist(),
                            maximum_single_iteration_index=int(single_iteration_index[stable_id,category]),
                            single_iteration_fluid_dv=iteration_fluid_dv[stable_id,detail_index].astype(np.float64).tolist(),
                            single_iteration_velocity_before_m_s=detail_velocity.tolist(),
                            single_iteration_position_m=iteration_position[stable_id,detail_index].astype(np.float64).tolist(),
                            single_iteration_constraint_value=float(iteration_constraint[stable_id,detail_index]),
                            single_iteration_constraint=('density_star_normalized' if category==3
                                else 'positive_density_derivative_normalized_per_s'),
                            single_iteration_kappa=float(iteration_kappa[stable_id,detail_index]),
                            single_iteration_density_kg_m3=float(iteration_density[stable_id,detail_index]),
                            single_iteration_alpha=float(iteration_alpha[stable_id,detail_index]),
                            alpha_fluid_individual_gradient_square_sum=float(
                                iteration_fluid_gradient_square_sum[stable_id,detail_index]),
                            alpha_fluid_gradient_sum=fluid_gradient_sum.tolist(),
                            alpha_boundary_gradient_sum=boundary_gradient_sum.tolist(),
                            alpha_denominator_reconstructed=denominator,
                            alpha_denominator_ratio_to_filled_lattice=(denominator/
                                self.filled_lattice_alpha_reference['denominator_per_m2']),
                            alpha_reconstructed=(1./denominator if denominator>1.e-5 else 0.),
                            boundary_neighbor_rest_volume_sum_m3=float(
                                iteration_boundary_volume_sum[stable_id,detail_index]),
                            single_iteration_fluid_neighbors=int(
                                iteration_fluid_neighbors[stable_id,detail_index]),
                            single_iteration_rigid_neighbors=int(
                                iteration_rigid_neighbors[stable_id,detail_index]),
                            single_iteration_nearest_rigid_distance_m=float(
                                iteration_nearest_rigid_distance[stable_id,detail_index]),
                            single_iteration_nearest_rigid_object_id=int(
                                iteration_nearest_rigid_object[stable_id,detail_index]),
                            single_iteration_nearest_rigid_offset_m=detail_offset.tolist(),
                            single_iteration_relative_normal_velocity_before_m_s=float(np.dot(
                                detail_velocity-detail_rigid_velocity,detail_normal)))
                    rows.append(row)
                result['categories'][name]=rows
            return result

        @ti.kernel
        def snapshot_diagnostic_velocity_stage(self,stage:ti.i32):
            for i in range(self.container.particle_num[None]):
                if self.container.particle_materials[i]==self.container.material_fluid:
                    self.diagnostic_stage_velocities[self.container.stable_ids[i],stage]=self.container.particle_velocities[i]

        def snapshot_diagnostic_stage(self,stage):
            if self.diagnose_minimum_dt_failure or self.impact_diagnostics_enabled or self.solver_verification_enabled:
                self.snapshot_diagnostic_velocity_stage(stage)

        def capture_minimum_dt_failure(self,maximum_dt,candidate,speed):
            n=int(self.container.particle_num[None])
            materials=self.container.particle_materials.to_numpy()[:n]
            fluid_indices=np.flatnonzero(materials==self.container.material_fluid)
            velocities=self.container.particle_velocities.to_numpy()[:n]
            positions=self.container.particle_positions.to_numpy()[:n]
            fluid_speeds=np.linalg.norm(velocities[fluid_indices].astype(np.float64),axis=1)
            current_index=int(fluid_indices[int(np.argmax(fluid_speeds))])
            stable_id=int(self.container.stable_ids.to_numpy()[current_index])
            position=positions[current_index].astype(np.float64)
            displacement=positions.astype(np.float64)-position
            distances=np.linalg.norm(displacement,axis=1)
            neighbor_mask=(distances<float(self.container.dh))
            neighbor_mask[current_index]=False
            fluid_neighbor_mask=neighbor_mask & (materials==self.container.material_fluid)
            rigid_neighbor_mask=neighbor_mask & (materials==self.container.material_rigid)
            fluid_distances=distances[fluid_neighbor_mask]
            rigid_distances=distances[rigid_neighbor_mask]
            stages=self.diagnostic_stage_velocities.to_numpy()[stable_id].astype(np.float64)
            akinci_acceleration=self.akinci_accelerations.to_numpy()[stable_id].astype(np.float64)
            stage_names=('pre_nonpressure','post_nonpressure','post_density_projection',
                'post_domain_boundary_pre_divergence','post_divergence_projection')
            stage_velocities={name:stages[i].tolist() for i,name in enumerate(stage_names)}
            stage_speeds={name:float(np.linalg.norm(stages[i])) for i,name in enumerate(stage_names)}
            stage_deltas={f'{stage_names[i]}_minus_{stage_names[i-1]}':(stages[i]-stages[i-1]).tolist()
                for i in range(1,len(stage_names))}
            rigid_now=float(self.rigid_solver.max_surface_speed())
            rigid_future=float(self.rigid_solver.max_surface_speed(self.container.total_time+float(maximum_dt)))
            self.failure_diagnostics=dict(
                reason='adaptive_candidate_below_minimum_dt',simulated_seconds=float(self.container.total_time),
                requested_maximum_dt_s=float(maximum_dt),candidate_dt_s=float(candidate),
                minimum_dt_s=float(self.minimum_dt),reported_characteristic_speed_m_s=float(speed),
                cfl_speed_limit_at_minimum_dt_m_s=float(self.cfl_fraction*self.spacing/self.minimum_dt),
                fluid_max_speed_m_s=float(fluid_speeds.max()),rigid_surface_speed_m_s=rigid_now,
                future_rigid_surface_speed_m_s=rigid_future,current_particle_index=current_index,
                stable_id=stable_id,simulation_position_m=position.tolist(),
                current_velocity_m_s=velocities[current_index].astype(np.float64).tolist(),
                current_density_kg_m3=float(self.container.particle_densities.to_numpy()[current_index]),
                support_radius_m=float(self.container.dh),fluid_neighbor_count=int(fluid_neighbor_mask.sum()),
                rigid_neighbor_count=int(rigid_neighbor_mask.sum()),total_neighbor_count=int(neighbor_mask.sum()),
                nearest_fluid_neighbor_m=(float(fluid_distances.min()) if len(fluid_distances) else None),
                nearest_rigid_neighbor_m=(float(rigid_distances.min()) if len(rigid_distances) else None),
                previous_substep_dt_s=float(self.dt[None]),stage_velocities_m_s=stage_velocities,
                stage_speeds_m_s=stage_speeds,stage_velocity_deltas_m_s=stage_deltas,
                akinci_acceleration_m_s2=akinci_acceleration.tolist(),
                akinci_acceleration_norm_m_s2=float(np.linalg.norm(akinci_acceleration)),
                akinci_velocity_increment_at_previous_dt_m_s=(akinci_acceleration*float(self.dt[None])).tolist())

        def _step(self):
            self.projection_status={}
            # Sample prescribed motion at the midpoint used by the fluid solve.
            midpoint=self.container.total_time+.5*float(self.dt[None])
            self.rigid_solver.prepare_fluid_step(midpoint)
            self.renew_rigid_particle_state()
            self.container.prepare_neighborhood_search()
            self.refresh_volume_boundary_samples()
            self.mark_near_moving_boundary();self.compute_density();self.compute_alpha()
            if self.impact_diagnostics_enabled or self.solver_verification_enabled:self.reset_impact_substep_diagnostics()
            self.snapshot_diagnostic_stage(0)
            if self.solver_verification_enabled:self.prepare_receiver_verification()
            self.compute_non_pressure_acceleration();self.update_fluid_velocity();self.snapshot_diagnostic_stage(1)
            self.correct_density_error();self.snapshot_diagnostic_stage(2)
            self.update_fluid_position()
            self.rigid_solver.step()
            self.container.insert_object();self.rigid_solver.insert_rigid_object();self.renew_rigid_particle_state()
            if self.container.dim==3:self.enforce_domain_boundary_3D(self.container.material_fluid)
            else:self.enforce_domain_boundary_2D(self.container.material_fluid)
            self.container.prepare_neighborhood_search();self.refresh_volume_boundary_samples();self.mark_near_moving_boundary()
            self.compute_density();self.compute_alpha();self.snapshot_diagnostic_stage(3)
            self.correct_divergence_error();self.snapshot_diagnostic_stage(4)

        def max_characteristic_speed(self,future_time=None,allow_invalid=False):
            self.reduce_fluid_speed();ti.sync()
            if self.metric_invalid[None]:
                if allow_invalid:return float('inf')
                raise RuntimeError('Nonfinite fluid velocity before step')
            return max(float(np.sqrt(self.metric_max_speed_sq[None])),
                self.rigid_solver.max_surface_speed(),self.rigid_solver.max_surface_speed(future_time))

        def suggest_time_step(self,maximum):
            speed=self.max_characteristic_speed(self.container.total_time+maximum)
            cfl_dt=maximum if speed<=1e-12 else self.cfl_fraction*self.spacing/speed
            return min(float(maximum),cfl_dt),speed

        def _restore_step(self,fluid_time,rigid_snapshot,reaction_snapshot):
            self.container.restore_fluid_state();self.container.total_time=fluid_time
            self.rigid_solver.restore(rigid_snapshot);self.renew_rigid_particle_state()
            self.container.rigid_body_forces.from_numpy(reaction_snapshot[0])
            self.container.rigid_body_torques.from_numpy(reaction_snapshot[1])
            self.container.prepare_neighborhood_search();self.refresh_volume_boundary_samples()
            self.mark_near_moving_boundary();self.compute_density();self.compute_alpha()
            if self.solver_verification_enabled:
                self.check_restored_fluid_snapshot()
                rigid_matches=(np.array_equal(self.rigid_solver.state.body_q.numpy(),rigid_snapshot['q'])
                    and np.array_equal(self.rigid_solver.state.body_qd.numpy(),rigid_snapshot['qd'])
                    and np.array_equal(self.rigid_solver.state.body_f.numpy(),rigid_snapshot['f']))
                reactions_match=(np.array_equal(self.container.rigid_body_forces.to_numpy(),reaction_snapshot[0])
                    and np.array_equal(self.container.rigid_body_torques.to_numpy(),reaction_snapshot[1]))
                if self.verification_rollback_mismatch[None] or not rigid_matches or not reactions_match:
                    raise RuntimeError('Rejected substep snapshot was not restored exactly')
                self.rollback_verified_substeps+=1

        @ti.kernel
        def check_restored_fluid_snapshot(self):
            self.verification_rollback_mismatch[None]=0
            for i in range(self.container.particle_num[None]):
                if self.container.particle_materials[i]==self.container.material_fluid:
                    stable_id=self.container.stable_ids[i]
                    for axis in ti.static(range(self.container.dim)):
                        if (self.container.particle_positions[i][axis]!=self.container.snapshot_positions[stable_id][axis]
                                or self.container.particle_velocities[i][axis]!=self.container.snapshot_velocities[stable_id][axis]):
                            ti.atomic_max(self.verification_rollback_mismatch[None],1)

        def adaptive_step(self,maximum_dt):
            self.last_rejected_step_diagnostics=None
            self.last_trial_records=[]
            candidate,speed=self.suggest_time_step(min(float(maximum_dt),self.base_dt,self.adaptive_dt_cap))
            candidate,redistributed=redistribute_short_frame_tail(
                float(maximum_dt),candidate,self.minimum_dt)
            if redistributed:
                self.frame_tail_redistributions+=1
            retries=0;pressure_retries=0
            while True:
                if candidate<self.minimum_dt and maximum_dt>=self.minimum_dt:
                    if self.diagnose_minimum_dt_failure:
                        if self.last_rejected_step_diagnostics is not None:
                            self.failure_diagnostics=dict(self.last_rejected_step_diagnostics)
                            self.failure_diagnostics.update(
                                reason='adaptive_candidate_below_minimum_dt_after_rejection',
                                candidate_dt_s=float(candidate),minimum_dt_s=float(self.minimum_dt))
                        else:
                            self.capture_minimum_dt_failure(maximum_dt,candidate,speed)
                    raise RuntimeError(f'Adaptive step fell below minimum dt {self.minimum_dt:.9g}s')
                self.dt[None]=candidate;self.rigid_solver.set_time_step(candidate)
                fluid_time=float(self.container.total_time);rigid_snapshot=self.rigid_solver.snapshot()
                reaction_snapshot=(self.container.rigid_body_forces.to_numpy(),self.container.rigid_body_torques.to_numpy())
                self.container.save_fluid_state();ti.sync()
                super().step();self.reduce_step_displacement();ti.sync()
                displacement=float(np.sqrt(self.metric_max_displacement_sq[None]))
                invalid=bool(self.metric_invalid[None])
                post_speed=self.max_characteristic_speed(allow_invalid=True)
                invalid=invalid or not np.isfinite(post_speed)
                invalid=invalid or any(not value['finite'] for value in self.projection_status.values())
                predicted_travel=post_speed*candidate
                safe=(not invalid and displacement<=self.cfl_fraction*self.spacing*(1.+1e-4)
                        and predicted_travel<=self.cfl_fraction*self.spacing*(1.+1e-4))
                converged=all(value['converged'] for value in self.projection_status.values())
                pressure_decision=projection_retry_decision(converged,pressure_retries,candidate,
                    self.minimum_dt,self.pressure_convergence_retries)
                self.last_trial_records.append(dict(start_time_s=fluid_time,dt_s=candidate,
                    retry_index=retries,pressure_retry_index=pressure_retries,cfl_safe=bool(safe),
                    projection_status={key:dict(value) for key,value in self.projection_status.items()},
                    pressure_decision=pressure_decision,
                    accepted=bool(safe and pressure_decision!='halve')))
                if safe and pressure_decision!='halve':
                    if not converged:self.unconverged_accepted_substeps+=1
                    break
                if self.diagnose_minimum_dt_failure:
                    self.capture_minimum_dt_failure(maximum_dt,candidate,speed)
                    self.failure_diagnostics.update(
                        reason='adaptive_trial_rejected_before_rollback',trial_dt_s=float(candidate),
                        trial_max_displacement_m=float(displacement),trial_post_speed_m_s=float(post_speed),
                        trial_predicted_travel_m=float(predicted_travel),trial_invalid=bool(invalid),
                        trial_retry_index=int(retries+1))
                    self.last_rejected_step_diagnostics=dict(self.failure_diagnostics)
                    self.failure_diagnostics=None
                self._restore_step(fluid_time,rigid_snapshot,reaction_snapshot);self.rejected_substeps+=1;retries+=1
                if not converged and pressure_decision=='halve':
                    self.pressure_rejected_substeps+=1;pressure_retries+=1
                if retries>self.max_retries:
                    reason=('nonfinite state' if invalid else
                        f'displacement {displacement:.6g}m, predicted travel {predicted_travel:.6g}m')
                    raise RuntimeError(f'Adaptive step retry limit exceeded: {reason}')
                candidate*=.5
                candidate,redistributed=redistribute_short_frame_tail(float(maximum_dt),candidate,self.minimum_dt)
                if redistributed:self.frame_tail_redistributions+=1
            self.accepted_substeps+=1
            if (self.impact_diagnostics_enabled
                    and self.impact_diagnostic_window[0]<=self.container.total_time<=self.impact_diagnostic_window[1]):
                self.update_impact_peaks(float(self.container.total_time))
            if retries:self.adaptive_dt_cap=candidate
            else:self.adaptive_dt_cap=min(self.base_dt,self.adaptive_dt_cap*(1.5 if self.pressure_convergence_retries else 1.02))
            self.minimum_accepted_dt=min(self.minimum_accepted_dt,candidate)
            self.maximum_accepted_dt=max(self.maximum_accepted_dt,candidate)
            self.last_step_metrics=dict(dt=candidate,pre_step_characteristic_speed_m_s=speed,
                max_fluid_displacement_m=displacement,retries=retries,
                post_step_characteristic_speed_m_s=post_speed,predicted_next_travel_m=predicted_travel,
                next_step_cap_s=self.adaptive_dt_cap,
                density_iterations=self.last_density_iterations,divergence_iterations=self.last_divergence_iterations,
                local_density_projection_error=self.last_local_density_projection_error,
                local_divergence_projection_error_per_s=self.last_local_divergence_projection_error,
                near_boundary_max_density_error=None,near_boundary_max_divergence_per_s=None)
            if self.volume_maps_enabled:
                self.last_step_metrics.update(pressure_projection_status={key:dict(value)
                    for key,value in self.projection_status.items()},pressure_retries=pressure_retries,
                    pressure_converged=converged,accepted_unconverged=not converged)
                self.last_step_metrics['volume_projection_statistics']={
                    name:{
                        'maximum_pressure_over_density_squared':float(self.volume_projection_max_pressure[index]),
                        'maximum_acceleration_m_s2':float(self.volume_projection_max_acceleration[index]),
                        'maximum_fluid_acceleration_m_s2':float(self.volume_projection_max_fluid_acceleration[index]),
                        'maximum_boundary_acceleration_m_s2':float(self.volume_projection_max_boundary_acceleration[index]),
                        'maximum_velocity_increment_m_s':float(self.volume_projection_max_velocity_increment[index]),
                        'maximum_equation_residual':float(self.volume_projection_max_residual[index])}
                    for index,name in enumerate(('density','divergence'))}
            if (self.solver_verification_enabled
                    and self.solver_verification_window[0]<=self.container.total_time<=self.solver_verification_window[1]):
                self.collect_accepted_verification(float(self.container.total_time))
                self.last_step_metrics['collision_verification']=dict(
                    deficient_particle_count=int(self.verification_deficient_count[None]),
                    receiver_contact_particle_count=int(self.verification_receiver_contact_count[None]),
                    maximum_abs_normal_pressure_dv_m_s=dict(zip(
                        ('density_fluid','density_boundary','divergence_fluid','divergence_boundary'),
                        self.verification_max_components.to_numpy().astype(np.float64).tolist())),
                    boundary_dominant_count=int(self.verification_boundary_dominant[None]),
                    fluid_dominant_count=int(self.verification_fluid_dominant[None]))
                self.last_step_metrics['collision_verification']['component_closure_error_m_s']=dict(zip(
                    ('density','divergence'),self.verification_component_closure_error.to_numpy().astype(np.float64).tolist()))
            return dict(self.last_step_metrics)

        def collect_frame_diagnostics(self):
            self.reduce_near_boundary_residuals();ti.sync()
            self.last_step_metrics.update(
                near_boundary_max_density_error=float(self.metric_near_density[None]),
                near_boundary_max_divergence_per_s=float(self.metric_near_divergence[None]))
            if self.akinci_enabled:
                maximum=float(self.reduce_akinci_acceleration())
                self.last_step_metrics.update(max_surface_acceleration_m_s2=maximum,
                    surface_velocity_increment_m_s=maximum*self.last_step_metrics['dt'])
            return dict(self.last_step_metrics)

    original_factory=base_module.PyBulletSolver
    base_module.PyBulletSolver=NewtonRigidSolver
    try:solver=CoupledSolver(c)
    finally:base_module.PyBulletSolver=original_factory
    ratio=len(fluid_positions)/c.particle_max_num
    solver.max_error=solver.max_error_V=5e-5*ratio
    # The global error is a mean over particle_num, which includes boundary
    # samples, hence the ratio.  The local error is a maximum over fluid
    # particles only; by default it keeps the historical (ratio-scaled) value,
    # an explicit tolerance is used as given.
    if local_residual_tolerance is None:
        solver.local_max_error=solver.local_max_error_V=solver.max_error
    else:
        if not math.isfinite(local_residual_tolerance) or local_residual_tolerance<=0:
            raise ValueError('Local residual tolerance must be finite and positive')
        solver.local_max_error=solver.local_max_error_V=float(local_residual_tolerance)
    solver.m_max_iterations=solver.m_max_iterations_v=300
    solver.akinci_coefficient[None]=float(akinci_coefficient)
    solver.akinci_enabled=bool(akinci_coefficient)
    # The inherited attraction term is intentionally disabled; the overridden
    # method above is the only fluid surface-force path.
    solver.surface_tension=0.
    solver.prepare()
    c.restore_velocities(np.ascontiguousarray(fluid_velocities,np.float32))
    ti.sync()
    return c,solver

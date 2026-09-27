"""Replace SPH_Project's rigid integrator while keeping its DFSPH kernels.

Both engines use metres, Y-up and world-space COM wrenches.  Prescribed
boundaries are sampled at the fluid-step midpoint, then committed to the
exact end pose.  The adapter also exposes reversible Newton state so a
fluid step which violates the moving-boundary CFL can be retried safely.
"""
import importlib
import json
import sys
from pathlib import Path

import numpy as np
import taichi as ti
import trimesh
import newton
import warp as wp
from scipy.spatial.transform import Rotation


class NewtonRigidSolver:
    def __init__(self, container, gravity, dt):
        self.container=container; self.dt=float(dt); self.total_time=0.
        self.bodies=container.cfg.get_rigid_bodies()
        self.present_rigid_object=[]; self.bindings={}; self.drives={}
        self.body_samples={}
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
            last_prescribed_power=dict(self.last_prescribed_power),
            cumulative_prescribed_work=dict(self.cumulative_prescribed_work),
            cumulative_fluid_impulse={oid:value.copy() for oid,value in self.cumulative_fluid_impulse.items()})

    def restore(self,snapshot):
        self.state.body_q.assign(snapshot['q']);self.state.body_qd.assign(snapshot['qd'])
        self.state.body_f.assign(snapshot['f']);self.total_time=snapshot['total_time']
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
        for oid,(index,_) in self.bindings.items():
            rotation=Rotation.from_quat(q[index,3:]).as_matrix()
            c.rigid_body_original_centers_of_mass[oid]=self.com[index]
            c.rigid_body_centers_of_mass[oid]=q[index,:3]+rotation@self.com[index]
            c.rigid_body_rotations[oid]=rotation
            c.rigid_body_velocities[oid]=qd[index,:3]
            c.rigid_body_angular_velocities[oid]=qd[index,3:]

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
                   surface_tension=0.):
    """bodies: local mesh/samples, world pose, density, static/free/prescribed mode."""
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

    class CoupledSolver(DFSPHSolver):
        def __init__(self,container):
            super().__init__(container)
            self.spacing=float(spacing)
            self.base_dt=float(dt)
            self.adaptive_dt_cap=float(dt)
            self.cfl_fraction=.25
            self.max_retries=8
            self.minimum_dt=float(dt)/64
            self.metric_max_speed_sq=ti.field(ti.f32,shape=())
            self.metric_max_displacement_sq=ti.field(ti.f32,shape=())
            self.metric_invalid=ti.field(ti.i32,shape=())
            self.metric_near_density=ti.field(ti.f32,shape=())
            self.metric_near_divergence=ti.field(ti.f32,shape=())
            self.near_moving_boundary_mask=ti.field(ti.i32,shape=self.container.particle_max_num)
            self.last_density_iterations=0
            self.last_divergence_iterations=0
            self.accepted_substeps=0
            self.rejected_substeps=0
            self.frame_tail_redistributions=0
            self.minimum_accepted_dt=float('inf')
            self.maximum_accepted_dt=0.
            self.last_step_metrics={}

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
        def count_moving_boundary_neighbor(self,p_i,p_j,ret:ti.template()):
            if self.container.particle_materials[p_j]==self.container.material_rigid and self.container.particle_is_dynamic[p_j]:
                ret+=1

        @ti.kernel
        def mark_near_moving_boundary(self):
            self.near_moving_boundary_mask.fill(0)
            for i in range(self.container.particle_num[None]):
                if self.container.particle_materials[i]==self.container.material_fluid:
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

        def correct_density_error(self):
            self.compute_density_star()
            for iteration in range(self.m_max_iterations):
                self.compute_kappa();self.correct_density_error_step();self.compute_density_star()
                global_error=self.compute_density_error()
                local_error=self.reduce_near_density_star()
                if global_error<=self.max_error and local_error<=self.max_error and iteration>=1:break
            self.last_density_iterations=iteration+1
            self.last_local_density_projection_error=float(local_error)

        def correct_divergence_error(self):
            self.compute_density_derivative()
            eta=self.max_error_V*self.density_0/self.dt[None]
            local_eta=self.max_error_V/self.dt[None]
            for iteration in range(self.m_max_iterations_v):
                self.compute_kappa_v();self.correct_divergence_step();self.compute_density_derivative()
                global_error=self.compute_density_derivative_error()
                local_error=self.reduce_near_divergence()
                if global_error<=eta and local_error<=local_eta:break
            self.last_divergence_iterations=iteration+1
            self.last_local_divergence_projection_error=float(local_error)

        def _step(self):
            # Sample prescribed motion at the midpoint used by the fluid solve.
            midpoint=self.container.total_time+.5*float(self.dt[None])
            self.rigid_solver.prepare_fluid_step(midpoint)
            self.renew_rigid_particle_state()
            self.container.prepare_neighborhood_search()
            self.mark_near_moving_boundary();self.compute_density();self.compute_alpha()
            self.compute_non_pressure_acceleration();self.update_fluid_velocity();self.correct_density_error()
            self.update_fluid_position()
            self.rigid_solver.step()
            self.container.insert_object();self.rigid_solver.insert_rigid_object();self.renew_rigid_particle_state()
            if self.container.dim==3:self.enforce_domain_boundary_3D(self.container.material_fluid)
            else:self.enforce_domain_boundary_2D(self.container.material_fluid)
            self.container.prepare_neighborhood_search();self.mark_near_moving_boundary()
            self.compute_density();self.compute_alpha();self.correct_divergence_error()

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
            self.container.prepare_neighborhood_search();self.mark_near_moving_boundary();self.compute_density();self.compute_alpha()

        def adaptive_step(self,maximum_dt):
            candidate,speed=self.suggest_time_step(min(float(maximum_dt),self.base_dt,self.adaptive_dt_cap))
            tail=float(maximum_dt)-candidate
            if 0.<tail<self.minimum_dt and maximum_dt>=2*self.minimum_dt:
                candidate=float(maximum_dt)/2
                self.frame_tail_redistributions+=1
            retries=0
            while True:
                if candidate<self.minimum_dt and maximum_dt>=self.minimum_dt:
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
                predicted_travel=post_speed*candidate
                if (not invalid and displacement<=self.cfl_fraction*self.spacing*(1.+1e-4)
                        and predicted_travel<=self.cfl_fraction*self.spacing*(1.+1e-4)):break
                self._restore_step(fluid_time,rigid_snapshot,reaction_snapshot);self.rejected_substeps+=1;retries+=1
                if retries>self.max_retries:
                    reason=('nonfinite state' if invalid else
                        f'displacement {displacement:.6g}m, predicted travel {predicted_travel:.6g}m')
                    raise RuntimeError(f'Adaptive step retry limit exceeded: {reason}')
                candidate*=.5
            self.accepted_substeps+=1
            if retries:self.adaptive_dt_cap=candidate
            else:self.adaptive_dt_cap=min(self.base_dt,self.adaptive_dt_cap*1.02)
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
            return dict(self.last_step_metrics)

        def collect_frame_diagnostics(self):
            self.reduce_near_boundary_residuals();ti.sync()
            self.last_step_metrics.update(
                near_boundary_max_density_error=float(self.metric_near_density[None]),
                near_boundary_max_divergence_per_s=float(self.metric_near_divergence[None]))
            return dict(self.last_step_metrics)

    original_factory=base_module.PyBulletSolver
    base_module.PyBulletSolver=NewtonRigidSolver
    try:solver=CoupledSolver(c)
    finally:base_module.PyBulletSolver=original_factory
    ratio=len(fluid_positions)/c.particle_max_num
    solver.max_error=solver.max_error_V=5e-5*ratio
    solver.m_max_iterations=solver.m_max_iterations_v=300
    solver.surface_tension=float(surface_tension)
    solver.prepare()
    c.restore_velocities(np.ascontiguousarray(fluid_velocities,np.float32))
    ti.sync()
    return c,solver

"""Replace SPH_Project's rigid integrator while keeping its DFSPH kernels.

The upstream step consumes pressure/viscosity reactions before its final
divergence solve. That last reaction is carried into the next rigid step,
as in upstream. Both engines use metres, Y-up and world-space COM wrenches.
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
        self.last_wrenches={}; self.last_applied_wrenches={}
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
        self.model=builder.finalize()
        sdf_indices=self.model.shape_sdf_index.numpy()
        if np.any(sdf_indices<0):raise RuntimeError('Rigid mesh missing its precomputed SDF binding')
        for item,index in zip(self.collision_geometry,sdf_indices):item['sdf_index']=int(index)
        self.state=self.model.state();self.next_state=self.model.state()
        self.control=self.model.control();self.contacts=self.model.contacts()
        self.solver=newton.solvers.SolverXPBD(self.model,iterations=8,angular_damping=0.)
        self.com=self.model.body_com.numpy()
        velocities=self.state.body_qd.numpy()
        for body in self.bodies:
            if body['objectId'] in self.bindings:
                index,_=self.bindings[body['objectId']]
                velocities[index,:3]=body['velocity']
        if len(velocities):self.state.body_qd.assign(velocities)

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
        # Preserve external forces already supplied by the caller.
        body_forces=self.state.body_f.numpy()
        for oid,(index,prescribed) in self.bindings.items():
            wrench=np.r_[forces[oid],torques[oid]].astype(np.float32)
            if not np.isfinite(wrench).all():raise RuntimeError('Nonfinite liquid wrench')
            self.last_wrenches[oid]=wrench.copy()
            self.last_applied_wrenches[oid]=np.zeros(6,np.float32) if prescribed else wrench.copy()
            if not prescribed:body_forces[index]+=wrench
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


def create_backend(scene, fluid_positions, fluid_velocities, bodies, spacing, dt, upstream):
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

    # The numerical settings match the previously unified GPU comparison.
    spec=json.loads(Path(scene).read_text())
    c=ImportedContainer(SimConfig(str(scene)))
    c.rigid_sdf_options=dict(target_voxel_size=spacing/2)

    class CoupledSolver(DFSPHSolver):
        def correct_density_error(self):
            self.compute_density_star()
            for iteration in range(self.m_max_iterations):
                self.compute_kappa();self.correct_density_error_step();self.compute_density_star()
                if self.compute_density_error()<=self.max_error and iteration>=1:break

    original_factory=base_module.PyBulletSolver
    base_module.PyBulletSolver=NewtonRigidSolver
    try:solver=CoupledSolver(c)
    finally:base_module.PyBulletSolver=original_factory
    ratio=len(fluid_positions)/c.particle_max_num
    solver.max_error=solver.max_error_V=5e-5*ratio
    solver.m_max_iterations=solver.m_max_iterations_v=300
    solver.surface_tension=0.
    solver.prepare()
    c.restore_velocities(np.ascontiguousarray(fluid_velocities,np.float32))
    ti.sync()
    return c,solver

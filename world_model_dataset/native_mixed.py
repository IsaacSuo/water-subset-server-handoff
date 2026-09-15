"""Isaac 6.0 backend for one rigid projectile and one volume-deformable target."""
from __future__ import annotations

import argparse
import json
import sys
import time
import traceback
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))

from world_model_dataset.actions import due
from world_model_dataset.contract import artifact
from world_model_dataset.io import digest,file_hash,inside,read_json,write_json


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episode',type=Path,required=True);args=parser.parse_args();out=args.episode
    ep=read_json(out/'episode.prepared.json')
    if digest(ep['inputs'])!=ep['inputs_sha256']:raise ValueError('Input hash mismatch')
    for item in [ep['action'],*ep['artifacts']]:
        if artifact(out,item['path'])!=item:raise ValueError('Prepared artifact changed')
    for path,checksum in ep['source_files'].items():
        if file_hash(inside(ROOT,path))!=checksum:raise ValueError(f'Source changed after prepare: {path}')
    if (out/'state').exists():raise FileExistsError('Refusing to overwrite native state')

    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'renderer':'RayTracedLighting','width':640,'height':480})
    attached=False;simulation=None;contact_stream=None
    try:
        import carb
        import numpy as np
        import omni.physics.tensors as tensors
        import omni.usd
        from omni.physx import get_physx_interface,get_physx_simulation_interface
        from omni.physx.scripts import deformableUtils,physicsUtils
        from pxr import Gf,Sdf,Usd,UsdGeom,UsdPhysics,UsdShade,PhysxSchema,PhysicsSchemaTools,UsdUtils
        from soft_body.tet_quality import compute_tet_deformation,signed_tetrahedron_volumes

        spec=ep['spec'];inputs=ep['inputs'];objects=inputs['objects']
        if spec['event_id']!='V05':raise ValueError('Mixed backend currently implements V05 only')
        soft=[obj for obj in objects if obj['physics']['kind']=='volumetric']
        rigid=[obj for obj in objects if obj['physics']['kind']=='rigid']
        if len(soft)!=1 or len(rigid)!=1:raise ValueError('Expected one rigid and one volumetric object')
        fixture=read_json(out/'fixture.json');geometry_index=read_json(out/'geometry/index.json')
        numerics=inputs['numerics'];hz=spec['timing']['physics_hz'];dt=1/hz
        duration=spec['timing']['duration_s'];steps=round(duration*hz);stride=hz//spec['timing']['capture_hz']

        settings=carb.settings.get_settings();settings.set('/physics/updateToUsd',True)
        settings.set('/physics/updateVelocitiesToUsd',True);settings.set('/physics/updateParticlesToUsd',True)
        omni.usd.get_context().new_stage();stage=omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(stage,1.)
        UsdPhysics.SetStageKilogramsPerUnit(stage,1.);UsdGeom.Xform.Define(stage,'/World')
        stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
        scene=UsdPhysics.Scene.Define(stage,'/World/PhysicsScene')
        gravity=np.asarray(inputs['environment']['gravity_m_s2'],dtype=float);gravity_magnitude=float(np.linalg.norm(gravity))
        scene.CreateGravityDirectionAttr(Gf.Vec3f(*map(float,gravity/gravity_magnitude)))
        scene.CreateGravityMagnitudeAttr(gravity_magnitude)
        px=PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim());px.CreateEnableGPUDynamicsAttr(True)
        px.CreateBroadphaseTypeAttr('GPU');px.CreateSolverTypeAttr('TGS');px.CreateTimeStepsPerSecondAttr(hz)
        px.CreateEnableExternalForcesEveryIterationAttr(numerics['external_forces_every_iteration'])
        px.CreateGpuCollisionStackSizeAttr(64*1024*1024)
        px.CreateGpuMaxDeformableSurfaceContactsAttr(1048576);px.CreateGpuMaxDeformableVolumeContactsAttr(1048576)

        def contact_api(prim):
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)

        def collision(prim,material_path=None):
            UsdPhysics.CollisionAPI.Apply(prim);api=PhysxSchema.PhysxCollisionAPI.Apply(prim)
            api.CreateContactOffsetAttr(numerics['contact_offset_m']);api.CreateRestOffsetAttr(numerics['rest_offset_m'])
            if material_path is not None:physicsUtils.add_physics_material_to_prim(stage,prim,Sdf.Path(material_path))
            contact_api(prim)

        fixture_material='/World/FixtureMaterial';UsdShade.Material.Define(stage,fixture_material)
        fixture_profile=fixture.get('fixture_material',{})
        fm=UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(fixture_material))
        fm.CreateStaticFrictionAttr(fixture_profile.get('static_friction',.4))
        fm.CreateDynamicFrictionAttr(fixture_profile.get('dynamic_friction',.3))
        fm.CreateRestitutionAttr(fixture_profile.get('restitution',.1))
        fpx=PhysxSchema.PhysxMaterialAPI.Apply(stage.GetPrimAtPath(fixture_material))
        fpx.CreateFrictionCombineModeAttr(fixture_profile.get('friction_combine_mode','average'))
        fpx.CreateRestitutionCombineModeAttr(fixture_profile.get('restitution_combine_mode','average'))
        fixture_positions={}
        for item in fixture['boxes']:
            path='/World/'+item['id'];cube=UsdGeom.Cube.Define(stage,path);cube.CreateSizeAttr(1.)
            cube.AddTranslateOp().Set(Gf.Vec3d(*item['position_m']))
            q=item['orientation_xyzw'];cube.AddOrientOp().Set(Gf.Quatf(q[3],Gf.Vec3f(*q[:3])))
            cube.AddScaleOp().Set(Gf.Vec3f(*item['size_m']));collision(cube.GetPrim(),fixture_material)
            fixture_positions[item['id']]=item['position_m']

        actors={};soft_actor=None;rigid_actor=None
        for obj in objects:
            oid=obj['instance_id'];kind=obj['physics']['kind'];profile=obj['physics'];root='/World/'+oid
            with np.load(out/geometry_index[oid]['path'],allow_pickle=False) as data:
                vertices=data['vertices'].copy();triangles=data['triangles'].copy()
                mass=float(data['mass_kg']);inertia=data['inertia_kg_m2'].copy()
            xf=UsdGeom.Xform.Define(stage,root);xf.AddTranslateOp().Set(Gf.Vec3d(*fixture['subject_positions_m'][oid]))
            q=obj['orientation_xyzw'];xf.AddOrientOp().Set(Gf.Quatf(q[3],Gf.Vec3f(*q[:3])))
            visual=UsdGeom.Mesh.Define(stage,root+'/Visual');visual.CreatePointsAttr([Gf.Vec3f(*map(float,p)) for p in vertices])
            visual.CreateFaceVertexCountsAttr([3]*len(triangles));visual.CreateFaceVertexIndicesAttr(triangles.ravel().tolist())
            visual.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none);visual.CreateDisplayColorAttr([Gf.Vec3f(*obj['appearance']['color'])])
            material_path='/World/PhysicsMaterial_'+oid;UsdShade.Material.Define(stage,material_path)
            actor=dict(oid=oid,kind=kind,root=root,xf=xf,visual=visual,vertices=vertices,triangles=triangles,
                       mass=mass,inertia=inertia,material_path=material_path,bind_tet=None,tet_indices=None)
            if kind=='rigid':
                material=UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(material_path))
                material.CreateStaticFrictionAttr(profile['static_friction']);material.CreateDynamicFrictionAttr(profile['dynamic_friction'])
                material.CreateRestitutionAttr(profile['restitution']);material.CreateDensityAttr(profile['density_kg_m3'])
                material_px=PhysxSchema.PhysxMaterialAPI.Apply(stage.GetPrimAtPath(material_path))
                material_px.CreateFrictionCombineModeAttr('min');material_px.CreateRestitutionCombineModeAttr('average')
                rb=UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim());rb.CreateVelocityAttr(Gf.Vec3f(0));rb.CreateAngularVelocityAttr(Gf.Vec3f(0))
                bodypx=PhysxSchema.PhysxRigidBodyAPI.Apply(xf.GetPrim())
                bodypx.CreateSolverPositionIterationCountAttr(numerics['rigid_position_iterations'])
                bodypx.CreateSolverVelocityIterationCountAttr(numerics['rigid_velocity_iterations'])
                bodypx.CreateLinearDampingAttr(profile['linear_damping']);bodypx.CreateAngularDampingAttr(profile['angular_damping'])
                bodypx.CreateEnableSpeculativeCCDAttr(False)
                mass_api=UsdPhysics.MassAPI.Apply(xf.GetPrim());mass_api.CreateMassAttr(mass);mass_api.CreateCenterOfMassAttr(Gf.Vec3f(0))
                eigenvalues,eigenvectors=np.linalg.eigh(inertia)
                if np.linalg.det(eigenvectors)<0:eigenvectors[:,0]*=-1
                principal=Gf.Matrix3d(*map(float,eigenvectors.T.ravel())).ExtractRotation().GetQuat()
                mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float,eigenvalues)));mass_api.CreatePrincipalAxesAttr(Gf.Quatf(principal))
                collision_path=root+'/Collision'
                if obj['geometry']['rigid_collision']=='sphere':
                    collider=UsdGeom.Sphere.Define(stage,collision_path);collider.CreateRadiusAttr(obj['geometry']['characteristic_size_m']/2)
                    collider.CreateVisibilityAttr(UsdGeom.Tokens.invisible);collision(collider.GetPrim(),material_path)
                else:
                    collision(visual.GetPrim(),material_path)
                    UsdPhysics.MeshCollisionAPI.Apply(visual.GetPrim()).CreateApproximationAttr(obj['geometry']['rigid_collision'])
                contact_api(xf.GetPrim());actor.update(rb=rb,collision_path=collision_path);rigid_actor=actor
            else:
                if not deformableUtils.add_deformable_material(stage,material_path,density=profile['density_kg_m3'],
                    static_friction=.4,dynamic_friction=profile['dynamic_friction'],youngs_modulus=profile['youngs_modulus_pa'],
                    poissons_ratio=profile['poissons_ratio']):raise RuntimeError('Deformable material creation failed')
                material_prim=stage.GetPrimAtPath(material_path);material_prim.ApplyAPI('PhysxDeformableMaterialAPI')
                damping=material_prim.GetAttribute('physxDeformableMaterial:elasticityDamping')
                if not damping or not damping.Set(profile['elasticity_damping']):raise RuntimeError('Elasticity damping unavailable')
                simulation_path=root+'/Simulation';collision_path=root+'/Collision'
                if not deformableUtils.create_auto_volume_deformable_hierarchy(stage,Sdf.Path(root),Sdf.Path(simulation_path),
                    Sdf.Path(collision_path),visual.GetPath(),True,True,True):raise RuntimeError('Auto volume hierarchy failed')
                prim=xf.GetPrim();prim.GetAttribute('physxDeformableBody:resolution').Set(obj['geometry']['deformable_resolution'])
                prim.GetAttribute('omniphysics:mass').Set(mass);prim.GetAttribute('physxDeformableBody:remeshingEnabled').Set(True)
                prim.GetAttribute('physxDeformableBody:forceConforming').Set(True);prim.GetAttribute('physxDeformableBody:targetTriangleCount').Set(0)
                prim.ApplyAPI('PhysxBaseDeformableBodyAPI')
                for name,value in {'linearDamping':profile['linear_damping'],'settlingDamping':0.,
                    'solverPositionIterationCount':numerics['deformable_position_iterations'],'selfCollision':False,'enableSpeculativeCCD':True}.items():
                    attr=prim.GetAttribute('physxDeformableBody:'+name)
                    if not attr or not attr.Set(value):raise RuntimeError('Unavailable deformable attribute: '+name)
                physicsUtils.add_physics_material_to_prim(stage,prim,Sdf.Path(material_path))
                collision(stage.GetPrimAtPath(collision_path));contact_api(prim)
                actor.update(simulation_path=simulation_path,collision_path=collision_path);soft_actor=actor
            actors[oid]=actor

        actions=read_json(out/'action.json')['commands'];action_applications=[0]*len(actions)
        state_dir=out/'state';state_dir.mkdir();step_index=0
        counts={'headers':0,'points':0,'rigid_points':0,'actor_pairs':{}};contact_stream=(out/'contacts.jsonl').open('x',encoding='utf-8')
        def belongs(path,actor):return path==actor['root'] or path.startswith(actor['root']+'/')
        def on_contact(headers,data):
            for header in headers:
                paths=[str(PhysicsSchemaTools.intToSdfPath(getattr(header,key))) for key in ('actor0','actor1','collider0','collider1')]
                points=[]
                for index in range(header.contact_data_offset,header.contact_data_offset+header.num_contact_data):
                    value=data[index];points.append(dict(position_m=list(value.position),normal=list(value.normal),
                        impulse_ns=list(value.impulse),separation_m=float(value.separation)))
                row=dict(schema_version='0.1.0',time_s=step_index*dt,physics_step=step_index,dt_s=dt,
                    event_type=str(header.type),actor0=paths[0],actor1=paths[1],collider0=paths[2],collider1=paths[3],
                    source='physx_native_contact_report',points=points)
                contact_stream.write(json.dumps(row,allow_nan=False)+'\n');counts['headers']+=1;counts['points']+=len(points)
                if any(belongs(path,rigid_actor) for path in paths):counts['rigid_points']+=len(points)

        simulation=get_physx_simulation_interface();subscription=simulation.subscribe_contact_report_events(on_contact)
        stage_id=UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        for _ in range(3):app.update()
        simulation.attach_stage(stage_id);attached=True;get_physx_interface().force_load_physics_from_usd()
        tensor_view=tensors.create_simulation_view('warp',stage_id)
        soft_actor['tensor_body']=tensor_view.create_volume_deformable_body_view(soft_actor['root'])
        material_view=tensor_view.create_deformable_material_view(soft_actor['material_path']+'*')
        material_readback=dict(status='native',count=material_view.count,
            youngs_modulus_pa=float(material_view.get_youngs_modulus().numpy()[0,0]),
            poissons_ratio=float(material_view.get_poissons_ratio().numpy()[0,0]),
            dynamic_friction=float(material_view.get_dynamic_friction().numpy()[0,0]))
        def native_nodal_velocities(mesh,count):
            # Explicit simulate/fetch exports PhysX velocities to USD. The
            # stopped-timeline tensor view may be CPU and lacks this getter.
            # Reuse the supported V01 route; never infer velocities from positions.
            for name in ('omniphysics:velocities','velocities'):
                attr=mesh.GetPrim().GetAttribute(name);values=attr.Get() if attr else None
                if values is not None and len(values)==count:
                    return np.asarray(values,dtype=np.float64)
            return soft_actor['tensor_body'].get_simulation_nodal_velocities().numpy()[0,:count].astype(np.float64)
        frames=[];substep_min_j=float('inf');substep_inverted=0;substep_checked=0;start=time.monotonic()
        impact_substeps={'checked_steps':0,'maximum_target_nonrigid_rms_m':0.,
            'maximum_target_local_displacement_m':0.,
            'maximum_target_axis_compression_fraction':0.,'maximum_rigid_speed_m_s':0.,
            'maximum_soft_nodal_speed_m_s':0.,'minimum_sampled_surface_gap_m':float('inf'),
            'maximum_sampled_penetration_m':0.,'geometric_contact_first_time_s':None,
            'geometric_contact_last_time_s':None,
            'penetration_semantics':'rigid analytic sphere versus deformable collision-node sampling; diagnostic lower bound, not solver penetration'}

        def capture(step):
            cache=UsdGeom.XformCache(Usd.TimeCode.Default());objects_state={};geometry_paths={}
            for oid,actor in actors.items():
                matrix=cache.GetLocalToWorldTransform(actor['xf'].GetPrim());transform=Gf.Transform(matrix)
                translation=transform.GetTranslation();quat=transform.GetRotation().GetQuat()
                orientation=[*map(float,quat.GetImaginary()),float(quat.GetReal())]
                local=np.asarray(actor['visual'].GetPointsAttr().Get(),dtype=np.float64)
                visual_matrix=cache.GetLocalToWorldTransform(actor['visual'].GetPrim())
                world=np.asarray([visual_matrix.Transform(Gf.Vec3d(*point)) for point in local])
                tet_points=np.empty((0,3));tets=np.empty((0,4),dtype=np.int32);nodal=np.empty((0,3))
                collision_world=np.empty((0,3));collision_tets=np.empty((0,4),dtype=np.int32);metrics={}
                if actor['kind']=='rigid':
                    linear=np.asarray(actor['rb'].GetVelocityAttr().Get(),dtype=float)
                    angular=np.radians(np.asarray(actor['rb'].GetAngularVelocityAttr().Get(),dtype=float));centre=np.asarray(translation)
                    body_rotation=np.asarray(matrix)[:3,:3].T;local_w=body_rotation.T@angular
                    energy=.5*actor['mass']*float(linear@linear)+.5*float(local_w@actor['inertia']@local_w)
                    inertia_diag=np.linalg.eigvalsh(actor['inertia']).tolist()
                else:
                    mesh=UsdGeom.TetMesh.Get(stage,actor['simulation_path']);tet_points=np.asarray(mesh.GetPointsAttr().Get(),dtype=np.float64)
                    tets=np.asarray(mesh.GetTetVertexIndicesAttr().Get(),dtype=np.int32).reshape(-1,4)
                    if actor['bind_tet'] is None:
                        bind=mesh.GetPrim().GetAttribute('deformablePose:default:omniphysics:points').Get()
                        if bind is None:raise RuntimeError('No native bind pose')
                        actor['bind_tet']=np.asarray(bind,dtype=np.float64);actor['tet_indices']=tets.copy()
                    if not np.array_equal(actor['tet_indices'],tets):raise RuntimeError('Simulation topology changed')
                    metrics=compute_tet_deformation(tet_points,tets,actor['bind_tet'])
                    volumes=np.abs(signed_tetrahedron_volumes(actor['bind_tet'],tets));weights=np.zeros(len(actor['bind_tet']))
                    np.add.at(weights,tets.ravel(),np.repeat(volumes/4,4));weights/=weights.sum()
                    nodal=np.zeros_like(tet_points) if step==0 else native_nodal_velocities(mesh,len(tet_points))
                    linear=weights@nodal;angular=None;energy=.5*actor['mass']*float(weights@(nodal*nodal).sum(axis=1));inertia_diag=None
                    sim_matrix=cache.GetLocalToWorldTransform(mesh.GetPrim());sim_world=np.asarray([sim_matrix.Transform(Gf.Vec3d(*p)) for p in tet_points])
                    centre=weights@sim_world;collision_mesh=UsdGeom.TetMesh.Get(stage,actor['collision_path'])
                    collision_points=np.asarray(collision_mesh.GetPointsAttr().Get(),dtype=np.float64)
                    collision_matrix=cache.GetLocalToWorldTransform(collision_mesh.GetPrim())
                    collision_world=np.asarray([collision_matrix.Transform(Gf.Vec3d(*p)) for p in collision_points])
                    collision_tets=np.asarray(collision_mesh.GetTetVertexIndicesAttr().Get(),dtype=np.int32).reshape(-1,4)
                    metrics['volume_ratio']=float(np.abs(signed_tetrahedron_volumes(tet_points,tets)).sum()/volumes.sum())
                metrics['height_m']=float(np.ptp(world[:,2]));metrics['minimum_z_m']=float(world[:,2].min())
                filename=f'state/frame_{len(frames):04d}_{oid}.npz'
                with (out/filename).open('xb') as stream:
                    np.savez(stream,surface_world_m=world.astype(np.float32),surface_triangles=actor['triangles'],
                        simulation_points_m=tet_points.astype(np.float32),simulation_tets=tets,
                        simulation_nodal_velocities_m_s=nodal.astype(np.float32),collision_world_m=collision_world.astype(np.float32),
                        collision_tets=collision_tets,simulation_local_to_world=np.array(cache.GetLocalToWorldTransform(
                            stage.GetPrimAtPath(actor.get('simulation_path','/World')))) if actor['kind']=='volumetric' else np.eye(4),
                        simulation_bind_points_m=np.empty((0,3)) if actor['bind_tet'] is None else actor['bind_tet'],time_s=step*dt)
                geometry_paths[oid]=filename
                objects_state[oid]=dict(physics_kind=actor['kind'],position_m=centre.tolist(),orientation_xyzw=orientation,
                    linear_velocity_m_s=linear.tolist(),angular_velocity_rad_s=None if angular is None else angular.tolist(),
                    mass_kg=actor['mass'],inertia_kg_m2=inertia_diag,kinetic_energy_j=energy,
                    potential_energy_j=-actor['mass']*float(gravity@centre),geometry=artifact(out,filename),metrics=metrics)
            state_path=f'state/frame_{len(frames):04d}.json';write_json(out/state_path,
                dict(schema_version='0.1.0',time_s=step*dt,physics_step=step,objects=objects_state))
            frames.append(dict(time_s=step*dt,physics_step=step,state=state_path,geometries=geometry_paths,
                fixture_positions=fixture_positions,native_kinematic_poses={}))

        capture(0)
        bind_centered=soft_actor['bind_tet']-soft_actor['bind_tet'].mean(axis=0)
        bind_axis_extent=float(np.ptp(soft_actor['bind_tet'][:,0]))
        projectile_radius=rigid[0]['geometry']['characteristic_size_m']/2
        for step in range(steps):
            for command_index,command in enumerate(actions):
                if command['kind']=='initial_velocity' and due(command,step,hz):
                    body=UsdPhysics.RigidBodyAPI(actors[command['target']]['xf'].GetPrim());parameters=command['parameters']
                    body.GetVelocityAttr().Set(Gf.Vec3f(*parameters['linear_m_s']))
                    body.GetAngularVelocityAttr().Set(Gf.Vec3f(*np.degrees(parameters['angular_rad_s'])))
                    action_applications[command_index]+=1
                elif command['kind']!='initial_velocity':raise ValueError('Unsupported mixed command '+command['kind'])
            step_index=step+1;simulation.simulate(dt,step*dt);simulation.fetch_results()
            mesh=UsdGeom.TetMesh.Get(stage,soft_actor['simulation_path']);current=np.asarray(mesh.GetPointsAttr().Get(),dtype=np.float64)
            ratios=signed_tetrahedron_volumes(current,soft_actor['tet_indices'])/signed_tetrahedron_volumes(soft_actor['bind_tet'],soft_actor['tet_indices'])
            substep_min_j=min(substep_min_j,float(ratios.min()));substep_inverted=max(substep_inverted,int(np.count_nonzero(ratios<0)));substep_checked+=1
            current_centered=current-current.mean(axis=0)
            u,_,vt=np.linalg.svd(bind_centered.T@current_centered);rotation=u@vt
            if np.linalg.det(rotation)<0:u[:,-1]*=-1;rotation=u@vt
            local_displacements=current_centered-bind_centered@rotation
            shape_rms=float(np.sqrt(np.mean(np.sum(local_displacements**2,axis=1))))
            local_max=float(np.linalg.norm(local_displacements,axis=1).max())
            axis_compression=max(0.,1-float(np.ptp(current[:,0]))/bind_axis_extent)
            rigid_speed=float(np.linalg.norm(np.asarray(rigid_actor['rb'].GetVelocityAttr().Get(),dtype=float)))
            soft_velocities=native_nodal_velocities(mesh,len(current))
            soft_speed=float(np.linalg.norm(soft_velocities,axis=1).max())
            cache=UsdGeom.XformCache(Usd.TimeCode.Default())
            rigid_centre=np.asarray(Gf.Transform(cache.GetLocalToWorldTransform(rigid_actor['xf'].GetPrim())).GetTranslation())
            collision_mesh=UsdGeom.TetMesh.Get(stage,soft_actor['collision_path'])
            collision_points=np.asarray(collision_mesh.GetPointsAttr().Get(),dtype=np.float64)
            collision_matrix=cache.GetLocalToWorldTransform(collision_mesh.GetPrim())
            collision_world=np.asarray([collision_matrix.Transform(Gf.Vec3d(*point)) for point in collision_points])
            sampled_gap=float(np.linalg.norm(collision_world-rigid_centre,axis=1).min()-projectile_radius)
            if not all(np.isfinite(value) for value in (shape_rms,axis_compression,rigid_speed,soft_speed,sampled_gap)):
                raise RuntimeError('Non-finite V05 substep diagnostic')
            impact_substeps['checked_steps']+=1
            impact_substeps['maximum_target_nonrigid_rms_m']=max(impact_substeps['maximum_target_nonrigid_rms_m'],shape_rms)
            impact_substeps['maximum_target_local_displacement_m']=max(impact_substeps['maximum_target_local_displacement_m'],local_max)
            impact_substeps['maximum_target_axis_compression_fraction']=max(impact_substeps['maximum_target_axis_compression_fraction'],axis_compression)
            impact_substeps['maximum_rigid_speed_m_s']=max(impact_substeps['maximum_rigid_speed_m_s'],rigid_speed)
            impact_substeps['maximum_soft_nodal_speed_m_s']=max(impact_substeps['maximum_soft_nodal_speed_m_s'],soft_speed)
            impact_substeps['minimum_sampled_surface_gap_m']=min(impact_substeps['minimum_sampled_surface_gap_m'],sampled_gap)
            impact_substeps['maximum_sampled_penetration_m']=max(impact_substeps['maximum_sampled_penetration_m'],-sampled_gap)
            if sampled_gap<=numerics['contact_offset_m']:
                contact_time=step_index*dt
                if impact_substeps['geometric_contact_first_time_s'] is None:impact_substeps['geometric_contact_first_time_s']=contact_time
                impact_substeps['geometric_contact_last_time_s']=contact_time
            if step_index%stride==0:
                app.update();capture(step_index)
                print(f'DATASET {spec["episode_id"]} {step_index}/{steps} t={step_index*dt:.3f}',flush=True)

        contact_stream.close();contact_stream=None;write_json(out/'state/index.json',dict(schema_version='0.1.0',complete=True,frames=frames))
        stage.GetRootLayer().Export(str(out/'native_final.usda'))
        evidence=out/'capability_probes/soft_contact_impulse.json';evidence.parent.mkdir()
        write_json(evidence,dict(schema_version='0.1.0',status='unavailable',backend_version='Isaac Sim 6.0.1 / PhysX extension 110.1.13',
            probe='PhysxContactReportAPI on rigid body, deformable root/collision mesh and floor; threshold=0; callback active for every explicit step',
            callback_headers=counts['headers'],callback_points=counts['points'],
            reason='Public callback does not identify a reliable deformable-side contact impulse; no zero or estimated impulse substituted',
            rigid_control_evidence='The same stream records native rigid/floor points when present'))
        write_json(out/'native_report.json',dict(schema_version='0.1.0',status='physics_completed',elapsed_seconds=time.monotonic()-start,
            captured_frames=len(frames),simulated_seconds=duration,contact_counts=counts,
            physical_representation='rigid_volumetric_mixed',runtime='Isaac Sim 6.0.1 / PhysX 110.1.13',dt_s=dt,numerics=numerics,
            object_ids=list(actors),deformable_material_tensor_readback=material_readback,
            soft_velocity_source='PhysX native USD velocity attribute; tensor getter fallback; no position differencing',
            action_applications=[dict(command_index=i,kind=command['kind'],target=command['target'],applications=action_applications[i]) for i,command in enumerate(actions)],
            substep_tet_audit=dict(checked_steps=substep_checked,minimum_j=substep_min_j,inverted_tets=substep_inverted),
            substep_impact_audit=impact_substeps))
        print('DATASET_PHYSICS_COMPLETE '+str(out),flush=True)
    except Exception as exc:
        traceback.print_exc()
        if not (out/'native_failure.json').exists():
            write_json(out/'native_failure.json',dict(error=f'{type(exc).__name__}: {exc}',traceback=traceback.format_exc()))
        raise
    finally:
        if contact_stream is not None:contact_stream.close()
        if attached:simulation.detach_stage()
        app.close()


if __name__=='__main__':main()

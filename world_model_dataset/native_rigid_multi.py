"""Reusable Isaac 6.0 backend for episodes containing multiple rigid bodies."""
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
    parser.add_argument('--episode',type=Path,required=True)
    args=parser.parse_args();out=args.episode
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
        import omni.usd
        from omni.physx import get_physx_interface,get_physx_simulation_interface
        from omni.physx.scripts import physicsUtils
        from pxr import Gf,Sdf,Usd,UsdGeom,UsdPhysics,UsdShade,PhysxSchema,PhysicsSchemaTools,UsdUtils

        spec=ep['spec'];inputs=ep['inputs'];objects=inputs['objects']
        if len(objects)<2 or any(o['physics']['kind']!='rigid' for o in objects):
            raise ValueError('Multi-rigid backend requires at least two rigid objects')
        fixture=read_json(out/'fixture.json');geom_index=read_json(out/'geometry/index.json')
        numerics=inputs['numerics'];hz=spec['timing']['physics_hz'];dt=1/hz
        duration=spec['timing']['duration_s'];steps=round(duration*hz)
        stride=hz//spec['timing']['capture_hz']

        settings=carb.settings.get_settings()
        settings.set('/physics/updateToUsd',True);settings.set('/physics/updateVelocitiesToUsd',True)
        omni.usd.get_context().new_stage();stage=omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(stage,1.)
        UsdPhysics.SetStageKilogramsPerUnit(stage,1.)
        UsdGeom.Xform.Define(stage,'/World');stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
        scene=UsdPhysics.Scene.Define(stage,'/World/PhysicsScene')
        gravity=np.asarray(inputs['environment']['gravity_m_s2'],dtype=float);gm=float(np.linalg.norm(gravity))
        scene.CreateGravityDirectionAttr(Gf.Vec3f(*map(float,gravity/gm)));scene.CreateGravityMagnitudeAttr(gm)
        px=PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        px.CreateEnableGPUDynamicsAttr(True);px.CreateBroadphaseTypeAttr('GPU');px.CreateSolverTypeAttr('TGS')
        px.CreateTimeStepsPerSecondAttr(hz);px.CreateEnableExternalForcesEveryIterationAttr(numerics['external_forces_every_iteration'])
        px.CreateGpuCollisionStackSizeAttr(64*1024*1024)

        fixture_material='/World/FixtureMaterial';UsdShade.Material.Define(stage,fixture_material)
        fm=UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(fixture_material))
        # The canonical collision lane supports the bodies without injecting a
        # horizontal friction impulse. Object-object material response remains
        # active, so restitution/friction counterfactuals still concern the pair.
        fm.CreateStaticFrictionAttr(0.);fm.CreateDynamicFrictionAttr(0.);fm.CreateRestitutionAttr(0.)
        fixture_px=PhysxSchema.PhysxMaterialAPI.Apply(stage.GetPrimAtPath(fixture_material))
        fixture_px.CreateFrictionCombineModeAttr('min');fixture_px.CreateRestitutionCombineModeAttr('average')

        def contact_api(prim):
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)

        def collision(prim,material_path):
            UsdPhysics.CollisionAPI.Apply(prim)
            col=PhysxSchema.PhysxCollisionAPI.Apply(prim)
            col.CreateContactOffsetAttr(numerics['contact_offset_m']);col.CreateRestOffsetAttr(numerics['rest_offset_m'])
            physicsUtils.add_physics_material_to_prim(stage,prim,Sdf.Path(material_path));contact_api(prim)

        fixture_positions={}
        for item in fixture['boxes']:
            path='/World/'+item['id'];cube=UsdGeom.Cube.Define(stage,path);cube.CreateSizeAttr(1.)
            cube.AddTranslateOp().Set(Gf.Vec3d(*item['position_m']))
            q=item['orientation_xyzw'];cube.AddOrientOp().Set(Gf.Quatf(q[3],Gf.Vec3f(*q[:3])))
            cube.AddScaleOp().Set(Gf.Vec3f(*item['size_m']));cube.CreateDisplayColorAttr([Gf.Vec3f(.42,.45,.48)])
            collision(cube.GetPrim(),fixture_material);fixture_positions[item['id']]=item['position_m']
            if item['kinematic']:UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim()).CreateKinematicEnabledAttr(True)

        actors={};prims={}
        for obj in objects:
            oid=obj['instance_id'];root='/World/'+oid;profile=obj['physics']
            with np.load(out/geom_index[oid]['path'],allow_pickle=False) as data:
                vertices=data['vertices'].copy();triangles=data['triangles'].copy()
                mass=float(data['mass_kg']);inertia=data['inertia_kg_m2'].copy()
            material_path='/World/PhysicsMaterial_'+oid;UsdShade.Material.Define(stage,material_path)
            material=UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(material_path))
            material.CreateStaticFrictionAttr(profile['static_friction']);material.CreateDynamicFrictionAttr(profile['dynamic_friction'])
            material.CreateRestitutionAttr(profile['restitution']);material.CreateDensityAttr(profile['density_kg_m3'])
            material_px=PhysxSchema.PhysxMaterialAPI.Apply(stage.GetPrimAtPath(material_path))
            material_px.CreateFrictionCombineModeAttr('min');material_px.CreateRestitutionCombineModeAttr('average')

            xf=UsdGeom.Xform.Define(stage,root);xf.AddTranslateOp().Set(Gf.Vec3d(*fixture['subject_positions_m'][oid]))
            q=obj['orientation_xyzw'];xf.AddOrientOp().Set(Gf.Quatf(q[3],Gf.Vec3f(*q[:3])))
            visual=UsdGeom.Mesh.Define(stage,root+'/Visual')
            visual.CreatePointsAttr([Gf.Vec3f(*map(float,v)) for v in vertices])
            visual.CreateFaceVertexCountsAttr([3]*len(triangles));visual.CreateFaceVertexIndicesAttr(triangles.ravel().tolist())
            visual.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none);visual.CreateDisplayColorAttr([Gf.Vec3f(*obj['appearance']['color'])])
            rb=UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim());rb.CreateVelocityAttr(Gf.Vec3f(0));rb.CreateAngularVelocityAttr(Gf.Vec3f(0))
            bodypx=PhysxSchema.PhysxRigidBodyAPI.Apply(xf.GetPrim())
            bodypx.CreateSolverPositionIterationCountAttr(numerics['rigid_position_iterations'])
            bodypx.CreateSolverVelocityIterationCountAttr(numerics['rigid_velocity_iterations'])
            bodypx.CreateLinearDampingAttr(profile['linear_damping']);bodypx.CreateAngularDampingAttr(profile['angular_damping'])
            # R03's bounded speed/time-step domain stays well below one body
            # radius per step. Speculative CCD creates an anticipatory positive-
            # separation constraint here and suppresses the configured bounce.
            bodypx.CreateEnableSpeculativeCCDAttr(False)
            mass_api=UsdPhysics.MassAPI.Apply(xf.GetPrim());mass_api.CreateMassAttr(mass);mass_api.CreateCenterOfMassAttr(Gf.Vec3f(0))
            eigenvalues,eigenvectors=np.linalg.eigh(inertia)
            if np.linalg.det(eigenvectors)<0:eigenvectors[:,0]*=-1
            rotation=Gf.Matrix3d(*map(float,eigenvectors.T.ravel())).ExtractRotation().GetQuat()
            mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float,eigenvalues)));mass_api.CreatePrincipalAxesAttr(Gf.Quatf(rotation))
            coll_path=root+'/Collision'
            if obj['geometry']['rigid_collision']=='sphere':
                collider=UsdGeom.Sphere.Define(stage,coll_path);collider.CreateRadiusAttr(obj['geometry']['characteristic_size_m']/2)
                collider.CreateVisibilityAttr(UsdGeom.Tokens.invisible);collision(collider.GetPrim(),material_path)
            else:
                collision(visual.GetPrim(),material_path)
                UsdPhysics.MeshCollisionAPI.Apply(visual.GetPrim()).CreateApproximationAttr(obj['geometry']['rigid_collision'])
                if obj['geometry']['rigid_collision']=='sdf':
                    PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(visual.GetPrim()).CreateSdfResolutionAttr(128)
            contact_api(xf.GetPrim());prims[oid]=xf.GetPrim()
            actors[oid]=dict(root=root,xf=xf,visual=visual,rb=rb,mass=mass,inertia=inertia,
                             vertices=vertices,triangles=triangles)

        actions=read_json(out/'action.json')['commands'];action_applications=[0]*len(actions)
        state_dir=out/'state';state_dir.mkdir();step_index=0
        counts={'headers':0,'points':0,'subject_points':0,'interbody_points':0,
                'per_object_points':{oid:0 for oid in actors}}
        contacts_seen=set();contact_stream=(out/'contacts.jsonl').open('x',encoding='utf-8')

        def belongs(path,oid):
            root=actors[oid]['root'];return path==root or path.startswith(root+'/')

        def on_contact(headers,data):
            for header in headers:
                paths=[str(PhysicsSchemaTools.intToSdfPath(getattr(header,key))) for key in ('actor0','actor1','collider0','collider1')]
                points=[]
                for index in range(header.contact_data_offset,header.contact_data_offset+header.num_contact_data):
                    value=data[index]
                    points.append(dict(position_m=list(value.position),normal=list(value.normal),impulse_ns=list(value.impulse),separation_m=float(value.separation)))
                row=dict(schema_version='0.1.0',time_s=step_index*dt,physics_step=step_index,dt_s=dt,
                         event_type=str(header.type),actor0=paths[0],actor1=paths[1],collider0=paths[2],collider1=paths[3],
                         source='physx_native_contact_report',points=points)
                contact_stream.write(json.dumps(row,allow_nan=False)+'\n')
                counts['headers']+=1;counts['points']+=len(points)
                touched=[oid for oid in actors if any(belongs(path,oid) for path in paths)]
                if touched:counts['subject_points']+=len(points)
                for oid in touched:counts['per_object_points'][oid]+=len(points)
                if len(touched)>=2:counts['interbody_points']+=len(points)
                contacts_seen.add(tuple(paths[:2]))

        simulation=get_physx_simulation_interface();subscription=simulation.subscribe_contact_report_events(on_contact)
        stage_id=UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        for _ in range(3):app.update()
        simulation.attach_stage(stage_id);attached=True
        frames=[];start=time.monotonic()

        def capture(step):
            cache=UsdGeom.XformCache(Usd.TimeCode.Default());objects_state={};geometry_paths={}
            for oid,actor in actors.items():
                matrix=cache.GetLocalToWorldTransform(actor['xf'].GetPrim());transform=Gf.Transform(matrix)
                tr=transform.GetTranslation();quat=transform.GetRotation().GetQuat()
                orientation=[*map(float,quat.GetImaginary()),float(quat.GetReal())]
                local=np.asarray(actor['visual'].GetPointsAttr().Get(),dtype=np.float64)
                visual_matrix=cache.GetLocalToWorldTransform(actor['visual'].GetPrim())
                world=np.asarray([visual_matrix.Transform(Gf.Vec3d(*point)) for point in local])
                linear=np.asarray(actor['rb'].GetVelocityAttr().Get(),dtype=float)
                angular=np.radians(np.asarray(actor['rb'].GetAngularVelocityAttr().Get(),dtype=float))
                rotation_world=np.asarray(matrix)[:3,:3].T;local_w=rotation_world.T@angular
                energy=.5*actor['mass']*float(linear@linear)+.5*float(local_w@actor['inertia']@local_w)
                filename=f'state/frame_{len(frames):04d}_{oid}.npz'
                with (out/filename).open('xb') as stream:
                    np.savez(stream,surface_world_m=world.astype(np.float32),surface_triangles=actor['triangles'],
                             simulation_points_m=np.empty((0,3),dtype=np.float32),simulation_tets=np.empty((0,4),dtype=np.int32),
                             simulation_nodal_velocities_m_s=np.empty((0,3),dtype=np.float32),
                             collision_world_m=np.empty((0,3),dtype=np.float32),collision_tets=np.empty((0,4),dtype=np.int32),
                             simulation_local_to_world=np.eye(4),simulation_bind_points_m=np.empty((0,3)),time_s=step*dt)
                geometry_paths[oid]=filename
                objects_state[oid]=dict(physics_kind='rigid',position_m=list(map(float,tr)),orientation_xyzw=orientation,
                    linear_velocity_m_s=linear.tolist(),angular_velocity_rad_s=angular.tolist(),mass_kg=actor['mass'],
                    inertia_kg_m2=np.linalg.eigvalsh(actor['inertia']).tolist(),kinetic_energy_j=energy,
                    potential_energy_j=-actor['mass']*float(gravity@np.asarray(tr)),geometry=artifact(out,filename),
                    metrics={'height_m':float(np.ptp(world[:,2])),'minimum_z_m':float(world[:,2].min())})
            row=dict(schema_version='0.1.0',time_s=step*dt,physics_step=step,objects=objects_state)
            state_path=f'state/frame_{len(frames):04d}.json';write_json(out/state_path,row)
            frames.append(dict(time_s=step*dt,physics_step=step,state=state_path,geometries=geometry_paths,
                               fixture_positions=fixture_positions,native_kinematic_poses={}))

        get_physx_interface().force_load_physics_from_usd();capture(0)
        for step in range(steps):
            for command_index,command in enumerate(actions):
                if command['kind']=='initial_velocity' and due(command,step,hz):
                    body=UsdPhysics.RigidBodyAPI(prims[command['target']]);parameters=command['parameters']
                    body.GetVelocityAttr().Set(Gf.Vec3f(*parameters['linear_m_s']))
                    body.GetAngularVelocityAttr().Set(Gf.Vec3f(*np.degrees(parameters['angular_rad_s'])))
                    action_applications[command_index]+=1
                elif command['kind']!='initial_velocity':raise ValueError('Unsupported multi-rigid command '+command['kind'])
            step_index=step+1;simulation.simulate(dt,step*dt);simulation.fetch_results()
            if step_index%stride==0:
                app.update();capture(step_index)
                print(f'DATASET {spec["episode_id"]} {step_index}/{steps} t={step_index*dt:.3f} interbody={counts["interbody_points"]}',flush=True)

        contact_stream.close();contact_stream=None
        write_json(out/'state/index.json',dict(schema_version='0.1.0',complete=True,frames=frames))
        stage.GetRootLayer().Export(str(out/'native_final.usda'))
        report=dict(schema_version='0.1.0',status='physics_completed',elapsed_seconds=time.monotonic()-start,
            captured_frames=len(frames),simulated_seconds=duration,contact_counts=counts,
            contact_actor_pairs=[list(pair) for pair in contacts_seen],physical_representation='rigid_multi',
            runtime='Isaac Sim 6.0.1 / PhysX 110.1.13',dt_s=dt,numerics=numerics,object_ids=list(actors),
            action_applications=[dict(command_index=i,kind=command['kind'],target=command['target'],applications=action_applications[i]) for i,command in enumerate(actions)])
        write_json(out/'native_report.json',report);print('DATASET_PHYSICS_COMPLETE '+str(out),flush=True)
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

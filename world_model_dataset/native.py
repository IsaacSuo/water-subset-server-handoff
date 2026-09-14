"""Isaac 6.0 native backend. Run with Isaac's python, never the CPU environment.

Consumes a prepared episode without importing a legacy script or advancing time
through rendering. Contact reporting is probed per physical representation.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
import traceback
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))

from world_model_dataset.io import read_json,write_json,file_hash,inside,digest
from world_model_dataset.actions import due,sample_trajectory
from world_model_dataset.contract import artifact


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episode',type=Path,required=True)
    args=parser.parse_args();out=args.episode
    ep=read_json(out/'episode.prepared.json')
    if digest(ep['inputs'])!=ep['inputs_sha256']:raise ValueError('Input hash mismatch')
    for a in [ep['action'],*ep['artifacts']]:
        if artifact(out,a['path'])!=a:raise ValueError('Prepared artifact changed')
    for p,h in ep['source_files'].items():
        if file_hash(inside(ROOT,p))!=h:raise ValueError(f'Source changed after prepare: {p}')
    if (out/'state').exists():raise FileExistsError('Refusing to overwrite native state')
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'renderer':'RayTracedLighting','width':640,'height':480})
    attached=False;simulation=None;contact_stream=None
    try:
        import carb
        import numpy as np
        import omni.usd
        from omni.physx import get_physx_simulation_interface,get_physx_interface
        from omni.physx.scripts import physicsUtils,deformableUtils
        from pxr import Gf,Sdf,Usd,UsdGeom,UsdPhysics,UsdShade,UsdLux,PhysxSchema,PhysicsSchemaTools,UsdUtils
        from soft_body.tet_quality import compute_tet_deformation,signed_tetrahedron_volumes

        settings=carb.settings.get_settings()
        settings.set('/physics/updateToUsd',True)
        settings.set('/physics/updateVelocitiesToUsd',True)
        settings.set('/physics/updateParticlesToUsd',True)
        omni.usd.get_context().new_stage()
        stage=omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(stage,1.0)
        UsdPhysics.SetStageKilogramsPerUnit(stage,1.0)
        UsdGeom.Xform.Define(stage,'/World');stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
        spec=ep['spec'];inputs=ep['inputs'];subject=inputs['objects'][0];profile=subject['physics']
        kind=profile['kind'];numerics=inputs['numerics'];fixture=read_json(out/'fixture.json');geom_index=read_json(out/'geometry/index.json')
        hz=spec['timing']['physics_hz'];dt=1/hz;duration=spec['timing']['duration_s']
        stride=hz//spec['timing']['capture_hz'];steps=round(duration*hz)
        scene=UsdPhysics.Scene.Define(stage,'/World/PhysicsScene')
        gravity=np.array(inputs['environment']['gravity_m_s2']);gm=float(np.linalg.norm(gravity))
        scene.CreateGravityDirectionAttr(Gf.Vec3f(*map(float,gravity/gm)));scene.CreateGravityMagnitudeAttr(gm)
        px=PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        px.CreateEnableGPUDynamicsAttr(True);px.CreateBroadphaseTypeAttr('GPU');px.CreateSolverTypeAttr('TGS')
        px.CreateTimeStepsPerSecondAttr(hz);px.CreateEnableExternalForcesEveryIterationAttr(numerics['external_forces_every_iteration'])
        # Explicit modest capacities; no server-sized resource assumptions.
        px.CreateGpuCollisionStackSizeAttr(64*1024*1024)
        px.CreateGpuMaxDeformableSurfaceContactsAttr(1048576)
        px.CreateGpuMaxDeformableVolumeContactsAttr(1048576)
        physical='/World/PhysicsMaterial'
        UsdShade.Material.Define(stage,physical)
        if kind=='volumetric':
            if not deformableUtils.add_deformable_material(stage,physical,density=profile['density_kg_m3'],
                static_friction=.4,dynamic_friction=profile['dynamic_friction'],
                youngs_modulus=profile['youngs_modulus_pa'],poissons_ratio=profile['poissons_ratio']):
                raise RuntimeError('Deformable material creation failed')
        else:
            mat=UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(physical))
            mat.CreateStaticFrictionAttr(profile.get('static_friction',.4));mat.CreateDynamicFrictionAttr(profile['dynamic_friction'])
            mat.CreateRestitutionAttr(profile.get('restitution',0.));mat.CreateDensityAttr(profile['density_kg_m3'])
            pmat=PhysxSchema.PhysxMaterialAPI.Apply(stage.GetPrimAtPath(physical))
            pmat.CreateFrictionCombineModeAttr('average');pmat.CreateRestitutionCombineModeAttr('average')
        fixture_material='/World/FixtureMaterial'
        UsdShade.Material.Define(stage,fixture_material)
        fm=UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(fixture_material))
        fixture_profile=fixture.get('fixture_material',{})
        fm.CreateStaticFrictionAttr(fixture_profile.get('static_friction',.4))
        fm.CreateDynamicFrictionAttr(fixture_profile.get('dynamic_friction',.3))
        fm.CreateRestitutionAttr(fixture_profile.get('restitution',.1))
        fixture_pmat=PhysxSchema.PhysxMaterialAPI.Apply(stage.GetPrimAtPath(fixture_material))
        fixture_pmat.CreateFrictionCombineModeAttr(fixture_profile.get('friction_combine_mode','average'))
        fixture_pmat.CreateRestitutionCombineModeAttr(fixture_profile.get('restitution_combine_mode','average'))
        translations={};prims={}

        def contact_api(prim):
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)

        def collision(prim,material_path=physical):
            UsdPhysics.CollisionAPI.Apply(prim)
            col=PhysxSchema.PhysxCollisionAPI.Apply(prim)
            col.CreateContactOffsetAttr(numerics['contact_offset_m']);col.CreateRestOffsetAttr(numerics['rest_offset_m'])
            if material_path is not None:physicsUtils.add_physics_material_to_prim(stage,prim,Sdf.Path(material_path))
            contact_api(prim)

        for b in fixture['boxes']:
            path='/World/'+b['id'];cube=UsdGeom.Cube.Define(stage,path);cube.CreateSizeAttr(1.)
            translations[b['id']]=cube.AddTranslateOp();translations[b['id']].Set(Gf.Vec3d(*b['position_m']))
            q=b['orientation_xyzw'];cube.AddOrientOp().Set(Gf.Quatf(q[3],Gf.Vec3f(*q[:3])))
            cube.AddScaleOp().Set(Gf.Vec3f(*b['size_m']));cube.CreateDisplayColorAttr([Gf.Vec3f(.42,.45,.48)])
            prim=cube.GetPrim();collision(prim,fixture_material)
            if b['kinematic']:
                body=UsdPhysics.RigidBodyAPI.Apply(prim);body.CreateKinematicEnabledAttr(True)
            prims[b['id']]=prim

        o=subject;oid=o['instance_id'];root='/World/'+oid
        with np.load(out/geom_index[oid]['path'],allow_pickle=False) as data:
            rest_vertices=data['vertices'].copy();triangles=data['triangles'].copy()
            mass=float(data['mass_kg']);inertia=data['inertia_kg_m2'].copy()
        position=fixture['subject_position_m'];q=o['orientation_xyzw']
        xf=UsdGeom.Xform.Define(stage,root);xf.AddTranslateOp().Set(Gf.Vec3d(*position));xf.AddOrientOp().Set(Gf.Quatf(q[3],Gf.Vec3f(*q[:3])))
        visual=UsdGeom.Mesh.Define(stage,root+'/Visual')
        visual.CreatePointsAttr([Gf.Vec3f(*map(float,v)) for v in rest_vertices])
        visual.CreateFaceVertexCountsAttr([3]*len(triangles));visual.CreateFaceVertexIndicesAttr(triangles.ravel().tolist())
        visual.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        visual.CreateDisplayColorAttr([Gf.Vec3f(*o['appearance']['color'])])
        prims[oid]=xf.GetPrim();sim_path=root+'/Simulation';coll_path=root+'/Collision'
        rb=None
        if kind=='rigid':
            rb=UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim());rb.CreateVelocityAttr(Gf.Vec3f(0));rb.CreateAngularVelocityAttr(Gf.Vec3f(0))
            bodypx=PhysxSchema.PhysxRigidBodyAPI.Apply(xf.GetPrim())
            bodypx.CreateSolverPositionIterationCountAttr(numerics['rigid_position_iterations']);bodypx.CreateSolverVelocityIterationCountAttr(numerics['rigid_velocity_iterations'])
            bodypx.CreateLinearDampingAttr(profile['linear_damping']);bodypx.CreateAngularDampingAttr(profile['angular_damping'])
            bodypx.CreateEnableSpeculativeCCDAttr(True)
            ma=UsdPhysics.MassAPI.Apply(xf.GetPrim());ma.CreateMassAttr(mass);ma.CreateCenterOfMassAttr(Gf.Vec3f(0))
            eigenvalues,eigenvectors=np.linalg.eigh(inertia)
            if np.linalg.det(eigenvectors)<0:eigenvectors[:,0]*=-1
            rotation=Gf.Matrix3d(*map(float,eigenvectors.T.ravel())).ExtractRotation().GetQuat()
            ma.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float,eigenvalues)))
            ma.CreatePrincipalAxesAttr(Gf.Quatf(rotation))
            if o['geometry']['rigid_collision']=='sphere':
                collider=UsdGeom.Sphere.Define(stage,coll_path);collider.CreateRadiusAttr(fixture['D_m']/2)
                collider.CreateVisibilityAttr(UsdGeom.Tokens.invisible);collision(collider.GetPrim())
            else:
                collision(visual.GetPrim())
                UsdPhysics.MeshCollisionAPI.Apply(visual.GetPrim()).CreateApproximationAttr(o['geometry']['rigid_collision'])
                if o['geometry']['rigid_collision']=='sdf':
                    PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(visual.GetPrim()).CreateSdfResolutionAttr(128)
            contact_api(xf.GetPrim())
        else:
            ok=deformableUtils.create_auto_volume_deformable_hierarchy(stage,Sdf.Path(root),Sdf.Path(sim_path),
                Sdf.Path(coll_path),visual.GetPath(),True,True,True)
            if not ok:raise RuntimeError('Auto volume hierarchy failed')
            prim=xf.GetPrim();prim.GetAttribute('physxDeformableBody:resolution').Set(o['geometry']['deformable_resolution'])
            prim.GetAttribute('omniphysics:mass').Set(mass)
            prim.GetAttribute('physxDeformableBody:remeshingEnabled').Set(True)
            prim.GetAttribute('physxDeformableBody:forceConforming').Set(True)
            prim.GetAttribute('physxDeformableBody:targetTriangleCount').Set(0)
            prim.ApplyAPI('PhysxBaseDeformableBodyAPI')
            for name,value in {'linearDamping':profile['linear_damping'],'settlingDamping':0.,
                                'solverPositionIterationCount':numerics['deformable_position_iterations'],'selfCollision':False,'enableSpeculativeCCD':True}.items():
                attr=prim.GetAttribute('physxDeformableBody:'+name)
                if not attr or not attr.Set(value):raise RuntimeError('Unavailable deformable attribute: '+name)
            material_prim=stage.GetPrimAtPath(physical)
            # Do not silently ignore a configured material parameter.
            material_prim.ApplyAPI('PhysxDeformableMaterialAPI')
            damping_attr=material_prim.GetAttribute('physxDeformableMaterial:elasticityDamping')
            if damping_attr and damping_attr.IsValid():damping_attr.Set(profile['elasticity_damping'])
            elif profile.get('elasticity_damping',0):
                raise RuntimeError('Configured elasticity damping not exposed by this runtime')
            physicsUtils.add_physics_material_to_prim(stage,prim,Sdf.Path(physical))
            # The deformable material is inherited from the root binding. A
            # second child binding prevented PhysX 110.1's material tensor view
            # from registering the material and silently selected defaults.
            collision(stage.GetPrimAtPath(coll_path),None);contact_api(prim)

        actions=read_json(out/'action.json')['commands'];action_applications=[0]*len(actions);state_dir=out/'state';state_dir.mkdir()
        step_index=0;counts={'headers':0,'points':0,'subject_points':0};contacts_seen=set()
        contact_stream=(out/'contacts.jsonl').open('x',encoding='utf-8') if kind=='rigid' else None

        def on_contact(headers,data):
            for h in headers:
                paths=[str(PhysicsSchemaTools.intToSdfPath(getattr(h,k))) for k in ('actor0','actor1','collider0','collider1')]
                points=[]
                for index in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data):
                    c=data[index]
                    points.append(dict(position_m=list(c.position),normal=list(c.normal),impulse_ns=list(c.impulse),separation_m=float(c.separation)))
                row=dict(schema_version='0.1.0',time_s=step_index*dt,physics_step=step_index,dt_s=dt,event_type=str(h.type),
                         actor0=paths[0],actor1=paths[1],collider0=paths[2],collider1=paths[3],source='physx_native_contact_report',points=points)
                if contact_stream is not None:
                    contact_stream.write(json.dumps(row,allow_nan=False)+'\n')
                counts['headers']+=1;counts['points']+=len(points)
                if any(p==root or p.startswith(root+'/') for p in paths):counts['subject_points']+=len(points)
                contacts_seen.add(tuple(paths[:2]))

        simulation=get_physx_simulation_interface()
        subscription=simulation.subscribe_contact_report_events(on_contact)
        stage_id=UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        # A stopped timeline means app.update only synchronizes/renderers; the
        # sole physics clock is the explicit simulate/fetch loop below.
        for _ in range(3):app.update()
        simulation.attach_stage(stage_id);attached=True
        frames=[];sample_rows=[];bind_tet=None;tet_indices=None;tensor_body=None;tensor_view=None;material_readback=None
        substep_min_j=float('inf');substep_inverted=0;substep_checked=0
        start=time.monotonic()

        def capture(step):
            nonlocal bind_tet,tet_indices
            cache=UsdGeom.XformCache(Usd.TimeCode.Default())
            matrix=cache.GetLocalToWorldTransform(xf.GetPrim())
            transform=Gf.Transform(matrix);tr=transform.GetTranslation();qt=transform.GetRotation().GetQuat()
            orientation=[*map(float,qt.GetImaginary()),float(qt.GetReal())]
            vertices=np.asarray(visual.GetPointsAttr().Get(),dtype=np.float64)
            world=np.asarray([cache.GetLocalToWorldTransform(visual.GetPrim()).Transform(Gf.Vec3d(*v)) for v in vertices])
            centre=np.array(tr);angular=None;linear=np.zeros(3);metrics={};tet_points=np.empty((0,3));tets=np.empty((0,4),dtype=np.int32);nodal=np.empty((0,3))
            if rb is not None:
                linear=np.array(rb.GetVelocityAttr().Get());angular=np.radians(np.array(rb.GetAngularVelocityAttr().Get()))
                energy=.5*mass*float(linear@linear)
                body_rotation=np.array(matrix)[:3,:3].T
                local_w=body_rotation.T@angular;energy+=.5*float(local_w@inertia@local_w)
                inertia_diag=np.linalg.eigvalsh(inertia).tolist()
            else:
                mesh=UsdGeom.TetMesh.Get(stage,sim_path)
                points=mesh.GetPointsAttr().Get()
                if not points:raise RuntimeError('Empty simulation TetMesh')
                tet_points=np.asarray(points,dtype=np.float64)
                tets=np.asarray(mesh.GetTetVertexIndicesAttr().Get(),dtype=np.int32).reshape(-1,4)
                if bind_tet is None:
                    bind=mesh.GetPrim().GetAttribute('deformablePose:default:omniphysics:points').Get()
                    if bind is None:raise RuntimeError('No native bind pose')
                    bind_tet=np.asarray(bind,dtype=np.float64);tet_indices=tets.copy()
                if not np.array_equal(tet_indices,tets):raise RuntimeError('Simulation topology changed')
                metrics=compute_tet_deformation(tet_points,tets,bind_tet)
                metrics['volume_ratio']=float(np.abs(signed_tetrahedron_volumes(tet_points,tets)).sum()/np.abs(signed_tetrahedron_volumes(bind_tet,tets)).sum())
                centre=world.mean(axis=0)
                # At t=0 GPU velocity buffers are not initialized. These zeros
                # are the authored initial condition, not a measured state.
                nodal=np.zeros_like(tet_points)
                if step:
                    for velocity_name in ('omniphysics:velocities','velocities'):
                        velocity_attr=mesh.GetPrim().GetAttribute(velocity_name)
                        values=velocity_attr.Get() if velocity_attr else None
                        if values is not None and len(values)==len(tet_points):
                            nodal=np.asarray(values,dtype=np.float64)
                            break
                    else:
                        nodal=tensor_body.get_simulation_nodal_velocities().numpy()[0,:len(tet_points)].astype(np.float64)
                if nodal.shape!=tet_points.shape:raise RuntimeError('Native velocity/node shape mismatch')
                volumes=np.abs(signed_tetrahedron_volumes(bind_tet,tets));weights=np.zeros(len(bind_tet))
                np.add.at(weights,tets.ravel(),np.repeat(volumes/4,4));weights/=weights.sum()
                linear=weights@nodal;energy=.5*mass*float(weights@(nodal*nodal).sum(axis=1));inertia_diag=None
                sim_matrix=cache.GetLocalToWorldTransform(mesh.GetPrim())
                sim_world=np.asarray([sim_matrix.Transform(Gf.Vec3d(*p)) for p in tet_points])
                centre=weights@sim_world
                metrics['velocity_source']='authored_initial_zero' if step==0 else 'physx_native_nodes'
                metrics['energy_semantics']='derived_lumped_rest_tet_volume_weights_explicit_total_mass'
            metrics['height_m']=float(np.ptp(world[:,2]));metrics['minimum_z_m']=float(world[:,2].min())
            filename=f'state/frame_{len(frames):04d}.npz'
            collision_world=np.empty((0,3));collision_tets=np.empty((0,4),dtype=np.int32)
            if kind=='volumetric':
                collision_mesh=UsdGeom.TetMesh.Get(stage,coll_path)
                collision_points=np.asarray(collision_mesh.GetPointsAttr().Get(),dtype=np.float64)
                collision_matrix=cache.GetLocalToWorldTransform(collision_mesh.GetPrim())
                collision_world=np.asarray([collision_matrix.Transform(Gf.Vec3d(*p)) for p in collision_points])
                collision_tets=np.asarray(collision_mesh.GetTetVertexIndicesAttr().Get(),dtype=np.int32).reshape(-1,4)
            with (out/filename).open('xb') as f:
                np.savez(f,surface_world_m=world.astype(np.float32),surface_triangles=triangles,
                         simulation_points_m=tet_points.astype(np.float32),simulation_tets=tets,
                         simulation_nodal_velocities_m_s=nodal.astype(np.float32),
                         collision_world_m=collision_world.astype(np.float32),collision_tets=collision_tets,
                         simulation_local_to_world=np.array(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(sim_path))) if kind=='volumetric' else np.eye(4),
                         simulation_bind_points_m=np.empty((0,3)) if bind_tet is None else bind_tet,
                         time_s=step*dt)
            body_state=dict(physics_kind=kind,position_m=centre.tolist(),orientation_xyzw=orientation,
                linear_velocity_m_s=linear.tolist(),angular_velocity_rad_s=None if angular is None else angular.tolist(),
                mass_kg=mass,inertia_kg_m2=inertia_diag,kinetic_energy_j=energy,potential_energy_j=-mass*float(gravity@centre),
                geometry=artifact(out,filename),metrics=metrics)
            row=dict(schema_version='0.1.0',time_s=step*dt,physics_step=step,objects={oid:body_state})
            jsonfile=f'state/frame_{len(frames):04d}.json';write_json(out/jsonfile,row)
            frame=dict(time_s=step*dt,physics_step=step,state=jsonfile,geometry=filename,
                       fixture_positions={k:list(map(float,op.Get())) for k,op in translations.items()})
            frame['native_kinematic_poses']={}
            for b in fixture['boxes']:
                if b['kinematic']:
                    actual=get_physx_interface().get_rigidbody_transformation('/World/'+b['id'])
                    frame['native_kinematic_poses'][b['id']]={k:list(v) if hasattr(v,'__len__') and not isinstance(v,str) else v for k,v in actual.items()}
            frames.append(frame);sample_rows.append(row)

        # Force cooking/start without advancing physical time.
        get_physx_interface().force_load_physics_from_usd()
        if kind=='volumetric':
            import omni.physics.tensors as tensors
            tensor_view=tensors.create_simulation_view('warp',stage_id)
            tensor_body=tensor_view.create_volume_deformable_body_view(root)
            if tensor_body.count!=1:raise RuntimeError('Expected exactly one native volume body')
            material_view=tensor_view.create_deformable_material_view(physical+'*')
            try:
                material_readback=dict(status='native',count=material_view.count,
                    youngs_modulus_pa=float(material_view.get_youngs_modulus().numpy()[0,0]),
                    poissons_ratio=float(material_view.get_poissons_ratio().numpy()[0,0]),
                    dynamic_friction=float(material_view.get_dynamic_friction().numpy()[0,0]))
            except Exception as exc:
                material_readback=dict(status='unavailable',count=0,reason=f'{type(exc).__name__}: {exc}')
        capture(0)
        for step in range(steps):
            t=step*dt
            for command_index,command in enumerate(actions):
                if command['kind'] in ('release','remove_support') and due(command,step,hz):
                    UsdPhysics.CollisionAPI(prims[command['target']]).GetCollisionEnabledAttr().Set(False)
                    UsdGeom.Imageable(prims[command['target']]).CreateVisibilityAttr(UsdGeom.Tokens.invisible)
                    action_applications[command_index]+=1
                elif command['kind']=='initial_velocity' and due(command,step,hz):
                    body=UsdPhysics.RigidBodyAPI(prims[command['target']]);p=command['parameters']
                    body.GetVelocityAttr().Set(Gf.Vec3f(*p['linear_m_s']))
                    body.GetAngularVelocityAttr().Set(Gf.Vec3f(*np.degrees(p['angular_rad_s'])))
                    action_applications[command_index]+=1
                elif command['kind']=='kinematic_trajectory' and command['start_time_s']<=t+dt<=command['end_time_s']:
                    translations[command['target']].Set(Gf.Vec3d(*sample_trajectory(command,t+dt)))
                    action_applications[command_index]+=1
            step_index=step+1
            simulation.simulate(dt,t);simulation.fetch_results()
            if kind=='volumetric':
                mesh=UsdGeom.TetMesh.Get(stage,sim_path)
                current=np.asarray(mesh.GetPointsAttr().Get(),dtype=np.float64)
                ratios=signed_tetrahedron_volumes(current,tet_indices)/signed_tetrahedron_volumes(bind_tet,tet_indices)
                substep_min_j=min(substep_min_j,float(np.min(ratios)))
                substep_inverted=max(substep_inverted,int(np.count_nonzero(ratios<0)))
                substep_checked+=1
            if step_index%stride==0:
                app.update();capture(step_index)
                print(f'DATASET {spec["episode_id"]} {step_index}/{steps} t={step_index*dt:.3f} contacts={counts["subject_points"]}',flush=True)
        if contact_stream is not None:
            contact_stream.close();contact_stream=None
        if kind=='volumetric':
            evidence=out/'capability_probes/soft_contact_impulse.json'
            evidence.parent.mkdir()
            write_json(evidence,dict(schema_version='0.1.0',status='unavailable',
                backend_version='Isaac Sim 6.0.1 / PhysX extension 110.1.13',
                probe=f'PhysxContactReportAPI on deformable root, collision tet mesh and all {spec["event_id"]} fixture colliders; threshold=0; callback subscribed before all explicit steps',
                callback_headers=counts['headers'],callback_points=counts['points'],
                reason=f'Installed public contact-report path produced no deformable contact records during verified {spec["event_id"]} fixture interaction; no zero or estimated impulses substituted',
                rigid_control_evidence='r01_native_probe02 reported 1051 subject contact points through the same callback family'))
        write_json(out/'state/index.json',dict(schema_version='0.1.0',complete=True,frames=frames))
        stage.GetRootLayer().Export(str(out/'native_final.usda'))
        report=dict(schema_version='0.1.0',status='physics_completed',elapsed_seconds=time.monotonic()-start,
                    captured_frames=len(frames),simulated_seconds=duration,contact_counts=counts,
                    contact_actor_pairs=[list(p) for p in contacts_seen],physical_representation=kind,
                    runtime='Isaac Sim 6.0.1 / PhysX 110.1.13',dt_s=dt,numerics=numerics,
                    deformable_material_tensor_readback=material_readback,
                    action_applications=[dict(command_index=i,kind=command['kind'],target=command['target'],applications=action_applications[i]) for i,command in enumerate(actions)],
                    deformable_material_attributes={a.GetName():str(a.Get()) for a in stage.GetPrimAtPath(physical).GetAttributes()},
                    substep_tet_audit=None if kind=='rigid' else dict(checked_steps=substep_checked,minimum_j=substep_min_j,inverted_tets=substep_inverted))
        write_json(out/'native_report.json',report)
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

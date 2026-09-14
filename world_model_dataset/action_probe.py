"""Bounded native PhysX smoke probe for the four generic M2 actions."""
from __future__ import annotations

import argparse
import sys
import traceback
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from world_model_dataset.actions import due,sample_trajectory
from world_model_dataset.fixtures import flat_ground,ramp,stairs,obstacles,compression_plates
from world_model_dataset.io import write_json


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'renderer':'RayTracedLighting'})
    simulation=None;attached=False
    try:
        import numpy as np
        import omni.usd
        from omni.physx import get_physx_simulation_interface,get_physx_interface
        from pxr import Gf,UsdGeom,UsdPhysics,PhysxSchema,UsdUtils
        omni.usd.get_context().new_stage();stage=omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(stage,1.)
        UsdGeom.Xform.Define(stage,'/World');stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
        scene=UsdPhysics.Scene.Define(stage,'/World/PhysicsScene');scene.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1));scene.CreateGravityMagnitudeAttr(9.81)
        px=PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim());px.CreateTimeStepsPerSecondAttr(120);px.CreateSolverTypeAttr('TGS')

        translations={};prims={}
        def cube(name,position,size,rigid=False,kinematic=False):
            shape=UsdGeom.Cube.Define(stage,'/World/'+name);shape.CreateSizeAttr(1.)
            translations[name]=shape.AddTranslateOp();translations[name].Set(Gf.Vec3d(*position));shape.AddScaleOp().Set(Gf.Vec3f(*size))
            prim=shape.GetPrim();UsdPhysics.CollisionAPI.Apply(prim)
            if rigid or kinematic:
                body=UsdPhysics.RigidBodyAPI.Apply(prim);body.CreateKinematicEnabledAttr(kinematic)
            prims[name]=prim;return prim

        cube('ground',[0,0,-.05],[8,8,.1])
        cube('release_support',[-1,-1,.45],[.6,.6,.1],kinematic=True)
        cube('remove_support',[-1,1,.45],[.6,.6,.1],kinematic=True)
        cube('release_body',[-1,-1,.65],[.2,.2,.2],rigid=True)
        cube('remove_body',[-1,1,.65],[.2,.2,.2],rigid=True)
        cube('velocity_body',[0,0,.15],[.2,.2,.2],rigid=True)
        cube('trajectory_body',[1,0,.5],[.25,.25,.25],kinematic=True)
        commands=[
            dict(kind='release',target='release_support',start_time_s=.1,end_time_s=.1,parameters={'method':'disable_collision'}),
            dict(kind='remove_support',target='remove_support',start_time_s=.2,end_time_s=.2,parameters={'method':'disable_collision'}),
            dict(kind='initial_velocity',target='velocity_body',start_time_s=.1,end_time_s=.1,
                 parameters={'linear_m_s':[1.,0.,0.],'angular_rad_s':[0.,0.,0.]}),
            dict(kind='kinematic_trajectory',target='trajectory_body',start_time_s=.1,end_time_s=.6,
                 parameters={'interpolation':'smoothstep','from_m':[1.,0.,.5],'to_m':[2.,0.,.5]}),
        ]
        # Load every standard fixture family through the same native box path.
        fixture_sets={'flat_ground':flat_ground(.2),'ramp':ramp(.2,20.),'stairs':stairs(.2),
                      'obstacles':obstacles(.2),'compression_plates':compression_plates(.2,.2)}
        fixture_counts={}
        for group,(name,boxes) in enumerate(fixture_sets.items()):
            fixture_counts[name]=len(boxes)
            for index,b in enumerate(boxes):
                p=np.asarray(b['position_m'])+[0,10.+group*3.,0]
                prim=cube(f'fixture_{group}_{index}',p,b['size_m'],kinematic=b['kinematic'])
                q=b['orientation_xyzw'];UsdGeom.Xformable(prim).AddOrientOp().Set(Gf.Quatf(q[3],Gf.Vec3f(*q[:3])))

        simulation=get_physx_simulation_interface();stage_id=UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        for _ in range(3):app.update()
        simulation.attach_stage(stage_id);attached=True;get_physx_interface().force_load_physics_from_usd()
        applied={c['kind']:0 for c in commands};dt=1/120
        for step in range(120):
            time_s=step*dt
            for command in commands:
                if command['kind'] in ('release','remove_support') and due(command,step,120):
                    UsdPhysics.CollisionAPI(prims[command['target']]).GetCollisionEnabledAttr().Set(False);applied[command['kind']]+=1
                elif command['kind']=='initial_velocity' and due(command,step,120):
                    p=command['parameters'];body=UsdPhysics.RigidBodyAPI(prims[command['target']])
                    body.GetVelocityAttr().Set(Gf.Vec3f(*p['linear_m_s']));body.GetAngularVelocityAttr().Set(Gf.Vec3f(*np.degrees(p['angular_rad_s'])));applied[command['kind']]+=1
                elif command['kind']=='kinematic_trajectory' and command['start_time_s']<=time_s<=command['end_time_s']:
                    translations[command['target']].Set(Gf.Vec3d(*sample_trajectory(command,time_s)));applied[command['kind']]+=1
            simulation.simulate(dt,time_s);simulation.fetch_results()
            if (step+1)%20==0:app.update()
        poses={name:get_physx_interface().get_rigidbody_transformation('/World/'+name)
               for name in ('release_body','remove_body','velocity_body','trajectory_body')}
        positions={name:list(map(float,value['position'])) for name,value in poses.items()}
        checks={
            'each_one_shot_exactly_once':all(applied[k]==1 for k in ('release','remove_support','initial_velocity')),
            'release_causes_fall':positions['release_body'][2]<.2,
            'remove_support_causes_fall':positions['remove_body'][2]<.2,
            'initial_velocity_causes_translation':positions['velocity_body'][0]>.05,
            'kinematic_trajectory_reaches_target':abs(positions['trajectory_body'][0]-2.)<1e-4,
            'five_fixture_families_loaded':set(fixture_counts)=={'flat_ground','ramp','stairs','obstacles','compression_plates'},
        }
        write_json(args.output,dict(schema_version='0.1.0',backend_version='Isaac Sim 6.0.1 / PhysX extension 110.1.13',
            physics_hz=120,commands=commands,applications=applied,final_positions_m=positions,
            fixture_primitive_counts=fixture_counts,checks=checks,passed=all(checks.values())))
        print('M2_ACTION_PROBE '+str(all(checks.values())),flush=True)
    except Exception as exc:
        traceback.print_exc()
        if not args.output.exists():write_json(args.output,dict(schema_version='0.1.0',passed=False,error=f'{type(exc).__name__}: {exc}'))
        raise
    finally:
        if attached:simulation.detach_stage()
        app.close()


if __name__=='__main__':main()

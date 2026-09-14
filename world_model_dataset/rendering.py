"""Isaac RTX replay renderer: a physics-free stage built from fixed native caches."""
from __future__ import annotations

import argparse
import json
import sys
import traceback
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from world_model_dataset.io import read_json,write_json
from world_model_dataset.motion import fixed_topology_motion


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--episode',type=Path,required=True)
    parser.add_argument('--max-frames',type=int,default=0,help='Explicit diagnostic prefix; never a complete episode')
    args=parser.parse_args();out=args.episode
    physics=read_json(out/'physics_validation.json')
    if not physics['passed']:raise ValueError('Physical audit must pass before rendering')
    dest=out/'observations';dest.mkdir(exist_ok=False)
    ep=read_json(out/'episode.prepared.json');index=read_json(out/'state/index.json');fixture=read_json(out/'fixture.json')
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'renderer':'RayTracedLighting','width':640,'height':480})
    try:
        import numpy as np
        import omni.usd
        import omni.replicator.core as rep
        from PIL import Image
        from pxr import UsdGeom,UsdLux,UsdShade,Sdf,Gf,Semantics
        rep.orchestrator.set_capture_on_play(False)
        omni.usd.get_context().new_stage();stage=omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(stage,1.)
        UsdGeom.Xform.Define(stage,'/World')
        # Deliberately no PhysicsScene/RigidBodyAPI/deformable APIs in this stage.
        light=UsdLux.DomeLight.Define(stage,'/World/Light');light.CreateIntensityAttr(700.)
        sun=UsdLux.DistantLight.Define(stage,'/World/Key');sun.CreateIntensityAttr(1600.)
        sun.AddRotateXYZOp().Set(Gf.Vec3f(-35,-25,-15))

        def label(prim,name):
            sem=Semantics.SemanticsAPI.Apply(prim,'Semantics')
            sem.CreateSemanticTypeAttr('class');sem.CreateSemanticDataAttr(name)

        ops={};boxes={}
        for b in fixture['boxes']:
            cube=UsdGeom.Cube.Define(stage,'/World/'+b['id']);cube.CreateSizeAttr(1.)
            ops[b['id']]=cube.AddTranslateOp();ops[b['id']].Set(Gf.Vec3d(*b['position_m']))
            q=b['orientation_xyzw'];cube.AddOrientOp().Set(Gf.Quatf(q[3],Gf.Vec3f(*q[:3])))
            cube.AddScaleOp().Set(Gf.Vec3f(*b['size_m']));cube.CreateDisplayColorAttr([Gf.Vec3f(.42,.45,.48)])
            label(cube.GetPrim(),b['id']);boxes[b['id']]=cube
        oid=ep['spec']['objects'][0]['instance_id'];subject=UsdGeom.Mesh.Define(stage,'/World/'+oid)
        subject.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none);label(subject.GetPrim(),oid)
        appearance=ep['inputs']['objects'][0]['appearance']
        material=UsdShade.Material.Define(stage,'/World/Appearance');shader=UsdShade.Shader.Define(stage,'/World/Appearance/Shader')
        shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*appearance['color']))
        for name in ('roughness','metallic','opacity'):shader.CreateInput(name,Sdf.ValueTypeNames.Float).Set(appearance[name])
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
        UsdShade.MaterialBindingAPI.Apply(subject.GetPrim()).Bind(material)
        cameras=ep['inputs']['cameras'];w,h=cameras['resolution'];d=fixture['D_m']
        # Fixed by event/fixture, not fitted to actual (counterfactual) trajectories.
        scale=d*(10. if ep['spec']['event_id']=='R01' else 1.)
        offset=np.array([25*d,0,0]) if ep['spec']['event_id']=='R01' else np.zeros(3)
        streams={};calibration={}
        for c in cameras['cameras']:
            name=c['id'];cam=UsdGeom.Camera.Define(stage,'/World/camera_'+name)
            eye=np.array(c['position_D'])*scale+offset;target=np.array(c['target_D'])*scale+offset
            matrix=Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1)).GetInverse()
            cam.AddTransformOp().Set(matrix);cam.CreateFocalLengthAttr(cameras['focal_length_mm'])
            cam.CreateHorizontalApertureAttr(cameras['horizontal_aperture_mm'])
            cam.CreateVerticalApertureAttr(cameras['horizontal_aperture_mm']*h/w)
            cam.CreateClippingRangeAttr(Gf.Vec2f(.01,100))
            rp=rep.create.render_product(str(cam.GetPath()),(w,h))
            anns={key:rep.AnnotatorRegistry.get_annotator(key) for key in ('rgb','distance_to_image_plane','semantic_segmentation','normals','camera_params')}
            for a in anns.values():a.attach([rp])
            streams[name]=anns;(dest/name).mkdir()
            f=w*cameras['focal_length_mm']/cameras['horizontal_aperture_mm']
            calibration[name]=dict(world_from_camera_usd=np.array(matrix).tolist(),matrix_convention='USD row-vector',
                                   intrinsic_opencv=[[f,0,w/2],[0,f,h/2],[0,0,1]],resolution=[w,h],
                                   optical_axis_conversion='USD camera (x,y,z) -> OpenCV (x,-y,-z)')
        frames=index['frames'][:args.max_frames] if args.max_frames else index['frames'];outputs=[];previous_surface=None
        for i,frame in enumerate(frames):
            with np.load(out/frame['geometry'],allow_pickle=False) as data:
                surface=np.array(data['surface_world_m'],copy=True);faces=np.array(data['surface_triangles'],copy=True)
                subject.CreatePointsAttr([Gf.Vec3f(*map(float,v)) for v in surface])
                subject.CreateFaceVertexCountsAttr([3]*len(faces))
                subject.CreateFaceVertexIndicesAttr(faces.ravel().tolist())
            for name,p in frame['fixture_positions'].items():ops[name].Set(Gf.Vec3d(*p))
            if 'gate' in boxes:
                visible=frame['time_s']<=ep['spec']['action_parameters']['release_time_s']
                boxes['gate'].CreateVisibilityAttr(UsdGeom.Tokens.inherited if visible else UsdGeom.Tokens.invisible)
            rep.orchestrator.step(rt_subframes=1,delta_time=1/ep['spec']['timing']['capture_hz'],pause_timeline=True)
            if i==0:rep.orchestrator.step(rt_subframes=1,delta_time=0.,pause_timeline=True)
            for name,anns in streams.items():
                rgb=np.asarray(anns['rgb'].get_data());depth=np.asarray(anns['distance_to_image_plane'].get_data())
                seg=anns['semantic_segmentation'].get_data();normal=np.asarray(anns['normals'].get_data())
                if rgb.shape[:2]!=(h,w) or depth.shape!=(h,w):raise ValueError('Missing/misaligned RGB-D')
                valid=np.isfinite(depth)&(depth>0)&(depth<100)
                labels=[int(k) for k,v in seg['info']['idToLabels'].items() if v.get('class')==oid]
                motion,motion_valid=(np.zeros((h,w,2),dtype=np.float32),np.zeros((h,w),dtype=bool)) if previous_surface is None else fixed_topology_motion(
                    previous_surface,surface,faces,depth,np.asarray(seg['data'],dtype=np.uint32),labels,calibration[name])
                rgbpath=f'{name}/frame_{i:04d}.png';arraypath=f'{name}/frame_{i:04d}.npz'
                Image.fromarray(rgb[:,:,:3].astype(np.uint8)).save(dest/rgbpath)
                with (dest/arraypath).open('xb') as stream:
                    np.savez_compressed(stream,depth_m=np.where(valid,depth,0).astype(np.float32),depth_valid=valid,
                                        segmentation=np.asarray(seg['data'],dtype=np.uint32),normals_world=normal[...,:3].astype(np.float32),
                                        motion_previous_minus_current_px=motion,motion_valid=motion_valid,
                                        time_s=frame['time_s'])
                outputs.append(dict(time_s=frame['time_s'],physics_step=frame['physics_step'],camera_id=name,
                                    rgb=rgbpath,data=arraypath,segmentation_labels=seg['info']['idToLabels']))
            print(f'DATASET_RENDER {i+1}/{len(frames)}',flush=True)
            previous_surface=surface
        write_json(dest/'index.json',dict(schema_version='0.1.0',complete=not args.max_frames,frames=outputs,
                    cameras=calibration,render_backend='Isaac RTX',physics_rerun=False,
                    depth_units='m_optical_axis',normals_convention='world XYZ unit vectors; verified against analytical fixture normals during finalization',
                    motion_vectors_convention='derived fixed-topology subject material motion in image pixels, previous minus current: left/up positive; validity mask excludes background and frame 0',
                    renderer_source_sha256=__import__('hashlib').sha256(Path(__file__).read_bytes()).hexdigest()))
        print('DATASET_RENDER_COMPLETE',flush=True)
    except Exception as exc:
        traceback.print_exc();write_json(dest/'failure.json',dict(error=f'{type(exc).__name__}: {exc}'))
        raise
    finally:app.close()


if __name__=='__main__':main()

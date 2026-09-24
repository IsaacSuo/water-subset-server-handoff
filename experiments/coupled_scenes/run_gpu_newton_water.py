"""Run a prepared water scene on GPU DFSPH + Newton; basic integrity checks only."""
import argparse
import contextlib
import importlib.metadata
import json
import math
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from coupled_scene.gpu_dfsph.assets import scene_configuration
from coupled_scene.active_drive import motion_state
from coupled_scene.newton_dfsph.surface_assets import surface_motion


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=1/30)
    parser.add_argument('--hz',type=int,default=1200)
    parser.add_argument('--upstream',type=Path,default=Path('/mnt/y/tools/SPH_Project'))
    args=parser.parse_args()
    if args.hz<=0 or args.hz%30 or args.seconds<=0 or not math.isfinite(args.seconds) or abs(args.seconds*30-round(args.seconds*30))>1e-6:
        parser.error('Duration and time step must align to the 30 fps output')
    args.output.mkdir(parents=True,exist_ok=False);capture=args.output/'capture';capture.mkdir()
    info=json.loads((args.input/'input.json').read_text(encoding='utf-8'));meta=info['source_assets']
    with np.load(args.input/'input.npz') as data:arrays={k:data[k] for k in data.files}
    dt=1/args.hz;config=scene_configuration(info,dt)
    (args.output/'scene.json').write_text(json.dumps(config,indent=2))
    for name in ('receiver','donor'):
        if (args.input/(name+'.obj')).exists():shutil.copy2(args.input/(name+'.obj'),args.output/(name+'.obj'))
    bodies=[dict(b,vertices=arrays[b['name']+'_vertices'],triangles=arrays[b['name']+'_triangles'],samples=arrays[b['name']+'_samples']) for b in info['body_specs']]
    origin=np.asarray(info['origin_m']);shift=np.asarray(info['simulation_shift_m']);source_ids=arrays['source_ids']
    report=dict(status='initializing',backend='gpu_dfsph_newton',scope='independent_water_subset',
        scene_family=meta.get('scene_family','active_drive'),case_id=meta['case']['id'],fluid_device='cuda',rigid_engine='newton',
        particle_count=len(source_ids),spacing_m=info['spacing_m'],origin_m=origin.tolist(),source_assets=str(args.input),
        requested_seconds=args.seconds,simulation_hz=args.hz,
        diagnostic_only=True,workflow_check=True,production_accepted=False,reference_rigid_step=False,
        recycling=dict(enabled=False,removed_count=0),versions={n:importlib.metadata.version(n) for n in ('taichi','newton','warp-lang')},
        upstream_commit=subprocess.check_output(['git','-C',str(args.upstream),'rev-parse','HEAD'],text=True).strip(),rows=[])
    report.update({k:meta[k] for k in ('source_blend_sha256','geometry_sha256') if k in meta})
    frames=[];start=time.perf_counter()
    def save():
        temp=args.output/'probe_report.tmp';temp.write_text(json.dumps(report,indent=2),encoding='utf-8');temp.replace(args.output/'probe_report.json')
    save()
    try:
        from coupled_scene.gpu_dfsph.backend import create_backend
        import taichi as ti
        with (args.output/'solver.log').open('w',buffering=1024*1024) as log,contextlib.redirect_stdout(log):
            c,solver=create_backend(args.output/'scene.json',arrays['positions'],arrays['velocities'],bodies,info['spacing_m'],dt,args.upstream)
            rigid=solver.rigid_solver;motion=meta['case']['motion']
            report['rigid_collision_geometry']=rigid.collision_geometry
            if motion and meta['case']['id']=='02_stirring':motion=dict(motion,speed_rad_s=3.6)
            report['motion']=motion
            drives={}
            for b in bodies:
                if not b['prescribed']:continue
                pivot=np.asarray(b['translation'])
                def drive(t,com,pivot=pivot):
                    s=(surface_motion(motion,t) if report['scene_family']=='surface_study' or motion is None else motion_state(motion,t))
                    axis=np.zeros(3);axis[s['rotation_axis']]=s['angle_rad']
                    q=Rotation.from_rotvec(axis).as_quat();omega=np.asarray(s['angular_velocity_rad_s'])
                    v=np.asarray(s['linear_velocity_m_s'])+np.cross(omega,Rotation.from_quat(q).apply(com))
                    return np.r_[pivot+s['displacement_m'],q],np.r_[v,omega]
                drives[b['objectId']]=drive
            rigid.set_drives(drives)
            solver.renew_rigid_particle_state()
            report['status']='running';save()
            def capture_frame(t):
                n=c.particle_num[None];mask=c.particle_materials.to_numpy()[:n]==1
                p=c.particle_positions.to_numpy()[:n][mask];v=c.particle_velocities.to_numpy()[:n][mask]
                ids=c.stable_ids.to_numpy()[:n][mask]
                if not np.isfinite(p).all() or not np.isfinite(v).all():raise RuntimeError('Nonfinite fluid arrays')
                if len(ids)!=len(source_ids) or not np.array_equal(np.sort(ids),np.arange(len(source_ids))):raise RuntimeError('Fluid identity mismatch')
                q=rigid.state.body_q.numpy();qd=rigid.state.body_qd.numpy()
                donor=next(b for b in bodies if b['name']=='donor');index,_=rigid.bindings[donor['objectId']]
                pose=q[index].copy();pose[:3]+=origin-shift
                filename=f'frame_{len(frames):04d}.npz'
                np.savez(capture/filename,positions=(p+origin-shift).astype(np.float32),velocities=v,ids=source_ids[ids],
                    simulated_seconds=t,body_q=q,body_qd=qd)
                frames.append(dict(file=filename,recording_seconds=t,action_seconds=t,particle_count=len(ids),recycled_count=0,
                    native_position_m=pose[:3].tolist(),native_rotation_xyzw=pose[3:].tolist()))
                report['rows'].append(dict(action_seconds=t,rms_speed_m_s=float(np.sqrt(np.mean(np.sum(v*v,axis=1)))),
                    body_q=q.tolist(),body_qd=qd.tolist(),liquid_wrenches={str(k):v.tolist() for k,v in rigid.last_wrenches.items()},
                    applied_wrenches={str(k):v.tolist() for k,v in rigid.last_applied_wrenches.items()}))
                report.update(last_action_seconds=t,recorded_frames=len(frames),wall_seconds=time.perf_counter()-start);save()
            capture_frame(0.)
            for step in range(1,round(args.seconds*args.hz)+1):
                solver.step()
                if step%(args.hz//30)==0:ti.sync();capture_frame(step*dt)
            report['status']='completed'
    except BaseException as exc:
        report.update(status='failed',error=f'{type(exc).__name__}: {exc}');raise
    finally:
        report['wall_seconds']=time.perf_counter()-start;save()
        (capture/'manifest.json').write_text(json.dumps(dict(complete=report['status']=='completed',backend=report['backend'],
            fps=30,frames=frames,source_assets=meta,moving_object=meta.get('moving_object','PouringPitcher'),
            diagnostic_only=True,workflow_check=True,production_accepted=False,recycling=report['recycling']),indent=2),encoding='utf-8')
    print(json.dumps({k:report.get(k) for k in ('status','backend','case_id','last_action_seconds','wall_seconds')}),flush=True)


if __name__=='__main__':main()

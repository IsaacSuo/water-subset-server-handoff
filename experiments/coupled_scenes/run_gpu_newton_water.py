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
from coupled_scene.gpu_dfsph.server_paths import mapping_entry, resolve_handoff_path
from coupled_scene.active_drive import motion_state
from coupled_scene.newton_dfsph.surface_assets import surface_motion


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=1/30)
    parser.add_argument('--hz',type=int,default=1200)
    parser.add_argument('--upstream',type=Path,default=Path('/mnt/y/tools/SPH_Project'))
    parser.add_argument('--stirring-speed-rad-s',type=float,default=10.,
        help='Explicit target speed for the stirring case; does not alter its authored ramp timing')
    parser.add_argument('--surface-tension',type=float,default=0.,
        help='SPH_Project numerical surface-tension coefficient; recorded explicitly in the report')
    parser.add_argument('--thin-feature-stabilization',action=argparse.BooleanOptionalAction,default=True,
        help='Geometry-aware anisotropic fluid pressure filtering for deficient thin sheets and jets')
    parser.add_argument('--initial-frame',type=Path,
        help='Diagnostic warm start from one of this workflow\'s captured frames; full runs must omit it')
    args=parser.parse_args()
    if args.hz<=0 or args.hz%30 or args.seconds<=0 or not math.isfinite(args.seconds) or abs(args.seconds*30-round(args.seconds*30))>1e-6:
        parser.error('Duration and time step must align to the 30 fps output')
    if not math.isfinite(args.surface_tension) or args.surface_tension<0:
        parser.error('Surface tension must be finite and nonnegative')
    args.output.mkdir(parents=True,exist_ok=False);capture=args.output/'capture';capture.mkdir()
    info=json.loads((args.input/'input.json').read_text(encoding='utf-8'));meta=info['source_assets']
    source_paths=[]
    for key in ('source_blend','source_layout'):
        if meta.get(key):source_paths.append(mapping_entry(key,meta[key],resolve_handoff_path(meta[key],ROOT)))
    (args.output/'source_input.json').write_text((args.input/'input.json').read_text(encoding='utf-8'),encoding='utf-8')
    (args.output/'server_path_mapping.json').write_text(json.dumps(dict(source_input=str(args.input.resolve()),
        source_input_preserved=True,mappings=source_paths),indent=2),encoding='utf-8')
    with np.load(args.input/'input.npz') as data:arrays={k:data[k] for k in data.files}
    start_time=0.;warm_start_roundoff=None
    if args.initial_frame:
        with np.load(args.initial_frame) as frame:
            frame_ids=np.asarray(frame['ids']);source_ids=np.asarray(arrays['source_ids'])
            source_order=np.argsort(source_ids);frame_order=np.argsort(frame_ids)
            if len(frame_ids)!=len(source_ids) or not np.array_equal(source_ids[source_order],frame_ids[frame_order]):
                raise ValueError('Initial frame particle identities do not match the prepared input')
            positions=np.empty_like(frame['positions']);velocities=np.empty_like(frame['velocities'])
            positions[source_order]=frame['positions'][frame_order]
            velocities[source_order]=frame['velocities'][frame_order]
            start_time=float(frame['simulated_seconds'])
        if start_time<0 or start_time>=args.seconds or abs(start_time*30-round(start_time*30))>1e-6:
            parser.error('Initial frame time must be a 30 fps boundary before --seconds')
        origin=np.asarray(info['origin_m']);shift=np.asarray(info['simulation_shift_m'])
        simulation_positions=(positions-origin+shift).astype(np.float32)
        # Captures are world-space float32.  The inverse transform can move an
        # exactly clamped domain coordinate by a few ulps across the padding.
        padding=2*float(info['spacing_m']);upper=np.asarray(info['domain_end'])-padding
        corrected=np.clip(simulation_positions,padding,upper).astype(np.float32)
        adjustment=np.abs(corrected-simulation_positions)
        if float(adjustment.max())>1e-6:
            raise ValueError('Initial frame lies outside the numerical domain by more than float32 roundoff')
        warm_start_roundoff=dict(corrected_components=int(np.count_nonzero(adjustment)),
            corrected_particles=int(np.count_nonzero(np.any(adjustment>0,axis=1))),
            maximum_adjustment_m=float(adjustment.max()))
        arrays['positions']=corrected
        arrays['velocities']=velocities.astype(np.float32)
    dt=1/args.hz;config=scene_configuration(info,dt)
    (args.output/'scene.json').write_text(json.dumps(config,indent=2))
    for name in ('receiver','donor'):
        if (args.input/(name+'.obj')).exists():shutil.copy2(args.input/(name+'.obj'),args.output/(name+'.obj'))
    bodies=[dict(b,vertices=arrays[b['name']+'_vertices'],triangles=arrays[b['name']+'_triangles'],samples=arrays[b['name']+'_samples']) for b in info['body_specs']]
    origin=np.asarray(info['origin_m']);shift=np.asarray(info['simulation_shift_m']);source_ids=arrays['source_ids']
    report=dict(status='initializing',backend='gpu_dfsph_newton',scope='independent_water_subset',
        scene_family=meta.get('scene_family','active_drive'),case_id=meta['case']['id'],fluid_device='cuda',rigid_engine='newton',
        particle_count=len(source_ids),spacing_m=info['spacing_m'],origin_m=origin.tolist(),source_assets=str(args.input),
        requested_seconds=args.seconds,simulation_hz=args.hz,surface_tension=args.surface_tension,
        thin_feature_stabilization=dict(enabled=args.thin_feature_stabilization,
            method='fluid_covariance_anisotropic_dfsph_pressure_filter',fluid_neighbor_limit=20,
            alpha_operator='upstream_isotropic_reference',ordinary_free_surface_protection='first_moment_asymmetry',
            rigid_contact_pressure_filtered=False,particle_count_changes=False),
        initial_seconds=start_time,initial_frame=str(args.initial_frame.resolve()) if args.initial_frame else None,
        warm_start_diagnostic=bool(args.initial_frame),warm_start_roundoff=warm_start_roundoff,
        time_integration=dict(method='moving_boundary_cfl_with_rollback',base_hz=args.hz,
            cfl_displacement_fraction=.25,minimum_dt_s=dt/64,frame_alignment_hz=30),
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
            c,solver=create_backend(args.output/'scene.json',arrays['positions'],arrays['velocities'],bodies,info['spacing_m'],dt,args.upstream,
                surface_tension=args.surface_tension,thin_feature_stabilization=args.thin_feature_stabilization)
            rigid=solver.rigid_solver;motion=meta['case']['motion']
            report['rigid_collision_geometry']=rigid.collision_geometry
            if motion and meta['case']['id']=='02_stirring':
                if args.stirring_speed_rad_s<=0 or not math.isfinite(args.stirring_speed_rad_s):
                    raise ValueError('Stirring speed must be finite and positive')
                motion=dict(motion,speed_rad_s=args.stirring_speed_rad_s)
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
            c.total_time=start_time;rigid.total_time=start_time
            rigid.apply_drives(start_time);rigid.publish()
            solver.renew_rigid_particle_state()
            report['energy_accounting']=dict(reference_seconds=start_time,
                motor_work_sign='positive means prescribed rigid motion does work on liquid',
                mechanical_energy='fluid kinetic plus gravitational potential energy',
                balance='motor work minus change in fluid mechanical energy; includes physical/numerical dissipation')
            report['status']='running';save()
            energy_reference=None
            def capture_frame(t):
                nonlocal energy_reference
                n=c.particle_num[None];mask=c.particle_materials.to_numpy()[:n]==1
                p=c.particle_positions.to_numpy()[:n][mask];v=c.particle_velocities.to_numpy()[:n][mask]
                masses=c.particle_masses.to_numpy()[:n][mask].astype(np.float64)
                ids=c.stable_ids.to_numpy()[:n][mask]
                if not np.isfinite(p).all() or not np.isfinite(v).all():raise RuntimeError('Nonfinite fluid arrays')
                if len(ids)!=len(source_ids) or not np.array_equal(np.sort(ids),np.arange(len(source_ids))):raise RuntimeError('Fluid identity mismatch')
                q=rigid.state.body_q.numpy();qd=rigid.state.body_qd.numpy()
                world_positions=p+origin-shift
                kinetic_energy=.5*float(np.sum(masses*np.sum(v.astype(np.float64)**2,axis=1)))
                potential_energy=9.81*float(np.sum(masses*world_positions[:,1]))
                mechanical_energy=kinetic_energy+potential_energy
                if energy_reference is None:energy_reference=mechanical_energy
                motor_work=float(sum(rigid.cumulative_prescribed_work.values()))
                energy_ledger=dict(fluid_mass_kg=float(masses.sum()),kinetic_energy_j=kinetic_energy,
                    gravitational_potential_energy_j=potential_energy,mechanical_energy_j=mechanical_energy,
                    mechanical_energy_change_j=mechanical_energy-energy_reference,
                    prescribed_motor_work_to_fluid_j=motor_work,
                    balance_motor_minus_mechanical_change_j=motor_work-(mechanical_energy-energy_reference),
                    prescribed_power_w={str(k):float(value) for k,value in rigid.last_prescribed_power.items()},
                    cumulative_fluid_impulse={str(k):value.tolist() for k,value in rigid.cumulative_fluid_impulse.items()})
                donor=next(b for b in bodies if b['name']=='donor');index,_=rigid.bindings[donor['objectId']]
                pose=q[index].copy();pose[:3]+=origin-shift
                filename=f'frame_{len(frames):04d}.npz'
                np.savez(capture/filename,positions=world_positions.astype(np.float32),velocities=v,ids=source_ids[ids],
                    simulated_seconds=t,body_q=q,body_qd=qd)
                frames.append(dict(file=filename,recording_seconds=t,action_seconds=t,particle_count=len(ids),recycled_count=0,
                    native_position_m=pose[:3].tolist(),native_rotation_xyzw=pose[3:].tolist()))
                report['rows'].append(dict(action_seconds=t,rms_speed_m_s=float(np.sqrt(np.mean(np.sum(v*v,axis=1)))),
                    body_q=q.tolist(),body_qd=qd.tolist(),liquid_wrenches={str(k):v.tolist() for k,v in rigid.last_wrenches.items()},
                    applied_wrenches={str(k):v.tolist() for k,v in rigid.last_applied_wrenches.items()},
                    integrator=dict(solver.last_step_metrics),energy=energy_ledger))
                report.update(last_action_seconds=t,recorded_frames=len(frames),wall_seconds=time.perf_counter()-start);save()
            capture_frame(start_time)
            for frame_index in range(round(start_time*30)+1,round(args.seconds*30)+1):
                target=frame_index/30
                while c.total_time<target-1e-9:
                    solver.adaptive_step(target-c.total_time)
                if abs(c.total_time-target)<=1e-9:
                    c.total_time=target;rigid.total_time=target
                solver.collect_frame_diagnostics();ti.sync()
                report['time_integration'].update(accepted_substeps=solver.accepted_substeps,
                    rejected_substeps=solver.rejected_substeps,
                    frame_tail_redistributions=solver.frame_tail_redistributions,
                    minimum_accepted_dt_s=solver.minimum_accepted_dt,
                    maximum_accepted_dt_s=solver.maximum_accepted_dt,
                    next_step_cap_s=solver.adaptive_dt_cap,
                    effective_average_hz=solver.accepted_substeps/(target-start_time))
                capture_frame(target)
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

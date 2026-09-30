"""Serial A/B pressure verification and matched native-cache rendering."""

import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
from scipy.spatial import cKDTree


ROOT = Path(__file__).resolve().parents[2]


def write_json(path, value):
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(value, indent=2, ensure_ascii=False))
    temp.replace(path)


def summarize(simulation, window):
    report = json.loads((simulation / 'probe_report.json').read_text())
    rows = [row for row in report['rows'] if window[0]-1.e-8 <= row['action_seconds'] <= window[1]+1.e-8]
    result = dict(status=report['status'],last_time_s=report.get('last_action_seconds'),
        wall_seconds=report.get('wall_seconds'),time_integration=report['time_integration'],
        receiver_rebound=report.get('receiver_rebound_summary'),sampled_frames=[])
    for row in rows:
        t = row['action_seconds']
        frame = simulation / 'capture' / f'frame_{round(t*30):04d}.npz'
        if report.get('initial_seconds',0.0) != 0.0:
            raise ValueError('A/B verification must start from the original input')
        with np.load(frame) as data:
            positions = data['positions'].astype(np.float64)
            # Counts exclude self and include only real fluid particles.  Match
            # the solver's strict support test rather than a larger render kernel.
            counts = cKDTree(positions).query_ball_point(positions,
                np.nextafter(2.*report['spacing_m'],0.),workers=8,return_length=True)-1
        energy = row['energy']
        result['sampled_frames'].append(dict(time_s=t,
            deficient_particle_count=int(np.count_nonzero(counts<20)),
            kinetic_energy_j=energy['kinetic_energy_j'],
            gravitational_potential_energy_j=energy['gravitational_potential_energy_j'],
            mechanical_energy_j=energy['mechanical_energy_j'],
            prescribed_motor_work_to_fluid_j=energy['prescribed_motor_work_to_fluid_j'],
            mechanical_minus_reported_motor_work_j=energy['mechanical_energy_j']-energy['prescribed_motor_work_to_fluid_j'],
            projection_status=row['integrator'].get('pressure_projection_status'),
            wall_density=row.get('wall_density')))
    trials = simulation / 'pressure_substeps.jsonl'
    accepted=[];rejected=0
    if trials.exists():
        with trials.open() as stream:
            for line in stream:
                record=json.loads(line)
                if window[0] <= record['end_time_s'] <= window[1]+1.e-8:
                    accepted.append(record)
                    rejected += sum(not trial['accepted'] for trial in record['trials'])
        result['collision_substeps']=dict(accepted=len(accepted),rejected_trials=rejected,
            accepted_unconverged=sum(record['metrics']['accepted_unconverged'] for record in accepted),
            density_cap=sum(record['metrics']['density_iterations']>=300 for record in accepted),
            divergence_cap=sum(record['metrics']['divergence_iterations']>=300 for record in accepted),
            all_boundary_density_residual_max=max((record['metrics']['pressure_projection_status']['density']['final_all_boundary_compression_residual']
                for record in accepted),default=None),
            all_boundary_divergence_residual_per_s_max=max((record['metrics']['pressure_projection_status']['divergence']['final_all_boundary_compression_residual']
                for record in accepted),default=None))
        boundary=sum(record['metrics'].get('collision_verification',{}).get('boundary_dominant_count',0) for record in accepted)
        fluid=sum(record['metrics'].get('collision_verification',{}).get('fluid_dominant_count',0) for record in accepted)
        result['collision_substeps'].update(boundary_dominant_particle_substeps=boundary,
            fluid_dominant_particle_substeps=fluid,
            boundary_dominant_fraction=boundary/(boundary+fluid) if boundary+fluid else None,
            definition='largest absolute receiver-normal density/divergence pressure component per receiver-contact particle and accepted substep; differs from the earlier first-separation cohort statistic 45.3%')
    result['stationary_wall_density']=[dict(time_s=row['action_seconds'],statistics=row['wall_density'])
        for row in report['rows'] if 0.5<=row['action_seconds']<=1.5 and 'wall_density' in row]
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--baseline',type=Path,help='Completed baseline; optional for standalone B on another server')
    parser.add_argument('--input',type=Path,help='Prepared original scene input for standalone B without a baseline cache')
    parser.add_argument('--gpu',type=int,default=2)
    parser.add_argument('--case',choices=('A','B','both'),default='both')
    parser.add_argument('--existing-a',type=Path,help='Completed original-input A simulation to include without rerunning it')
    parser.add_argument('--allow-shared-gpu',action='store_true',help='Render on the selected GPU without waiting for unrelated jobs to finish')
    args=parser.parse_args()
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    baseline=args.baseline.resolve() if args.baseline else None
    if baseline is None:
        if args.case!='B' or args.input is None:
            parser.error('Without --baseline, specify --case B and --input for an original-input run')
        # Exactly the already agreed B configuration. No local report/cache is
        # required on the rental server, and no numerical defaults are inferred.
        reference=dict(simulation_hz=1200,akinci_coefficient=1.0,
            time_integration=dict(minimum_dt_divisor=64),
            pressure_solver=dict(minimum_density_iterations=2,minimum_divergence_iterations=1))
        source=args.input.resolve()
    else:
        reference=json.loads((baseline/'probe_report.json').read_text())
        if reference['status']!='completed' or reference.get('initial_frame'):
            raise ValueError('Expected a completed original-input baseline')
        source=Path(reference['source_assets'])
        if not source.is_absolute():source=ROOT/source
        if args.input is not None and args.input.resolve()!=source.resolve():
            parser.error('--input must match the provided baseline input')
    window=(3.5,4.166666666666667)
    state=dict(status='running',gpu=args.gpu,window_seconds=list(window),cases={},
        baseline=str(baseline) if baseline else None,source_input=str(source),
        source_input_sha256=hashlib.sha256((source/'input.npz').read_bytes()).hexdigest(),
        note='A changes the deficiency pressure update only; B additionally enforces all-boundary residuals, up to four convergence halvings, and 1.5 step recovery. No physics pass/fail threshold.',
        limitations='Rebound ratios alone do not distinguish physical redirection from numerical injection. Energy excludes surface potential; legacy prescribed work has a delayed divergence contribution.')
    existing_a=args.existing_a.resolve() if args.existing_a else None
    if existing_a is not None:
        previous=json.loads((existing_a/'probe_report.json').read_text())
        if previous['status']!='completed' or previous.get('initial_frame'):
            raise ValueError('Reused A must be a completed original-input simulation')
        state['reused_A']=str(existing_a)
    state['allow_shared_gpu']=args.allow_shared_gpu
    code_files=('coupled_scene/gpu_dfsph/backend.py','coupled_scene/gpu_dfsph/pressure_projection.py',
        'experiments/coupled_scenes/run_gpu_newton_water.py','experiments/coupled_scenes/run_pressure_projection_ab.py')
    state['code_sha256']={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in code_files}
    write_json(out/'status.json',state)
    env=dict(os.environ,CUDA_VISIBLE_DEVICES=str(args.gpu),PYTHONUNBUFFERED='1')

    def run(command,log,case,phase):
        state['active_case']=case;state['phase']=phase
        state['cases'][case]['phase']=phase
        write_json(out/'status.json',state)
        with log.open('x') as stream:
            process=subprocess.Popen(command,cwd=ROOT,env=env,stdout=stream,stderr=subprocess.STDOUT)
            state['child_pid']=process.pid;write_json(out/'status.json',state)
            while process.poll() is None:
                time.sleep(10)
                progress_path=out/case/('simulation/probe_report.json' if phase=='simulating' else 'appearance/video_status.json')
                if progress_path.exists():
                    progress=json.loads(progress_path.read_text())
                    state['progress']={key:progress.get(key) for key in (
                        'status','phase','last_action_seconds','recorded_frames','wall_seconds','rebuilt_frames','rendered_frames','error') if key in progress}
                    write_json(out/'status.json',state)
            return process.returncode

    cases=(('A_lock_zero',0,'moving'),('B_lock_zero_all_boundary_retry',4,'all'))
    if args.case!='both':cases=tuple(case for case in cases if case[0].startswith(args.case+'_'))
    for case,retries,scope in cases:
        directory=out/case;directory.mkdir(exist_ok=False)
        state['cases'][case]={}
        if any(hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=digest
                for name,digest in state['code_sha256'].items()):
            state['cases'][case].update(phase='simulation_failed',error='Source code changed during A/B queue')
            write_json(out/'status.json',state)
            continue
        simulation=directory/'simulation'
        command=[sys.executable,'-u',str(ROOT/'experiments/coupled_scenes/run_gpu_newton_water.py'),
            '--input',str(source),'--output',str(simulation),'--seconds',str(window[1]),
            '--hz',str(reference['simulation_hz']),'--upstream',str(ROOT/'vendor/SPH_Project'),
            '--akinci-coefficient',str(reference['akinci_coefficient']),
            '--minimum-dt-divisor',str(reference['time_integration']['minimum_dt_divisor']),
            '--minimum-density-iterations',str(reference['pressure_solver']['minimum_density_iterations']),
            '--minimum-divergence-iterations',str(reference['pressure_solver']['minimum_divergence_iterations']),
            '--diagnose-minimum-dt-failure',
            '--boundary-model','volume_maps_bender2019','--pressure-convergence-retries',str(retries),
            '--local-residual-boundary-scope',scope,'--solver-verification-window',str(window[0]),str(window[1])]
        code=run(command,directory/'simulation.console.log',case,'simulating')
        state['cases'][case]['simulation_returncode']=code
        report=json.loads((simulation/'probe_report.json').read_text()) if (simulation/'probe_report.json').exists() else {}
        if code or report.get('status')!='completed':
            state['cases'][case].update(phase='simulation_failed',error=report.get('error'))
            write_json(out/'status.json',state)
            continue
        state['cases'][case]['unconverged_accepted_substeps']=report['time_integration']['unconverged_accepted_substeps']
        render=[sys.executable,'-u',str(ROOT/'experiments/coupled_scenes/run_active_pour_video.py'),
            '--assets',str(ROOT/'output/coupled_scenes/active_pour_water_minus10_assets_v10'),
            '--design',str(ROOT/'output/coupled_scenes/active_pour_lowered_design_v7'),
            '--simulation',str(simulation),'--output',str(directory/'appearance'),
            '--desktop',str(directory/'delivery'),'--unaccepted-backend-preview',
            '--video-name',case+'.mp4']
        if args.allow_shared_gpu:render.append('--allow-shared-gpu')
        code=run(render,directory/'render.console.log',case,'rendering')
        state['cases'][case].update(render_returncode=code,phase='complete' if code==0 else 'render_failed')
        write_json(out/'status.json',state)

    comparisons={}
    comparison_paths=([('baseline',baseline)] if baseline else [])+[(case,out/case/'simulation') for case in state['cases']]
    if existing_a is not None and 'A_lock_zero' not in state['cases']:
        comparison_paths.append(('A_lock_zero',existing_a))
    for name,path in comparison_paths:
        if (path/'probe_report.json').exists():
            try:comparisons[name]=summarize(path,window)
            except Exception as exc:comparisons[name]=dict(error=f'{type(exc).__name__}: {exc}')
    write_json(out/'comparison.json',dict(window_seconds=list(window),runs=comparisons,limitations=state['limitations']))
    with (out/'comparison_frames.csv').open('x',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=['run','time_s','deficient_particle_count',
            'kinetic_energy_j','gravitational_potential_energy_j','mechanical_energy_j',
            'prescribed_motor_work_to_fluid_j','mechanical_minus_reported_motor_work_j'])
        writer.writeheader()
        for name,result in comparisons.items():
            for row in result.get('sampled_frames',[]):
                writer.writerow(dict(run=name,**{key:row[key] for key in writer.fieldnames if key!='run'}))
    state.update(status=('finished' if all(case['phase']=='complete' for case in state['cases'].values())
        else 'finished_with_failures'),phase='finished',child_pid=None)
    write_json(out/'status.json',state)


if __name__=='__main__':
    main()

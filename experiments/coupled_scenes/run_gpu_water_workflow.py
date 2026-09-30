"""Serial plumbing run; record failures and continue independent scene work.

No physical acceptance rules, retries, automatic fixes, or parameter searches.
"""
import argparse
import json
import math
import shutil
import subprocess
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from coupled_scene.gpu_dfsph.assets import prepare
from coupled_scene.gpu_dfsph.server_paths import resolve_hdri
from run_newton_surface_video import local_path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--phase',choices=['prepare','simulate','render','run'],required=True)
    parser.add_argument('--cases',nargs='+')
    parser.add_argument('--input-root',type=Path,help='Reuse previously prepared scene inputs without overwriting them')
    parser.add_argument('--full-duration',action='store_true',help='Run each scene for its authored duration from the subset registry')
    parser.add_argument('--upstream',type=Path,default=ROOT/'vendor/SPH_Project')
    parser.add_argument('--akinci-coefficient',type=float,default=0.,
        help='Akinci 2013 numerical fluid-fluid surface-force coefficient passed unchanged to every simulation')
    parser.add_argument('--minimum-density-iterations',type=int,default=2)
    parser.add_argument('--minimum-divergence-iterations',type=int,default=1)
    parser.add_argument('--minimum-dt-divisor',type=int,default=64)
    parser.add_argument('--stirring-speed-rad-s',type=float,default=10.,
        help='Explicit stirring target speed; ignored by non-stirring cases')
    parser.add_argument('--splashsurf',type=Path,default=Path(sys.executable).parent/'pysplashsurf')
    parser.add_argument('--blender',default=shutil.which('blender') or 'blender')
    parser.add_argument('--hdri',type=Path,default=resolve_hdri(ROOT))
    args=parser.parse_args()
    if not math.isfinite(args.akinci_coefficient) or args.akinci_coefficient<0:
        parser.error('Akinci coefficient must be finite and nonnegative')
    if not 1<=args.minimum_density_iterations<=300 or not 1<=args.minimum_divergence_iterations<=300:
        parser.error('Minimum DFSPH iteration counts must be between 1 and 300')
    if args.minimum_dt_divisor<1:
        parser.error('Minimum time-step divisor must be a positive integer')
    if not math.isfinite(args.stirring_speed_rad_s) or args.stirring_speed_rad_s<=0:
        parser.error('Stirring speed must be finite and positive')
    cases=json.loads((ROOT/'configs/independent_water_subset.json').read_text(encoding='utf-8'))['cases']
    cases=[c for c in cases if not args.cases or c['id'] in args.cases]
    args.output.mkdir(parents=True,exist_ok=True)
    status=dict(phase=args.phase,scope='independent_water_subset',backend='gpu_dfsph_newton',
        full_duration=args.full_duration,input_root=str(args.input_root or args.output),
        surface_tension_model=('akinci2013_paper_corrected' if args.akinci_coefficient else 'disabled'),
        akinci_coefficient=args.akinci_coefficient,stirring_speed_rad_s=args.stirring_speed_rad_s,cases=[])
    status['pressure_solver']=dict(minimum_density_iterations=args.minimum_density_iterations,
        minimum_divergence_iterations=args.minimum_divergence_iterations,
        maximum_density_iterations=300,maximum_divergence_iterations=300,
        minimum_dt_divisor=args.minimum_dt_divisor)
    def save():
        target=args.output/(args.phase+'_status.json');temp=target.with_suffix('.tmp')
        temp.write_text(json.dumps(status,indent=2,ensure_ascii=False),encoding='utf-8');temp.replace(target)
    for case in cases:
        folder=args.output/case['id'];folder.mkdir(exist_ok=True)
        row=dict(case_id=case['id'],status='running');status['cases'].append(row);save()
        inputs=(args.input_root or args.output)/case['id']/'inputs'
        print(args.phase+' '+case['id'],flush=True)
        try:
            if args.phase=='prepare':
                with (folder/'prepare.log').open('w') as log:
                    import contextlib
                    with contextlib.redirect_stdout(log):info=prepare(ROOT,case,folder/'inputs')
                row['fluid_particles']=info['fluid_particles'];row['boundary_particles']=info['boundary_particles']
            else:
                row['stages']=[]
                for phase in (['simulate','render'] if args.phase=='run' else [args.phase]):
                    row['phase']=phase
                    if phase=='simulate':
                        seconds=case['seconds'] if args.full_duration else 1/30
                        row['requested_seconds']=seconds
                        command=[sys.executable,'-u',str(ROOT/'experiments/coupled_scenes/run_gpu_newton_water.py'),
                            '--input',str(inputs),'--output',str(folder/'simulation'),'--seconds',str(seconds),
                            '--upstream',str(args.upstream),'--akinci-coefficient',str(args.akinci_coefficient),
                            '--minimum-density-iterations',str(args.minimum_density_iterations),
                            '--minimum-divergence-iterations',str(args.minimum_divergence_iterations),
                            '--minimum-dt-divisor',str(args.minimum_dt_divisor),
                            '--stirring-speed-rad-s',str(args.stirring_speed_rad_s)]
                    elif case['family']=='surface_study':
                        command=[sys.executable,str(ROOT/'experiments/coupled_scenes/run_newton_surface_video.py'),
                            '--simulation',str(folder/'simulation'),'--output',str(folder/'appearance'),
                            '--splashsurf',str(args.splashsurf),'--blender',str(args.blender),'--hdri',str(args.hdri)]
                    else:
                        meta=json.loads((inputs/'input.json').read_text(encoding='utf-8'))['source_assets']
                        command=[sys.executable,str(ROOT/'experiments/coupled_scenes/run_active_pour_video.py'),
                            '--assets',str(ROOT/case['source']),'--design',str(local_path(meta['source_blend']).parent),
                            '--simulation',str(folder/'simulation'),'--output',str(folder/'appearance'),
                            '--desktop',str(folder/'delivery'),'--video-name','workflow_preview.mp4','--unaccepted-backend-preview']
                        command.extend(['--splashsurf',str(args.splashsurf),'--blender',str(args.blender),'--hdri',str(args.hdri)])
                    stage=dict(phase=phase,status='running',command=command);row['stages'].append(stage);save()
                    log_name='simulation.log' if phase=='simulate' else phase+'.log'
                    with (folder/log_name).open('x') as log:
                        subprocess.run(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,check=True)
                    stage['status']='completed';save()
            row['status']='completed'
        except Exception as exc:
            row.update(status='failed',error=f'{type(exc).__name__}: {exc}',automatic_correction=False)
            if row.get('stages') and row['stages'][-1]['status']=='running':row['stages'][-1]['status']='failed'
        save()
    status['finished']=True;save()


if __name__=='__main__':main()

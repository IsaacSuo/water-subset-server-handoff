"""Serial plumbing run; record failures and continue independent scene work.

No physical acceptance rules, retries, automatic fixes, or parameter searches.
"""
import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from coupled_scene.gpu_dfsph.assets import prepare
from run_newton_surface_video import local_path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--phase',choices=['prepare','simulate','render','run'],required=True)
    parser.add_argument('--cases',nargs='+')
    parser.add_argument('--input-root',type=Path,help='Reuse previously prepared scene inputs without overwriting them')
    parser.add_argument('--full-duration',action='store_true',help='Run each scene for its authored duration from the subset registry')
    args=parser.parse_args()
    cases=json.loads((ROOT/'configs/independent_water_subset.json').read_text(encoding='utf-8'))['cases']
    cases=[c for c in cases if not args.cases or c['id'] in args.cases]
    args.output.mkdir(parents=True,exist_ok=True)
    status=dict(phase=args.phase,scope='independent_water_subset',backend='gpu_dfsph_newton',
        full_duration=args.full_duration,input_root=str(args.input_root or args.output),cases=[])
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
                            '--input',str(inputs),'--output',str(folder/'simulation'),'--seconds',str(seconds)]
                    elif case['family']=='surface_study':
                        command=[sys.executable,str(ROOT/'experiments/coupled_scenes/run_newton_surface_video.py'),
                            '--simulation',str(folder/'simulation'),'--output',str(folder/'appearance')]
                    else:
                        meta=json.loads((inputs/'input.json').read_text(encoding='utf-8'))['source_assets']
                        command=[sys.executable,str(ROOT/'experiments/coupled_scenes/run_active_pour_video.py'),
                            '--assets',str(ROOT/case['source']),'--design',str(local_path(meta['source_blend']).parent),
                            '--simulation',str(folder/'simulation'),'--output',str(folder/'appearance'),
                            '--desktop',str(folder/'delivery'),'--video-name','workflow_preview.mp4','--unaccepted-backend-preview']
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

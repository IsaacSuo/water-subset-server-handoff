"""Run small phenomenon comparisons through existing rigid/material generators.

No rendering, training or outcome-based filtering. A failed job does not block
unrelated jobs; resume only skips episodes with verified completion manifests.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

from .io import file_hash, read_json, write_json
from .causal_runner import ROOT, prepare, invoke, package_physics
from .local import idle


def run(plan_path, output, ids=None):
    plan_path=Path(plan_path).resolve();plan=read_json(plan_path)
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=True)
    record=output/'batch.json'
    if record.exists():
        report=read_json(record)
        if report['plan_sha256']!=file_hash(plan_path):raise ValueError('Plan changed; use new batch output')
    else:
        report=dict(plan=str(plan_path),plan_sha256=file_hash(plan_path),jobs={})
    for job in plan['jobs']:
        if ids and job['id'] not in ids:continue
        previous=report['jobs'].get(job['id'])
        if previous and previous['status']=='generated':
            ep=Path(previous['episode']);manifest=ep/previous['manifest']
            if file_hash(manifest)!=previous['manifest_sha256']:raise ValueError('Generated episode changed')
            continue
        run_dir=output/job['id'];start=time.monotonic()
        item=dict(job,status='running');report['jobs'][job['id']]=item
        record.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        try:
            if run_dir.exists():raise FileExistsError('Partial run preserved; choose a new job/output for retry: '+str(run_dir))
            config=ROOT/job['config']
            if job['backend']=='rigid':
                command=[sys.executable,'-m','world_model_dataset.real_scene_batch','--spec',str(config),'--output',str(run_dir)]
                ep=run_dir/'episodes'/job['id']
            elif job['backend']=='material':
                command=[sys.executable,plan['material_entry'],'--config',str(config),'--runtime',str(ROOT/plan['material_runtime']),
                         '--mainline',str(ROOT),'--output',str(run_dir)]
                # Our sequential runner checks free VRAM; desktop allocation alone
                # is not an active simulation and must not prevent local generation.
                idle();command.append('--allow-busy-gpu');ep=run_dir/'episode'
            elif job['backend']=='native':
                prepare(config,run_dir);idle();invoke('native_causal_rigid.py',run_dir,'simulation.log');package_physics(run_dir)
                command=None;ep=run_dir
            else:raise ValueError('Unsupported generator: '+job['backend'])
            if command:
                with (output/(job['id']+'.log')).open('x') as log:
                    subprocess.run(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,check=True)
            from .causal_loader import open_episode
            episode=open_episode(ep,require_complete=False)
            count=sum(1 for _ in episode.states())
            manifest=next(ep/n for n in ('episode.json','episode.physics.json') if (ep/n).exists())
            item.update(status='generated',episode=str(ep),manifest=manifest.name,manifest_sha256=file_hash(manifest),states=count,
                        quality='unreviewed',elapsed_s=round(time.monotonic()-start,2))
            print('GENERATED',job['id'],count,flush=True)
        except Exception as exc:
            item.update(status='failed',error=repr(exc),elapsed_s=round(time.monotonic()-start,2))
            print('FAILED',job['id'],repr(exc),flush=True)
        record.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--ids',nargs='+');args=parser.parse_args()
    report=run(args.plan,args.output,args.ids)
    if any(j['status']=='failed' for j in report['jobs'].values()):raise SystemExit(1)


if __name__=='__main__':main()

"""WSL serial local execution; explicit argv, logs, GPU check, no remote writes."""
import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

from .contract import ROOT,prepare,validate_episode
from .audit import audit
from .io import read_json,write_json


def windows_path(path):
    return subprocess.check_output(['wslpath','-w',str(Path(path).resolve())],text=True).strip()


def invoke(script,output,logname,extra=()):
    values=[r'Y:\isaacsim\python.bat',windows_path(ROOT/'world_model_dataset'/script),'--episode',windows_path(output),*extra]
    command='& '+' '.join("'"+v.replace("'","''")+"'" for v in values)+'; exit $LASTEXITCODE'
    with (output/logname).open('x',encoding='utf-8') as stream:
        proc=subprocess.run(['powershell.exe','-NoProfile','-Command',command],cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT)
    if proc.returncode:raise RuntimeError(f'Native exit {proc.returncode}; see {logname}')
    # SimulationApp fast shutdown may turn an exception into exit 0.
    if (output/'native_failure.json').exists() or (output/'observations/failure.json').exists():
        raise RuntimeError('Native backend wrote failure report despite process exit 0')


def idle(timeout_s=60):
    """Capacity guard for small jobs; other GPU processes are allowed.

    Utilization and nonzero allocation do not imply contention.  Wait only when
    free VRAM is below the declared reserve needed to start another local Isaac
    job.  Heavy jobs can raise the reserve with DATASET_MIN_FREE_GPU_MIB.
    """
    minimum_free=int(os.environ.get('DATASET_MIN_FREE_GPU_MIB','4096'))
    deadline=time.monotonic()+timeout_s
    while True:
        lines=subprocess.check_output(['nvidia-smi','--query-gpu=memory.free','--format=csv,noheader,nounits'],text=True).splitlines()
        if any(int(free)>=minimum_free for free in lines):
            return
        if time.monotonic()>=deadline:
            raise RuntimeError(f'No GPU has the requested {minimum_free} MiB free; refusing likely OOM')
        time.sleep(1)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('spec');parser.add_argument('output',type=Path);parser.add_argument('--render',action='store_true')
    args=parser.parse_args();idle();prepare(args.spec,args.output)
    validate_episode(args.output/'episode.prepared.json',check_source=True)
    prepared=read_json(args.output/'episode.prepared.json')
    backends={'R03':'native_rigid_multi.py','V05':'native_mixed.py'}
    backend=backends.get(prepared['spec']['event_id'],'native.py')
    invoke(backend,args.output,'simulation.log')
    if not (args.output/'native_report.json').exists():raise RuntimeError('No completion report')
    result=audit(args.output)
    print('PHYSICS_AUDIT',result['passed'],flush=True)
    if args.render and result['passed']:
        invoke('rendering.py',args.output,'render.log')
    write_json(args.output/'workflow_report.json',dict(physics_passed=result['passed'],
        observations_complete=(args.output/'observations/index.json').exists(),m3_accepted=False,
        note='Full M3 capability and cross-episode checks are still required'))
    # Physics-only development runs intentionally stop here.  Rendering and
    # dataset finalization can be added later without rerunning the simulation.
    if args.render:
        from .finalize import finalize
        finalize(args.output)


if __name__=='__main__':main()

"""WSL serial local execution; explicit argv, logs, GPU check, no remote writes."""
import argparse
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
    """Wait through the previous Isaac process's short GPU teardown tail."""
    deadline=time.monotonic()+timeout_s
    while True:
        lines=subprocess.check_output(['nvidia-smi','--query-gpu=memory.used,utilization.gpu','--format=csv,noheader,nounits'],text=True).splitlines()
        if not any(int(m)>1024 or int(u)>15 for m,u in (line.split(',') for line in lines)):
            return
        if time.monotonic()>=deadline:
            raise RuntimeError('GPU remained busy; refusing to contend with another application')
        time.sleep(1)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('spec');parser.add_argument('output',type=Path);parser.add_argument('--render',action='store_true')
    args=parser.parse_args();idle();prepare(args.spec,args.output)
    validate_episode(args.output/'episode.prepared.json',check_source=True)
    prepared=read_json(args.output/'episode.prepared.json')
    backend='native_rigid_multi.py' if prepared['spec']['event_id']=='R03' else 'native.py'
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

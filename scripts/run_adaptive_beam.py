"""Object/region request -> unloaded probe -> bound adaptive load -> unified episode.

This command explicitly runs physics. It never edits a generated configuration,
changes solver code, registers a catalog, or overwrites a failed attempt.
"""
import argparse,subprocess,sys
from pathlib import Path
from world_model_dataset.io import read_json,write_json,file_hash
from world_model_dataset.phenomenon_experiment import run


def stages(folder,names):
    for stage in names:
        ledger=run(folder/'generated/experiment.json',folder/'execution',stage=stage)
        if any(j['status']=='failed' for j in ledger['jobs'].values()):raise RuntimeError('Failed attempt retained; do not overwrite '+str(folder))
    return Path(ledger['jobs']['baseline']['stages']['package']['result']['episode'])


def construct(request,folder,calibration=None):
    command=[sys.executable,'-B','-m','world_model_dataset.experiment_construct','--request',str(request),'--output',str(folder/'generated')]
    if calibration:command+=['--calibration-episode',str(calibration)]
    subprocess.run(command,check=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--request',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--calibration-episode',type=Path,help='Optional compatible completed probe; exact compatibility is checked by constructor')
    a=p.parse_args();r=read_json(a.request);profile=read_json(a.request.parent/r['profile'])
    if r['phenomenon']!='beam_load_hold_withdraw' or 'calibration' not in profile:raise ValueError('Adaptive beam profile required')
    a.output.mkdir(parents=True,exist_ok=False)
    probe=a.calibration_episode
    if probe is None:
        construct(a.request,a.output/'probe');probe=stages(a.output/'probe',('prepare','simulate','package','audit'))
    construct(a.request,a.output/'load',probe)
    episode=stages(a.output/'load',('prepare','simulate','package','audit','observe'))
    write_json(a.output/'delivery.json',dict(request=str(a.request.resolve()),request_sha256=file_hash(a.request),
        calibration_episode=str(probe.resolve()),episode=str(episode.resolve()),training_admission=False))

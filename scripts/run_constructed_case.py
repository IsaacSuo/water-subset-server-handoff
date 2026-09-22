"""Run already constructed inputs through explicit stages, never register a catalog."""
import argparse
from pathlib import Path
from world_model_dataset.phenomenon_experiment import run

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--folder',type=Path,required=True)
    p.add_argument('--stages',nargs='+',default=['prepare','simulate','package','audit','observe'])
    a=p.parse_args()
    for stage in a.stages:
        if stage not in ('prepare','simulate','package','audit','observe'):raise ValueError('No catalog stage allowed')
        ledger=run(a.folder/'generated/experiment.json',a.folder/'execution',stage=stage)
        if any(j['status']=='failed' for j in ledger['jobs'].values()):raise SystemExit('Failed; artifacts preserved')

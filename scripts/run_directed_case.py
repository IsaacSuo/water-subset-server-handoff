"""Explicit, bounded one-case execution; no automatic retry or registration."""
import argparse
from pathlib import Path
from world_model_dataset.phenomenon_experiment import run
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--folder',type=Path,required=True);a=p.parse_args()
 for stage in ('prepare','simulate','package','audit','observe'):
  ledger=run(a.folder/'generated/experiment.json',a.folder/'execution',stage=stage)
  if any(j.get('status')=='failed' for j in ledger['jobs'].values()):raise RuntimeError('Attempt failed and retained; no retry: '+stage)

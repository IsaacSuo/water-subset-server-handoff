"""WSL launcher for selected Blender Cycles renders; never runs physics."""
from __future__ import annotations

import argparse
import os
import subprocess
from pathlib import Path

from .contract import ROOT,validate_episode


def win(path):return subprocess.check_output(['wslpath','-w',str(Path(path).resolve())],text=True).strip()


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('episode',type=Path);parser.add_argument('output',type=Path)
    parser.add_argument('--frames',default='auto');parser.add_argument('--samples',type=int,default=32);args=parser.parse_args()
    validate_episode(args.episode/'episode.json',require_complete=True)
    blender=Path(os.environ.get('BLENDER_BIN','/mnt/d/Program Files (x86)/Blender/blender.exe'))
    if not blender.is_file():raise FileNotFoundError(blender)
    values=[str(blender),'--background','--python',win(ROOT/'world_model_dataset/cycles_render.py'),'--','--episode',win(args.episode),
            '--output',win(args.output),'--frames',args.frames,'--samples',str(args.samples)]
    result=subprocess.run(values,cwd=ROOT)
    if result.returncode:raise SystemExit(result.returncode)


if __name__=='__main__':main()

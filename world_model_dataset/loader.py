"""Unified, pickle-free loader for rigid and volume-deformable episodes."""
from pathlib import Path

import numpy as np

from .contract import validate_episode,validate_schema,artifact
from .io import read_json,inside


class Episode:
    def __init__(self,path,require_complete=True):
        self.root=Path(path)
        manifest=self.root/'episode.json'
        if not manifest.exists():manifest=self.root/'episode.prepared.json'
        self.manifest=validate_episode(manifest,require_complete=require_complete)
        self.index=read_json(self.root/'state/index.json')

    def capability(self,name,allowed=('native','derived')):
        value=self.manifest['capabilities'].get(name)
        if value is None or value['status'] not in allowed:
            status='undeclared' if value is None else value['status']
            reason='' if value is None else value.get('reason') or ''
            raise RuntimeError(f'Capability {name} is {status}: {reason}')
        return value

    def contacts(self):
        rigid=self.manifest['capabilities'].get('rigid_contact_impulse',{})
        if rigid.get('status')=='native':
            self.capability('rigid_contact_impulse',('native',))
        else:
            # Select the capability that matches the episode representation so
            # callers receive the real exclusion reason instead of an empty set.
            self.capability('soft_contact_impulse',('native',))
        import json
        with (self.root/'contacts.jsonl').open() as stream:
            for line in stream:
                row=json.loads(line);validate_schema(row,'contact');yield row

    def states(self):
        for frame in self.index['frames']:
            state=read_json(inside(self.root,frame['state']));validate_schema(state,'state')
            for body in state['objects'].values():
                if artifact(self.root,body['geometry']['path'])!=body['geometry']:raise ValueError('Geometry hash mismatch')
            yield state

    def geometry(self,index):
        frame=self.index['frames'][index]
        with np.load(inside(self.root,frame['geometry']),allow_pickle=False) as data:
            return {key:data[key].copy() for key in data.files}

    def observations(self):
        self.capability('rgb_depth_segmentation',('native',))
        index=read_json(self.root/'observations/index.json')
        for row in index['frames']:
            with np.load(inside(self.root/'observations',row['data']),allow_pickle=False) as data:
                yield row,{key:data[key].copy() for key in data.files}

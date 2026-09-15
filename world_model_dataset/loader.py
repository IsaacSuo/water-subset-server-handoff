"""Unified, pickle-free loader for rigid and volume-deformable episodes."""
from pathlib import Path

import numpy as np

from .contract import validate_episode,validate_schema,artifact
from .io import read_json,inside


def _legacy_manual_physics_acceptance(root,manifest,validation):
    """Recognize the narrow, documented R01 pre-policy compatibility case."""
    root=Path(root);matrix=root.parent.parent
    review_path=matrix/'manual_physics_review.json';definition_path=matrix/'matrix.json'
    failures={item['name'] for item in validation.get('checks',[]) if item['status']=='fail'}
    if failures!={'surface_fixture_intersection_audit'} or not review_path.is_file() or not definition_path.is_file():
        return False
    review=read_json(review_path);event_id=manifest['spec']['event_id'];section=review.get(event_id,{})
    definition=read_json(definition_path)
    return (root.parent.name=='episodes' and section.get('decision')=='accepted' and
            manifest['episode_id'] in definition.get('episodes',[]))


class Episode:
    def __init__(self,path,require_complete=True):
        self.root=Path(path)
        self.require_complete=require_complete
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
        elif not getattr(self,'require_complete',True) and (self.root/'native_report.json').is_file():
            # Physics-only development caches intentionally retain a prepared
            # lifecycle until observations exist. Permit their already-audited
            # native rigid stream without pretending the episode is complete.
            report=read_json(self.root/'native_report.json')
            validation=read_json(self.root/'physics_validation.json')
            kind=report.get('physical_representation','')
            accepted=validation['passed'] or _legacy_manual_physics_acceptance(self.root,self.manifest,validation)
            if not kind.startswith('rigid') or not accepted or not (self.root/'contacts.jsonl').is_file():
                if kind=='volumetric':
                    probe=read_json(self.root/'capability_probes/soft_contact_impulse.json')
                    raise RuntimeError(f"Capability soft_contact_impulse is {probe['status']}: {probe['reason']}")
                raise RuntimeError('Native rigid contact stream is not available in this development cache')
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

    def geometry(self,index,instance_id=None):
        frame=self.index['frames'][index]
        if 'geometries' in frame:
            if instance_id is None:
                if len(frame['geometries'])!=1:
                    raise ValueError('instance_id is required for a multi-object frame')
                instance_id=next(iter(frame['geometries']))
            if instance_id not in frame['geometries']:
                raise KeyError(f'No geometry for instance {instance_id}')
            path=frame['geometries'][instance_id]
        else:
            if instance_id is not None:
                state=read_json(inside(self.root,frame['state']))
                if instance_id not in state['objects']:raise KeyError(f'No geometry for instance {instance_id}')
            path=frame['geometry']
        with np.load(inside(self.root,path),allow_pickle=False) as data:
            return {key:data[key].copy() for key in data.files}

    def observations(self):
        self.capability('rgb_depth_segmentation',('native',))
        index=read_json(self.root/'observations/index.json')
        for row in index['frames']:
            with np.load(inside(self.root/'observations',row['data']),allow_pickle=False) as data:
                yield row,{key:data[key].copy() for key in data.files}

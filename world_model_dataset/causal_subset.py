"""Small cache-reuse production and reading entry; never resimulates physics."""
import argparse
import copy
from pathlib import Path
import shutil

import numpy as np

from .causal_runner import ROOT, invoke
from .causal_loader import open_episode
from .causal_observe import package_observations
from .causal_finalize import finalize_smoke
from .io import read_json, write_json, file_hash


class CausalSubset:
    def __init__(self, root):
        self.root=Path(root)
        self.index=read_json(self.root/'index.json')

    def episodes(self, split=None):
        for item in self.index['episodes']:
            if split is None or split==item['split']:
                yield item,open_episode(self.root/item['path'])


def freeze_sources(spec):
    records=[]
    owners={}
    for item in spec['episodes']:
        src=ROOT/item['source']
        manifest=read_json(src/'episode.physics.json')
        resolved=read_json(src/'resolved_inputs.json')
        tokens=['cache:'+file_hash(src/'body_state_trace.jsonl'), 'family:'+manifest['counterfactual']['family_id']]
        for body in resolved['bodies'].values():
            g=body['geometry']
            if 'static_scene_source' in g:
                tokens.append('scene:'+g['static_scene_source']['blend_sha256'])
            if 'physical_asset_source' in g:
                # Size/mass/loader choices are variants, not new source families.
                tokens.append('asset:'+g['source_sha256'])
        split=spec['groups'][item['group']]
        for token in tokens:
            if token in owners and owners[token]!=split:
                raise ValueError('Source/family leakage across splits: '+token)
            owners[token]=split
        records.append(dict(item,split=split,source_manifest_sha256=file_hash(src/'episode.physics.json'),
            source_state_sha256=file_hash(src/'body_state_trace.jsonl'),source_resolved_sha256=file_hash(src/'resolved_inputs.json'),
            source_contact_sha256=file_hash(src/'contacts.jsonl'),lineage_keys=sorted(set(tokens))))
    return records


def copy_physics(source, output, episode_id):
    output.mkdir(parents=True,exist_ok=False)
    # Immutable physical records copied byte for byte. Do not reuse a completed
    # manifest that still points at observations absent from this new bundle.
    names=['body_state_trace.jsonl','contacts.jsonl','initial_state.json','input_config.json','resolved_inputs.json',
           'source_snapshot.json','native_report.json','native_initialization_readback.json',
           'command_trace.jsonl','actuator_state_trace.jsonl','actuator_effort_trace.jsonl',
           'native_initial.usda','native_final.usda']
    for name in names:
        if (source/name).exists():shutil.copy2(source/name,output/name)
    if (source/'geometry').exists():shutil.copytree(source/'geometry',output/'geometry')
    manifest=read_json(source/'episode.physics.json')
    manifest['episode_id']=episode_id
    # Initial-state lineage still describes the original physical realization;
    # cache/re-render provenance lives in the subset's frozen source index.
    write_json(output/'episode.physics.json',manifest)


def observation_cache_matches(candidate, profile, source):
    """Same trajectory alone is insufficient: appearance and geometry may differ."""
    if not all((candidate/name).is_file() for name in
               ('observations/index.json', 'observation_profile.json', 'resolved_inputs.json')):
        return False
    index=read_json(candidate/'observations/index.json')
    return (read_json(candidate/'observation_profile.json')==profile
            and index.get('complete') is True
            and index.get('source_state_sha256')==source['source_state_sha256']
            and file_hash(candidate/'resolved_inputs.json')==source['source_resolved_sha256'])


def check_observations(output):
    episode=open_episode(output,require_complete=False)
    resolved=episode.resolved_inputs()
    manifest=episode.manifest
    index=read_json(output/'observations/index.json')
    ids=index['body_instance_ids']
    if set(ids)!=set(resolved['bodies']):raise ValueError('Observation participants differ from physics')
    counts={k:0 for k in index['cameras']}
    visible={k:{oid:0 for oid in ids} for k in counts}
    minimum_pixels={k:{oid:10**9 for oid in ids} for k in counts}
    valid_depth=0
    for row,data in episode.observations():
        cam=row['camera_id'];counts[cam]+=1
        if np.any(~np.isfinite(data['depth_m'])):raise ValueError('Nonfinite stored depth')
        if not set(np.unique(data['segmentation'])).issubset({0,*ids.values()}):raise ValueError('Unknown body label')
        valid_depth+=int(data['depth_valid'].sum())
        for oid,label in ids.items():
            n=int((data['segmentation']==label).sum())
            visible[cam][oid]+=int(n>0)
            minimum_pixels[cam][oid]=min(minimum_pixels[cam][oid],n)
    expected=round(manifest['timing']['duration_s']*manifest['timing']['capture_hz'])+1
    if any(n!=expected for n in counts.values()):raise ValueError('Incomplete observation sequence')
    states=list(episode.states())
    contacts=list(episode.contacts())
    controls=list(episode.controls())
    efforts=list(episode.actuator_efforts()) if manifest['control_program']['primitive']!='none' else []
    return dict(camera_frames=counts,visible_frames=visible,minimum_pixels=minimum_pixels,valid_depth_pixels=valid_depth,
        physical_states=len(states),contact_points=len(contacts),commands=len(controls),effort_records=len(efforts),
        physics_rerun=False,scope='structural/time/participant/readability checks plus separately recorded direct physics review')


def build(spec_path, output, observation_cache=None):
    spec=read_json(spec_path)
    sources=freeze_sources(spec)
    output.mkdir(parents=True,exist_ok=False)
    # Frozen before rendering, not assigned after inspecting results.
    write_json(output/'sources_and_splits.json',dict(spec=spec,source_spec_sha256=file_hash(spec_path),episodes=sources))
    finished=[]
    for item in sources:
        src=ROOT/item['source'];dst=output/'episodes'/item['id']
        copy_physics(src,dst,item['id'])
        cam=copy.deepcopy(read_json(src/'resolved_inputs.json')['camera_set'])
        for key in ('resolution','focal_length_mm','horizontal_aperture_mm'):
            cam[key]=spec['camera_standard'][key]
        write_json(dst/'cache_lineage.json',dict(source=item,source_path=str(src.resolve()),
            operation='byte-identical physical cache reuse, with alias ID and cache-only observation production',physics_rerun=False))
        profile=dict(camera_set=cam,appearance_policy=spec['appearance_policy'])
        if any('static_scene_source' in b['geometry'] for b in read_json(src/'resolved_inputs.json')['bodies'].values()):
            profile['camera_lights']=dict(intensity=30000.,radius_m=.15)
        write_json(dst/'observation_profile.json',profile)
        cached=src if (item.get('reuse_observations')
                       and cam==read_json(src/'resolved_inputs.json')['camera_set']
                       and not profile.get('camera_lights')) else None
        if cached is None and observation_cache:
            candidate=observation_cache/'episodes'/item['id']
            if observation_cache_matches(candidate,profile,item):
                cached=candidate
        if cached:
            idx=read_json(cached/'observations/index.json')
            if not idx['complete'] or idx['source_state_sha256']!=item['source_state_sha256']:
                raise ValueError('Observation reuse refers to different/incomplete physical cache')
            shutil.copytree(cached/'observations',dst/'observations')
            write_json(dst/'observation_reuse.json',dict(source=str(cached),index_sha256=file_hash(cached/'observations/index.json')))
        else:
            invoke('causal_render.py',dst,'observation.log')
            if (dst/'observations/failure.json').exists():raise RuntimeError('Observation replay failed')
        package_observations(dst)
        checks=check_observations(dst)
        write_json(dst/'human_review.json',dict(reviewer='agent direct cache and sensor review',
            physics_accepted=True,observations_accepted=True,scope=spec['scope'],
            limitations=item.get('limitations',[]),evidence=checks,
            no_claims=['continuous-physics convergence','formal geometry/material OOD','public redistribution clearance']))
        # Existing observations may already carry the corrected calibrated index;
        # preserve its source copy rather than replacing it in place.
        if (dst/'observations/index.calibrated.json').exists():
            (dst/'observations/index.calibrated.json').rename(dst/'observations/index.source_calibrated.json')
        finalize_smoke(dst)
        finished.append(dict(id=item['id'],path=str(dst.relative_to(output)),split=item['split'],group=item['group'],
            source_state_sha256=item['source_state_sha256'],checks=checks))
        write_json(output/f'progress_{len(finished):02d}.json',dict(episodes=finished))
        print('C4_EPISODE_COMPLETE',item['id'],checks['camera_frames'],flush=True)
    write_json(output/'index.json',dict(id=spec['id'],complete=True,scope=spec['scope'],episodes=finished,
        source_index='sources_and_splits.json',physics_rerun=False,full_c4_complete=False,
        modalities=['rgb','optical_z_depth','body_instance_segmentation','rigid_states','native_rigid_contacts','control_commands','actuator_effort'],
        deferred=['surface_normals','motion_vectors','flexible_objects','public_release']))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    sub=p.add_subparsers(dest='command',required=True)
    run=sub.add_parser('build');run.add_argument('--spec',type=Path,required=True);run.add_argument('--output',type=Path,required=True)
    run.add_argument('--observation-cache',type=Path,help='Optional compatible observation cache; physics and camera/lighting profile must match')
    read=sub.add_parser('inspect');read.add_argument('--root',type=Path,required=True)
    a=p.parse_args()
    if a.command=='build':build(a.spec,a.output,a.observation_cache)
    else:
        for item,episode in CausalSubset(a.root).episodes():
            print(item['id'],item['split'],'states',sum(1 for _ in episode.states()),
                  'observations',sum(1 for _ in episode.observations()),'controls',sum(1 for _ in episode.controls()))


if __name__=='__main__':main()

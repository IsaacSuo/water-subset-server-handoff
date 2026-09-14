"""Small, deterministic M4 event matrices and their serial physics runner."""
from __future__ import annotations

import copy
import subprocess
import sys
from pathlib import Path

from .contract import CONFIG,ROOT,resolve,validate_pair
from .io import read_json,write_json


def _orientation(geometry):
    return [2**-.5,0.,0.,2**-.5] if geometry=='cylinder' else [0.,0.,0.,1.]


def generate_r03(config_path=CONFIG/'prototypes/M4_R03.json'):
    cfg=read_json(config_path);rows=[];pairs=[]
    for geometry in cfg['geometries']:
        base_id=f'r03_{geometry}_reference';family=f'r03_{geometry}'
        objects=[]
        for instance,appearance in (('left','neutral_blue'),('right','neutral_orange')):
            objects.append(dict(instance_id=instance,object_id=geometry,physics_profile_id='rigid_reference',
                                appearance_profile_id=appearance,orientation_xyzw=_orientation(geometry)))
        base=dict(schema_version='0.1.0',episode_id=base_id,event_id='R03',environment_id='canonical_studio',
            camera_set_id='two_fixed',numerics_profile_id='reference',objects=objects,
            fixture_parameters={'separation_D':cfg['separation_D'],'lateral_offset_D':cfg['lateral_offset_D']},
            action_parameters={'start_time_s':.5,'left_speed_m_s':cfg['baseline_speed_m_s'],'right_speed_m_s':cfg['baseline_speed_m_s']},
            timing={'physics_hz':240,'capture_hz':60,'duration_s':2.},seed=cfg['seed'],
            counterfactual={'family_id':family,'baseline_episode_id':None,'changed_pointer':None},split='development')
        rows.append(base)
        for profile,pointer in (('rigid_bouncy','/objects/1/physics/restitution'),('rigid_heavy','/objects/1/physics/density_kg_m3')):
            variant=copy.deepcopy(base);suffix=profile.removeprefix('rigid_');variant['episode_id']=f'r03_{geometry}_right_{suffix}'
            variant['objects'][1]['physics_profile_id']=profile
            variant['counterfactual'].update(baseline_episode_id=base_id,changed_pointer=pointer)
            rows.append(variant);pairs.append((base_id,variant['episode_id']))
        if geometry=='sphere':
            variant=copy.deepcopy(base);variant['episode_id']='r03_sphere_lateral_miss'
            variant['fixture_parameters']['lateral_offset_D']=cfg['sphere_extra_variants']['miss_lateral_offset_D']
            variant['counterfactual'].update(baseline_episode_id=base_id,changed_pointer='/fixture_parameters/lateral_offset_D')
            rows.append(variant);pairs.append((base_id,variant['episode_id']))
            variant=copy.deepcopy(base);variant['episode_id']='r03_sphere_stationary_right'
            variant['action_parameters']['right_speed_m_s']=cfg['sphere_extra_variants']['stationary_right_speed_m_s']
            variant['counterfactual'].update(baseline_episode_id=base_id,changed_pointer='/action_parameters/right_speed_m_s')
            rows.append(variant);pairs.append((base_id,variant['episode_id']))
    if len(rows)!=cfg['episodes'] or len(pairs)!=cfg['pairs']:raise AssertionError('R03 matrix size mismatch')
    by_id={row['episode_id']:row for row in rows}
    for base,variant in pairs:validate_pair(by_id[base],by_id[variant])
    for row in rows:resolve(row)
    return rows,pairs


def ensure_r03_specs(root):
    root=Path(root);rows,pairs=generate_r03();definition={'schema_version':'0.1.0','event_id':'R03',
        'episodes':[row['episode_id'] for row in rows],'pairs':[list(pair) for pair in pairs]}
    if root.exists():
        if read_json(root/'matrix.json')!=definition:raise ValueError('Existing R03 matrix definition differs')
        for row in rows:
            if read_json(root/'specs'/(row['episode_id']+'.json'))!=row:raise ValueError('Existing R03 spec differs: '+row['episode_id'])
        return rows,pairs
    (root/'specs').mkdir(parents=True)
    for row in rows:write_json(root/'specs'/(row['episode_id']+'.json'),row)
    write_json(root/'matrix.json',definition);return rows,pairs


def main():
    import argparse
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('output',type=Path)
    parser.add_argument('--limit',type=int,default=0);args=parser.parse_args()
    rows,pairs=ensure_r03_specs(args.output)
    if args.limit:rows=rows[:args.limit]
    results=args.output/'results';results.mkdir(exist_ok=True);outcomes=[]
    for row in rows:
        episode=args.output/'episodes'/row['episode_id'];episode.parent.mkdir(exist_ok=True)
        result_path=results/(row['episode_id']+'.json')
        if result_path.exists():outcome=read_json(result_path)
        else:
            if not episode.exists():
                command=[sys.executable,'-m','world_model_dataset.local',str(args.output/'specs'/(row['episode_id']+'.json')),str(episode)]
                returncode=subprocess.run(command,cwd=ROOT).returncode
            else:returncode=0
            validation=read_json(episode/'physics_validation.json') if (episode/'physics_validation.json').exists() else None
            if returncode or not validation:raise RuntimeError(f'R03 execution incomplete: {row["episode_id"]}')
            outcome={'episode_id':row['episode_id'],'physics_passed':validation['passed']}
            write_json(result_path,outcome)
        outcomes.append(outcome)
        if not outcome['physics_passed']:raise RuntimeError('R03 numerical integrity failed: '+row['episode_id'])
    complete=len(rows)==len(generate_r03()[0])
    summary={'schema_version':'0.1.0','event_id':'R03','requested':len(rows),'accepted':len(outcomes),
             'complete':complete,'outcomes':outcomes}
    write_json(args.output/('summary.json' if complete else f'progress_{len(rows):03d}.json'),summary)


if __name__=='__main__':main()

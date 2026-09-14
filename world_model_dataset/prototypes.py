"""Deterministic M3 prototype matrix, pairing and replay acceptance."""
from __future__ import annotations

import copy
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

from .contract import CONFIG,ROOT,validate_pair,resolve
from .io import read_json,write_json
from .loader import Episode


def _orientation(geometry):
    # Cylinder/capsule longitudinal axis is authored Z; rotate it to world Y so
    # ramp motion tests cross-sectional rolling rather than unstable end-over-end motion.
    return [2**-.5,0.,0.,2**-.5] if geometry in ('cylinder','capsule') else [0.,0.,0.,1.]


def _common(episode,event,obj,physics,family,seed,numerics='reference'):
    return dict(schema_version='0.1.0',episode_id=episode,event_id=event,
        environment_id='canonical_studio',camera_set_id='two_fixed',numerics_profile_id=numerics,
        objects=[dict(instance_id='subject',object_id=obj,physics_profile_id=physics,
                      appearance_profile_id='neutral_blue',orientation_xyzw=_orientation(obj))],
        seed=seed,counterfactual=dict(family_id=family,baseline_episode_id=None,changed_pointer=None),split='development')


def generate(config_path=CONFIG/'prototypes/M3.json'):
    cfg=read_json(config_path);seed=cfg['seed'];rows=[];pairs=[]
    for geometry in cfg['R01']['geometries']:
        base_id=f'r01_{geometry}_angle20';family=f'r01_{geometry}_angle'
        base=_common(base_id,'R01',geometry,'rigid_reference',family,seed)
        base.update(fixture_parameters=dict(angle_deg=20.,length_D=4.),action_parameters=dict(release_time_s=.5),
                    timing=dict(physics_hz=240,capture_hz=30,duration_s=4.))
        rows.append(base)
        for angle in (10.,30.):
            v=copy.deepcopy(base);v['episode_id']=f'r01_{geometry}_angle{int(angle)}';v['fixture_parameters']['angle_deg']=angle
            v['counterfactual'].update(baseline_episode_id=base_id,changed_pointer='/fixture_parameters/angle_deg')
            rows.append(v);pairs.append((base_id,v['episode_id']))
        base_id=f'r01_{geometry}_friction_reference';family=f'r01_{geometry}_friction'
        base=copy.deepcopy(base);base['episode_id']=base_id;base['counterfactual']={'family_id':family,'baseline_episode_id':None,'changed_pointer':None}
        rows.append(base)
        v=copy.deepcopy(base);v['episode_id']=f'r01_{geometry}_friction_low';v['objects'][0]['physics_profile_id']='rigid_low_friction'
        v['counterfactual'].update(baseline_episode_id=base_id,changed_pointer='/objects/0/physics/dynamic_friction')
        rows.append(v);pairs.append((base_id,v['episode_id']))
    for compression in cfg['V02']['compression_fractions']:
        suffix=str(compression).replace('.','p');base_id=f'v02_c{suffix}_modulus_reference';family=f'v02_c{suffix}_modulus'
        base=_common(base_id,'V02','rounded_cube','elastic_reference',family,seed,cfg['V02']['numerics_profile_id'])
        base.update(fixture_parameters=dict(compression_fraction=compression),
                    action_parameters=dict(start_time_s=1.,compression_duration_s=1.,hold_duration_s=.5,withdraw_duration_s=.5),
                    timing=dict(physics_hz=240,capture_hz=30,duration_s=5.))
        rows.append(base)
        for profile in ('elastic_soft','elastic_stiff'):
            v=copy.deepcopy(base);v['episode_id']=f'v02_c{suffix}_modulus_{profile.split("_")[-1]}'
            v['objects'][0]['physics_profile_id']=profile
            v['counterfactual'].update(baseline_episode_id=base_id,changed_pointer='/objects/0/physics/youngs_modulus_pa')
            rows.append(v);pairs.append((base_id,v['episode_id']))
        base_id=f'v02_c{suffix}_speed_reference';family=f'v02_c{suffix}_speed'
        base=copy.deepcopy(base);base['episode_id']=base_id;base['counterfactual']={'family_id':family,'baseline_episode_id':None,'changed_pointer':None}
        rows.append(base)
        v=copy.deepcopy(base);v['episode_id']=f'v02_c{suffix}_speed_fast';v['action_parameters']['compression_duration_s']=.5
        v['counterfactual'].update(baseline_episode_id=base_id,changed_pointer='/action_parameters/compression_duration_s')
        rows.append(v);pairs.append((base_id,v['episode_id']))
    if len(rows)!=cfg['R01']['episodes']+cfg['V02']['episodes']:raise AssertionError('Matrix size mismatch')
    by_id={r['episode_id']:r for r in rows}
    for base,variant in pairs:validate_pair(by_id[base],by_id[variant])
    for row in rows:resolve(row)
    return rows,pairs


def write_specs(root):
    root=Path(root);root.mkdir(parents=True,exist_ok=False);specs=root/'specs';specs.mkdir()
    rows,pairs=generate()
    for row in rows:write_json(specs/(row['episode_id']+'.json'),row)
    write_json(root/'matrix.json',dict(schema_version='0.1.0',episodes=[r['episode_id'] for r in rows],pairs=[list(p) for p in pairs]))
    return rows,pairs


def ensure_specs(root):
    """Create once, or verify an interrupted matrix before resuming it."""
    root=Path(root);rows,pairs=generate()
    if not root.exists():return write_specs(root)
    expected=dict(schema_version='0.1.0',episodes=[r['episode_id'] for r in rows],pairs=[list(p) for p in pairs])
    if read_json(root/'matrix.json')!=expected:raise ValueError('Existing matrix definition differs')
    for row in rows:
        if read_json(root/'specs'/(row['episode_id']+'.json'))!=row:
            raise ValueError(f'Existing matrix spec differs: {row["episode_id"]}')
    return rows,pairs


def compare_replay(first,second,require_complete=True):
    """Check exact action replay plus backend-appropriate state repeatability.

    PhysX rigid GPU runs are bit-repeatable in the current probe. Volume
    deformables are not bit-deterministic, so their trajectory is held to a
    declared D-relative envelope and aggregate-metric agreement instead of
    being falsely described as exact state replay.
    """
    a=Episode(first,require_complete=require_complete);b=Episode(second,require_complete=require_complete)
    if a.manifest['inputs']!=b.manifest['inputs']:raise ValueError('Replay inputs differ')
    if read_json(a.root/'action.json')!=read_json(b.root/'action.json'):raise ValueError('Replay actions differ')
    af=list(a.states());bf=list(b.states())
    if len(af)!=len(bf):raise ValueError('Replay state count mismatch')
    worst=0.;velocity_worst=0.
    for i,(x,y) in enumerate(zip(af,bf)):
        if x['time_s']!=y['time_s']:raise ValueError('Replay time mismatch')
        gx=a.geometry(i);gy=b.geometry(i)
        for key in ('surface_world_m','simulation_points_m'):
            if gx[key].shape!=gy[key].shape:raise ValueError('Replay shape mismatch')
            if gx[key].size:worst=max(worst,float(np.max(np.abs(gx[key]-gy[key]))))
        key='simulation_nodal_velocities_m_s'
        if gx[key].shape!=gy[key].shape:raise ValueError('Replay velocity shape mismatch')
        if gx[key].size:velocity_worst=max(velocity_worst,float(np.max(np.abs(gx[key]-gy[key]))))
    kind=af[0]['objects'][next(iter(af[0]['objects']))]['physics_kind']
    d=a.manifest['inputs']['objects'][0]['geometry']['characteristic_size_m']
    tolerance=1e-7 if kind=='rigid' else .02*d
    metrics_a=read_json(a.root/'metrics.json');metrics_b=read_json(b.root/'metrics.json')
    aggregate={}
    if kind=='volumetric':
        aggregate={
            'compression_fraction_difference':abs(metrics_a['max_compression_fraction']-metrics_b['max_compression_fraction']),
            'final_shape_residual_difference_D':abs(metrics_a['final_shape_residual_m']-metrics_b['final_shape_residual_m'])/d,
        }
        if aggregate['compression_fraction_difference']>.01 or aggregate['final_shape_residual_difference_D']>.01:
            raise ValueError(f'Deformable replay aggregate drift: {aggregate}')
    if worst>tolerance:raise ValueError(f'Replay differs by {worst} > {tolerance}')
    return dict(kind=kind,action_replay='exact',state_repeatability='bit_exact' if kind=='rigid' else 'bounded_gpu_backend',
                maximum_absolute_position_difference_m=worst,maximum_absolute_nodal_velocity_difference_m_s=velocity_worst,
                position_tolerance_m=tolerance,aggregate_differences=aggregate,passed=True)


def assess_pairs(root,pairs,require_complete=True):
    root=Path(root);specs=root/'specs';episodes=root/'episodes';reports=[]
    for base,variant in pairs:
        base_spec=read_json(specs/(base+'.json'));variant_spec=read_json(specs/(variant+'.json'))
        validate_pair(base_spec,variant_spec)
        a=Episode(episodes/base,require_complete=require_complete)
        b=Episode(episodes/variant,require_complete=require_complete)
        instance=base_spec['objects'][0]['instance_id']
        with np.load(a.root/'geometry'/(instance+'.npz'),allow_pickle=False) as ga:
            with np.load(b.root/'geometry'/(instance+'.npz'),allow_pickle=False) as gb:
                initial=float(np.max(np.abs(ga['vertices']-gb['vertices'])))
        if initial>1e-7:raise ValueError(f'Counterfactual rest geometry differs: {base}, {variant}: {initial}')
        reports.append(dict(baseline_episode_id=base,variant_episode_id=variant,
                            changed_pointer=variant_spec['counterfactual']['changed_pointer'],
                            rest_geometry_maximum_difference_m=initial,
                            initial_state_semantics=('world pose is fixture-derived and may change with the declared fixture variable'
                                if variant_spec['counterfactual']['changed_pointer'].startswith('/fixture_parameters/')
                                else 'same world fixture; native initial state generated from identical rest geometry'),passed=True))
    return reports


def assess_replays(root,require_complete=True):
    root=Path(root);episodes=root/'episodes'
    pairs=[(f'r01_{g}_angle20',f'r01_{g}_friction_reference') for g in ('sphere','cylinder','rounded_cube','capsule')]
    pairs += [(f'v02_c{str(c).replace(".","p")}_modulus_reference',f'v02_c{str(c).replace(".","p")}_speed_reference') for c in (.1,.25,.4)]
    return [dict(first=a,second=b,**compare_replay(episodes/a,episodes/b,require_complete=require_complete)) for a,b in pairs]


def main():
    import argparse
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('output',type=Path)
    parser.add_argument('--event',choices=('R01','V02','all'),default='all');parser.add_argument('--no-render',action='store_true')
    parser.add_argument('--limit',type=int,default=0);args=parser.parse_args()
    rows,pairs=ensure_specs(args.output);selected=[r for r in rows if args.event=='all' or r['event_id']==args.event]
    if args.limit:selected=selected[:args.limit]
    result_dir=args.output/'results';result_dir.mkdir(exist_ok=True);outcomes=[]
    report_name='physics_validation.json' if args.no_render else 'validation.json'
    for row in selected:
        episode=args.output/'episodes'/row['episode_id'];episode.parent.mkdir(exist_ok=True)
        result_path=result_dir/(row['episode_id']+'.json')
        if result_path.exists():
            outcome=read_json(result_path)
        else:
            if episode.exists():
                validation=read_json(episode/report_name) if (episode/report_name).exists() else None
                if not validation or not validation['passed']:
                    raise RuntimeError(f'Existing incomplete/failed immutable episode requires diagnosis: {episode}')
                returncode=0
            else:
                command=[sys.executable,'-m','world_model_dataset.local',str(args.output/'specs'/(row['episode_id']+'.json')),str(episode)]
                if not args.no_render:command.append('--render')
                result=subprocess.run(command,cwd=ROOT);returncode=result.returncode
                validation=read_json(episode/report_name) if (episode/report_name).exists() else None
                if returncode or validation is None:
                    raise RuntimeError(f'Prototype execution did not produce a result: {row["episode_id"]} (exit {returncode})')
            outcome=dict(episode_id=row['episode_id'],returncode=returncode,
                stage='physics' if args.no_render else 'complete_episode',
                accepted=bool(validation and validation['passed']),missing=None if validation is None else validation['missing_required'])
            write_json(result_path,outcome)
        outcomes.append(outcome)
        if not outcome['accepted']:raise RuntimeError(f'Prototype episode rejected: {row["episode_id"]}')
    complete_matrix=len(selected)==len(rows) and all(o['accepted'] for o in outcomes)
    pair_reports=assess_pairs(args.output,pairs) if complete_matrix and not args.no_render else []
    replay_reports=assess_replays(args.output) if complete_matrix and not args.no_render else []
    summary=dict(schema_version='0.1.0',requested=len(selected),outcomes=outcomes,
        accepted=sum(o['accepted'] for o in outcomes),pair_reports=pair_reports,replay_reports=replay_reports,
        complete=True,physics_matrix_accepted=complete_matrix,
        prototype_accepted=(complete_matrix and not args.no_render and
                            all(x['passed'] for x in pair_reports+replay_reports)))
    if (args.output/'summary.json').exists():
        if read_json(args.output/'summary.json')!=summary:raise ValueError('Existing summary differs')
    else:write_json(args.output/'summary.json',summary)


if __name__=='__main__':main()

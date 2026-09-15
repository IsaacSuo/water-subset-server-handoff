"""Small, deterministic M4 event matrices and their serial physics runner."""
from __future__ import annotations

import copy
import subprocess
import sys
from pathlib import Path

from .contract import CONFIG,ROOT,resolve,validate_pair
from .io import read_json,write_json


def _orientation(geometry):
    return [2**-.5,0.,0.,2**-.5] if geometry in ('cylinder','capsule') else [0.,0.,0.,1.]


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


def generate_v01(config_path=CONFIG/'prototypes/M4_V01.json'):
    cfg=read_json(config_path);rows=[];pairs=[]
    for geometry in cfg['geometries']:
        base_id=f'v01_{geometry}_reference';family=f'v01_{geometry}'
        base=dict(schema_version='0.1.0',episode_id=base_id,event_id='V01',environment_id='canonical_studio',
            camera_set_id='two_fixed',numerics_profile_id=cfg['numerics_profile_id'],
            objects=[dict(instance_id='subject',object_id=geometry,physics_profile_id='elastic_reference',
                          appearance_profile_id='neutral_blue',orientation_xyzw=_orientation(geometry))],
            fixture_parameters={'drop_height_D':cfg['baseline_drop_height_D']},action_parameters={},
            timing={'physics_hz':240,'capture_hz':60,'duration_s':cfg['duration_s']},seed=cfg['seed'],
            counterfactual={'family_id':family,'baseline_episode_id':None,'changed_pointer':None},split='development')
        rows.append(base)
        for profile in ('elastic_soft','elastic_stiff'):
            variant=copy.deepcopy(base);suffix=profile.removeprefix('elastic_');variant['episode_id']=f'v01_{geometry}_{suffix}'
            variant['objects'][0]['physics_profile_id']=profile
            variant['counterfactual'].update(baseline_episode_id=base_id,changed_pointer='/objects/0/physics/youngs_modulus_pa')
            rows.append(variant);pairs.append((base_id,variant['episode_id']))
        if geometry=='rounded_cube':
            for height in cfg['rounded_cube_height_variants_D']:
                variant=copy.deepcopy(base);suffix=str(height).replace('.','p');variant['episode_id']=f'v01_rounded_cube_height_{suffix}D'
                variant['fixture_parameters']['drop_height_D']=height
                variant['counterfactual'].update(baseline_episode_id=base_id,changed_pointer='/fixture_parameters/drop_height_D')
                rows.append(variant);pairs.append((base_id,variant['episode_id']))
    if len(rows)!=cfg['episodes'] or len(pairs)!=cfg['pairs']:raise AssertionError('V01 matrix size mismatch')
    by_id={row['episode_id']:row for row in rows}
    for base,variant in pairs:validate_pair(by_id[base],by_id[variant])
    for row in rows:resolve(row)
    return rows,pairs


def ensure_v01_specs(root):
    return _ensure_specs(root,'V01',*generate_v01())


def generate_r02(config_path=CONFIG/'prototypes/M4_R02.json'):
    cfg=read_json(config_path);rows=[];pairs=[]
    for geometry in cfg['geometries']:
        base_id=f'r02_{geometry}_reference';family=f'r02_{geometry}'
        base=dict(schema_version='0.1.0',episode_id=base_id,event_id='R02',environment_id='canonical_studio',
            camera_set_id='two_fixed',numerics_profile_id='reference',
            objects=[dict(instance_id='subject',object_id=geometry,physics_profile_id='rigid_reference',
                          appearance_profile_id='neutral_blue',orientation_xyzw=_orientation(geometry))],
            fixture_parameters={'step_count':cfg['step_count'],'step_height_D':cfg['baseline_step_height_D'],
                                'tread_depth_D':cfg['tread_depth_D']},
            action_parameters={'start_time_s':.5,'push_speed_m_s':cfg['baseline_push_speed_m_s']},
            timing={'physics_hz':240,'capture_hz':60,'duration_s':cfg['duration_s']},seed=cfg['seed'],
            counterfactual={'family_id':family,'baseline_episode_id':None,'changed_pointer':None},split='development')
        rows.append(base)
        for profile,pointer in (('rigid_low_friction','/objects/0/physics/dynamic_friction'),
                                ('rigid_bouncy','/objects/0/physics/restitution')):
            variant=copy.deepcopy(base);suffix=profile.removeprefix('rigid_');variant['episode_id']=f'r02_{geometry}_{suffix}'
            variant['objects'][0]['physics_profile_id']=profile
            variant['counterfactual'].update(baseline_episode_id=base_id,changed_pointer=pointer)
            rows.append(variant);pairs.append((base_id,variant['episode_id']))
        if geometry=='rounded_cube':
            for height in cfg['rounded_cube_height_variants_D']:
                variant=copy.deepcopy(base);suffix=str(height).replace('.','p');variant['episode_id']=f'r02_rounded_cube_height_{suffix}D'
                variant['fixture_parameters']['step_height_D']=height
                variant['counterfactual'].update(baseline_episode_id=base_id,changed_pointer='/fixture_parameters/step_height_D')
                rows.append(variant);pairs.append((base_id,variant['episode_id']))
    if len(rows)!=cfg['episodes'] or len(pairs)!=cfg['pairs']:raise AssertionError('R02 matrix size mismatch')
    by_id={row['episode_id']:row for row in rows}
    for base,variant in pairs:validate_pair(by_id[base],by_id[variant])
    for row in rows:resolve(row)
    return rows,pairs


def ensure_r02_specs(root):
    return _ensure_specs(root,'R02',*generate_r02())


def generate_v02_corrected():
    """Nine corrected-material runs; historical M3 specs/caches stay immutable."""
    from .prototypes import generate
    original,original_pairs=generate()
    rows=[copy.deepcopy(row) for row in original
          if row['event_id']=='V02' and '_modulus_' in row['episode_id']]
    selected={row['episode_id'] for row in rows}
    pairs=[pair for pair in original_pairs if set(pair)<=selected]
    for row in rows:
        row['episode_id']+='_material_corrected'
        cf=row['counterfactual'];cf['family_id']+='_material_corrected'
        if cf['baseline_episode_id']:cf['baseline_episode_id']+='_material_corrected'
    pairs=[(a+'_material_corrected',b+'_material_corrected') for a,b in pairs]
    by_id={row['episode_id']:row for row in rows}
    for a,b in pairs:validate_pair(by_id[a],by_id[b])
    for row in rows:resolve(row)
    return rows,pairs


def ensure_v02_corrected_specs(root):
    return _ensure_specs(root,'V02',*generate_v02_corrected())


def generate_v05(config_path=CONFIG/'prototypes/M4_V05.json'):
    cfg=read_json(config_path);base=read_json(CONFIG/'examples/v05_sphere_soft_cube_impact.json')
    base.update(episode_id='v05_sphere_soft_cube_reference',seed=cfg['seed'],
        numerics_profile_id=cfg['numerics_profile_id'],
        timing={'physics_hz':240,'capture_hz':60,'duration_s':cfg['duration_s']},
        fixture_parameters={'separation_D':cfg['separation_D'],'impact_offset_D':0.},
        action_parameters={'start_time_s':.5,'impact_speed_m_s':cfg['baseline_speed_m_s']},
        counterfactual={'family_id':'v05_sphere_soft_cube','baseline_episode_id':None,'changed_pointer':None})
    rows=[base];pairs=[]
    def variant(suffix,pointer):
        row=copy.deepcopy(base);row['episode_id']='v05_sphere_soft_cube_'+suffix
        row['counterfactual'].update(baseline_episode_id=base['episode_id'],changed_pointer=pointer)
        rows.append(row);pairs.append((base['episode_id'],row['episode_id']));return row
    for speed,label in zip(cfg['speed_variants_m_s'],('speed_low','speed_high')):
        row=variant(label,'/action_parameters/impact_speed_m_s');row['action_parameters']['impact_speed_m_s']=speed
    for profile in cfg['projectile_mass_profiles']:
        row=variant('projectile_'+profile.removeprefix('rigid_'),'/objects/1/physics/density_kg_m3')
        row['objects'][1]['physics_profile_id']=profile
    for profile in cfg['target_modulus_profiles']:
        row=variant('target_'+profile.removeprefix('elastic_'),'/objects/0/physics/youngs_modulus_pa')
        row['objects'][0]['physics_profile_id']=profile
    for offset,label in zip(cfg['impact_offset_variants_D'],('eccentric','miss')):
        row=variant(label,'/fixture_parameters/impact_offset_D');row['fixture_parameters']['impact_offset_D']=offset
    # Geometry cases also change resolved collision representation/mesh fields;
    # do not label them as leaf-level single-variable pairs in frozen v0.1.
    for oid,index,label in (('capsule',1,'capsule_soft_cube'),('sphere',0,'sphere_soft_sphere')):
        row=copy.deepcopy(base);row['episode_id']='v05_'+label+'_reference';row['objects'][index]['object_id']=oid
        row['counterfactual']={'family_id':'v05_'+label,'baseline_episode_id':None,'changed_pointer':None};rows.append(row)
    if len(rows)!=cfg['episodes'] or len(pairs)!=cfg['pairs']:raise AssertionError('V05 matrix size mismatch')
    by_id={row['episode_id']:row for row in rows}
    for a,b in pairs:validate_pair(by_id[a],by_id[b])
    for row in rows:resolve(row)
    return rows,pairs


def ensure_v05_specs(root):
    return _ensure_specs(root,'V05',*generate_v05())


def generate_r04(config_path=CONFIG/'prototypes/M4_R04.json'):
    cfg=read_json(config_path);rows=[];pairs=[]
    for geometry in cfg['geometries']:
        base_id=f'r04_{geometry}_reference';family=f'r04_{geometry}'
        base=dict(schema_version='0.1.0',episode_id=base_id,event_id='R04',environment_id='canonical_studio',
            camera_set_id='two_fixed',numerics_profile_id='reference',
            objects=[dict(instance_id='subject',object_id=geometry,physics_profile_id='rigid_reference',
                          appearance_profile_id='neutral_blue',orientation_xyzw=_orientation(geometry))],
            fixture_parameters={'obstacle_offset_D':cfg['baseline_obstacle_offset_D'],
                                'subject_offset_D':cfg['baseline_subject_offset_D']},
            action_parameters={'start_time_s':.5,'push_distance_D':cfg['push_distance_D'],
                               'push_duration_s':cfg['baseline_push_duration_s']},
            timing={'physics_hz':240,'capture_hz':60,'duration_s':cfg['duration_s']},seed=cfg['seed'],
            counterfactual={'family_id':family,'baseline_episode_id':None,'changed_pointer':None},split='development')
        rows.append(base)
        low=copy.deepcopy(base);low['episode_id']=f'r04_{geometry}_low_friction';low['objects'][0]['physics_profile_id']='rigid_low_friction'
        low['counterfactual'].update(baseline_episode_id=base_id,changed_pointer='/objects/0/physics/dynamic_friction')
        rows.append(low);pairs.append((base_id,low['episode_id']))
        if geometry=='sphere':
            for duration,label in zip(cfg['sphere_duration_variants_s'],('push_slow','push_fast')):
                row=copy.deepcopy(base);row['episode_id']=f'r04_sphere_{label}';row['action_parameters']['push_duration_s']=duration
                row['counterfactual'].update(baseline_episode_id=base_id,changed_pointer='/action_parameters/push_duration_s')
                rows.append(row);pairs.append((base_id,row['episode_id']))
            for offset in cfg['sphere_obstacle_offset_variants_D']:
                label='negative' if offset<0 else 'positive';row=copy.deepcopy(base);row['episode_id']=f'r04_sphere_obstacle_{label}'
                row['fixture_parameters']['obstacle_offset_D']=offset
                row['counterfactual'].update(baseline_episode_id=base_id,changed_pointer='/fixture_parameters/obstacle_offset_D')
                rows.append(row);pairs.append((base_id,row['episode_id']))
            for offset,label in zip(cfg['sphere_subject_offset_variants_D'],('mirror_branch','wide_miss')):
                row=copy.deepcopy(base);row['episode_id']=f'r04_sphere_{label}';row['fixture_parameters']['subject_offset_D']=offset
                row['counterfactual'].update(baseline_episode_id=base_id,changed_pointer='/fixture_parameters/subject_offset_D')
                rows.append(row);pairs.append((base_id,row['episode_id']))
    if len(rows)!=cfg['episodes'] or len(pairs)!=cfg['pairs']:raise AssertionError('R04 matrix size mismatch')
    by_id={row['episode_id']:row for row in rows}
    for a,b in pairs:validate_pair(by_id[a],by_id[b])
    for row in rows:resolve(row)
    return rows,pairs


def ensure_r04_specs(root):
    return _ensure_specs(root,'R04',*generate_r04())


def generate_v03(config_path=CONFIG/'prototypes/M4_V03.json'):
    cfg=read_json(config_path);rows=[];pairs=[]
    base_id='v03_cube_load_reference';family='v03_cube_load'
    base=dict(schema_version='0.1.0',episode_id=base_id,event_id='V03',environment_id='canonical_studio',
        camera_set_id='two_fixed',numerics_profile_id=cfg['numerics_profile_id'],
        objects=[
            dict(instance_id='target',object_id='rounded_cube',physics_profile_id=cfg['baseline_target_profile'],
                 appearance_profile_id='neutral_blue',orientation_xyzw=[0.,0.,0.,1.]),
            dict(instance_id='load',object_id='rounded_cube',physics_profile_id=cfg['baseline_load_profile'],
                 appearance_profile_id='neutral_orange',orientation_xyzw=[0.,0.,0.,1.]),
        ],
        fixture_parameters={'load_clearance_D':cfg['load_clearance_D'],'load_offset_D':0.},
        action_parameters={'load_start_time_s':cfg['load_start_time_s'],'load_duration_s':cfg['baseline_load_duration_s']},
        timing={'physics_hz':240,'capture_hz':60,'duration_s':cfg['duration_s']},seed=cfg['seed'],
        counterfactual={'family_id':family,'baseline_episode_id':None,'changed_pointer':None},split='development')
    rows.append(base)
    def variant(label,pointer):
        row=copy.deepcopy(base);row['episode_id']='v03_'+label
        row['counterfactual'].update(baseline_episode_id=base_id,changed_pointer=pointer)
        rows.append(row);pairs.append((base_id,row['episode_id']));return row
    for profile,label in zip(cfg['load_mass_profiles'],('load_light','load_heavy')):
        variant(label,'/objects/1/physics/density_kg_m3')['objects'][1]['physics_profile_id']=profile
    for profile,label in zip(cfg['target_modulus_profiles'],('target_soft','target_stiff')):
        variant(label,'/objects/0/physics/youngs_modulus_pa')['objects'][0]['physics_profile_id']=profile
    for duration,label in zip(cfg['load_duration_variants_s'],('duration_short','duration_long')):
        variant(label,'/action_parameters/load_duration_s')['action_parameters']['load_duration_s']=duration
    variant('load_eccentric','/fixture_parameters/load_offset_D')['fixture_parameters']['load_offset_D']=cfg['load_offset_variant_D']
    for geometry in cfg['load_geometry_variants']:
        row=copy.deepcopy(base);row['episode_id']=f'v03_{geometry}_load_reference';row['objects'][1]['object_id']=geometry
        row['counterfactual']={'family_id':f'v03_{geometry}_load','baseline_episode_id':None,'changed_pointer':None};rows.append(row)
    for geometry in cfg['target_geometry_variants']:
        row=copy.deepcopy(base);row['episode_id']=f'v03_cube_load_soft_{geometry}_reference';row['objects'][0]['object_id']=geometry
        row['counterfactual']={'family_id':f'v03_soft_{geometry}_target','baseline_episode_id':None,'changed_pointer':None};rows.append(row)
    if len(rows)!=cfg['episodes'] or len(pairs)!=cfg['pairs']:raise AssertionError('V03 matrix size mismatch')
    by_id={row['episode_id']:row for row in rows}
    for a,b in pairs:validate_pair(by_id[a],by_id[b])
    for row in rows:resolve(row)
    return rows,pairs


def ensure_v03_specs(root):
    return _ensure_specs(root,'V03',*generate_v03())


def generate_r05(config_path=CONFIG/'prototypes/M4_R05.json'):
    cfg=read_json(config_path);base_id='r05_cube_stack_reference';family='r05_cube_stack'
    objects=[dict(instance_id=f'level_{i}',object_id='rounded_cube',physics_profile_id='rigid_reference',
                  appearance_profile_id='neutral_blue' if i%2==0 else 'neutral_orange',orientation_xyzw=[0.,0.,0.,1.])
             for i in range(cfg['object_count'])]
    base=dict(schema_version='0.1.0',episode_id=base_id,event_id='R05',environment_id='canonical_studio',
        camera_set_id='two_fixed',numerics_profile_id='reference',objects=objects,
        fixture_parameters={'support_height_D':cfg['baseline_support_height_D'],'level_offset_D':cfg['baseline_level_offset_D']},
        action_parameters={'remove_time_s':cfg['remove_time_s']},
        timing={'physics_hz':240,'capture_hz':60,'duration_s':cfg['duration_s']},seed=cfg['seed'],
        counterfactual={'family_id':family,'baseline_episode_id':None,'changed_pointer':None},split='development')
    rows=[base];pairs=[]
    def variant(label,pointer):
        row=copy.deepcopy(base);row['episode_id']='r05_cube_stack_'+label
        row['counterfactual'].update(baseline_episode_id=base_id,changed_pointer=pointer)
        rows.append(row);pairs.append((base_id,row['episode_id']));return row
    pointers={'rigid_low_friction':'dynamic_friction','rigid_bouncy':'restitution',
              'rigid_light':'density_kg_m3','rigid_heavy':'density_kg_m3'}
    for profile in cfg['top_material_variants']:
        row=variant('top_'+profile.removeprefix('rigid_'),f'/objects/3/physics/{pointers[profile]}')
        row['objects'][3]['physics_profile_id']=profile
    for value in cfg['level_offset_variants_D']:
        label='vertical' if value==0 else 'lean_boundary';row=variant(label,'/fixture_parameters/level_offset_D')
        row['fixture_parameters']['level_offset_D']=value
    for value,label in zip(cfg['support_height_variants_D'],('support_low','support_high')):
        row=variant(label,'/fixture_parameters/support_height_D');row['fixture_parameters']['support_height_D']=value
    for geometry in cfg['top_geometry_variants']:
        row=copy.deepcopy(base);row['episode_id']=f'r05_cube_stack_top_{geometry}_reference';row['objects'][3]['object_id']=geometry
        row['counterfactual']={'family_id':f'r05_top_{geometry}','baseline_episode_id':None,'changed_pointer':None};rows.append(row)
    if len(rows)!=cfg['episodes'] or len(pairs)!=cfg['pairs']:raise AssertionError('R05 matrix size mismatch')
    by_id={row['episode_id']:row for row in rows}
    for a,b in pairs:validate_pair(by_id[a],by_id[b])
    for row in rows:resolve(row)
    return rows,pairs


def ensure_r05_specs(root):
    return _ensure_specs(root,'R05',*generate_r05())


def generate_v04(config_path=CONFIG/'prototypes/M4_V04.json'):
    cfg=read_json(config_path);base_id='v04_cube_aperture_reference';family='v04_cube_aperture'
    base=dict(schema_version='0.1.0',episode_id=base_id,event_id='V04',environment_id='canonical_studio',
        camera_set_id='two_fixed',numerics_profile_id=cfg['numerics_profile_id'],
        objects=[dict(instance_id='subject',object_id='rounded_cube',physics_profile_id=cfg['baseline_target_profile'],
                      appearance_profile_id='neutral_blue',orientation_xyzw=[0.,0.,0.,1.])],
        fixture_parameters={'aperture_width_D':cfg['baseline_aperture_width_D']},
        action_parameters={'start_time_s':cfg['start_time_s'],'push_distance_D':cfg['baseline_push_distance_D'],
                           'push_duration_s':cfg['baseline_push_duration_s']},
        timing={'physics_hz':240,'capture_hz':60,'duration_s':cfg['duration_s']},seed=cfg['seed'],
        counterfactual={'family_id':family,'baseline_episode_id':None,'changed_pointer':None},split='development')
    rows=[base];pairs=[]
    def variant(label,pointer):
        row=copy.deepcopy(base);row['episode_id']='v04_cube_aperture_'+label
        row['counterfactual'].update(baseline_episode_id=base_id,changed_pointer=pointer)
        rows.append(row);pairs.append((base_id,row['episode_id']));return row
    for width,label in zip(cfg['aperture_width_variants_D'],('narrow','wide')):
        variant(label,'/fixture_parameters/aperture_width_D')['fixture_parameters']['aperture_width_D']=width
    for profile,label in zip(cfg['target_modulus_profiles'],('soft','stiff')):
        variant(label,'/objects/0/physics/youngs_modulus_pa')['objects'][0]['physics_profile_id']=profile
    for duration,label in zip(cfg['push_duration_variants_s'],('push_fast','push_slow')):
        variant(label,'/action_parameters/push_duration_s')['action_parameters']['push_duration_s']=duration
    for distance,label in zip(cfg['push_distance_variants_D'],('push_short','push_long')):
        variant(label,'/action_parameters/push_distance_D')['action_parameters']['push_distance_D']=distance
    for geometry in cfg['geometry_variants']:
        row=copy.deepcopy(base);row['episode_id']=f'v04_{geometry}_aperture_reference';row['objects'][0]['object_id']=geometry
        row['objects'][0]['orientation_xyzw']=_orientation(geometry)
        row['counterfactual']={'family_id':f'v04_{geometry}_aperture','baseline_episode_id':None,'changed_pointer':None};rows.append(row)
    if len(rows)!=cfg['episodes'] or len(pairs)!=cfg['pairs']:raise AssertionError('V04 matrix size mismatch')
    by_id={row['episode_id']:row for row in rows}
    for a,b in pairs:validate_pair(by_id[a],by_id[b])
    for row in rows:resolve(row)
    return rows,pairs


def ensure_v04_specs(root):
    return _ensure_specs(root,'V04',*generate_v04())


def generate_real_rigid_supplement(config_path=CONFIG/'prototypes/M4_REAL_RIGID.json'):
    cfg=read_json(config_path);common=dict(schema_version='0.1.0',environment_id='canonical_studio',
        camera_set_id='two_fixed',numerics_profile_id='reference',seed=cfg['seed'],split='development')
    def obj(instance,geometry,appearance='neutral_blue'):
        return dict(instance_id=instance,object_id=geometry,physics_profile_id='rigid_reference',
                    appearance_profile_id=appearance,orientation_xyzw=[0.,0.,0.,1.])
    rows=[
        dict(common,episode_id='r03_asset_banana_reference',event_id='R03',
            objects=[obj('left','asset_banana'),obj('right','asset_banana','neutral_orange')],
            fixture_parameters={'separation_D':4.,'lateral_offset_D':0.},
            action_parameters={'start_time_s':.5,'left_speed_m_s':1.,'right_speed_m_s':1.},
            timing={'physics_hz':240,'capture_hz':60,'duration_s':2.},
            counterfactual={'family_id':'r03_asset_banana','baseline_episode_id':None,'changed_pointer':None}),
        dict(common,episode_id='r02_asset_carrot_reference',event_id='R02',objects=[obj('subject','asset_carrot')],
            fixture_parameters={'step_count':5,'step_height_D':.3,'tread_depth_D':1.5},
            action_parameters={'start_time_s':.5,'push_speed_m_s':1.4},
            timing={'physics_hz':240,'capture_hz':60,'duration_s':4.},
            counterfactual={'family_id':'r02_asset_carrot','baseline_episode_id':None,'changed_pointer':None}),
        dict(common,episode_id='r04_asset_chair_reference',event_id='R04',objects=[obj('subject','asset_chair')],
            fixture_parameters={'obstacle_offset_D':0.,'subject_offset_D':-.15},
            action_parameters={'start_time_s':.5,'push_distance_D':1.5,'push_duration_s':.3},
            timing={'physics_hz':240,'capture_hz':60,'duration_s':5.},
            counterfactual={'family_id':'r04_asset_chair','baseline_episode_id':None,'changed_pointer':None}),
        dict(common,episode_id='r05_stack_asset_elephant_reference',event_id='R05',
            objects=[obj(f'level_{i}','rounded_cube','neutral_blue' if i%2==0 else 'neutral_orange') for i in range(3)]+[
                obj('level_3','asset_elephant','neutral_orange')],
            fixture_parameters={'support_height_D':.6,'level_offset_D':.15},action_parameters={'remove_time_s':1.},
            timing={'physics_hz':240,'capture_hz':60,'duration_s':4.},
            counterfactual={'family_id':'r05_stack_asset_elephant','baseline_episode_id':None,'changed_pointer':None}),
    ]
    if len(rows)!=cfg['episodes']:raise AssertionError('Real-rigid supplement size mismatch')
    for row in rows:resolve(row)
    return rows,[]


def ensure_real_rigid_specs(root):
    return _ensure_specs(root,'REAL_RIGID',*generate_real_rigid_supplement())


def _ensure_specs(root,event_id,rows,pairs):
    root=Path(root);definition={'schema_version':'0.1.0','event_id':event_id,
        'episodes':[row['episode_id'] for row in rows],'pairs':[list(pair) for pair in pairs]}
    if root.exists():
        if read_json(root/'matrix.json')!=definition:raise ValueError(f'Existing {event_id} matrix definition differs')
        for row in rows:
            if read_json(root/'specs'/(row['episode_id']+'.json'))!=row:raise ValueError(f'Existing {event_id} spec differs: '+row['episode_id'])
        return rows,pairs
    (root/'specs').mkdir(parents=True)
    for row in rows:write_json(root/'specs'/(row['episode_id']+'.json'),row)
    write_json(root/'matrix.json',definition);return rows,pairs


def main():
    import argparse
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('output',type=Path)
    parser.add_argument('--event',choices=('R03','V01','R02','V02','V05','R04','V03','R05','V04','REAL_RIGID'),default='R03');parser.add_argument('--limit',type=int,default=0);args=parser.parse_args()
    generators={'R03':generate_r03,'V01':generate_v01,'R02':generate_r02,'V02':generate_v02_corrected,'V05':generate_v05,'R04':generate_r04,'V03':generate_v03,'R05':generate_r05,'V04':generate_v04,'REAL_RIGID':generate_real_rigid_supplement}
    ensure_functions={'R03':ensure_r03_specs,'V01':ensure_v01_specs,'R02':ensure_r02_specs,'V02':ensure_v02_corrected_specs,'V05':ensure_v05_specs,'R04':ensure_r04_specs,'V03':ensure_v03_specs,'R05':ensure_r05_specs,'V04':ensure_v04_specs,'REAL_RIGID':ensure_real_rigid_specs}
    generator=generators[args.event];ensure=ensure_functions[args.event]
    rows,pairs=ensure(args.output)
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
            if returncode or not validation:raise RuntimeError(f'{args.event} execution incomplete: {row["episode_id"]}')
            outcome={'episode_id':row['episode_id'],'physics_passed':validation['passed']}
            write_json(result_path,outcome)
        outcomes.append(outcome)
        if not outcome['physics_passed']:raise RuntimeError(f'{args.event} numerical integrity failed: '+row['episode_id'])
    complete=len(rows)==len(generator()[0])
    summary={'schema_version':'0.1.0','event_id':args.event,'requested':len(rows),'accepted':len(outcomes),
             'complete':complete,'outcomes':outcomes}
    write_json(args.output/('summary.json' if complete else f'progress_{len(rows):03d}.json'),summary)


if __name__=='__main__':main()

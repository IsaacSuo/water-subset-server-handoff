"""Configurable objects/conditions on unchanged exported existing-scene geometry."""
import argparse
import copy
from pathlib import Path

import numpy as np
import trimesh

from .causal_runner import ROOT, load_config, prepare, invoke, package_physics
from .io import read_json, write_json, file_hash
from .phenomenon_recipes02 import add_body


def recipe(spec, experiment, asset, condition):
    scene_path=ROOT/spec['scene_export']
    scene=read_json(scene_path)
    active=experiment['kind']=='push_obstruction'
    config=load_config(ROOT/'configs/dataset/v0_2'/('c2_rigid_push.json' if active else 'c2_none_collision.json'))
    manifest=read_json(ROOT/config['manifest_template'])
    config['initial_state_overrides']={}
    config.pop('body_profile_overrides',None)
    config['prototype_id']=experiment['id']
    config['timing']['duration_s']=3.0
    manifest['timing']=copy.deepcopy(config['timing'])
    manifest['system']['bodies']=[]
    manifest['initial_state']['body_states']={}
    manifest['initial_state']['participant_ids']=[]
    manifest['environment']['environment_body_ids']=[]
    manifest['environment']['environment_id']=spec['scene_id']
    material=dict(mass_kg=None,static_friction=.3,dynamic_friction=.2,restitution=.1,linear_damping=0.,angular_damping=0.)
    config['physics_profiles']['scene']=material
    floor_data=np.load(scene_path.parent/scene['groups']['room']['path'])
    floor=trimesh.Trimesh(floor_data['vertices'],floor_data['triangles'],process=False)
    def height(x,y):
        hits,_,_=floor.ray.intersects_location([[x,y,1.]],[[0,0,-1]],multiple_hits=False)
        if len(hits)!=1:raise ValueError('No source floor at requested object placement')
        return float(hits[0,2])
    for key,group in scene['groups'].items():
        geometry=dict(shape='mesh',static_scene_source=dict(path=str((scene_path.parent/group['path']).resolve()),
            sha256=group['sha256'],scene_manifest_path=str(scene_path.resolve()),scene_manifest_sha256=file_hash(scene_path),
            blend_sha256=scene['source_sha256'],source_objects=group['objects']))
        add_body(config,manifest,key,geometry,'scene',[0,0,0],kind='static',appearance='matte_gray',role='environment')
    geometry={k:copy.deepcopy(v) for k,v in asset.items() if k!='mass_kg'}
    config['physics_profiles']['subject']=dict(material,mass_kg=asset['mass_kg'])
    if geometry['shape']=='box':
        half_z=geometry['size_m'][2]/2;length=geometry['size_m'][0]
    elif geometry['shape']=='sphere':
        half_z=geometry['radius_m'];length=2*half_z
    else:
        raise ValueError('This batch accepts applicable primitive assets; external assets keep their existing separate bridge')
    x,y=experiment['subject_xy']
    if active:
        add_body(config,manifest,'load',geometry,'subject',[x,y,height(x,y)+half_z+.0002])
        px,py=experiment['pusher_xy']
        add_body(config,manifest,'pusher',dict(shape='box',size_m=[.08,.24,.24]),'actuator_1kg',
                 [px,py,height(px,py)+.14],appearance='matte_orange',role='actuator')
        manifest['system']['joints'][0]['upper_limit']=2.0
        command=manifest['control_program']['commands'][0]
        command.update(start_time_s=.2,end_time_s=2.6)
        command['target']['velocity_m_s']=condition['speed_m_s']
    else:
        config['physics_profiles']['subject'].update(static_friction=.05,dynamic_friction=.05,restitution=.8)
        palette=[[.95,.4,.08],[.1,.45,.85],[.2,.65,.45],[.65,.35,.75]]
        for i in range(experiment['count']):
            position=[x+i*(length+condition['gap_m']),y]
            config['appearance_profiles']['chain_'+str(i)]=dict(color=palette[i%len(palette)],roughness=.5,metallic=0.)
            add_body(config,manifest,'body'+str(i),geometry,'subject',
                [*position,height(*position)+half_z+.0002],[experiment['initial_speed_m_s'] if i==0 else 0,0,0],appearance='chain_'+str(i))
    config['camera_set']=dict(id=experiment['id']+'_fixed_pair',resolution=[640,480],focal_length_mm=35.,horizontal_aperture_mm=36.,
        cameras=[dict(id=name,position_m=pos,target_m=experiment['target']) for name,pos in zip(('front','rear'),experiment['cameras'])])
    config['native_scene_design']=dict(scene_id=spec['scene_id'],kind=experiment['kind'],condition=condition,
        source_blend=scene['source_blend'],source_blend_sha256=scene['source_sha256'],scene_export=spec['scene_export'],
        publication_scope=spec['publication_scope'],unchanged_environment=True,
        actual_floor_height_m=height(x,y),asset=asset)
    return config,manifest


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--spec',type=Path,default=ROOT/'configs/dataset/v0_2/native_scene_batch02.json')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--experiment')
    p.add_argument('--condition')
    a=p.parse_args();spec=read_json(a.spec)
    a.output.mkdir(parents=True,exist_ok=False)
    records=[]
    for ex in spec['experiments']:
        if a.experiment and ex['id']!=a.experiment:continue
        for aid in ex['assets']:
            for condition in ex['conditions']:
                if a.condition and condition['id']!=a.condition:continue
                name=ex['id']+'_'+aid+'_'+condition['id']
                root=a.output/name;root.mkdir()
                config,manifest=recipe(spec,ex,spec['assets'][aid],condition)
                write_json(root/'manifest.json',manifest)
                config['manifest_template']=str((root/'manifest.json').resolve())
                write_json(root/'config.json',config)
                episode=root/'episode'
                prepare(root/'config.json',episode)
                invoke('native_causal_rigid.py',episode,'simulation.log')
                package_physics(episode)
                records.append(dict(id=name,episode=str(episode.resolve()),experiment=ex['id'],asset=aid,condition=condition))
                print('NATIVE_SCENE_COMPLETED',name,flush=True)
    write_json(a.output/'index.json',dict(episodes=records))


if __name__=='__main__':main()

"""First in-situ representative: unchanged Blue Wall sideboard to original floor."""
import argparse
import copy
from pathlib import Path

import numpy as np
import trimesh

from .causal_runner import ROOT, prepare, invoke, package_physics
from .io import read_json, write_json, file_hash
from .phenomenon_recipes02 import build, add_body


def make_recipe(scene_path, speed=.8):
    scene = read_json(scene_path)
    meshes = {}
    for key, group in scene['groups'].items():
        with np.load(scene_path.parent/group['path']) as data:
            meshes[key] = trimesh.Trimesh(data['vertices'], data['triangles'], process=False)
    # Chosen measured clear band between the original plant/clock and magazines.
    # Subject size is explicit: 8 cm, not a silently scaled earlier asset.
    size = .08
    x, y = -.44, -.17
    grid = np.array([[x+dx,y+dy,1.18] for dx in np.linspace(-size/2,size/2,5)
                     for dy in np.linspace(-size/2,size/2,5)])
    hits, idx, _ = meshes['support'].ray.intersects_location(grid, np.tile([0,0,-1],(len(grid),1)), multiple_hits=False)
    if len(np.unique(idx)) != len(grid) or np.ptp(hits[:,2]) > .001:
        raise ValueError('Selected footprint lacks continuous level source support')
    top = float(hits[:,2].max())
    # Check real surrounding triangles, not only scene-object bounding boxes.
    other_hits, _, _ = meshes['surroundings'].ray.intersects_location(grid, np.tile([0,0,-1],(len(grid),1)),multiple_hits=True)
    if len(other_hits) and np.any((other_hits[:,2]>=top)&(other_hits[:,2]<=top+size+.002)):
        raise ValueError('Source decoration occupies initial footprint')
    config, manifest = build({'kind':'support_edge'}, {'speed_m_s':.8},
                             {'shape':'box','size_m':[size]*3,'mass_kg':700*size**3})
    config['prototype_id'] = 'in_situ_blue_wall_support_edge'
    config['timing']['duration_s'] = 2.0
    manifest['timing'] = copy.deepcopy(config['timing'])
    manifest['system']['bodies'] = []
    manifest['initial_state']['body_states'] = {}
    manifest['initial_state']['participant_ids'] = []
    manifest['environment']['environment_body_ids'] = []
    manifest['environment']['environment_id'] = 'blue_wall_original_geometry'
    config['physics_profiles']['ground'].update(static_friction=.3,dynamic_friction=.2,restitution=.1)
    config['physics_profiles']['subject'].update(static_friction=.3,dynamic_friction=.2,restitution=.1)
    for key,group in scene['groups'].items():
        source = dict(path=str((scene_path.parent/group['path']).resolve()), sha256=group['sha256'],
                      scene_manifest_path=str(scene_path.resolve()), scene_manifest_sha256=file_hash(scene_path),
                      blend_sha256=scene['source_sha256'], source_objects=group['objects'])
        add_body(config,manifest,key,dict(shape='mesh',static_scene_source=source),
                 'ground',[0,0,0],kind='static',appearance='matte_gray',role='environment')
    add_body(config,manifest,'subject',dict(shape='box',size_m=[size]*3),'subject',
             [x,y,top+size/2+.0002],[0,-speed,0],appearance='matte_orange')
    config['camera_set'] = dict(id='blue_wall_fixed_pair',resolution=[960,640],focal_length_mm=45.,horizontal_aperture_mm=36.,
        cameras=[dict(id='front',position_m=[-1.3,-1.9,1.5],target_m=[-.44,-.45,.48]),
                 dict(id='side',position_m=[-1.8,-.9,1.25],target_m=[-.44,-.45,.48])])
    config['scene_design'] = dict(primary='support loss at an existing edge, followed by fall and floor impact',
        material='rigid; assigned friction/restitution, not measured wood properties',
        source=str(scene_path),support_top_m=top,subject_size_m=size,subject_density_kg_m3=700,
        initial_speed_m_s=speed,initial_footprint_rays=len(grid),initial_support_height_range_m=[float(hits[:,2].min()),top],
        control='none; initial velocity only; environment remains fixed',
        observation='2 seconds, native state/contact every 1/240 second; short cache-only replay',
        variation_axes='none in first representative; later initial speed/orientation or suitable object geometry',
        environment_geometry='entire 194 evaluated source meshes, grouped into support/room/surroundings without decimation',
        license_evidence='/mnt/y/scene_research_20260916/downloads/blue_wall/info.txt',
        attribution='Blue Wall, Greg Zaal / Poly Haven, CC0; https://blog.polyhaven.com/blue-wall-scene-file/')
    return config, manifest


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--prepare-only',action='store_true')
    p.add_argument('--speed',type=float,default=.8)
    a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    config,manifest=make_recipe(a.scene.resolve(),a.speed)
    write_json(a.output/'manifest.json',manifest)
    config['manifest_template']=str((a.output/'manifest.json').resolve())
    write_json(a.output/'config.json',config)
    episode=a.output/'episode'
    prepare(a.output/'config.json',episode)
    if not a.prepare_only:
        invoke('native_causal_rigid.py',episode,'simulation.log')
        package_physics(episode)
        print('IN_SITU_COMPLETE',episode,flush=True)


if __name__=='__main__':main()

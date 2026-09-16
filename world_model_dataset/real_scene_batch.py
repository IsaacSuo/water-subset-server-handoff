"""Data-driven real-asset episodes on unchanged exported scene triangles."""
import argparse
import copy
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

from .causal_runner import ROOT, load_config, prepare, invoke, package_physics
from .io import read_json, write_json, file_hash
from .physical_asset_smoke import make_config
from .physical_asset_bridge import resolve_asset
from .phenomenon_recipes02 import add_body
from .local import idle


def recipe(spec, shot):
    c = load_config(ROOT/'configs/dataset/v0_2/c2_none_collision.json')
    m = read_json(ROOT/c['manifest_template'])
    c['initial_state_overrides'] = {}
    c.pop('body_profile_overrides', None)
    c['prototype_id'] = shot['id']
    c['timing'].update(duration_s=shot.get('duration_s', 3.0))
    c['numerics'].update(gpu_dynamics=True, position_iterations=32,
                         contact_offset_m=.001, speculative_ccd=False)
    m['timing'] = copy.deepcopy(c['timing'])
    m['system']['bodies'] = []
    m['initial_state']['participant_ids'] = []
    m['initial_state']['body_states'] = {}
    m['environment']['environment_body_ids'] = []
    m['environment']['environment_id'] = spec['scene_id']
    scene_path = ROOT/spec['scene_export']
    scene = read_json(scene_path)
    c['physics_profiles']['scene'] = dict(c['physics_profiles']['floor_reference'],
        static_friction=.4, dynamic_friction=.3, restitution=.1)
    meshes = {}
    for name, info in scene['groups'].items():
        path = scene_path.parent/info['path']
        source = dict(path=str(path.resolve()), sha256=info['sha256'],
            scene_manifest_path=str(scene_path.resolve()), scene_manifest_sha256=file_hash(scene_path),
            blend_sha256=scene['source_sha256'], source_objects=info['objects'])
        add_body(c,m,name,dict(shape='mesh',static_scene_source=source),'scene',[0,0,0],
                 kind='static',appearance='matte_gray',role='environment')
        with np.load(path) as z:
            meshes[name] = trimesh.Trimesh(z['vertices'],z['triangles'],process=False)
    placements = []
    for item in shot['bodies']:
        oid = item['id']
        ac = make_config(spec['exploration_root'],item['asset'],item['size_m'],
                         item.get('mass_kg'),item.get('density_kg_m3',700.),
                         library_root=Path(spec['exploration_root'])/spec['library'])
        g = ac['geometry_profiles']['exploration_asset']
        material = ac['physics_profiles']['asset_development']
        material.update(item.get('material',{}))
        mesh = resolve_asset(copy.deepcopy(g),copy.deepcopy(material),ROOT)
        rotation = Rotation.from_euler('xyz',item.get('rotation_deg',[90,0,0]),degrees=True)
        posed = rotation.apply(mesh.vertices)
        xy = item['xy_m']
        surface = meshes[item.get('support_group','support')]
        hits,_,_ = surface.ray.intersects_location([[*xy,item.get('ray_start_z_m',2.)]],[[0,0,-1]],multiple_hits=False)
        if len(hits)!=1:
            raise ValueError('No original support for '+oid)
        position = [*xy,float(hits[0,2]-posed[:,2].min()+item.get('clearance_m',.001))]
        c['physics_profiles'][oid] = material
        add_body(c,m,oid,g,oid,position,item.get('velocity_m_s',[0,0,0]))
        m['initial_state']['body_states'][oid].update(orientation_xyzw=rotation.as_quat().tolist(),
            angular_velocity_rad_s=item.get('angular_velocity_rad_s',[0,0,0]))
        placements.append(dict(id=oid,position_m=position,posed_bounds_m=[posed.min(0).tolist(),posed.max(0).tolist()],
                               support_hit_m=hits[0].tolist(),size_m=item['size_m']))
    c['camera_set'] = dict(id=shot['id']+'_camera',resolution=[640,400],focal_length_mm=45.,
        horizontal_aperture_mm=36.,cameras=[dict(id='main',**shot['camera'])])
    c['real_scene_design'] = dict(shot=shot,placements=placements,scene_export=str(scene_path),
        source_blend=scene['source_blend'],unchanged_environment=True,control='none',
        physical_assets='SDF, no convex substitution',scope=spec['scope'])
    return c,m


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--spec',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--ids',nargs='+')
    p.add_argument('--prepare-only',action='store_true')
    a=p.parse_args();spec=read_json(a.spec);a.output.mkdir(parents=True,exist_ok=True)
    for shot in spec['shots']:
        if a.ids and shot['id'] not in a.ids:continue
        episode=a.output/'episodes'/shot['id']
        if episode.exists():
            raise FileExistsError('Use a new shot ID for new physics: '+str(episode))
        c,m=recipe(spec,shot)
        folder=a.output/'recipes'/shot['id'];folder.mkdir(parents=True)
        write_json(folder/'manifest.json',m);c['manifest_template']=str((folder/'manifest.json').resolve())
        write_json(folder/'config.json',c)
        prepare(folder/'config.json',episode)
        print('PREPARED',shot['id'],flush=True)
        if not a.prepare_only:
            idle();invoke('native_causal_rigid.py',episode,'simulation.log');package_physics(episode)
            print('PHYSICS_COMPLETE',shot['id'],flush=True)


if __name__=='__main__':main()

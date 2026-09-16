"""Size-aware finite push recipes, preserving existing source meshes and episodes."""
import argparse
import copy
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh

from .causal_runner import ROOT, prepare, invoke, package_physics
from .physical_asset_smoke import make_config
from .physical_asset_bridge import resolve_asset
from .phenomenon_recipes02 import add_body
from .io import read_json, write_json, file_hash


def pad_layout(bounds, axis, position, floor_z, settings):
    """Use posed dimensions, not source-axis maximum extent or a universal pad."""
    lo, hi = np.asarray(bounds)
    extents = hi-lo
    along = {'X': 0, 'Y': 1}[axis]
    across = 1-along
    size = np.zeros(3)
    size[along] = settings['pad_thickness_fraction']*max(extents)
    size[across] = settings['pad_width_fraction']*extents[across]
    size[2] = settings['pad_height_fraction']*extents[2]
    center = np.array(position, dtype=float)
    center[along] += lo[along]-settings['approach_gap_fraction']*max(extents)-size[along]/2
    center[across] += (lo[across]+hi[across])/2
    center[2] = floor_z+settings['ground_clearance_m']+size[2]/2
    return size, center


def recipe(spec, item):
    explore = Path(spec['exploration_root'])
    library = explore/item['library'] if 'library' in item else None
    c = make_config(explore, item['asset'], item['size_m'], item.get('mass_kg'),
                    item.get('density_kg_m3', 700.), library_root=library)
    m = read_json(ROOT/c['manifest_template'])
    c['initial_state_overrides'] = {}
    c['body_profile_overrides'] = {}
    c.pop('place_on_floor')
    c['prototype_id'] = spec['id']+'_'+item['scene']
    c['timing'] = copy.deepcopy(spec['timing'])
    m['timing'] = copy.deepcopy(c['timing'])
    c['numerics'].update(position_iterations=32, velocity_iterations=4, speculative_ccd=True)
    body = next(b for b in m['system']['bodies'] if b['instance_id']=='load')
    body.update(geometry_id='exploration_asset', physics_profile_id='asset_development')
    # Resolve a temporary descriptor for dimensions/inertia; prepare resolves the
    # pinned package again into the immutable episode-local physics geometry.
    g = copy.deepcopy(c['geometry_profiles']['exploration_asset'])
    material = copy.deepcopy(c['physics_profiles']['asset_development'])
    mesh = resolve_asset(g, material, ROOT)
    orientation = item['orientation_xyzw']
    posed = Rotation.from_quat(orientation).apply(mesh.vertices)
    bounds = np.array([posed.min(0), posed.max(0)])
    xy = item.get('subject_xy', [0., 0.])
    floor_z = 0.
    if item['scene'] != 'canonical':
        scene_path = ROOT/spec['scenes'][item['scene']]['export']
        scene = read_json(scene_path)
        for group, info in scene['groups'].items():
            geometry = dict(shape='mesh', static_scene_source=dict(
                path=str((scene_path.parent/info['path']).resolve()), sha256=info['sha256'],
                scene_manifest_path=str(scene_path.resolve()), scene_manifest_sha256=file_hash(scene_path),
                blend_sha256=scene['source_sha256'], source_objects=info['objects']))
            add_body(c, m, group, geometry, 'fixture_reference', [0,0,0], kind='static',
                     appearance='matte_gray', role='environment')
        m['system']['bodies'] = [b for b in m['system']['bodies'] if b['instance_id']!='floor']
        del m['initial_state']['body_states']['floor']
        m['initial_state']['participant_ids'].remove('floor')
        m['environment']['environment_body_ids'].remove('floor')
        with np.load(scene_path.parent/scene['groups']['room']['path']) as data:
            floor = trimesh.Trimesh(data['vertices'], data['triangles'], process=False)
        hits, _, _ = floor.ray.intersects_location([[*xy, .15]], [[0,0,-1]], multiple_hits=False)
        if len(hits)!=1:
            raise ValueError('No original floor at requested initial position')
        floor_z = float(hits[0,2])
        m['environment']['environment_id'] = item['scene']+'_original_geometry'
    position = [*xy, floor_z-bounds[0,2]+.0005]
    m['initial_state']['body_states']['load'].update(position_m=position, orientation_xyzw=orientation)
    layout = dict(spec['layout'], **item.get('layout_overrides', {}))
    pad_size, pad_position = pad_layout(bounds, item['axis'], position, floor_z, layout)
    c['geometry_profiles']['pusher_pad']['size_m'] = pad_size.tolist()
    m['initial_state']['body_states']['pusher']['position_m'] = pad_position.tolist()
    # Mass remains a declared experimental parameter, not inferred material.
    # Uniform-density COM/inertia within the supplied solid geometry is retained.
    actuator_mass = max(.15, .5*material['mass_kg'])
    c['physics_profiles']['actuator_1kg']['mass_kg'] = actuator_mass
    acceleration = spec['control']['nominal_acceleration_m_s2']
    force = 1.25*(material['static_friction']*material['mass_kg']*9.81
                  + acceleration*(material['mass_kg']+actuator_mass))
    controller = m['control_program']['controllers'][0]
    controller['max_force_n'] = force
    command = m['control_program']['commands'][0]
    command.update(start_time_s=spec['control']['start_s'], end_time_s=spec['control']['end_s'])
    command['limits']['max_force_n'] = force
    command['target'].update(velocity_m_s=spec['control']['target_speed_m_s'],
                             velocity_gain_n_s_m=max(12., 20.*material['mass_kg']))
    m['system']['joints'][0].update(axis=item['axis'], upper_limit=1.5)
    c['camera_set'] = dict(id=item['id']+'_proportional_pair', resolution=[640,480],
        focal_length_mm=35., horizontal_aperture_mm=36., cameras=[])
    along = {'X': 0, 'Y': 1}[item['axis']]
    forward = np.eye(3)[along]; side = np.eye(3)[1-along]
    target = np.array([*xy, floor_z+.32*(bounds[1,2]-bounds[0,2])])+.30*forward
    distance = max(1.25, 3.*max(bounds[1]-bounds[0]))
    for name, sign in [('front', -1), ('rear', 1)]:
        eye = target+.30*forward+sign*distance*side+np.array([0,0,.8*distance])
        c['camera_set']['cameras'].append(dict(id=name, position_m=eye.tolist(), target_m=target.tolist()))
    c['proportion_design'] = dict(scope=spec['scope'], item=item, posed_bounds_m=bounds.tolist(),
        subject_extents_m=(bounds[1]-bounds[0]).tolist(), mass_kg=material['mass_kg'],
        pad_size_m=pad_size.tolist(), pad_mass_kg=actuator_mass, force_limit_n=force, layout=layout,
        force_rule='1.25*(mu_static*m*g + nominal_acceleration*(subject_mass+actuator_mass)); design estimate, not measured resistance',
        source_scene=spec.get('scenes', {}).get(item['scene']), floor_z_m=floor_z)
    return c, m


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--spec', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--ids', nargs='+')
    p.add_argument('--prepare-only', action='store_true')
    p.add_argument('--resume', action='store_true', help='Reuse completed physical records; never rerun a partially simulated episode')
    p.add_argument('--observe', action='store_true', help='Render existing physical caches only')
    a = p.parse_args(); spec = read_json(a.spec)
    if a.observe:
        from .causal_observe import package_observations
        for item in spec['episodes']:
            if a.ids and item['id'] not in a.ids: continue
            episode = a.output/'episodes'/item['id']
            if not (episode/'episode.physics.json').exists():
                raise ValueError('Physical cache not complete: '+item['id'])
            resolved = read_json(episode/'resolved_inputs.json')
            if not (episode/'observations/index.json').exists():
                profile = dict(camera_set=resolved['camera_set'])
                if item['scene'] != 'canonical':
                    profile['camera_lights'] = dict(intensity=30000., radius_m=.15)
                    overrides = spec['scenes'][item['scene']].get('observation_camera_overrides', {})
                    for camera in profile['camera_set']['cameras']:
                        camera.update(overrides.get(camera['id'], {}))
                if not (episode/'observation_profile.json').exists():
                    write_json(episode/'observation_profile.json', profile)
                invoke('causal_render.py', episode, 'observation.log')
            index = read_json(episode/'observations/index.json')
            if not index['complete'] or index['source_state_sha256'] != file_hash(episode/'body_state_trace.jsonl'):
                raise ValueError('Incomplete or mismatched observation cache: '+item['id'])
            if not (episode/'episode.observed.json').exists(): package_observations(episode)
            print('PROPORTIONAL_OBSERVATION_COMPLETE', item['id'], flush=True)
        return
    a.output.mkdir(parents=True, exist_ok=a.resume)
    records = []
    for item in spec['episodes']:
        if a.ids and item['id'] not in a.ids: continue
        episode = a.output/'episodes'/item['id']
        if a.resume and episode.exists():
            c = read_json(episode/'input_config.json')
            if c['proportion_design']['item'] != item:
                raise ValueError('Existing episode has different parameters; use a new episode ID')
            if not (episode/'episode.physics.json').exists() and not a.prepare_only:
                if not (episode/'native_report.json').exists():
                    raise RuntimeError('Incomplete native run: inspect its process/log rather than automatically rerun')
                package_physics(episode)
        else:
            c, m = recipe(spec, item)
            folder = a.output/'recipes'/item['id']; folder.mkdir(parents=True)
            write_json(folder/'manifest.json', m)
            c['manifest_template'] = str((folder/'manifest.json').resolve())
            write_json(folder/'config.json', c)
            prepare(folder/'config.json', episode)
            if not a.prepare_only:
                invoke('native_causal_rigid.py', episode, 'simulation.log')
                package_physics(episode)
        records.append(dict(id=item['id'], asset=item['asset'], episode=str(episode.resolve()),
                            status='prepared' if a.prepare_only else 'physics_completed_pending_direct_review'))
        progress = a.output/f'progress_{len(records):02d}.json'
        if not progress.exists(): write_json(progress, dict(episodes=records))
        print('PROPORTIONAL_EPISODE_COMPLETE', item['id'], c['proportion_design'], flush=True)
    index = dict(episodes=records, scope=spec['scope'])
    if (a.output/'index.json').exists():
        if read_json(a.output/'index.json') != index:
            raise ValueError('Existing batch index has a different selection; use a new output directory')
    else:
        write_json(a.output/'index.json', index)


if __name__ == '__main__': main()

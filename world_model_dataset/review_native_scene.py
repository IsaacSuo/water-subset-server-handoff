"""Direct post-run notes for the two Blue Wall development trajectories."""
import argparse
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh

from .causal_loader import CausalEpisode
from .io import read_json, write_json, file_hash


def review(root):
    episode=CausalEpisode(root,require_complete=False)
    rows=list(episode.states())
    contacts=list(episode.contacts())
    resolved=episode.resolved_inputs()
    original=rows[0]['body_states']
    unchanged=True
    same_mesh={}
    for oid,b in resolved['bodies'].items():
        g=b['geometry']
        if 'static_scene_source' not in g:
            continue
        unchanged &= all(row['body_states'][oid]==original[oid] for row in rows)
        with np.load(root/g['mesh']['path']) as packed, np.load(g['static_scene_source']['path']) as source:
            same_mesh[oid]=all(np.array_equal(packed[k],source[k]) for k in ('vertices','triangles'))
    states=[r['body_states']['subject'] for r in rows]
    size=resolved['bodies']['subject']['geometry']['size_m']
    local=trimesh.creation.box(size).vertices
    world=np.asarray([Rotation.from_quat(s['orientation_xyzw']).apply(local)+s['position_m'] for s in states])
    # This selected route is entirely on the measured Room floor patch z=0.
    # Verify that against the actual triangles at every sampled corner XY.
    g=resolved['bodies']['room']['geometry']
    with np.load(root/g['mesh']['path']) as data:
        room=trimesh.Trimesh(data['vertices'],data['triangles'],process=False)
    ray=world.reshape(-1,3).copy()
    ray[:,2]=.2
    hit,indices,_=room.ray.intersects_location(ray,np.tile([0,0,-1],(len(ray),1)),multiple_hits=False)
    flat_floor_verified=len(np.unique(indices))==len(ray) and np.max(np.abs(hit[:,2]))<1e-6
    penetration=np.maximum(0,-world[:,:,2].min(1)) if flat_floor_verified else None
    by_actor={}
    for oid in ('support','room','surroundings'):
        cs=[c for c in contacts if oid in c['actor_ids']]
        by_actor[oid]=dict(points=len(cs),first_s=cs[0]['time_s'] if cs else None,
            last_s=cs[-1]['time_s'] if cs else None,
            minimum_reported_separation_m=min((c['separation_m'] for c in cs),default=None))
    return dict(episode=str(root),state_sha256=file_hash(root/'body_state_trace.jsonl'),
        states=len(rows),duration_s=rows[-1]['time_s'],contacts=by_actor,
        environment_states_constant=bool(unchanged),scene_triangles_identical_to_export=same_mesh,
        final_position_m=states[-1]['position_m'],final_speed_m_s=float(np.linalg.norm(states[-1]['linear_velocity_m_s'])),
        peak_speed_m_s=float(max(np.linalg.norm(s['linear_velocity_m_s']) for s in states)),
        maximum_rotation_from_initial_deg=float(max(Rotation.from_quat(s['orientation_xyzw']).magnitude() for s in states)*180/np.pi),
        route_floor_ray_verified=bool(flat_floor_verified),
        max_sampled_floor_penetration_m=float(penetration.max()) if penetration is not None else None,
        floor_penetration_peak_time_s=rows[int(penetration.argmax())]['time_s'] if penetration is not None else None,
        floor_penetration_steps_over_2mm=int((penetration>.002).sum()) if penetration is not None else None,
        post_t0_operations=episode.manifest['implementation']['post_t0_operations'],
        limitations=['development representative, not high-accuracy impact certification',
            'static environment mass response not modelled',
            'assigned contact material parameters are not measured real scene materials',
            'short Blender replay is not formal multimodal sensor delivery'])


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,required=True)
    a=p.parse_args()
    scene=read_json(a.root/'scene_geometry/scene.json')
    record=dict(source_blend_sha256=scene['source_sha256'],
        source_unchanged=file_hash(scene['source_blend'])==scene['source_sha256'],
        source_objects=sum(len(g['objects']) for g in scene['groups'].values()),
        triangles=sum(g['triangles'] for g in scene['groups'].values()),
        episodes=[review(a.root/folder/'episode') for folder in ('representative','edge_speed12')])
    write_json(a.root/'cache_review.json',record)
    print(record)


if __name__=='__main__':main()

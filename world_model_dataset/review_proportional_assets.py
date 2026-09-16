"""Direct post-run diagnostics for size-aware push examples; no pre-run gate."""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from threadpoolctl import threadpool_limits

from .io import read_json, write_json, file_hash


def review(episode):
    rows = [json.loads(line) for line in (episode/'body_state_trace.jsonl').read_text().splitlines()]
    contacts = [json.loads(line) for line in (episode/'contacts.jsonl').read_text().splitlines()]
    efforts = [json.loads(line) for line in (episode/'actuator_effort_trace.jsonl').read_text().splitlines()]
    resolved = read_json(episode/'resolved_inputs.json')
    design = read_json(episode/'input_config.json')['proportion_design']
    states = [row['body_states']['load'] for row in rows]
    positions = np.array([s['position_m'] for s in states])
    velocities = np.array([s['linear_velocity_m_s'] for s in states])
    if not np.isfinite(positions).all() or not np.isfinite(velocities).all():
        raise ValueError('Nonfinite state in '+str(episode))
    pairs = {}
    for c in contacts:
        pair = '/'.join(sorted(c['actor_ids']))
        item = pairs.setdefault(pair, dict(points=0, first_nonzero_impulse_s=None, minimum_reported_separation_m=1e9))
        item['points'] += 1
        item['minimum_reported_separation_m'] = min(item['minimum_reported_separation_m'], c['separation_m'])
        if item['first_nonzero_impulse_s'] is None and np.linalg.norm(c['impulse_ns']) > 0:
            item['first_nonzero_impulse_s'] = c['time_s']
    with np.load(episode/resolved['bodies']['load']['geometry']['mesh']['path']) as data:
        vertices = data['vertices'].astype(float)
    # Bare-floor check for canonical scenes only: the bathroom has a real rug
    # and other supports; one infinite plane would be misleading there.
    minimum_z = None
    if design['item']['scene'] == 'canonical':
        heights = []
        with threadpool_limits(limits=1):
            for s in states:
                axis = Rotation.from_quat(s['orientation_xyzw']).as_matrix()[2]
                heights.append(float(np.min(vertices@axis)+s['position_m'][2]))
        minimum_z = min(heights)
    active = [e for e in efforts if e['command_active'] and e['time_s'] >= 2.4]
    initial, final = rows[0]['body_states'], rows[-1]['body_states']
    return dict(episode=str(episode), state_sha256=file_hash(episode/'body_state_trace.jsonl'),
        states=len(rows), duration_s=rows[-1]['time_s'], proportion_design=design,
        subject_displacement_m=(positions[-1]-positions[0]).tolist(),
        subject_final_position_m=positions[-1].tolist(), maximum_speed_m_s=float(np.linalg.norm(velocities,axis=1).max()),
        final_speed_m_s=float(np.linalg.norm(velocities[-1])), canonical_source_mesh_minimum_z_m=minimum_z,
        contacts=pairs, pusher_displacement_m=(np.array(final['pusher']['position_m'])-initial['pusher']['position_m']).tolist(),
        late_active_saturation_fraction=float(np.mean([e['saturated'] for e in active])) if active else None,
        applied_force_max_n=max(abs(e['applied_force_n']) for e in efforts),
        all_environment_poses_fixed=all(row['body_states'][oid]['position_m']==initial[oid]['position_m']
            and row['body_states'][oid]['orientation_xyzw']==initial[oid]['orientation_xyzw']
            for oid in initial if oid not in ('pusher','load') for row in rows),
        observation_index_exists=(episode/'observations/index.json').exists(),
        material_face_warning='getMaterialFromInternalFaceIndex' in (episode/'simulation.log').read_text(errors='replace'))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    results={}
    for episode in sorted(a.root.glob('*/episodes/*')):
        if (episode/'episode.physics.json').exists():
            results[episode.name]=review(episode)
            print('REVIEWED',episode.name,results[episode.name]['subject_displacement_m'],flush=True)
    write_json(a.output,dict(scope='direct sampled cache diagnostics; not automatic asset admission or convergence certification',episodes=results))


if __name__=='__main__': main()

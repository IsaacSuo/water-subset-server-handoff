"""Bounded local-motion review of a cloth cache against its own old configuration.

This documents fixed-configuration behaviour, not convergence or real-material
accuracy. Regions follow identities selected from the reference final state.
"""
import argparse
from pathlib import Path

import numpy as np

from .causal_loader import open_episode
from .io import file_hash, write_json


def load_surface(path):
    ep = open_episode(path, require_complete=False)
    carrier = next(b['instance_id'] for b in ep.manifest['system']['bodies']
                   if b['physics_kind'] == 'surface')
    rows = list(ep.soft_geometries(carrier))
    return ep, np.array([r['time_s'] for r, _ in rows]), np.array([
        g['surface_world_m'] for _, g in rows]), np.array([
        g['surface_nodal_velocities_m_s'] for _, g in rows]), rows[0][1]['surface_triangles']


def window(t, x, v, mask, start, end):
    selected = (t >= start - 1e-8) & (t <= end + 1e-8)
    points = x[selected][:, mask]
    speed = np.linalg.norm(v[selected][:, mask], axis=-1)
    return dict(samples=int(selected.sum()), rms_speed_m_s=float(np.sqrt(np.mean(speed ** 2))),
                peak_speed_m_s=float(speed.max()),
                max_frame_displacement_m=float(np.linalg.norm(np.diff(points, axis=0), axis=-1).max()),
                max_z_range_m=float(np.ptp(points[:, :, 2], axis=0).max()))


def review(current, reference, output, edge_x):
    ep, t, x, v, faces = load_surface(current)
    old, ot, ox, ov, of = load_surface(reference)
    if not np.array_equal(faces, of) or not np.array_equal(x[0], ox[0]):
        raise ValueError('Reference must have identical initial points and topology')
    if not np.allclose(np.diff(t), 1 / 60) or not np.allclose(np.diff(ot), 1 / 60):
        raise ValueError('This review compares native 60 Hz state samples only')
    if t[-1] < 5 or ot[-1] < 2:
        raise ValueError('Need five-second candidate and two-second reference')
    ref = ox[-1]
    masks = dict(all=np.ones(len(ref), bool),
                 edge=(ref[:, 0] > edge_x - .05) & (ref[:, 0] < edge_x + .04) & (ref[:, 2] > .78),
                 hanging=ref[:, 2] < .78)
    result = dict(format='cloth-stability-review/1',
                  source_states_sha256=ep.manifest['trajectory']['states']['sha256'],
                  reference_states_sha256=old.manifest['trajectory']['states']['sha256'],
                  initial_points_and_topology_unchanged=True,
                  region_definition=dict(edge_x_m=edge_x, selection='reference final identities',
                                         edge_x_offsets_m=[-.05, .04], edge_z_min_m=.78,
                                         hanging_z_max_m=.78), regions={}, training_admission=False,
                  limits='Native 60 Hz local-motion check; no force, convergence, calibration or continuous collision claim')
    for name, mask in masks.items():
        if not mask.any():
            raise ValueError('Empty review region: ' + name)
        result['regions'][name] = dict(node_ids=np.flatnonzero(mask).tolist(),
            old_1_5_to_2=window(ot, ox, ov, mask, 1.5, 2),
            new_1_5_to_2=window(t, x, v, mask, 1.5, 2),
            new_2_5_to_3=window(t, x, v, mask, 2.5, 3),
            new_4_5_to_5=window(t, x, v, mask, 4.5, 5))
    # Explicit, narrow local-jitter criteria; never called a material admission.
    tail = result['regions']['all']['new_4_5_to_5']
    checks = dict(finite=bool(np.isfinite(x).all() and np.isfinite(v).all()),
                  tail_rms_under_1cm_s=tail['rms_speed_m_s'] < .01,
                  tail_peak_under_20cm_s=tail['peak_speed_m_s'] < .2,
                  tail_frame_motion_under_1mm=tail['max_frame_displacement_m'] < .001)
    result['checks'] = checks
    result['status'] = 'local_stability_checked' if all(checks.values()) else 'needs_stability_review'
    write_json(output, result)
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--episode', type=Path, required=True)
    p.add_argument('--reference', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--edge-x', type=float, required=True)
    a = p.parse_args()
    result = review(a.episode, a.reference, a.output, a.edge_x)
    print(result['status'], result['regions']['all']['new_4_5_to_5'], file_hash(a.output))


if __name__ == '__main__':
    main()

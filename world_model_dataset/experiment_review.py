"""Representation-preserving record checks and measured outcome diagnostics."""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from .phenomenon_collect import audit


def verify_alignment(ep, timing, observed=False):
    rows = list(ep.states())
    hz, saved, duration = timing['physics_hz'], timing['state_hz'], timing['duration_s']
    expected = np.arange(round(duration * saved) + 1) / saved
    times = np.array([r['time_s'] for r in rows])
    steps = np.array([r['physics_step'] for r in rows])
    if len(rows) != len(expected) or not np.allclose(times, expected, atol=1e-8, rtol=0):
        raise ValueError('State times do not match requested duration/state rate')
    if not np.array_equal(steps, np.arange(len(rows)) * (hz // saved)):
        raise ValueError('State physics_step does not match world time')
    by_step = {r['physics_step']: r for r in rows}
    counts = {'states': len(rows), 'commands': 0, 'actuator_states': 0, 'efforts': 0, 'observations': 0}
    def trace(iterator, name):
        previous = -1
        for row in iterator:
            step = row['physics_step']
            if step <= previous or step < 0 or step > round(duration * hz):
                raise ValueError('Invalid ' + name + ' step order')
            if abs(row['time_s'] - step / hz) > 1e-8:
                raise ValueError(name + ' time/physics-step mismatch')
            previous = step
            counts[name] += 1
            yield row
    commands = list(trace(ep.controls(), 'commands'))
    for ctrl in ep.manifest['control_program']['controllers']:
        cid = ctrl['controller_id']
        actual = list(trace(ep.actuator_states(cid), 'actuator_states'))
        efforts = list(trace(ep.actuator_efforts(cid), 'efforts'))
        if len(commands) != round(duration * hz) or len(efforts) != len(commands):
            raise ValueError('Missing step commands/applied efforts')
        actual_steps = [s['physics_step'] for s in actual]
        saved_steps = list(by_step)
        physics_steps = list(range(round(duration * hz)))
        if actual_steps not in (saved_steps, physics_steps):
            raise ValueError('Actual actuator trace must cover saved states or every physical input step')
        for sample in actual:
            state = by_step.get(sample['physics_step'])
            if state is None:
                continue  # Retain native multirate records; never interpolate.
            oid = sample.get('actuator_instance_id', ctrl.get('actuator_instance_id'))
            if oid is None or oid not in state['body_states']:
                raise ValueError('Actuator references unknown participant')
            for field in ('position_m', 'orientation_xyzw', 'linear_velocity_m_s', 'angular_velocity_rad_s'):
                if field in sample and not np.allclose(sample[field], state['body_states'][oid][field], atol=1e-7, rtol=0):
                    raise ValueError('Actual actuator trace differs from native body state: ' + field)
    # audit() separately streams every native geometry with hash/time/finite
    # checks; avoid rereading every NPZ at each resumable stage transition.
    if observed:
        for row, arrays in ep.observations():
            state = by_step.get(row['physics_step'])
            if state is None or abs(state['time_s'] - row['time_s']) > 1e-9:
                raise ValueError('Observation is not associated with a saved state')
            counts['observations'] += 1
    return dict(counts=counts, start_s=float(times[0]), end_s=float(times[-1]),
                physics_hz=hz, state_hz=saved, interpolation=False,
                semantics='commands/efforts are step inputs; terminal state has no following command')


def beam_outcomes(ep):
    r = ep.resolved_inputs(); cfg = r['config']
    with np.load(ep.record_path(r['raw_native'])) as z:
        a = {k: z[k] for k in z.files}
    t = a['time']; schedule = np.asarray(cfg['plate']['schedule'])
    commands = list(ep.controls())
    command_t = np.array([r['time_s'] for r in commands])
    target_z = np.array([r['target_position_m'][2] for r in commands])
    if not np.allclose(target_z, np.interp(command_t, schedule[:,0], schedule[:,1]), atol=1e-9, rtol=0):
        raise ValueError('Actual command trace does not implement the requested schedule')
    low = schedule[:, 1].min(); high = schedule[0, 1]
    phases = [(p[0], q[0]) for p, q in zip(schedule[:-1], schedule[1:]) if p[1] == q[1] == low]
    if low == high:
        return dict(intent='high hold control', actual_plate_z_range_m=[float(a['plate_pose'][:,2].min()), float(a['plate_pose'][:,2].max())])
    half = np.asarray(cfg['plate']['size_m']) / 2
    delta = a['collision_world'] - a['plate_pose'][:, None, :3]
    local = np.einsum('fvi,fij->fvj', delta, Rotation.from_quat(a['plate_pose'][:,3:]).as_matrix())
    q = abs(local) - half
    signed = np.linalg.norm(np.maximum(q, 0), axis=-1) + np.minimum(q.max(-1), 0)
    bottom = (q.argmax(-1) == 2) & (local[:,:,2] < 0) & (signed <= .004)
    hold = np.zeros(len(t), bool)
    for start, end in phases:
        hold |= (t >= start) & (t < end)
    withdrawn = t >= schedule[-2,0]
    tip = a['rest_x'][:,0] > a['rest_x'][:,0].max() - .02
    z_tip = a['x'][:,tip,2].mean(1)
    return dict(intent='load, hold, withdraw', hold_intervals_s=phases,
                actual_plate_downward_travel_m=float(high - a['plate_pose'][:,2].min()),
                hold_every_sample_bottom_proximity=bool(hold.any() and bottom[hold].any(1).all()),
                actual_postwithdraw_minimum_gap_m=float(signed[withdrawn].min()) if withdrawn.any() else None,
                physical_withdrawal_observed=bool(withdrawn.any() and signed[withdrawn].min() > .004),
                tip_z_initial_hold_final_m=[float(z_tip[0]), float(z_tip[hold].mean()) if hold.any() else None, float(z_tip[-1])],
                attachment_maximum_displacement_m=float(np.linalg.norm(a['x'][:,a['attachment_node_ids']] - a['rest_x'][a['attachment_node_ids']], axis=-1).max()),
                contact_force='unavailable', attachment_reaction='unavailable',
                limits='sampled native collision-node distances to actual plate; no continuous-contact or force claim')


def review(ep, doc):
    result = audit(ep, doc['phenomenon'])
    result['alignment'] = verify_alignment(ep, doc['timing'])
    result['experiment_intent'] = doc['phenomenon']
    result['outcome_filtering'] = False
    result['training_admission'] = False
    if doc['backend']['kind'] == 'beam':
        result['measured_outcome'] = beam_outcomes(ep)
    elif doc['backend']['kind'] == 'cloth':
        rows = list(ep.geometries('cloth'))
        tail = [(r, g) for r, g in rows if r['time_s'] >= doc['timing']['duration_s'] - .5]
        positions = np.asarray([g['surface_world_m'] for _, g in tail])
        velocity = np.asarray([g['surface_nodal_velocities_m_s'] for _, g in tail])
        result['measured_outcome'] = dict(
            final_half_second_maximum_frame_displacement_m=float(np.linalg.norm(np.diff(positions, axis=0), axis=-1).max()) if len(positions)>1 else None,
            final_half_second_velocity_rms_m_s=float(np.sqrt(np.mean(np.sum(velocity**2, axis=-1)))),
            final_half_second_peak_velocity_m_s=float(np.linalg.norm(velocity, axis=-1).max()),
            limits='fixed-configuration local settling diagnostic, not convergence/material calibration')
    return result

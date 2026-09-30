"""Offline, read-only impact checks for a completed pressure-projection run.

Only files the simulation already wrote are read: pressure_substeps.jsonl,
probe_report.json and capture/frame_*.npz.  Nothing is simulated and no
physical pass/fail threshold is added; the output is a set of indicators.

Three questions are separated:

1. Substeps: does the density-projection velocity increment scale like 1/dt,
   i.e. is it a position correction whose size in metres is what stays fixed?
2. Frame caches: do particles end up with more mechanical head than any fluid
   had before the impact, and does total mechanical energy ever rise?
3. Report rows: how large is the realised post-advection wall density error
   compared with the tolerance the projection converged to?
"""

import argparse
import collections
import csv
import json
from pathlib import Path

import numpy as np


GRAVITY = 9.81


def write_json(path, value):
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(value, indent=2, ensure_ascii=False))
    temp.replace(path)


def write_csv(path, rows):
    fields = []
    for row in rows:
        fields.extend(key for key in row if key not in fields)
    with path.open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def distribution(values):
    values = np.asarray([value for value in values if value is not None], dtype=np.float64)
    if not len(values):
        return dict(count=0, median=None, p95=None, maximum=None)
    return dict(count=int(len(values)), median=float(np.median(values)),
        p95=float(np.percentile(values, 95)), maximum=float(values.max()))


def log_slope(dt, increment):
    """Least-squares exponent b in increment ~ dt**b; -1 is a fixed displacement."""
    dt = np.asarray(dt, dtype=np.float64)
    increment = np.asarray(increment, dtype=np.float64)
    mask = np.isfinite(dt) & np.isfinite(increment) & (dt > 0.) & (increment > 0.)
    if np.count_nonzero(mask) < 3 or np.ptp(np.log10(dt[mask])) == 0.:
        return dict(samples=int(np.count_nonzero(mask)), exponent=None, correlation=None)
    x = np.log10(dt[mask])
    y = np.log10(increment[mask])
    exponent, _ = np.polyfit(x, y, 1)
    correlation = float(np.corrcoef(x, y)[0, 1]) if np.ptp(y) > 0. else None
    return dict(samples=int(len(x)), exponent=float(exponent), correlation=correlation)


def classify_trial(trial):
    if trial['accepted']:
        return 'accepted'
    converged = all(status['converged'] for status in trial['projection_status'].values())
    safe = 'cfl_safe' if trial['cfl_safe'] else 'cfl_unsafe'
    return f"rejected_{'converged' if converged else 'unconverged'}_{safe}"


def analyze_substeps(path, window, spacing):
    rows = []
    trials = collections.Counter()
    previous_dt = None
    with path.open() as stream:
        for line in stream:
            if not line.strip():
                continue
            record = json.loads(line)
            metrics = record['metrics']
            dt = float(metrics['dt'])
            if window[0]-1.e-9 <= record['end_time_s'] <= window[1]+1.e-9:
                statistics = metrics.get('volume_projection_statistics', {})
                density = statistics.get('density', {})
                divergence = statistics.get('divergence', {})
                normal = (metrics.get('collision_verification') or {}).get(
                    'maximum_abs_normal_pressure_dv_m_s', {})
                density_dv = density.get('maximum_velocity_increment_m_s')
                pre_speed = metrics.get('pre_step_characteristic_speed_m_s')
                rows.append(dict(end_time_s=record['end_time_s'], dt_s=dt, previous_dt_s=previous_dt,
                    dt_shrank=bool(previous_dt is not None and dt < previous_dt*(1.-1.e-6)),
                    retries=metrics.get('retries'), pressure_retries=metrics.get('pressure_retries'),
                    density_iterations=metrics.get('density_iterations'),
                    divergence_iterations=metrics.get('divergence_iterations'),
                    pre_step_speed_m_s=pre_speed,
                    post_step_speed_m_s=metrics.get('post_step_characteristic_speed_m_s'),
                    density_dv_max_m_s=density_dv,
                    density_implied_displacement_m=None if density_dv is None else density_dv*dt,
                    density_implied_displacement_spacings=(None if density_dv is None
                        else density_dv*dt/spacing),
                    density_dv_over_pre_step_speed=(density_dv/pre_speed
                        if density_dv is not None and pre_speed else None),
                    density_boundary_acceleration_max_m_s2=density.get('maximum_boundary_acceleration_m_s2'),
                    density_fluid_acceleration_max_m_s2=density.get('maximum_fluid_acceleration_m_s2'),
                    divergence_dv_max_m_s=divergence.get('maximum_velocity_increment_m_s'),
                    receiver_normal_density_boundary_dv_m_s=normal.get('density_boundary'),
                    receiver_normal_density_fluid_dv_m_s=normal.get('density_fluid'),
                    receiver_normal_divergence_boundary_dv_m_s=normal.get('divergence_boundary'),
                    receiver_normal_divergence_fluid_dv_m_s=normal.get('divergence_fluid')))
                trials.update(classify_trial(trial) for trial in record['trials'])
            previous_dt = dt
    if not rows:
        return rows, dict(accepted_substeps=0)

    def column(name, selected=None):
        return [row[name] for row in (rows if selected is None else selected)]

    dt = np.asarray(column('dt_s'))
    shrank = [row for row in rows if row['dt_shrank']]
    steady = [row for row in rows if not row['dt_shrank']]
    # Octaves below the largest accepted step in the window.
    octave = np.floor(np.log2(dt.max()/dt)+1.e-9).astype(int)
    bins = []
    for value in sorted(set(octave.tolist())):
        selected = [row for row, item in zip(rows, octave) if item == value]
        bins.append(dict(octaves_below_largest_dt=value,
            dt_s_min=float(min(column('dt_s', selected))), dt_s_max=float(max(column('dt_s', selected))),
            density_dv_m_s=distribution(column('density_dv_max_m_s', selected)),
            density_implied_displacement_m=distribution(column('density_implied_displacement_m', selected))))
    ratios = [value for value in column('density_dv_over_pre_step_speed') if value is not None]
    summary = dict(accepted_substeps=len(rows), trials=dict(trials),
        dt_s=distribution(dt),
        scaling_exponent=dict(
            definition='b in max velocity increment ~ dt**b over accepted substeps; about -1 means a fixed '
                'displacement is imposed whatever the step, about 0 means a step-independent velocity change',
            density_all_fluid=log_slope(dt, [value or 0. for value in column('density_dv_max_m_s')]),
            receiver_normal_density_boundary=log_slope(dt,
                [value or 0. for value in column('receiver_normal_density_boundary_dv_m_s')]),
            receiver_normal_density_fluid=log_slope(dt,
                [value or 0. for value in column('receiver_normal_density_fluid_dv_m_s')]),
            divergence_all_fluid=log_slope(dt, [value or 0. for value in column('divergence_dv_max_m_s')])),
        after_step_reduction=dict(
            definition='accepted substeps whose dt is smaller than the previous accepted dt, against all others',
            reduced=dict(density_dv_m_s=distribution(column('density_dv_max_m_s', shrank)),
                density_implied_displacement_m=distribution(column('density_implied_displacement_m', shrank))),
            not_reduced=dict(density_dv_m_s=distribution(column('density_dv_max_m_s', steady)),
                density_implied_displacement_m=distribution(column('density_implied_displacement_m', steady)))),
        by_dt_octave=bins,
        density_dv_over_pre_step_speed=dict(
            definition='largest density-projection velocity increment divided by the fastest fluid or rigid '
                'surface speed before the step; above one the projection alone outruns everything present',
            distribution=distribution(ratios),
            substeps_above_one=int(sum(value > 1. for value in ratios))),
        limitations='Domain maxima are not restricted to the receiver; the receiver-normal columns are. '
            'Only the accepted trial of each substep carries velocity statistics.')
    return rows, summary


def analyze_report(report, window):
    rows = []
    for row in report['rows']:
        t = row['action_seconds']
        if not window[0]-1.e-8 <= t <= window[1]+1.e-8:
            continue
        integrator = row.get('integrator') or {}
        energy = row['energy']
        tolerance = ((integrator.get('pressure_projection_status') or {}).get('density') or {}).get('tolerance')
        error = integrator.get('near_boundary_max_density_error')
        item = dict(time_s=t, last_dt_s=integrator.get('dt'),
            kinetic_energy_j=energy['kinetic_energy_j'],
            gravitational_potential_energy_j=energy['gravitational_potential_energy_j'],
            mechanical_energy_j=energy['mechanical_energy_j'],
            prescribed_motor_work_to_fluid_j=energy['prescribed_motor_work_to_fluid_j'],
            mechanical_minus_motor_work_j=energy['mechanical_energy_j']-energy['prescribed_motor_work_to_fluid_j'],
            realised_near_boundary_density_error=error, density_projection_tolerance=tolerance,
            realised_error_over_tolerance=(error/tolerance if error is not None and tolerance else None),
            near_boundary_divergence_per_s=integrator.get('near_boundary_max_divergence_per_s'))
        wall = row.get('wall_density')
        if wall:
            item['fluid_neighbors_below_20'] = wall['fluid_neighbors_below_20']
            for name, region in wall['regions'].items():
                for key, value in region.items():
                    item[f'{name}_{key}'] = value
        rows.append(item)
    if not rows:
        return rows, dict(frames=0)
    balance = np.asarray([row['mechanical_minus_motor_work_j'] for row in rows])
    steps = np.diff(balance)
    rises = [dict(from_time_s=rows[index]['time_s'], to_time_s=rows[index+1]['time_s'], increase_j=float(value))
        for index, value in enumerate(steps) if value > 0.]
    keys = sorted({key for row in rows for key in row})
    penetrated = {key: max(row.get(key) or 0 for row in rows)
        for key in keys if key.endswith('_penetrated_count')}
    p95 = {key: max((row[key] for row in rows if row.get(key) is not None), default=None)
        for key in keys if key.endswith('_density_ratio_p95')}
    summary = dict(frames=len(rows),
        realised_wall_density=dict(
            definition='largest rho/rho0-1 over boundary-contact particles after advection, sampled at the '
                'last substep of each frame, against the tolerance the linearised projection met',
            error=distribution([row['realised_near_boundary_density_error'] for row in rows]),
            error_over_tolerance=distribution([row['realised_error_over_tolerance'] for row in rows]),
            first_layer_density_ratio_p95_maximum=p95, penetrated_count_maximum=penetrated),
        energy=dict(
            definition='fluid kinetic plus gravitational energy minus reported prescribed motor work',
            net_change_j=float(balance[-1]-balance[0]),
            frame_to_frame_increases=rises,
            increase_total_j=float(sum(item['increase_j'] for item in rises)),
            kinetic_energy_j_maximum=float(max(row['kinetic_energy_j'] for row in rows)),
            limitations=report.get('energy_accounting', {}).get('limitations')))
    return rows, summary


def neighbor_counts(positions, radius):
    from scipy.spatial import cKDTree
    return cKDTree(positions).query_ball_point(positions, np.nextafter(radius, 0.),
        workers=-1, return_length=True)-1


def analyze_frames(capture, window, reference_end, margins, particle_mass, spacing,
        isolated_neighbors, count_neighbors):
    index = []
    for path in sorted(capture.glob('frame_*.npz')):
        with np.load(path) as data:
            index.append((float(data['simulated_seconds']), path))
    index.sort()
    rows = []
    reference_ids = None
    particle_reference = None   # largest head each particle had up to reference_end
    global_reference = None     # largest head any particle had up to reference_end
    top_reference = None        # highest fluid elevation up to reference_end
    for t, path in index:
        if t > window[1]+1.e-8:
            break
        with np.load(path) as data:
            order = np.argsort(data['ids'], kind='stable')
            ids = data['ids'][order]
            positions = data['positions'][order].astype(np.float64)
            velocities = data['velocities'][order].astype(np.float64)
        if reference_ids is None:
            reference_ids = ids
        elif not np.array_equal(ids, reference_ids):
            raise ValueError(f'Particle identities differ in {path.name}')
        speed = np.linalg.norm(velocities, axis=1)
        head = positions[:, 1]+speed*speed/(2.*GRAVITY)
        apex = positions[:, 1]+np.maximum(velocities[:, 1], 0.)**2/(2.*GRAVITY)
        if t >= window[0]-1.e-8 and particle_reference is not None:
            excess = head-particle_reference
            row = dict(time_s=t, max_speed_m_s=float(speed.max()),
                p99_speed_m_s=float(np.percentile(speed, 99)),
                max_head_minus_reference_m=float(head.max()-global_reference),
                max_apex_minus_reference_top_m=float(apex.max()-top_reference),
                particle_head_gain_j=float(particle_mass*GRAVITY*np.maximum(excess, 0.).sum()),
                particle_head_loss_j=float(particle_mass*GRAVITY*np.maximum(-excess, 0.).sum()),
                max_particle_head_gain_m=float(excess.max()))
            for margin in margins:
                label = f'{margin:g}m'
                row[f'above_reference_head_plus_{label}'] = int(np.count_nonzero(head > global_reference+margin))
                row[f'apex_above_reference_top_plus_{label}'] = int(np.count_nonzero(apex > top_reference+margin))
                row[f'own_head_gain_above_{label}'] = int(np.count_nonzero(excess > margin))
            if count_neighbors:
                counts = neighbor_counts(positions, 2.*spacing)
                isolated = counts < isolated_neighbors
                row.update(neighbors_below_20=int(np.count_nonzero(counts < 20)),
                    isolated_count=int(np.count_nonzero(isolated)),
                    no_neighbor_count=int(np.count_nonzero(counts == 0)))
                if isolated.any():
                    row.update(isolated_max_speed_m_s=float(speed[isolated].max()),
                        isolated_p95_speed_m_s=float(np.percentile(speed[isolated], 95)),
                        isolated_max_own_head_gain_m=float(excess[isolated].max()),
                        isolated_max_head_minus_reference_m=float(head[isolated].max()-global_reference))
                    for margin in margins:
                        label = f'{margin:g}m'
                        row[f'isolated_own_head_gain_above_{label}'] = int(
                            np.count_nonzero(excess[isolated] > margin))
                        row[f'isolated_above_reference_head_plus_{label}'] = int(
                            np.count_nonzero(head[isolated] > global_reference+margin))
            rows.append(row)
        if t <= reference_end+1.e-8:
            particle_reference = head if particle_reference is None else np.maximum(particle_reference, head)
            global_reference = float(head.max()) if global_reference is None else max(global_reference, float(head.max()))
            top = float(positions[:, 1].max())
            top_reference = top if top_reference is None else max(top_reference, top)
    summary = dict(frames=len(rows), reference_end_s=reference_end,
        reference_head_m=global_reference, reference_top_elevation_m=top_reference,
        particle_mass_kg=particle_mass, isolated_neighbor_threshold=isolated_neighbors if count_neighbors else None,
        definition='head is world height plus speed squared over 2g; the references are maxima over every '
            'cached frame up to reference_end, globally and per particle; apex is the ballistic rise of the '
            'upward velocity alone; isolated means fewer fluid neighbours than the threshold within 2 spacings',
        limitations='Pressure legitimately moves energy between particles in unsteady flow, so a particle '
            'gaining head is an indicator rather than proof; head above the global reference, or total '
            'gains that are not small beside total losses, are the stronger signs. The reference includes '
            'any head the prescribed donor gave the fluid before reference_end but not afterwards. '
            'Frames are 30 fps samples; peaks between frames are not seen.')
    if rows:
        summary['maximum_over_frames'] = {key: max(row[key] for row in rows if key in row)
            for key in sorted({key for row in rows for key in row}) if key != 'time_s'}
    return rows, summary


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--simulation', type=Path, required=True,
        help='Directory holding probe_report.json, pressure_substeps.jsonl and capture/')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--window', type=float, nargs=2, default=(3.5, 4.166666666666667),
        metavar=('START', 'END'))
    parser.add_argument('--reference-end', type=float,
        help='Last time whose frames define the pre-impact head references; defaults to the window start')
    parser.add_argument('--head-margins', type=float, nargs='+', default=(.01, .05), metavar='METRES')
    parser.add_argument('--isolated-neighbors', type=int, default=6,
        help='A particle with fewer fluid neighbours than this within two spacings counts as isolated')
    parser.add_argument('--skip-neighbor-counts', action='store_true',
        help='Omit the per-frame neighbour search (avoids the SciPy dependency)')
    args = parser.parse_args()
    window = tuple(args.window)
    if not window[0] < window[1]:
        parser.error('The window must be increasing')
    reference_end = window[0] if args.reference_end is None else args.reference_end
    simulation = args.simulation.resolve()
    report = json.loads((simulation/'probe_report.json').read_text())
    spacing = float(report['spacing_m'])
    args.output.mkdir(parents=True, exist_ok=False)
    summary = dict(simulation=str(simulation), status=report['status'],
        window_seconds=list(window), time_integration=report.get('time_integration'),
        note='Indicators only; no physical pass/fail threshold is applied.')

    substep_path = simulation/'pressure_substeps.jsonl'
    if substep_path.exists():
        rows, summary['substeps'] = analyze_substeps(substep_path, window, spacing)
        if rows:
            write_csv(args.output/'substeps.csv', rows)
    else:
        summary['substeps'] = dict(error='pressure_substeps.jsonl is absent; the run had no solver verification window')

    rows, summary['report_frames'] = analyze_report(report, window)
    if rows:
        write_csv(args.output/'report_frames.csv', rows)

    energy = next((row['energy'] for row in report['rows'] if row.get('energy')), None)
    particle_mass = (energy['fluid_mass_kg']/report['particle_count'] if energy
        else 1000.*.8*spacing**3)
    rows, summary['cache_frames'] = analyze_frames(simulation/'capture', window, reference_end,
        tuple(args.head_margins), particle_mass, spacing, args.isolated_neighbors,
        not args.skip_neighbor_counts)
    if rows:
        write_csv(args.output/'cache_frames.csv', rows)

    write_json(args.output/'summary.json', summary)
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()

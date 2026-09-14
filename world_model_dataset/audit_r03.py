"""Lightweight post-run inspection and measurements for R03 rigid collisions."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .contract import artifact,validate_schema
from .io import inside,read_json,write_json


def audit_r03(output):
    out=Path(output);ep=read_json(out/'episode.prepared.json');spec=ep['spec']
    report=read_json(out/'native_report.json');index=read_json(out/'state/index.json')
    fixture=read_json(out/'fixture.json');geometry_index=read_json(out/'geometry/index.json')
    ids=[o['instance_id'] for o in spec['objects']]
    checks=[]
    def check(name,ok,detail):checks.append({'name':name,'status':'pass' if ok else 'fail','detail':detail})

    times=[];states={oid:[] for oid in ids};references={};topologies={};minimum_z=float('inf')
    for frame_number,frame in enumerate(index['frames']):
        row=read_json(inside(out,frame['state']));validate_schema(row,'state')
        expected=frame_number/spec['timing']['capture_hz']
        if abs(row['time_s']-expected)>1e-8 or row['physics_step']!=round(expected*spec['timing']['physics_hz']):
            raise ValueError('State time misalignment')
        if set(row['objects'])!=set(ids) or set(frame['geometries'])!=set(ids):raise ValueError('Object set changed')
        for oid in ids:
            body=row['objects'][oid];path=frame['geometries'][oid]
            if artifact(out,path)!=body['geometry']:raise ValueError('Geometry checksum mismatch')
            with np.load(inside(out,path),allow_pickle=False) as data:
                if not all(np.isfinite(data[key]).all() for key in data.files):raise ValueError('Non-finite cache array')
                vertices=data['surface_world_m'];faces=data['surface_triangles']
                if oid not in references:references[oid]=vertices.copy();topologies[oid]=faces.copy()
                if vertices.shape!=references[oid].shape or not np.array_equal(faces,topologies[oid]):
                    raise ValueError(f'Surface topology changed for {oid}')
                minimum_z=min(minimum_z,float(vertices[:,2].min()))
            states[oid].append(body)
        times.append(row['time_s'])

    expected=round(spec['timing']['duration_s']*spec['timing']['capture_hz'])+1
    check('complete_time_axis',index['complete'] and len(times)==expected,f'{len(times)}/{expected} frames')
    check('finite_fixed_topology',True,f'{len(ids)} objects, {len(times)} finite aligned states')
    check('no_ground_escape',minimum_z>-.02,f'minimum surface z {minimum_z:.6g} m')
    records=report.get('action_applications',[])
    check('initial_velocities_executed',len(records)==len(ids) and all(x['kind']=='initial_velocity' and x['applications']==1 for x in records),str(records))

    def belongs(path,oid):
        root='/World/'+oid;return path==root or path.startswith(root+'/')
    interbody=[];all_points=0;last_step=-1
    with (out/'contacts.jsonl').open() as stream:
        for line in stream:
            row=json.loads(line);validate_schema(row,'contact')
            if row['physics_step']<last_step:raise ValueError('Contact time axis went backwards')
            if abs(row['time_s']-row['physics_step']/spec['timing']['physics_hz'])>1e-8:raise ValueError('Contact time mismatch')
            last_step=row['physics_step'];all_points+=len(row['points'])
            paths=[row[key] for key in ('actor0','actor1','collider0','collider1')]
            if all(any(belongs(path,oid) for path in paths) for oid in ids):
                interbody.extend((row,point) for point in row['points'])
    checks.append({'name':'native_interbody_contact_measurement','status':'pass',
                   'detail':f'{len(interbody)} native interbody points; {all_points} total points; zero is a valid miss outcome'})

    positions={oid:np.asarray([body['position_m'] for body in states[oid]]) for oid in ids}
    velocities={oid:np.asarray([body['linear_velocity_m_s'] for body in states[oid]]) for oid in ids}
    angular={oid:np.asarray([body['angular_velocity_rad_s'] for body in states[oid]]) for oid in ids}
    masses={oid:states[oid][0]['mass_kg'] for oid in ids};inertias={}
    for oid in ids:
        with np.load(inside(out,geometry_index[oid]['path']),allow_pickle=False) as data:
            inertias[oid]=data['inertia_kg_m2'].copy()
    times_array=np.asarray(times)
    total_momentum=sum(masses[oid]*velocities[oid] for oid in ids)
    total_ke=sum(np.asarray([body['kinetic_energy_j'] for body in states[oid]]) for oid in ids)
    first_time=min((row['time_s'] for row,_ in interbody),default=None)
    last_time=max((row['time_s'] for row,_ in interbody),default=None)
    pre_index=None;post_index=None
    if first_time is not None:
        before=np.flatnonzero(times_array<first_time);after=np.flatnonzero(times_array>last_time)
        pre_index=int(before[-1]) if len(before) else 0;post_index=int(after[0]) if len(after) else len(times)-1
    momentum_error=None;momentum_error_absolute=None;energy_ratio=None;angular_change=None
    if pre_index is not None:
        scale=max(float(np.linalg.norm(total_momentum[pre_index])),sum(masses[oid]*float(np.linalg.norm(velocities[oid][pre_index])) for oid in ids),1e-9)
        momentum_error_absolute=float(np.linalg.norm(total_momentum[post_index]-total_momentum[pre_index]))
        momentum_error=momentum_error_absolute/scale
        energy_ratio=float(total_ke[post_index]/max(total_ke[pre_index],1e-12))
        from scipy.spatial.transform import Rotation
        def angular_momentum(index):
            value=np.zeros(3)
            for oid in ids:
                rotation=Rotation.from_quat(states[oid][index]['orientation_xyzw']).as_matrix()
                spin=rotation@inertias[oid]@rotation.T@angular[oid][index]
                value+=np.cross(positions[oid][index],masses[oid]*velocities[oid][index])+spin
            return value
        angular_change=float(np.linalg.norm(angular_momentum(post_index)-angular_momentum(pre_index)))
    impulses=np.asarray([point['impulse_ns'] for _,point in interbody],dtype=float) if interbody else np.empty((0,3))
    pair_distance=np.linalg.norm(positions[ids[1]]-positions[ids[0]],axis=1)
    metrics=dict(source='derived_from_native_states_and_contacts',object_ids=ids,times_s=times,
        first_interbody_contact_time_s=first_time,last_interbody_contact_time_s=last_time,
        contact_duration_s=None if first_time is None else last_time-first_time,
        interbody_contact_points=len(interbody),maximum_interbody_impulse_ns=float(np.linalg.norm(impulses,axis=1).max()) if len(impulses) else None,
        minimum_com_separation_m=float(pair_distance.min()),linear_momentum_error_kg_m_s=momentum_error_absolute,
        momentum_relative_error_pre_post=momentum_error,kinetic_energy_ratio=energy_ratio,
        angular_momentum_change_kg_m2_s=angular_change,comparison_capture_indices={'pre':pre_index,'post':post_index},
        total_linear_momentum_kg_m_s=total_momentum.tolist(),total_kinetic_energy_j=total_ke.tolist(),
        final_poses={oid:{'position_m':positions[oid][-1].tolist(),'orientation_xyzw':states[oid][-1]['orientation_xyzw']} for oid in ids},
        objects={oid:{'positions_m':positions[oid].tolist(),'linear_velocities_m_s':velocities[oid].tolist(),
                      'angular_velocities_rad_s':angular[oid].tolist(),'final_position_m':positions[oid][-1].tolist(),
                      'final_orientation_xyzw':states[oid][-1]['orientation_xyzw'],
                      'max_speed_m_s':float(np.linalg.norm(velocities[oid],axis=1).max())} for oid in ids},
        interpretation='Momentum and energy are diagnostic measurements, not pass gates; floor contact supplies external friction impulse.')
    passed=not any(item['status']=='fail' for item in checks)
    validation={'schema_version':'0.1.0','passed':passed,'checks':checks,'missing_required':[] if passed else ['physical_audit']}
    validate_schema(validation,'validation');write_json(out/'metrics.json',metrics);write_json(out/'physics_validation.json',validation)
    return validation

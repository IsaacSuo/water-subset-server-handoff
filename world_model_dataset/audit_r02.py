"""Post-run cache and native-contact inspection for R02 stair drops."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .contract import artifact,validate_schema
from .io import inside,read_json,write_json


def audit_r02(output):
    out=Path(output);ep=read_json(out/'episode.prepared.json');spec=ep['spec']
    report=read_json(out/'native_report.json');index=read_json(out/'state/index.json');fixture=read_json(out/'fixture.json')
    oid=spec['objects'][0]['instance_id'];checks=[]
    def check(name,ok,detail):checks.append({'name':name,'status':'pass' if ok else 'fail','detail':detail})

    times=[];bodies=[];positions=[];orientations=[];velocities=[];angular=[]
    reference_shape=None;reference_faces=None;minimum_z=float('inf')
    for frame_number,frame in enumerate(index['frames']):
        row=read_json(inside(out,frame['state']));validate_schema(row,'state')
        expected=frame_number/spec['timing']['capture_hz']
        if abs(row['time_s']-expected)>1e-8 or row['physics_step']!=round(expected*spec['timing']['physics_hz']):
            raise ValueError('State time misalignment')
        body=row['objects'][oid]
        if artifact(out,frame['geometry'])!=body['geometry']:raise ValueError('Geometry checksum mismatch')
        with np.load(inside(out,frame['geometry']),allow_pickle=False) as data:
            if not all(np.isfinite(data[key]).all() for key in data.files):raise ValueError('Non-finite cache array')
            vertices=data['surface_world_m'];faces=data['surface_triangles']
            if reference_shape is None:reference_shape=vertices.shape;reference_faces=faces.copy()
            if vertices.shape!=reference_shape or not np.array_equal(faces,reference_faces):raise ValueError('Surface topology changed')
            minimum_z=min(minimum_z,float(vertices[:,2].min()))
        times.append(row['time_s']);bodies.append(body);positions.append(body['position_m'])
        orientations.append(body['orientation_xyzw']);velocities.append(body['linear_velocity_m_s'])
        angular.append(body['angular_velocity_rad_s'])

    expected=round(spec['timing']['duration_s']*spec['timing']['capture_hz'])+1
    check('complete_time_axis',index['complete'] and len(times)==expected,f'{len(times)}/{expected} frames')
    check('finite_fixed_topology',True,f'{len(times)} finite aligned states')
    check('no_ground_escape',minimum_z>-.02,f'minimum surface z {minimum_z:.6g} m')
    applications=report.get('action_applications',[])
    check('initial_velocity_executed',len(applications)==1 and applications[0]['kind']=='initial_velocity' and applications[0]['applications']==1,str(applications))

    fixture_ids={'floor',*fixture['step_ids']};first_contact={};last_contact={};point_counts={};impulse_norm_sums={}
    last_step=-1;subject_points=0
    def fixture_from_paths(paths):
        return next((name for name in fixture_ids if any(path=='/World/'+name or path.startswith('/World/'+name+'/') for path in paths)),None)
    with (out/'contacts.jsonl').open() as stream:
        for line in stream:
            row=json.loads(line);validate_schema(row,'contact')
            if row['physics_step']<last_step:raise ValueError('Contact time axis went backwards')
            if abs(row['time_s']-row['physics_step']/spec['timing']['physics_hz'])>1e-8:raise ValueError('Contact time mismatch')
            last_step=row['physics_step'];paths=[row[key] for key in ('actor0','actor1','collider0','collider1')]
            touches_subject=any(path=='/World/'+oid or path.startswith('/World/'+oid+'/') for path in paths)
            fixture_id=fixture_from_paths(paths)
            if not touches_subject or fixture_id is None:continue
            count=len(row['points']);subject_points+=count
            first_contact.setdefault(fixture_id,row['time_s']);last_contact[fixture_id]=row['time_s']
            point_counts[fixture_id]=point_counts.get(fixture_id,0)+count
            impulse_norm_sums[fixture_id]=impulse_norm_sums.get(fixture_id,0.)+sum(float(np.linalg.norm(point['impulse_ns'])) for point in row['points'])
    check('rigid_contact_impulse',subject_points>0,f'{subject_points} native subject/fixture contact points')

    positions=np.asarray(positions);velocities=np.asarray(velocities);angular=np.asarray(angular)
    rotations=Rotation.from_quat(orientations);relative=rotations[0].inv()*rotations
    orientation_change=relative.magnitude();speeds=np.linalg.norm(velocities,axis=1);angular_speeds=np.linalg.norm(angular,axis=1)
    start=spec['action_parameters']['start_time_s'];post=np.asarray(times)>=start
    contacted=sorted(first_contact,key=lambda name:first_contact[name])
    metrics={'source':'derived_from_native_states_and_contacts','times_s':times,
        'positions_m':positions.tolist(),'linear_velocities_m_s':velocities.tolist(),
        'angular_velocities_rad_s':angular.tolist(),'contacted_fixture_ids':contacted,
        'first_contact_time_by_fixture_s':first_contact,'last_contact_time_by_fixture_s':last_contact,
        'contact_point_count_by_fixture':point_counts,'contact_impulse_norm_sum_by_fixture_ns':impulse_norm_sums,
        'distinct_steps_contacted':sum(name.startswith('step_') for name in contacted),
        'maximum_speed_m_s':float(speeds[post].max()),'maximum_angular_speed_rad_s':float(angular_speeds[post].max()),
        'maximum_orientation_change_rad':float(orientation_change[post].max()),
        'minimum_surface_z_m':minimum_z,'final_position_m':positions[-1].tolist(),
        'final_orientation_xyzw':orientations[-1],'final_speed_m_s':float(speeds[-1]),
        'final_angular_speed_rad_s':float(angular_speeds[-1]),
        'interpretation':'Contact order and impulse sums are measured outcomes; skipped steps are not treated as failures.'}
    passed=not any(item['status']=='fail' for item in checks)
    validation={'schema_version':'0.1.0','passed':passed,'checks':checks,'missing_required':[] if passed else ['physical_audit']}
    validate_schema(validation,'validation');write_json(out/'metrics.json',metrics);write_json(out/'physics_validation.json',validation)
    return validation

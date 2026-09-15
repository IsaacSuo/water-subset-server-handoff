"""Post-run cache/contact inspection for R04 kinematic obstacle pushing."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .contract import artifact,validate_schema
from .io import inside,read_json,write_json


def audit_r04(output):
    out=Path(output);ep=read_json(out/'episode.prepared.json');spec=ep['spec']
    report=read_json(out/'native_report.json');index=read_json(out/'state/index.json');fixture=read_json(out/'fixture.json')
    oid=spec['objects'][0]['instance_id'];checks=[]
    def check(name,ok,detail):checks.append({'name':name,'status':'pass' if ok else 'fail','detail':detail})

    times=[];positions=[];velocities=[];angular=[];orientations=[];surface_min_x=[];surface_max_x=[];surface_min_y=[];surface_max_y=[]
    reference_shape=None;reference_faces=None;minimum_z=float('inf');maximum_tracking_error=0.;pusher_positions=[]
    for frame_number,frame in enumerate(index['frames']):
        row=read_json(inside(out,frame['state']));validate_schema(row,'state');expected=frame_number/spec['timing']['capture_hz']
        if abs(row['time_s']-expected)>1e-8 or row['physics_step']!=round(expected*spec['timing']['physics_hz']):
            raise ValueError('State time misalignment')
        body=row['objects'][oid]
        if artifact(out,frame['geometry'])!=body['geometry']:raise ValueError('Geometry checksum mismatch')
        with np.load(inside(out,frame['geometry']),allow_pickle=False) as data:
            if not all(np.isfinite(data[key]).all() for key in data.files):raise ValueError('Non-finite cache array')
            vertices=data['surface_world_m'];faces=data['surface_triangles']
            if reference_shape is None:reference_shape=vertices.shape;reference_faces=faces.copy()
            if vertices.shape!=reference_shape or not np.array_equal(faces,reference_faces):raise ValueError('Topology changed')
            minimum_z=min(minimum_z,float(vertices[:,2].min()));surface_min_x.append(float(vertices[:,0].min()));surface_max_x.append(float(vertices[:,0].max()))
            surface_min_y.append(float(vertices[:,1].min()));surface_max_y.append(float(vertices[:,1].max()))
        commanded=np.asarray(frame['fixture_positions'][fixture['pusher_id']],dtype=float)
        native=frame.get('native_kinematic_poses',{}).get(fixture['pusher_id'])
        if not native or not native.get('ret_val'):raise ValueError('Native pusher pose unavailable')
        actual=np.asarray(native['position'],dtype=float);maximum_tracking_error=max(maximum_tracking_error,float(np.linalg.norm(commanded-actual)))
        pusher_positions.append(actual.tolist());times.append(row['time_s']);positions.append(body['position_m'])
        velocities.append(body['linear_velocity_m_s']);angular.append(body['angular_velocity_rad_s']);orientations.append(body['orientation_xyzw'])

    expected=round(spec['timing']['duration_s']*spec['timing']['capture_hz'])+1
    check('complete_time_axis',index['complete'] and len(times)==expected,f'{len(times)}/{expected} frames')
    check('finite_fixed_topology',True,f'{len(times)} finite aligned states')
    check('no_ground_escape',minimum_z>-.02,f'minimum surface z {minimum_z:.6g} m')
    check('pusher_native_tracking',maximum_tracking_error<1e-5,f'max command/native error {maximum_tracking_error:.6g} m')
    applications=report.get('action_applications',[])
    check('kinematic_push_executed',len(applications)==1 and applications[0]['kind']=='kinematic_trajectory' and applications[0]['applications']>0,str(applications))

    fixture_ids={item['id'] for item in fixture['boxes']};first={};last={};points={};impulses={};last_step=-1
    def belongs(path,name):
        root='/World/'+name;return path==root or path.startswith(root+'/')
    with (out/'contacts.jsonl').open() as stream:
        for line in stream:
            row=json.loads(line);validate_schema(row,'contact');step=row['physics_step']
            if step<last_step or abs(row['time_s']-step/spec['timing']['physics_hz'])>1e-8:raise ValueError('Contact time misalignment')
            last_step=step;paths=[row[key] for key in ('actor0','actor1','collider0','collider1')]
            if not any(belongs(path,oid) for path in paths):continue
            name=next((candidate for candidate in fixture_ids if any(belongs(path,candidate) for path in paths)),None)
            if name is None:continue
            first.setdefault(name,row['time_s']);last[name]=row['time_s'];points[name]=points.get(name,0)+len(row['points'])
            impulses[name]=impulses.get(name,0.)+sum(float(np.linalg.norm(point['impulse_ns'])) for point in row['points'])
    contact_order=sorted(first,key=lambda name:first[name]);nonfloor=[name for name in contact_order if name!='floor']
    check('native_subject_fixture_contacts',sum(points.values())>0,f'{sum(points.values())} native points; non-floor order {nonfloor}')
    check('pusher_reached_subject',fixture['pusher_id'] in first,f"first contact {first.get(fixture['pusher_id'])}")

    positions=np.asarray(positions);velocities=np.asarray(velocities);angular=np.asarray(angular);times_array=np.asarray(times)
    d=fixture['D_m'];start=spec['action_parameters']['start_time_s'];active=times_array>=start
    from scipy.spatial.transform import Rotation
    obstacle=next(item for item in fixture['boxes'] if item['id']==fixture['obstacle_id'])
    half=np.asarray(obstacle['size_m'])/2;corners=np.array([[x,y,z] for x in (-half[0],half[0]) for y in (-half[1],half[1]) for z in (-half[2],half[2])])
    obstacle_world=Rotation.from_quat(obstacle['orientation_xyzw']).apply(corners)+obstacle['position_m']
    obstacle_min,obstacle_max=obstacle_world.min(0),obstacle_world.max(0)
    centre_past=np.asarray(positions)[:,0]>obstacle['position_m'][0]
    clear_forward=np.asarray(surface_min_x)>obstacle_max[0]
    clear_side=(np.asarray(surface_max_y)<obstacle_min[1])|(np.asarray(surface_min_y)>obstacle_max[1])
    fully_cleared=bool(np.any(clear_forward|(centre_past&clear_side)))
    obstacle_contact=fixture['obstacle_id'] in first
    metrics={'source':'derived_from_native_states_contacts_and_kinematic_pose','times_s':times,
        'positions_m':positions.tolist(),'linear_velocities_m_s':velocities.tolist(),
        'angular_velocities_rad_s':angular.tolist(),'orientations_xyzw':orientations,
        'pusher_positions_m':pusher_positions,'pusher_tracking_error_m':maximum_tracking_error,
        'contacted_fixture_ids':contact_order,'nonfloor_contact_order':nonfloor,
        'first_contact_time_by_fixture_s':first,'last_contact_time_by_fixture_s':last,
        'contact_point_count_by_fixture':points,'contact_impulse_norm_sum_by_fixture_ns':impulses,
        'forward_progress_D':float((positions[:,0].max()-positions[0,0])/d),
        'final_forward_displacement_D':float((positions[-1,0]-positions[0,0])/d),
        'maximum_lateral_displacement_D':float(np.max(np.abs(positions[:,1]-positions[0,1]))/d),
        'final_lateral_displacement_D':float((positions[-1,1]-positions[0,1])/d),
        'maximum_speed_m_s':float(np.linalg.norm(velocities[active],axis=1).max()),
        'maximum_angular_speed_rad_s':float(np.linalg.norm(angular[active],axis=1).max()),
        'obstacle_contact_observed':obstacle_contact,'fully_cleared_obstacle':fully_cleared,
        'clearance_modes_observed':{'forward':bool(np.any(clear_forward)),'lateral_route':bool(np.any(centre_past&clear_side))},
        'stuck_behind_obstacle':bool(obstacle_contact and not fully_cleared),
        'minimum_surface_z_m':minimum_z,'final_position_m':positions[-1].tolist(),
        'final_velocity_m_s':velocities[-1].tolist(),
        'interpretation':'Clear/stuck, branch direction and contact order are measured outcomes, not success gates.'}
    passed=not any(item['status']=='fail' for item in checks)
    validation={'schema_version':'0.1.0','passed':passed,'checks':checks,'missing_required':[] if passed else ['physical_audit']}
    validate_schema(validation,'validation');write_json(out/'metrics.json',metrics);write_json(out/'physics_validation.json',validation)
    return validation

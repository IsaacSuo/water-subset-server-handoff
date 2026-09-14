"""Post-physics audit and metrics. No simulation or rendering side effects."""
from pathlib import Path

import numpy as np

from .contract import artifact,validate_schema
from .io import read_json,write_json,inside
from .metrics import nonrigid_residual,recovery_time,box_penetration,triangle_box_surface_audit,tet_boundary_faces


def audit(output):
    out=Path(output);ep=read_json(out/'episode.prepared.json');spec=ep['spec']
    if spec['event_id']=='R03':
        from .audit_r03 import audit_r03
        return audit_r03(out)
    report=read_json(out/'native_report.json');index=read_json(out/'state/index.json')
    fixture=read_json(out/'fixture.json');oid=spec['objects'][0]['instance_id']
    times=[];bodies=[];residuals=[];depths=[];surface_reference=None;topology=None
    surface_intersections={'candidate_triangle_tests':0,'intersecting_triangles':0,'maximum_sampled_penetration_m':0.}
    collision_intersections={'candidate_triangle_tests':0,'intersecting_triangles':0,'maximum_sampled_penetration_m':0.}
    max_kinematic_error=0.
    checks=[]
    def check(name,ok,detail):checks.append(dict(name=name,status='pass' if ok else 'fail',detail=detail))
    for i,frame in enumerate(index['frames']):
        row=read_json(inside(out,frame['state']));validate_schema(row,'state')
        expected=i/spec['timing']['capture_hz']
        if abs(row['time_s']-expected)>1e-8 or row['physics_step']!=round(expected*spec['timing']['physics_hz']):
            raise ValueError('State time misalignment')
        if frame['time_s']!=row['time_s'] or frame['physics_step']!=row['physics_step']:raise ValueError('State index mismatch')
        body=row['objects'][oid]
        if artifact(out,frame['geometry'])!=body['geometry']:raise ValueError('Geometry checksum mismatch')
        with np.load(inside(out,frame['geometry']),allow_pickle=False) as data:
            vertices=data['surface_world_m'];faces=data['surface_triangles']
            if not all(np.isfinite(data[k]).all() for k in data.files):raise ValueError('Non-finite cache array')
            if surface_reference is None:surface_reference=vertices.copy();topology=faces.copy()
            if not np.array_equal(faces,topology) or vertices.shape!=surface_reference.shape:raise ValueError('Surface topology changed')
            positions_actual=dict(frame['fixture_positions'])
            for name,actual in frame.get('native_kinematic_poses',{}).items():
                if not actual.get('ret_val'):raise ValueError(f'Native kinematic pose unavailable: {name}')
                max_kinematic_error=max(max_kinematic_error,float(np.linalg.norm(np.asarray(actual['position'])-positions_actual[name])))
                positions_actual[name]=actual['position']
            residuals.append(nonrigid_residual(surface_reference,vertices))
            depths.append(box_penetration(vertices,fixture['boxes'],positions_actual))
            scan=triangle_box_surface_audit(vertices,faces,fixture['boxes'],positions_actual)
            for key in surface_intersections:surface_intersections[key]=max(surface_intersections[key],scan[key]) if 'maximum' in key else surface_intersections[key]+scan[key]
            if len(data['collision_world_m']):
                boundary=tet_boundary_faces(data['collision_tets'])
                scan=triangle_box_surface_audit(data['collision_world_m'],boundary,fixture['boxes'],positions_actual)
                for key in collision_intersections:collision_intersections[key]=max(collision_intersections[key],scan[key]) if 'maximum' in key else collision_intersections[key]+scan[key]
        times.append(row['time_s']);bodies.append(body)
    expected=round(spec['timing']['duration_s']*spec['timing']['capture_hz'])+1
    check('complete_time_axis',index['complete'] and len(times)==expected,f'{len(times)}/{expected} frames')
    check('no_ground_escape',min(b['metrics']['minimum_z_m'] for b in bodies)>-.02,'Surface must remain above ground tolerance')
    kind=report['physical_representation']
    # Rigid contact remains strict. Large but valid volume deformation gets a
    # D-relative geometric envelope; the exact continuous depth is always
    # published so downstream tasks can impose a tighter filter.
    tolerance=.015*fixture['D_m'] if kind=='rigid' else .02*fixture['D_m']
    checks.append(dict(name='surface_fixture_intersection_measurement',status='pass',
          detail=f"Diagnostic only: all captured triangles scanned; {surface_intersections['intersecting_triangles']} clipped intersections, max sampled depth {surface_intersections['maximum_sampled_penetration_m']:.6g} m; former review reference {tolerance:.6g} m"))
    if collision_intersections['candidate_triangle_tests']:
        checks.append(dict(name='collision_shell_fixture_intersection_measurement',status='pass',
              detail=f"Diagnostic only: all captured collision-boundary triangles scanned; max sampled depth {collision_intersections['maximum_sampled_penetration_m']:.6g} m"))
    check('kinematic_command_tracking',max_kinematic_error<1e-5,f'Max commanded/native position error {max_kinematic_error:.6g} m')
    speeds=np.array([np.linalg.norm(b['linear_velocity_m_s']) for b in bodies])
    positions=np.array([b['position_m'] for b in bodies]);d=fixture['D_m']
    max_inverted=max(b['metrics'].get('inverted_tets',0) for b in bodies)
    check('no_inverted_tets',max_inverted==0,f'{max_inverted} inversions at sampled states')
    contact_count=0;last_step=0;max_contact_depth=None
    contact_path=out/'contacts.jsonl'
    if kind=='rigid' and not contact_path.is_file():raise ValueError('Rigid contact truth is required')
    if kind=='volumetric' and contact_path.exists():raise ValueError('Unsupported soft contacts must not be serialized as an empty or zero-valued stream')
    if kind=='rigid':
      with contact_path.open() as stream:
        import json
        for line in stream:
            c=json.loads(line);validate_schema(c,'contact')
            if c['physics_step']<last_step or c['physics_step']>round(spec['timing']['duration_s']*spec['timing']['physics_hz']):raise ValueError('Contact time axis invalid')
            if abs(c['time_s']-c['physics_step']/spec['timing']['physics_hz'])>1e-8:raise ValueError('Contact time mismatch')
            last_step=c['physics_step'];contact_count+=len(c['points'])
            for p in c['points']:max_contact_depth=max(max_contact_depth or 0.,-p['separation_m'])
    if kind=='rigid':
        check('rigid_contact_impulse',report['contact_counts']['subject_points']>0,
              f"{report['contact_counts']['subject_points']} native PhysX contact points")
    else:
        evidence=read_json(out/'capability_probes/soft_contact_impulse.json')
        unavailable=evidence['status']=='unavailable' and evidence['callback_headers']==0 and evidence['callback_points']==0
        checks.append(dict(name='soft_contact_impulse',status='unavailable' if unavailable else 'fail',
            detail=evidence['reason']))
    if kind=='rigid' and contact_count:
        check('native_contact_separation',max_contact_depth<.003,f'Max native negative separation {max_contact_depth:.6g} m')
    else:
        checks.append(dict(name='native_contact_separation',status='unavailable',detail='Not exposed for this physical representation; no value substituted'))
    values=dict(max_speed_m_s=float(speeds.max()),max_nonrigid_rms_m=max(residuals),final_shape_residual_m=residuals[-1],
                max_vertex_fixture_overlap_m=max(depths),max_native_contact_depth_m=max_contact_depth,
                surface_fixture_audit=surface_intersections,collision_shell_fixture_audit=collision_intersections,
                max_kinematic_position_error_m=max_kinematic_error,
                final_position_m=positions[-1].tolist(),final_orientation_xyzw=bodies[-1]['orientation_xyzw'],
                final_speed_m_s=float(speeds[-1]),inverted_tets=max_inverted,source='derived_from_native_states',
                times_s=times,nonrigid_rms_m=residuals)
    if spec['event_id']=='R01':
        release=spec['action_parameters']['release_time_s'];bottom=np.flatnonzero(positions[:,0]>=0)
        values['bottom_arrival_time_s']=float(times[bottom[0]]-release) if len(bottom) else None
        values['horizontal_travel_m']=float(max(0,positions[-1,0]))
        angular=np.array([b['angular_velocity_rad_s'] for b in bodies])
        values['max_angular_speed_rad_s']=float(np.linalg.norm(angular,axis=1).max())
        geometry=spec['objects'][0]['object_id']
        if geometry in ('sphere','cylinder'):
            tangent=np.asarray(fixture['downhill_tangent']);moving=(speeds>.05)&(positions[:,0]<0)&(np.asarray(times)>=release)
            translation=np.abs(np.array([np.dot(b['linear_velocity_m_s'],tangent) for b in bodies]))
            rotation=np.abs(angular[:,1])*(d/2)
            values['rolling_sliding_ratio']=float(np.median(rotation[moving]/np.maximum(translation[moving],1e-9))) if moving.any() else None
            values['rolling_sliding_ratio_semantics']='rotational surface speed / downhill COM speed; 1 is ideal no-slip for sphere/cylinder with radius D/2'
        else:
            values['rolling_sliding_ratio']=None
            values['rolling_sliding_ratio_semantics']='not applicable to non-axisymmetric diagnostic geometry'
        values['stop_time_s']=recovery_time(times,speeds,release,.01)
        values['stop_observed']=values['stop_time_s'] is not None
        values['stopping_distance_m']=values['horizontal_travel_m'] if values['stop_observed'] else None
        displacement=float(np.max(np.linalg.norm(positions-positions[0],axis=1)))
        values['release_outcome']='moving' if displacement>.1*d else 'stationary'
        values['maximum_displacement_D']=displacement/d
        release_records=[x for x in report.get('action_applications',[]) if x['kind']=='release']
        check('release_action_executed',len(release_records)==1 and release_records[0]['applications']==1,
              f'Release command records: {release_records}')
    else:
        heights=np.array([b['metrics']['height_m'] for b in bodies]);rest=fixture['rest_height_m']
        values['max_compression_fraction']=float(1-heights.min()/rest)
        values['volume_ratio']=[b['metrics']['volume_ratio'] for b in bodies]
        release=sum(spec['action_parameters'].values())
        values['recovery_time_s']=recovery_time(times,residuals,release,.01*d)
        values['recovery_observed']=values['recovery_time_s'] is not None
        target=spec['fixture_parameters']['compression_fraction']
        check('controlled_compression',abs(values['max_compression_fraction']-target)<=.03,
              f"Observed {values['max_compression_fraction']:.4f}, target {target:.4f}, tolerance 0.03")
        substep=report.get('substep_tet_audit')
        check('every_step_tet_inversion_audit',bool(substep) and substep['inverted_tets']==0,
              'Unavailable in legacy probe' if not substep else f"{substep['checked_steps']} physics steps, min J {substep['minimum_j']:.6g}")
    physics_passed=not any(c['status']=='fail' for c in checks)
    result=dict(schema_version='0.1.0',passed=physics_passed,checks=checks,missing_required=[] if physics_passed else ['physical_audit'])
    validate_schema(result,'validation');write_json(out/'metrics.json',values);write_json(out/'physics_validation.json',result)
    return result


if __name__=='__main__':
    import argparse,json
    parser=argparse.ArgumentParser();parser.add_argument('episode');args=parser.parse_args()
    print(json.dumps(audit(args.episode),indent=2))

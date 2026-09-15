"""Physics-only cache review for V04 soft-body aperture traversal."""
from __future__ import annotations

from pathlib import Path

import numpy as np

from .contract import artifact,validate_schema
from .io import inside,read_json,write_json
from .metrics import nonrigid_residual,recovery_time,tet_boundary_faces,triangle_box_surface_audit


def audit_v04(output):
    out=Path(output);ep=read_json(out/'episode.prepared.json');spec=ep['spec']
    fixture=read_json(out/'fixture.json');report=read_json(out/'native_report.json');index=read_json(out/'state/index.json')
    oid=spec['objects'][0]['instance_id'];checks=[]
    def check(name,ok,detail):checks.append({'name':name,'status':'pass' if ok else 'fail','detail':detail})

    start=spec['action_parameters']['start_time_s'];end=start+spec['action_parameters']['push_duration_s']
    reference_index=round(start*spec['timing']['capture_hz']);reference_frame=index['frames'][reference_index]
    with np.load(inside(out,reference_frame['geometry']),allow_pickle=False) as data:reference=data['surface_world_m'].copy()
    reference_width=float(np.ptp(reference[:,1]));wall_boxes=[b for b in fixture['boxes'] if b['id'].startswith('wall_')]
    times=[];positions=[];velocities=[];residuals=[];widths=[];surface_min_x=[];surface_max_x=[]
    surface_topology=None;tet_topology=None;sampled_inversions=0;minimum_z=float('inf');tracking_error=0.
    wall_surface={'candidate_triangle_tests':0,'intersecting_triangles':0,'maximum_sampled_penetration_m':0.}
    wall_collision={'candidate_triangle_tests':0,'intersecting_triangles':0,'maximum_sampled_penetration_m':0.}
    for frame_number,frame in enumerate(index['frames']):
        row=read_json(inside(out,frame['state']));validate_schema(row,'state');expected=frame_number/spec['timing']['capture_hz']
        if abs(row['time_s']-expected)>1e-8 or row['physics_step']!=round(expected*spec['timing']['physics_hz']):raise ValueError('State time misalignment')
        body=row['objects'][oid]
        if artifact(out,frame['geometry'])!=body['geometry']:raise ValueError('Geometry checksum mismatch')
        with np.load(inside(out,frame['geometry']),allow_pickle=False) as data:
            if not all(np.isfinite(data[key]).all() for key in data.files):raise ValueError('Non-finite cache array')
            surface=data['surface_world_m'];faces=data['surface_triangles'];tets=data['simulation_tets']
            if surface_topology is None:surface_topology=(surface.shape,faces.copy());tet_topology=tets.copy()
            if surface.shape!=surface_topology[0] or not np.array_equal(faces,surface_topology[1]) or not np.array_equal(tets,tet_topology):
                raise ValueError('Soft topology changed')
            residuals.append(nonrigid_residual(reference,surface));widths.append(float(np.ptp(surface[:,1])))
            surface_min_x.append(float(surface[:,0].min()));surface_max_x.append(float(surface[:,0].max()));minimum_z=min(minimum_z,float(surface[:,2].min()))
            sampled_inversions=max(sampled_inversions,int(body['metrics'].get('inverted_tets',0)))
            scan=triangle_box_surface_audit(surface,faces,wall_boxes,{b['id']:b['position_m'] for b in wall_boxes})
            for key in wall_surface:wall_surface[key]=max(wall_surface[key],scan[key]) if 'maximum' in key else wall_surface[key]+scan[key]
            collision=data['collision_world_m'];collision_tets=data['collision_tets']
            if len(collision):
                scan=triangle_box_surface_audit(collision,tet_boundary_faces(collision_tets),wall_boxes,{b['id']:b['position_m'] for b in wall_boxes})
                for key in wall_collision:wall_collision[key]=max(wall_collision[key],scan[key]) if 'maximum' in key else wall_collision[key]+scan[key]
        native=frame.get('native_kinematic_poses',{}).get(fixture['pusher_id'])
        if not native or not native.get('ret_val'):raise ValueError('Native pusher pose unavailable')
        tracking_error=max(tracking_error,float(np.linalg.norm(np.asarray(native['position'])-np.asarray(frame['fixture_positions'][fixture['pusher_id']]))))
        times.append(row['time_s']);positions.append(body['position_m']);velocities.append(body['linear_velocity_m_s'])

    expected=round(spec['timing']['duration_s']*spec['timing']['capture_hz'])+1
    check('complete_time_axis',index['complete'] and len(times)==expected,f'{len(times)}/{expected} frames')
    check('finite_fixed_topology',True,f'fixed surface and {len(tet_topology)} Tets')
    check('no_ground_escape',minimum_z>-.02,f'minimum surface z {minimum_z:.6g} m')
    check('no_sampled_inverted_tets',sampled_inversions==0,f'{sampled_inversions} sampled inversions')
    check('pusher_native_tracking',tracking_error<1e-5,f'max command/native error {tracking_error:.6g} m')
    applications=report.get('action_applications',[])
    check('kinematic_push_executed',len(applications)==1 and applications[0]['kind']=='kinematic_trajectory' and applications[0]['applications']>0,str(applications))
    substep=report.get('substep_tet_audit') or {};steps=round(spec['timing']['duration_s']*spec['timing']['physics_hz'])
    check('every_step_tet_inversion_audit',substep.get('checked_steps')==steps and substep.get('inverted_tets')==0,
          f"{substep.get('checked_steps')} steps, minimum J {substep.get('minimum_j')}")
    material=report.get('deformable_material_tensor_readback') or {};declared=ep['inputs']['objects'][0]['physics']
    material_ok=(material.get('status')=='native' and material.get('count')==1 and
        abs(material.get('youngs_modulus_pa',-1)-declared['youngs_modulus_pa'])<=1e-5*declared['youngs_modulus_pa'] and
        abs(material.get('poissons_ratio',-1)-declared['poissons_ratio'])<=1e-6 and
        abs(material.get('dynamic_friction',-1)-declared['dynamic_friction'])<=1e-6)
    check('deformable_material_native_readback',material_ok,f'declared E={declared["youngs_modulus_pa"]}; native={material}')
    evidence=read_json(out/'capability_probes/soft_contact_impulse.json')
    checks.append({'name':'soft_contact_impulse','status':'unavailable' if evidence['status']=='unavailable' else 'fail','detail':evidence['reason']})
    checks.append({'name':'wall_intersection_measurement','status':'pass',
        'detail':f"surface max {wall_surface['maximum_sampled_penetration_m']} m; collision-shell max {wall_collision['maximum_sampled_penetration_m']} m; diagnostic, not solver contact"})

    times_array=np.asarray(times);positions=np.asarray(positions);velocities=np.asarray(velocities);residuals=np.asarray(residuals);widths=np.asarray(widths)
    active=np.flatnonzero((times_array>=start)&(times_array<=end+1e-8));wall_back=fixture['wall_center_x_m']+fixture['wall_thickness_m']/2
    fully=np.asarray(surface_min_x)>wall_back;entered=np.asarray(surface_max_x)>wall_back
    if fully[-1]:outcome='passed'
    elif entered[-1]:outcome='partially_through'
    else:outcome='stuck_before_aperture'
    recovery={}
    for fraction in (.005,.01):recovery[str(fraction)]={'threshold_m':fraction*fixture['D_m'],'sustained_hold_s':.5,
        'time_after_push_s':recovery_time(times_array,residuals,end,fraction*fixture['D_m'])}
    final_speed=float(np.linalg.norm(velocities[-1]));motion_settled=final_speed<.01
    metrics={'source':'derived_from_native_fixed_topology_states','times_s':times,'positions_m':positions.tolist(),
        'linear_velocities_m_s':velocities.tolist(),'push_start_time_s':start,'push_end_time_s':end,
        'reference_capture_index':reference_index,'reference_width_m':reference_width,
        'aperture_width_m':fixture['aperture_width_m'],'aperture_width_D':spec['fixture_parameters']['aperture_width_D'],
        'aperture_outcome':outcome,'ever_fully_passed':bool(np.any(fully)),'final_fully_passed':bool(fully[-1]),
        'maximum_lateral_compression_fraction':float(max(0.,1-widths[active].min()/reference_width)),
        'maximum_nonrigid_rms_m':float(residuals[active].max()),'final_shape_residual_m':float(residuals[-1]),
        'recovery_thresholds_D':recovery,'forward_com_displacement_D':float((positions[-1,0]-positions[reference_index,0])/fixture['D_m']),
        'maximum_speed_m_s':float(np.linalg.norm(velocities[active],axis=1).max()),'final_speed_m_s':final_speed,
        'motion_settled_at_end':motion_settled,'observation_right_censored':not motion_settled,
        'minimum_tet_jacobian':substep.get('minimum_j'),'inverted_tets':substep.get('inverted_tets'),
        'wall_surface_audit':wall_surface,'wall_collision_shell_audit':wall_collision,
        'pusher_tracking_error_m':tracking_error,'nonrigid_rms_m':residuals.tolist(),'lateral_width_m':widths.tolist(),
        'surface_min_x_m':surface_min_x,'surface_max_x_m':surface_max_x,
        'interpretation':'Pass/stuck is a measured outcome. Wall overlap is analytical captured-mesh evidence, not a native soft-contact impulse.'}
    passed=not any(item['status']=='fail' for item in checks)
    validation={'schema_version':'0.1.0','passed':passed,'checks':checks,'missing_required':[] if passed else ['physical_audit']}
    validate_schema(validation,'validation');write_json(out/'metrics.json',metrics);write_json(out/'physics_validation.json',validation)
    return validation

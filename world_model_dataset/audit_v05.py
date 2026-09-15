"""Physics-only inspection and measurements for V05 rigid/soft impacts."""
from __future__ import annotations

from pathlib import Path
import json

import numpy as np

from .contract import artifact,validate_schema
from .io import inside,read_json,write_json
from .metrics import nonrigid_residual


def velocity_comparison_indices(times,start_time,first_contact,last_contact):
    times=np.asarray(times)
    # A miss has no contact interval: use the first moving capture, not launch rest.
    before=(np.flatnonzero(times<first_contact) if first_contact is not None else
            np.flatnonzero(times>start_time)[:1])
    after=np.flatnonzero(times>(last_contact if last_contact is not None else times[-1]))
    return (int(before[-1]) if len(before) else 0,
            int(after[0]) if len(after) else len(times)-1)


def audit_v05(output):
    out=Path(output);ep=read_json(out/'episode.prepared.json');spec=ep['spec'];fixture=read_json(out/'fixture.json')
    report=read_json(out/'native_report.json');index=read_json(out/'state/index.json')
    target_id=fixture['target_id'];projectile_id=fixture['projectile_id'];ids={target_id,projectile_id};checks=[]
    def check(name,ok,detail):checks.append({'name':name,'status':'pass' if ok else 'fail','detail':detail})

    times=[];target_positions=[];target_velocities=[];projectile_positions=[];projectile_velocities=[]
    target_residuals=[];references={};topologies={};tet_topology=None;minimum_z=float('inf');sampled_inversions=0
    for frame_number,frame in enumerate(index['frames']):
        row=read_json(inside(out,frame['state']));validate_schema(row,'state')
        expected=frame_number/spec['timing']['capture_hz']
        if abs(row['time_s']-expected)>1e-8 or row['physics_step']!=round(expected*spec['timing']['physics_hz']):
            raise ValueError('State time misalignment')
        if set(row['objects'])!=ids or set(frame['geometries'])!=ids:raise ValueError('Mixed object set changed')
        for oid in ids:
            body=row['objects'][oid];path=frame['geometries'][oid]
            if artifact(out,path)!=body['geometry']:raise ValueError('Geometry checksum mismatch')
            with np.load(inside(out,path),allow_pickle=False) as data:
                if not all(np.isfinite(data[key]).all() for key in data.files):raise ValueError('Non-finite cache array')
                surface=data['surface_world_m'];faces=data['surface_triangles'];minimum_z=min(minimum_z,float(surface[:,2].min()))
                if oid not in references:references[oid]=surface.copy();topologies[oid]=faces.copy()
                if surface.shape!=references[oid].shape or not np.array_equal(faces,topologies[oid]):raise ValueError(f'Surface topology changed: {oid}')
                if oid==target_id:
                    tets=data['simulation_tets']
                    if tet_topology is None:tet_topology=tets.copy()
                    if not np.array_equal(tets,tet_topology):raise ValueError('Target Tet topology changed')
                    target_residuals.append(nonrigid_residual(references[oid],surface))
                    sampled_inversions=max(sampled_inversions,int(body['metrics'].get('inverted_tets',0)))
        target=row['objects'][target_id];projectile=row['objects'][projectile_id]
        target_positions.append(target['position_m']);target_velocities.append(target['linear_velocity_m_s'])
        projectile_positions.append(projectile['position_m']);projectile_velocities.append(projectile['linear_velocity_m_s'])
        times.append(row['time_s'])

    expected=round(spec['timing']['duration_s']*spec['timing']['capture_hz'])+1
    check('complete_shared_time_axis',index['complete'] and len(times)==expected,f'2 objects, {len(times)}/{expected} frames')
    check('finite_fixed_topology',True,f'fixed rigid surface, soft surface and {len(tet_topology)} target Tets')
    check('no_sampled_inverted_tets',sampled_inversions==0,f'{sampled_inversions} sampled inversions')
    check('no_ground_escape',minimum_z>-.02,f'minimum surface z {minimum_z:.6g} m')
    applications=report.get('action_applications',[])
    check('initial_velocity_executed',len(applications)==1 and applications[0]['target']==projectile_id and applications[0]['applications']==1,str(applications))
    contact_rows=0;contact_points=0;previous_step=0
    with (out/'contacts.jsonl').open() as stream:
        for line in stream:
            contact=json.loads(line);validate_schema(contact,'contact');step=contact['physics_step']
            if (step<previous_step or step>round(spec['timing']['duration_s']*spec['timing']['physics_hz']) or
                    abs(contact['time_s']-step/spec['timing']['physics_hz'])>1e-8):
                raise ValueError('Native rigid contact time misalignment')
            previous_step=step;contact_rows+=1;contact_points+=len(contact['points'])
    check('native_rigid_contact_stream',True,
          f'{contact_rows} records, {contact_points} points; rigid/fixture only, not rigid/soft impulse supervision')

    substep=report.get('substep_tet_audit') or {}
    check('every_step_tet_inversion_audit',substep.get('checked_steps')==round(spec['timing']['duration_s']*spec['timing']['physics_hz']) and substep.get('inverted_tets')==0,
          f"{substep.get('checked_steps')} steps, minimum J {substep.get('minimum_j')}")
    impact=report.get('substep_impact_audit') or {}
    finite_impact=impact.get('checked_steps')==substep.get('checked_steps') and all(np.isfinite(impact.get(key,np.nan)) for key in (
        'maximum_target_nonrigid_rms_m','maximum_target_local_displacement_m','maximum_target_axis_compression_fraction',
        'maximum_rigid_speed_m_s','maximum_soft_nodal_speed_m_s','minimum_sampled_surface_gap_m','maximum_sampled_penetration_m'))
    check('finite_every_step_impact_diagnostics',finite_impact,f"{impact.get('checked_steps')} finite steps")
    checks.append({'name':'sampled_rigid_soft_penetration_measurement','status':'pass',
        'detail':f"diagnostic lower bound {impact.get('maximum_sampled_penetration_m')} m; no universal pass threshold"})

    material=report.get('deformable_material_tensor_readback') or {}
    expected_material=next(obj['physics'] for obj in ep['inputs']['objects'] if obj['instance_id']==target_id)
    material_ok=(material.get('status')=='native' and material.get('count')==1 and
        abs(material.get('youngs_modulus_pa',-1)-expected_material['youngs_modulus_pa'])<=1e-5*expected_material['youngs_modulus_pa'] and
        abs(material.get('poissons_ratio',-1)-expected_material['poissons_ratio'])<=1e-6 and
        abs(material.get('dynamic_friction',-1)-expected_material['dynamic_friction'])<=1e-6)
    check('deformable_material_native_readback',material_ok,
          f"declared E={expected_material['youngs_modulus_pa']}, nu={expected_material['poissons_ratio']}, friction={expected_material['dynamic_friction']}; native={material}")
    evidence=read_json(out/'capability_probes/soft_contact_impulse.json')
    checks.append({'name':'soft_contact_impulse','status':'unavailable' if evidence['status']=='unavailable' else 'fail','detail':evidence['reason']})

    times_array=np.asarray(times);target_positions=np.asarray(target_positions);target_velocities=np.asarray(target_velocities)
    projectile_positions=np.asarray(projectile_positions);projectile_velocities=np.asarray(projectile_velocities)
    first=impact.get('geometric_contact_first_time_s');last=impact.get('geometric_contact_last_time_s')
    pre_index,post_index=velocity_comparison_indices(times_array,spec['action_parameters']['start_time_s'],first,last)
    pre_velocity=projectile_velocities[pre_index];post_velocity=projectile_velocities[post_index]
    target_displacement=np.linalg.norm(target_positions-target_positions[0],axis=1)
    metrics={'source':'derived_from_native_states_and_every_step_diagnostics','target_id':target_id,'projectile_id':projectile_id,
        'times_s':times,'geometric_contact_first_time_s':first,'geometric_contact_last_time_s':last,
        'geometric_contact_observed':first is not None,'comparison_capture_indices':{'pre':pre_index,'post':post_index},
        'projectile_pre_contact_velocity_m_s':pre_velocity.tolist(),'projectile_post_contact_velocity_m_s':post_velocity.tolist(),
        'projectile_speed_change_m_s':float(np.linalg.norm(post_velocity)-np.linalg.norm(pre_velocity)),
        'target_maximum_com_displacement_m':float(target_displacement.max()),
        'target_final_com_displacement_m':float(target_displacement[-1]),
        'target_maximum_capture_nonrigid_rms_m':float(max(target_residuals)),
        'target_final_shape_residual_m':float(target_residuals[-1]),
        'target_maximum_substep_nonrigid_rms_m':impact.get('maximum_target_nonrigid_rms_m'),
        'target_maximum_substep_local_displacement_m':impact.get('maximum_target_local_displacement_m'),
        'target_maximum_axis_compression_fraction':impact.get('maximum_target_axis_compression_fraction'),
        'target_maximum_tet_edge_shortening_fraction':impact.get('maximum_tet_edge_shortening_fraction'),
        'penetration_semantics':impact.get('penetration_semantics'),
        'target_minimum_tet_jacobian':substep.get('minimum_j'),'inverted_tets':substep.get('inverted_tets'),
        'minimum_sampled_surface_gap_m':impact.get('minimum_sampled_surface_gap_m'),
        'maximum_sampled_penetration_m':impact.get('maximum_sampled_penetration_m'),
        'maximum_rigid_speed_m_s':impact.get('maximum_rigid_speed_m_s'),
        'maximum_soft_nodal_speed_m_s':impact.get('maximum_soft_nodal_speed_m_s'),
        'target_positions_m':target_positions.tolist(),'projectile_positions_m':projectile_positions.tolist(),
        'target_linear_velocities_m_s':target_velocities.tolist(),'projectile_linear_velocities_m_s':projectile_velocities.tolist(),
        'interpretation':'Geometric contact and penetration are sampled diagnostics; no deformable solver impulse is inferred.'}
    passed=not any(item['status']=='fail' for item in checks)
    validation={'schema_version':'0.1.0','passed':passed,'checks':checks,'missing_required':[] if passed else ['physical_audit']}
    validate_schema(validation,'validation');write_json(out/'metrics.json',metrics);write_json(out/'physics_validation.json',validation)
    return validation

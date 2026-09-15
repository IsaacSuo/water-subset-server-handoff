"""Physics-only cache review for V03 dynamic rigid loading and removal."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .contract import artifact,validate_schema
from .io import inside,read_json,write_json
from .metrics import nonrigid_residual,recovery_time


def audit_v03(output):
    out=Path(output);ep=read_json(out/'episode.prepared.json');spec=ep['spec']
    fixture=read_json(out/'fixture.json');report=read_json(out/'native_report.json');index=read_json(out/'state/index.json')
    target_id=fixture['target_id'];load_id=fixture['load_id'];ids={target_id,load_id};checks=[]
    def check(name,ok,detail):checks.append({'name':name,'status':'pass' if ok else 'fail','detail':detail})

    interaction=report.get('substep_impact_audit') or {}
    start=spec['action_parameters']['load_start_time_s'];remove_time=start+spec['action_parameters']['load_duration_s']
    first_contact=interaction.get('geometric_contact_first_time_s');capture_hz=spec['timing']['capture_hz']
    reference_index=round(start*capture_hz);pre_remove_index=round(remove_time*capture_hz)
    contact_settle_time=min(remove_time,max(first_contact or start,start)+.5);settled_index=round(contact_settle_time*capture_hz)
    reference_frame=index['frames'][reference_index]
    with np.load(inside(out,reference_frame['geometries'][target_id]),allow_pickle=False) as data:
        reference_surface=data['surface_world_m'].copy()
    reference_height=float(np.ptp(reference_surface[:,2]))

    times=[];heights=[];target_errors=[];target_positions=[];target_velocities=[];load_positions=[];load_velocities=[]
    topologies={};tet_topology=None;sampled_inversions=0;target_minimum_z=float('inf')
    settled_surface=None;pre_remove_surface=None
    for frame_number,frame in enumerate(index['frames']):
        row=read_json(inside(out,frame['state']));validate_schema(row,'state');expected=frame_number/spec['timing']['capture_hz']
        if abs(row['time_s']-expected)>1e-8 or row['physics_step']!=round(expected*spec['timing']['physics_hz']):
            raise ValueError('State time misalignment')
        if set(row['objects'])!=ids or set(frame['geometries'])!=ids:raise ValueError('Mixed object set changed')
        for oid in ids:
            body=row['objects'][oid];path=frame['geometries'][oid]
            if artifact(out,path)!=body['geometry']:raise ValueError('Geometry checksum mismatch')
            with np.load(inside(out,path),allow_pickle=False) as data:
                if not all(np.isfinite(data[key]).all() for key in data.files):raise ValueError('Non-finite cache array')
                surface=data['surface_world_m'];faces=data['surface_triangles']
                if oid not in topologies:topologies[oid]=(surface.shape,faces.copy())
                shape,topology=topologies[oid]
                if surface.shape!=shape or not np.array_equal(faces,topology):raise ValueError(f'Surface topology changed: {oid}')
                if oid==target_id:
                    tets=data['simulation_tets']
                    if tet_topology is None:tet_topology=tets.copy()
                    if not np.array_equal(tets,tet_topology):raise ValueError('Target Tet topology changed')
                    heights.append(float(np.ptp(surface[:,2])));target_errors.append(nonrigid_residual(reference_surface,surface))
                    if frame_number==settled_index:settled_surface=surface.copy()
                    if frame_number==pre_remove_index:pre_remove_surface=surface.copy()
                    target_minimum_z=min(target_minimum_z,float(surface[:,2].min()))
                    sampled_inversions=max(sampled_inversions,int(body['metrics'].get('inverted_tets',0)))
        target=row['objects'][target_id];load=row['objects'][load_id]
        target_positions.append(target['position_m']);target_velocities.append(target['linear_velocity_m_s'])
        load_positions.append(load['position_m']);load_velocities.append(load['linear_velocity_m_s']);times.append(row['time_s'])

    expected=round(spec['timing']['duration_s']*spec['timing']['capture_hz'])+1
    check('complete_shared_time_axis',index['complete'] and len(times)==expected,f'2 objects, {len(times)}/{expected} frames')
    check('finite_fixed_topology',True,f'fixed rigid/soft surfaces and {len(tet_topology)} target Tets')
    check('no_ground_escape',target_minimum_z>-.02,f'target minimum surface z {target_minimum_z:.6g} m')
    check('no_sampled_inverted_tets',sampled_inversions==0,f'{sampled_inversions} sampled inversions')

    applications=report.get('action_applications',[]);kinds=[row.get('kind') for row in applications]
    actions_ok=(kinds==['release','remove_support'] and all(row.get('target')==load_id and row.get('applications')==1 for row in applications))
    check('load_release_and_removal_executed',actions_ok,str(applications))
    removals=report.get('actor_deactivations',[])
    check('load_actor_deactivated',len(removals)==1 and removals[0]['target']==load_id and abs(removals[0]['time_s']-remove_time)<1e-8,str(removals))

    contact_rows=0;contact_points=0;previous_step=0
    with (out/'contacts.jsonl').open() as stream:
        for line in stream:
            contact=json.loads(line);validate_schema(contact,'contact');step=contact['physics_step']
            if step<previous_step or abs(contact['time_s']-step/spec['timing']['physics_hz'])>1e-8:raise ValueError('Contact time misalignment')
            previous_step=step;contact_rows+=1;contact_points+=len(contact['points'])
    check('native_rigid_contact_stream',True,
          f'{contact_rows} records, {contact_points} points; not deformable impulse supervision')

    substep=report.get('substep_tet_audit') or {};steps=round(spec['timing']['duration_s']*spec['timing']['physics_hz'])
    check('every_step_tet_inversion_audit',substep.get('checked_steps')==steps and substep.get('inverted_tets')==0,
          f"{substep.get('checked_steps')} steps, minimum J {substep.get('minimum_j')}")
    finite_interaction=interaction.get('checked_steps')==steps and all(np.isfinite(interaction.get(key,np.nan)) for key in (
        'maximum_target_nonrigid_rms_m','maximum_target_local_displacement_m','maximum_target_axis_compression_fraction',
        'maximum_rigid_speed_m_s','maximum_soft_nodal_speed_m_s','minimum_sampled_surface_gap_m','maximum_sampled_penetration_m'))
    check('finite_every_step_interaction_diagnostics',finite_interaction,f"{interaction.get('checked_steps')} finite steps")
    check('geometric_load_contact_observed',first_contact is not None and first_contact<remove_time,
          f'first sampled contact {first_contact}, removal {remove_time}')
    checks.append({'name':'sampled_rigid_soft_penetration_measurement','status':'pass',
        'detail':f"diagnostic lower bound {interaction.get('maximum_sampled_penetration_m')} m; no universal pass threshold"})

    material=report.get('deformable_material_tensor_readback') or {}
    declared=next(obj['physics'] for obj in ep['inputs']['objects'] if obj['instance_id']==target_id)
    material_ok=(material.get('status')=='native' and material.get('count')==1 and
        abs(material.get('youngs_modulus_pa',-1)-declared['youngs_modulus_pa'])<=1e-5*declared['youngs_modulus_pa'] and
        abs(material.get('poissons_ratio',-1)-declared['poissons_ratio'])<=1e-6 and
        abs(material.get('dynamic_friction',-1)-declared['dynamic_friction'])<=1e-6)
    check('deformable_material_native_readback',material_ok,f'declared E={declared["youngs_modulus_pa"]}; native={material}')
    evidence=read_json(out/'capability_probes/soft_contact_impulse.json')
    checks.append({'name':'soft_contact_impulse','status':'unavailable' if evidence['status']=='unavailable' else 'fail','detail':evidence['reason']})

    times_array=np.asarray(times);target_positions=np.asarray(target_positions);load_positions=np.asarray(load_positions)
    errors=np.asarray(target_errors)
    active=np.flatnonzero((times_array>=start)&(times_array<=remove_time+1e-8))
    compression=np.maximum(0.,1-np.asarray(heights)/reference_height)
    sag=target_positions[reference_index,2]-target_positions[:,2]
    if settled_surface is None or pre_remove_surface is None:raise ValueError('Missing V03 hold comparison captures')
    hold_change=nonrigid_residual(settled_surface,pre_remove_surface)
    d=fixture['D_m'];recovery={}
    for fraction in (.005,.01):
        recovery[str(fraction)]={'threshold_m':fraction*d,'sustained_hold_s':.5,
            'time_from_removal_s':recovery_time(times_array,errors,remove_time,fraction*d)}
    metrics={'source':'derived_from_native_states_and_every_step_diagnostics','target_id':target_id,'load_id':load_id,
        'times_s':times,'load_start_time_s':start,'load_remove_time_s':remove_time,
        'geometric_contact_first_time_s':first_contact,'geometric_contact_last_time_s':interaction.get('geometric_contact_last_time_s'),
        'preload_reference_capture_index':reference_index,'preload_reference_time_s':float(times_array[reference_index]),
        'maximum_height_compression_fraction':float(compression[active].max()),
        'maximum_com_sag_m':float(sag[active].max()),'maximum_nonrigid_rms_m':float(errors[active].max()),
        'pre_remove_height_compression_fraction':float(compression[pre_remove_index]),
        'pre_remove_com_sag_m':float(sag[pre_remove_index]),
        'pre_remove_nonrigid_rms_m':float(errors[pre_remove_index]),
        'load_contact_retained_until_removal':bool(interaction.get('geometric_contact_last_time_s') is not None and
            interaction['geometric_contact_last_time_s']>=remove_time-1/spec['timing']['physics_hz']-1e-8),
        'maximum_substep_nonrigid_rms_m':interaction.get('maximum_target_nonrigid_rms_m'),
        'maximum_substep_local_displacement_m':interaction.get('maximum_target_local_displacement_m'),
        'maximum_tet_edge_shortening_fraction':interaction.get('maximum_tet_edge_shortening_fraction'),
        'hold_shape_change_m':hold_change,'final_shape_residual_m':float(errors[-1]),
        'recovery_thresholds_D':recovery,'minimum_tet_jacobian':substep.get('minimum_j'),'inverted_tets':substep.get('inverted_tets'),
        'minimum_sampled_surface_gap_m':interaction.get('minimum_sampled_surface_gap_m'),
        'maximum_sampled_penetration_m':interaction.get('maximum_sampled_penetration_m'),
        'target_positions_m':target_positions.tolist(),'load_positions_m':load_positions.tolist(),
        'target_linear_velocities_m_s':target_velocities,'load_linear_velocities_m_s':load_velocities,
        'equilibrium_shape_errors_m':errors.tolist(),'height_compression_fraction':compression.tolist(),
        'interpretation':'Loading is native gravity-driven rigid/deformable interaction. Geometric contact is sampled; soft impulse is unavailable and never estimated.'}
    passed=not any(item['status']=='fail' for item in checks)
    validation={'schema_version':'0.1.0','passed':passed,'checks':checks,'missing_required':[] if passed else ['physical_audit']}
    validate_schema(validation,'validation');write_json(out/'metrics.json',metrics);write_json(out/'physics_validation.json',validation)
    return validation

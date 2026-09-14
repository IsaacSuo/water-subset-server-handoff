"""Post-run cache inspection and measurements for V01 soft-body drop/rebound."""
from __future__ import annotations

from pathlib import Path

import numpy as np

from .contract import artifact,validate_schema
from .io import inside,read_json,write_json
from .metrics import box_penetration,nonrigid_residual,recovery_time


def audit_v01(output):
    out=Path(output);ep=read_json(out/'episode.prepared.json');spec=ep['spec'];fixture=read_json(out/'fixture.json')
    report=read_json(out/'native_report.json');index=read_json(out/'state/index.json');oid=spec['objects'][0]['instance_id']
    checks=[]
    def check(name,ok,detail):checks.append({'name':name,'status':'pass' if ok else 'fail','detail':detail})
    times=[];bodies=[];positions=[];velocities=[];heights=[];residuals=[];penetrations=[]
    surface_reference=None;surface_topology=None;simulation_topology=None;max_inverted=0
    for frame_number,frame in enumerate(index['frames']):
        row=read_json(inside(out,frame['state']));validate_schema(row,'state')
        expected=frame_number/spec['timing']['capture_hz']
        if abs(row['time_s']-expected)>1e-8 or row['physics_step']!=round(expected*spec['timing']['physics_hz']):
            raise ValueError('State time misalignment')
        body=row['objects'][oid]
        if artifact(out,frame['geometry'])!=body['geometry']:raise ValueError('Geometry checksum mismatch')
        with np.load(inside(out,frame['geometry']),allow_pickle=False) as data:
            if not all(np.isfinite(data[key]).all() for key in data.files):raise ValueError('Non-finite cache array')
            surface=data['surface_world_m'];faces=data['surface_triangles'];tets=data['simulation_tets'];tet_points=data['simulation_points_m']
            if surface_reference is None:
                surface_reference=surface.copy();surface_topology=faces.copy();simulation_topology=tets.copy()
            if surface.shape!=surface_reference.shape or not np.array_equal(faces,surface_topology) or not np.array_equal(tets,simulation_topology):
                raise ValueError('V01 topology changed')
            max_inverted=max(max_inverted,int(body['metrics'].get('inverted_tets',0)))
            positions.append(body['position_m']);velocities.append(body['linear_velocity_m_s']);heights.append(float(np.ptp(surface[:,2])))
            residuals.append(nonrigid_residual(surface_reference,surface));penetrations.append(box_penetration(surface,fixture['boxes'],frame['fixture_positions']))
        times.append(row['time_s']);bodies.append(body)
    times_array=np.asarray(times);positions=np.asarray(positions);velocities=np.asarray(velocities);heights=np.asarray(heights)
    expected=round(spec['timing']['duration_s']*spec['timing']['capture_hz'])+1
    check('complete_time_axis',index['complete'] and len(times)==expected,f'{len(times)}/{expected} frames')
    check('finite_fixed_topology',True,f'{len(times)} finite aligned states')
    check('no_sampled_inverted_tets',max_inverted==0,f'{max_inverted} inverted Tets in captured states')
    substep=report.get('substep_tet_audit')
    check('every_step_tet_inversion_audit',bool(substep) and substep['inverted_tets']==0,
          'missing every-step report' if not substep else f"{substep['checked_steps']} steps, minimum J {substep['minimum_j']:.6g}")
    check('no_ground_escape',min(body['metrics']['minimum_z_m'] for body in bodies)>-.02,
          f"minimum surface z {min(body['metrics']['minimum_z_m'] for body in bodies):.6g} m")
    check('free_fall_has_no_hidden_command',not read_json(out/'action.json')['commands'] and not report.get('action_applications'),
          'empty action command stream; initial elevation plus gravity')
    evidence=read_json(out/'capability_probes/soft_contact_impulse.json')
    checks.append({'name':'soft_contact_impulse','status':'unavailable' if evidence['status']=='unavailable' else 'fail','detail':evidence['reason']})
    material=report.get('deformable_material_tensor_readback') or {};expected_material=ep['inputs']['objects'][0]['physics']
    material_ok=(material.get('status')=='native' and material.get('count')==1 and
        abs(material.get('youngs_modulus_pa',-1)-expected_material['youngs_modulus_pa'])<=1e-5*expected_material['youngs_modulus_pa'] and
        abs(material.get('poissons_ratio',-1)-expected_material['poissons_ratio'])<=1e-6 and
        abs(material.get('dynamic_friction',-1)-expected_material['dynamic_friction'])<=1e-6)
    check('deformable_material_native_readback',material_ok,
          f"declared E={expected_material['youngs_modulus_pa']}, nu={expected_material['poissons_ratio']}, friction={expected_material['dynamic_friction']}; native={material}")

    vz=velocities[:,2];impact_candidates=np.flatnonzero((vz[:-1]<0)&(vz[1:]>=0))+1
    impact_index=int(impact_candidates[0]) if len(impact_candidates) else int(np.argmin(positions[:,2]))
    apex_candidates=np.flatnonzero((np.arange(len(vz)-1)>=impact_index)&(vz[:-1]>0)&(vz[1:]<=0))+1
    apex_index=int(apex_candidates[0]) if len(apex_candidates) else None
    rest_height=fixture['rest_height_m'];d=fixture['D_m']
    max_compression=float(1-heights.min()/rest_height)
    recovery=recovery_time(times,residuals,times[impact_index],.01*d)
    lateral=np.linalg.norm(positions[:,:2]-positions[0,:2],axis=1)
    values={'source':'derived_from_native_states','times_s':times,'positions_m':positions.tolist(),
        'linear_velocities_m_s':velocities.tolist(),'impact_time_s':times[impact_index],
        'impact_detection':'first captured vertical COM velocity transition from downward to nonnegative',
        'maximum_downward_speed_m_s':float(max(0,-vz.min())),'max_compression_fraction':max_compression,
        'minimum_height_m':float(heights.min()),'rebound_apex_time_s':None if apex_index is None else times[apex_index],
        'rebound_height_m':None if apex_index is None else float(positions[apex_index,2]-positions[impact_index,2]),
        'shape_recovery_time_s':recovery,'final_shape_residual_m':float(residuals[-1]),
        'maximum_nonrigid_rms_m':float(max(residuals)),'nonrigid_rms_m':list(map(float,residuals)),
        'maximum_lateral_com_displacement_m':float(lateral.max()),
        'final_lateral_com_displacement_m':float(lateral[-1]),
        'max_vertex_fixture_overlap_m':float(max(penetrations)),'inverted_tets':max_inverted,
        'final_position_m':positions[-1].tolist(),'final_speed_m_s':float(np.linalg.norm(velocities[-1])),
        'interpretation':'Impact/rebound are geometric and kinematic measurements; no soft contact impulse is inferred.'}
    passed=not any(item['status']=='fail' for item in checks)
    validation={'schema_version':'0.1.0','passed':passed,'checks':checks,'missing_required':[] if passed else ['physical_audit']}
    validate_schema(validation,'validation');write_json(out/'metrics.json',values);write_json(out/'physics_validation.json',validation)
    return validation

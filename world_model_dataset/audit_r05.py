"""Physics-only cache and contact-graph review for R05 stack collapse."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .contract import artifact,validate_schema
from .io import inside,read_json,write_json


def audit_r05(output):
    out=Path(output);ep=read_json(out/'episode.prepared.json');spec=ep['spec']
    fixture=read_json(out/'fixture.json');report=read_json(out/'native_report.json');index=read_json(out/'state/index.json')
    ids=fixture['stack_order'];fixture_ids=[box['id'] for box in fixture['boxes']];checks=[]
    def check(name,ok,detail):checks.append({'name':name,'status':'pass' if ok else 'fail','detail':detail})

    times=[];states={oid:[] for oid in ids};references={};topologies={};minimum_z=float('inf')
    for frame_number,frame in enumerate(index['frames']):
        row=read_json(inside(out,frame['state']));validate_schema(row,'state');expected=frame_number/spec['timing']['capture_hz']
        if abs(row['time_s']-expected)>1e-8 or row['physics_step']!=round(expected*spec['timing']['physics_hz']):
            raise ValueError('State time misalignment')
        if set(row['objects'])!=set(ids) or set(frame['geometries'])!=set(ids):raise ValueError('Object set changed')
        for oid in ids:
            body=row['objects'][oid];path=frame['geometries'][oid]
            if artifact(out,path)!=body['geometry']:raise ValueError('Geometry checksum mismatch')
            with np.load(inside(out,path),allow_pickle=False) as data:
                if not all(np.isfinite(data[key]).all() for key in data.files):raise ValueError('Non-finite cache array')
                vertices=data['surface_world_m'];faces=data['surface_triangles']
                if oid not in references:references[oid]=vertices.shape;topologies[oid]=faces.copy()
                if vertices.shape!=references[oid] or not np.array_equal(faces,topologies[oid]):raise ValueError(f'Topology changed: {oid}')
                minimum_z=min(minimum_z,float(vertices[:,2].min()))
            states[oid].append(body)
        times.append(row['time_s'])

    expected=round(spec['timing']['duration_s']*spec['timing']['capture_hz'])+1
    check('complete_time_axis',index['complete'] and len(times)==expected,f'{len(ids)} objects, {len(times)}/{expected} frames')
    check('finite_fixed_topology',True,f'{len(ids)} fixed rigid surfaces')
    check('no_ground_escape',minimum_z>-.02,f'minimum surface z {minimum_z:.6g} m')
    applications=report.get('action_applications',[])
    check('support_removal_executed',len(applications)==1 and applications[0]['kind']=='remove_support' and
          applications[0]['target']==fixture['support_id'] and applications[0]['applications']==1,str(applications))

    entity_roots={name:'/World/'+name for name in [*ids,*fixture_ids]}
    def entities(paths):
        return [name for name,root in entity_roots.items() if any(path==root or path.startswith(root+'/') for path in paths)]
    active=set();timeline=[];all_pairs=set();point_count=0;interbody_points=0;pre_graph=None
    previous_step=-1;minimum_separation=0.;remove_time=spec['action_parameters']['remove_time_s']
    remove_step=round(remove_time*spec['timing']['physics_hz'])
    with (out/'contacts.jsonl').open() as stream:
        for line in stream:
            row=json.loads(line);validate_schema(row,'contact');step=row['physics_step']
            if step<previous_step or abs(row['time_s']-step/spec['timing']['physics_hz'])>1e-8:raise ValueError('Contact time mismatch')
            if pre_graph is None and step>=remove_step:pre_graph=set(active)
            previous_step=step;paths=[row[key] for key in ('actor0','actor1','collider0','collider1')];touched=entities(paths)
            if len(touched)>=2:
                pair=tuple(sorted(touched[:2]));before=set(active);all_pairs.add(pair)
                if row['event_type'].endswith('CONTACT_LOST'):active.discard(pair)
                else:active.add(pair)
                if before!=active:timeline.append({'time_s':row['time_s'],'event_type':row['event_type'],'edge':list(pair),'active_edges':[list(x) for x in sorted(active)]})
                if pair[0] in ids and pair[1] in ids:interbody_points+=len(row['points'])
            point_count+=len(row['points'])
            for point in row['points']:minimum_separation=min(minimum_separation,float(point['separation_m']))
    if pre_graph is None:pre_graph=set(active)
    # PhysX does not reliably emit CONTACT_LOST when a collider is disabled.
    # Preserve sleeping contacts from the native event state, but explicitly
    # remove the support edge whose collision was disabled by the action.
    final_graph={pair for pair in active if fixture['support_id'] not in pair}
    check('native_contact_graph_stream',point_count>0 and interbody_points>0,
          f'{point_count} native points, {interbody_points} interbody points, {len(all_pairs)} observed edges')

    times_array=np.asarray(times);reference_index=int(np.flatnonzero(times_array<=remove_time+1e-8)[-1])
    positions={oid:np.asarray([body['position_m'] for body in states[oid]]) for oid in ids}
    velocities={oid:np.asarray([body['linear_velocity_m_s'] for body in states[oid]]) for oid in ids}
    orientations={oid:np.asarray([body['orientation_xyzw'] for body in states[oid]]) for oid in ids}
    post=np.flatnonzero(times_array>=remove_time);pre=np.flatnonzero((times_array>=max(0.,remove_time-.25))&(times_array<=remove_time))
    displacement=np.column_stack([np.linalg.norm(positions[oid]-positions[oid][reference_index],axis=1) for oid in ids])
    angle=np.empty((len(times),len(ids)))
    for column,oid in enumerate(ids):
        dots=np.abs(orientations[oid]@orientations[oid][reference_index]);angle[:,column]=2*np.arccos(np.clip(dots,-1.,1.))
    moved=(displacement>.1*fixture['D_m'])|(angle>np.deg2rad(10))
    onset_candidates=post[np.any(moved[post],axis=1)];onset=None if not len(onset_candidates) else float(times_array[onset_candidates[0]])
    height_drop={oid:float((positions[oid][reference_index,2]-positions[oid][post,2].min())/fixture['D_m']) for oid in ids}
    speed=np.column_stack([np.linalg.norm(velocities[oid],axis=1) for oid in ids])
    final_centres=np.asarray([positions[oid][-1] for oid in ids]);extent=np.ptp(final_centres,axis=0)/fixture['D_m']
    pre_objects={tuple(edge) for edge in pre_graph if edge[0] in ids and edge[1] in ids}
    final_objects={tuple(edge) for edge in final_graph if edge[0] in ids and edge[1] in ids}
    metrics={'source':'derived_from_native_states_and_contact_events','object_ids':ids,'times_s':times,
        'remove_time_s':remove_time,'reference_capture_index':reference_index,
        'pre_removal_contact_graph':[list(edge) for edge in sorted(pre_graph)],
        'final_contact_graph':[list(edge) for edge in sorted(final_graph)],
        'contact_graph_snapshot_semantics':'native FOUND/PERSIST/LOST state; support edges explicitly removed after declared collision disable',
        'pre_removal_interbody_graph':[list(edge) for edge in sorted(pre_objects)],
        'final_interbody_graph':[list(edge) for edge in sorted(final_objects)],
        'contact_graph_edit_count':len(pre_graph^final_graph),'interbody_graph_edit_count':len(pre_objects^final_objects),
        'contact_graph_timeline':timeline,'observed_contact_edges':[list(edge) for edge in sorted(all_pairs)],
        'native_contact_points':point_count,'native_interbody_contact_points':interbody_points,
        'minimum_native_contact_separation_m':minimum_separation,
        'collapse_onset_time_s':onset,'collapse_onset_thresholds':{'com_displacement_D':.1,'orientation_change_deg':10.},
        'maximum_height_drop_D':max(height_drop.values()),'height_drop_D_by_object':height_drop,
        'maximum_orientation_change_rad':float(angle[post].max()),
        'pre_removal_maximum_speed_m_s':float(speed[pre].max()),'maximum_speed_m_s':float(speed[post].max()),
        'final_maximum_speed_m_s':float(speed[-1].max()),'final_spatial_extent_D':extent.tolist(),
        'final_positions_m':{oid:positions[oid][-1].tolist() for oid in ids},
        'final_orientations_xyzw':{oid:orientations[oid][-1].tolist() for oid in ids},
        'interpretation':'Collapse onset thresholds and graph edits are derived measurements. Native per-point rigid impulses remain in contacts.jsonl.'}
    passed=not any(item['status']=='fail' for item in checks)
    validation={'schema_version':'0.1.0','passed':passed,'checks':checks,'missing_required':[] if passed else ['physical_audit']}
    validate_schema(validation,'validation');write_json(out/'metrics.json',metrics);write_json(out/'physics_validation.json',validation)
    return validation

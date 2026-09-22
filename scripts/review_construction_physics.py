"""Read completed constructor trials; never prepare, simulate or alter a cache."""
import argparse
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from shapely import intersects_xy

from world_model_dataset.causal_loader import open_episode
from world_model_dataset.experiment_geometry import Geometry
from world_model_dataset.experiment_review import verify_alignment
from world_model_dataset.io import read_json, write_json, file_hash


def review(folder, request_path, output):
    folder=Path(folder); request=read_json(request_path)
    output.mkdir(parents=True,exist_ok=False)
    doc=read_json(folder/'generated/experiment.json')
    construction=read_json(folder/'generated/construction.json')
    ledger=read_json(folder/'execution/workflow.json'); job=ledger['jobs']['baseline']
    ep_path=Path(job['stages']['package']['result']['episode'])
    ep=open_episode(ep_path,require_complete=False); rows=list(ep.states())
    result=dict(episode=str(ep_path),request=str(request_path),request_sha256=file_hash(request_path),
        construction_sha256=file_hash(folder/'generated/construction.json'),
        alignment=verify_alignment(ep,doc['timing']),physics_runs=0 if 'cache' in job['stages']['prepare']['result'] else 1,
        source_scene_unchanged=True,post_t0_manual_state_changes=False,
        configuration_check='passed',execution='completed',
        qualification='human review pending; fixed-configuration state prediction candidate only',
        convergence='not tested',real_material_calibration='not established')
    preserved=[]
    for path,checksum in construction['source_pins'].items():
        if file_hash(path)!=checksum:
            snapshot=Path(job['attempts'][0])/'sources'/(checksum+Path(path).suffix)
            assert file_hash(snapshot)==checksum,path
            preserved.append(dict(original_path=path,snapshot=str(snapshot),sha256=checksum))
    result['historical_sources_verified_from_snapshot']=preserved
    run=Path(job['stages']['prepare']['result']['run'])
    if doc['backend']['kind']=='rigid':
        authority=read_json(ep_path/'native_report.json')
        assert authority['post_t0_direct_state_writes']==0
    else:
        authority=read_json(run/'execution.json')
        assert authority['post_t0_state_authority']=='native solver only'
    for body in ep.manifest['system']['bodies']:
        if body['physics_kind']=='static':
            oid=body['instance_id']
            assert all(row['body_states'][oid]==rows[0]['body_states'][oid] for row in rows)
    if doc['backend']['kind']=='rigid':
        oid=doc['input']['participants'][0]['id']; resolved=ep.resolved_inputs()
        with np.load(ep.record_path(resolved['bodies'][oid]['geometry']['mesh'])) as data:
            vertices=data['vertices']
        positions=np.asarray([r['body_states'][oid]['position_m'] for r in rows])
        bounds=[]
        for row in rows:
            s=row['body_states'][oid]
            v=Rotation.from_quat(s['orientation_xyzw']).apply(vertices)+s['position_m']
            bounds.append([v.min(0),v.max(0)])
        bounds=np.asarray(bounds); throat=construction['calculations']['throat_x_m']
        times=np.array([r['time_s'] for r in rows]); roi=np.array(doc['scene']['region_bounds_m'])
        entered=bounds[:,1,0]>=throat; cleared=bounds[:,0,0]>throat
        lateral=(bounds[:,0,1]>=roi[0,1])&(bounds[:,1,1]<=roi[1,1])
        contacts=list(ep.contacts());pairs={};separation=[]
        for row in contacts:
            key=' / '.join(row['actor_ids'])
            pairs[key]=pairs.get(key,0)+1
            if 'separation_m' in row:separation.append(row['separation_m'])
        result['measurement']=dict(initial_position_m=positions[0].tolist(),final_position_m=positions[-1].tolist(),
            position_bounds_m=[positions.min(0).tolist(),positions.max(0).tolist()],throat_x_m=throat,
            first_leading_edge_at_throat_s=float(times[entered][0]) if entered.any() else None,
            first_trailing_edge_beyond_throat_s=float(times[cleared][0]) if cleared.any() else None,
            lateral_roi_preserved=bool(lateral.all()),
            contact_rows=len(contacts),contact_pair_counts=pairs,
            minimum_reported_separation_m=min(separation) if separation else None,
            interpretation='crossed selected throat' if (cleared & lateral).any() else
                'reached throat but did not clear' if entered.any() else 'did not reach selected throat',
            limits='throat and original ROI tests use actual posed mesh; passage past throat is not proof of traversing every downstream obstacle')
        write_json(output/'trajectory_bounds.json',dict(time_s=times.tolist(),bounds_m=bounds.tolist()))
    else:
        geometry=Geometry(doc['scene']);height,surface=geometry.support(request['support_group'])
        edge=construction['calculations']['world_edge_point_m'][0]
        samples=[];rest=None;edges=None;max_ratio=0.;max_speed=0.
        for row,g in ep.geometries('cloth'):
            v=g['surface_world_m'];f=g['surface_triangles']
            if rest is None:
                rest=v.copy();free_ids=np.flatnonzero(rest[:,0]>edge)
                edges=np.unique(np.sort(np.concatenate([f[:,[0,1]],f[:,[1,2]],f[:,[2,0]]]),axis=1),axis=0)
                rest_lengths=np.linalg.norm(rest[edges[:,0]]-rest[edges[:,1]],axis=1)
            ratio=np.linalg.norm(v[edges[:,0]]-v[edges[:,1]],axis=1)/rest_lengths
            max_ratio=max(max_ratio,float(ratio.max()))
            speed=np.linalg.norm(g['surface_nodal_velocities_m_s'],axis=1)
            max_speed=max(max_speed,float(speed.max()))
            inside=intersects_xy(surface,v[:,0],v[:,1])
            near=inside & (abs(v[:,2]-height)<=.004)
            samples.append(dict(time_s=row['time_s'],bounds_m=[v.min(0).tolist(),v.max(0).tolist()],
                projected_support_node_fraction=float(inside.mean()),near_top_node_fraction=float(near.mean()),
                minimum_z_over_support_m=float(v[inside,2].min()) if inside.any() else None,
                original_free_nodes_mean_z_m=float(v[free_ids,2].mean()),
                original_free_nodes_min_z_m=float(v[free_ids,2].min()),
                peak_speed_m_s=float(speed.max()),rms_speed_m_s=float(np.sqrt(np.mean(speed**2)))))
        result['measurement']=dict(support_height_m=height,vertices=len(rest),frames=len(samples),
            first=samples[0],final=samples[-1],maximum_edge_length_ratio=max_ratio,peak_nodal_speed_m_s=max_speed,
            last_half_second_peak_speed_m_s=max(s['peak_speed_m_s'] for s in samples if s['time_s']>=samples[-1]['time_s']-.5),
            contact_force='unavailable',attachment_reaction='unavailable',
            limits='near-top nodes use a 4 mm geometric band, not solver contact or force truth; edge stretch and velocity are diagnostics, not convergence')
        write_json(output/'cloth_measurements.json',dict(samples=samples))
    write_json(output/'physical_review.json',result)
    print(result)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--folder',type=Path,required=True)
    p.add_argument('--request',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();review(a.folder,a.request,a.output)

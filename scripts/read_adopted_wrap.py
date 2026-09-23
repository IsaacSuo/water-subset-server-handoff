"""Read adopted wrap through the unified API; no solver, render or catalog writes."""
import argparse
from pathlib import Path
import numpy as np
from world_model_dataset.causal_loader import open_episode
from world_model_dataset.experiment_adapters import physical_config
from world_model_dataset.experiment_review import verify_alignment
from world_model_dataset.io import read_json,write_json,file_hash
from scripts.pin_wrap_delivery import verify_maps


def review(folder):
    doc=read_json(folder/'generated/experiment.json');workflow=read_json(folder/'execution/workflow.json')
    job=workflow['jobs']['baseline'];binding=job['stages']['prepare']['result']['cache']
    source=Path(binding['source_run']);ep=open_episode(job['stages']['package']['result']['episode'],require_complete=False)
    alignment=verify_alignment(ep,doc['timing']);rows=list(ep.states());resolved=ep.resolved_inputs()
    current=read_json(folder/'generated/backend_input.json');original=read_json(source/'requested_config.json')
    if physical_config(current)!=physical_config(original):raise ValueError('Authored physical inputs changed')
    cache_origin=read_json(ep.root.parent/'cache_origin.json')
    if cache_origin['physics_rerun'] or cache_origin['binding']!=binding:raise ValueError('External origin mismatch')
    raw=ep.record_path(resolved['raw_native'])
    if file_hash(raw)!=file_hash(source/'native/states.npz'):raise ValueError('Native trajectory changed')
    for name in ('contact_candidates.jsonl','runtime_sources.json','backend_sources.zip'):
        if file_hash(ep.root/'native'/name)!=file_hash(source/'native'/name):raise ValueError('Native diagnostic/source changed: '+name)
    mapped=verify_maps(ep.root/'inputs')
    carrier=next(oid for oid,state in rows[0]['body_states'].items() if 'geometry' in state and 'topology' in state)
    geometries=list(ep.geometries(carrier));topology=ep.soft_topology(carrier)
    construction=read_json(folder/'generated/construction.json');identity=construction['support_identity']
    rod=identity['original_object'];start=rod['triangle_start'];end=start+rod['triangle_count']
    with np.load(ep.root/'inputs'/(identity['group']+'_face_mapping.npz')) as m:
        ids=m['backend_to_original_face'];mask=(ids>=start)&(ids<end)
        np.testing.assert_array_equal(ids[mask],np.arange(start,end))
        np.testing.assert_array_equal(m['backend_to_object'][mask],np.full(end-start,identity['object_index']))
    with np.load(raw) as z:
        center=np.array(construction['calculations']['rod_center_m']);axis=np.array(construction['calculations']['rod_axis'])
        across=np.cross(axis,[0,0,1]);last=z['centerline'][-1];projected=(last-center)@across;heights=last[:,2]-center[2]
        final=dict(centerline_cross_rod_sides=bool(projected.min()<0<projected.max()),both_endpoints_below_rod=bool(heights[0]<0 and heights[-1]<0),centerline_above_rod=bool(heights.max()>0))
        gap=float(z['joint_endpoint_gap'].max())
    physical=read_json(source/'physical_review.json')
    if physical['native_sha256']!=file_hash(raw):raise ValueError('Physical review belongs to another trajectory')
    if any(final[k]!=physical['final_geometry'][k] for k in final):raise ValueError('Final geometry review disagrees')
    import json
    with (ep.root/'native/contact_candidates.jsonl').open() as stream:
        steps=[json.loads(line)['step'] for line in stream]
    if len(steps)!=960 or len(set(steps))!=960:raise ValueError('Incomplete candidate step log')
    return dict(source_revision='7011d1771058b19eef1a0d3cb93c74a1e6952c3f',external_run=str(source),
        entry_episode=str(ep.root),manifest_sha256=file_hash(ep.root/'episode.json'),native_sha256=file_hash(raw),
        physics_executed_by='material branch wrap_original_v2',entry_physics_rerun=False,stages=list(job['stages']),
        authored_physical_inputs_equal=True,alignment=alignment,geometry_carrier=carrier,native_geometry_frames=len(geometries),
        topology_fields=list(topology),collision_maps=mapped,selected_rod_original_faces=list(range(start,end)),
        final_geometry=final,max_joint_gap_m=gap,sampled_surface_overlap_m=-physical['minimum_rod_clearance_m'],
        candidate_steps=len(steps),rod_candidate_review=physical['rod_contact_candidates'],
        existing_physical_review=dict(path=str(source/'physical_review.json'),sha256=file_hash(source/'physical_review.json')),
        conclusion='This example retains a single U-shaped wrap around the selected original rod',
        limits=physical['limitations'],contact_force='unavailable',attachment_reaction='unavailable',rope_native_tension='unavailable',
        training_admission=False,use_qualification='mainline_review_pending')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--folder',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();write_json(a.output,review(a.folder))

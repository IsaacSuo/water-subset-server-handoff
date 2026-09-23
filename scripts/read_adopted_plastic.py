"""CPU-only adopted plastic state/internal-field/contact reading and regression check."""
import argparse
from pathlib import Path
import numpy as np
from world_model_dataset.causal_loader import open_episode
from world_model_dataset.experiment_adapters import physical_config
from world_model_dataset.experiment_review import verify_alignment
from world_model_dataset.io import read_json,write_json,file_hash


def review(folder):
    doc=read_json(folder/'generated/experiment.json');job=read_json(folder/'execution/workflow.json')['jobs']['baseline']
    ep=open_episode(job['stages']['package']['result']['episode'],require_complete=False)
    delivery=read_json(folder/'backend_delivery.json');source=Path(delivery['cases']['box']['external_run'])
    origin=read_json(ep.root.parent/'import_origin.json')
    if origin['physics_rerun'] or origin['external_run']!=str(source):raise ValueError('External execution origin mismatch')
    for name,checksum in origin['files_sha256'].items():
        # Original source files remain bound even when package context intentionally only copies inputs/native.
        if file_hash(source/name)!=checksum:raise ValueError('External evidence changed: '+name)
    resolved=ep.resolved_inputs();raw=ep.record_path(resolved['raw_native'])
    if file_hash(raw)!=delivery['cases']['box']['native_sha256']:raise ValueError('Adopted trajectory changed')
    old=read_json('output/layout_completion_v1/plastic/generated/backend_input.json')
    current=read_json(folder/'generated/backend_input.json')
    if physical_config(old)!=physical_config(current):raise ValueError('Original authored physical layout changed')
    fields={};frames=0;initial=None;last=None
    for row,g in ep.geometries('mpm_block'):
        if initial is None:initial=g['particle_world_m']
        last=g;frames+=1
        for key,value in g.items():fields[key]=dict(shape=list(value.shape),dtype=str(value.dtype))
    topology=ep.soft_topology('mpm_block')
    impulses=np.zeros(3);active=[];count=0;intervals=0
    for r in ep.material_contact_diagnostics():
        intervals+=1;count+=len(r['collider_ids'])
        impulses+=r['impulse_on_colliders_Ns'].astype(float).sum(0)
        if len(r['collider_ids']):active.append(r['physics_step'])
        if r['supervision_admitted'] or r['force_truth']:raise ValueError('Diagnostic upgraded to force truth')
    evidence=Path(delivery['cases']['box']['artifact_manifest']).parent
    box=read_json(evidence/'box_result.json');regression=read_json(evidence/'regression_result.json')
    if box['states_sha256']!=file_hash(raw):raise ValueError('Box review is for another raw cache')
    reg=Path(delivery['cases']['regression']['external_run'])
    oldreg=Path(regression['reference'])
    if regression['states_sha256']!=file_hash(reg/'native/states.npz'):raise ValueError('Regression review is for another cache')
    equal={}
    with np.load(reg/'native/states.npz') as a,np.load(oldreg/'native/states.npz') as b:
        for key in sorted(set(a.files)&set(b.files)):
            x,y=a[key],b[key];equal[key]=x.shape==y.shape and x.dtype==y.dtype and x.tobytes()==y.tobytes()
    if len(equal)!=15 or not all(equal.values()):raise ValueError('Expected 15 bitwise equal regression arrays')
    if frames!=385 or intervals!=384 or len(active)!=box['contact']['active_steps']:raise ValueError('Incomplete physical state/contact intervals')
    np.testing.assert_allclose(impulses,box['contact']['total_impulse_on_colliders_Ns'],atol=1e-7,rtol=1e-6)
    return dict(actual_code_revision=delivery['source_revision'],delivery_revision=delivery['delivery_revision'],
        source_material_run=str(source),entry_episode=str(ep.root),manifest_sha256=file_hash(ep.root/'episode.json'),
        native_sha256=file_hash(raw),physics_executed_by='material branch plastic_zero_support_box_v2',entry_new_physics_runs=0,
        stages=list(job['stages']),original_authored_layout_equal=True,prepared_physics_binding=job['stages']['prepare']['result']['cache'],
        alignment=verify_alignment(ep,doc['timing']),geometry_frames=frames,geometry_fields=fields,topology=topology,
        initial_particle_span_m=np.ptp(initial,axis=0).tolist(),final_particle_span_m=np.ptp(last['particle_world_m'],axis=0).tolist(),
        final_Jp_range=[float(last['material_Jp'].min()),float(last['material_Jp'].max())],
        contact_diagnostics=dict(intervals=intervals,records=count,active_steps=len(active),first_active_step=min(active),
            total_impulse_on_colliders_Ns=impulses.tolist(),semantics='native impulse on collider; not admitted contact/force supervision'),
        zero_support_events=box['zero_support_events'],box_residuals=box['solver'],
        regression=dict(external_run=str(reg),compatibility_only=True,states=289,steps=288,bitwise_equal_arrays=equal,
            residuals=regression['solver'],zero_support_events=regression['zero_support_events'],native_sha256=file_hash(reg/'native/states.npz')),
        observations='unavailable; no rendering requested',quality_limits=box['quality_limits'],
        scope='fixed Q1/P1d/GS, static subgrid contacts, valid supported two-node extrapolation ray; unsupported paths reject',
        nonaffine_boundary_and_local_contact_bias_not_excluded=True,training_admission=False,use_qualification='mainline_review_pending')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--folder',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();write_json(a.output,review(a.folder))

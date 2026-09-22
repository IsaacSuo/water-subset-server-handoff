"""Compare constructed free/blocked/low-force cases through the unified reader."""
import copy,argparse
from pathlib import Path
import numpy as np
from world_model_dataset.io import read_json,write_json,file_hash
from world_model_dataset.causal_loader import open_episode


def compare(folders):
    result={};reference=None;initial=None
    for name,folder in folders.items():
        doc=read_json(folder/'generated/experiment.json');physical=copy.deepcopy(doc['input'])
        if name=='blocked':physical.pop('opposing_fixture')
        if name=='low_force':physical['gripper']['max_force_n']=4.
        environment=copy.deepcopy(doc['scene']);environment['source_records'].pop('construction',None)
        invariant=dict(input=physical,scene=environment,timing=doc['timing'],observations=doc['observations'],backend=doc['backend'])
        if reference is None:reference=invariant
        if invariant!=reference:raise ValueError(name+': unintended nonintervention configuration difference')
        job=read_json(folder/'execution/workflow.json')['jobs']['baseline'];ep=open_episode(job['stages']['observe']['result']['episode'])
        rows=list(ep.states());last=rows[-1];p0=rows[0]['body_states']['cloth']['geometry'];p1=last['body_states']['cloth']['geometry']
        with np.load(ep.record_path(p0)) as z:a=z['surface_world_m'].astype(float);faces=z['surface_triangles']
        with np.load(ep.record_path(p1)) as z:b=z['surface_world_m'].astype(float);velocity=z['surface_nodal_velocities_m_s'].astype(float)
        if initial is None:initial=a
        if not np.array_equal(initial,a):raise ValueError(name+': actual initial cloth nodes differ')
        edges=np.unique(np.sort(np.concatenate([faces[:,[0,1]],faces[:,[1,2]],faces[:,[2,0]]]),axis=1),axis=0)
        lengths=lambda points:np.linalg.norm(points[edges[:,1]]-points[edges[:,0]],axis=1)
        strain=lengths(b)/lengths(a)-1;delta=b-a;shape_delta=delta-delta.mean(0)
        g=np.array([row['body_states']['Gripper']['position_m'] for row in rows]);eff=list(ep.actuator_efforts('gripper_impedance'))
        force=np.array([row['force_world_n'][0] for row in eff]);ctrl=ep.manifest['control_program']['controllers'][0]['controller_id']
        result[name]=dict(episode=str(ep.root) if hasattr(ep,'root') else job['stages']['observe']['result']['episode'],
            input_sha256=file_hash(folder/'generated/experiment.json'),states=len(rows),commands=len(list(ep.controls())),
            actual_records=len(list(ep.actuator_states(ctrl))),effort_records=len(eff),youngs_modulus_Pa=doc['input']['material']['youngs_modulus_Pa'],
            gripper_displacement_m=(g[-1]-g[0]).tolist(),cloth_centroid_displacement_m=delta.mean(0).tolist(),
            translation_removed_displacement_rms_m=float(np.sqrt(np.mean(np.sum(shape_delta**2,axis=-1)))),
            final_edge_strain_percentiles=np.quantile(strain,[0,.5,.95,1]).tolist(),final_z_range_m=float(np.ptp(b[:,2])),
            final_nodal_speed_rms_m_s=float(np.sqrt(np.mean(np.sum(velocity**2,axis=-1)))),
            applied_force_x_range_n=[float(force.min()),float(force.max())],cap_saturation_fraction=float((abs(force)>=doc['input']['gripper']['max_force_n']*.999).mean()),
            contact_force='unavailable',attachment_reaction='unavailable')
    return dict(nonintervention_configuration_equal=True,actual_initial_nodes_identical=True,cases=result,
        limits='edge strain is native surface-edge geometry; translation-removed displacement also includes rotation/bending; applied force is not contact/attachment reaction',training_admission=False)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,default=Path('output/layout_closure_v2/cloth_comparison.json'));args=parser.parse_args()
    root=Path('output/layout_closure_v2')
    r=compare(dict(free=Path('output/layout_completion_v1/cloth_drag'),blocked=root/'cloth_blocked',low_force=root/'cloth_low_force'))
    write_json(args.output,r);print(r)

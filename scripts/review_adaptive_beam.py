"""Measure native phases relative to the unloaded calibration, not command names."""
import argparse
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from world_model_dataset.io import read_json,write_json,file_hash


def review(folder):
    job=read_json(folder/'execution/workflow.json')['jobs']['baseline']
    raw=Path(job['stages']['prepare']['result']['run'])/'native/frames.npz'
    doc=read_json(folder/'generated/experiment.json');cfg=doc['input']
    calibration=read_json(folder/'generated/construction.json')['calculations']['calibration']
    probe=Path(calibration['episode'])/'native/frames.npz'
    with np.load(raw) as a,np.load(probe) as p:
        t=a['time'];pose=a['plate_pose'];schedule=np.array(cfg['plate']['schedule']);warm,load_end,hold_end,withdraw_end=schedule[1:-1,0]
        local=np.einsum('fvi,fij->fvj',a['collision_world']-pose[:,None,:3],Rotation.from_quat(pose[:,3:]).as_matrix())
        q=abs(local)-np.array(cfg['plate']['size_m'])/2
        gap=np.linalg.norm(np.maximum(q,0),axis=-1)+np.minimum(q.max(-1),0)
        near=((q.argmax(-1)==2)&(local[:,:,2]<0)&(gap<=.004)).any(1)
        tip=a['rest_x'][:,0]>a['rest_x'][:,0].max()-.02;z=a['x'][:,tip,2].mean(1)
        matching_topology=np.array_equal(a['rest_x'],p['rest_x']) and np.array_equal(a['tets'],p['tets'])
        hold=(t>=load_end)&(t<hold_end);after=t>=withdraw_end
        before=(t>=warm-.5)&(t<=warm)
        low=float(np.mean(z[hold]));baseline=float(np.mean(z[before]));final=float(np.mean(z[t>=t[-1]-.5]))
        # Read applied effort from native commands; not a contact/attachment force.
        commands=a['effort_steps'];held_commands=commands[(commands[:,0]>=load_end)&(commands[:,0]<hold_end)]
        report=dict(raw_native_sha256=file_hash(raw),calibration_native_sha256=file_hash(probe),matching_native_topology=matching_topology,
            phases_s=dict(warmup_end=warm,load_end=load_end,hold_end=hold_end,withdraw_end=withdraw_end,end=float(t[-1])),
            hold_proximity_fraction=float(near[hold].mean()),hold_every_sample_proximity=bool(near[hold].all()),
            hold_minimum_signed_node_gap_m=float(gap[hold].min()),hold_maximum_nearest_node_gap_m=float(gap[hold].min(1).max()),
            actual_postwithdraw_minimum_gap_m=float(gap[after].min()),
            actual_preload_minimum_gap_m=float(gap[t<warm].min()),
            first_loading_proximity_s=float(t[(t>=warm)&near][0]) if ((t>=warm)&near).any() else None,
            hold_applied_force_z_range_n=[float(held_commands[:,3].min()),float(held_commands[:,3].max())],
            hold_applied_force_z_mean_n=float(held_commands[:,3].mean()),
            hold_plate_peak_speed_m_s=float(np.linalg.norm(a['plate_velocity'][hold,:3],axis=-1).max()),
            tip_z_before_load_hold_final_m=[baseline,low,final],additional_hold_tip_deflection_m=baseline-low,
            postwithdraw_tip_rebound_m=final-low,residual_tip_deflection_m=baseline-final,
            attachment_max_displacement_m=float(np.linalg.norm(a['x'][:,a['attachment_node_ids']]-a['rest_x'][a['attachment_node_ids']],axis=-1).max()),
            contact_force='unavailable',attachment_reaction='unavailable',training_admission=False,
            limits='sampled native collision-node proximity and deformation; actuator effort is external applied force, not contact force; no convergence claim')
        report['native_fields']=a.files
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--folder',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();r=review(a.folder);write_json(a.output,r);print(r)

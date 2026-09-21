"""Cache-only paired beam diagnostics, bound to the two immutable state streams."""
import argparse
import copy
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .causal_loader import open_episode
from .io import write_json


def review_pair(load_path, hold_path, output):
    episodes=[open_episode(p,require_complete=False) for p in (load_path,hold_path)]
    resolved=[ep.resolved_inputs() for ep in episodes]
    configs=[copy.deepcopy(r['config']) for r in resolved]
    schedules=[c['plate'].pop('schedule') for c in configs]
    if configs[0]!=configs[1] or schedules[0]==schedules[1]:
        raise ValueError('Beam comparison must change only plate.schedule')
    data=[]
    for ep,r in zip(episodes,resolved):
        with np.load(ep.record_path(r['raw_native']),allow_pickle=False) as z:
            data.append({k:z[k] for k in z.files})
        ep.record_path(r['native_attachment'])
    a,b=data
    for name in ('time','tets','rest_x','attachment_node_ids'):
        if not np.array_equal(a[name],b[name]):raise ValueError('Pair identity mismatch: '+name)
    if not np.array_equal(a['x'][0],b['x'][0]):raise ValueError('Pair initial geometry mismatch')
    t=a['time'];schedule=np.asarray(schedules[0]);minimum=schedule[:,1].min()
    plateaus=[(p[0],q[0]) for p,q in zip(schedule[:-1],schedule[1:]) if p[1]==q[1]==minimum]
    if len(plateaus)!=1:raise ValueError('Expected one loaded hold phase')
    start,end=plateaus[0];post_start=schedule[-2,0]
    hold=(t>=start)&(t<end);post=t>=post_start
    tip=a['rest_x'][:,0]>a['rest_x'][:,0].max()-.02
    tip_a=a['x'][:,tip].mean(1);tip_b=b['x'][:,tip].mean(1)
    half=np.asarray(configs[0]['plate']['size_m'])/2
    delta=a['collision_world']-a['plate_pose'][:,:3,None].transpose(0,2,1)
    local=np.einsum('fvi,fij->fvj',delta,Rotation.from_quat(a['plate_pose'][:,3:]).as_matrix())
    q=abs(local)-half
    signed=np.linalg.norm(np.maximum(q,0),axis=-1)+np.minimum(q.max(-1),0)
    near=signed<=.004;bottom=(q.argmax(-1)==2)&(local[:,:,2]<0)&near
    if not near[hold].any():raise ValueError('No loaded hold proximity evidence')
    root_motion=float(np.linalg.norm(a['x'][:,a['attachment_node_ids']]-a['rest_x'][a['attachment_node_ids']],axis=-1).max())
    metrics=dict(hold_interval_s=[start,end],postload_start_s=float(post_start),tip_region_last_m=.02,
        hold_tip_delta_vs_high_control_m=float((tip_a[hold,2]-tip_b[hold,2]).mean()),
        postload_mean_delta_vs_high_control_m=float((tip_a[post,2]-tip_b[post,2]).mean()),
        hold_every_sample_has_bottom_proximity=bool(bottom[hold].any(1).all()),
        hold_near_bottom_node_fraction=float(bottom[hold].sum()/near[hold].sum()),
        postload_minimum_oriented_plate_gap_m=float(signed[post].min()),
        maximum_attachment_node_displacement_m=root_motion,
        peak_applied_force_n=float(abs(a['effort_steps'][:,3]).max()),
        initial_state_and_topology_equal=True,only_config_change='plate.schedule')
    checks=dict(hold_bottom_proximity=metrics['hold_every_sample_has_bottom_proximity'],
        downward_difference=metrics['hold_tip_delta_vs_high_control_m']<-.005,
        physical_clearance=metrics['postload_minimum_oriented_plate_gap_m']>.004,
        return_towards_high_control=abs(metrics['postload_mean_delta_vs_high_control_m'])<.005,
        root_held=root_motion<.001)
    result=dict(format='beam-pair-review/1',status='paired_bending_checked' if all(checks.values()) else 'needs_design_review',
        source_states_sha256=[ep.manifest['trajectory']['states']['sha256'] for ep in episodes],
        native_cache_sha256=[r['raw_native']['sha256'] for r in resolved],
        checks=checks,metrics=metrics,training_admission=False,
        limits='Sampled collision-node distances and fixed-configuration response, not contact forces, convergence or calibrated material truth')
    write_json(output,result)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--load',type=Path,required=True);p.add_argument('--hold',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    r=review_pair(a.load,a.hold,a.output);print(r['status'],r['metrics'],flush=True)


if __name__=='__main__':main()

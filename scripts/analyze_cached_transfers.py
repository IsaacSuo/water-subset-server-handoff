"""Cache-only event/velocity and native cable spin analysis; no solver imports."""
import argparse
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from world_model_dataset.causal_loader import open_episode
from world_model_dataset.io import read_json,write_json,file_hash


def episode(folder):
    job=read_json(folder/'execution/workflow.json')['jobs']['baseline']
    return open_episode(job['stages']['observe']['result']['episode'])


def collision(folder):
    ep=episode(folder);rows=list(ep.states());t=np.array([r['time_s'] for r in rows])
    contacts=[c for c in ep.contacts() if set(c['actor_ids'])=={'body_a','body_b'}]
    start=min(c['time_s'] for c in contacts);end=max(c['time_s'] for c in contacts)
    result=dict(first_interbody_contact_s=start,last_interbody_contact_s=end,bodies={},
        ground_first_contact_s=min(c['time_s'] for c in ep.contacts() if 'room' in c['actor_ids']))
    for oid in ('body_a','body_b'):
        v=np.array([r['body_states'][oid]['linear_velocity_m_s'] for r in rows])
        p=np.array([r['body_states'][oid]['position_m'] for r in rows])
        samples=[]
        for time in (0,start-.05,start-1/240,start,start+1/240,start+.025,start+.05,start+.1,end,end+.05):
            i=int(abs(t-time).argmin());samples.append(dict(time_s=float(t[i]),velocity_m_s=v[i].tolist(),position_m=p[i].tolist()))
        window=(t>=start)&(t<=end)
        result['bodies'][oid]=dict(samples=samples,contact_interval_vx_range_m_s=[float(v[window,0].min()),float(v[window,0].max())],
            contact_interval_net_x_m=float(p[window][-1,0]-p[window][0,0]))
    impulses=np.asarray([c['impulse_ns'] for c in contacts])
    result['reported_interbody_impulse_norm_sum_ns']=float(np.linalg.norm(impulses,axis=1).sum())
    result['limits']='event-aligned observational evidence; ground contact and shape rotation also affect velocity; no counterfactual causal estimate'
    return result


def spin(folder):
    ep=episode(folder);r=ep.resolved_inputs();path=ep.record_path(r['raw_native'])
    with np.load(path) as a:
        t=a['time'];ids=a['segment_body_ids'];q=a['body_q'][:,ids,3:];w=a['body_qd'][:,ids,3:]
        ends=a['segment_end_world_m'];axis=ends-a['body_q'][:,ids,:3];axis/=np.linalg.norm(axis,axis=-1)[...,None]
        parallel=np.sum(w*axis,axis=-1);perp=np.linalg.norm(w-parallel[...,None]*axis,axis=-1)
        tail=t>=t[-1]-.5;dt=np.diff(t)
        rot=Rotation.from_quat(q.reshape(-1,4)).as_matrix().reshape(q.shape[:2]+(3,3))
        delta=Rotation.from_matrix((rot[1:]@rot[:-1].transpose(0,1,3,2)).reshape(-1,3,3)).as_rotvec().reshape(w[1:].shape)/dt[:,None,None]
        midw=(w[1:]+w[:-1])/2;mt=tail[1:]&tail[:-1]
        records=[]
        for i,body in enumerate(ids):
            records.append(dict(body_id=int(body),native_omega_peak_rad_s=float(np.linalg.norm(w[tail,i],axis=-1).max()),
                axial_omega_rms_rad_s=float(np.sqrt(np.mean(parallel[tail,i]**2))),
                axial_omega_signed_range_rad_s=[float(parallel[tail,i].min()),float(parallel[tail,i].max())],
                axial_sign_reversals=int(np.count_nonzero(np.diff(np.sign(parallel[tail,i])))),
                transverse_omega_rms_rad_s=float(np.sqrt(np.mean(perp[tail,i]**2))),
                pose_delta_omega_rms_rad_s=float(np.sqrt(np.mean(np.sum(delta[mt,i]**2,axis=-1)))),
                pose_delta_vs_native_midpoint_rms_rad_s=float(np.sqrt(np.mean(np.sum((delta[mt,i]-midw[mt,i])**2,axis=-1)))),
                axis_change_peak_rad=float(np.arccos(np.clip(np.sum(axis[1:,i]*axis[:-1,i],axis=-1),-1,1))[mt].max())))
        line=a['centerline'];velocity=np.diff(line,axis=0)/dt[:,None,None]
        local_w=np.einsum('fsji,fsj->fsi',rot,w)
        energy=.5*np.einsum('fsi,sij,fsj->fs',local_w,a['body_inertia'][ids],local_w)
        return dict(raw_native_sha256=file_hash(path),velocity_order=a['body_velocity_order'].tolist(),
            definition='native world linear velocity at COM followed by native world angular velocity XYZ',
            segments=records,tail_centerline_peak_sampled_speed_m_s=float(np.linalg.norm(velocity[mt],axis=-1).max()),
            tail_segment_rotational_energy_sum_range_j=[float(energy[tail].sum(1).min()),float(energy[tail].sum(1).max())],
            tail_centerline_point_range_max_m=float(np.linalg.norm(np.ptp(line[tail],axis=0),axis=-1).max()),
            attachment_gap_max_m=float(a['attachment_gap_m'].max()),attachment_gap_tail_max_m=float(a['attachment_gap_m'][tail].max()),
            joint_endpoint_gap_max_m=float(a['joint_endpoint_gap'].max()),
            limits='60 Hz pose increments can alias within-step dynamics; axial symmetry makes spin poorly visible; not quality admission or native tension')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--kind',choices=['collision','spin'],required=True);p.add_argument('--folder',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();write_json(a.output,globals()[a.kind](a.folder))

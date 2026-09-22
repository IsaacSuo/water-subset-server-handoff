"""Read completed native caches; geometry residuals are not measured forces."""
import argparse
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from world_model_dataset.io import read_json,write_json


def measure(folder):
    doc=read_json(folder/'generated/experiment.json')
    job=read_json(folder/'execution/workflow.json')['jobs']['baseline']
    ep=Path(job['stages']['package']['result']['episode'])
    if doc['backend']['kind']=='rope':
        with np.load(ep/'native/states.npz') as a:
            result={key:dict(initial_max=float(a[key][0].max()),maximum=float(a[key].max()),final_max=float(a[key][-1].max()))
                    for key in ('attachment_gap_m','joint_endpoint_gap') if key in a}
            line=a['centerline']
            result.update(native_segment_length_range_m=[float(a['segment_lengths'].min()),float(a['segment_lengths'].max())],
                end_point_initial_final_m=[line[0,-1].tolist(),line[-1,-1].tolist()],
                end_point_displacement_m=(line[-1,-1]-line[0,-1]).tolist(),
                first_point_max_displacement_m=float(np.linalg.norm(line[:,0]-line[0,0],axis=-1).max()),
                sampled_centerline_length_initial_final_m=np.linalg.norm(np.diff(line[[0,-1]],axis=1),axis=-1).sum(1).tolist(),
                interpretation='native geometry/connection residuals; not native tension or convergence proof')
    elif doc['backend']['kind']=='beam':
        cfg=doc['input']
        with np.load(ep/'native/frames.npz') as a:
            t=a['time'];pose=a['plate_pose']
            local=np.einsum('fvi,fij->fvj',a['collision_world']-pose[:,None,:3],Rotation.from_quat(pose[:,3:]).as_matrix())
            q=abs(local)-np.array(cfg['plate']['size_m'])/2
            signed=np.linalg.norm(np.maximum(q,0),axis=-1)+np.minimum(q.max(-1),0)
            near=((q.argmax(-1)==2)&(local[:,:,2]<0)&(signed<=.004)).any(1)
            tip=a['rest_x'][:,0]>a['rest_x'][:,0].max()-.02;z=a['x'][:,tip,2].mean(1)
            schedule=np.asarray(cfg['plate']['schedule']);start=schedule[1,0];i=int(abs(t-start).argmin())
            result=dict(bottom_proximity_first_last_s=t[near][[0,-1]].tolist() if near.any() else None,
                bottom_proximity_samples=int(near.sum()),before_loading_time_s=float(t[i]),before_loading_tip_z_m=float(z[i]),
                initial_tip_z_m=float(z[0]),preload_tip_drop_m=float(z[0]-z[i]),
                limits='sampled geometry proximity, not contact force; lack of proximity excludes continuous plate loading')
    else:
        raise ValueError('Only native rope and beam diagnostics supported')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--folder',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();result=measure(a.folder);write_json(a.output,result);print(result)

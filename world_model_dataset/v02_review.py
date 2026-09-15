"""Supplemental corrected-V02 review; original cache/metrics/manifests stay intact."""
from pathlib import Path

import numpy as np

from .contract import artifact,validate_pair
from .io import inside,read_json,write_json
from .metrics import equilibrium_recovery_metrics


def review_episode(root):
    root=Path(root);ep=read_json(root/'episode.prepared.json');spec=ep['spec']
    if spec['event_id']!='V02':raise ValueError('Expected V02 cache')
    if not read_json(root/'physics_validation.json')['passed']:raise ValueError('Cache integrity failed')
    frames=read_json(root/'state/index.json')['frames'];surfaces=[];times=[];topology=None
    for frame in frames:
        state=read_json(inside(root,frame['state']));oid=spec['objects'][0]['instance_id']
        if artifact(root,frame['geometry'])!=state['objects'][oid]['geometry']:raise ValueError('Cache checksum changed')
        with np.load(inside(root,frame['geometry']),allow_pickle=False) as data:
            if topology is None:topology=data['surface_triangles'].copy()
            if not np.array_equal(topology,data['surface_triangles']):raise ValueError('Surface topology changed')
            surfaces.append(data['surface_world_m'].copy());times.append(state['time_s'])
    ap=spec['action_parameters'];unload=ap['start_time_s']+ap['compression_duration_s']+ap['hold_duration_s']
    result=equilibrium_recovery_metrics(times,surfaces,ap['start_time_s'],unload,
                                      unload+ap['withdraw_duration_s'],ep['inputs']['objects'][0]['geometry']['characteristic_size_m'])
    result['episode_id']=spec['episode_id']
    return result


def review_matrix(root):
    root=Path(root);matrix=read_json(root/'matrix.json');results=[]
    for oid in matrix['episodes']:
        results.append(review_episode(root/'episodes'/oid))
    for a,b in matrix['pairs']:
        validate_pair(read_json(root/'specs'/(a+'.json')),read_json(root/'specs'/(b+'.json')))
        with np.load(root/'episodes'/a/'geometry/subject.npz',allow_pickle=False) as ga:
            with np.load(root/'episodes'/b/'geometry/subject.npz',allow_pickle=False) as gb:
                if not np.array_equal(ga['vertices'],gb['vertices']):raise ValueError('Paired rest geometry changed')
    report={'schema_version':'0.1.0','episodes':results,'strict_pairs':matrix['pairs'],
        'legacy_metrics_unchanged':True,'replay_status':'pending',
        'scope':'Recovery review only; no soft contact force/impulse inference; no observation-complete release.'}
    write_json(root/'equilibrium_recovery_review.json',report)
    return report


def main():
    import argparse
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('matrix',type=Path);args=parser.parse_args()
    report=review_matrix(args.matrix)
    for item in report['episodes']:
        print(item['episode_id'],'final return error mm',1000*item['final_equilibrium_shape_error_m'],
              '1mm-equivalent recovery',item['thresholds_D']['0.005'],flush=True)


if __name__=='__main__':main()

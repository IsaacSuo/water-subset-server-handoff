"""Measure cantilever motion using native node identities and root-relative frames."""
import argparse
from pathlib import Path

import numpy as np

from .causal_loader import open_episode
from .io import write_json


def review(path, output, edge_x):
    ep=open_episode(path,require_complete=False)
    records=[];sections=[];initial=None
    for row,g in ep.soft_geometries('soft'):
        x=g['simulation_world_m']
        if initial is None:
            initial=x.copy();root=initial[:,0]<edge_x-.05
            tip=initial[:,0]>initial[:,0].max()-.025
            if root.sum()<4 or tip.sum()<4:raise ValueError('Insufficient root/tip nodes')
            root0=initial[root].mean(0);centred=initial[root]-root0
            edges=np.linspace(initial[:,0].min()-1e-6,initial[:,0].max()+1e-6,13)
            bins=[(initial[:,0]>=a)&(initial[:,0]<b) for a,b in zip(edges[:-1],edges[1:])]
            if any(not b.any() for b in bins):raise ValueError('Empty beam section')
        centre=x[root].mean(0)
        u,_,vh=np.linalg.svd(centred.T@(x[root]-centre))
        sign=np.eye(3);sign[-1,-1]=np.linalg.det(u@vh);rotation=u@sign@vh
        rigid_prediction=(initial-root0)@rotation+centre
        residual=x-rigid_prediction
        metrics=row['body_states']['soft']['metrics']
        records.append(dict(time_s=row['time_s'],tip_relative_z_m=float(residual[tip,2].mean()),
            root_translation_m=float(np.linalg.norm(centre-root0)),root_x_shift_m=float(centre[0]-root0[0]),
            root_nonrigid_rms_m=float(np.sqrt(np.mean(np.sum(residual[root]**2,axis=-1)))),
            minimum_tet_j=metrics['minimum_j'],plate_gap_m=metrics['sampled_actuator_gap_m'],
            max_native_speed_m_s=metrics['maximum_nodal_speed_m_s']))
        sections.append([x[b].mean(0).tolist() for b in bins])
    t=np.array([r['time_s'] for r in records]);d=np.array([r['tip_relative_z_m'] for r in records])
    baseline=float(d[(t>=.4)&(t<=.5)].mean())
    loaded=float(d[(t>=1.75)&(t<=2)].mean());final=float(d[t>=t[-1]-.5].mean())
    load_deflection=loaded-baseline
    loading_gaps=[r['plate_gap_m'] for r in records if .5<=r['time_s']<=2]
    contact_evidence=min(loading_gaps)<=.004
    result=dict(format='bending-cache-review/1',source_states_sha256=ep.manifest['trajectory']['states']['sha256'],
        representation='native simulation nodes; same root/tip/section identities throughout',
        root_selection_x_below_m=edge_x-.05,tip_last_length_m=.025,
        baseline_tip_relative_z_m=baseline,loaded_tip_relative_z_m=loaded,final_tip_relative_z_m=final,
        deflection_during_loading_command_m=load_deflection,
        minimum_plate_gap_during_loading_command_m=min(loading_gaps),
        loading_near_contact_fraction=sum(g<=.004 for g in loading_gaps)/len(loading_gaps),
        recovery_fraction=(final-loaded)/(baseline-loaded) if abs(load_deflection)>1e-8 else None,
        max_root_translation_m=max(r['root_translation_m'] for r in records),
        max_root_x_shift_m=max(abs(r['root_x_shift_m']) for r in records),
        final_plate_gap_m=records[-1]['plate_gap_m'],minimum_tet_j=min(r['minimum_tet_j'] for r in records),
        records=records,section_centres_world_m=sections,training_admission=False,
        limits='Contact-held beam, not ideal fixed-node cantilever; reference is gravity-loaded; motion during a command alone does not prove actuator loading. No contact-force or convergence claim')
    result['checks']=dict(loading_contact_evidence=contact_evidence,downward_motion_during_command=load_deflection<-.005,
        recovery=result['recovery_fraction'] is not None and result['recovery_fraction']>.7,
        root_remains_held=result['max_root_translation_m']<.005,
        real_unload=contact_evidence and result['final_plate_gap_m']>.004,positive_tet_j=result['minimum_tet_j']>0)
    result['status']='bending_response_observed' if all(result['checks'].values()) else 'needs_design_review'
    write_json(output,result)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--episode',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--edge-x',type=float,required=True)
    a=p.parse_args();r=review(a.episode,a.output,a.edge_x)
    print({k:v for k,v in r.items() if k not in ('records','section_centres_world_m')})


if __name__=='__main__':main()

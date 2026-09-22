"""Summarize actual completed states/contacts/controls; does not run physics."""
import argparse
from pathlib import Path
import numpy as np
from world_model_dataset.causal_loader import open_episode
from world_model_dataset.experiment_review import verify_alignment
from world_model_dataset.io import read_json,write_json,file_hash


def summarize(folder,output):
    ledger=read_json(folder/'execution/workflow.json');job=ledger['jobs']['baseline']
    doc=read_json(folder/'generated/experiment.json')
    source=job['stages'].get('observe',{}).get('result',{}).get('episode') or job['stages']['package']['result']['episode']
    ep=open_episode(source,require_complete='observe' in job['stages']);rows=list(ep.states())
    times=np.array([r['time_s'] for r in rows]);tail=times>=times[-1]-.5;subjects={}
    for b in ep.manifest['system']['bodies']:
        if b['physics_kind']=='static':continue
        oid=b['instance_id'];p=np.array([r['body_states'][oid]['position_m'] for r in rows])
        velocities=[r['body_states'][oid].get('linear_velocity_m_s') for r in rows]
        angular=[r['body_states'][oid].get('angular_velocity_rad_s') for r in rows]
        v=np.array(velocities) if all(x is not None for x in velocities) else None
        w=np.array(angular) if all(x is not None for x in angular) else None
        subjects[oid]=dict(initial_position_m=p[0].tolist(),final_position_m=p[-1].tolist(),displacement_m=(p[-1]-p[0]).tolist(),
            pose_semantics=rows[-1]['body_states'][oid].get('pose_semantics','native body pose'),
            last_half_second_position_range_m=np.ptp(p[tail],axis=0).tolist(),last_half_second_peak_speed_m_s=float(np.linalg.norm(v[tail],axis=1).max()) if v is not None else None,
            last_half_second_peak_angular_speed_rad_s=float(np.linalg.norm(w[tail],axis=1).max()) if w is not None else None)
        if 'geometry' in rows[0]['body_states'][oid]:
            start=None;end=None
            for _,g in ep.geometries(oid):
                field=next((k for k in ('surface_world_m','simulation_world_m','particle_world_m','centerline') if k in g),None)
                if field:
                    if start is None:start=g[field]
                    end=g[field]
            if start is not None:subjects[oid]['native_point_centroid_displacement_m']=(end.mean(0)-start.mean(0)).tolist()
    pairs={}
    if ep.manifest['trajectory']['contacts']['status']=='available':
        for c in ep.contacts():
            k=' / '.join(c['actor_ids']);d=pairs.setdefault(k,dict(count=0,first_time_s=c['time_s'],last_time_s=c['time_s'],minimum_separation_m=0.,last_half_second_count=0))
            d['count']+=1;d['last_time_s']=c['time_s'];d['minimum_separation_m']=min(d['minimum_separation_m'],c['separation_m'])
            d['last_half_second_count']+=int(c['time_s']>=times[-1]-.5)
    traces={}
    for controller in ep.manifest['control_program']['controllers']:
        cid=controller['controller_id'];eff=list(ep.actuator_efforts(cid));actual=list(ep.actuator_states(cid))
        traces[cid]=dict(efforts=len(eff),actual=len(actual),first_effort=eff[0],last_effort=eff[-1])
    report=dict(requested_phenomenon=doc['phenomenon'],source_episode=str(source),source_manifest_sha256=file_hash(Path(source)/('episode.json' if (Path(source)/'episode.json').exists() else 'episode.physics.json')),
        stages=list(job['stages']),errors=job['errors'],alignment=verify_alignment(ep,doc['timing'],observed='observe' in job['stages']),subjects=subjects,
        native_contact_pairs=pairs,control_traces=traces,backend_review=read_json(job['stages']['audit']['result']['report']) if 'audit' in job['stages'] else None,
        training_admission=False,use_qualification='human_use_review_pending',no_convergence_claim=True)
    write_json(output,report)
    print(doc['phenomenon'],report['alignment']['counts'],subjects,pairs)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--folder',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();summarize(a.folder,a.output)

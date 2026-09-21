"""Measured small-pair outcomes; never filter an episode by intended outcome."""
import argparse
import copy
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .causal_loader import open_episode
from .io import file_hash, read_json, write_json


def review(batch_path, output):
    batch=read_json(batch_path);pairs={};result={}
    for job in batch['jobs'].values():
        if job['status']!='generated':raise ValueError('Incomplete job: '+job['id'])
        spec=read_json(job['config']);shot=spec['shots'][0]
        if len(shot['bodies'])!=1:raise ValueError('This review expects single-subject comparisons')
        oid=shot['bodies'][0]['id'];ep=open_episode(job['episode'],require_complete=False)
        if file_hash(Path(job['episode'])/job['manifest'])!=job['manifest_sha256']:raise ValueError('Changed source manifest')
        resolved=ep.resolved_inputs();body=resolved['bodies'][oid]
        with np.load(ep.record_path(body['geometry']['mesh'])) as z:vertices=z['vertices']
        rows=list(ep.states());poses=[r['body_states'][oid] for r in rows]
        world=[Rotation.from_quat(p['orientation_xyzw']).apply(vertices)+p['position_m'] for p in poses]
        bounds=np.asarray([[v.min(0),v.max(0)] for v in world]);positions=np.asarray([p['position_m'] for p in poses])
        env={b['instance_id']:resolved['bodies'][b['instance_id']]['geometry']['mesh']['sha256']
             for b in ep.manifest['system']['bodies'] if b['physics_kind']=='static'}
        contacts=list(ep.contacts());support=shot['bodies'][0].get('support_group','support')
        environment_contacts=[c for c in contacts if any(actor in env and actor!=support for actor in c['actor_ids'])]
        contact_counts={actor:sum(actor in c['actor_ids'] for c in contacts) for actor in env}
        measured=dict(states=len(rows),duration_s=rows[-1]['time_s'],
            source_states_sha256=ep.manifest['trajectory']['states']['sha256'],
            initial_position_m=positions[0].tolist(),final_position_m=positions[-1].tolist(),
            translation_m=(positions[-1]-positions[0]).tolist(),
            centre_path_length_m=float(np.linalg.norm(np.diff(positions,axis=0),axis=1).sum()),
            initial_world_extents_m=(bounds[0,1]-bounds[0,0]).tolist(),
            final_world_bounds_m=bounds[-1].tolist(),mass_kg=body['physics']['mass_kg'],
            derived_density_kg_m3=body['physics']['density_kg_m3'],
            derived_inertia_tensor_kg_m2=body['geometry']['inertia_tensor_kg_m2'],
            subject_friction={k:body['physics'][k] for k in ('static_friction','dynamic_friction')},
            scene_friction={k:resolved['bodies']['room']['physics'][k] for k in ('static_friction','dynamic_friction')},
            scene_mesh_sha256=env,collision=body['geometry']['collision_approximation'],
            native_contact_records=len(contacts),original_obstacle_contact_records=len(environment_contacts),
            contact_counts_by_scene_group=contact_counts,
            minimum_native_contact_separation_m=min((c['separation_m'] for c in contacts),default=None),
            checkpoints=[])
        for time in (.25,.5,1.,2.):
            idx=min(range(len(rows)),key=lambda i:abs(rows[i]['time_s']-time));p=poses[idx]
            measured['checkpoints'].append(dict(time_s=rows[idx]['time_s'],position_m=p['position_m'],
                linear_velocity_m_s=p['linear_velocity_m_s'],angular_velocity_rad_s=p['angular_velocity_rad_s']))
        if job['family']=='passage_asset_scale':
            # G13 atlas defines the exit at X=.6 and a sampled .2 m wide strip.
            in_restriction=(bounds[:,1,0]>=-.15)&(bounds[:,0,0]<=.4)
            measured.update(exit_plane_x_m=.6,entire_body_beyond_exit_at_final=bool(bounds[-1,0,0]>.6),
                first_entire_body_exit_time_s=next((r['time_s'] for r,b in zip(rows,bounds) if b[0,0]>.6),None),
                max_abs_centre_y_offset_in_restriction_m=float(abs(positions[in_restriction,1]+1).max()) if in_restriction.any() else None,
                stayed_inside_sampled_width_in_restriction=bool(np.all(bounds[in_restriction,0,1]>=-1.1)&np.all(bounds[in_restriction,1,1]<=-.9)))
        result[job['id']]=measured
        normalized=copy.deepcopy(spec);ns=normalized['shots'][0]
        for key in ('id','title'):ns.pop(key,None)
        subject=ns['bodies'][0]
        if job['family']=='rolling_friction':
            for key in ('static_friction','dynamic_friction'):subject['material'].pop(key)
        elif job['family']=='passage_asset_scale':subject.pop('size_m')
        else:raise ValueError('Unsupported review family')
        pairs.setdefault((job['template_id'],job['asset']),[]).append((job['id'],normalized,env))
    checked=[]
    for key,items in pairs.items():
        if len(items)!=2:raise ValueError('Expected two conditions per asset')
        if items[0][1:]!=items[1][1:]:raise ValueError('Pair changed more than declared body fields or scene geometry')
        checked.append(dict(template=key[0],asset=key[1],episodes=[v[0] for v in items],
                            only_declared_inputs_change=True,identical_scene_meshes=True))
    write_json(output,dict(pairs=checked,episodes=result,training_admission=False,
        limits='Native cached samples and rigid contacts; input coefficient pair is not measured effective friction; size also changes derived density/inertia/support height; no outcome-based selection'))
    print({k:dict(translation_m=v['translation_m'],exit=v.get('entire_body_beyond_exit_at_final'),
                  obstacle_contacts=v['original_obstacle_contact_records']) for k,v in result.items()},flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--batch',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();review(a.batch,a.output)


if __name__=='__main__':main()

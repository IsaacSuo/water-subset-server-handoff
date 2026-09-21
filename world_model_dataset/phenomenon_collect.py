"""Audit fresh pilot caches and package unified, calibrated diagnostic data.

Source episodes are preserved. Completion means records are readable; quality
limitations and unavailable contact forces remain explicit, not training admission.
"""
import argparse
import copy
import json
from pathlib import Path
import shutil

import numpy as np
from PIL import Image

from .causal_loader import open_episode
from .causal_contract import audit_causal_manifest
from .io import read_json, write_json, file_hash
from .probe_episode import artifact
from .phenomenon_observations import camera, GeometryView


def audit(ep, family=None):
    rows=list(ep.states());first=rows[0];last=rows[-1]
    fixed=[b['instance_id'] for b in ep.manifest['system']['bodies'] if b['physics_kind']=='static']
    for row in rows:
        for oid in fixed:
            for key in ('position_m','orientation_xyzw'):
                if not np.allclose(row['body_states'][oid][key],first['body_states'][oid][key],atol=1e-7):
                    raise ValueError('Static participant moved: '+oid)
    carriers=[oid for oid,state in first['body_states'].items() if 'topology' in state and 'geometry' in state]
    info={};warnings=[]
    for oid in carriers:
        topology=ep.soft_topology(oid);entry=dict(topology_representation=topology.get('representation','native_tetrahedral_volume'),frames=0)
        initial=None;end=None;peak_displacement=0.;min_area=float('inf');min_j=float('inf');jp=[]
        for row,g in ep.geometries(oid):
            entry['frames']+=1
            entry.setdefault('native_fields',list(g))
            field=next((k for k in ('simulation_world_m','surface_world_m','particle_world_m','centerline') if k in g),None)
            if field:
                points=g[field]
                if initial is None:initial=points.copy()
                if points.shape!=initial.shape:raise ValueError('Native identity count changed')
                peak_displacement=max(peak_displacement,float(np.linalg.norm(points-initial,axis=-1).max()));end=points
            if 'surface_triangles' in g:
                tri=g['surface_triangles'];v=g['surface_world_m']
                if tri.ndim!=2 or tri.shape[1]!=3 or tri.min()<0 or tri.max()>=len(v):raise ValueError('Invalid surface connectivity')
                t=v[tri];min_area=min(min_area,float(np.linalg.norm(np.cross(t[:,1]-t[:,0],t[:,2]-t[:,0]),axis=1).min()/2))
            if 'material_Jp' in g:jp.append(float(g['material_Jp'].mean()))
            if 'surface_nodal_velocities_m_s' in g:
                entry['final_nodal_rms_speed_m_s']=float(np.sqrt(np.mean(np.sum(g['surface_nodal_velocities_m_s']**2,axis=1))))
                speed=float(np.linalg.norm(g['surface_nodal_velocities_m_s'],axis=1).max())
                entry['peak_nodal_speed_m_s']=max(speed,entry.get('peak_nodal_speed_m_s',0.))
                if row['time_s']>=rows[-1]['time_s']-.5:
                    entry['last_half_second_peak_nodal_speed_m_s']=max(speed,entry.get('last_half_second_peak_nodal_speed_m_s',0.))
            metrics=row['body_states'][oid].get('metrics',{})
            if 'minimum_j' in metrics:min_j=min(min_j,metrics['minimum_j'])
        if initial is not None:
            entry.update(initial_bounds_m=[initial.min(0).tolist(),initial.max(0).tolist()],final_bounds_m=[end.min(0).tolist(),end.max(0).tolist()],
                         max_corresponding_point_displacement_m=peak_displacement)
        if min_area!=float('inf'):
            entry['minimum_sampled_triangle_area_m2']=min_area
            if min_area<=1e-12:raise ValueError('Degenerate native surface')
        if min_j!=float('inf'):
            entry['minimum_tet_J']=min_j
            if min_j<=0:raise ValueError('Inverted volume element')
        if jp:entry.update(initial_Jp_mean=jp[0],final_Jp_mean=jp[-1],Jp_mean_min=min(jp))
        info[oid]=entry
        if 'surface_nodal_velocities_m_s' in entry['native_fields']:
            warnings.append('Cloth contact stability is under independent review; finite readable records are not physical-quality admission')
    subjects=[b['instance_id'] for b in ep.manifest['system']['bodies'] if b['role']=='subject']
    displacements={oid:(np.asarray(last['body_states'][oid].get('centre_of_mass_m',last['body_states'][oid]['position_m']))-
                       first['body_states'][oid].get('centre_of_mass_m',first['body_states'][oid]['position_m'])).tolist() for oid in subjects}
    result=dict(finite_native_streams=True,static_participants_unchanged=True,states=len(rows),duration_s=last['time_s'],
                representations=info,subject_reference_displacements_m=displacements,
                displacement_semantics='COM where exported, otherwise body reference frame; surface root is not cloth COM',
                warnings=warnings,coverage='cache and keyframe checks only; no convergence, full collision or training admission claim')
    if ep.manifest['control_program']['primitive']!='none':
        controls=list(ep.controls());states=list(ep.actuator_states());efforts=list(ep.actuator_efforts())
        if not controls or not states or not efforts:raise ValueError('Missing active control stream')
        ctrl=ep.manifest['control_program']['controllers'][0]
        peak=max(abs(r['applied_force_n']) for r in efforts)
        if peak>ctrl['max_force_n']+1e-4:raise ValueError('Actuator force exceeds declared limit')
        result['actuator']=dict(commands=len(controls),state_records=len(states),effort_records=len(efforts),
            max_force_n=ctrl['max_force_n'],peak_applied_force_n=peak,
            saturated_step_fraction=sum(bool(r['saturated']) for r in efforts)/len(efforts),
            effort_semantics='applied bounded controller force, not measured contact reaction')
        if family in (None, 'elastic_finite_load_hold_unload'):
            soft=next(oid for oid in info if 'minimum_tet_J' in info[oid])
            metric=[r['body_states'][soft]['metrics'] for r in rows]
            initial_height=next(r['body_states'][soft]['metrics']['height_m'] for r in rows if r['time_s']>=.5)
            min_height=min(m['height_m'] for m in metric);last_metric=metric[-1]
            clear=[r['time_s'] for r in rows if r['time_s']>=2 and r['body_states'][soft]['metrics']['sampled_actuator_gap_m']>.004]
            result['actuator'].update(
                initial_loaded_reference_height_m=initial_height,minimum_height_m=min_height,final_height_m=last_metric['height_m'],
                final_sampled_actuator_gap_m=last_metric['sampled_actuator_gap_m'],first_clearance_after_unload_s=clear[0] if clear else None,
                recovery_fraction=(last_metric['height_m']-min_height)/(initial_height-min_height) if initial_height>min_height else None,
                unload_verified=last_metric['sampled_actuator_gap_m']>.004)
            if not result['actuator']['unload_verified']:warnings.append('Unloading command did not establish final physical clearance')
    else:
        assert not list(ep.controls())
    if ep.manifest['trajectory']['contacts']['status']=='available' and ep.manifest['capabilities'].get('rigid_contact_impulse',{}).get('status')=='native':
        contacts=list(ep.contacts())
        result['native_contacts']=dict(count=len(contacts),minimum_separation_m=min((c['separation_m'] for c in contacts),default=None))
    return result


def apply_quality_evidence(ep, review, evidence):
    """Accept an explicit local review only for the exact recorded state stream."""
    path=Path(evidence['path'])
    if file_hash(path)!=evidence['sha256']:raise ValueError('Quality evidence hash mismatch')
    result=read_json(path)
    if result.get('source_states_sha256')!=ep.manifest['trajectory']['states']['sha256']:
        raise ValueError('Quality evidence belongs to a different physical cache')
    if result.get('format')!='cloth-stability-review/1':raise ValueError('Unsupported quality evidence')
    review['local_stability_review']=result
    if result['status']=='local_stability_checked' and result.get('checks') and all(result['checks'].values()):
        review['warnings']=[w for w in review['warnings'] if not w.startswith('Cloth contact stability')]
        review['warnings'].append('Local cloth stability checked for this cache only; no convergence or material calibration claim')
    return review


def collect(job,destination):
    source=Path(job['episode']).resolve();destination=Path(destination).resolve()
    ep=open_episode(source,require_complete=False);review=audit(ep, job['family'])
    if job.get('quality_evidence'):apply_quality_evidence(ep,review,job['quality_evidence'])
    destination.mkdir(parents=True,exist_ok=False)
    out=destination/'episode';shutil.copytree(source,out)
    ep=open_episode(out,require_complete=False)
    write_json(destination/'cache_review.json',review)
    if job.get('quality_evidence'):write_json(destination/'quality_evidence.json',review['local_stability_review'])
    write_json(out/'source_manifest.json',ep.manifest)
    view=GeometryView(ep);cam=camera(job['observation_camera']);static=view.render_static(cam)
    obs=out/'observations';obs.mkdir(exist_ok=False)
    stride=ep.manifest['timing']['physics_hz']//job['observation_hz'];frames=[]
    source_hash=ep.manifest['trajectory']['states']['sha256']
    visibility={oid:0 for oid in view.subjects}
    for row in ep.states():
        if row['physics_step']%stride:continue
        raster=copy.copy(static);raster.depth=static.depth.copy();raster.rgb=static.rgb.copy();raster.seg=static.seg.copy()
        view.render(raster,row);name=f"{row['physics_step']:06d}"
        Image.fromarray(raster.rgb).save(obs/(name+'.png'))
        np.savez_compressed(obs/(name+'.npz'),**raster.arrays(),time_s=row['time_s'],physics_step=row['physics_step'])
        for oid in view.subjects:visibility[oid]+=int(np.count_nonzero(raster.seg==view.body_ids[oid]))
        frames.append(dict(time_s=row['time_s'],physics_step=row['physics_step'],camera_id='main',rgb=name+'.png',data=name+'.npz',
                           rgb_sha256=file_hash(obs/(name+'.png')),data_sha256=file_hash(obs/(name+'.npz'))))
    if not frames or not any(visibility.values()):raise ValueError('No observed subject')
    write_json(obs/'index.json',dict(frames=frames,cameras={'main':cam},segmentation_ids=view.body_ids,
        render_backend='mainline CPU perspective z-buffer; native cached geometry',source_state_sha256=source_hash,
        observation_hz=job['observation_hz'],geometry_representation=dict(kind=view.kind,
            rigid='source physical mesh or declared box',surface='native triangles',volume='native cached display surface; simulation Tet fields remain authoritative',
            rope='tessellation of native collision capsules; not material surface topology',plastic='recorded initial-radius particle glyph union; not continuum surface'),
        appearance='diagnostic flat colors and shading, not source asset texture',physics_rerun=False,hidden_plane=False,
        subject_visible_pixel_sum=visibility))
    if ep.manifest['trajectory']['interaction_annotations']['status']=='available':annotations=ep.annotations()
    else:annotations=dict(labels=[],reason='native contact stream retained; no unverified phenomenon labels')
    if ep.manifest['trajectory']['outcomes']['status']=='available':outcomes=ep.outcomes()
    else:outcomes={}
    annotations['pilot_design']=dict(family=job['family'],condition=job['condition'],variable=job['comparison_variable'],
        semantics='experimental intent, not a required outcome')
    outcomes['pilot_cache_review']=review
    write_json(out/'pilot_annotations.json',annotations);write_json(out/'pilot_outcomes.json',outcomes)
    manifest=copy.deepcopy(ep.manifest)
    manifest['timing']['capture_hz']=job['observation_hz']
    manifest['trajectory']['observations']=artifact(out,'observations/index.json','calibrated diagnostic cache-only RGB/depth/body IDs at 10 Hz')
    manifest['trajectory']['interaction_annotations']=artifact(out,'pilot_annotations.json','source annotations plus explicit design metadata')
    manifest['trajectory']['outcomes']=artifact(out,'pilot_outcomes.json','source outcomes and measured cache diagnostics; not training labels')
    manifest['capabilities']['rgb_depth_segmentation']=dict(status='derived',source='CPU render of cached physical geometry; diagnostic appearance',reason=None)
    manifest['lifecycle']='completed'
    accepted=audit_causal_manifest(manifest,require_complete=True)
    if not accepted['accepted']:raise ValueError(accepted['errors'])
    # This is a derived copy: preserve the original source package/manifest.
    (out/'episode.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    complete=open_episode(out);observations=sum(1 for _ in complete.observations())
    result=dict(id=job['id'],family=job['family'],group=job['group'],condition=job['condition'],
        episode=str(out),manifest_sha256=file_hash(out/'episode.json'),source_episode=str(source),source_manifest_sha256=job['manifest_sha256'],
        source_states_sha256=source_hash,observations=observations,states=review['states'],
        representations=review['representations'],quality='needs_stability_review' if any('Cloth contact stability' in w for w in review['warnings']) else 'cache_checked_with_declared_limits',training_admission=False,
        observation_representation='diagnostic RGB-depth-IDs, no source textures',review='cache_review.json')
    write_json(destination/'result.json',result)
    return result


def refresh_review(destination):
    """Update diagnostic review metadata without modifying physical states or images."""
    destination=Path(destination);out=destination/'episode';ep=open_episode(out)
    result=read_json(destination/'result.json')
    if file_hash(out/'episode.json')!=result['manifest_sha256']:raise ValueError('Manifest changed before review refresh')
    review=audit(ep, result['family'])
    evidence=destination/'quality_evidence.json'
    if evidence.exists():apply_quality_evidence(ep,review,dict(path=str(evidence),sha256=file_hash(evidence)))
    for path in (destination/'cache_review.json',out/'pilot_outcomes.json',out/'episode.json',destination/'result.json'):
        backup=path.with_name(path.name+'.before_review_refresh')
        if not backup.exists():shutil.copyfile(path,backup)
    (destination/'cache_review.json').write_text(json.dumps(review,indent=2)+'\n')
    outcomes=ep.outcomes();outcomes['pilot_cache_review']=review
    (out/'pilot_outcomes.json').write_text(json.dumps(outcomes,indent=2)+'\n')
    manifest=copy.deepcopy(ep.manifest)
    manifest['trajectory']['outcomes']=artifact(out,'pilot_outcomes.json','source outcomes and refreshed cache diagnostics')
    (out/'episode.json').write_text(json.dumps(manifest,indent=2)+'\n')
    result.update(manifest_sha256=file_hash(out/'episode.json'),representations=review['representations'],
        quality='needs_stability_review' if any('Cloth contact stability' in w for w in review['warnings']) else 'cache_checked_with_declared_limits')
    (destination/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--batch',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--ids',nargs='+');p.add_argument('--refresh-review',action='store_true')
    p.add_argument('--quality-evidence',action='append',default=[],metavar='ID=REVIEW_JSON')
    a=p.parse_args();batch=read_json(a.batch);a.output=a.output.resolve();a.output.mkdir(parents=True,exist_ok=True)
    for assignment in a.quality_evidence:
        oid,separator,value=assignment.partition('=')
        if not separator or oid not in batch['jobs']:p.error('--quality-evidence requires an existing job ID and a review path')
        path=Path(value).resolve()
        batch['jobs'][oid]['quality_evidence']=dict(path=str(path),sha256=file_hash(path))
    records=[]
    for job in batch['jobs'].values():
        if job['status']!='generated' or (a.ids and job['id'] not in a.ids):continue
        dest=a.output/job['id']
        if (dest/'result.json').exists():
            records.append(refresh_review(dest) if a.refresh_review else read_json(dest/'result.json'));continue
        try:
            result=collect(job,dest);records.append(result);print('COLLECTED',job['id'],result['observations'],flush=True)
        except Exception as exc:
            records.append(dict(id=job['id'],status='failed_collection',error=repr(exc)))
            print('COLLECTION_FAILED',job['id'],repr(exc),flush=True)
    # Include prior completed jobs even on an incremental collection call.
    by_id={r['id']:r for r in records}
    for result in a.output.glob('*/result.json'):
        r=read_json(result);by_id.setdefault(r['id'],r)
    (a.output/'index.json').write_text(json.dumps(dict(format='phenomenon-pilot/1',episodes=list(by_id.values()),
        training=False,splits=False,completed_semantics='record completeness, not numerical convergence or training admission'),ensure_ascii=False,indent=2)+'\n')
    if any(r.get('status')=='failed_collection' for r in records):raise SystemExit(1)


if __name__=='__main__':main()

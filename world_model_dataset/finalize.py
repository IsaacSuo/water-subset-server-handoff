"""Package existing results, with missing capabilities preventing acceptance."""
from pathlib import Path

import numpy as np

from .contract import artifact,validate_schema
from .io import read_json,write_json,inside


def checks_accepted(checks,missing):
    """Unavailable capabilities exclude tasks; only failed gates reject data."""
    return not missing and all(check['status']!='fail' for check in checks)


def motion_validation_summary(centroid_tests,warp_hits,frame_median_magnitudes,release_outcome):
    """Validate image motion without mistaking stationary outcomes for failures.

    Centroid displacement is a useful independent convention check for translating
    objects, but it is not a valid proxy for material motion during rotation or
    deformation.  A physically stationary R01 outcome therefore takes a separate
    zero-signal path: the derived field must remain sub-pixel and its endpoints must
    stay consistent with the previous subject segmentation.
    """
    median_hit=float(np.median(warp_hits)) if warp_hits else None
    endpoint_ok=bool(warp_hits) and median_hit>.75
    median_cosine=float(np.median([x[0] for x in centroid_tests])) if centroid_tests else None
    median_ratio=float(np.median([x[1] for x in centroid_tests])) if centroid_tests else None
    translating_ok=(len(centroid_tests)>=5 and median_cosine>.7 and .2<median_ratio<5)
    p95_frame_magnitude=(float(np.quantile(frame_median_magnitudes,.95))
                         if frame_median_magnitudes else None)
    stationary_ok=(release_outcome=='stationary' and p95_frame_magnitude is not None
                   and p95_frame_magnitude<.1)
    mode='translating' if translating_ok else 'stationary_zero_signal' if stationary_ok else 'unverified'
    return dict(passed=endpoint_ok and (translating_ok or stationary_ok),mode=mode,
                endpoint_median_hit_fraction=median_hit,centroid_test_count=len(centroid_tests),
                centroid_median_cosine=median_cosine,centroid_median_magnitude_ratio=median_ratio,
                frame_median_magnitude_p95_px=p95_frame_magnitude)


def finalize(out):
    out=Path(out);ep=read_json(out/'episode.prepared.json')
    physical=read_json(out/'physics_validation.json');native=read_json(out/'native_report.json')
    checks=list(physical['checks']);missing=list(physical['missing_required'])
    if (out/'observations/index.json').exists():
        observations=read_json(out/'observations/index.json')
        state=read_json(out/'state/index.json');times=[r['time_s'] for r in state['frames']]
        camera_ids={c['id'] for c in ep['inputs']['cameras']['cameras']}
        pairs=set();visible_pixels={};normal_tests=[];by_camera={c:[] for c in camera_ids}
        for row in observations['frames']:
            key=(row['time_s'],row['camera_id'])
            if key in pairs:raise ValueError('Duplicate camera-time observation')
            pairs.add(key)
            if not inside(out/'observations',row['rgb']).is_file():raise ValueError('Missing RGB')
            with np.load(inside(out/'observations',row['data']),allow_pickle=False) as data:
                if not np.isfinite(data['depth_m']).all():raise ValueError('Non-finite depth')
                shape=tuple(reversed(ep['inputs']['cameras']['resolution']))
                if any(data[k].shape!=shape for k in ('depth_m','depth_valid','segmentation')):raise ValueError('Observation shape mismatch')
                if data['normals_world'].shape!=shape+(3,) or data['motion_previous_minus_current_px'].shape!=shape+(2,) or data['motion_valid'].shape!=shape:raise ValueError('Normal/motion shape mismatch')
                subject_labels=[int(k) for k,v in row['segmentation_labels'].items() if v.get('class')==ep['spec']['objects'][0]['instance_id']]
                subject_mask=np.isin(data['segmentation'],subject_labels)
                visible_pixels[key]=int(np.count_nonzero(subject_mask))
                yy,xx=np.nonzero(subject_mask)
                centroid=None if not len(xx) else np.array([xx.mean(),yy.mean()])
                motion_mask=subject_mask&data['motion_valid']
                motion=np.median(data['motion_previous_minus_current_px'][motion_mask],axis=0) if np.any(motion_mask) else np.zeros(2)
                by_camera[row['camera_id']].append((row['time_s'],centroid,motion))
                fixture=read_json(out/'fixture.json')
                from scipy.spatial.transform import Rotation
                for name in ('floor','ramp'):
                    box=next((b for b in fixture['boxes'] if b['id']==name),None)
                    if box is None:continue
                    labels=[int(k) for k,v in row['segmentation_labels'].items() if v.get('class')==name]
                    mask=np.isin(data['segmentation'],labels)
                    if np.count_nonzero(mask)>20:
                        observed=np.median(data['normals_world'][mask],axis=0);observed/=np.linalg.norm(observed)
                        expected=Rotation.from_quat(box['orientation_xyzw']).apply([0,0,1])
                        normal_tests.append(float(observed@expected))
        aligned=observations['complete'] and pairs=={(t,c) for t in times for c in camera_ids}
        checks.append(dict(name='rgb_depth_segmentation_alignment',status='pass' if aligned else 'fail',detail=f'{len(pairs)} unique camera/time samples'))
        time_covered=all(any(visible_pixels.get((t,c),0)>8 for c in camera_ids) for t in times)
        camera_coverage={c:sum(visible_pixels.get((t,c),0)>8 for t in times)/len(times) for c in camera_ids}
        visibility_ok=time_covered and all(value>=.8 for value in camera_coverage.values())
        visible=sum(value>8 for value in visible_pixels.values())
        checks.append(dict(name='subject_visibility',status='pass' if visibility_ok else 'fail',
            detail=f'{visible}/{len(pairs)} views have >8 subject pixels; every time covered={time_covered}; per-camera fractions={camera_coverage}'))
        normals_ok=bool(normal_tests) and min(normal_tests)>.99
        checks.append(dict(name='normal_world_convention',status='pass' if normals_ok else 'fail',detail=f'{len(normal_tests)} fixture samples, minimum expected dot {min(normal_tests) if normal_tests else None}'))
        motion_tests=[]
        for rows in by_camera.values():
            rows.sort()
            for previous,current in zip(rows,rows[1:]):
                if previous[1] is None or current[1] is None:continue
                expected=previous[1]-current[1];observed=current[2]
                if np.linalg.norm(expected)<.25:continue
                cosine=float(expected@observed/(np.linalg.norm(expected)*max(np.linalg.norm(observed),1e-12)))
                ratio=float(np.linalg.norm(observed)/np.linalg.norm(expected))
                motion_tests.append((cosine,ratio))
        warp_hits=[];frame_median_magnitudes=[]
        subject_class=ep['spec']['objects'][0]['instance_id']
        for camera_id in camera_ids:
            rows=sorted((row for row in observations['frames'] if row['camera_id']==camera_id),
                        key=lambda row:row['time_s'])
            previous_mask=None
            for row in rows:
                with np.load(inside(out/'observations',row['data']),allow_pickle=False) as data:
                    labels=[int(k) for k,v in row['segmentation_labels'].items()
                            if v.get('class')==subject_class]
                    current_mask=np.isin(data['segmentation'],labels)
                    valid=current_mask&data['motion_valid']
                    if np.any(valid):
                        flow=data['motion_previous_minus_current_px'][valid]
                        frame_median_magnitudes.append(float(np.median(np.linalg.norm(flow,axis=1))))
                        if previous_mask is not None:
                            yy,xx=np.nonzero(valid)
                            old_x=np.rint(xx+flow[:,0]).astype(np.int64)
                            old_y=np.rint(yy+flow[:,1]).astype(np.int64)
                            in_bounds=((old_x>=0)&(old_x<current_mask.shape[1])&
                                       (old_y>=0)&(old_y<current_mask.shape[0]))
                            hit=np.zeros(len(xx),dtype=bool)
                            hit[in_bounds]=previous_mask[old_y[in_bounds],old_x[in_bounds]]
                            warp_hits.append(float(np.mean(hit)))
                    previous_mask=current_mask
        metrics=read_json(out/'metrics.json')
        motion_summary=motion_validation_summary(motion_tests,warp_hits,frame_median_magnitudes,
                                                 metrics.get('release_outcome'))
        motion_ok=motion_summary['passed']
        checks.append(dict(name='motion_vector_image_convention',status='pass' if motion_ok else 'fail',
            detail=('mode={mode}; endpoint median hit={endpoint_median_hit_fraction}; '
                    'centroid samples={centroid_test_count}, median cosine={centroid_median_cosine}, '
                    'median magnitude ratio={centroid_median_magnitude_ratio}; '
                    'frame-median magnitude p95={frame_median_magnitude_p95_px} px').format(**motion_summary)))
        if not aligned:missing.append('aligned_observations')
        if not visibility_ok:missing.append('subject_visibility')
        if not normals_ok:missing.append('verified_normal_coordinate_convention')
        if not motion_ok:missing.append('verified_motion_vectors')
    else:
        observations={};missing.append('observations')
    kind=native['physical_representation'];rigid_kind=kind.startswith('rigid');soft_kind='volumetric' in kind
    soft_evidence='capability_probes/soft_contact_impulse.json'
    ep['capabilities']={
        'rigid_contact_impulse':dict(status='native' if rigid_kind else 'not_applicable',source='PhysX contact callback',
            reason='Rigid/fixture contacts only; rigid/soft contact impulse remains unavailable' if rigid_kind and soft_kind else None,
            backend_version='Isaac Sim 6.0.1 / PhysX extension 110.1.13',evidence_path='contacts.jsonl' if rigid_kind else ''),
        'soft_contact_impulse':dict(status='unavailable' if soft_kind else 'not_applicable',source='PhysX contact capability probe',
            reason='No reliable public deformable contact impulse output; no empty, zero or estimated impulse substituted' if soft_kind else None,
            backend_version='Isaac Sim 6.0.1 / PhysX extension 110.1.13',evidence_path=soft_evidence if soft_kind else ''),
        'rgb_depth_segmentation':dict(status='native' if observations else 'unavailable',source='Isaac RTX annotators',reason=None if observations else 'Not rendered'),
        'surface_normals':dict(status='native' if observations and normals_ok else 'unavailable',source='Isaac RTX normals annotator plus analytical fixture check',reason=None if observations and normals_ok else 'Convention validation failed or not rendered'),
        'motion_vectors':dict(status='derived' if observations and motion_ok else 'unavailable',source='Fixed-topology native mesh material correspondence projected through calibrated cameras; RTX depth/segmentation visibility',reason=None if observations and motion_ok else 'Temporal convention validation failed or not rendered'),
        'shape_metrics':dict(status='derived',source='Native surface and simulation tet caches',reason=None)}
    # An explicitly unavailable, capability-masked signal is not a failed
    # physical episode. It only excludes tasks that require that capability.
    passed=checks_accepted(checks,missing)
    result=dict(schema_version='0.1.0',passed=passed,checks=checks,missing_required=sorted(set(missing)))
    validate_schema(result,'validation');write_json(out/'validation.json',result)
    ep['lifecycle']='completed'
    ep['artifacts']=[artifact(out,p.relative_to(out).as_posix()) for p in sorted(out.rglob('*'))
                     if p.is_file() and p.name not in ('episode.prepared.json','episode.json','action.json') and p.suffix!='.log']
    validate_schema(ep,'episode');write_json(out/'episode.json',ep)
    return result


if __name__=='__main__':
    import argparse,json
    parser=argparse.ArgumentParser();parser.add_argument('episode');args=parser.parse_args()
    print(json.dumps(finalize(args.episode),indent=2))

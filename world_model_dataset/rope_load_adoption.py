"""Cache-only adoption of native cable/load states; never infer cable tension."""
import argparse
import copy
import itertools
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .causal_loader import open_episode
from .io import file_hash, read_json, write_json
from .phenomenon_collect import effort_summary


def inspect(path):
    path=Path(path).resolve();ep=open_episode(path,require_complete=False)
    resolved=ep.resolved_inputs();cfg=resolved['config']
    with np.load(ep.record_path(resolved['raw_native']),allow_pickle=False) as z:
        a={k:z[k] for k in z.files}
    t=a['time'];q=a['body_q'];v=a['body_qd'];ids=a['load_body_ids']
    expected_steps=np.arange(len(t))*(cfg['physics_hz']//cfg['state_hz'])
    np.testing.assert_allclose(t,expected_steps/cfg['physics_hz'],atol=1e-10,rtol=0)
    if t[0]!=0 or t[-1]!=cfg['duration_s']:raise ValueError('Missing endpoints')
    for key in ('body_q','body_qd','centerline','centerline_velocity'):
        if not np.isfinite(a[key]).all():raise ValueError('Nonfinite native states')
    if not (np.all(a['body_mass']>0) and np.all(a['body_inv_mass']>0)
            and np.all(a['body_flags']==1) and len(a['fixed_body_ids'])==0):
        raise ValueError('Expected finite mass dynamic cable and loads')
    names={int(b):'segment_'+str(b) for b in a['segment_body_ids']}
    names.update({int(b):obj['id'] for b,obj in zip(ids,cfg['loads'])})
    count=0
    for i,(row,g) in enumerate(ep.geometries('segment_0')):
        if row['time_s']!=t[i] or row['physics_step']!=expected_steps[i]:raise ValueError('State clock mismatch')
        for key in ('body_q','body_qd','centerline','centerline_velocity'):
            np.testing.assert_array_equal(g[key],a[key][i])
        for b,oid in names.items():
            s=row['body_states'][oid]
            for key,expected in [('position_m',q[i,b,:3]),('orientation_xyzw',q[i,b,3:]),
                                 ('linear_velocity_m_s',v[i,b,:3]),('angular_velocity_rad_s',v[i,b,3:])]:
                np.testing.assert_array_equal(s[key],expected)
            # Exported COM uses float32 native quaternion arithmetic.
            np.testing.assert_allclose(s['centre_of_mass_m'],q[i,b,:3]+Rotation.from_quat(q[i,b,3:]).apply(a['body_com'][b]),atol=5e-7,rtol=0)
        count+=1
    if count!=len(t):raise ValueError('State count mismatch')
    top=ep.soft_topology('segment_0')
    for key in ('joint_parent','joint_child','joint_X_p','joint_X_c','attachment_joint_ids','segment_body_ids','load_body_ids'):
        np.testing.assert_array_equal(top[key],a[key])
    native_trace=[json.loads(line) for line in (path/'native/effort.jsonl').read_text().splitlines()]
    streams=[list(ep.controls()),list(ep.actuator_states()),list(ep.actuator_efforts())]
    n=round(cfg['duration_s']*cfg['physics_hz'])
    for stream in streams:
        if len(stream)!=n or len(native_trace)!=n:raise ValueError('Trace count mismatch')
        for k,(row,native) in enumerate(zip(stream,native_trace)):
            if {key:row[key] for key in native}!=native:raise ValueError('Native trace differs')
            if row['physics_step']!=k or abs(row['time_s']-k/cfg['physics_hz'])>1e-10:raise ValueError('Trace clock mismatch')
    for i,step in enumerate(expected_steps[:-1]):
        np.testing.assert_array_equal(native_trace[step]['actual_body_q'],q[i,ids])
        np.testing.assert_array_equal(native_trace[step]['actual_body_qd'],v[i,ids])
    force=effort_summary(streams[2],cfg['load_control']['max_force_n'])
    for row in native_trace:
        f=np.asarray(row['applied_force_world_n'])
        if np.any(f[1:]) or np.any(row['applied_torque_world_nm']):raise ValueError('Unexpected off-axis drive')
        if row['time_s']>=cfg['load_control']['release_at_s'] and np.any(f):raise ValueError('Release is not zero force')
    if ep.manifest['trajectory']['contacts']['status']!='unavailable':raise ValueError('Unexpected contact capability')
    runtime=resolved['runtime']
    if 'Newton SolverVBD' not in runtime['backend']:raise ValueError('Wrong backend')
    center=a['centerline'];ratio=np.linalg.norm(center[:,-1]-center[:,0],axis=1)/np.linalg.norm(np.diff(center,axis=1),axis=2).sum(1)
    start=np.searchsorted(t,cfg['load_control']['hold_until_s'])
    rear=q[:,ids[1],0]-q[start,ids[1],0]
    taut=np.flatnonzero(ratio>=.95);onset=np.flatnonzero((np.arange(len(t))>=start)&(rear<-.001))
    support=resolved['static_geometries']['original_support']
    with np.load(ep.record_path(support)) as z:table_top=float(z['vertices'][:,2].max())
    depths=[]
    for b in ids:
        shape=np.flatnonzero(a['shape_body']==b)
        if len(shape)!=1 or a['shape_type'][shape[0]]!=7:raise ValueError('Expected one native box shape')
        j=shape[0];local=np.array(list(itertools.product((-1,1),repeat=3)))*a['shape_scale'][j]
        local=Rotation.from_quat(a['shape_transform'][j,3:]).apply(local)+a['shape_transform'][j,:3]
        corners=np.array([Rotation.from_quat(p[b,3:]).apply(local)+p[b,:3] for p in q])
        depths.append(float(corners[:,:,2].min()-table_top))
    direct={tuple(sorted((int(i),int(j)))) for i in np.flatnonzero(a['shape_body']==ids[0]) for j in np.flatnonzero(a['shape_body']==ids[1])}
    candidates={tuple(sorted(map(int,p))) for p in a['contact_shape_pairs']}
    report=dict(episode=str(path),source_manifest_sha256=file_hash(path/'episode.json'),
        source_states_sha256=ep.manifest['trajectory']['states']['sha256'],native_sha256=file_hash(ep.record_path(resolved['raw_native'])),
        states=count,commands=n,actual_states=n,efforts=n,native_streams_exact=True,
        physics_hz=cfg['physics_hz'],state_hz=cfg['state_hz'],runtime=runtime,force=force,
        ratio_initial=float(ratio[0]),ratio_max=float(ratio.max()),
        taut_time_s=float(t[taut[0]]) if len(taut) else None,rear_onset_s=float(t[onset[0]]) if len(onset) else None,
        front_travel_m=float(q[-1,ids[0],0]-q[0,ids[0],0]),rear_travel_after_settle_m=float(rear[-1]),
        joint_gap_max_m=float(a['joint_endpoint_gap'].max()),attachment_gap_max_m=float(a['attachment_gap_m'].max()),
        box_lowest_corner_minus_table_top_m=depths,direct_box_contact_candidates=bool(direct&candidates),
        tension_contact_force_impulse='unavailable',ratio_is_tension=False)
    return cfg,a,resolved,report


def review(long_episode,short_episode,evidence,policy,output):
    cfg,a,r,current=inspect(long_episode);old_cfg,b,old_r,old=inspect(short_episode)
    before=copy.deepcopy(old_cfg);before['load_control']['displacement_m']=cfg['load_control']['displacement_m']
    before.pop('scene_request',None);after=copy.deepcopy(cfg);after.pop('scene_request',None)
    if before!=after:raise ValueError('Unexpected physical configuration change')
    for key in ('body_mass','body_inertia','body_com','joint_X_p','joint_X_c','joint_parent','joint_child',
                'shape_scale','shape_transform','shape_material_ke','shape_material_kd','shape_material_mu'):
        np.testing.assert_array_equal(a[key],b[key])
    for key in ('body_q','body_qd'):np.testing.assert_array_equal(a[key][0],b[key][0])
    if r['static_geometries']!=old_r['static_geometries']:raise ValueError('Original scene meshes changed')
    pre=a['time']<=cfg['load_control']['hold_until_s']
    drift=float(np.abs(a['body_q'][pre,:,:3]-b['body_q'][pre,:,:3]).max())
    checks=dict(native_streams_exact=True,only_target_travel_changed=True,original_scene_preserved=True,
        old_short_target_did_not_tauten=old['ratio_max']<.95,
        slack_to_taut=current['ratio_initial']<.95 and current['ratio_max']>=.95,
        rear_response_after_taut=current['taut_time_s'] is not None and current['rear_onset_s'] is not None and current['rear_onset_s']>=current['taut_time_s'],
        no_direct_box_contact_candidates=not current['direct_box_contact_candidates'])
    use=dict(status='human_use_review_pending',recommendation='conditional_candidate_for_fixed_configuration_state_prediction',
        engineering='native states, controls, actual actuator states and effort verified',
        numerical_quality='sampled box penetration and connection gaps retained; no convergence or calibrated real-world truth',
        repeatability='not certified; compared runs have different target travel',
        max_pre_pull_position_difference_m=drift,training_admission=False,
        force_supervision='unavailable',policy=dict(path=str(Path(policy).resolve()),sha256=file_hash(policy)))
    report=dict(format='rope-load-adoption/1',status='functional_cache_checked' if all(checks.values()) else 'needs_functional_review',
        checks=checks,source_states_sha256=[current['source_states_sha256']],episode=current,
        preserved_failed_short_target=old,use_review=use,training_admission=False,
        upstream_evidence=[dict(path=str(Path(p).resolve()),sha256=file_hash(p)) for p in evidence])
    write_json(output,report)
    print(report['status'],dict(states=current['states'],taut_s=current['taut_time_s'],rear_travel_m=current['rear_travel_after_settle_m']),flush=True)
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--episode',type=Path,required=True);p.add_argument('--short-episode',type=Path,required=True)
    p.add_argument('--evidence',type=Path,action='append',default=[])
    p.add_argument('--policy',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();review(a.episode,a.short_episode,a.evidence,a.policy,a.output)


if __name__=='__main__':main()

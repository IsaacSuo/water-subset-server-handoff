"""Read-only adoption check of native gripper cloth caches, without simulation."""
import argparse
import copy
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .causal_loader import open_episode
from .io import file_hash, read_json, write_json


def inspect(path):
    ep=open_episode(path,require_complete=False);resolved=ep.resolved_inputs();cfg=resolved['config']
    with np.load(ep.record_path(resolved['native_cache']),allow_pickle=False) as z:
        native={k:z[k] for k in z.files}
    attachment=read_json(ep.record_path(resolved['native_attachments']))
    np.testing.assert_array_equal(attachment['gripper']['node_ids'],native['attachment_node_ids'])
    for key in native:
        if not np.isfinite(native[key]).all():raise ValueError('Nonfinite native field: '+key)
    expected_steps=np.arange(len(native['time']))*(cfg['physics_hz']//cfg['state_hz'])
    np.testing.assert_allclose(native['time'],expected_steps/cfg['physics_hz'],atol=1e-10,rtol=0)
    np.testing.assert_array_equal(native['x'][0],native['rest_x'])
    count=0
    for i,(row,g) in enumerate(ep.geometries('cloth')):
        if row['time_s']!=native['time'][i] or row['physics_step']!=expected_steps[i]:raise ValueError('State time mismatch')
        for field,key in [('surface_world_m','x'),('surface_nodal_velocities_m_s','v')]:
            np.testing.assert_array_equal(g[field],native[key][i])
        np.testing.assert_array_equal(g['surface_triangles'],native['triangles'])
        for oid,key in [('Gripper','gripper')]+([('OpposingFixture','fixture')] if 'opposing_fixture' in cfg else []):
            state=row['body_states'][oid]
            for field,expected in [('position_m',native[key+'_pose'][i,:3]),('orientation_xyzw',native[key+'_pose'][i,3:]),
                ('linear_velocity_m_s',native[key+'_velocity'][i,:3]),('angular_velocity_rad_s',native[key+'_velocity'][i,3:])]:
                np.testing.assert_array_equal(state[field],expected)
        count+=1
    if count!=len(native['time']):raise ValueError('State count mismatch')
    commands=list(ep.controls());actual=list(ep.actuator_states());efforts=list(ep.actuator_efforts())
    if len(actual)!=count or len(commands)!=len(efforts) or len(commands)!=round(cfg['physics_hz']*cfg['duration_s']):raise ValueError('Trace count mismatch')
    for stream,field,col in [(commands,'target_position_m',1),(commands,'target_velocity_m_s',2),(efforts,'force_world_n',3)]:
        np.testing.assert_array_equal([r[field][0] for r in stream],native['effort_steps'][:,col])
        np.testing.assert_array_equal([r['time_s'] for r in stream],native['effort_steps'][:,0])
        np.testing.assert_array_equal([r['physics_step'] for r in stream],np.arange(len(stream)))
    np.testing.assert_array_equal([r['position_m'] for r in actual],native['gripper_pose'][:,:3])
    np.testing.assert_array_equal([r['linear_velocity_m_s'] for r in actual],native['gripper_velocity'][:,:3])
    np.testing.assert_array_equal(ep.soft_topology('cloth')['surface_triangles'],native['triangles'])
    from .phenomenon_collect import effort_summary
    force=effort_summary(efforts,cfg['gripper']['max_force_n'])
    if ep.manifest['trajectory']['contacts']['status']!='unavailable':raise ValueError('Unexpected contact capability')
    t=native['time'];tail=t>=4.5;dx=np.diff(native['x'],axis=0)
    report=dict(episode=str(Path(path).resolve()),states=count,commands=len(commands),efforts=len(efforts),actual_states=len(actual),
        source_manifest_sha256=file_hash(Path(path)/'episode.json'),source_states_sha256=ep.manifest['trajectory']['states']['sha256'],
        native_cache_sha256=resolved['native_cache']['sha256'],native_attachments_sha256=resolved['native_attachments']['sha256'],
        exact_native_match=True,physics_hz=cfg['physics_hz'],saved_state_hz=cfg['state_hz'],
        gripper_travel_m=float(native['gripper_pose'][-1,0]-native['gripper_pose'][0,0]),
        cloth_mean_translation_x_m=float((native['x'][-1,:,0]-native['x'][0,:,0]).mean()),
        nodes=len(native['rest_x']),triangles=len(native['triangles']),attached_nodes=len(attachment['gripper']['node_ids']),
        tail_interval_s=[4.5,5.],tail_max_node_frame_displacement_m=float(np.linalg.norm(dx,axis=-1)[t[1:]>=4.5].max()),
        tail_peak_native_speed_m_s=float(np.linalg.norm(native['v'][tail],axis=-1).max()),
        tail_native_rms_speed_m_s=float(np.sqrt(np.mean(np.sum(native['v'][tail]**2,axis=-1)))),force=force)
    return cfg,native,resolved,report


def review(paths, upstream_evidence, output, user_visual_acceptance=False):
    entries=[inspect(p) for p in paths];cfg,a,_,_=entries[0];bc,b,_,_=entries[1];lc,l,_,_=entries[2]
    other=copy.deepcopy(bc);other.pop('opposing_fixture')
    if other!=cfg:raise ValueError('Blocked changes more than opposing fixture')
    other=copy.deepcopy(lc);other['gripper']['max_force_n']=cfg['gripper']['max_force_n']
    if other!=cfg:raise ValueError('Low force changes more than force limit')
    for candidate in (b,l):
        for key in ('rest_x','triangles','time','attachment_node_ids'):np.testing.assert_array_equal(a[key],candidate[key])
        np.testing.assert_array_equal(a['x'][0],candidate['x'][0])
    scene=lambda r:{k:v['geometry']['mesh']['sha256'] for k,v in r['bodies'].items() if v['geometry']['shape']=='mesh'}
    if not all(scene(r)==scene(entries[0][2]) for _,_,r,_ in entries):raise ValueError('Scene mesh changed')
    travel=[e[3]['gripper_travel_m'] for e in entries];g=cfg['gripper']
    half_g=np.abs(Rotation.from_quat(b['gripper_pose'][:,3:]).as_matrix())@(np.asarray(g['size_m'])/2)
    half_f=np.abs(Rotation.from_quat(b['fixture_pose'][:,3:]).as_matrix())@(np.asarray(bc['opposing_fixture']['size_m'])/2)
    sep=abs(b['gripper_pose'][:,:3]-b['fixture_pose'][:,:3])-half_g-half_f
    displacement=b['gripper_pose'][:,0]-g['center_m'][0]
    checks=dict(native_streams_exact=True,only_declared_configuration_changes=True,initial_nodes_topology_timing_equal=True,
        original_scene_meshes_equal=True,free_travel=travel[0]>.1,blocked_response=travel[1]<travel[0]/2,
        low_force_response=travel[2]<travel[0]/10,blocked_no_direct_fixture_collision=bool(np.all(np.any(sep>0,axis=1))),
        blocked_no_guide_limit=bool(displacement.min()>g['guide_limits_m'][0]+.01 and displacement.max()<g['guide_limits_m'][1]-.01))
    source=Path(upstream_evidence).resolve();upstream=read_json(source)
    report=dict(format='cloth-gripper-adoption/1',status='functional_cache_checked' if all(checks.values()) else 'needs_functional_review',
        checks=checks,source_states_sha256=[e[3]['source_states_sha256'] for e in entries],episodes=[e[3] for e in entries],
        blocked_fraction_effort_at_limit_after_1_5s=float(np.mean(abs(b['effort_steps'][b['effort_steps'][:,0]>=1.5,3])>=g['max_force_n']-1e-6)),
        upstream_stability_evidence=dict(path=str(source),sha256=file_hash(source),status=upstream['status'],
            interpretation='Upstream sampled geometric and repeat-run evidence; not re-run here, not convergence or calibrated material admission'),
        visual_acceptance=dict(status='user_accepted_normal_speed_preview' if user_visual_acceptance else 'not_evaluated_by_cache_check',
            source='explicit caller record of user acceptance' if user_visual_acceptance else None,
            limits='Mild local chatter remains; original upstream quality metadata is preserved'),
        numerical_quality='improved_candidate_residual_contact_chatter',training_admission=False,
        contact_and_attachment_reaction='unavailable')
    write_json(output,report);print(report['status'],dict(travel_m=travel),flush=True)
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--episodes',nargs=3,type=Path,required=True,metavar='FREE_BLOCKED_LOW')
    p.add_argument('--upstream-evidence',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--user-visual-acceptance',action='store_true',help='Record explicit user acceptance of the supplied preview; never inferred from cache checks')
    a=p.parse_args();review(a.episodes,a.upstream_evidence,a.output,a.user_visual_acceptance)


if __name__=='__main__':main()

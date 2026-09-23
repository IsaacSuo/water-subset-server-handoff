"""Read new original-pipe pair with source-face mapping and native runtime identity."""
import argparse,copy,json,hashlib,zipfile
from pathlib import Path
import numpy as np
import trimesh
from world_model_dataset.causal_loader import open_episode
from world_model_dataset.experiment_review import verify_alignment
from world_model_dataset.io import read_json,write_json,file_hash,digest
from scripts.pin_wrap_delivery import verify_maps

def main():
 p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();out={};invariants=[];runtime_maps=[];actual=[]
 for name in ('W_G01_equal','W_G01_unequal'):
  f=a.root/name;doc=read_json(f/'generated/experiment.json');construction=read_json(f/'generated/construction.json');cal=construction['calculations'];identity=construction['support_identity'];j=read_json(f/'execution/workflow.json')['jobs']['baseline'];root=Path(j['stages']['observe']['result']['episode']);ep=open_episode(root,require_complete=True);rows=list(ep.states());alignment=verify_alignment(ep,doc['timing'],observed=True);carrier=next(k for k,v in rows[0]['body_states'].items() if 'geometry' in v and 'topology' in v);geometry=list(ep.geometries(carrier));topology=ep.soft_topology(carrier);maps=verify_maps(root/'inputs')
  selected=np.asarray(identity['planning_section']['original_group_triangle_indices']);group=identity['group']
  with np.load(root/'inputs'/(group+'_face_mapping.npz')) as m:
   native_ids=np.flatnonzero(np.isin(m['backend_to_original_face'],selected));assert set(m['backend_to_original_face'][native_ids])==set(selected);assert np.all(m['backend_to_object'][native_ids]==identity['object_index'])
  execution_path=next((f/'execution/jobs').glob('*/attempt_001/*/execution.json'));executed=read_json(execution_path);prepared=execution_path.parent/'prepared';entry=Path(doc['backend']['entry']).parent
  for source,sha in executed['sources'].items():
   fixed=entry/source if source!='newton_io.py' else entry.parent/'generation/vendor/experiments/newton_backend_probe/common/newton_io.py'
   assert file_hash(prepared/source)==sha==file_hash(fixed),source
  rt=read_json(root/'native/runtime_sources.json');archive=root/'native'/rt['source_archive'];assert file_hash(archive)==rt['source_archive_sha256']
  with zipfile.ZipFile(archive) as zz:
   assert set(zz.namelist())==set(rt['source_sha256'])
   assert all(hashlib.sha256(zz.read(k)).hexdigest()==v for k,v in rt['source_sha256'].items())
  runtime_maps.append(rt['source_sha256']);cfg=copy.deepcopy(doc);cfg.pop('id');cfg.pop('conditions');cfg['scene']['source_records'].pop('construction');cfg['input']['rope'].pop('centerline_m');invariants.append(digest(cfg))
  raw=root/'native/states.npz'
  with np.load(raw) as z:
   arrays={k:z[k] for k in z.files};points=z['centerline'];ends=z['segment_end_world_m'].reshape(len(z['time']),-1,2,3);t=z['time'];q=z['body_q'];v=z['body_qd'];mass=z['body_mass'];lengths=z['segment_lengths'];gap=z['joint_endpoint_gap'];assert len(z['fixed_body_ids'])==0;assert str(z['body_velocity_order'][0])=='linear_xyz_angular_xyz';assert np.isfinite(q).all() and np.isfinite(v).all()
   actual.append({k:z[k].copy() for k in ('body_mass','body_inertia','segment_lengths','shape_scale','shape_material_mu','joint_target_ke','joint_target_kd')})
  assert len(geometry)==len(t)==121
  for i,(_,g) in enumerate(geometry):np.testing.assert_array_equal(g['centerline'],points[i])
  policy=read_json(root/'inputs'/(group+'_collision_policy.json'))
  with np.load(root/'inputs'/policy['original_copy']) as z:tri=z['triangles'][selected];verts=z['vertices'];mesh=trimesh.Trimesh(verts,tri,process=False)
  samples=(ends[:,:,0,None,:]+(ends[:,:,1,:]-ends[:,:,0,:])[:,:,None,:]*np.linspace(0,1,11)[None,None,:,None]).reshape(-1,3)
  _,distance,_=trimesh.proximity.closest_point(mesh,samples);rr=doc['input']['rope']['radius_m'];minclear=float(distance.min()-rr);center=np.asarray(cal['rod_center_m']);rad=cal['rod_envelope_radius_m'];cross=points[:,:,0]-center[0];height=points[:,:,2]-center[2];u=(cross.min(1)<0)&(cross.max(1)>0)&(height.max(1)>0)&(height[:,0]<0)&(height[:,-1]<0);below=points[:,:,2].max(1)+rr<center[2]-rad;lost=np.flatnonzero(below);tail=t>=t[-1]-.5
  candidates=0;steps=[]
  with (root/'native/contact_candidates.jsonl').open() as stream:
   for line in stream:
    record=json.loads(line);steps.append(record['step']);candidates+=len(record['rigid_contact_shape0'])
  assert len(steps)==960 and len(set(steps))==960
  potential=float((mass*q[0,:,2]).sum()*9.81);out[name]=dict(episode=str(root),manifest_sha256=file_hash(root/'episode.json'),native_sha256=file_hash(raw),physics_origin='new entry-line execution, not external cache reuse',fixed_backend_revision='7011d1771058b19eef1a0d3cb93c74a1e6952c3f',executed_backend_sources=executed['sources'],executed_backend_matches_fixed_snapshot=True,alignment=alignment,geometry_carrier=carrier,geometry_frames=len(geometry),topology_fields=list(topology),collision_maps=maps,selected_original_faces=selected.tolist(),selected_backend_face_ids=native_ids.tolist(),full_original_collision_retained=True,runtime_archive_sha256=rt['source_archive_sha256'],runtime_source_content_sha256=digest(rt['source_sha256']),runtime_source_files=len(rt['source_sha256']),runtime_git_status=rt['newton_git'],leg_segments=cal['leg_segments'],actual_segment_length_range_m=[float(lengths.min()),float(lengths.max())],initial_total_mass_kg=float(mass.sum()),initial_mass_centroid_m=np.average(q[0,:,:3],axis=0,weights=mass).tolist(),initial_potential_world_origin_j=potential,final_centerline_bounds_m=[points[-1].min(0).tolist(),points[-1].max(0).tolist()],endpoint_displacement_m=(points[-1,[0,-1]]-points[0,[0,-1]]).tolist(),last_u_proxy_time_s=float(t[np.flatnonzero(u)[-1]]) if u.any() else None,final_u_proxy=bool(u[-1]),first_wholly_below_target_rod_time_s=float(t[lost[0]]) if len(lost) else None,remains_below_after_first=bool(below[lost[0]:].all()) if len(lost) else False,max_joint_gap_m=float(gap.max()),sampled_min_selected_rod_surface_clearance_m=minclear,surface_sample_axis_spacing_max_m=float(np.linalg.norm(ends[:,:,1]-ends[:,:,0],axis=2).max()/10),last_half_second_peak_body_speed_m_s=float(np.linalg.norm(v[tail,:,:3],axis=2).max()),last_half_second_peak_body_angular_speed_rad_s=float(np.linalg.norm(v[tail,:,3:],axis=2).max()),candidate_steps=len(steps),candidate_records=candidates,contact_force='unavailable',native_tension='unavailable',training_admission=False,limitations=['U proxy is geometric, not a topological/contact guarantee','sampled surfaces only; no frame-between penetration or convergence certificate','pre-solve candidates are not contact reaction truth','late falling rope can leave fixed diagnostic camera'])
 assert invariants[0]==invariants[1];assert runtime_maps[0]==runtime_maps[1]
 differences={}
 for k in actual[0]:
  difference=float(np.abs(actual[0][k]-actual[1][k]).max());differences[k]=difference
  if k=='shape_scale':
   # Native capsule half-length uses float32 world endpoint differences.
   bound=float(np.spacing(np.float32(np.abs(points[0]).max())));assert difference<=bound
  elif k=='segment_lengths':assert difference<1e-12
  else:np.testing.assert_array_equal(actual[0][k],actual[1][k],err_msg=k)
 write_json(a.output,dict(cases=out,nonintervention_sha256=invariants[0],same_executed_runtime_sources=True,native_nonintervention_arrays_checked=list(actual[0]),native_max_absolute_differences=differences,capsule_half_length_float32_world_ulp_bound_m=bound,qualification='descriptive equal/unequal layout comparison only; mainline review pending'))
 print({k:{kk:v[kk] for kk in ('final_u_proxy','first_wholly_below_target_rod_time_s','max_joint_gap_m','sampled_min_selected_rod_surface_clearance_m','last_half_second_peak_body_angular_speed_rad_s')} for k,v in out.items()})
if __name__=='__main__':main()

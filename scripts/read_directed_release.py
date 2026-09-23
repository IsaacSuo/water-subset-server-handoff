"""Actual passive release pose, contact and invariant audit; no outcome expectation filter."""
import argparse
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from world_model_dataset.io import read_json,write_json,file_hash,digest
from world_model_dataset.causal_loader import open_episode

def main():
 p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();out={};frozen={}
 for asset in ('vase','rock'):
  frozen[asset]=[]
  for side in ('restoring','tipping'):
   name='R_'+asset+'_'+side;f=a.root/name;doc=read_json(f/'generated/experiment.json');cal=read_json(f/'generated/construction.json')['calculations'];j=read_json(f/'execution/workflow.json')['jobs']['baseline'];root=Path(j['stages']['observe']['result']['episode']);ep=open_episode(root,require_complete=True);rows=list(ep.states());s=[r['body_states']['subject'] for r in rows];time=np.array([r['time_s'] for r in rows]);pos=np.array([r['position_m'] for r in s]);q=Rotation.from_quat([r['orientation_xyzw'] for r in s]);base=np.asarray(cal['base_rotation_matrix']);up=base.T@np.array([0,0,1]);angle=np.degrees(np.arccos(np.clip(q.apply(up)[:,2],-1,1)));v=np.asarray([r['linear_velocity_m_s'] for r in s]);w=np.asarray([r['angular_velocity_rad_s'] for r in s]);contacts=list(ep.contacts());sep=np.array([c['separation_m'] for c in contacts]);resolved=read_json(root/'resolved_inputs.json');body=resolved['bodies']['subject'];invariant={k:doc[k] for k in ('backend','scene','control','actuation','timing','observations')};invariant['scene']=dict(invariant['scene']);invariant['scene']['source_records']={k:v for k,v in invariant['scene']['source_records'].items() if k!='construction'};invariant.update(resolved_geometry=body['geometry'],resolved_physics=body['physics'],numerics=resolved['numerics']);frozen[asset].append(digest(invariant));assert np.linalg.norm(v[0])==0 and np.linalg.norm(w[0])==0
   meta=body['geometry']['physical_asset_provenance']['asset_manifest'];source=Path(body['geometry']['physical_asset_source']['directory'])
   with np.load(source/meta['geometry']) as zz:vertices=zz['vertices'].astype(float)*doc['input']['participants'][0]['size_m']
   bounds=np.array([[pp+(vertices@rr.T).min(0),pp+(vertices@rr.T).max(0)] for pp,rr in zip(pos,q.as_matrix())]);swept=np.array([bounds[:,0].min(0)-.01,bounds[:,1].max(0)+.01]);roi=np.asarray(doc['scene']['region_bounds_m']);assert np.all(swept[0]>roi[0]) and np.all(swept[1]<roi[1])
   out[name]=dict(sampled_whole_surface_envelope_plus_10mm_m=swept.tolist(),initial_com_above_planning_barrier_m=cal['initial_com_height_above_support_m']-cal['quasistatic_barrier_height_m'],episode=str(root),manifest_sha256=file_hash(root/'episode.json'),states=len(rows),native_contacts=len(contacts),tilt_from_reference_up_deg=dict(initial=float(angle[0]),final=float(angle[-1]),min=float(angle.min()),max=float(angle.max())),critical_planning_angle_deg=cal['critical_angle_deg'],position_displacement_m=(pos[-1]-pos[0]).tolist(),initial_com_height_m=cal['initial_com_height_above_support_m'],planned_initial_potential_j=cal['initial_potential_j'],last_half_second_peak_speed_m_s=float(np.linalg.norm(v[time>=time[-1]-.5],axis=1).max()),last_half_second_peak_angular_speed_rad_s=float(np.linalg.norm(w[time>=time[-1]-.5],axis=1).max()),minimum_native_separation_m=float(sep.min()) if len(sep) else None,contact_first_last_s=[contacts[0]['time_s'],contacts[-1]['time_s']] if contacts else None,contact_note='native callbacks may stop when body sleeps; absence of later callbacks is not loss of support',training_admission=False)
  assert frozen[asset][0]==frozen[asset][1],asset+' nonintervention changed'
 scene=read_json(a.root/'garage_geometry/scene.json');lo,hi=np.asarray(scene['selection_bounds_m']);sweep=[]
 for excluded in scene['excluded_outside_neighbourhood']:
  b=np.asarray(excluded['bounds_m'])
  if np.all(b[1]>=lo) and np.all(b[0]<=hi):sweep.append(excluded['name'])
 write_json(a.output,dict(cases=out,pair_nonintervention_sha256=frozen,garage_original_support_mesh_sha256=scene['groups']['support']['sha256'],excluded_objects_with_AABB_neighbourhood_overlap=sweep,qualification='descriptive poses/contact changes; no calibrated ceramic, force truth or convergence claim'))
 print(out)
if __name__=='__main__':main()

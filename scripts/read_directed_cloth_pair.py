"""Read a completed passive-cloth pair; geometric support proxies are not forces."""
import argparse,hashlib
from pathlib import Path
import numpy as np
from world_model_dataset.io import read_json,write_json,file_hash,digest
from world_model_dataset.causal_loader import open_episode

def array_hash(a):return hashlib.sha256(str(a.dtype).encode()+str(a.shape).encode()+np.ascontiguousarray(a).tobytes()).hexdigest()
def main():
 p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();results={};common=[]
 for name in ('F_rect_25','F_rect_45'):
  f=a.root/name;doc=read_json(f/'generated/experiment.json');c=read_json(f/'generated/construction.json')['calculations'];j=read_json(f/'execution/workflow.json')['jobs']['baseline'];ep=open_episode(j['stages']['observe']['result']['episode'],require_complete=True);rows=list(ep.states());geometry=list(ep.geometries('cloth'));g0=geometry[0][1];gf=geometry[-1][1];x=g0['surface_world_m'];y=gf['surface_world_m'];edge=c['edge_x_m'];z=c['support_height_m'];initial_supported=x[:,0]<edge;free=x[:,0]>edge
  with np.load(Path(j['stages']['observe']['result']['episode'])/'native/frames.npz') as data:
   rest=data['rest_x'];tri=data['triangles'];uv=data['uv'];v=data['v'];xt=data['x'];time=data['time']
  # Native rectangle rest nodes may be placed in world coordinates; UV and face identity are intrinsic.
  local=np.column_stack(((uv.astype(float)-.5)*np.asarray(doc['input']['cloth']['size_m']),np.zeros(len(uv))))
  world=(np.c_[local,np.ones(len(local))]@np.asarray(doc['input']['cloth']['world_from_mesh']).T)[:,:3]
  error=float(np.abs(world-x).max());assert error<2e-6
  mesh={'canonical_local_vertices':array_hash(local),'uv':array_hash(uv),'triangles':array_hash(tri),'requested_rectangle':doc['input']['cloth']['size_m'],'cells':doc['input']['cloth']['cells']}
  common.append(mesh)
  edges=np.unique(np.sort(np.concatenate([tri[:,[0,1]],tri[:,[1,2]],tri[:,[2,0]]]),axis=1),axis=0);rest_len=np.linalg.norm(x[edges[:,1]]-x[edges[:,0]],axis=1);final_len=np.linalg.norm(y[edges[:,1]]-y[edges[:,0]],axis=1)
  near=lambda pts:((pts[:,0]<=edge)&(np.abs(pts[:,2]-z)<=.008))
  results[name]=dict(episode=str(ep.root) if hasattr(ep,'root') else j['stages']['observe']['result']['episode'],manifest_sha256=file_hash(Path(j['stages']['observe']['result']['episode'])/'episode.json'),states=len(rows),observations=51,actual_t0_mesh_mapping_max_error_m=error,mesh=mesh,mesh_content_sha256=digest(mesh),initial_supported_area_fraction=c['supported_area_fraction'],centroid_displacement_m=(y.mean(0)-x.mean(0)).tolist(),final_min_z_m=float(y[:,2].min()),final_below_support_depth_m=float(z-y[:,2].min()),initially_supported_nodes=int(initial_supported.sum()),initially_free_nodes=int(free.sum()),final_initial_support_nodes_near_support_fraction=float(near(y)[initial_supported].mean()),final_all_nodes_near_support_fraction=float(near(y).mean()),support_node_mean_xy_shift_m=(y[initial_supported,:2]-x[initial_supported,:2]).mean(0).tolist(),free_nodes_mean_down_m=float((x[free,2]-y[free,2]).mean()),final_edge_length_ratio_quantiles=np.quantile(final_len/rest_len,[0,.5,.95,1]).tolist(),last_half_second_peak_node_speed_m_s=float(np.linalg.norm(v[time>=time[-1]-.5],axis=2).max()),support_proxy='native node x at/before original edge and z within 8 mm of original support top; node fraction, not area or measured contact',contact_force='unavailable',training_admission=False)
 assert common[0]==common[1]
 docs=[read_json(a.root/n/'generated/experiment.json') for n in results];assert all(docs[0][k]==docs[1][k] for k in ('backend','timing','control','actuation','observations'));assert all(docs[0]['input'][k]==docs[1]['input'][k] for k in docs[0]['input'] if k!='cloth');assert docs[0]['input']['cloth'].keys()==docs[1]['input']['cloth'].keys()
 write_json(a.output,dict(group='F',same_intrinsic_mesh_content=True,cases=results,qualification='descriptive trajectory only; mainline review pending'))
 print(results)
if __name__=='__main__':main()

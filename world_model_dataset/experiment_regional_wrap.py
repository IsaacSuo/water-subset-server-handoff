"""Bounded original-rod selection and uniform-segment U planning; collision unedited."""
import copy
import numpy as np
from .experiment_contract import keys
from .experiment_geometry import clip,require
from .io import read_json,file_hash

def regional_wrap(obj,cond,profile,geo,support):
 from .experiment_construct import number,passage_camera
 keys(obj,{'length_m','radius_m','density_kg_m3'},{'length_m','radius_m','density_kg_m3'},'wrap rope')
 keys(cond,{'end_condition','left_leg_fraction'},{'end_condition','left_leg_fraction'},'wrap conditions')
 keys(support,{'group','object_index','section_axis'},{'group','object_index','section_axis'},'original bounded rod selector')
 require(support['section_axis']=='Y','rod_axis','bounded section currently supports original-world Y straight rods only')
 require(cond['end_condition']=='free','end_condition','directed uniform U rule requires two free ends')
 fraction=number(cond,'left_leg_fraction');require(.3<=fraction<=.7,'leg_fraction','0.3..0.7')
 length=number(obj,'length_m');r=number(obj,'radius_m');number(obj,'density_kg_m3');group=support['group'];index=support['object_index']
 manifest=read_json(geo.scene['source_records']['scene']);require(group in manifest['groups'] and type(index) is int and 0<=index<len(manifest['groups'][group]['objects']),'rod_selection','unknown original object')
 info=manifest['groups'][group];sel=info['objects'][index];spec=next((m for m in geo.scene['collision']['meshes'] if m['id']==group),None)
 require(spec is not None and file_hash(spec['path'])==info['sha256'],'rod_source','complete original collision group required')
 tri=geo.meshes[group][sel['triangle_start']:sel['triangle_start']+sel['triangle_count']];lo,hi=geo.bounds
 mask=np.all(tri.max(1)>=lo,axis=1)&np.all(tri.min(1)<=hi,axis=1);ids=np.flatnonzero(mask);points=[]
 for t in tri[mask]:
  clipped=clip(clip(t,1,lo[1],True),1,hi[1],False)
  if len(clipped):points.extend(clipped)
 require(len(points)>0,'rod_section','no original triangles in section')
 points=np.unique(points,axis=0);center=(points.min(0)+points.max(0))/2
 require(np.all(points.min(0)[[0,2]]>lo[[0,2]]+1e-5) and np.all(points.max(0)[[0,2]]<hi[[0,2]]-1e-5),'rod_section','ROI clips cross-section; enlarge or isolate another section')
 radial=np.linalg.norm(points[:,[0,2]]-center[[0,2]],axis=1);radius=float(radial.max());bar_length=float(np.ptp(points[:,1]));require(bar_length>8*radius and radius>0,'rod_shape','section must be slender')
 require(radial.min()>.9*radius,'rod_section','non-round/merged cross-section or intersecting fixtures; selection unavailable')
 # Verify cross-section exists at both ends, not merely a finite isolated fragment.
 for yy in (lo[1],hi[1]):
  ring=points[np.abs(points[:,1]-yy)<1e-7];require(len(ring)>=8,'rod_section','not a continuous pipe through entire section')
 n=int(np.ceil(length/profile['discretization_m']));require(12<=n<=128,'segments','12..128 uniform segments');step=length/n;gap=max(.002,r);need=radius+r+gap
 require(step<2*need,'segment_length','segment too long relative to selected rod')
 arc_n=int(np.ceil(np.pi/(2*np.arcsin(step/(2*need)))))
 if (n-arc_n)%2:arc_n+=1
 require(n-arc_n>=8 and length<bar_length,'wrap_length','need arc, at least eight leg segments, and longer original rod section')
 R=step/(2*np.sin(np.pi/(2*arc_n)));angles=np.linspace(np.pi,0,arc_n+1);arc=center+R*np.column_stack([np.cos(angles),np.zeros(len(angles)),np.sin(angles)])
 nl=int(round((n-arc_n)*fraction));nr=n-arc_n-nl
 path=np.concatenate([arc[0]-np.arange(nl,0,-1)[:,None]*[0,0,step],arc,arc[-1]-np.arange(1,nr+1)[:,None]*[0,0,step]])
 require(np.allclose(np.linalg.norm(np.diff(path,axis=0),axis=1),step,rtol=0,atol=1e-12),'segment_lengths','uniform length derivation failed')
 # Cover each native capsule by shorter conservative boxes. This only refines
 # the clearance test, never changes native capsule length or collision mesh.
 for a,b in zip(path[:-1],path[1:]):
  probes=np.linspace(a,b,max(2,int(np.ceil(step/r))+1))
  for c,d in zip(probes[:-1],probes[1:]):geo.check_box(np.minimum(c,d)-r,np.maximum(c,d)+r,'wrap_clearance')
 cfg=copy.deepcopy(profile['input']);cfg['rope']=dict(copy.deepcopy(profile['rope_properties']),centerline_m=path.tolist(),radius_m=r,density_kg_m3=obj['density_kg_m3'],end_condition='free')
 geo.support_identity=dict(group=group,object_index=index,original_object=sel,mesh_sha256=info['sha256'],planning_section=dict(axis='Y',bounds_m=geo.bounds.tolist(),original_group_triangle_indices=(ids+sel['triangle_start']).tolist(),object_local_triangle_indices=ids.tolist(),operation='analysis-only Y clipping; entire original collision group retained'))
 target=center-[0,0,(n-arc_n)*step/4]
 # High original pipes may be hidden behind ceilings from an upward view.
 # Test a bounded set of below-side cameras against the complete original mesh.
 import trimesh
 triangles=np.concatenate(list(geo.meshes.values()));mesh=trimesh.Trimesh(triangles.reshape(-1,3),np.arange(triangles.size//3).reshape(-1,3),process=False)
 anchors=np.array([arc[0]-[0,0,(n-arc_n)*step*.7],arc[-1]-[0,0,(n-arc_n)*step*.7],center+[0,0,R]])
 span=max(.25,length*2);camera=None
 for direction in ([.8,-.25,-.5],[-.8,-.25,-.5],[0,-.8,-.5],[0,.8,-.5]):
  origin=target+span*np.array(direction);rays=anchors-origin;lens=np.linalg.norm(rays,axis=1)
  hits,indices,_=mesh.ray.intersects_location(np.repeat(origin[None],len(anchors),axis=0),rays/lens[:,None],multiple_hits=True)
  if len(hits) and np.any(np.linalg.norm(hits-origin,axis=1)<lens[indices]-1e-5):continue
  camera=dict(target_m=target.tolist(),position_m=origin.tolist(),ortho_scale_m=span);break
 require(camera is not None,'observation_visibility','no tested below-side camera sees both leg envelopes and top')
 return cfg,dict(design='uniform-segment upper U around bounded original Y pipe section',rod_center_m=center.tolist(),rod_axis=[0,1,0],rod_envelope_radius_m=radius,wrap_radius_m=float(R),initial_clearance_margin_m=gap,centerline_length_m=length,segment_count=n,segment_length_m=step,arc_segments=arc_n,leg_segments=[nl,nr],requested_left_leg_fraction=fraction,effective_left_leg_fraction=nl/(nl+nr),endpoint_heights_m=[float(path[0,2]),float(path[-1,2])],observation_camera=camera,limits='round horizontal original Y pipe section only, free ends, leg fraction quantized to equal segments; box-cover initial clearance, no guarantee of future retained wrap, no native tension/contact force truth')

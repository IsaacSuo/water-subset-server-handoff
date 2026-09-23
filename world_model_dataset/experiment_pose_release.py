"""Passive release about a geometry-derived support boundary, no imposed motion."""
import copy
from pathlib import Path
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from shapely.geometry import MultiPoint,Point,box
from .experiment_contract import keys
from .experiment_geometry import require
from .io import read_json

def pose_release(obj,cond,profile,geo,support):
 from .experiment_construct import body_geometry,passage_camera,number
 keys(obj,{'id','asset','size_m','mass_kg','density_kg_m3','material'},{'id','asset','size_m'},'pose release object')
 keys(cond,{'release_side'},{'release_side'},'pose release condition')
 require(cond['release_side'] in ('restoring','tipping'),'release_side','restoring or tipping candidate')
 library=profile['asset_library'];path=Path(library['exploration_root'])/library['library']/obj['asset'];meta=read_json(path/'asset.json');geo.pin(path/'asset.json')
 require(geo.pin(path/meta['geometry'])==meta['geometry_sha256'],'asset_hash','geometry changed')
 with np.load(path/meta['geometry']) as z:vertices=z['vertices'].astype(float)*number(obj,'size_m')
 require(np.isfinite(vertices).all(),'asset_geometry','finite normalized asset required')
 hull=trimesh.convex.convex_hull(vertices);poses,prob=trimesh.poses.compute_stable_poses(hull,center_mass=[0,0,0],sigma=0,n_samples=1)
 require(len(poses)>0,'stable_pose','no convex stable pose')
 heights=[np.ptp(vertices@t[:3,:3].T,axis=0)[2] for t in poses];i=int(np.argmax(heights));base=poses[i,:3,:3];v=vertices@base.T
 band=min(.0005,obj['size_m']*.0025);ids=np.flatnonzero(v[:,2]<=v[:,2].min()+band);poly=MultiPoint(v[ids,:2]).convex_hull
 require(poly.geom_type=='Polygon' and poly.contains(Point(0,0)),'support_polygon','COM not strictly inside contact-band support polygon')
 coords=np.asarray(poly.exterior.coords);candidates=[]
 for a,b in zip(coords[:-1],coords[1:]):
  d=b-a;u=float(np.clip(-a@d/(d@d),0,1));q=a+u*d;candidates.append((np.linalg.norm(q),q,a,b))
 distance,q,a,b=min(candidates,key=lambda t:t[0]);height=-v[:,2].min();require(height>0 and distance>band,'support_barrier','unresolved support margin')
 outward=q/distance;axis=np.array([-outward[1],outward[0],0]);critical=float(np.arctan2(distance,height));factor={'restoring':.65,'tipping':1.35}[cond['release_side']];angle=critical*factor
 require(angle<np.pi/3,'release_angle','rule limited to tilts under 60 degrees')
 tilt=Rotation.from_rotvec(axis*angle).as_matrix();rotation=tilt@base;o=copy.deepcopy(obj);o['rotation_deg']=Rotation.from_matrix(rotation).as_euler('xyz',degrees=True).tolist();low,high=body_geometry(o,library,geo)
 tolerance=2e-5;z,surface=geo.support(support,planarity_tolerance_m=tolerance);lo,hi=geo.bounds;pivot=np.r_[q,v[:,2].min()];xy=(lo[:2]+hi[:2])/2-(tilt@pivot)[:2];gap=.002;position=np.r_[xy,z+gap-low[2]]
 require(surface.buffer(1e-7).covers(box(*(position+low)[:2],*(position+high)[:2])),'support_coverage','posed footprint exceeds original support')
 radius=float(np.linalg.norm(vertices,axis=1).max());require(surface.covers(box(*(xy-radius),*(xy+radius))),'release_neighbourhood','whole rotational sweep requires original support')
 geo.check_box(position+low,position+high,'pose_initial_clearance');geo.check_box(np.r_[xy-radius,z+gap],np.r_[xy+radius,z+2*radius+gap],'release_sweep_clearance')
 body=dict(o,xy_m=xy.tolist(),support_group=support,ray_start_z_m=z+.0001,clearance_m=gap,velocity_m_s=[0,0,0],angular_velocity_rad_s=[0,0,0])
 mass=obj.get('mass_kg');target=np.r_[(lo[:2]+hi[:2])/2,z+obj['size_m']*.4];span=max(.6,obj['size_m']*3)
 return dict(participants=[body],asset_library=copy.deepcopy(library)),dict(design='passive tilted release about nearest support-polygon boundary from tallest convex stable pose',candidate=cond['release_side'],base_rotation_matrix=base.tolist(),release_rotation_matrix=rotation.tolist(),tilt_axis_world=axis.tolist(),critical_angle_deg=float(np.degrees(critical)),release_angle_deg=float(np.degrees(angle)),support_band_m=band,support_vertex_indices=ids.tolist(),support_edge_xy_m=[a.tolist(),b.tolist()],reference_pivot_m=pivot.tolist(),stable_com_height_m=float(height),initial_com_height_above_support_m=float(position[2]-z),initial_potential_j=(float(mass*9.81*(position[2]-z)) if mass is not None else None),quasistatic_barrier_height_m=float(np.hypot(height,distance)),planning_convex_hull_only=True,collision_representation='unchanged physical asset SDF; original concave surface retained',floor_planarity_tolerance_m=tolerance,initial_clearance_m=gap,observation_camera=passage_camera(geo,target,target+[0,0,.03],span),limits='normalized source COM zero and uniform volumetric inertia assumption; contact-band polygon and convex quasistatic barrier plan candidates, not dynamic outcome; no calibrated ceramic material or convergence claim')

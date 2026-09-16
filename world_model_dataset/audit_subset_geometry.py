"""Small post-run RGB-D/state cross-check, not a new simulation admission gate."""
import argparse
from pathlib import Path

import numpy as np
from scipy.ndimage import binary_erosion
from scipy.spatial.transform import Rotation
import trimesh

from .causal_subset import CausalSubset
from .io import read_json,write_json


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True)
    a=p.parse_args();report={};mesh_cache={}
    for item,episode in CausalSubset(a.root).episodes():
        root=episode.root;resolved=episode.resolved_inputs()
        index=read_json(episode.record_path(episode.manifest['trajectory']['observations']))
        states={r['physics_step']:r for r in episode.states()}
        steps=sorted({f['physics_step'] for f in index['frames']});chosen={steps[0],steps[len(steps)//2],steps[-1]}
        local={}
        for oid,b in resolved['bodies'].items():
            g=b['geometry']
            if g['shape']=='mesh':
                key=g['mesh']['sha256']
                if key not in mesh_cache:
                    with np.load(root/g['mesh']['path']) as d:mesh_cache[key]=trimesh.Trimesh(d['vertices'],d['triangles'],process=False)
                local[oid]=mesh_cache[key]
            elif g['shape']=='box':local[oid]=trimesh.creation.box(g['size_m'])
            else:local[oid]=trimesh.creation.icosphere(subdivisions=4,radius=g['radius_m'])
        errors=[];mismatches=0;missing=0;details=[]
        for row in index['frames']:
            if row['physics_step'] not in chosen:continue
            cal=index['cameras'][row['camera_id']];matrix=np.asarray(cal['world_from_camera_usd']);K=np.asarray(cal['intrinsic_opencv'])
            with np.load(root/'observations'/row['data']) as d:
                seg=d['segmentation'];depth=d['depth_m'];pixels=[]
                for label in index['body_instance_ids'].values():
                    yy,xx=np.where(binary_erosion(seg==label,iterations=2))
                    if len(yy):
                        for i in np.linspace(0,len(yy)-1,min(6,len(yy)),dtype=int):pixels.append((xx[i],yy[i]))
                if not pixels:continue
                xy=np.asarray(pixels);directions=np.c_[(xy[:,0]-K[0,2])/K[0,0],-(xy[:,1]-K[1,2])/K[1,1],-np.ones(len(xy))]@matrix[:3,:3]
                origin=np.tile(matrix[3,:3],(len(xy),1));nearest=np.full(len(xy),np.inf);hitlabels=np.zeros(len(xy),int)
                for oid,mesh in local.items():
                    s=states[row['physics_step']]['body_states'][oid];rotation=Rotation.from_quat(s['orientation_xyzw'])
                    pts,ray,_=mesh.ray.intersects_location(rotation.inv().apply(origin-s['position_m']),rotation.inv().apply(directions),multiple_hits=True)
                    if not len(pts):continue
                    pts=rotation.apply(pts)+s['position_m'];z=-((pts-origin[ray])@matrix[:3,:3].T)[:,2]
                    for j,value in zip(ray,z):
                        if 0<value<nearest[j]:nearest[j]=value;hitlabels[j]=index['body_instance_ids'][oid]
                finite=np.isfinite(nearest);err=np.abs(nearest[finite]-depth[xy[finite,1],xy[finite,0]])
                mis=int((hitlabels[finite]!=seg[xy[finite,1],xy[finite,0]]).sum())
                errors.extend(err.tolist());mismatches+=mis;missing+=int((~finite).sum())
                details.append(dict(step=row['physics_step'],camera=row['camera_id'],rays=len(pixels),label_mismatches=mis,max_depth_error_m=float(err.max()) if len(err) else None))
        report[item['id']]=dict(rays=len(errors),label_mismatches=mismatches,missing_intersections=missing,
            median_depth_error_m=float(np.median(errors)),max_depth_error_m=max(errors),samples=details,
            note='visible source mesh ray checks at first/middle/last states; spheres approximated by high-resolution mesh, not exact RTX tessellation')
        print('GEOMETRY_CHECK',item['id'],report[item['id']]['median_depth_error_m'],report[item['id']]['max_depth_error_m'],mismatches,flush=True)
    write_json(a.root/'geometry_observation_review.json',report)


if __name__=='__main__':main()

"""Prepare closed real-asset inputs for the material line's scene-mesh interface.

This only prepares inputs. The chosen backend remains responsible for simulation,
its supported material model and raw state output. No convex/primitive fallback.
"""
import argparse
import copy
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh

from .io import read_json, write_json, file_hash


def prepare(template, package, output, size, xy, clearance, spacing, rotation_deg):
    cfg=copy.deepcopy(read_json(template));metadata=read_json(package/'asset.json')
    source=package/metadata['geometry']
    if file_hash(source)!=metadata['geometry_sha256']:raise ValueError('Asset mesh hash mismatch')
    with np.load(source) as z:v=z['vertices']*size;f=z['triangles']
    mesh=trimesh.Trimesh(v,f,process=False)
    if not mesh.is_watertight or not mesh.is_winding_consistent:raise ValueError('Closed consistent source required')
    output.mkdir(parents=True,exist_ok=False)
    local=output/'appearance_reference.npz';np.savez_compressed(local,vertices=v,triangles=f)
    rotation=Rotation.from_euler('xyz',rotation_deg,degrees=True);posed=rotation.apply(v)
    # The template names an original mesh, not a fitted support plane.
    support=Path(cfg['environment'][0]['path'])
    if not support.is_absolute():support=template.parent/support
    with np.load(support) as z:floor=trimesh.Trimesh(z['vertices'],z['triangles'],process=False)
    hits,_,_=floor.ray.intersects_location([[*xy,5.]],[[0,0,-1]],multiple_hits=False)
    if len(hits)!=1:raise ValueError('No original support at requested XY')
    position=np.array([*xy,hits[0,2]+clearance-posed[:,2].min()]);transform=np.eye(4)
    transform[:3,:3]=rotation.as_matrix();transform[:3,3]=position
    cfg['object']=dict(kind='closed_mesh',path=str(local.resolve()),spacing_m=spacing,
        density_kg_m3=cfg['object']['density_kg_m3'],world_from_mesh=transform.tolist())
    for item in cfg['environment']:
        item['path']=str((template.parent/item['path']).resolve())
    for key,path in cfg.get('source_records',{}).items():cfg['source_records'][key]=str((template.parent/path).resolve())
    cfg['source_records']['asset']=str((package/'asset.json').resolve())
    cfg['scope']=('Real closed source asset '+metadata['asset_id']+
        ' with an assigned experimental material on unchanged original scene meshes; '
        'not a measurement of the scanned object material.')
    cfg['appearance_request']=dict(asset_package=str(package.resolve()),size_m=size,
        source_local_mesh=str(local.resolve()),position_m=position.tolist(),rotation_xyzw=rotation.as_quat().tolist(),
        mapping='derived source-UV surface from material-point neighborhoods; not native surface topology')
    cfg['source_asset_parameters']=dict(max_extent_m=size,rotation_deg=rotation_deg,
        original_support_hit_m=hits[0].tolist(),initial_clearance_m=clearance,
        material_semantics='experimental assigned material; not measured properties of the scanned object')
    write_json(output/'config.json',cfg)
    return output/'config.json'


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--template',type=Path,required=True)
    p.add_argument('--library',type=Path,required=True)
    p.add_argument('--assets',nargs='+',required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--size',type=float,required=True)
    p.add_argument('--xy',type=float,nargs=2,required=True)
    p.add_argument('--clearance',type=float,default=.12)
    p.add_argument('--spacing',type=float,default=.0075)
    p.add_argument('--rotation-deg',type=float,nargs=3,default=[0,0,0])
    a=p.parse_args()
    for asset in a.assets:
        if Path(asset).name!=asset:raise ValueError('Expected an asset directory name')
        print(prepare(a.template.resolve(),a.library/asset,a.output/asset,a.size,a.xy,
                      a.clearance,a.spacing,a.rotation_deg))


if __name__=='__main__':main()

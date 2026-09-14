"""Calibrated fixed-topology material motion for replayed mesh caches."""
from __future__ import annotations

import numpy as np


def project_world(points_world_m, calibration):
    """Project world points to image pixels and return optical-axis depth."""
    points=np.asarray(points_world_m,dtype=np.float64)
    world_from_camera=np.asarray(calibration['world_from_camera_usd'],dtype=np.float64)
    camera_from_world=np.linalg.inv(world_from_camera)
    homogeneous=np.concatenate((points,np.ones((len(points),1))),axis=1)
    usd_camera=homogeneous@camera_from_world
    cv=np.column_stack((usd_camera[:,0],-usd_camera[:,1],-usd_camera[:,2]))
    intrinsic=np.asarray(calibration['intrinsic_opencv'],dtype=np.float64)
    uv=np.column_stack((intrinsic[0,0]*cv[:,0]/cv[:,2]+intrinsic[0,2],
                        intrinsic[1,1]*cv[:,1]/cv[:,2]+intrinsic[1,2]))
    return uv,cv[:,2]


def fixed_topology_motion(previous_world_m,current_world_m,triangles,depth_m,
                          segmentation,subject_labels,calibration):
    """Return previous-minus-current pixel motion for visible subject material.

    Each current subject pixel is assigned to the closest projected current
    triangle. Perspective-correct barycentric coordinates identify the same
    material point in the previous fixed-topology mesh. RTX depth and semantic
    segmentation are used only for visibility/disambiguation.
    """
    previous=np.asarray(previous_world_m,dtype=np.float64)
    current=np.asarray(current_world_m,dtype=np.float64)
    triangles=np.asarray(triangles,dtype=np.int64)
    depth=np.asarray(depth_m,dtype=np.float64)
    segmentation=np.asarray(segmentation)
    height,width=depth.shape
    motion=np.zeros((height,width,2),dtype=np.float32)
    valid=np.zeros((height,width),dtype=bool)
    if previous.shape!=current.shape or previous.ndim!=2 or previous.shape[1]!=3:
        raise ValueError('Motion meshes must have matching Nx3 vertices')
    if triangles.ndim!=2 or triangles.shape[1]!=3:
        raise ValueError('Motion topology must contain triangles')
    subject=np.isin(segmentation,list(subject_labels))&np.isfinite(depth)&(depth>0)
    if not np.any(subject):return motion,valid
    current_uv,current_z=project_world(current,calibration)
    previous_uv,previous_z=project_world(previous,calibration)
    best=np.full((height,width),np.inf,dtype=np.float64)
    for face in triangles:
        uv=current_uv[face];z=current_z[face]
        if np.any(z<=0) or not np.isfinite(uv).all():continue
        xmin=max(0,int(np.floor(uv[:,0].min())));xmax=min(width-1,int(np.ceil(uv[:,0].max())))
        ymin=max(0,int(np.floor(uv[:,1].min())));ymax=min(height-1,int(np.ceil(uv[:,1].max())))
        if xmin>xmax or ymin>ymax:continue
        yy,xx=np.mgrid[ymin:ymax+1,xmin:xmax+1]
        x=xx.astype(np.float64);y=yy.astype(np.float64)
        x0,y0=uv[0];x1,y1=uv[1];x2,y2=uv[2]
        denominator=(y1-y2)*(x0-x2)+(x2-x1)*(y0-y2)
        if abs(denominator)<1e-12:continue
        b0=((y1-y2)*(x-x2)+(x2-x1)*(y-y2))/denominator
        b1=((y2-y0)*(x-x2)+(x0-x2)*(y-y2))/denominator
        b2=1.-b0-b1
        inside=(b0>=-1e-6)&(b1>=-1e-6)&(b2>=-1e-6)&subject[ymin:ymax+1,xmin:xmax+1]
        if not np.any(inside):continue
        inv_depth=b0/z[0]+b1/z[1]+b2/z[2]
        positive=inv_depth>0
        rendered_depth=1./np.where(positive,inv_depth,np.nan)
        error=np.abs(rendered_depth-depth[ymin:ymax+1,xmin:xmax+1])
        local_best=best[ymin:ymax+1,xmin:xmax+1]
        choose=inside&positive&(error<local_best)
        if not np.any(choose):continue
        # Perspective-correct material barycentrics.
        weights=np.stack((b0/z[0],b1/z[1],b2/z[2]),axis=-1)/inv_depth[...,None]
        old_uv=weights@previous_uv[face]
        candidate=old_uv-np.stack((x,y),axis=-1)
        local_motion=motion[ymin:ymax+1,xmin:xmax+1]
        local_valid=valid[ymin:ymax+1,xmin:xmax+1]
        local_motion[choose]=candidate[choose]
        local_valid[choose]=True
        local_best[choose]=error[choose]
    return motion,valid

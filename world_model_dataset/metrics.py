"""Derived geometric metrics, never substitutes for native contact truth."""
import numpy as np


def nonrigid_residual(rest,current):
    rest=np.asarray(rest,dtype=np.float64);current=np.asarray(current,dtype=np.float64)
    if rest.shape!=current.shape or rest.ndim!=2 or rest.shape[1]!=3:
        raise ValueError('Shape mismatch')
    a=rest-rest.mean(0);b=current-current.mean(0)
    u,_,vt=np.linalg.svd(a.T@b);r=u@vt
    if np.linalg.det(r)<0:u[:,-1]*=-1;r=u@vt
    residual=b-a@r
    return float(np.sqrt(np.mean(np.sum(residual**2,axis=1))))


def recovery_time(times,values,release_time,threshold,hold_s=.5):
    """First sustained return; None means right-censored, not never recovers."""
    times=np.asarray(times);values=np.asarray(values)
    for i,t in enumerate(times):
        if t<release_time or values[i]>threshold:continue
        j=np.searchsorted(times,t+hold_s-1e-8)
        if j<len(times) and np.all(values[i:j+1]<=threshold):return float(t-release_time)
    return None


def box_penetration(vertices,fixture_boxes,positions):
    """Vertex-in-OBB depth, diagnostic only: not complete triangle intersection."""
    from scipy.spatial.transform import Rotation
    worst=0.
    for b in fixture_boxes:
        if b['id']=='gate':continue  # release disables it; use actual contacts instead
        rotation=Rotation.from_quat(b['orientation_xyzw']).as_matrix()
        local=(vertices-np.asarray(positions[b['id']]))@rotation
        margin=np.asarray(b['size_m'])/2-np.abs(local)
        interior=(margin>0).all(axis=1)
        if interior.any():worst=max(worst,float(margin[interior].min(axis=1).max()))
    return worst


def triangle_box_surface_audit(vertices,triangles,fixture_boxes,positions):
    """Clip every candidate triangle against analytical OBB volumes.

    This is a complete captured-frame surface scan, unlike vertex-only tests.
    The reported depth samples every clipped polygon vertex, edge midpoint and
    centroid; exact crossing count comes from nondegenerate clipped polygons.
    """
    from scipy.spatial.transform import Rotation
    vertices=np.asarray(vertices,dtype=np.float64);triangles=np.asarray(triangles,dtype=np.int64).reshape(-1,3)
    exact=0;worst=0.;tested=0

    def clip(poly,axis,limit,keep_less):
        if not len(poly):return poly
        result=[];previous=poly[-1];pin=previous[axis]<=limit+1e-10 if keep_less else previous[axis]>=limit-1e-10
        for current in poly:
            cin=current[axis]<=limit+1e-10 if keep_less else current[axis]>=limit-1e-10
            if cin!=pin:
                amount=(limit-previous[axis])/(current[axis]-previous[axis])
                result.append(previous+amount*(current-previous))
            if cin:result.append(current)
            previous,pin=current,cin
        return np.asarray(result,dtype=np.float64)

    for b in fixture_boxes:
        if b['id']=='gate':continue
        rotation=Rotation.from_quat(b['orientation_xyzw']).as_matrix()
        local=(vertices-np.asarray(positions[b['id']]))@rotation
        half=np.asarray(b['size_m'],dtype=np.float64)/2
        tri=local[triangles]
        candidates=np.flatnonzero(np.all(tri.max(axis=1)>=-half-1e-10,axis=1)&np.all(tri.min(axis=1)<=half+1e-10,axis=1))
        tested+=len(candidates)
        for index in candidates:
            poly=tri[index]
            for axis in range(3):
                poly=clip(poly,axis,half[axis],True)
                poly=clip(poly,axis,-half[axis],False)
                if len(poly)<3:break
            if len(poly)<3:continue
            area=sum(np.linalg.norm(np.cross(poly[i]-poly[0],poly[i+1]-poly[0]))/2 for i in range(1,len(poly)-1))
            if area<=1e-14:continue
            exact+=1
            samples=[*poly,poly.mean(axis=0)]
            samples.extend((poly[i]+poly[(i+1)%len(poly)])/2 for i in range(len(poly)))
            margin=half-np.abs(np.asarray(samples))
            worst=max(worst,float(np.max(np.minimum.reduce(margin,axis=1))))
    return {'candidate_triangle_tests':tested,'intersecting_triangles':exact,'maximum_sampled_penetration_m':worst,
            'method':'complete analytical OBB clipping over all captured surface triangles; depth over clipped vertices, edge midpoints and centroid'}


def tet_boundary_faces(tets):
    from collections import Counter
    tets=np.asarray(tets,dtype=np.int64).reshape(-1,4)
    faces=[]
    for a,b,c,d in tets:faces.extend(((b,c,d),(a,c,d),(a,b,d),(a,b,c)))
    counts=Counter(tuple(sorted(map(int,f))) for f in faces)
    return np.asarray([f for f in faces if counts[tuple(sorted(map(int,f))) ]==1],dtype=np.int32)

"""Low-cost calibrated RGB/depth/ID observations of cached physical geometry.

This is a diagnostic observation representation, not textured appearance or
contact truth. No implicit floor, smoothing, simulation or invented connectivity.
"""
import numpy as np
from scipy.spatial.transform import Rotation
import trimesh


def camera(design,width=160,height=120):
    origin=np.asarray(design['position_m'],float);target=np.asarray(design['target_m'],float)
    forward=target-origin;distance=np.linalg.norm(forward);forward/=distance
    right=np.cross(forward,[0,0,1]);right/=np.linalg.norm(right);down=np.cross(forward,right)
    rotation=np.column_stack((right,down,forward))
    focal=width*distance/design.get('ortho_scale_m',1.2)
    k=np.array([[focal,0,(width-1)/2],[0,focal,(height-1)/2],[0,0,1.]])
    return dict(resolution=[width,height],projection='perspective',intrinsic_opencv=k.tolist(),
                world_from_camera_cv=rotation.tolist(),camera_origin_m=origin.tolist(),
                pixel_centres='integer centres',depth_semantics='optical Z metres; 0 invalid',near_m=.005)


class Raster:
    def __init__(self,cam):
        self.cam=cam;self.width,self.height=cam['resolution'];self.k=np.asarray(cam['intrinsic_opencv'])
        self.rotation=np.asarray(cam['world_from_camera_cv']);self.origin=np.asarray(cam['camera_origin_m']);self.near=cam['near_m']
        self.depth=np.full((self.height,self.width),np.inf)
        self.seg=np.zeros((self.height,self.width),np.uint16)
        self.rgb=np.full((self.height,self.width,3),230,np.uint8)
        self.yy,self.xx=np.mgrid[:self.height,:self.width]
        self.rays=np.stack(((self.xx-self.k[0,2])/self.k[0,0],(self.yy-self.k[1,2])/self.k[1,1],np.ones_like(self.xx)),-1)

    def mesh(self,vertices,faces,body_id,color):
        pc=(np.asarray(vertices)-self.origin)@self.rotation
        triangles=pc[np.asarray(faces,dtype=int)]
        kept=triangles[np.min(triangles[:,:,2],axis=1)>=self.near]
        crossing=triangles[(np.min(triangles[:,:,2],axis=1)<self.near)&(np.max(triangles[:,:,2],axis=1)>=self.near)]
        clipped=[]
        for triangle in crossing:
            poly=[]
            for a,b in zip(triangle,np.roll(triangle,-1,axis=0)):
                if a[2]>=self.near:poly.append(a)
                if (a[2]>=self.near)!=(b[2]>=self.near):poly.append(a+(b-a)*((self.near-a[2])/(b[2]-a[2])))
            for i in range(1,len(poly)-1):clipped.append([poly[0],poly[i],poly[i+1]])
        if clipped:kept=np.concatenate((kept,np.asarray(clipped)))
        if not len(kept):return
        uv=kept[:,:,:2]/kept[:,:,2,None]*np.diag(self.k)[:2]+self.k[:2,2]
        lo=np.floor(uv.min(axis=1)).astype(int);hi=np.ceil(uv.max(axis=1)).astype(int)
        visible=(hi[:,0]>=0)&(hi[:,1]>=0)&(lo[:,0]<self.width)&(lo[:,1]<self.height)
        for triangle,screen,lower,upper in zip(kept[visible],uv[visible],lo[visible],hi[visible]):
            x0,y0=np.maximum(lower,[0,0]);x1,y1=np.minimum(upper+1,[self.width,self.height])
            a,b,c=screen;den=(b[1]-c[1])*(a[0]-c[0])+(c[0]-b[0])*(a[1]-c[1])
            if abs(den)<1e-12:continue
            px=self.xx[y0:y1,x0:x1];py=self.yy[y0:y1,x0:x1]
            u=((b[1]-c[1])*(px-c[0])+(c[0]-b[0])*(py-c[1]))/den
            v=((c[1]-a[1])*(px-c[0])+(a[0]-c[0])*(py-c[1]))/den
            w=1-u-v;inv=u/triangle[0,2]+v/triangle[1,2]+w/triangle[2,2]
            hit=np.divide(1.,inv,out=np.full_like(inv,np.inf),where=inv>0)
            mask=(u>=-1e-9)&(v>=-1e-9)&(w>=-1e-9)&(hit>=self.near)&(hit<self.depth[y0:y1,x0:x1])
            if not mask.any():continue
            n=np.cross(triangle[1]-triangle[0],triangle[2]-triangle[0]);n/=max(np.linalg.norm(n),1e-30)
            shade=.4+.6*abs(n[2])
            self.depth[y0:y1,x0:x1][mask]=hit[mask]
            self.seg[y0:y1,x0:x1][mask]=body_id
            self.rgb[y0:y1,x0:x1][mask]=np.clip(np.asarray(color)*shade*255,0,255).astype(np.uint8)

    def spheres(self,points,radii,body_id,color):
        # MPM observation glyphs use the recorded initial particle support radius.
        # They do not define a continuum surface or change native material fields.
        pc=(points-self.origin)@self.rotation
        for c,r in zip(pc,radii):
            if c[2]-r<=self.near:raise ValueError('Camera intersects a particle glyph')
            bounds=[]
            for axis in (0,1):
                p=[self.k[axis,axis]*(c[axis]+a*r)/(c[2]+b*r)+self.k[axis,2] for a in (-1,1) for b in (-1,1)]
                bounds.append((int(np.floor(min(p))),int(np.ceil(max(p)))+1))
            x0,x1=max(0,bounds[0][0]),min(self.width,bounds[0][1]);y0,y1=max(0,bounds[1][0]),min(self.height,bounds[1][1])
            if x0>=x1 or y0>=y1:continue
            ray=self.rays[y0:y1,x0:x1];aa=(ray*ray).sum(-1);bb=ray@c;disc=bb*bb-aa*(c@c-r*r)
            hit=(bb-np.sqrt(np.maximum(disc,0)))/aa
            mask=(disc>=0)&(hit>self.near)&(hit<self.depth[y0:y1,x0:x1])
            self.depth[y0:y1,x0:x1][mask]=hit[mask];self.seg[y0:y1,x0:x1][mask]=body_id
            self.rgb[y0:y1,x0:x1][mask]=np.asarray(color)*255

    def arrays(self):
        valid=np.isfinite(self.depth)
        return dict(depth_m=np.where(valid,self.depth,0).astype(np.float32),depth_valid=valid,segmentation=self.seg)


def posed(vertices,state):
    return Rotation.from_quat(state['orientation_xyzw']).apply(vertices)+state['position_m']


def tet_boundary(tets, points):
    """Extract the exterior faces of recorded Tets, without adding vertices."""
    tets=np.asarray(tets,dtype=int);points=np.asarray(points)
    faces=np.concatenate([tets[:,f] for f in ((1,2,3),(0,3,2),(0,1,3),(0,2,1))])
    opposite=np.concatenate([tets[:,i] for i in range(4)])
    _,inverse,counts=np.unique(np.sort(faces,axis=1),axis=0,return_inverse=True,return_counts=True)
    if np.any(counts>2):raise ValueError('Nonmanifold Tet boundary')
    keep=counts[inverse]==1;faces=faces[keep];opposite=opposite[keep]
    tri=points[faces]
    inward=np.einsum('ij,ij->i',np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]),points[opposite]-tri[:,0])>0
    faces[inward]=faces[inward][:,[0,2,1]]
    return faces


class GeometryView:
    def __init__(self,episode):
        self.ep=episode;self.resolved=episode.resolved_inputs();self.first=next(episode.states());self.fixed={};self.local={}
        self.body_ids={b['instance_id']:i+1 for i,b in enumerate(episode.manifest['system']['bodies'])}
        self.subjects=[b['instance_id'] for b in episode.manifest['system']['bodies'] if b['role']=='subject']
        self.native=None;self.particle_radii=None;self.boundaries={}
        self.volume_representation='native cached display surface; simulation Tet fields remain authoritative'
        if 'config' in self.resolved and 'bodies' not in self.resolved:
            self.kind=self.resolved['config']['kind']
            for oid,record in self.resolved['static_geometries'].items():
                with np.load(episode.record_path(record)) as z:self.fixed[oid]=(z['vertices'],z['triangles'])
            if self.kind=='rope':
                with np.load(episode.record_path(self.resolved['raw_native'])) as z:
                    self.native={key:z[key] for key in ('shape_body','shape_transform','shape_scale','body_ids')}
                    for key in ('segment_body_ids','load_body_ids','shape_type'):
                        if key in z:self.native[key]=z[key]
                segments=self.native.get('segment_body_ids',self.native['body_ids'])
                names={int(b):'segment_'+str(b) for b in segments}
                loads=self.resolved['config'].get('loads',[])
                load_ids=self.native.get('load_body_ids',[])
                if len(loads)!=len(load_ids):raise ValueError('Native load identity count mismatch')
                names.update({int(b):obj['id'] for b,obj in zip(load_ids,loads)})
                for b,oid in names.items():
                    indices=np.flatnonzero(self.native['shape_body']==b)
                    if len(indices)!=1:raise ValueError('Expected one recorded shape per rope/load body')
                    idx=indices[0]
                    if b in segments:
                        r,half=self.native['shape_scale'][idx,:2]
                        mesh=trimesh.creation.capsule(radius=r,height=2*half,count=[8,12])
                    else:
                        # Recorded Newton GeoType.BOX, scale stores half extents.
                        if self.native['shape_type'][idx]!=7:raise ValueError('Unsupported native load shape')
                        mesh=trimesh.creation.box(extents=2*self.native['shape_scale'][idx])
                    # trimesh capsule is centred on Z; retain recorded shape pose.
                    transform=self.native['shape_transform'][idx]
                    self.local[oid]=(Rotation.from_quat(transform[3:]).apply(mesh.vertices)+transform[:3],mesh.faces)
            if self.kind=='plastic':
                top=episode.soft_topology('mpm_block')
                with np.load(episode.record_path(top['static'])) as z:self.particle_radii=z['particle_radius_m']
        else:
            self.kind=self.resolved.get('config',{}).get('kind') or ('volume' if any(b['physics_kind']=='volumetric' for b in episode.manifest['system']['bodies']) else 'rigid')
            if self.kind=='beam':self.volume_representation='exterior faces extracted from native simulation Tets; original node identities, no smoothing or reconstructed surface'
            for oid,definition in self.resolved['bodies'].items():
                g=definition['geometry'];state=self.first['body_states'][oid]
                if g['shape']=='soft_box':
                    if self.kind=='beam':
                        with np.load(episode.record_path(state['geometry'])) as z:
                            self.boundaries[oid]=tet_boundary(z['simulation_tets'],z['simulation_world_m'])
                    continue
                if g['shape']=='mesh':
                    with np.load(episode.record_path(g['mesh'])) as z:v,f=z['vertices'],z['triangles']
                elif g['shape']=='box':
                    mesh=trimesh.creation.box(extents=g['size_m']);v,f=mesh.vertices,mesh.faces
                else:raise ValueError('Unsupported observation shape '+g['shape'])
                body=next(b for b in episode.manifest['system']['bodies'] if b['instance_id']==oid)
                if body['physics_kind']=='static':self.fixed[oid]=(posed(v,state),f)
                else:self.local[oid]=(v,f)

    def render_static(self,cam):
        raster=Raster(cam)
        for oid,(v,f) in self.fixed.items():raster.mesh(v,f,self.body_ids[oid],[.45,.38,.3])
        return raster

    def render(self,raster,row):
        for oid,(v,f) in self.local.items():raster.mesh(posed(v,row['body_states'][oid]),f,self.body_ids[oid],[.85,.4,.13])
        for oid in self.subjects:
            state=row['body_states'][oid]
            if 'geometry' not in state or oid.startswith('segment_'):continue
            with np.load(self.ep.record_path(state['geometry'])) as g:
                if oid in self.boundaries:raster.mesh(g['simulation_world_m'],self.boundaries[oid],self.body_ids[oid],[.15,.5,.7])
                elif 'surface_world_m' in g:raster.mesh(g['surface_world_m'],g['surface_triangles'],self.body_ids[oid],[.15,.5,.7])
                elif 'particle_world_m' in g:raster.spheres(g['particle_world_m'],self.particle_radii,self.body_ids[oid],[.8,.32,.13])
        return raster

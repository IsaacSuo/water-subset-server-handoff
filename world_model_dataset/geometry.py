"""CPU-only diagnostic meshes. D=max rest extent; mass=density*closed volume."""
from __future__ import annotations

import numpy as np
import trimesh


def make_geometry(entry):
    kind=entry['generator']
    if kind=='sphere':
        mesh=trimesh.creation.icosphere(subdivisions=3,radius=.5)
    elif kind=='cylinder':
        mesh=trimesh.creation.cylinder(radius=.5,height=.75,sections=48)
    elif kind=='rounded_cube':
        sphere=trimesh.creation.icosphere(subdivisions=2,radius=.12)
        points=np.concatenate([sphere.vertices+np.array([x,y,z]) for x in (-.38,.38) for y in (-.38,.38) for z in (-.38,.38)])
        mesh=trimesh.convex.convex_hull(points)
        mesh=mesh.subdivide().subdivide()
    elif kind=='capsule':
        mesh=trimesh.creation.capsule(height=.6,radius=.2,count=[16,24])
    elif kind=='l_shape':
        polygon=np.array([[0,0],[1,0],[1,.35],[.35,.35],[.35,1],[0,1]])
        vertices=np.concatenate([np.column_stack((polygon,np.full(6,z))) for z in (-.2,.2)])
        top=np.array([[0,1,3],[1,2,3],[0,3,5],[3,4,5]])
        faces=list(top[:,::-1])+list(top+6)
        for i in range(6):
            j=(i+1)%6;faces.extend([[i,j,j+6],[i,j+6,i+6]])
        mesh=trimesh.Trimesh(vertices=vertices,faces=faces,process=True)
    elif kind=='ring':
        n,m=48,16;points=[];faces=[]
        for i in range(n):
            a=2*np.pi*i/n
            for j in range(m):
                b=2*np.pi*j/m;r=.35+.15*np.cos(b)
                points.append([r*np.cos(a),r*np.sin(a),.15*np.sin(b)])
                k=i*m+j;l=((i+1)%n)*m+j;u=i*m+(j+1)%m;v=((i+1)%n)*m+(j+1)%m
                faces.extend([[k,l,v],[k,v,u]])
        mesh=trimesh.Trimesh(vertices=points,faces=faces,process=True)
    else:
        raise ValueError(f'Unimplemented geometry: {kind}')
    mesh.fix_normals()
    if not mesh.is_watertight or mesh.volume<=0:
        raise ValueError(f'Geometry not a closed positive volume: {kind}')
    mesh.apply_scale(entry['characteristic_size_m']/max(mesh.extents))
    mesh.apply_translation(-mesh.center_mass)
    # The meshed sphere approximates the analytic collision sphere. Its analytic
    # mass/inertia are used for rigid spheres and this distinction is recorded.
    return mesh


def save_geometry(output, inputs):
    from pathlib import Path
    from .io import write_json
    output=Path(output);folder=output/'geometry';folder.mkdir()
    report={}
    for o in inputs['objects']:
        mesh=make_geometry(o['geometry']);mass=mesh.volume*o['physics']['density_kg_m3']
        inertia=mesh.moment_inertia*o['physics']['density_kg_m3']
        semantics='closed_visual_mesh'
        if o['physics']['kind']=='rigid' and o['geometry']['rigid_collision']=='sphere':
            r=o['geometry']['characteristic_size_m']/2
            mass=4*np.pi*r**3/3*o['physics']['density_kg_m3'];inertia=np.eye(3)*(2*mass*r*r/5)
            semantics='analytic_collision_sphere'
        with (folder/(o['instance_id']+'.npz')).open('xb') as f:
            np.savez(f,vertices=np.asarray(mesh.vertices,dtype=np.float32),triangles=np.asarray(mesh.faces,dtype=np.int32),
                     mass_kg=mass,inertia_kg_m2=inertia)
        report[o['instance_id']]={'path':'geometry/'+o['instance_id']+'.npz','rest_volume_m3':float(mesh.volume),
                                  'mass_kg':float(mass),'mass_semantics':semantics,
                                  'rest_bounds_m':mesh.bounds.tolist(),'watertight':bool(mesh.is_watertight)}
    write_json(output/'geometry/index.json',report)
    return report

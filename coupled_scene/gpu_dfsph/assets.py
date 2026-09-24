"""Scene import and deterministic boundary sampling, with no CPU fluid solve."""
import json
from pathlib import Path

import numpy as np
import trimesh

from coupled_scene.newton_dfsph.assets import load_assets
from coupled_scene.newton_dfsph.surface_assets import load_surface_layout, tank_mesh


def prepare(root,case,output):
    root=Path(root);output=Path(output);output.mkdir(parents=True,exist_ok=False)
    if case['id']=='free_body_interface':
        tank=dict(inner_size_m=[.24,.16,.12],proxy_thickness_m=.016)
        wall=tank_mesh(tank);cube=trimesh.creation.box(extents=[.032]*3)
        cube.apply_translation([.01,0,0])  # Exercise an offset mesh origin/COM.
        with np.load(root/'output/coupled_scenes/sph_unified_20260924/seed/common.npz') as seed:
            p=seed['positions']-np.array([.16,.04,.1],np.float32)
        arrays=dict(positions=p,velocities=np.zeros_like(p),receiver_vertices=np.asarray(wall.vertices,np.float32),
            receiver_triangles=np.asarray(wall.faces,np.int32),donor_vertices=np.asarray(cube.vertices,np.float32),donor_triangles=np.asarray(cube.faces,np.int32))
        meta=dict(case=dict(id=case['id'],motion=None),scene_family='interface_fixture',receiver=dict(position_m=[0,0,0]),
            donor=dict(position_m=[0,.075,0]),moving_object='FreeCube',particle_count=len(p))
        ids=np.arange(len(p),dtype=np.uint32);origin=np.zeros(3);spacing=.004
    else:
        path=root/case['source']
        meta,arrays,ids,origin,spacing=(load_surface_layout(path) if case['family']=='surface_study' else load_assets(path))
    body_specs=[];clouds=[];saved=dict(arrays,source_ids=ids)
    for name in ('receiver','donor'):
        if not len(arrays[name+'_vertices']):continue
        mesh=trimesh.Trimesh(arrays[name+'_vertices'],arrays[name+'_triangles'],process=False)
        samples=np.asarray(mesh.voxelized(pitch=spacing).fill().points,np.float32)
        if len(samples)==0:raise ValueError('Empty rigid boundary sampling')
        saved[name+'_samples']=samples
        position=np.asarray(meta[name]['position_m'])-origin
        moving=name=='donor'
        body_specs.append(dict(name=name,objectId=len(body_specs)+1,translation=position.tolist(),quaternion_xyzw=[0,0,0,1],
            isDynamic=moving,prescribed=moving and case['id']!='free_body_interface',density=500 if case['id']=='free_body_interface' and moving else 1000))
        clouds.append(mesh.vertices+position)
        mesh.export(output/(name+'.obj'))
    bounds=np.vstack(clouds+[arrays['positions']])
    # A numerical search domain only; scene walls remain the imported meshes.
    # Include authored apparatus sweeps; no inlet or particle removal is enabled.
    lower=bounds.min(0)-.5;upper=bounds.max(0)+.5
    shift=-lower
    for b in body_specs:b['translation']=(np.asarray(b['translation'])+shift).tolist()
    saved['positions']=arrays['positions']+shift
    np.savez(output/'input.npz',**saved)
    info=dict(case=case,source_assets=meta,origin_m=origin.tolist(),simulation_shift_m=shift.tolist(),
        spacing_m=spacing,body_specs=body_specs,domain_end=(upper-lower).tolist(),
        fluid_particles=len(ids),boundary_particles={b['name']:len(saved[b['name']+'_samples']) for b in body_specs})
    (output/'input.json').write_text(json.dumps(info,indent=2,ensure_ascii=False),encoding='utf-8')
    return info


def scene_configuration(info,dt):
    common=dict(geometryFile='imported-arrays',rotationAxis=[0,1,0],rotationAngle=0,scale=[1,1,1],velocity=[0,0,0],color=[70,120,180],entryTime=-1)
    return dict(Configuration=dict(domainStart=[0,0,0],domainEnd=info['domain_end'],addDomainBox=False,
        particleRadius=info['spacing_m']/2,density0=1000,gravitation=[0,-9.81,0],simulationMethod='dfsph',
        viscosityMethod='standard',viscosity=.001,viscosity_b=.001,timeStepSize=dt,exportObj=False),
        FluidBodies=[dict(common,objectId=0,translation=[0,0,0],density=1000)],
        RigidBodies=[dict(common,**b) for b in info['body_specs']])

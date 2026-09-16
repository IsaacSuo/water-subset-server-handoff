"""Blender cache-only original-scene replay with verified glTF/body alignment.

Run with Blender --background --python this_file -- --plan replay_plan.json.
No source scene or physics cache is modified. Appearance and physics stay separate.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys

import bpy
import numpy as np
from mathutils import Matrix, Vector, Quaternion
from mathutils.kdtree import KDTree


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def checked(path, expected):
    if sha(path) != expected:
        raise ValueError('Source/cache hash mismatch: '+str(path))
    return path


def vertices(obj):
    array=np.empty((len(obj.data.vertices),3),dtype=np.float64)
    obj.data.vertices.foreach_get('co',array.ravel())
    return array


def max_nearest(a,b):
    tree=KDTree(len(b))
    for i,point in enumerate(b):tree.insert(point,i)
    tree.balance()
    return max(tree.find(point)[2] for point in a)


def import_asset(oid, geometry, episode):
    provenance=geometry['physical_asset_provenance']
    package=Path(provenance['source_package'])
    metadata=read(checked(package/'asset.json',provenance['asset_manifest_sha256']))
    entry=package/metadata['appearance']['entry']
    checked(entry,metadata['source_sha256'])
    before=set(bpy.data.objects)
    bpy.ops.import_scene.gltf(filepath=str(entry))
    imported=set(bpy.data.objects)-before
    bpy.context.view_layer.update()
    size=provenance['parameters']
    scale=np.repeat(size['max_extent_m'],3) if 'max_extent_m' in size else np.asarray(size['extents_m'])/metadata['normalized_extents']
    # Blender's standard glTF importer has already changed Y-up source coordinates
    # to Z-up. Undo precisely that basis before the source-frame normalization.
    importer_basis=Matrix.Rotation(np.pi/2,4,'X')
    normalize=Matrix(metadata['source_scene_to_normalized'])
    body_from_import=Matrix.Diagonal((*scale,1.)) @ normalize @ importer_basis.inverted()
    root=bpy.data.objects.new('cache_'+oid,None)
    bpy.context.scene.collection.objects.link(root)
    root.rotation_mode='QUATERNION'
    parts=[];points=[]
    for obj in imported:
        if obj.type!='MESH':continue
        transform=body_from_import @ obj.matrix_world.copy()
        obj.data=obj.data.copy()
        obj.data.transform(transform)
        obj.parent=None
        obj.matrix_world=Matrix.Identity(4)
        obj.parent=root
        points.append(vertices(obj));parts.append(obj)
    for obj in imported-set(parts):bpy.data.objects.remove(obj,do_unlink=True)
    visual=np.concatenate(points)
    collision_path=episode/geometry['mesh']['path']
    checked(collision_path,geometry['mesh']['sha256'])
    with np.load(collision_path) as z:physical=z['vertices']
    error=max(max_nearest(visual,physical),max_nearest(physical,visual))
    tolerance=max(float(np.ptp(physical,axis=0).max())*1e-4,2e-6)
    if error>tolerance:
        raise ValueError(f'{oid}: visual/physics vertex error {error} > {tolerance}')
    return root,dict(entry=str(entry),entry_sha256=sha(entry),
        body_from_import_matrix=[list(r) for r in body_from_import],
        visual_bounds_m=[visual.min(0).tolist(),visual.max(0).tolist()],
        collision_bounds_m=[physical.min(0).tolist(),physical.max(0).tolist()],
        bidirectional_vertex_error_m=error,tolerance_m=tolerance,
        mesh_parts=len(parts),original_materials=[m.name for obj in parts for m in obj.data.materials])


def fabric_material():
    """Woven appearance on the native UV; no geometry displacement or thickening."""
    mat=bpy.data.materials.new('woven_cotton');mat.use_nodes=True
    n=mat.node_tree.nodes;l=mat.node_tree.links;p=n.get('Principled BSDF')
    p.inputs['Base Color'].default_value=(.12,.27,.40,1)
    p.inputs['Roughness'].default_value=.88
    if 'Sheen Weight' in p.inputs:p.inputs['Sheen Weight'].default_value=.3
    coord=n.new('ShaderNodeTexCoord')
    wave=n.new('ShaderNodeTexWave');wave.wave_type='BANDS';wave.bands_direction='X'
    wave.inputs['Scale'].default_value=110
    l.new(coord.outputs['UV'],wave.inputs['Vector'])
    bump=n.new('ShaderNodeBump');bump.inputs['Strength'].default_value=.2;bump.inputs['Distance'].default_value=.00025
    l.new(wave.outputs['Color'],bump.inputs['Height']);l.new(bump.outputs['Normal'],p.inputs['Normal'])
    return mat


def cloth_object(episode,row):
    path=episode/row['body_states']['cloth']['geometry']['path']
    checked(path,row['body_states']['cloth']['geometry']['sha256'])
    with np.load(path) as z:
        v=z['surface_world_m'];f=z['surface_triangles']
    mesh=bpy.data.meshes.new('native_cloth');mesh.from_pydata(v.tolist(),[],f.tolist());mesh.update()
    obj=bpy.data.objects.new('native_cloth',mesh);bpy.context.scene.collection.objects.link(obj)
    topology=read(episode/'topology.json')
    # Parameterization is the physical rest mesh, never a deforming world projection.
    uv=topology.get('rest_uv',topology.get('uv'))
    if uv is None:
        with np.load(episode/'native/cloth.npz') as z:uv=z['uv']
    uv=np.asarray(uv)
    layer=mesh.uv_layers.new(name='RestUV')
    for loop in mesh.loops:layer.data[loop.index].uv=uv[loop.vertex_index]
    mesh.materials.append(fabric_material())
    for poly in mesh.polygons:poly.use_smooth=True
    return obj


def rope_objects(episode):
    """Display each actual native collision capsule at its body-local transform."""
    with np.load(episode/'native/states.npz') as z:a={k:z[k] for k in z.files}
    mat=bpy.data.materials.new('braided_rope');mat.use_nodes=True
    p=mat.node_tree.nodes.get('Principled BSDF');p.inputs['Base Color'].default_value=(.50,.22,.065,1);p.inputs['Roughness'].default_value=.82
    objects={}
    for body in a['body_ids']:
        shape=int(np.flatnonzero(a['shape_body']==body)[0]);radius,half=a['shape_scale'][shape,:2]
        rings=[(radius*np.cos(t),-half+radius*np.sin(t)) for t in np.linspace(-np.pi/2,0,5)]
        rings += [(radius*np.cos(t),half+radius*np.sin(t)) for t in np.linspace(0,np.pi/2,5)]
        n=16;v=np.asarray([[r*np.cos(t),r*np.sin(t),z] for r,z in rings for t in np.arange(n)*2*np.pi/n]);faces=[]
        for j in range(len(rings)-1):
            for i in range(n):
                aa=j*n+i;bb=j*n+(i+1)%n;faces.extend([(aa,bb,bb+n),(aa,bb+n,aa+n)])
        mesh=bpy.data.meshes.new('native_capsule');mesh.from_pydata(v.tolist(),[],faces)
        transform=a['shape_transform'][shape];q=transform[3:];rotation=Quaternion((q[3],*q[:3])).to_matrix().to_4x4();rotation.translation=Vector(transform[:3])
        mesh.transform(rotation)
        obj=bpy.data.objects.new(f'segment_{body}',mesh);bpy.context.scene.collection.objects.link(obj);obj.rotation_mode='QUATERNION'
        mesh.materials.append(mat)
        for poly in mesh.polygons:poly.use_smooth=True
        objects[obj.name]=obj
    return objects


def surface_weights(rest_particles,rest_surface,k=24):
    """Local affine MLS, derived display mapping; not a native MPM surface."""
    tree=KDTree(len(rest_particles))
    for i,point in enumerate(rest_particles):tree.insert(point,i)
    tree.balance();indices=[];weights=[]
    for v in rest_surface:
        near=tree.find_n(v,min(k,len(rest_particles)));ids=np.array([p[1] for p in near])
        delta=rest_particles[ids]-v;h=max(np.linalg.norm(delta,axis=1).max(),1e-9)
        matrix=np.c_[np.ones(len(ids)),delta/h]
        w=1/(np.sum((delta/h)**2,axis=1)+.05)
        coeff=np.linalg.pinv(matrix.T@(w[:,None]*matrix),rcond=1e-10)[:,0]
        weights.append(w*(matrix@coeff));indices.append(ids)
    indices=np.asarray(indices);weights=np.asarray(weights)
    reconstructed=np.einsum('nk,nkj->nj',weights,rest_particles[indices])
    error=float(np.linalg.norm(reconstructed-rest_surface,axis=1).max())
    if error>2e-6:raise ValueError('Rest material-coordinate mapping failed: '+str(error))
    return indices,weights,error


def plastic_objects(episode,appearance):
    package=Path(appearance['asset_package']);metadata=read(package/'asset.json')
    geometry=dict(physical_asset_provenance=dict(source_package=str(package),asset_manifest_sha256=sha(package/'asset.json'),
        parameters=dict(max_extent_m=appearance['size_m'])),mesh=dict(path=appearance['source_local_mesh'],sha256=sha(appearance['source_local_mesh'])))
    root,alignment=import_asset('plastic_asset',geometry,episode)
    with np.load(episode/'native/states.npz') as z:rest=z['x'][0]
    q=appearance.get('rotation_xyzw',[0,0,0,1]);rot=np.asarray(Quaternion((q[3],*q[:3])).to_matrix());position=np.asarray(appearance['position_m'])
    drivers=[]
    for obj in root.children:
        points=vertices(obj)@rot.T+position
        indices,weights,error=surface_weights(rest,points)
        drivers.append((obj,indices,weights))
    alignment.update(display_mapping='source UV surface, local affine MLS from 24 nearest rest material points; derived, not native topology',
        max_rest_reconstruction_error_m=max(surface_weights(rest,vertices(o)@rot.T+position)[2] for o in root.children))
    return drivers,alignment


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plan',type=Path,required=True)
    p.add_argument('--ids',nargs='+')
    p.add_argument('--keyframes',action='store_true')
    a=p.parse_args(sys.argv[sys.argv.index('--')+1:]);plan=read(a.plan)
    source=read(plan['scene_manifest']);blend=Path(source['source_blend'])
    checked(blend,source['source_sha256'])
    bpy.ops.wm.open_mainfile(filepath=str(blend),load_ui=False,use_scripts=False)
    scene=bpy.context.scene
    if source['metres_per_unit']!=1:raise ValueError('Only original metre scenes are currently supported')
    scene.frame_set(source['frame'])
    scene.render.engine=plan.get('engine','CYCLES')
    if scene.render.engine=='CYCLES':
        scene.cycles.samples=plan.get('samples',8);scene.cycles.use_denoising=True
        if plan.get('device')=='CUDA':
            preferences=bpy.context.preferences.addons['cycles'].preferences
            preferences.compute_device_type='CUDA';preferences.get_devices()
            cuda=[d for d in preferences.devices if d.type=='CUDA']
            if not cuda:raise RuntimeError('Requested CUDA renderer is unavailable')
            for device in preferences.devices:device.use=device.type=='CUDA'
            scene.cycles.device='GPU'
    scene.render.resolution_x,scene.render.resolution_y=plan.get('resolution',[640,400])
    scene.render.resolution_percentage=100
    scene.render.use_persistent_data=True
    scene.render.image_settings.file_format='PNG';scene.render.image_settings.color_mode='RGB'
    scene.render.film_transparent=False;scene.render.use_compositing=False;scene.render.use_sequencer=False
    original_objects=set(bpy.data.objects)
    for shot in plan['shots']:
        if a.ids and shot['id'] not in a.ids:continue
        scene.render.resolution_x,scene.render.resolution_y=shot.get('resolution',plan.get('resolution',[640,400]))
        for obj in set(bpy.data.objects)-original_objects:bpy.data.objects.remove(obj,do_unlink=True)
        ep=Path(shot['episode']);kind=shot.get('kind','rigid')
        state_path=ep/('body_state_trace.jsonl' if kind=='rigid' else 'states.jsonl')
        rows=[json.loads(line) for line in state_path.read_text().splitlines()]
        resolved=read(ep/'resolved_inputs.json')
        camera=shot.get('camera') or resolved['camera_set']['cameras'][0]
        camdata=bpy.data.cameras.new('cache_camera');cam=bpy.data.objects.new('cache_camera',camdata)
        scene.collection.objects.link(cam);scene.camera=cam
        cam.location=camera['position_m'];cam.rotation_euler=(Vector(camera['target_m'])-cam.location).to_track_quat('-Z','Y').to_euler()
        camdata.type=camera.get('projection','ORTHO');camdata.ortho_scale=camera.get('ortho_scale_m',1.0)
        camdata.lens=camera.get('focal_length_mm',45.);camdata.clip_start=.01;camdata.clip_end=100
        objects={};alignment={};drivers=[]
        if kind=='rigid':
            for oid,body in resolved['bodies'].items():
                g=body['geometry']
                if 'static_scene_source' in g:
                    if g['static_scene_source']['blend_sha256']!=source['source_sha256']:raise ValueError('Different collision scene')
                    continue
                objects[oid],alignment[oid]=import_asset(oid,g,ep)
        elif kind=='cloth':objects['cloth']=cloth_object(ep,rows[0])
        elif kind=='rope':objects=rope_objects(ep)
        elif kind=='plastic':drivers,alignment['plastic_asset']=plastic_objects(ep,shot['appearance'])
        else:raise ValueError('Unsupported cache adapter: '+kind)
        out=Path(shot['output']);out.mkdir(parents=True,exist_ok=True)
        fps=shot.get('fps',plan.get('fps',12))
        indices=sorted(set(np.argmin(abs(np.asarray([r['time_s'] for r in rows])-t)).item()
            for t in np.arange(0,rows[-1]['time_s']+1e-8,1/fps)))
        if a.keyframes:
            times=np.asarray([r['time_s'] for r in rows])
            indices=[int(np.argmin(abs(times-t))) for t in shot.get('key_times_s',[0,.15,.5,rows[-1]['time_s']])]
        records=[];previous_signature=None;previous_path=None
        for frame,index in enumerate(indices):
            row=rows[index];signature=hashlib.sha256()
            for oid,obj in objects.items():
                s=row['body_states'][oid]
                if kind in ('rigid','rope'):
                    obj.location=s['position_m'];q=s['orientation_xyzw'];obj.rotation_quaternion=Quaternion((q[3],*q[:3]))
                    signature.update(np.asarray([*s['position_m'],*q],dtype=np.float64).tobytes())
                else:
                    path=checked(ep/s['geometry']['path'],s['geometry']['sha256'])
                    with np.load(path) as z:
                        co=z['surface_world_m'].astype(float);obj.data.vertices.foreach_set('co',co.ravel());signature.update(co.tobytes())
                    obj.data.update()
            if kind=='plastic':
                state=row['body_states']['mpm_block'];path=checked(ep/state['geometry']['path'],state['geometry']['sha256'])
                with np.load(path) as z:points=z['particle_world_m']
                for obj,indices,weights in drivers:
                    v=np.einsum('nk,nkj->nj',weights,points[indices])
                    if not np.isfinite(v).all():raise ValueError('Nonfinite derived display surface')
                    obj.data.vertices.foreach_set('co',v.ravel());obj.data.update()
                    signature.update(v.tobytes())
            scene.render.filepath=str(out/f'frame_{frame:04d}.png')
            identical=signature.hexdigest()==previous_signature
            if identical:shutil.copyfile(previous_path,scene.render.filepath)
            else:bpy.ops.render.render(write_still=True)
            records.append(dict(frame=frame,state_index=index,time_s=row['time_s'],path=Path(scene.render.filepath).name,identical_geometry_reused=identical))
            previous_signature=signature.hexdigest();previous_path=scene.render.filepath
            print('REPLAY_FRAME',shot['id'],frame,row['time_s'],flush=True)
        report=dict(id=shot['id'],source_blend=str(blend),source_sha256=sha(blend),
            state_path=str(state_path),state_sha256=sha(state_path),physics_rerun=False,
            appearance_alignment=alignment,camera=camera,source_lighting_preserved=True,
            engine=scene.render.engine,device=plan.get('device','CPU'),resolution=[scene.render.resolution_x,scene.render.resolution_y],
            fps=fps,keyframes_only=a.keyframes,frames=records)
        (out/'replay.json').write_text(json.dumps(report,indent=2))
    checked(blend,source['source_sha256'])


if __name__=='__main__':main()

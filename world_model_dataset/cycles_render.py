"""Blender-side selected-frame renderer for accepted fixed-topology episodes."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

import bpy
import numpy as np
from mathutils import Vector


def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))


def look(camera,eye,target):
    camera.location=eye
    camera.rotation_euler=(Vector(target)-Vector(eye)).to_track_quat('-Z','Y').to_euler()


def material(name,color,roughness=.4,metallic=0.):
    value=bpy.data.materials.new(name);value.diffuse_color=(*color,1.);value.use_nodes=True
    nodes=value.node_tree.nodes;nodes.clear();node=nodes.new('ShaderNodeBsdfPrincipled');output=nodes.new('ShaderNodeOutputMaterial')
    value.node_tree.links.new(node.outputs['BSDF'],output.inputs['Surface']);node.inputs['Base Color'].default_value=(*color,1.)
    node.inputs['Roughness'].default_value=roughness;node.inputs['Metallic'].default_value=metallic
    return value


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--episode',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--frames',default='auto');parser.add_argument('--samples',type=int,default=32)
    argv=sys.argv[sys.argv.index('--')+1:] if '--' in sys.argv else []
    args=parser.parse_args(argv);episode=args.episode;output=args.output
    if output.exists():raise FileExistsError(output)
    output.mkdir(parents=True)
    manifest=read(episode/'episode.json');validation=read(episode/'validation.json')
    if not validation['passed']:raise ValueError('Cycles input episode is not accepted')
    state=read(episode/'state/index.json');fixture=read(episode/'fixture.json');camera_cfg=manifest['inputs']['cameras']
    if args.frames=='auto':
        count=len(state['frames']);indices=sorted(set((0,count//3,2*count//3,count-1)))
    else:indices=sorted(set(int(x) for x in args.frames.split(',')))
    if not indices or indices[0]<0 or indices[-1]>=len(state['frames']):raise ValueError('Frame selection outside cache')

    bpy.ops.wm.read_factory_settings(use_empty=True);scene=bpy.context.scene
    scene.render.engine='BLENDER_EEVEE'  # Initialize robustly before Cycles device discovery.
    resolved_device='CPU'
    try:
        preferences=bpy.context.preferences.addons['cycles'].preferences;preferences.compute_device_type='OPTIX';preferences.get_devices()
        for device in preferences.devices:
            device.use=device.type!='CPU'
            if device.use:resolved_device=device.name
        scene.render.engine='CYCLES';scene.cycles.device='GPU';scene.cycles.samples=args.samples
    except Exception:
        scene.render.engine='CYCLES';scene.cycles.device='CPU';scene.cycles.samples=args.samples
    width,height=camera_cfg['resolution'];scene.render.resolution_x=width;scene.render.resolution_y=height;scene.render.resolution_percentage=100
    scene.render.image_settings.file_format='PNG';scene.render.film_transparent=False
    scene.world=bpy.data.worlds.new('DatasetWorld');scene.world.color=manifest['inputs']['environment']['background_color']
    gray=material('Fixture',[.42,.45,.48],.65)
    boxes={}
    for box in fixture['boxes']:
        bpy.ops.mesh.primitive_cube_add(size=1,location=box['position_m']);obj=bpy.context.object;obj.name=box['id'];obj.dimensions=box['size_m']
        obj.rotation_mode='QUATERNION';q=box['orientation_xyzw'];obj.rotation_quaternion=(q[3],q[0],q[1],q[2]);obj.data.materials.append(gray);boxes[box['id']]=obj
    oid=manifest['spec']['objects'][0]['instance_id'];first=np.load(episode/state['frames'][0]['geometry'])
    mesh=bpy.data.meshes.new(oid);mesh.from_pydata(first['surface_world_m'].tolist(),[],first['surface_triangles'].tolist());mesh.update()
    subject=bpy.data.objects.new(oid,mesh);bpy.context.collection.objects.link(subject)
    appearance=manifest['inputs']['objects'][0]['appearance'];subject.data.materials.append(material('Subject',appearance['color'],appearance['roughness'],appearance['metallic']))
    bpy.ops.object.light_add(type='AREA',location=(2,-2,4));bpy.context.object.data.energy=900;bpy.context.object.data.shape='DISK';bpy.context.object.data.size=4
    bpy.ops.object.light_add(type='SUN',location=(0,0,4));bpy.context.object.rotation_euler=(math.radians(25),math.radians(-20),math.radians(-25));bpy.context.object.data.energy=2.5
    d=fixture['D_m'];scale=d*(10. if manifest['spec']['event_id']=='R01' else 1.);offset=np.array([25*d,0,0]) if manifest['spec']['event_id']=='R01' else np.zeros(3)
    cameras={}
    for cfg in camera_cfg['cameras']:
        data=bpy.data.cameras.new(cfg['id']);cam=bpy.data.objects.new(cfg['id'],data);bpy.context.collection.objects.link(cam)
        data.lens=camera_cfg['focal_length_mm'];data.sensor_width=camera_cfg['horizontal_aperture_mm']
        look(cam,np.array(cfg['position_D'])*scale+offset,np.array(cfg['target_D'])*scale+offset);cameras[cfg['id']]=cam;(output/cfg['id']).mkdir()
    rendered=[]
    for index in indices:
        frame=state['frames'][index];geometry=np.load(episode/frame['geometry'])
        mesh.vertices.foreach_set('co',geometry['surface_world_m'].astype(np.float64).ravel());mesh.update()
        for name,position in frame['fixture_positions'].items():boxes[name].location=position
        if 'gate' in boxes:boxes['gate'].hide_render=frame['time_s']>manifest['spec']['action_parameters']['release_time_s']
        for name,camera in cameras.items():
            scene.camera=camera;relative=f'{name}/frame_{index:04d}.png';scene.render.filepath=str(output/relative);bpy.ops.render.render(write_still=True)
            rendered.append(dict(camera_id=name,frame_index=index,time_s=frame['time_s'],path=relative))
    source_hash=hashlib.sha256((episode/'episode.json').read_bytes()).hexdigest()
    report=dict(schema_version='0.1.0',valid=len(rendered)==len(indices)*len(cameras),source_episode_id=manifest['episode_id'],
        source_episode_manifest_sha256=source_hash,physics_rerun=False,renderer='Blender Cycles',samples=args.samples,
        requested_device='OPTIX',resolved_device=resolved_device,selected_frames=indices,renders=rendered)
    with (output/'report.json').open('x',encoding='utf-8') as stream:json.dump(report,stream,indent=2);stream.write('\n')
    print('DATASET_CYCLES_COMPLETE '+str(output),flush=True)


if __name__=='__main__':main()

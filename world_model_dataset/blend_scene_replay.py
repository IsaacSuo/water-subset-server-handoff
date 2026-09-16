"""Cache-only replay in the original scene; no source save or physics simulation."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import shutil

import bpy
from mathutils import Vector


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--blend',type=Path,required=True)
    p.add_argument('--episode',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--steps',type=int,nargs='+')
    p.add_argument('--engine',choices=['eevee','workbench'],default='eevee')
    a=p.parse_args(sys.argv[sys.argv.index('--')+1:])
    state_path=a.episode/'body_state_trace.jsonl'
    rows=[json.loads(line) for line in state_path.read_text().splitlines()]
    resolved=read(a.episode/'resolved_inputs.json')
    source=resolved['bodies']['support']['geometry']['static_scene_source']
    if digest(a.blend)!=source['blend_sha256']:
        raise ValueError('Source blend mismatch')
    bpy.ops.wm.open_mainfile(filepath=str(a.blend),load_ui=False,use_scripts=False)
    scene=bpy.context.scene
    scene.render.engine='BLENDER_EEVEE' if a.engine=='eevee' else 'BLENDER_WORKBENCH'
    scene.render.resolution_x,scene.render.resolution_y=960,640
    scene.render.resolution_percentage=100
    scene.render.image_settings.file_format='PNG'
    scene.render.image_settings.color_mode='RGB'
    scene.render.film_transparent=False
    scene.render.use_compositing=False
    scene.render.use_sequencer=False
    # The legacy scene has a 64-strength world shader. Expose the Eevee preview
    # for readable geometry, without modifying its materials/lights or geometry.
    scene.view_settings.view_transform='AgX'
    scene.view_settings.look='AgX - Medium High Contrast'
    scene.view_settings.exposure=-3.0
    if hasattr(scene,'eevee'):
        scene.eevee.taa_render_samples=16
        scene.eevee.use_raytracing=False
    scene.display.shading.light='STUDIO'
    scene.display.shading.color_type='MATERIAL'
    scene.display.shading.show_shadows=True
    scene.display.shading.show_cavity=True
    # Independent camera; existing scene objects, materials and lights stay intact.
    camdata=bpy.data.cameras.new('episode_camera')
    cam=bpy.data.objects.new('episode_camera',camdata)
    scene.collection.objects.link(cam)
    camera=resolved['camera_set']['cameras'][0]
    cam.location=camera['position_m']
    cam.rotation_euler=(Vector(camera['target_m'])-cam.location).to_track_quat('-Z','Y').to_euler()
    camdata.type='ORTHO'
    camdata.ortho_scale=1.85
    camdata.clip_start=.01
    camdata.clip_end=100
    scene.camera=cam
    subjects={}
    for oid,body in resolved['bodies'].items():
        if 'static_scene_source' in body['geometry']:
            continue
        g=body['geometry']
        if g['shape']!='box':
            raise ValueError('First in-situ replay supports box subjects')
        bpy.ops.mesh.primitive_cube_add(size=1.)
        obj=bpy.context.object
        obj.name='episode_'+oid
        obj.dimensions=g['size_m']
        obj.rotation_mode='QUATERNION'
        mat=bpy.data.materials.new('episode_'+oid)
        color=body['appearance']['color']
        mat.diffuse_color=(*color,1)
        mat.use_nodes=True
        principled=mat.node_tree.nodes.get('Principled BSDF')
        principled.inputs['Base Color'].default_value=(*color,1)
        principled.inputs['Roughness'].default_value=body['appearance']['roughness']
        obj.data.materials.append(mat)
        subjects[oid]=obj
    a.output.mkdir(parents=True,exist_ok=True)
    steps=a.steps if a.steps is not None else list(range(0,len(rows),8))
    previous_state = None
    previous_path = None
    for step in steps:
        row=rows[step]
        for oid,obj in subjects.items():
            s=row['body_states'][oid]
            obj.location=s['position_m']
            q=s['orientation_xyzw']
            obj.rotation_quaternion=(q[3],*q[:3])
        scene.render.filepath=str(a.output/f'step_{step:04d}.png')
        signature=[row['body_states'][oid] for oid in subjects]
        if signature == previous_state:
            shutil.copyfile(previous_path,scene.render.filepath)
        else:
            bpy.ops.render.render(write_still=True)
        previous_state=signature
        previous_path=scene.render.filepath
        print('CACHE_REPLAY',step,row['time_s'],flush=True)
    (a.output/'replay.json').write_text(json.dumps(dict(source_blend=str(a.blend),
        source_sha256=digest(a.blend),state_sha256=digest(state_path),physics_rerun=False,
        engine=scene.render.engine,resolution=[960,640],camera=camera,ortho_scale_m=camdata.ortho_scale,
        view_transform=scene.view_settings.view_transform,exposure=scene.view_settings.exposure,
        samples=[dict(physics_step=i,time_s=rows[i]['time_s'],path=f'step_{i:04d}.png') for i in steps]),indent=2))


if __name__=='__main__':main()

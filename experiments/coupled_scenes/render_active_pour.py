"""Render accepted v3 design using native water and recorded pitcher transforms."""
import argparse
import hashlib
import json
import sys
from pathlib import Path
import bpy
import numpy as np
from mathutils import Vector,Quaternion

ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from experiments.coupled_scenes.blender_coupled_event_overlay import _read_obj,_water_material


def spray_points_node_group(material):
    """Geometry nodes: render spray vertices as water spheres of one radius."""
    tree=bpy.data.node_groups.new('SprayDroplets','GeometryNodeTree')
    tree.interface.new_socket(name='Geometry',in_out='INPUT',socket_type='NodeSocketGeometry')
    tree.interface.new_socket(name='Geometry',in_out='OUTPUT',socket_type='NodeSocketGeometry')
    source=tree.nodes.new('NodeGroupInput');target=tree.nodes.new('NodeGroupOutput')
    points=tree.nodes.new('GeometryNodeMeshToPoints');points.name='DropletPoints'
    assign=tree.nodes.new('GeometryNodeSetMaterial');assign.inputs['Material'].default_value=material
    tree.links.new(source.outputs['Geometry'],points.inputs['Mesh'])
    tree.links.new(points.outputs['Points'],assign.inputs['Geometry'])
    tree.links.new(assign.outputs['Geometry'],target.inputs['Geometry'])
    return tree


def spray_mesh(path,name):
    """Droplet centres from the reconstruction, converted like the water mesh."""
    with np.load(path) as data:
        centres=np.asarray(data['droplet_positions'],dtype=np.float64).reshape(-1,3)
        radius=float(data['droplet_radius'])
    mesh=bpy.data.meshes.new(name)
    mesh.from_pydata(np.column_stack([centres[:,0],-centres[:,2],centres[:,1]]).tolist(),[],[])
    return mesh,radius,len(centres)
from experiments.coupled_scenes.blender_server_assets import reload_hdri
from experiments.coupled_scenes.run_active_pour_probe import atomic_json


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--blend',type=Path,required=True)
    parser.add_argument('--surfaces',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--hdri',type=Path,required=True)
    args=parser.parse_args(sys.argv[sys.argv.index('--')+1:])
    args.blend=args.blend.resolve();args.surfaces=args.surfaces.resolve();args.output=args.output.resolve()
    sequence=json.loads((args.surfaces/'sequence.json').read_text(encoding='utf-8'));assert sequence['complete']
    if sequence.get('source_blend_sha256'):
        assert hashlib.sha256(args.blend.read_bytes()).hexdigest()==sequence['source_blend_sha256'],'Mismatched pitcher layout'
    args.output.mkdir(parents=True,exist_ok=False)
    bpy.ops.wm.open_mainfile(filepath=str(args.blend));reload_hdri(args.hdri);scene=bpy.context.scene
    scene.camera=bpy.data.objects['DesignView_00'];scene.render.fps=30
    scene.render.resolution_x=1280;scene.render.resolution_y=960;scene.render.resolution_percentage=100
    scene.cycles.samples=64;scene.cycles.seed=0;scene.cycles.use_animated_seed=False;scene.render.use_motion_blur=False
    prefs=bpy.context.preferences.addons['cycles'].preferences;prefs.compute_device_type='OPTIX';prefs.get_devices()
    for device in prefs.devices:device.use=device.type!='CPU'
    scene.cycles.device='GPU'
    donor=bpy.data.objects[sequence.get('moving_object','PouringPitcher')];donor.animation_data_clear();donor.rotation_mode='QUATERNION'
    parent_inverse=donor.parent.matrix_world.inverted()
    water=bpy.data.objects.new('NativeWater',bpy.data.meshes.new('EmptyWater'));scene.collection.objects.link(water)
    material=_water_material();rendered=[]
    spray=None
    if any('spray' in frame for frame in sequence['frames']):
        spray=bpy.data.objects.new('NativeSpray',bpy.data.meshes.new('EmptySpray'));scene.collection.objects.link(spray)
        droplets=spray_points_node_group(material)
        spray.modifiers.new('Droplets','NODES').node_group=droplets
    for index,frame in enumerate(sequence['frames']):
        scene.frame_set(index+1)
        x,y,z=frame['native_position_m'];donor.location=parent_inverse@Vector((x,-z,y))
        qx,qy,qz,qw=frame['native_rotation_xyzw'];donor.rotation_quaternion=Quaternion((qw,qx,-qz,qy))
        bpy.context.view_layer.update()
        vertices,faces=_read_obj(args.surfaces/frame['surface'])
        mesh=bpy.data.meshes.new(f'Water_{index:04d}');mesh.from_pydata(vertices,[],faces);mesh.update()
        mesh.materials.append(material)
        for polygon in mesh.polygons:polygon.use_smooth=True
        old=water.data;water.data=mesh
        if old.users==0:bpy.data.meshes.remove(old)
        del vertices,faces
        droplet_count=0
        if spray is not None:
            old=spray.data
            if 'spray' in frame:
                spray.data,radius,droplet_count=spray_mesh(args.surfaces/frame['spray'],f'Spray_{index:04d}')
                droplets.nodes['DropletPoints'].inputs['Radius'].default_value=radius
            else:
                spray.data=bpy.data.meshes.new(f'Spray_{index:04d}')
            if old.users==0:bpy.data.meshes.remove(old)
        filename=f'frame_{index:04d}.png';scene.render.filepath=str(args.output/filename);bpy.ops.render.render(write_still=True)
        rendered.append(dict(file=filename,action_seconds=frame['action_seconds'],spray_droplets=droplet_count))
        atomic_json(args.output/'render_manifest.json',dict(complete=len(rendered)==len(sequence['frames']),frames=rendered,fps=30,
            diagnostic_only=sequence.get('diagnostic_only',False)))
        print(f'RENDER {index+1}/{len(sequence["frames"])}',flush=True)


if __name__=='__main__':main()

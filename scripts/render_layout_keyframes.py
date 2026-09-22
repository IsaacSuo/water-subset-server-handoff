"""Three cache-only frames in unchanged original Blender scene/materials."""
import argparse,json,sys
from pathlib import Path
import numpy as np
import bpy
from mathutils import Vector,Quaternion
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'world_model_dataset'))
import scene_cache_replay as replay


def material(name,color,metallic=0.):
    m=bpy.data.materials.new(name);m.use_nodes=True;s=m.node_tree.nodes.get('Principled BSDF')
    s.inputs['Base Color'].default_value=(*color,1);s.inputs['Roughness'].default_value=.55;s.inputs['Metallic'].default_value=metallic
    return m


def box_actor(oid,size):
    bpy.ops.mesh.primitive_cube_add(size=1);o=bpy.context.object;o.name=oid
    for v in o.data.vertices:v.co*=Vector(size)
    o.data.materials.append(material(oid,(.35,.38,.42),.65));o.rotation_mode='QUATERNION';return o


def main():
    p=argparse.ArgumentParser();p.add_argument('--folder',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--times',nargs='+',type=float);a=p.parse_args(sys.argv[sys.argv.index('--')+1:])
    doc=replay.read(a.folder/'generated/experiment.json');job=replay.read(a.folder/'execution/workflow.json')['jobs']['baseline']
    ep=Path(job['stages']['package']['result']['episode']);kind=doc['backend']['kind']
    source=replay.read(doc['scene']['source_records']['scene']);blend=Path(source['source_blend']);replay.checked(blend,source['source_sha256'])
    bpy.ops.wm.open_mainfile(filepath=str(blend),load_ui=False,use_scripts=False);scene=bpy.context.scene;scene.frame_set(source['frame'])
    scene.render.engine='CYCLES';scene.cycles.samples=4;scene.cycles.use_denoising=True
    pref=bpy.context.preferences.addons['cycles'].preferences;pref.compute_device_type='CUDA';pref.get_devices()
    for device in pref.devices:device.use=device.type=='CUDA'
    scene.cycles.device='GPU';scene.render.resolution_x=480;scene.render.resolution_y=320;scene.render.resolution_percentage=100
    scene.render.use_compositing=False;scene.render.use_sequencer=False;scene.render.image_settings.file_format='PNG'
    camera=doc['observations']['camera'];cd=bpy.data.cameras.new('cache_camera');cam=bpy.data.objects.new('cache_camera',cd);scene.collection.objects.link(cam)
    cam.location=camera['position_m'];cam.rotation_euler=(Vector(camera['target_m'])-cam.location).to_track_quat('-Z','Y').to_euler()
    cd.type='ORTHO';cd.ortho_scale=camera['ortho_scale_m'];scene.camera=cam
    state_path=ep/('body_state_trace.jsonl' if kind in ('rigid','beam') else 'states.jsonl')
    rows=[json.loads(l) for l in state_path.read_text().splitlines()];resolved=replay.read(ep/'resolved_inputs.json')
    objects={};alignment={};cfg=doc['input'];drivers=[]
    if kind=='rigid':
        for oid,b in resolved['bodies'].items():
            if 'static_scene_source' in b['geometry']:continue
            o,alignment[oid]=replay.import_asset(oid,b['geometry'],ep);objects[oid]=(o,'rigid')
    elif kind=='cloth':
        objects['cloth']=(replay.cloth_object(ep,rows[0]),'surface')
        for key,oid in [('gripper','Gripper'),('opposing_fixture','OpposingFixture')]:
            if key in cfg:objects[oid]=(box_actor(oid,cfg[key]['size_m']),'rigid')
    elif kind=='rope':
        with np.load(ep/'native/states.npz') as z:native={k:z[k] for k in z.files}
        for body in native.get('segment_body_ids',native['body_ids']):
            shape=int(np.flatnonzero(native['shape_body']==body)[0]);radius,half=native['shape_scale'][shape,:2]
            rings=[(radius*np.cos(t),-half+radius*np.sin(t)) for t in np.linspace(-np.pi/2,0,5)]+[(radius*np.cos(t),half+radius*np.sin(t)) for t in np.linspace(0,np.pi/2,5)]
            n=12;v=[[r*np.cos(t),r*np.sin(t),z] for r,z in rings for t in np.arange(n)*2*np.pi/n];f=[]
            for j in range(len(rings)-1):
                for i in range(n):aa=j*n+i;bb=j*n+(i+1)%n;f.extend([(aa,bb,bb+n),(aa,bb+n,aa+n)])
            mesh=bpy.data.meshes.new('native_capsule');mesh.from_pydata(v,[],f)
            t=native['shape_transform'][shape];q=t[3:];matrix=Quaternion((q[3],*q[:3])).to_matrix().to_4x4();matrix.translation=Vector(t[:3]);mesh.transform(matrix)
            oid='segment_'+str(body);o=bpy.data.objects.new(oid,mesh);scene.collection.objects.link(o);o.rotation_mode='QUATERNION'
            mesh.materials.append(material('rope',(.5,.22,.065)));objects[oid]=(o,'rigid')
        for load in cfg.get('loads',[]):objects[load['id']]=(box_actor(load['id'],load['size_m']),'rigid')
    elif kind=='beam':
        for oid in ('Fixture','Plate'):objects[oid]=(box_actor(oid,cfg[oid.lower()]['size_m']),'rigid')
        with np.load(replay.checked(ep/rows[0]['body_states']['Beam']['geometry']['path'],rows[0]['body_states']['Beam']['geometry']['sha256'])) as z:
            points=z['simulation_world_m'];tet=z['simulation_tets']
        faces=np.concatenate([tet[:,slots] for slots in ((0,1,2),(0,2,3),(0,3,1),(1,3,2))]);_,idx,count=np.unique(np.sort(faces,axis=1),axis=0,return_index=True,return_counts=True);faces=faces[idx[count==1]]
        mesh=bpy.data.meshes.new('native_beam');mesh.from_pydata(points.tolist(),[],faces.tolist());o=bpy.data.objects.new('Beam',mesh);scene.collection.objects.link(o)
        mesh.materials.append(material('beam',(.47,.24,.09)));objects['Beam']=(o,'simulation')
    elif kind=='plastic':
        # Declared synthetic box: map its original surface using existing MLS;
        # this is a display surface, never claimed as native material topology.
        with np.load(ep/'native/states.npz') as z:rest=z['x'][0]
        bpy.ops.mesh.primitive_cube_add(size=1);o=bpy.context.object;o.name='plastic_display';size=cfg['object']['size_m'];tf=np.asarray(cfg['object']['world_from_mesh'])
        bpy.context.view_layer.objects.active=o;bpy.ops.object.mode_set(mode='EDIT');bpy.ops.mesh.subdivide(number_cuts=5);bpy.ops.object.mode_set(mode='OBJECT')
        local=replay.vertices(o)*size;world=(np.c_[local,np.ones(len(local))]@tf.T)[:,:3]
        ids,weights,error=replay.surface_weights(rest,world);drivers=[(o,ids,weights)]
        o.data.materials.append(material('plastic_material',(.48,.26,.12)));alignment['display_surface']=dict(method='original box surface mapped by local affine MLS',rest_error_m=error,native_topology=False)
    else:raise ValueError(kind)
    a.output.mkdir(parents=True,exist_ok=False);frames=[];times=np.array([r['time_s'] for r in rows])
    for frame,t in enumerate(a.times or [0,float(times[-1])*.4,float(times[-1])]):
        i=int(abs(times-t).argmin());row=rows[i]
        for oid,(o,mode) in objects.items():
            s=row['body_states'][oid]
            if mode=='rigid':o.location=s['position_m'];q=s['orientation_xyzw'];o.rotation_quaternion=Quaternion((q[3],*q[:3]))
            else:
                with np.load(replay.checked(ep/s['geometry']['path'],s['geometry']['sha256'])) as z:v=z['surface_world_m' if mode=='surface' else 'simulation_world_m']
                o.data.vertices.foreach_set('co',v.astype(float).ravel());o.data.update()
        for o,ids,weights in drivers:
            g=row['body_states']['mpm_block']['geometry']
            with np.load(replay.checked(ep/g['path'],g['sha256'])) as z:v=z['particle_world_m']
            o.data.vertices.foreach_set('co',np.einsum('nk,nkj->nj',weights,v[ids]).ravel());o.data.update()
        scene.render.filepath=str(a.output/f'frame_{frame:04d}.png');bpy.ops.render.render(write_still=True)
        frames.append(dict(time_s=row['time_s'],state_index=i,path=scene.render.filepath))
    (a.output/'replay.json').write_text(json.dumps(dict(source_scene=source,source_states_sha256=replay.sha(state_path),camera=camera,
        original_scene_materials_and_lighting_preserved=True,appearance_alignment=alignment,physics_rerun=False,frames=frames),indent=2))
    replay.checked(blend,source['source_sha256'])

if __name__=='__main__':main()

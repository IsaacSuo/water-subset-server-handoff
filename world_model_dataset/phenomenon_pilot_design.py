"""Prepare a bounded, explicit set of original-scene condition comparisons."""
import copy
from pathlib import Path

from .causal_runner import ROOT, load_config
from .io import read_json, write_json, file_hash
from .phenomenon_recipes02 import add_body


def main():
    folder=ROOT/'configs/dataset/v0_2/phenomenon_pilot_v1';folder.mkdir(exist_ok=False)
    showcase=ROOT/'output/world_model_dataset/v0_2/real_scene_showcase_v2'
    material=Path('/home/fangsuo/isaacsim_work_material_response/experiments/material_response/scene_input')
    spec=read_json(ROOT/'configs/dataset/v0_2/real_scene_showcase_v2.json')
    plan=dict(version='phenomenon-batch/1',purpose='four-process condition pilot, no training or split',
              material_entry=str(material/'material_entry.py'),material_runtime=str((folder/'runtime.json').relative_to(ROOT)),jobs=[])
    write_json(folder/'runtime.json',read_json(material/'examples/runtime.local.json'))
    def save(job_id,backend,family,group,value,config,camera,condition):
        path=folder/(job_id+'.json');write_json(path,config)
        plan['jobs'].append(dict(id=job_id,backend=backend,family=family,group=group,condition=value,
            config=str(path.relative_to(ROOT)),comparison_variable=condition,
            observation_camera=camera,observation_hz=10))
    for spin in (0,12):
        shot=copy.deepcopy(next(s for s in spec['shots'] if s['id']=='desk_roll_slide_v2'))
        shot['id']='roll_spin_'+str(spin);shot['duration_s']=2.
        shot['bodies']=[shot['bodies'][0]];shot['bodies'][0]['angular_velocity_rad_s']=[0,spin,0]
        cfg=dict(spec,shots=[shot]);save(shot['id'],'rigid','rolling_initial_spin',1,spin,cfg,shot['camera'],'onion initial Y angular velocity rad/s')
    for count in (3,4):
        shot=copy.deepcopy(next(s for s in spec['shots'] if s['id']=='desk_rearrangement_edge_v3'))
        shot['id']='rearrange_count_'+str(count);shot['duration_s']=2.
        shot['bodies']=shot['bodies'][:count]
        save(shot['id'],'rigid','multibody_count',1,count,dict(spec,shots=[shot]),shot['camera'],'presence of top onion; remaining bodies unchanged')
    for name in ('flat','upright'):
        shot=copy.deepcopy(next(s for s in spec['shots'] if s['id']=='chair_pair_'+name+'_v1'))
        shot['id']='passage_'+name
        save(shot['id'],'rigid','orientation_passage',1,name,dict(spec,shots=[shot]),shot['camera'],'initial orientation, with COM height derived from original floor support')
    def source_config(path):
        cfg=read_json(path);cfg['format']='material-scene/1'
        for key in ('diagnostic_camera','preview_fps','preview_slowdown','initial_geometry','appearance_request'):cfg.pop(key,None)
        for env in cfg['environment']:
            if 'geometry' in env:env['path']=str(path.parent/env.pop('geometry'))
            else:env['path']=str((path.parent/env['path']).resolve())
        if 'source_records' in cfg:cfg['source_records']={k:str((path.parent/v).resolve()) for k,v in cfg['source_records'].items()}
        return cfg
    camera=dict(position_m=[3.35,.65,1.4],target_m=[2.75,1.52,.83],ortho_scale_m=1.0)
    cloth=source_config(material/'examples/classroom_cloth_dense.json')
    cloth.update(duration_s=2.,kind='cloth')
    for x in (2.73,2.83):
        cfg=copy.deepcopy(cloth);cfg['cloth']['world_from_mesh'][0][3]=x
        save('cloth_edge_'+str(round(x*100)),'material','cloth_support_overlap',3,x,cfg,camera,'cloth initial X translation m')
    plastic=source_config(showcase/'material_runs/plastic_potato_v1/prepared/config.json');plastic['duration_s']=.8
    for height in (.04,.12):
        cfg=copy.deepcopy(plastic);cfg['object']['world_from_mesh'][2][3]+=(height-.12)
        save('plastic_drop_'+str(round(height*100)),'material','plastic_impact_height',2,height,cfg,camera,'initial lowest-point clearance m')
    rope=source_config(showcase/'material_runs/rope_G01_v1/prepared/config.json');rope['duration_s']=2.
    for end in ('free','first_clamped'):
        cfg=copy.deepcopy(rope);cfg['rope']['end_condition']=end
        save('rope_end_'+end,'material','rope_end_constraint',4,end,cfg,
             dict(position_m=[-2.085,-.833974596,2.65],target_m=[-1.585,-1.7,2.6],ortho_scale_m=.85),'free versus first whole segment fixed at initial pose')
    # Reuse native volume + finite impedance plate, adding the original table as
    # the physical support instead of placing a canonical rig in a backdrop.
    scene=read_json(showcase/'classroom_geometry/scene.json');table_z=.81563033
    for force in (50.,150.):
        jid='elastic_unload_force_'+str(int(force));c=load_config(ROOT/'configs/dataset/v0_2/c2_soft_compression.json')
        m=read_json(ROOT/c['manifest_template']);c['prototype_id']=jid
        c['timing'].update(duration_s=4.,capture_hz=10);m['timing']=copy.deepcopy(c['timing'])
        m['system']['bodies']=[b for b in m['system']['bodies'] if b['instance_id']!='floor']
        m['initial_state']['participant_ids']=['plate','soft'];m['initial_state']['body_states'].pop('floor')
        m['environment']['environment_body_ids']=[];m['environment']['environment_id']='original_teacher_desk'
        for oid,s in m['initial_state']['body_states'].items():s['position_m']=[2.65,1.52,s['position_m'][2]+table_z]
        for name,g in scene['groups'].items():
            path=showcase/'classroom_geometry'/g['path']
            source=dict(path=str(path),sha256=g['sha256'],scene_manifest_path=str(showcase/'classroom_geometry/scene.json'),
                        scene_manifest_sha256=file_hash(showcase/'classroom_geometry/scene.json'),blend_sha256=scene['source_sha256'],source_objects=g['objects'])
            add_body(c,m,name,dict(shape='mesh',static_scene_source=source),'fixture_reference',[0,0,0],kind='static',role='environment',appearance='matte_gray')
        c['camera_set']['cameras']=[dict(id='main',**camera)];c['camera_set']['resolution']=[160,120]
        controller=m['control_program']['controllers'][0];controller['max_force_n']=force
        commands=m['control_program']['commands']
        intervals=[(0,.5),(.5,1.25),(1.25,2.),(2.,2.5),(2.5,4.)]
        for command,(start,end) in zip(commands,intervals):
            command.update(start_time_s=start,end_time_s=end);command['limits']['max_force_n']=force
            for key in ('position_m','position_start_m','position_end_m'):
                if key in command['target']:command['target'][key]+=table_z
        m['counterfactual']=dict(family_id='elastic_force_limit',baseline_episode_id=None,changed_pointer=None)
        mpath=folder/(jid+'_manifest.json');write_json(mpath,m);c['manifest_template']=str(mpath)
        save(jid,'native','elastic_finite_load_hold_unload',2,force,c,camera,'actuator maximum force N; identical reference trajectory')
    write_json(folder/'batch.json',plan)
    print(folder/'batch.json',len(plan['jobs']),'jobs',flush=True)


if __name__=='__main__':main()

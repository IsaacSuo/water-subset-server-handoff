"""Write small semantic requests; never run prepare, physics or rendering."""
import copy
from pathlib import Path

from world_model_dataset.io import read_json, write_json

ROOT=Path(__file__).resolve().parents[1]
DEST=ROOT/'configs/dataset/v0_2/construction'


def main():
    import argparse, shutil
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(); dest=args.output
    dest.mkdir(parents=True,exist_ok=False)
    shutil.copytree(DEST/'profiles',dest/'profiles')
    for name in ('passage','cloth','roll','collision_chain','plastic','beam','rope'):
        old=read_json(ROOT/'configs/dataset/v0_2/experiment_api_v1'/f'{"multibody" if name=="collision_chain" else name}.json')
        if name=='collision_chain': old['phenomenon']='multibody_collision_propagation'
        scene=copy.deepcopy(old['scene'])
        if name in ('rope','plastic'):
            scene=copy.deepcopy(read_json(ROOT/'configs/dataset/v0_2/experiment_api_v1/cloth.json')['scene'])
            for mesh in scene['collision']['meshes']:
                mesh.pop('repair_winding',None)
                if name=='rope': mesh['friction']=.4
        scene['region_bounds_m']=[[2.2,1.2,.1],[3.5,1.8,1.5]]
        support='tabletop'
        if name in ('passage','roll','collision_chain'):
            support='room'
            scene['region_bounds_m']=[[-1.0,-1.4,-.1],[2.,-.6,.4]]
            obj=dict(id='subject',asset='ph_jug_01',size_m=.15,mass_kg=.3,rotation_deg=[0,0,0])
            if name=='passage': conditions=dict(speed_m_s=1.,clearance_regime='passable')
            elif name=='roll': conditions=dict(speed_m_s=.5,spin_ratio=1.)
            else:
                conditions=dict(speed_m_s=.5,gap_ratio=.2)
                obj=[dict(obj,id='body_a'),dict(obj,id='body_b',size_m=.12)]
        elif name=='cloth': obj=dict(size_m=[.5,.3]); conditions=dict(overhang_fraction=.3)
        elif name=='plastic': obj=dict(kind='box',size_m=[.12,.09,.08],density_kg_m3=900); conditions=dict(drop_height_m=.12)
        elif name=='beam':
            support='Tabletop'; obj=dict(size_m=[.38,.05,.025])
            conditions=dict(clamp_fraction=.13,deflection_fraction=.2,max_force_n=15.)
        else: obj=dict(length_m=.35,radius_m=.003,density_kg_m3=800); conditions=dict(clamp_fraction=.15)
        r=dict(format='phenomenon-construction/1',id='constructed_'+name,phenomenon=old['phenomenon'],
               profile='profiles/'+name+'.json',scene=scene,support_group=support,object=obj,conditions=conditions)
        write_json(dest/(name+'.json'),r)
    base='/mnt/y/isaacsim_work_cloth_gripper'
    examples=base+'/experiments/material_response/scene_input/examples/'
    for label,native in [('free','drag'),('blocked','blocked'),('low','lowforce')]:
        write_json(dest/('bridge_b_'+label+'.json'),dict(phenomenon='cloth_drag',workspace=base,
            config_path=examples+'classroom_cloth_'+native+'_depen005_trace5s.json',
            runtime=examples+'runtime.local.json',bounds=[[2.1,1.1,.1],[3.5,1.9,1.5]],experiment_id='bridge_b_'+label))
    base='/mnt/y/isaacsim_work_rope_load'
    write_json(dest/'bridge_c.json',dict(phenomenon='rope_finite_load',workspace=base,
        pinned_source='/mnt/y/isaacsim_work_construction/output/external_sources/rope_ca39d38',
        config_path=base+'/output/rope_load_c/inputs_travel24_v1/request.json',
        runtime=base+'/experiments/material_response/scene_input/examples/runtime.local.json',
        bounds=[[2.1,1.1,.1],[3.5,1.9,1.5]],experiment_id='bridge_c'))
    disabled=read_json(dest/'bridge_c.json')
    disabled.update(source_revision='05aabd376dc32916bb5541406ca67378da9fbc72',
        pinned_source='/mnt/y/isaacsim_work_construction/output/external_sources/rope_05aabd3',
        config_path=base+'/output/rope_load_c/inputs_disabled_v1/request.json',experiment_id='bridge_c_disabled')
    write_json(dest/'bridge_c_disabled.json',disabled)


if __name__=='__main__': main()

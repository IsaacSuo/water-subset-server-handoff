"""Phenomenon-led rigid asset batch; geometric selection, not asset-name staging."""
import argparse
import copy
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull, QhullError

from .causal_runner import ROOT,load_config,prepare,invoke,package_physics
from .io import read_json,write_json
from .physical_asset_smoke import make_config
from .physical_asset_bridge import resolve_asset
from .phenomenon_recipes02 import build,add_body


def select_geometry(mesh,kind):
    """Cheap layout hints only, not stable-pose or dynamic-contact certification."""
    lo,hi=mesh.bounds;d=float(mesh.extents.max())
    if kind=='support_edge':
        band=mesh.vertices[mesh.vertices[:,2]<=lo[2]+.01*d,:2]
        try:
            hull=ConvexHull(band)
            margin=float(-hull.equations[:,2].max())
            area=float(hull.volume)
        except QhullError:
            margin,area=-1.,0.
        return dict(selected=bool(margin>=.01*d and area>=.01*d*d),
                    reason='current-pose near-bottom support footprint, not a certified equilibrium',
                    bottom_band_m=.01*d,com_support_margin_m=margin,support_area_m2=area)
    if kind=='collision':
        radius=.3*d;grounded_z=-lo[2]
        candidates=[]
        for fy in (.5,.25,.75):
            for fz in (.5,.25,.75):
                point=lo+(hi-lo)*[0,fy,fz]
                if point[2]+grounded_z<radius:continue
                origin=[lo[0]-d,point[1],point[2]]
                hits,_,_=mesh.ray.intersects_location([origin],[[1,0,0]],multiple_hits=False)
                if len(hits):candidates.append(((fy-.5)**2+(fz-.5)**2,point.tolist()))
        if not candidates:return dict(selected=False,reason='No clear above-ground central-grid source-mesh hit ray')
        _,point=min(candidates,key=lambda item:item[0])
        return dict(selected=True,reason='source-mesh central-grid hit ray; moving target may change actual contact',
                    local_aim_yz_m=point[1:],projectile_radius_m=radius)
    return dict(selected=True,reason='positive-volume rigid geometry in free fall with ground clearance')


def recipe(exploration_root,name,kind,size=.2):
    asset_config=make_config(exploration_root,name,size)
    geometry=copy.deepcopy(asset_config['geometry_profiles']['exploration_asset'])
    material=copy.deepcopy(asset_config['physics_profiles']['asset_development'])
    preview_geometry=copy.deepcopy(geometry);preview_material=copy.deepcopy(material)
    mesh=resolve_asset(preview_geometry,preview_material,ROOT)
    selection=select_geometry(mesh,kind)
    selection.update(asset=name,kind=kind,extents_m=mesh.extents.tolist())
    if not selection['selected']:return None,None,selection
    lo,hi=mesh.bounds;d=float(mesh.extents.max())
    if kind in ('falling','support_edge'):
        config,manifest=build({'kind':kind},{'height_m':2*d,'speed_m_s':1.2},
                              {'shape':'box','size_m':mesh.extents.tolist(),'mass_kg':preview_material['mass_kg']})
        subject='left'
        state=manifest['initial_state']['body_states'][subject]
        if kind=='falling':state['position_m']=[0,0,2*d-lo[2]]
        else:
            top=2.25*d
            config['geometry_profiles']['platform']['size_m']=[6*d,5*d,.6*d]
            manifest['initial_state']['body_states']['platform']['position_m']=[-3*d,0,top-.3*d]
            state['position_m']=[-2.25*d,0,top-lo[2]]
    elif kind=='collision':
        config=load_config(ROOT/'configs/dataset/v0_2/c2_none_collision.json')
        manifest=read_json(ROOT/config['manifest_template'])
        config['initial_state_overrides']={};config.pop('body_profile_overrides',None)
        manifest['system']['bodies']=[];manifest['initial_state']['body_states']={}
        manifest['initial_state']['participant_ids']=[];manifest['environment']['environment_body_ids']=[]
        config['timing']['duration_s']=3.;manifest['timing']=copy.deepcopy(config['timing'])
        # Reserve collision travel space for the whole observation window; the
        # previous 8 m floor caused a fast light target to leave at ~2.9 s.
        floor_length=max(8.,4*1.2*config['timing']['duration_s']+4*d)
        add_body(config,manifest,'floor',{'shape':'box','size_m':[floor_length,2,.05]},'floor_reference',[0,0,-.025],kind='static',appearance='matte_gray',role='environment')
        add_body(config,manifest,'load',geometry,'asset_development',[0,0,-lo[2]])
        radius=selection['projectile_radius_m'];aim_y,aim_z=selection['local_aim_yz_m']
        config['physics_profiles']['projectile']=dict(config['physics_profiles']['rigid_reference'],mass_kg=.5,static_friction=0.,dynamic_friction=0.)
        add_body(config,manifest,'projectile',{'shape':'sphere','radius_m':radius},'projectile',
                 [lo[0]-radius-.5*d,aim_y,aim_z-lo[2]],[1.2,0,0],appearance='matte_orange')
        subject='load'
    else:raise ValueError(kind)
    config['numerics']['gpu_dynamics']=True
    config['prototype_id']='development_asset_'+kind
    manifest['environment']['environment_id']='self_built_asset_'+kind
    config['physics_profiles']['asset_development']=material
    config['geometry_profiles']['asset_geometry']=geometry
    body=next(b for b in manifest['system']['bodies'] if b['instance_id']==subject)
    body.update(geometry_id='asset_geometry',physics_profile_id='asset_development',appearance_profile_id='matte_blue')
    config['development_layout']=selection
    return config,manifest,selection


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--exploration-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--assets',nargs='+',default=['banana','elephant','chair','carrot'])
    p.add_argument('--kinds',nargs='+',default=['falling','support_edge','collision'])
    p.add_argument('--size',type=float,default=.2);p.add_argument('--prepare-only',action='store_true')
    a=p.parse_args();out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
    selected=[];pending=[];records=[]
    for kind in a.kinds:
        for name in a.assets:
            config,manifest,selection=recipe(a.exploration_root,name,kind,a.size);selected.append(selection)
            if config is None:continue
            folder=out/'recipes'/(kind+'_'+name);folder.mkdir(parents=True)
            write_json(folder/'manifest.json',manifest)
            config['manifest_template']=str(folder/'manifest.json')
            write_json(folder/'config.json',config)
            pending.append((kind,name,folder/'config.json',out/'episodes'/(kind+'_'+name)))
    write_json(out/'selection.json',{'scope':'development layout eligibility, not asset admission','assets':selected})
    for kind,name,path,episode in pending:
        prepare(path,episode)
        if not a.prepare_only:
            print('START',kind,name,flush=True);invoke('native_causal_rigid.py',episode,'simulation.log');package_physics(episode)
            print('DONE',kind,name,flush=True)
        records.append(dict(asset=name,kind=kind,episode=str(episode),status='prepared' if a.prepare_only else 'development_physics_completed'))
        write_json(out/('progress_'+str(len(records))+'.json'),{'episodes':records})
    write_json(out/'index.json',{'scope':'asset geometry development examples, not single-variable counterfactuals or release admission','episodes':records})


if __name__=='__main__':main()

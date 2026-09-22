"""Bounded volume, cantilever and passive cable construction on original support."""
import copy
from pathlib import Path

import numpy as np
from shapely.geometry import box

from .experiment_contract import keys
from .experiment_geometry import require, rectangle


def material(obj, cond, profile, geo, support, phenomenon):
    from .experiment_construct import number, transform
    z, surface = geo.support(support); lo,hi=geo.bounds
    cfg=copy.deepcopy(profile['input'])
    cy=(lo[1]+hi[1])/2
    if phenomenon=='plastic_impact':
        keys(obj, {'kind','size_m','path','density_kg_m3'}, {'kind','size_m','density_kg_m3'}, 'volume object')
        keys(cond, {'drop_height_m'}, {'drop_height_m'}, 'impact conditions')
        height=number(cond,'drop_height_m')
        number(obj,'density_kg_m3')
        native=dict(kind=obj['kind'],density_kg_m3=obj['density_kg_m3'],spacing_m=profile['discretization_m'])
        if obj['kind']=='box':
            dims=np.asarray(obj['size_m'],float)
            require(dims.shape==(3,) and np.isfinite(dims).all() and np.all(dims>0),'volume_size','positive xyz dimensions')
            low,high=-dims/2,dims/2; native['size_m']=dims.tolist()
        elif obj['kind']=='closed_mesh':
            import trimesh
            number(obj,'size_m'); path=Path(obj['path']); geo.pin(path)
            with np.load(path,allow_pickle=False) as data:
                m=trimesh.Trimesh(data['vertices'],data['triangles'],process=False)
            require(m.is_watertight and m.is_winding_consistent and m.volume>0,
                    'volume_mesh','require closed, consistently wound positive-volume NPZ')
            scale=obj['size_m']/max(m.extents); low,high=m.bounds*scale
            native.update(path=str(path.resolve()),scale=scale)
        else:
            raise ValueError('volume_kind: supported constructors are box and closed_mesh')
        require(np.min(high-low)>=3*native['spacing_m'],'discretization','object must span at least three particle spacings')
        center=np.array([(lo[0]+hi[0]-low[0]-high[0])/2,cy-(low[1]+high[1])/2,z+height-low[2]])
        footprint=box(*(center+low)[:2],*(center+high)[:2])
        require(surface.buffer(1e-7).covers(footprint),'landing_support','object projection not fully supported')
        geo.check_box(center+low,center+high)
        geo.check_box([*(center+low)[:2],z+1e-5],center+high,'drop_path')
        native['world_from_mesh']=transform(center); cfg['object']=native
        return cfg,dict(support_height_m=z,drop_height_m=height,object_bounds_m=[low.tolist(),high.tolist()],
                        limits='undeformed impact path only; plastic response and postimpact clearance unverified')

    components=list(surface.geoms) if hasattr(surface,'geoms') else [surface]
    components=[p for p in components if p.intersects(box(*lo[:2],*hi[:2]))]
    require(len(components)==1,'support_selection','select one connected horizontal support')
    surface=components[0]; edge=surface.bounds[2]
    if phenomenon=='beam_load_hold_withdraw':
        keys(obj,{'size_m'},{'size_m'},'beam object')
        keys(cond,{'clamp_fraction','deflection_fraction','max_force_n'},
             {'clamp_fraction','deflection_fraction','max_force_n'},'beam conditions')
        dims=np.asarray(obj['size_m'],float)
        require(dims.shape==(3,) and np.isfinite(dims).all() and np.all(dims>0),'beam_size','positive xyz dimensions')
        length,width,thickness=dims
        cf=number(cond,'clamp_fraction'); df=number(cond,'deflection_fraction',zero=True)
        require(.05<=cf<=.3 and df<=.3,'beam_fraction','clamp 0.05..0.3; deflection 0..0.3')
        require(length>=4*max(width,thickness),'beam_shape','cantilever rule requires length >= 4 * cross-section')
        clamp=length*cf; gap=cfg['contact_offset_m']*2
        root=edge-clamp
        footprint=rectangle([edge-clamp/2,cy],[clamp,width])
        require(surface.buffer(1e-7).covers(footprint),'root_support','clamped root footprint not on original support')
        center=np.array([root+length/2,cy,z+gap+thickness/2])
        geo.check_box(center-dims/2,center+dims/2)
        drop=length*(1-cf)*df
        clear_start=geo.edge_transition(edge,cy-width/2,cy+width/2,z,drop+gap,length-clamp)
        geo.check_box([clear_start,cy-width/2,z-drop-gap],
                      [root+length,cy+width/2,z-1e-5],'deflection_clearance')
        fixture_size=[clamp,width*1.4,max(.01,thickness*.8)]
        fixture_center=[root+clamp/2,cy,center[2]+thickness/2+gap+fixture_size[2]/2]
        plate_size=[min(length*(1-cf)*.3,.08),width*1.6,max(.01,thickness)]
        px=root+length-plate_size[0]/2
        pz=center[2]+thickness/2+gap+plate_size[2]/2
        travel=drop+gap
        geo.check_box(np.array([px,cy,pz])-np.array(plate_size)/2-[0,0,travel],
                      np.array([px,cy,pz])+np.array(plate_size)/2,'plate_sweep')
        geo.check_box(np.array(fixture_center)-np.array(fixture_size)/2,
                      np.array(fixture_center)+np.array(fixture_size)/2,'fixture_clearance')
        duration=profile['timing']['duration_s']; hz=profile['timing']['physics_hz']
        times=[round(duration*f*hz)/hz for f in (0,.125,.325,.5,.625,1)]
        cfg['beam']=dict(size_m=dims.tolist(),center_m=center.tolist(),**copy.deepcopy(profile['beam_properties']))
        cfg['fixture']=dict(size_m=fixture_size,center_m=fixture_center,length_m=clamp,mass_kg=1.)
        cfg['plate']=dict(size_m=plate_size,center_m=[px,cy,pz],**copy.deepcopy(profile['plate_properties']),
                          max_force_n=number(cond,'max_force_n'),schedule=[[t,pz-travel if i in (2,3) else pz] for i,t in enumerate(times)],
                          guide_limits_m=[-travel-gap,gap])
        return cfg,dict(support_edge_x_m=edge,clamp_length_m=clamp,free_span_m=length-clamp,
                        requested_deflection_m=drop,plate_travel_m=travel,
                        edge_contact_band_m=[edge,clear_start],edge_contact_effect='unverified',
                        constraint='native -X ideal attachment to visible world-fixed fixture; finite Z plate',
                        limits='command geometry only; load contact, withdrawal and beam response unverified')

    if phenomenon=='rope_passive':
        keys(obj,{'length_m','radius_m','density_kg_m3'},{'length_m','radius_m','density_kg_m3'},'rope object')
        keys(cond,{'clamp_fraction'},{'clamp_fraction'},'passive rope conditions')
        length=number(obj,'length_m'); radius=number(obj,'radius_m'); number(obj,'density_kg_m3')
        fraction=number(cond,'clamp_fraction')
        require(.05<=fraction<=.3,'clamp_fraction','passive clamped rule allows 0.05..0.3')
        require(length>=20*radius,'rope_shape','length must exceed 20 radii')
        clamp=length*fraction; gap=2*radius
        footprint=rectangle([edge-clamp/2,cy],[clamp,2*radius])
        require(surface.buffer(1e-7).covers(footprint),'anchor_support','selected original support cannot accommodate clamp interval')
        anchor=[edge-clamp,cy,z+radius+gap]
        geo.check_box([anchor[0]-radius,cy-radius,z+gap],
                      [anchor[0]+length+radius,cy+radius,z+gap+2*radius])
        clear_start=geo.edge_transition(edge,cy-radius,cy+radius,z,length-clamp+radius,length-clamp)
        geo.check_box([clear_start,cy-radius,z-(length-clamp)-radius],
                      [anchor[0]+length+radius,cy+radius,z-1e-5],'rope_swing_clearance')
        segments=max(4,int(np.ceil(length/profile['discretization_m'])))
        require(segments<=512,'discretization','passive rope segment budget exceeded')
        cfg['rope']=dict(copy.deepcopy(profile['rope_properties']),anchor_m=anchor,direction=[1,0,0],
                         length_m=length,clamp_length_m=clamp,segments=segments,end_condition='first_clamped',
                         radius_m=radius,density_kg_m3=obj['density_kg_m3'])
        return cfg,dict(support_edge_x_m=edge,segments=segments,clamp_length_m=clamp,
                        free_length_m=length-clamp,constraint='ideal first-clamped passive cable; no finite attached load',
                        edge_contact_band_m=[edge,clear_start],edge_contact_effect='unverified',
                        limits='reserved initial/downward prism; full 3D swinging envelope and physical response unverified')
    raise ValueError('construction_missing: '+phenomenon+' has no construction rule')

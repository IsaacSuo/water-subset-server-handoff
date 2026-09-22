"""Object/region driven layouts; no solver implementation or scene mutation."""
import copy
import numpy as np
from shapely.geometry import box
from .experiment_contract import keys
from .experiment_geometry import require


def cloth_drag(obj,cond,profile,geo,support):
    from .experiment_construct import number,transform,passage_camera
    keys(obj,{'size_m'},{'size_m'},'drag cloth')
    keys(cond,{'grip_fraction','travel_m','max_force_n','mode'}, {'grip_fraction','travel_m','max_force_n','mode'},'drag conditions')
    size=np.asarray(obj['size_m'],float)
    require(size.shape==(2,) and np.isfinite(size).all() and np.all(size>0),'cloth_size','positive rectangle dimensions')
    mode=cond['mode'];require(mode in ('free','blocked','low_force'),'drag_mode','free, blocked or low_force')
    grip=number(cond,'grip_fraction');travel=number(cond,'travel_m');force=number(cond,'max_force_n')
    require(.03<=grip<=.2,'grip_fraction','local edge fraction 0.03..0.2')
    if mode=='low_force':require(force<=profile['low_force_limit_n'],'low_force','requested cap exceeds declared low-force range')
    z,surface=geo.support(support);lo,hi=geo.bounds;spacing=profile['discretization_m']
    cells=np.ceil(size/spacing).astype(int);require(np.prod(cells+1)<=20000,'discretization','cloth node budget exceeded')
    cfg=copy.deepcopy(profile['input']);gap=max(2*cfg['collision']['contact_offset_m'],2*cfg['material']['thickness_m'])
    xmin=max(lo[0],surface.bounds[0]);xmax=min(hi[0],surface.bounds[2])
    center=np.array([(xmin+xmax-travel)/2,(lo[1]+hi[1])/2,z+gap]);low=center-np.r_[size/2,0.000001];high=center+np.r_[size/2,0.000001]
    require(surface.buffer(1e-7).covers(box(*low[:2],*high[:2])),'drag_support','cloth initial footprint not on original support')
    geo.check_box(low,high);geo.check_box(low,high+[travel,0,0],'drag_initial_plane_sweep')
    cfg['cloth']=dict(kind='rectangle',size_m=size.tolist(),cells=cells.tolist(),world_from_mesh=transform(center))
    gs=np.array([max(.012,2*spacing),min(size[1]*.4,.08),.01]);strip=max(size[0]*grip,spacing*1.01)
    require(strip<size[0]/3,'grip_selection','cloth too short for a local resolved grip')
    grip_center=np.array([high[0],center[1],center[2]+gs[2]/2+gap])
    duration=profile['timing']['duration_s'];hz=profile['timing']['physics_hz']
    times=[round(duration*f*hz)/hz for f in (0,.1,.3,.4,1)]
    gripper=dict(copy.deepcopy(profile['gripper_properties']),center_m=grip_center.tolist(),size_m=gs.tolist(),
        attachment_bounds_world_m=[[high[0]-strip,center[1]-gs[1]/2,center[2]-1e-5],[high[0]+1e-5,center[1]+gs[1]/2,center[2]+1e-5]],
        guide_limits_m=[-max(.02,travel*.2),travel+max(.02,travel*.2)],max_force_n=force,
        schedule=[[t,float(grip_center[0]+(travel if i>=2 else 0))] for i,t in enumerate(times)])
    geo.check_box(grip_center-gs/2,grip_center+gs/2+[travel,0,0],'gripper_sweep')
    cfg['gripper']=gripper
    if mode=='blocked':
        opposing=grip_center.copy();opposing[0]=low[0]
        cfg['opposing_fixture']=dict(center_m=opposing.tolist(),size_m=gs.tolist(),mass_kg=profile['gripper_properties']['mass_kg'],
            attachment_bounds_world_m=[[low[0]-1e-5,center[1]-gs[1]/2,center[2]-1e-5],[low[0]+strip,center[1]+gs[1]/2,center[2]+1e-5]])
        geo.check_box(opposing-gs/2,opposing+gs/2,'opposing_fixture_clearance')
    points=np.stack(np.meshgrid(np.linspace(low[0],high[0],cells[0]+1),np.linspace(low[1],high[1],cells[1]+1),[center[2]],indexing='ij'),-1).reshape(-1,3)
    selections={}
    for key in ('gripper','opposing_fixture'):
        if key not in cfg:continue
        a,b=np.asarray(cfg[key]['attachment_bounds_world_m']);ids=np.flatnonzero(np.all((points>=a)&(points<=b),axis=1))
        require(2<=len(ids)<len(points)/2,'attachment_selection','grip must select at least two and less than half of nodes')
        selections[key]=dict(node_count=len(ids),bounds_world_m=[a.tolist(),b.tolist()])
    target=center+[travel/2,0,0];span=max(size[0]+travel,size[1])*1.5
    return cfg,dict(design='supported rectangle, native finite X gripper, optional visible fixed opposing attachment',
        mode=mode,attachments=selections,requested_travel_m=travel,region_limited_travel_m=float(hi[0]-grip_center[0]-gs[0]/2),
        observation_camera=passage_camera(geo,center,target,span),
        limits='rectangle initially parallel to world XY; +X native guide; fixed opposing fixture is existing backend capability, not an infinitely driven actuator; attachment reaction unavailable')


def rearrange(objects,conditions,profile,geo,support):
    from .experiment_construct import body_geometry,number,passage_camera
    keys(conditions,{'layer_gap_m','offset_fraction'},{'layer_gap_m','offset_fraction'},'gravity stack')
    require(isinstance(objects,list) and 2<=len(objects)<=6,'stack_count','two to six objects required')
    require(len({o['id'] for o in objects})==len(objects),'participants','unique object IDs required')
    gap=number(conditions,'layer_gap_m');offset=number(conditions,'offset_fraction',zero=True)
    require(.002<=gap<=.03 and offset<=.35,'stack_conditions','gap 2..30 mm, offset fraction 0..0.35')
    library=profile['asset_library'];bounds=[body_geometry(o,library,geo) for o in objects]
    z,surface=geo.support(support);lo,hi=geo.bounds
    xy=(lo[:2]+hi[:2])/2;bottom=z+gap;participants=[];placements=[];previous=None
    for i,(obj,(low,high)) in enumerate(zip(objects,bounds)):
        if i:xy=xy+np.array([(-1)**i*offset*min(high[0]-low[0],bounds[i-1][1][0]-bounds[i-1][0][0]),0])
        center=np.r_[xy-(low[:2]+high[:2])/2,bottom-low[2]]
        footprint=box(*(center+low)[:2],*(center+high)[:2])
        require(surface.buffer(1e-7).covers(footprint),'stack_support','every falling footprint must remain over original support')
        geo.check_box(center+low,center+high)
        geo.check_box([*(center+low)[:2],z+1e-5],center+high,'stack_fall_clearance')
        overlap=None if previous is None else footprint.intersection(previous).area/min(footprint.area,previous.area)
        require(overlap is None or overlap>=.25,'stack_contact_opportunity','successive projected footprints overlap less than 25%')
        participants.append(dict(copy.deepcopy(obj),xy_m=center[:2].tolist(),support_group=support,
            ray_start_z_m=z+.0001,clearance_m=bottom-z,velocity_m_s=[0,0,0]))
        placements.append(dict(id=obj['id'],position_m=center.tolist(),bounds_m=[(center+low).tolist(),(center+high).tolist()],
            potential_support=support if i==0 else objects[i-1]['id'],projected_overlap_fraction=overlap,initial_vertical_gap_m=gap))
        bottom=center[2]+high[2]+gap;previous=footprint
    # Exact AABB separation is a conservative no-initial-intersection certificate.
    for a,b in zip(placements[:-1],placements[1:]):
        require(b['bounds_m'][0][2]-a['bounds_m'][1][2]>=gap-1e-8,'stack_overlap','initial layers intersect')
    p=np.array([s['position_m'] for s in placements]);target=p.mean(0)
    span=max(.4,(bottom-z)*2,np.ptp(p,axis=0).max()*2)
    camera=passage_camera(geo,p[-1],target,span)
    return dict(participants=participants,asset_library=copy.deepcopy(library)),dict(
        design='staggered gravity-settling stack; no initial motion or overlap',placements=placements,
        observation_camera=camera,initial_intersections=False,
        limits='horizontal original support, 2..6 assets, gravity-settling contact opportunities; not arbitrary stable packing; final stability must be measured')

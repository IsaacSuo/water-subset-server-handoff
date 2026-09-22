"""Object/region driven layouts; no solver implementation or scene mutation."""
import copy
import numpy as np
from shapely.geometry import box
from .experiment_contract import keys
from .experiment_geometry import require


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

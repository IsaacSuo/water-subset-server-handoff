"""Pure descriptions of reusable fixtures, in world metres, Z up."""
from __future__ import annotations

import math
import numpy as np


def box(name,position,size,angle_y=0.,kinematic=False,angle_z=0.):
    sy,cy=math.sin(angle_y/2),math.cos(angle_y/2);sz,cz=math.sin(angle_z/2),math.cos(angle_z/2)
    return dict(id=name,position_m=list(map(float,position)),size_m=list(map(float,size)),
                orientation_xyzw=[-sz*sy,cz*sy,sz*cy,cz*cy],kinematic=kinematic)


def flat_ground(d):
    # Covers the full four-second high-angle R01 rollout without changing the
    # finite, auditable fixture into an implicit infinite plane.
    return [box('floor',[36*d,0,-.1*d],[80*d,8*d,.2*d])]


def collision_ground(d):
    """Symmetric finite lane with room for rebound in either X direction."""
    return [box('floor',[0,0,-.1*d],[24*d,8*d,.2*d])]


def ramp(d,angle_deg,length_D=4):
    a=math.radians(angle_deg);length=length_D*d;thickness=.1*d
    n=np.array([math.sin(a),0,math.cos(a)]);u=np.array([math.cos(a),0,-math.sin(a)])
    centre=-.5*length*u-.5*thickness*n
    return flat_ground(d)+[box('ramp',centre,[length,2*d,thickness],a)]


def stairs(d,count=5,height_D=.3,tread_D=1.5):
    if int(count)!=count or not 4<=count<=6 or not .2<=height_D<=.5 or not 1.<=tread_D<=2.:
        raise ValueError('Outside stair domain')
    count=int(count);tread=tread_D*d
    return flat_ground(d)+[box(f'step_{i}',[(i-(count-1)/2)*tread,0,(count-i)*height_D*d/2],
                              [tread,3*d,(count-i)*height_D*d]) for i in range(count)]


def obstacles(d):
    return flat_ground(d)+[box('wall_left',[0,-d,.5*d],[.25*d,d,d]),
                           box('wall_right',[0,d,.5*d],[.25*d,d,d]),
                           box('obstacle',[2*d,.5*d,.5*d],[.5*d,.5*d,d])]


def compression_plates(d,height):
    return flat_ground(d)+[box('upper_plate',[0,0,1.05*height+.1*d],[2*d,2*d,.2*d],kinematic=True)]


def build_fixture(spec,inputs,vertices):
    from scipy.spatial.transform import Rotation
    if spec['event_id']=='R03':
        if not isinstance(vertices,dict):raise ValueError('R03 requires per-instance geometry')
        d=max(o['geometry']['characteristic_size_m'] for o in inputs['objects'])
        separation=spec['fixture_parameters']['separation_D']*d
        offset=spec['fixture_parameters']['lateral_offset_D']*d
        positions={}
        for index,o in enumerate(inputs['objects']):
            rotated=Rotation.from_quat(o['orientation_xyzw']).apply(vertices[o['instance_id']])
            positions[o['instance_id']]=[-separation/2 if index==0 else separation/2,
                                         -offset/2 if index==0 else offset/2,
                                         -float(rotated[:,2].min())+.0002]
        return {'boxes':collision_ground(d),'subject_positions_m':positions,'D_m':d,
                'collision_axis_world':[1.,0.,0.],'lateral_axis_world':[0.,1.,0.]}
    if spec['event_id']=='V05':
        if not isinstance(vertices,dict):raise ValueError('V05 requires per-instance geometry')
        soft=[o for o in inputs['objects'] if o['physics']['kind']=='volumetric']
        rigid=[o for o in inputs['objects'] if o['physics']['kind']=='rigid']
        if len(soft)!=1 or len(rigid)!=1:raise ValueError('V05 requires one volumetric target and one rigid projectile')
        target,projectile=soft[0],rigid[0];d=target['geometry']['characteristic_size_m']
        target_vertices=Rotation.from_quat(target['orientation_xyzw']).apply(vertices[target['instance_id']])
        projectile_vertices=Rotation.from_quat(projectile['orientation_xyzw']).apply(vertices[projectile['instance_id']])
        separation=spec['fixture_parameters']['separation_D']*d
        target_position=np.array([0.,0.,-float(target_vertices[:,2].min())+.0002])
        projectile_x=(float(target_vertices[:,0].min())-float(projectile_vertices[:,0].max())-separation)
        projectile_position=np.array([projectile_x,spec['fixture_parameters']['impact_offset_D']*d,
                                      -float(projectile_vertices[:,2].min())+.0002])
        # Size from the declared speed domain, not the chosen action speed:
        # speed pairs and hit/miss controls retain the same supporting fixture.
        speed_limit=inputs['event']['parameters']['impact_speed_m_s'][1]
        half_length=max(12*d,speed_limit*spec['timing']['duration_s']+4*d)
        ground=collision_ground(d);ground[0]['size_m'][0]=2*half_length
        return {'boxes':ground,'subject_positions_m':{
                    target['instance_id']:target_position.tolist(),projectile['instance_id']:projectile_position.tolist()},
                'target_id':target['instance_id'],'projectile_id':projectile['instance_id'],'D_m':d,
                'impact_axis_world':[1.,0.,0.],
                'fixture_material':{'static_friction':0.,'dynamic_friction':0.,'restitution':0.,
                                    'friction_combine_mode':'min','restitution_combine_mode':'average'}}
    o=inputs['objects'][0];d=o['geometry']['characteristic_size_m']
    rotated=Rotation.from_quat(o['orientation_xyzw']).apply(vertices)
    height=float(np.ptp(rotated[:,2]));bottom=float(rotated[:,2].min())
    if spec['event_id']=='V01':
        clearance=spec['fixture_parameters']['drop_height_D']*d
        return {'boxes':collision_ground(d),'subject_position_m':[0,0,clearance-bottom],
                'rest_height_m':height,'drop_clearance_m':clearance,'D_m':d,
                'fixture_material':{'static_friction':0.,'dynamic_friction':0.,'restitution':0.,
                                    'friction_combine_mode':'min','restitution_combine_mode':'average'}}
    if spec['event_id']=='V02':
        return {'boxes':compression_plates(d,height),'subject_position_m':[0,0,-bottom+.0002],
                'rest_height_m':height,'D_m':d}
    if spec['event_id']=='R02':
        fp=spec['fixture_parameters'];boxes=stairs(d,fp['step_count'],fp['step_height_D'],fp['tread_depth_D'])
        top=next(item for item in boxes if item['id']=='step_0')
        top_surface=top['position_m'][2]+top['size_m'][2]/2
        front=float(rotated[:,0].max());top_front=top['position_m'][0]+top['size_m'][0]/2
        edge_clearance=.05*d;start_x=top_front-front-edge_clearance
        return {'boxes':boxes,'subject_position_m':[start_x,0,top_surface-bottom+.0002],
                'rest_height_m':height,'D_m':d,'travel_axis_world':[1.,0.,0.],
                'step_ids':[f"step_{i}" for i in range(int(fp['step_count']))],
                'top_surface_z_m':top_surface,'bottom_surface_z_m':0.,
                'start_edge_clearance_m':edge_clearance}
    if spec['event_id']=='R04':
        fp=spec['fixture_parameters'];subject_x=-2*d;subject_y=fp['subject_offset_D']*d
        subject_position=[subject_x,subject_y,-bottom+.0002]
        pusher_thickness=.2*d;pusher_gap=.08*d
        pusher_x=subject_x+float(rotated[:,0].min())-pusher_gap-pusher_thickness/2
        obstacle_x=.5*d;obstacle_y=fp['obstacle_offset_D']*d
        boxes=[box('floor',[2*d,0,-.1*d],[14*d,6*d,.2*d]),
               box('rail_left',[2*d,-2.5*d,.6*d],[14*d,.2*d,1.2*d]),
               box('rail_right',[2*d,2.5*d,.6*d],[14*d,.2*d,1.2*d]),
               box('obstacle',[obstacle_x,obstacle_y,.6*d],[.8*d,.8*d,1.2*d],angle_z=math.pi/4),
               box('pusher',[pusher_x,subject_y,.65*d],[pusher_thickness,1.35*d,1.3*d],kinematic=True)]
        return {'boxes':boxes,'subject_position_m':subject_position,'rest_height_m':height,'D_m':d,
                'travel_axis_world':[1.,0.,0.],'lateral_axis_world':[0.,1.,0.],
                'pusher_id':'pusher','obstacle_id':'obstacle','pusher_start_position_m':boxes[-1]['position_m'],
                'obstacle_forward_face_x_m':obstacle_x+math.sqrt(2)*.8*d/2,
                'fixture_material':{'static_friction':.4,'dynamic_friction':.3,'restitution':0.,
                                    'friction_combine_mode':'min','restitution_combine_mode':'average'}}
    a=math.radians(spec['fixture_parameters']['angle_deg'])
    n=np.array([math.sin(a),0,math.cos(a)]);u=np.array([math.cos(a),0,-math.sin(a)])
    start=-3.1*d*u
    support=-float((rotated@n).min())
    position=start+(support+.0002)*n
    boxes=ramp(d,spec['fixture_parameters']['angle_deg'],spec['fixture_parameters']['length_D'])
    forward=float((rotated@u).max());gate_centre=position+(forward+.04*d+.0002)*u
    boxes.append(box('gate',gate_centre,[.08*d,2*d,2*d],a,True))
    return {'boxes':boxes,'subject_position_m':position.tolist(),'rest_height_m':height,'D_m':d,
            'ramp_normal':n.tolist(),'downhill_tangent':u.tolist(),'bottom_x_m':0.0}

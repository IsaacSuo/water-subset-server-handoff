"""Pure descriptions of reusable fixtures, in world metres, Z up."""
from __future__ import annotations

import math
import numpy as np


def box(name,position,size,angle_y=0.,kinematic=False):
    return dict(id=name,position_m=list(map(float,position)),size_m=list(map(float,size)),
                orientation_xyzw=[0.,math.sin(angle_y/2),0.,math.cos(angle_y/2)],kinematic=kinematic)


def flat_ground(d):
    # Covers the full four-second high-angle R01 rollout without changing the
    # finite, auditable fixture into an implicit infinite plane.
    return [box('floor',[36*d,0,-.1*d],[80*d,8*d,.2*d])]


def ramp(d,angle_deg,length_D=4):
    a=math.radians(angle_deg);length=length_D*d;thickness=.1*d
    n=np.array([math.sin(a),0,math.cos(a)]);u=np.array([math.cos(a),0,-math.sin(a)])
    centre=-.5*length*u-.5*thickness*n
    return flat_ground(d)+[box('ramp',centre,[length,2*d,thickness],a)]


def stairs(d,count=5,height_D=.3):
    if not 4<=count<=6 or not .2<=height_D<=.5:raise ValueError('Outside stair domain')
    return flat_ground(d)+[box(f'step_{i}',[(i-count/2)*d,0,(count-i)*height_D*d/2],
                              [d,2*d,(count-i)*height_D*d]) for i in range(count)]


def obstacles(d):
    return flat_ground(d)+[box('wall_left',[0,-d,.5*d],[.25*d,d,d]),
                           box('wall_right',[0,d,.5*d],[.25*d,d,d]),
                           box('obstacle',[2*d,.5*d,.5*d],[.5*d,.5*d,d])]


def compression_plates(d,height):
    return flat_ground(d)+[box('upper_plate',[0,0,1.05*height+.1*d],[2*d,2*d,.2*d],kinematic=True)]


def build_fixture(spec,inputs,vertices):
    from scipy.spatial.transform import Rotation
    o=inputs['objects'][0];d=o['geometry']['characteristic_size_m']
    rotated=Rotation.from_quat(o['orientation_xyzw']).apply(vertices)
    height=float(np.ptp(rotated[:,2]));bottom=float(rotated[:,2].min())
    if spec['event_id']=='V02':
        return {'boxes':compression_plates(d,height),'subject_position_m':[0,0,-bottom+.0002],
                'rest_height_m':height,'D_m':d}
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

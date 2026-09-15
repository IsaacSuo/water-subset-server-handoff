"""Event-independent action commands; all times are physical seconds."""
from __future__ import annotations

def smoothstep(u):
    u = min(1.0, max(0.0, float(u)))
    return u*u*(3.0-2.0*u)


def sample_trajectory(command, time_s):
    a, b = command["start_time_s"], command["end_time_s"]
    p = command["parameters"]
    if b <= a:
        raise ValueError("Trajectory requires positive duration")
    u = smoothstep((time_s-a)/(b-a))
    return [(1-u)*x+u*y for x,y in zip(p["from_m"],p["to_m"])]


def due(command, step, hz):
    """One-shot operations are delivered exactly once on an integer step."""
    return step == round(command["start_time_s"] * hz)


def compile_actions(spec, inputs, fixture=None):
    ap = spec["action_parameters"]
    commands = []
    if spec["event_id"] == "R01":
        commands.append(dict(kind="release", target="gate", start_time_s=ap["release_time_s"],
                             end_time_s=ap["release_time_s"], parameters={"method": "disable_collision"}))
    elif spec["event_id"] == "R03":
        t=ap['start_time_s']
        left_id,right_id=(o['instance_id'] for o in inputs['objects'])
        commands.extend([
            dict(kind='initial_velocity',target=left_id,start_time_s=t,end_time_s=t,
                 parameters={'linear_m_s':[ap['left_speed_m_s'],0.,0.], 'angular_rad_s':[0.,0.,0.]}),
            dict(kind='initial_velocity',target=right_id,start_time_s=t,end_time_s=t,
                 parameters={'linear_m_s':[-ap['right_speed_m_s'],0.,0.], 'angular_rad_s':[0.,0.,0.]}),
        ])
    elif spec["event_id"] == "V02":
        if fixture is None:
            raise ValueError('V02 action compilation requires its measured fixture')
        plate=next(b for b in fixture['boxes'] if b['id']=='upper_plate')
        def location(fraction):
            return [plate['position_m'][0],plate['position_m'][1],
                    fraction*fixture['rest_height_m']+.1*fixture['D_m']]
        t = ap["start_time_s"]
        low = 1-spec["fixture_parameters"]["compression_fraction"]
        for duration, start, end in ((ap["compression_duration_s"], 1.05, low),
                                     (ap["hold_duration_s"], low, low),
                                     (ap["withdraw_duration_s"], low, 1.4)):
            if duration > 0:
                commands.append(dict(kind="kinematic_trajectory", target="upper_plate", start_time_s=t,
                                     end_time_s=t+duration, parameters={"interpolation": "smoothstep",
                                         "from_m":location(start),"to_m":location(end)}))
            t += duration
    elif spec["event_id"] == "V01":
        # Initial elevation plus the declared environment gravity fully defines
        # free fall. No synthetic release impulse or hidden support is authored.
        pass
    elif spec["event_id"] == "R02":
        t=ap["start_time_s"]
        commands.append(dict(kind="initial_velocity",target=inputs["objects"][0]["instance_id"],
            start_time_s=t,end_time_s=t,
            parameters={"linear_m_s":[ap["push_speed_m_s"],0.,0.],"angular_rad_s":[0.,0.,0.]}))
    elif spec["event_id"] == "V05":
        rigid=[obj for obj in inputs["objects"] if obj["physics"]["kind"]=="rigid"]
        if len(rigid)!=1:raise ValueError("V05 requires exactly one rigid projectile")
        t=ap["start_time_s"]
        commands.append(dict(kind="initial_velocity",target=rigid[0]["instance_id"],
            start_time_s=t,end_time_s=t,
            parameters={"linear_m_s":[ap["impact_speed_m_s"],0.,0.],"angular_rad_s":[0.,0.,0.]}))
    elif spec["event_id"] == "R04":
        if fixture is None:raise ValueError('R04 action compilation requires its measured fixture')
        start=fixture['pusher_start_position_m'];distance=ap['push_distance_D']*fixture['D_m'];t=ap['start_time_s']
        commands.append(dict(kind='kinematic_trajectory',target=fixture['pusher_id'],start_time_s=t,
            end_time_s=t+ap['push_duration_s'],parameters={'interpolation':'smoothstep',
                'from_m':start,'to_m':[start[0]+distance,start[1],start[2]]}))
    elif spec["event_id"] == "V03":
        if fixture is None:raise ValueError('V03 action compilation requires its measured fixture')
        t=ap['load_start_time_s'];target=fixture['load_id']
        commands.extend([
            dict(kind='release',target=target,start_time_s=t,end_time_s=t,
                 parameters={'method':'set_dynamic'}),
            dict(kind='remove_support',target=target,start_time_s=t+ap['load_duration_s'],
                 end_time_s=t+ap['load_duration_s'],parameters={'method':'deactivate_actor'}),
        ])
    else:
        raise ValueError("Unimplemented event")
    return dict(schema_version="0.1.0", time_units="s", sample_semantics="command_at_step_start_state_after_step",
                physics_dt_s=1/spec["timing"]["physics_hz"], commands=commands)

"""Pure finite-force command evaluation; references are never body state writes."""
from __future__ import annotations

from .controllers import bounded_linear_impedance, bounded_velocity_effort


def reference(command, time_s):
    target = command["target"]
    if target.get("trajectory") == "smoothstep":
        duration = command["end_time_s"] - command["start_time_s"]
        u = max(0.0, min(1.0, (time_s - command["start_time_s"]) / duration))
        displacement = target["position_end_m"] - target["position_start_m"]
        return (target["position_start_m"] + displacement * u*u*(3-2*u),
                displacement * 6*u*(1-u) / duration)
    return target.get("position_m"), target.get("velocity_m_s", 0.0)


def evaluate(controller, commands, time_s, position_m, velocity_m_s):
    active = [c for c in commands if c["controller_id"] == controller["controller_id"] and
              c["start_time_s"] <= time_s < c["end_time_s"]]
    if len(active) > 1:
        raise ValueError("Overlapping commands for the same actuator")
    if not active:
        return {"requested_force_n": 0.0, "applied_force_n": 0.0, "saturated": False,
                "command_active": False, "command_id": None}
    command = active[0]
    if command["limits"]["max_force_n"] != controller["max_force_n"]:
        raise ValueError("Command and controller force limits disagree")
    position, velocity = reference(command, time_s)
    if controller["primitive"] == "impedance_control":
        decision = bounded_linear_impedance(position, velocity, position_m, velocity_m_s,
            controller["stiffness_n_m"], controller["damping_n_s_m"], controller["max_force_n"])
    else:
        decision = bounded_velocity_effort(velocity, velocity_m_s,
            command["target"]["velocity_gain_n_s_m"], controller["max_force_n"])
    return dict(decision, command_active=True, command_id=command["command_id"],
                reference_position_m=position, reference_velocity_m_s=velocity)

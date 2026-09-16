"""Backend-independent bounded controllers used by causal actuator probes."""
from __future__ import annotations

import math


def bounded_velocity_effort(target_velocity_m_s, measured_velocity_m_s, gain_n_s_m, max_force_n):
    """Return requested force, clipped applied force and saturation state."""
    values = (target_velocity_m_s, measured_velocity_m_s, gain_n_s_m, max_force_n)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Controller inputs must be finite")
    if gain_n_s_m <= 0 or max_force_n <= 0:
        raise ValueError("Controller gain and force limit must be positive")
    requested = gain_n_s_m * (target_velocity_m_s - measured_velocity_m_s)
    applied = max(-max_force_n, min(max_force_n, requested))
    return {
        "requested_force_n": requested,
        "applied_force_n": applied,
        "saturated": abs(requested) > max_force_n,
    }


def signed_work_increment(force_n, previous_position_m, current_position_m):
    values = (force_n, previous_position_m, current_position_m)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Work inputs must be finite")
    return force_n * (current_position_m - previous_position_m)


def bounded_linear_impedance(target_position_m, target_velocity_m_s, measured_position_m,
                             measured_velocity_m_s, stiffness_n_m, damping_n_s_m, max_force_n):
    """Return a force-limited one-dimensional spring-damper command."""
    values = (target_position_m, target_velocity_m_s, measured_position_m, measured_velocity_m_s,
              stiffness_n_m, damping_n_s_m, max_force_n)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Controller inputs must be finite")
    if stiffness_n_m <= 0 or damping_n_s_m < 0 or max_force_n <= 0:
        raise ValueError("Impedance stiffness/force must be positive and damping nonnegative")
    requested = (stiffness_n_m * (target_position_m - measured_position_m) +
                 damping_n_s_m * (target_velocity_m_s - measured_velocity_m_s))
    applied = max(-max_force_n, min(max_force_n, requested))
    return {
        "requested_force_n": requested,
        "applied_force_n": applied,
        "saturated": abs(requested) > max_force_n,
        "position_error_m": target_position_m - measured_position_m,
        "velocity_error_m_s": target_velocity_m_s - measured_velocity_m_s,
    }

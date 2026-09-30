"""Pressure projection updates and bounded convergence retries."""

import math

import taichi as ti


@ti.func
def pair_pressure_acceleration(volume_gradient, pressure_i, pressure_j):
    return -volume_gradient * (pressure_i + pressure_j)


@ti.func
def divergence_pressure_update(pressure, source, applied_operator, alpha, dt, active):
    # The mask is fixed for this projection.  Deficient particles still receive
    # neighboring pressure acceleration, but never generate their own pressure.
    updated = ti.cast(0.0, ti.f32)
    error = ti.cast(0.0, ti.f32)
    if active != 0:
        residual = source - applied_operator
        updated = ti.max(pressure - 0.5 * residual * alpha / dt, 0.0)
        error = -ti.min(residual, 0.0)
    return updated, error


def projection_retry_decision(converged, retries, dt, minimum_dt, max_halvings=4):
    """Return accept, halve, or accept_flagged for pressure convergence alone.

    CFL violations and nonfinite states are handled separately and can never
    use the flagged acceptance path.
    """
    if (not math.isfinite(dt) or not math.isfinite(minimum_dt)
            or dt <= 0.0 or minimum_dt <= 0.0 or retries < 0 or max_halvings < 0):
        raise ValueError("invalid pressure retry arguments")
    if converged:
        return "accept"
    if retries >= max_halvings or 0.5 * dt < minimum_dt:
        return "accept_flagged"
    return "halve"

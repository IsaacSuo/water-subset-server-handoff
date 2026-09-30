"""Read-only reference configurations for SPH_Project DFSPH diagnostics."""

import math

import numpy as np


def accumulated_pressure_jacobi_update(pressure, source, applied_operator, alpha, time_scale):
    """One non-negative 0.5-relaxed DFSPH pressure/rho^2 update."""
    values = np.asarray([pressure, source, applied_operator, alpha, time_scale], dtype=np.float64)
    if not np.isfinite(values).all() or alpha < 0.0 or time_scale <= 0.0:
        raise ValueError("invalid accumulated-pressure Jacobi inputs")
    equation_residual = float(source) - float(applied_operator)
    updated = max(float(pressure) - 0.5 * equation_residual * float(alpha) / float(time_scale), 0.0)
    compression_error = -min(equation_residual, 0.0)
    return updated, compression_error


def redistribute_short_frame_tail(remaining, candidate, minimum_dt, minimum_fraction=0.5):
    """Split a candidate and a short frame tail into two equal substeps.

    Density projection velocity corrections scale inversely with ``dt``.  A
    tiny output-frame alignment tail can therefore be less stable than either
    neighboring CFL-limited step even when it remains above the hard minimum.
    """
    values = np.asarray([remaining, candidate, minimum_dt, minimum_fraction], dtype=np.float64)
    if (not np.isfinite(values).all() or remaining <= 0.0 or candidate <= 0.0
            or minimum_dt <= 0.0 or not 0.0 < minimum_fraction <= 1.0):
        raise ValueError("invalid frame-tail redistribution inputs")
    candidate = min(float(candidate), float(remaining))
    tail = float(remaining) - candidate
    if (0.0 < tail < minimum_fraction * candidate
            and remaining >= 2.0 * minimum_dt):
        return 0.5 * float(remaining), True
    return candidate, False


def cubic_kernel_gradient(displacement, support_radius):
    """Match SPH_Project's three-dimensional cubic-spline gradient."""
    vector = np.asarray(displacement, dtype=np.float64)
    h = float(support_radius)
    if not math.isfinite(h) or h <= 0.0:
        raise ValueError("support radius must be finite and positive")
    radius = float(np.linalg.norm(vector))
    q = radius / h
    if radius <= 1.0e-5 or q > 1.0:
        return np.zeros(3, dtype=np.float64)
    factor = q * (3.0 * q - 2.0) if q <= 0.5 else -(1.0 - q) ** 2
    return 48.0 / (math.pi * h**3) * factor * vector / (radius * h)


def filled_lattice_alpha_reference(particle_spacing, support_radius, rest_volume):
    """Evaluate the project's alpha denominator on a filled cubic lattice.

    This reproduces the project's dimensionless-density DFSPH discretization:
    each neighbor contributes ``-V_j grad(W_ij)`` and the denominator is the
    sum of the squared neighbor gradients plus the squared central gradient.
    It is diagnostic metadata only and is never fed back into the solver.
    """
    spacing = float(particle_spacing)
    h = float(support_radius)
    volume = float(rest_volume)
    if not math.isfinite(spacing) or spacing <= 0.0:
        raise ValueError("particle spacing must be finite and positive")
    if not math.isfinite(h) or h <= 0.0:
        raise ValueError("support radius must be finite and positive")
    if not math.isfinite(volume) or volume <= 0.0:
        raise ValueError("rest volume must be finite and positive")

    extent = int(math.ceil(h / spacing))
    gradients = []
    for ix in range(-extent, extent + 1):
        for iy in range(-extent, extent + 1):
            for iz in range(-extent, extent + 1):
                if ix == 0 and iy == 0 and iz == 0:
                    continue
                offset = spacing * np.asarray([ix, iy, iz], dtype=np.float64)
                if float(np.linalg.norm(offset)) < h:
                    gradients.append(-volume * cubic_kernel_gradient(offset, h))
    if not gradients:
        raise ValueError("filled lattice has no neighbors inside the support radius")
    gradients = np.asarray(gradients, dtype=np.float64)
    gradient_sum = gradients.sum(axis=0)
    individual_square_sum = float(np.sum(gradients * gradients))
    denominator = individual_square_sum + float(np.dot(gradient_sum, gradient_sum))
    return {
        "configuration": "filled_interior_cubic_lattice",
        "particle_spacing_m": spacing,
        "support_radius_m": h,
        "rest_volume_m3": volume,
        "neighbor_count": int(len(gradients)),
        "individual_gradient_square_sum_per_m2": individual_square_sum,
        "central_gradient_sum_per_m": gradient_sum.tolist(),
        "denominator_per_m2": denominator,
        "alpha_m2": 1.0 / denominator,
    }

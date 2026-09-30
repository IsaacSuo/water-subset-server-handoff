"""Reference formulas for Akinci et al. 2013 fluid surface tension.

This module deliberately has no Taichi dependency.  It is the scalar source of
truth used by unit tests for the GPU implementation in :mod:`backend`.
"""

import math

import numpy as np


def cohesion_kernel(distance: float, support_radius: float) -> float:
    """Return the paper's three-dimensional cohesion kernel ``C(r)``.

    ``support_radius`` is the compact support radius h.  Coincident particles
    return zero because the associated direction vector is undefined; the GPU
    pair loop applies the same guard.
    """
    r = float(distance)
    h = float(support_radius)
    if not math.isfinite(r) or not math.isfinite(h):
        raise ValueError("distance and support radius must be finite")
    if h <= 0.0:
        raise ValueError("support radius must be positive")
    if r <= 0.0 or r > h:
        return 0.0

    q = r / h
    polynomial = (1.0 - q) ** 3 * q**3
    if q <= 0.5:
        polynomial = 2.0 * polynomial - 1.0 / 64.0
    return 32.0 / (math.pi * h**3) * polynomial


def _cubic_kernel_gradient(displacement: np.ndarray, support_radius: float) -> np.ndarray:
    """Match SPH_Project's three-dimensional cubic-spline gradient."""
    vector = np.asarray(displacement, dtype=np.float64)
    radius = float(np.linalg.norm(vector))
    h = float(support_radius)
    if radius <= 1.0e-5 or radius > h:
        return np.zeros(3, dtype=np.float64)
    q = radius / h
    factor = q * (3.0 * q - 2.0) if q <= 0.5 else -(1.0 - q) ** 2
    return 48.0 / (math.pi * h**3) * factor * vector / (radius * h)


def surface_accelerations(
    positions: np.ndarray,
    masses: np.ndarray,
    densities: np.ndarray,
    support_radius: float,
    coefficient: float,
    fluid_mask: np.ndarray | None = None,
    rest_density: float = 1000.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Small O(N^2) reference for tests, not a production solver.

    Densities are supplied by the caller because production densities include
    the solver's self and boundary contributions.  Only fluid-fluid pairs are
    used for the Akinci normals and forces.
    """
    points = np.asarray(positions, dtype=np.float64)
    particle_masses = np.asarray(masses, dtype=np.float64)
    particle_densities = np.asarray(densities, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("positions must have shape (N, 3)")
    count = len(points)
    if particle_masses.shape != (count,) or particle_densities.shape != (count,):
        raise ValueError("masses and densities must have shape (N,)")
    mask = np.ones(count, dtype=bool) if fluid_mask is None else np.asarray(fluid_mask, dtype=bool)
    if mask.shape != (count,):
        raise ValueError("fluid_mask must have shape (N,)")
    h = float(support_radius)
    k = float(coefficient)
    rho0 = float(rest_density)
    if (not math.isfinite(h) or h <= 0.0 or not math.isfinite(k) or k < 0.0
            or not math.isfinite(rho0) or rho0 <= 0.0):
        raise ValueError("support radius/rest density must be positive and coefficient nonnegative")
    if np.any(~np.isfinite(points)) or np.any(~np.isfinite(particle_masses)) or np.any(~np.isfinite(particle_densities)):
        raise ValueError("particle data must be finite")
    if np.any(particle_masses[mask] <= 0.0) or np.any(particle_densities[mask] <= 0.0):
        raise ValueError("fluid masses and densities must be positive")

    normals = np.zeros_like(points)
    fluid_indices = np.flatnonzero(mask)
    for i in fluid_indices:
        for j in fluid_indices:
            if i != j:
                normals[i] += particle_masses[j] / particle_densities[j] * _cubic_kernel_gradient(
                    points[i] - points[j], h
                )
        normals[i] *= h

    accelerations = np.zeros_like(points)
    for i in fluid_indices:
        for j in fluid_indices:
            if i == j:
                continue
            displacement = points[i] - points[j]
            distance = float(np.linalg.norm(displacement))
            if distance <= 0.0 or distance >= h:
                continue
            density_correction = 2.0 * rho0 / (particle_densities[i] + particle_densities[j])
            cohesion = particle_masses[j] * cohesion_kernel(distance, h) * displacement / distance
            accelerations[i] -= k * density_correction * (cohesion + normals[i] - normals[j])
    return normals, accelerations

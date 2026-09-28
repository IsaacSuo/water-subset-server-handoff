"""Reference math for geometry-aware pressure filtering in thin SPH features.

The GPU implementation lives in :mod:`coupled_scene.gpu_dfsph.backend` so it
can run inside Taichi kernels.  This NumPy version keeps the intended tensor
construction independently testable without importing a CUDA runtime.
"""
import numpy as np


def anisotropic_pressure_tensor(covariance, fluid_neighbor_count, deficiency_limit=20,
                                first_moment=None, weighted_radius_sum=None):
    """Return the normalized covariance tensor used for fluid pressure.

    A well-supported particle is deliberately unchanged.  For a deficient
    particle the covariance is divided by its spectral norm.  Its strongest
    supported direction therefore remains at unit scale while unsupported
    sheet-normal directions approach zero.  Covariance anisotropy activates
    the filter, while a one-sided first moment protects ordinary free surfaces.
    Degenerate neighborhoods return identity because their local geometry is
    not reliable enough to filter.
    """
    covariance=np.asarray(covariance,dtype=np.float64)
    if covariance.shape!=(3,3) or not np.isfinite(covariance).all():
        raise ValueError('Covariance must be a finite 3x3 matrix')
    if fluid_neighbor_count<0 or deficiency_limit<=0:
        raise ValueError('Neighbor counts and the deficiency limit must be valid')
    identity=np.eye(3,dtype=np.float64)
    if fluid_neighbor_count>=deficiency_limit:
        return identity
    covariance=.5*(covariance+covariance.T)
    eigenvalues=np.linalg.eigvalsh(covariance)
    maximum=float(eigenvalues[-1])
    if fluid_neighbor_count<2 or maximum<=np.finfo(np.float32).eps**2:
        return identity
    normalized=covariance/maximum
    anisotropy=float(np.clip(1.-max(0.,float(eigenvalues[0]))/maximum,0.,1.))
    asymmetry=0.
    if first_moment is not None or weighted_radius_sum is not None:
        first_moment=np.asarray(first_moment,dtype=np.float64)
        if first_moment.shape!=(3,) or not np.isfinite(first_moment).all():
            raise ValueError('First moment must be a finite 3-vector')
        if weighted_radius_sum is None or not np.isfinite(weighted_radius_sum) or weighted_radius_sum<0:
            raise ValueError('Weighted radius sum must be finite and nonnegative')
        if weighted_radius_sum>np.finfo(np.float32).eps**2:
            asymmetry=float(np.clip(np.linalg.norm(first_moment)/weighted_radius_sum,0.,1.))
    activation=anisotropy*(1.-asymmetry)
    return (1.-activation)*identity+activation*normalized

"""Bender et al. 2019 volume-map construction for the GPU DFSPH adapter.

The expensive map is generated from the same Newton mesh SDF used for rigid
collision, cached in object-local coordinates, and later sampled by Taichi.
The runtime solver never calls Warp from inside a Taichi kernel.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import numpy as np
import warp as wp
from newton._src.geometry.sdf_texture import TextureSDFData, texture_sample_sdf


VOLUME_MAP_IMPLEMENTATION = "bender2019_volume_map_v1"
REFERENCE_QUADRATURE_ORDER = 16  # SPlisHSPlasH p=30 resolves to 16 points/axis.
REFERENCE_VOLUME_SCALE = 0.8


def cubic_extension(distance, support_radius):
    """Paper equation 11: C(d)/C(0) outside, one inside, zero past h."""
    d = np.asarray(distance, dtype=np.float64)
    h = float(support_radius)
    if not math.isfinite(h) or h <= 0.0:
        raise ValueError("support radius must be finite and positive")
    q = d / h
    value = np.zeros_like(q)
    value[d <= 0.0] = 1.0
    inner = (d > 0.0) & (q <= 0.5)
    value[inner] = 6.0 * q[inner] ** 3 - 6.0 * q[inner] ** 2 + 1.0
    outer = (q > 0.5) & (q < 1.0)
    value[outer] = 2.0 * (1.0 - q[outer]) ** 3
    return float(value) if value.ndim == 0 else value


def sphere_gauss_legendre_quadrature(support_radius, order=REFERENCE_QUADRATURE_ORDER):
    """Tensor-product Gauss points in [-h,h]^3, culled to the support sphere."""
    h = float(support_radius)
    order = int(order)
    if not math.isfinite(h) or h <= 0.0:
        raise ValueError("support radius must be finite and positive")
    if order < 1:
        raise ValueError("quadrature order must be positive")
    abscissae, weights = np.polynomial.legendre.leggauss(order)
    offsets = []
    combined_weights = []
    for ix, x in enumerate(abscissae):
        for iy, y in enumerate(abscissae):
            for iz, z in enumerate(abscissae):
                unit = np.asarray([x, y, z], dtype=np.float64)
                if float(np.dot(unit, unit)) <= 1.0:
                    offsets.append(h * unit)
                    combined_weights.append(h**3 * weights[ix] * weights[iy] * weights[iz])
    return np.asarray(offsets, dtype=np.float32), np.asarray(combined_weights, dtype=np.float32)


def planar_boundary_volume(distance, support_radius, order=REFERENCE_QUADRATURE_ORDER):
    """Reference integral for an infinite planar solid with SDF d(x)=x."""
    offsets, weights = sphere_gauss_legendre_quadrature(support_radius, order)
    return REFERENCE_VOLUME_SCALE * float(
        np.dot(weights.astype(np.float64), cubic_extension(distance + offsets[:, 0], support_radius))
    )


def map_cache_key(vertices, triangles, support_radius, voxel_size, quadrature_order):
    digest = hashlib.sha256()
    digest.update(VOLUME_MAP_IMPLEMENTATION.encode("ascii"))
    digest.update(np.ascontiguousarray(vertices, dtype=np.float32).view(np.uint8))
    digest.update(np.ascontiguousarray(triangles, dtype=np.int32).view(np.uint8))
    digest.update(np.asarray(
        [support_radius, voxel_size, quadrature_order, REFERENCE_VOLUME_SCALE], dtype=np.float64
    ).view(np.uint8))
    return digest.hexdigest()


@wp.kernel
def _build_volume_map_kernel(
    sdf: TextureSDFData,
    lower: wp.vec3,
    voxel_size: float,
    dim_y: int,
    dim_z: int,
    support_radius: float,
    activation_margin: float,
    quadrature_offsets: wp.array(dtype=wp.vec3),
    quadrature_weights: wp.array(dtype=float),
    quadrature_count: int,
    distances: wp.array(dtype=float),
    volumes: wp.array(dtype=float),
):
    linear = wp.tid()
    yz = dim_y * dim_z
    ix = linear // yz
    remainder = linear - ix * yz
    iy = remainder // dim_z
    iz = remainder - iy * dim_z
    point = lower + voxel_size * wp.vec3(float(ix), float(iy), float(iz))
    center_distance = texture_sample_sdf(sdf, point)
    distances[linear] = center_distance
    volume = float(0.0)
    if center_distance > -activation_margin and center_distance < support_radius + activation_margin:
        for sample in range(quadrature_count):
            signed_distance = texture_sample_sdf(sdf, point + quadrature_offsets[sample])
            gamma = float(0.0)
            if signed_distance <= 0.0:
                gamma = 1.0
            elif signed_distance < support_radius:
                q = signed_distance / support_radius
                if q <= 0.5:
                    gamma = 6.0 * q * q * q - 6.0 * q * q + 1.0
                else:
                    one_minus_q = 1.0 - q
                    gamma = 2.0 * one_minus_q * one_minus_q * one_minus_q
            volume += quadrature_weights[sample] * gamma
        volume *= REFERENCE_VOLUME_SCALE
    volumes[linear] = volume


def _grid_geometry(vertices, support_radius, voxel_size):
    vertices = np.asarray(vertices, dtype=np.float64)
    voxel = float(voxel_size)
    padding = float(support_radius) + 2.0 * voxel
    lower = np.floor((vertices.min(axis=0) - padding) / voxel) * voxel
    upper = np.ceil((vertices.max(axis=0) + padding) / voxel) * voxel
    dims = np.rint((upper - lower) / voxel).astype(np.int64) + 1
    if np.any(dims < 2) or np.prod(dims, dtype=np.int64) > np.iinfo(np.int32).max:
        raise ValueError(f"unsupported volume-map dimensions: {dims.tolist()}")
    return lower, dims.astype(np.int32)


def build_or_load_volume_map(
    *,
    object_id,
    vertices,
    triangles,
    texture_sdf,
    support_radius,
    voxel_size,
    cache_dir,
    quadrature_order=REFERENCE_QUADRATURE_ORDER,
    device=None,
):
    """Build one dense, local-coordinate map and return arrays plus metadata."""
    vertices = np.asarray(vertices, dtype=np.float32)
    triangles = np.asarray(triangles, dtype=np.int32)
    support_radius = float(support_radius)
    voxel_size = float(voxel_size)
    key = map_cache_key(vertices, triangles, support_radius, voxel_size, quadrature_order)
    cache_dir = Path(cache_dir)
    cache_dir.mkdir(parents=True, exist_ok=True)
    path = cache_dir / f"{key}.volume_map.npz"
    if path.exists():
        with np.load(path, allow_pickle=False) as cached:
            metadata = json.loads(str(cached["metadata_json"]))
            if metadata["cache_key"] != key or metadata["implementation"] != VOLUME_MAP_IMPLEMENTATION:
                raise ValueError(f"volume-map cache identity mismatch: {path}")
            return dict(
                object_id=int(object_id),
                lower=cached["lower"].astype(np.float32),
                voxel_size=float(cached["voxel_size"]),
                dims=cached["dims"].astype(np.int32),
                distances=cached["distances"].astype(np.float32),
                volumes=cached["volumes"].astype(np.float32),
                metadata=metadata,
                cache_path=str(path),
                cache_hit=True,
            )

    lower, dims = _grid_geometry(vertices, support_radius, voxel_size)
    count = int(np.prod(dims, dtype=np.int64))
    offsets, weights = sphere_gauss_legendre_quadrature(support_radius, quadrature_order)
    with wp.ScopedDevice(device):
        offsets_gpu = wp.array(offsets, dtype=wp.vec3, device=device)
        weights_gpu = wp.array(weights, dtype=float, device=device)
        distances_gpu = wp.empty(count, dtype=float, device=device)
        volumes_gpu = wp.empty(count, dtype=float, device=device)
        wp.launch(
            _build_volume_map_kernel,
            dim=count,
            inputs=[
                texture_sdf,
                wp.vec3(*lower),
                voxel_size,
                int(dims[1]),
                int(dims[2]),
                support_radius,
                math.sqrt(3.0) * voxel_size,
                offsets_gpu,
                weights_gpu,
                len(weights),
                distances_gpu,
                volumes_gpu,
            ],
            device=device,
        )
        wp.synchronize_device(device)
        distances = distances_gpu.numpy().reshape(tuple(dims)).astype(np.float32)
        volumes = volumes_gpu.numpy().reshape(tuple(dims)).astype(np.float32)
    metadata = dict(
        implementation=VOLUME_MAP_IMPLEMENTATION,
        paper="Bender et al. 2019 Volume Maps",
        cache_key=key,
        object_id=int(object_id),
        support_radius_m=support_radius,
        voxel_size_m=voxel_size,
        dimensions=dims.tolist(),
        grid_lower_m=lower.tolist(),
        quadrature="tensor_product_gauss_legendre_culled_to_support_sphere",
        quadrature_order_per_axis=int(quadrature_order),
        quadrature_sample_count=int(len(weights)),
        reference_volume_scale=REFERENCE_VOLUME_SCALE,
        extension="cubic_spline_C(d)/C(0)",
        distance_source="same Newton texture SDF as rigid collision",
    )
    temporary = path.with_name(path.name + ".tmp.npz")
    np.savez_compressed(
        temporary,
        lower=lower.astype(np.float32),
        voxel_size=np.float32(voxel_size),
        dims=dims,
        distances=distances,
        volumes=volumes,
        metadata_json=np.asarray(json.dumps(metadata, sort_keys=True)),
    )
    temporary.replace(path)
    return dict(
        object_id=int(object_id), lower=lower.astype(np.float32), voxel_size=voxel_size,
        dims=dims, distances=distances, volumes=volumes, metadata=metadata,
        cache_path=str(path), cache_hit=False,
    )

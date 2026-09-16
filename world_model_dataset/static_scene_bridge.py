"""Immutable native scene triangles; never passed through COM/convex processing."""
from pathlib import Path

import numpy as np
import trimesh

from .io import file_hash


def resolve_static_scene(geometry, descriptor, initial, root):
    if descriptor['physics_kind'] != 'static':
        raise ValueError('Scene source must be static')
    if initial['position_m'] != [0,0,0] or initial['orientation_xyzw'] != [0,0,0,1]:
        raise ValueError('Scene source keeps original world coordinates')
    source = geometry['static_scene_source']
    path = Path(root)/source['path']
    if file_hash(path) != source['sha256']:
        raise ValueError('Scene mesh hash mismatch')
    with np.load(path, allow_pickle=False) as data:
        v, f = data['vertices'].copy(), data['triangles'].copy()
    if v.ndim != 2 or v.shape[1] != 3 or not np.isfinite(v).all():
        raise ValueError('Invalid scene vertices')
    if f.ndim != 2 or f.shape[1] != 3 or not np.issubdtype(f.dtype,np.integer) or not len(f) or f.min()<0 or f.max()>=len(v):
        raise ValueError('Invalid scene triangles')
    geometry.update(collision_approximation='none', coordinate_frame='original_scene_world_metres',
                    bounds_m=[v.min(0).tolist(),v.max(0).tolist()])
    return trimesh.Trimesh(vertices=v, faces=f, process=False)

"""Read-only triangle geometry for bounded construction rules; never cooks physics."""
from pathlib import Path

import numpy as np
import trimesh
from shapely.geometry import Polygon, box
from shapely.ops import unary_union

from .io import file_hash, read_json


def require(ok, code, detail):
    if not ok:
        raise ValueError(f'{code}: {detail}')


def clip(poly, axis, bound, lower):
    """Clip a 3D polygon to one closed half-space."""
    out = []
    for a, b in zip(poly, np.roll(poly, -1, axis=0)):
        ia, ib = (a[axis] >= bound, b[axis] >= bound) if lower else (a[axis] <= bound, b[axis] <= bound)
        if ia:
            out.append(a)
        if ia != ib:
            out.append(a + (b-a) * ((bound-a[axis])/(b[axis]-a[axis])))
    return np.asarray(out)


class Geometry:
    def __init__(self, scene):
        require((scene['units'], scene['up_axis'], scene['frame']) == ('m', 'Z', 'original_world'),
                'coordinates', 'require original world metres, Z-up; no scene transforms')
        self.bounds = np.asarray(scene['region_bounds_m'], float)
        require(self.bounds.shape == (2, 3) and np.isfinite(self.bounds).all()
                and np.all(self.bounds[1] > self.bounds[0]), 'region', 'invalid bounds')
        self.meshes, self.pins, self.solids = {}, {}, []
        if 'scene_export' in scene['collision']:
            path = Path(scene['collision']['scene_export']); self.pin(path)
            info = read_json(path)
            require(info['metres_per_unit'] == 1 and 'Z up' in info['coordinate_frame'],
                    'coordinates', 'scene export is not original metre/Z-up')
            specs = [dict(id=k, path=str(path.parent/v['path']), sha256=v['sha256'])
                     for k, v in info['groups'].items()]
        else:
            specs = scene['collision']['meshes']
        for item in specs:
            require(not {'scale', 'world_from_mesh'} & item.keys(), 'scene_transform', 'original mesh required')
            path = Path(item['path']); checksum = self.pin(path)
            require(item.get('sha256', checksum) == checksum, 'source_hash', str(path))
            with np.load(path, allow_pickle=False) as z:
                v, f = z['vertices'], z['triangles']
            require(v.ndim == 2 and v.shape[1] == 3 and np.isfinite(v).all()
                    and f.ndim == 2 and f.shape[1] == 3 and np.issubdtype(f.dtype, np.integer)
                    and len(f) and f.min() >= 0 and f.max() < len(v), 'mesh', str(path))
            require(item['id'] not in self.meshes, 'mesh_id', 'duplicate scene mesh')
            self.meshes[item['id']] = v[f].astype(float)
            mesh=trimesh.Trimesh(v,f,process=False)
            if mesh.is_watertight:
                self.solids.append(mesh)
        for path in scene['source_records'].values():
            self.pin(Path(path))

    def pin(self, path):
        checksum = file_hash(path); self.pins[str(Path(path).resolve())] = checksum
        return checksum

    def support(self, name):
        require(name in self.meshes, 'support', 'unknown selected mesh: '+name)
        tri = self.meshes[name]
        lo, hi = self.bounds
        # Blender float32 world exports can differ by a few micrometres across
        # a nominally horizontal room floor; never flatten the backend mesh.
        tolerance = 5e-6
        mask = (np.ptp(tri[:, :, 2], axis=1) < tolerance)
        mask &= (tri[:, :, 2].mean(1) >= lo[2]) & (tri[:, :, 2].mean(1) < hi[2])
        mask &= np.all(tri.max(1)[:, :2] > lo[:2], axis=1) & np.all(tri.min(1)[:, :2] < hi[:2], axis=1)
        require(mask.any(), 'support', 'no horizontal support triangles in selected region')
        height = float(tri[mask, :, 2].max())
        planar = tri[np.max(abs(tri[:, :, 2]-height), axis=1) < tolerance]
        surface = unary_union([Polygon(t[:, :2]) for t in planar])
        require(surface.area > 0, 'support', 'degenerate surface')
        return height, surface

    def projection(self, low_z, high_z):
        """Conservative triangle projection in a Z slab, preserving holes in XY."""
        polygons = []
        lo, hi = self.bounds
        for triangles in self.meshes.values():
            mask = (triangles.max(1)[:, 2] >= low_z) & (triangles.min(1)[:, 2] <= high_z)
            mask &= np.all(triangles.max(1)[:, :2] >= lo[:2], axis=1) & np.all(triangles.min(1)[:, :2] <= hi[:2], axis=1)
            for tri in triangles[mask]:
                p = clip(tri, 2, low_z, True)
                if len(p): p = clip(p, 2, high_z, False)
                if len(p) >= 3:
                    shape = Polygon(p[:, :2]).convex_hull
                    if not shape.is_empty: polygons.append(shape)
        return unary_union(polygons)

    def check_box(self, low, high, code='initial_clearance'):
        low, high = np.asarray(low), np.asarray(high)
        require(np.all(low >= self.bounds[0]-1e-7) and np.all(high <= self.bounds[1]+1e-7),
                'region_fit', 'object or reserved motion envelope exceeds selected region')
        footprint = box(*low[:2], *high[:2])
        obstacles = self.projection(low[2]+1e-7, high[2]-1e-7)
        require(not footprint.intersects(obstacles), code,
                'conservative object envelope intersects original scene triangles')
        center=(low+high)/2
        for mesh in self.solids:
            if np.all(center>mesh.bounds[0]) and np.all(center<mesh.bounds[1]):
                require(not mesh.contains([center])[0],code,'envelope is inside a closed original scene mesh')

    def edge_transition(self, edge, y_low, y_high, top, depth, free_length):
        """Separate a bounded rounded-edge contact band from clear hanging space.

        Original desks have bevels/frames ~1 cm beyond their horizontal top.
        Contact in this band is allowed and explicitly NOT a clearance pass.
        A disconnected obstacle or a deep ledge must still fail the prism test.
        """
        slab=self.projection(top-depth, top-1e-5).intersection(
            box(edge-1e-5,y_low,edge+free_length,y_high))
        parts=list(slab.geoms) if hasattr(slab,'geoms') else [slab]
        adjacent=[p for p in parts if not p.is_empty and p.bounds[0]<=edge+1e-5]
        end=max([edge]+[p.bounds[2] for p in adjacent])
        require(end-edge <= min(.025,free_length*.25), 'edge_transition',
                'original support/frame extends too far into the requested hanging region')
        return end+1e-5


def rectangle(center, size):
    p, s = np.asarray(center), np.asarray(size)/2
    return box(*(p-s), *(p+s))

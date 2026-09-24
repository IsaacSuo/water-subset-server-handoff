"""Checked ABI for the local SPlisHSPlasH bridge; no Isaac application required."""
import ctypes as C
import json
from pathlib import Path
import numpy as np


class ReferenceBridge:
    def __init__(self, dll):
        self.lib = C.CDLL(str(Path(dll).resolve()))
        f = np.ctypeslib.ndpointer(dtype=np.float32, flags='C_CONTIGUOUS')
        u = np.ctypeslib.ndpointer(dtype=np.uint32, flags='C_CONTIGUOUS')
        signatures = {
            'bridge_init': [C.c_char_p, C.c_char_p, C.c_char_p, C.c_int],
            'bridge_step': [f, C.c_int, C.c_float, f],
            'bridge_particles': [f, f, u, C.c_int],
            'bridge_pose': [C.c_int, f],
            'bridge_write_particles': [C.c_char_p, f, f, C.c_int, C.c_float],
            'bridge_close': [], 'bridge_version': [], 'bridge_body_count': [],
        }
        for name, args in signatures.items():
            getattr(self.lib, name).argtypes = args
            getattr(self.lib, name).restype = C.c_int
        self.lib.bridge_error.restype = C.c_char_p
        if self.lib.bridge_version() != 2:
            raise RuntimeError('Bridge ABI mismatch; rebuild native library')
        self.count = None

    def check(self, result):
        if result < 0:
            raise RuntimeError(self.lib.bridge_error().decode('utf-8', errors='replace'))
        return result

    def write_particles(self, path, positions, velocities, radius):
        p, v = (np.ascontiguousarray(a, dtype=np.float32) for a in (positions, velocities))
        if p.ndim != 2 or p.shape[1] != 3 or v.shape != p.shape:
            raise ValueError('Expected matching Nx3 particle arrays')
        self.check(self.lib.bridge_write_particles(str(Path(path).resolve()).encode(), p, v, len(p), radius))

    def initialize(self, scene, output, executable, threads=8):
        scene = Path(scene).resolve()
        if not scene.is_file(): raise FileNotFoundError(scene)
        spec=json.loads(scene.read_text(encoding='utf-8'))
        config=spec['Configuration']
        if config.get('boundaryHandlingMethod')!=2 or config.get('simulationMethod')!=4 or config.get('cflMethod')!=0:
            raise ValueError('This adapter requires fixed-step DFSPH with Bender volume-map boundaries')
        if spec.get('AnimationFields') or spec.get('Emitters'):
            raise ValueError('Fluid animation/emission is outside the water migration contract')
        for item,key in [(r,'geometryFile') for r in spec.get('RigidBodies',[])]+[(f,'particleFile') for f in spec.get('FluidModels',[])]:
            asset=Path(item[key]);asset=asset if asset.is_absolute() else scene.parent/asset
            if not asset.is_file(): raise FileNotFoundError(asset)
        self.count = self.check(self.lib.bridge_init(str(scene).encode(), str(Path(output).resolve()).encode(), str(Path(executable).resolve()).encode(), threads))
        self.bodies = self.lib.bridge_body_count()
        self.positions = np.empty((self.count, 3), np.float32)
        self.velocities = np.empty_like(self.positions)
        self.ids = np.empty(self.count, np.uint32)
        return self.count

    def step(self, states, dt):
        states = np.ascontiguousarray(states, dtype=np.float32)
        if states.shape != (self.bodies, 13):
            raise ValueError('One 13-component geometry-origin state per dynamic boundary')
        wrench = np.empty((self.bodies, 6), np.float32)
        self.check(self.lib.bridge_step(states, self.bodies, dt, wrench))
        return wrench

    def particles(self):
        n = self.check(self.lib.bridge_particles(self.positions, self.velocities, self.ids, self.count))
        if n != self.count:
            raise RuntimeError('Emission/deletion is not supported by this adapter')
        if not np.isfinite(self.positions).all() or not np.isfinite(self.velocities).all():
            raise RuntimeError('Nonfinite fluid state')
        return self.positions, self.velocities, self.ids

    def pose(self, index):
        pose = np.empty(7, np.float32)
        self.check(self.lib.bridge_pose(index, pose))
        return pose

    def close(self):
        self.check(self.lib.bridge_close())

"""Exact authored-mesh OpenVDB level sets supplied to Mantaflow at each substep.

Blender 5.0's default obstacle builder casts 26 rays per voxel.  This adapter
uses OpenVDB mesh-to-level-set instead, without offsets or simplified meshes.
The pressure solver, particles, advection, meshing and cache remain Mantaflow.
"""
import ctypes
import builtins
import gc
import math
import time
import types

import bpy
import numpy as np
import openvdb
from mathutils import Quaternion, Vector

from coupled_scene.active_drive import motion_state


class MeshSDFCollisions:
    def __init__(self, domain, objects, moving, motion, pivot_native, first, fps):
        if bpy.app.version != (5, 0, 1):
            raise RuntimeError('This Mantaflow grid adapter is verified for Blender 5.0.1 only')
        self.domain = domain
        self.motion = motion
        self.first = first
        self.fps = fps
        self.pivot = np.array([pivot_native[0], -pivot_native[2], pivot_native[1]])
        corners = np.array([domain.matrix_world @ v.co for v in domain.data.vertices])
        self.lo, self.extent = corners.min(0), np.ptp(corners, axis=0)
        self.geometry = []
        self.effectors = []
        for obj in objects:
            settings = next(m.effector_settings for m in obj.modifiers if m.type == 'FLUID')
            self.effectors.append((settings, settings.use_effector))
        depsgraph = bpy.context.evaluated_depsgraph_get()
        for obj in objects:
            evaluated = obj.evaluated_get(depsgraph)
            mesh = evaluated.to_mesh(); mesh.calc_loop_triangles()
            points = np.array([obj.matrix_world @ v.co for v in mesh.vertices], np.float32)
            triangles = np.array([t.vertices for t in mesh.loop_triangles], np.uint32)
            self.geometry.append((obj == moving, points, triangles))
            evaluated.to_mesh_clear()
        # Enables Mantaflow's obstacle fields; its tiny map is replaced completely.
        size = max(self.extent) / domain.modifiers['Fluid'].domain_settings.resolution_max
        bpy.ops.mesh.primitive_cube_add(size=size*.1, location=self.lo+self.extent-size*2)
        self.marker = bpy.context.object
        self.marker.name = 'SDFFieldAllocationMarker'; self.marker.hide_render = True
        modifier = self.marker.modifiers.new('Fluid', 'FLUID'); modifier.fluid_type = 'EFFECTOR'
        self.allocation_collection = bpy.data.collections.new('SDF field allocation')
        bpy.context.scene.collection.children.link(self.allocation_collection)
        self.allocation_collection.objects.link(self.marker)
        domain_settings = domain.modifiers['Fluid'].domain_settings
        self.original_effector_group = domain_settings.effector_group
        domain_settings.effector_group = self.allocation_collection
        self.dims = None
        self.static = None
        self.last_frame = None
        self.steps = 0
        self.seconds = 0.
        self.error = None
        self.namespaces = []
        self.original_import = None

    def _view(self, grid):
        size = grid.getSize()
        dims = (int(size.x), int(size.y), int(size.z))
        if dims != self.dims:
            raise RuntimeError(f'Unexpected Mantaflow grid shape: {dims}, expected {self.dims}')
        pointer = int(grid.getDataPointer(), 16)
        if not pointer or not (grid.getGridType() & 1):
            raise RuntimeError('Expected a dense Mantaflow float32 scalar grid')
        buffer = (ctypes.c_float * math.prod(dims)).from_address(pointer)
        return np.ctypeslib.as_array(buffer).reshape(dims, order='F')

    def _sdf(self, points, triangles):
        points = np.asarray((points-self.lo)/self.cell-.5, dtype=np.float32)
        # halfWidth only controls the distance band stored around the surface;
        # the zero isosurface is unchanged. This is NOT a collision offset.
        grid = openvdb.FloatGrid.createLevelSetFromPolygons(points, triangles=triangles, halfWidth=4.)
        result = np.empty(self.dims, np.float32)
        grid.copyToArray(result)
        return result

    def _update(self, namespace, suffix):
        started = time.perf_counter()
        solver = namespace['s'+suffix]
        if self.dims is None:
            size = solver.getGridSize()
            self.dims = (int(size.x), int(size.y), int(size.z))
            self.cell = self.extent / np.array(self.dims)
            self.static = np.full(self.dims, 9999., np.float32)
            for moving, points, triangles in self.geometry:
                if not moving:
                    np.minimum(self.static, self._sdf(points, triangles), out=self.static)
        frame = solver.frame
        portion = (solver.timePerFrame+solver.timestep)/solver.frameLength
        seconds = (frame-1)/self.fps if frame == self.first else (frame-2+portion)/self.fps
        state = motion_state(self.motion, seconds)
        # Native axis 2 maps to negative Blender Y.
        axis = np.zeros(3); axis[state['rotation_axis']] = 1.
        axis = Vector((axis[0], -axis[2], axis[1]))
        rotation = np.array(Quaternion(axis, state['angle_rad']).to_matrix())
        displacement = state['displacement_m']
        displacement = np.array([displacement[0], -displacement[2], displacement[1]])
        moving_geometry = next(g for g in self.geometry if g[0])
        world = (moving_geometry[1]-self.pivot) @ rotation.T + self.pivot + displacement
        phi = self._sdf(world, moving_geometry[2])
        stem = '_s'+suffix
        if not namespace['using_obstacle'+stem]:
            raise RuntimeError('Mantaflow obstacle fields are not enabled')
        np.copyto(self._view(namespace['phiObsIn'+stem]), phi)
        np.copyto(self._view(namespace['phiObsSIn'+stem]), self.static)
        mask = phi < 1.732
        self._view(namespace['numObs'+stem])[:] = mask
        # Analytic rigid-body velocity, converted from m/s to cells/Manta time.
        omega = state['angular_velocity_rad_s']
        omega = np.array([omega[0], -omega[2], omega[1]])
        linear = state['linear_velocity_m_s']
        linear = np.array([linear[0], -linear[2], linear[1]])
        coordinates = [(np.arange(n, dtype=np.float32)+.5)*self.cell[i]+self.lo[i]-self.pivot[i]-displacement[i]
                       for i, n in enumerate(self.dims)]
        x = coordinates[0][:, None, None]
        y = coordinates[1][None, :, None]
        z = coordinates[2][None, None, :]
        velocities = [linear[0]+omega[1]*z-omega[2]*y,
                      linear[1]+omega[2]*x-omega[0]*z,
                      linear[2]+omega[0]*y-omega[1]*x]
        scale = namespace['ratioBTimeToTimestep'+stem]
        for i, axis_name in enumerate('xyz'):
            target = self._view(namespace[axis_name+'_obvel'+stem])
            target[:] = np.where(mask, velocities[i]*scale/self.cell[i], 0.)
        self.steps += 1
        self.seconds += time.perf_counter()-started
        if frame != self.last_frame:
            print(f'EXACT_SDF frame={frame} t={seconds:.5f} cell_mm={max(self.cell)*1000:.4f}', flush=True)
            self.last_frame = frame

    def _patch_steps(self, namespace):
        for name, function in list(namespace.items()):
            if not name.startswith('fluid_pre_step_') or getattr(function, '_sdf_adapter', None) is self:
                continue
            suffix = name.rsplit('_', 1)[-1]
            def step(original=function, suffix=suffix):
                try:
                    self._update(namespace, suffix)
                except Exception as error:
                    self.error = error
                    raise
                return original()
            step._sdf_adapter = self
            step._sdf_original = function
            namespace[name] = step

    def _patch_messages(self, namespace):
        original = namespace.get('mantaMsg')
        if original is None or getattr(original, '_sdf_adapter', None) is self:
            return
        def message(*args, **kwargs):
            # Called before fluid_pre_step, also when Blender initializes a new
            # solver on its native bake thread. Avoid thread-local profilers.
            self._patch_steps(namespace)
            return original(*args, **kwargs)
        message._sdf_adapter = self
        message._sdf_original = original
        namespace['mantaMsg'] = message
        if not any(n is namespace for n in self.namespaces):
            self.namespaces.append(namespace)
        self._patch_steps(namespace)

    def install(self):
        self.original_import = builtins.__import__
        def importing(name, globals=None, locals=None, fromlist=(), level=0):
            result = self.original_import(name, globals, locals, fromlist, level)
            if globals and globals.get('__file__') == '<manta_namespace>':
                self._patch_messages(globals)
            return result
        builtins.__import__ = importing
        for module in gc.get_objects():
            if isinstance(module, types.ModuleType) and getattr(module, '__file__', '') == '<manta_namespace>':
                self._patch_messages(vars(module))

    def finish(self):
        builtins.__import__ = self.original_import
        for namespace in self.namespaces:
            for name, value in list(namespace.items()):
                if getattr(value, '_sdf_adapter', None) is self:
                    namespace[name] = value._sdf_original
        self.namespaces.clear()
        for settings, enabled in self.effectors:
            settings.use_effector = enabled
        self.domain.modifiers['Fluid'].domain_settings.effector_group = self.original_effector_group
        bpy.data.objects.remove(self.marker, do_unlink=True)
        bpy.data.collections.remove(self.allocation_collection)
        self.effectors.clear()
        self.geometry.clear()
        self.static = None
        if self.error is not None:
            raise RuntimeError('OpenVDB collision adapter failed') from self.error
        if not self.steps:
            raise RuntimeError('No OpenVDB collision steps were executed')
        return dict(backend='OpenVDB exact authored mesh SDF', substeps=self.steps,
                    sdf_seconds=self.seconds, dimensions=self.dims)

"""Blender: bake the pitcher pour with Mantaflow (FLIP) in the original design scene.

The pitcher follows the same analytic motion as the SPH runs (one-way
coupling: the pitcher, basin and countertop are collision effectors), and
starts filled with the same water, taken as the convex hull of the pour
assets' fill.  Optional overrides change the tilt duration and pitcher
position.  Bakes to the output folder, reports per-frame bake time, and
renders selected frames with the design camera and water material.

    blender --background -t 8 --python experiments/coupled_scenes/mantaflow_pour.py -- \
        --blend .../01_container_transfer.blend --assets .../assets_dir \
        --output out --resolution 384 --mesh-scale 1 --collision-backend openvdb \
        --timesteps-min 6 --timesteps-max 32 --particle-band-width 30 \
        --start-seconds 1.8 --end-seconds 3.8

The OpenVDB collision backend voxelizes the original evaluated triangle meshes
at every solver substep, with no surface offset or replacement collision hull.
Run final rendering on Windows with OptiX after the simulation checks pass.
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import bmesh
import bpy
import numpy as np
from mathutils import Quaternion, Vector

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from coupled_scene.active_drive import motion_state
from experiments.coupled_scenes.blender_coupled_event_overlay import _water_material
from experiments.coupled_scenes.blender_server_assets import reload_hdri

FPS = 30


def native_to_blender(p):
    return Vector((p[0], -p[2], p[1]))


def frame_of(seconds):
    return int(round(seconds * FPS)) + 1


def keyframe_pitcher(pitcher, motion, pivot, first, last):
    """Write one key per frame from the analytic motion (native Y-up, rotation about native z)."""
    pitcher.animation_data_clear()
    pitcher.rotation_mode = 'QUATERNION'
    bpy.context.preferences.edit.keyframe_new_interpolation_type = 'LINEAR'
    parent_inverse = pitcher.parent.matrix_world.inverted()
    for frame in range(first, last + 1):
        state = motion_state(motion, (frame - 1) / FPS)
        axis = [0., 0., 0.]
        axis[state['rotation_axis']] = state['angle_rad']
        angle = math.sqrt(sum(a * a for a in axis))
        # Native quaternion (x, y, z, w) of the rotation vector, then the render convention.
        if angle > 0:
            s = math.sin(angle / 2) / angle
            qx, qy, qz, qw = axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2)
        else:
            qx = qy = qz = 0.; qw = 1.
        position = [pivot[i] + state['displacement_m'][i] for i in range(3)]
        pitcher.location = parent_inverse @ native_to_blender(position)
        pitcher.rotation_quaternion = Quaternion((qw, qx, -qz, qy))
        pitcher.keyframe_insert('location', frame=frame)
        pitcher.keyframe_insert('rotation_quaternion', frame=frame)


def fill_object(positions_native):
    """Convex hull of the fill particle centres, in Blender coordinates."""
    mesh = bpy.data.meshes.new('PourFill')
    bm = bmesh.new()
    for p in positions_native:
        bm.verts.new(native_to_blender(p))
    bmesh.ops.convex_hull(bm, input=bm.verts[:])
    loose = [v for v in bm.verts if not v.link_faces]
    bmesh.ops.delete(bm, geom=loose, context='VERTS')
    bm.to_mesh(mesh); volume = bm.calc_volume(); bm.free()
    obj = bpy.data.objects.new('PourFill', mesh)
    bpy.context.scene.collection.objects.link(obj)
    return obj, volume


def add_fluid(obj, kind):
    modifier = obj.modifiers.new('Fluid', 'FLUID')
    modifier.fluid_type = kind
    return modifier


def validate_cache(scene, domain, pitcher_mesh, first, last, motion, cell_size):
    """Check containment and solver liquid volume, keeping mesh volume diagnostic.

    The particle mesher's surface radius biases the volume of thin streams and
    droplets. Use the resumable liquid level set for the volume regression.
    """
    import openvdb
    cache = Path(bpy.path.abspath(domain.modifiers['Fluid'].domain_settings.cache_directory))
    corners = np.array([domain.matrix_world @ v.co for v in domain.data.vertices])
    extent = np.ptp(corners, axis=0)
    rows = []
    for frame in range(first, last + 1):
        scene.frame_set(frame)
        evaluated = domain.evaluated_get(bpy.context.evaluated_depsgraph_get())
        mesh = evaluated.to_mesh()
        bm = bmesh.new(); bm.from_mesh(mesh); bm.transform(domain.matrix_world)
        if not bm.verts:
            raise RuntimeError(f'Empty liquid cache at frame {frame}')
        z_min = min(v.co.z for v in bm.verts)
        seconds = (frame - 1) / FPS
        row = dict(frame=frame, seconds=seconds, mesh_volume_liters=bm.calc_volume() * 1000.,
                   min_z_m=z_min, mesh_vertices=len(bm.verts))
        data_file = str(cache / 'data' / f'fluid_data_{frame:04d}.vdb')
        phi = openvdb.read(data_file, 'phi')
        dimensions = phi.metadata['file_base_resolution']
        occupancy = np.empty(dimensions, np.float32); phi.copyToArray(occupancy)
        np.subtract(.5, occupancy, out=occupancy); np.clip(occupancy, 0, 1, out=occupancy)
        flags_grid = openvdb.read(data_file, 'flags')
        flags = np.empty(dimensions, np.int32); flags_grid.copyToArray(flags)
        occupancy[(flags & 2) != 0] = 0
        row['solver_volume_liters'] = float(occupancy.sum(dtype=np.float64) * np.prod(extent / dimensions) * 1000.)
        if seconds <= motion['start_s']:
            bottom = min((pitcher_mesh.matrix_world @ v.co).z for v in pitcher_mesh.data.vertices)
            row['vertices_below_pitcher_bottom'] = sum(v.co.z < bottom - cell_size * .25 for v in bm.verts)
        rows.append(row)
        bm.free(); evaluated.to_mesh_clear()
        if frame == first or frame == last or (frame-first) % 10 == 0:
            print(f"VALIDATE frame={frame} solver_liters={row['solver_volume_liters']:.6f}", flush=True)
    hold = [r for r in rows if 'vertices_below_pitcher_bottom' in r]
    initial = hold[0]['mesh_volume_liters']
    drift = max(abs(r['mesh_volume_liters'] / initial - 1) for r in hold)
    solver_initial = hold[0]['solver_volume_liters']
    solver_drift = max(abs(r['solver_volume_liters'] / solver_initial - 1) for r in hold)
    passed = all(r['vertices_below_pitcher_bottom'] == 0 for r in hold) and drift < .1 and solver_drift < .1
    volume_ratio = [r['mesh_volume_liters'] / initial for r in rows]
    solver_ratio = [r['solver_volume_liters'] / solver_initial for r in rows]
    return dict(pre_motion_passed=passed, pre_motion_volume_max_relative_drift=drift,
                pre_motion_solver_volume_max_relative_drift=solver_drift,
                full_mesh_volume_ratio_range=[min(volume_ratio), max(volume_ratio)],
                full_solver_volume_ratio_range=[min(solver_ratio), max(solver_ratio)],
                volume_measurement='Integrated liquid level set voxel occupancy; obstacle cells excluded',
                full_volume_passed=min(solver_ratio) >= .85 and max(solver_ratio) <= 1.2,
                frames=rows)


def configure_optix(scene):
    preferences = bpy.context.preferences.addons['cycles'].preferences
    preferences.compute_device_type = 'OPTIX'
    preferences.get_devices()
    devices = [d for d in preferences.devices if d.type == 'OPTIX']
    if not devices:
        raise RuntimeError('No OptiX device available; refusing CPU fallback')
    for device in preferences.devices:
        device.use = device.type == 'OPTIX'
    scene.render.engine = 'CYCLES'; scene.cycles.device = 'GPU'
    return [d.name for d in devices]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--blend', type=Path, required=True)
    parser.add_argument('--assets', type=Path, required=True, help='Pour assets: fill positions and authored motion')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--hdri', type=Path)
    parser.add_argument('--resolution', type=int, default=384)
    parser.add_argument('--mesh-scale', type=int, default=1, help='Mantaflow mesh upres factor')
    parser.add_argument('--start-seconds', type=float, default=1.8)
    parser.add_argument('--end-seconds', type=float, default=3.8)
    parser.add_argument('--tilt-end-seconds', type=float, help='Override the authored tilt end (slower pour)')
    parser.add_argument('--hold-end-seconds', type=float)
    parser.add_argument('--stop-seconds', type=float)
    parser.add_argument('--pitcher-offset', type=float, nargs=3, default=(0., 0., 0.), metavar=('X', 'Y', 'Z'),
        help='Native (Y-up) offset of the pitcher pivot and its water, in metres')
    parser.add_argument('--domain-min', type=float, nargs=3, default=(-.62, .19, -8.42), help='Native domain corner')
    parser.add_argument('--domain-max', type=float, nargs=3, default=(.62, .98, -7.73), help='Native domain corner')
    parser.add_argument('--render-seconds', type=float, nargs='*', default=())
    parser.add_argument('--samples', type=int, default=32)
    parser.add_argument('--collision-thickness', type=float, default=0., help='Effector margin in voxels; keep zero for authored geometry')
    parser.add_argument('--fractional-obstacles', action='store_true', help='Enable fractional obstacle boundaries; off for the repaired thin-wall setup')
    parser.add_argument('--timesteps-min', type=int, default=6)
    parser.add_argument('--timesteps-max', type=int, default=32)
    parser.add_argument('--cfl', type=float, default=1.)
    parser.add_argument('--effector-subframes', type=int, default=0,
                        help='Extra swept samples per solver step; mesh already updates every solver step')
    parser.add_argument('--particle-band-width', type=float, default=30.,
                        help='Track particles through the vessel bulk, not only a 3-cell surface band')
    parser.add_argument('--collision-backend', choices=['native', 'openvdb'], default='openvdb')
    args = parser.parse_args(sys.argv[sys.argv.index('--') + 1:])
    if args.collision_backend == 'openvdb' and (args.collision_thickness != 0 or args.effector_subframes != 0):
        parser.error('OpenVDB uses the original surface at every solver substep; use zero collision thickness and effector subframes')
    args.output = args.output.resolve(); args.output.mkdir(parents=True, exist_ok=False)
    meta = json.loads((args.assets / 'assets.json').read_text(encoding='utf-8'))
    with np.load(args.assets / 'geometry_and_fill.npz') as data:
        fill = data['positions'].astype(np.float64)
    motion = dict(meta['case']['motion'])
    for key, value in (('tilt_end_s', args.tilt_end_seconds), ('hold_end_s', args.hold_end_seconds),
                       ('stop_s', args.stop_seconds)):
        if value is not None:
            motion[key] = value
    offset = np.asarray(args.pitcher_offset)
    pivot = np.asarray(meta['donor']['position_m']) + offset
    if not motion['start_s'] > args.start_seconds:
        raise ValueError('The bake must start before the pitcher moves')

    bpy.ops.wm.open_mainfile(filepath=str(args.blend.resolve()))
    scene = bpy.context.scene
    scene.render.fps = FPS; scene.render.fps_base = 1.
    first, last = frame_of(args.start_seconds), frame_of(args.end_seconds)
    scene.frame_start, scene.frame_end = first, last
    pitcher = bpy.data.objects['PouringPitcher']
    keyframe_pitcher(pitcher, motion, pivot, first, last)
    scene.frame_set(first)

    water, hull_volume = fill_object(fill + offset)
    flow = add_fluid(water, 'FLOW').flow_settings
    flow.flow_type = 'LIQUID'; flow.flow_behavior = 'GEOMETRY'; flow.flow_source = 'MESH'
    water.hide_render = True
    for name in ('Spouted ceramic pitcher', 'Low oval receiving basin', 'Honed stone countertop'):
        effector = add_fluid(bpy.data.objects[name], 'EFFECTOR').effector_settings
        effector.effector_type = 'COLLISION'; effector.use_effector = True
        # Use the authored closed mesh directly. Its wet sidewall is only
        # about 7.7-8.1 mm thick, so the original 9.7 mm cells under-resolve it.
        # The margin remains zero; resolution supplies collision detail.
        effector.surface_distance = args.collision_thickness
        if name == 'Spouted ceramic pitcher':
            effector.subframes = args.effector_subframes

    lower = native_to_blender(args.domain_min); upper = native_to_blender(args.domain_max)
    corner_min = Vector(tuple(min(a, b) for a, b in zip(lower, upper)))
    corner_max = Vector(tuple(max(a, b) for a, b in zip(lower, upper)))
    bpy.ops.mesh.primitive_cube_add(size=1., location=(corner_min + corner_max) / 2)
    domain = bpy.context.object; domain.name = 'PourDomain'
    domain.scale = corner_max - corner_min
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    settings = add_fluid(domain, 'DOMAIN').domain_settings
    settings.domain_type = 'LIQUID'
    settings.resolution_max = args.resolution
    settings.use_fractions = args.fractional_obstacles
    settings.timesteps_min = args.timesteps_min
    settings.timesteps_max = args.timesteps_max
    settings.cfl_condition = args.cfl
    settings.particle_band_width = args.particle_band_width
    settings.use_mesh = True
    settings.mesh_scale = args.mesh_scale
    settings.cache_type = 'ALL'
    settings.cache_resumable = True
    settings.cache_directory = str(args.output / 'cache')
    settings.cache_frame_start, settings.cache_frame_end = first, last
    domain.data.materials.append(_water_material())

    report = dict(product='mantaflow_pour', blend=str(args.blend.resolve()), assets=str(args.assets.resolve()),
        resolution=args.resolution, mesh_scale=args.mesh_scale, cell_size_m=float(max(corner_max - corner_min)) / args.resolution,
        motion=motion, pitcher_offset_m=offset.tolist(), start_seconds=args.start_seconds, end_seconds=args.end_seconds,
        frames=[first, last], fill_hull_volume_liters=hull_volume * 1000.,
        fill_particle_water_liters=len(fill) * .8 * float(meta['particle_spacing_m']) ** 3 * 1000.,
        threads=scene.render.threads, bake_complete=False)
    report.update(fractional_obstacles=settings.use_fractions,
        blender_version=bpy.app.version_string, collision_backend_requested=args.collision_backend,
        domain_min_blender_m=list(corner_min), domain_extent_m=list(corner_max-corner_min),
        collision_thickness_voxels=args.collision_thickness,
        timesteps=[settings.timesteps_min, settings.timesteps_max], cfl=settings.cfl_condition,
        effector_subframes=args.effector_subframes, particle_band_width=settings.particle_band_width)
    (args.output / 'report.json').write_text(json.dumps(report, indent=2))
    bpy.ops.wm.save_as_mainfile(filepath=str(args.output / 'mantaflow_pour.blend'))

    collision_adapter = None
    if args.collision_backend == 'openvdb':
        from experiments.coupled_scenes.mantaflow_vdb_collision import MeshSDFCollisions
        collision_adapter = MeshSDFCollisions(domain,
            [bpy.data.objects[n] for n in ('Spouted ceramic pitcher', 'Low oval receiving basin', 'Honed stone countertop')],
            bpy.data.objects['Spouted ceramic pitcher'], motion, pivot, first, FPS)
        collision_adapter.install()
    for obj in scene.objects:
        obj.select_set(obj == domain)
    bpy.context.view_layer.objects.active = domain
    started = time.perf_counter()
    try:
        with bpy.context.temp_override(object=domain, active_object=domain, selected_objects=[domain]):
            result = bpy.ops.fluid.bake_all()
    finally:
        if collision_adapter is not None:
            report['collision_backend'] = collision_adapter.finish()
    report.update(bake_result=list(result), bake_seconds=time.perf_counter() - started)
    data_files = sorted((args.output / 'cache' / 'data').glob('*'))
    stamps = sorted({f.stat().st_mtime for f in data_files})
    report.update(bake_complete='FINISHED' in result, cache_files=len(data_files),
        seconds_per_frame=report['bake_seconds'] / max(1, last - first + 1))
    (args.output / 'report.json').write_text(json.dumps(report, indent=2))
    bpy.ops.wm.save_mainfile()

    validation = validate_cache(scene, domain, bpy.data.objects['Spouted ceramic pitcher'],
                                first, last, motion, report['cell_size_m'])
    (args.output / 'validation.json').write_text(json.dumps(validation, indent=2))
    report['pre_motion_passed'] = validation['pre_motion_passed']
    (args.output / 'report.json').write_text(json.dumps(report, indent=2))
    if not validation['pre_motion_passed'] or not validation['full_volume_passed']:
        raise RuntimeError('Leak/volume regression failed; see validation.json')

    if args.render_seconds:
        if args.hdri:
            reload_hdri(args.hdri)
        scene.camera = bpy.data.objects['DesignView_00']
        scene.render.resolution_x = 1280; scene.render.resolution_y = 960; scene.render.resolution_percentage = 100
        scene.cycles.samples = args.samples; scene.cycles.seed = 0
        report['optix_devices'] = configure_optix(scene)
        scene.render.image_settings.file_format = 'PNG'
        report['renders'] = []
        for seconds in args.render_seconds:
            scene.frame_set(frame_of(seconds))
            scene.render.filepath = str(args.output / f'render_{seconds:.3f}s.png')
            bpy.ops.render.render(write_still=True)
            report['renders'].append(dict(seconds=seconds, frame=frame_of(seconds), path=scene.render.filepath))
        (args.output / 'report.json').write_text(json.dumps(report, indent=2))
        bpy.ops.wm.save_mainfile()
    print('MANTAFLOW', json.dumps({k: report[k] for k in ('resolution', 'cell_size_m', 'bake_complete', 'bake_seconds',
        'seconds_per_frame', 'fill_hull_volume_liters', 'fill_particle_water_liters')}), flush=True)


if __name__ == '__main__':
    main()

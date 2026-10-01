"""Blender 5.0.1: check actual cached liquid particles against vessel undersides.

Run in a separate Blender process after baking. The default checks every
reported frame; --frames permits an explicitly partial diagnostic check.
"""
import argparse
import gzip
import json
import struct
import sys
import tempfile
from pathlib import Path

import bpy
import numpy as np
import openvdb
from mathutils.geometry import convex_hull_2d

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from experiments.coupled_scenes.mantaflow_pour import FPS, keyframe_pitcher


def underside(obj):
    floor = min(v.co.z for v in obj.data.vertices)
    base = [v.co.xy for v in obj.data.vertices if abs(v.co.z-floor) < 1e-5]
    hull = [base[i] for i in convex_hull_2d(base)]
    if len(hull) < 3:
        raise RuntimeError(f'Cannot establish the flat underside footprint of {obj.name}')
    return floor, hull


def below_base(world, obj, floor, hull):
    inverse = np.array(obj.matrix_world.inverted())
    local = world @ inverse[:3, :3].T + inverse[:3, 3]
    candidates = local[(local[:, 2] < floor-.001) & (local[:, 2] > floor-.05)]
    signs = np.array([(b.x-a.x)*(candidates[:, 1]-a.y)-(b.y-a.y)*(candidates[:, 0]-a.x)
                      for a, b in zip(hull, hull[1:]+hull[:1])])
    return int(np.sum(np.all(signs >= -1e-7, axis=0) | np.all(signs <= 1e-7, axis=0)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='Completed bake directory')
    parser.add_argument('--blend', type=Path, help='Override the original design path recorded in report.json')
    parser.add_argument('--assets', type=Path, help='Override the original fill asset directory')
    parser.add_argument('--frames', type=int, nargs='+', help='Partial diagnostic selection; default: all frames')
    args = parser.parse_args(sys.argv[sys.argv.index('--')+1:])
    if bpy.app.version != (5, 0, 1):
        raise RuntimeError('The cache reader is verified for Blender 5.0.1 only')
    out = args.output.resolve()
    report = json.loads((out/'report.json').read_text())
    if not report['bake_complete']:
        raise RuntimeError('Bake is not complete')
    blend = (args.blend or Path(report['blend'])).resolve()
    assets = (args.assets or Path(report['assets'])).resolve()
    first, last = report['frames']
    selected = sorted(set(args.frames)) if args.frames else list(range(first, last+1))
    if any(f < first or f > last for f in selected):
        parser.error('Requested frames must lie within the reported bake range')
    # Avoid importing Manta with a loaded fluid domain: external audit solvers
    # can otherwise collide with Blender's internal solver namespaces.
    bpy.ops.wm.open_mainfile(filepath=str(blend))
    meta = json.loads((assets/'assets.json').read_text())
    pivot = np.asarray(meta['donor']['position_m']) + np.asarray(report['pitcher_offset_m'])
    keyframe_pitcher(bpy.data.objects['PouringPitcher'], report['motion'], pivot, first, last)
    scene = bpy.context.scene
    pitcher = bpy.data.objects['Spouted ceramic pitcher']
    basin = bpy.data.objects['Low oval receiving basin']
    pitcher_floor, pitcher_hull = underside(pitcher)
    basin_floor, basin_hull = underside(basin)
    lo = np.asarray(report.get('domain_min_blender_m', [-.62, 7.73, .19]))
    extent = np.asarray(report.get('domain_extent_m', [1.24, .69, .79]))
    import manta
    rows = []
    with tempfile.NamedTemporaryFile(prefix='particle_audit_', suffix='.uni', dir=out, delete=False) as handle:
        temporary = Path(handle.name)
    try:
        for frame in selected:
            scene.frame_set(frame)
            file = out/'cache/data'/f'fluid_data_{frame:04d}.vdb'
            metadata = openvdb.readAllGridMetadata(str(file))[0].metadata
            dimensions = metadata['file_base_resolution']
            solver = manta.Solver(name=f'audit_{frame}', gridSize=manta.vec3(*dimensions), dim=3)
            particles = solver.create(manta.BasicParticleSystem, name='particles')
            manta.load(name=str(file), objects=[particles], worldSize=float(max(extent)))
            particles.save(str(temporary))
            with gzip.open(temporary, 'rb') as handle:
                raw = handle.read()
            if raw[:4] != b'PB02':
                raise RuntimeError('Unexpected particle UNI record format')
            count = struct.unpack_from('<i', raw, 4)[0]
            if count <= 0 or len(raw) < count*16:
                raise RuntimeError(f'Invalid or empty particle cache at frame {frame}')
            records = np.frombuffer(raw, dtype=[('pos', '<f4', (3,)), ('flag', '<i4')],
                                    offset=len(raw)-count*16, count=count)
            world = records['pos'][(records['flag'] & 1) == 0]*(extent/np.asarray(dimensions)) + lo
            row = dict(frame=frame, seconds=(frame-1)/FPS, active_particles=len(world),
                       min_world_z=float(world[:, 2].min()),
                       particles_directly_under_base=below_base(world, pitcher, pitcher_floor, pitcher_hull),
                       particles_under_receiving_basin=below_base(world, basin, basin_floor, basin_hull))
            if row['seconds'] <= report['motion']['start_s']:
                bottom = min((pitcher.matrix_world @ v.co).z for v in pitcher.data.vertices)
                row['particles_below_pitcher_before_motion'] = int(np.sum(world[:, 2] < bottom-.001))
            rows.append(row)
            print('PARTICLE_FRAME', json.dumps(row), flush=True)
            del particles, solver
    finally:
        temporary.unlink(missing_ok=True)
    checked = {row['frame'] for row in rows}
    complete = set(range(first, last+1)).issubset(checked)
    passed = bool(rows) and all(not (row['particles_directly_under_base'] or
        row['particles_under_receiving_basin'] or row.get('particles_below_pitcher_before_motion', 0)) for row in rows)
    result = dict(frames=rows, complete=complete, all_frames_checked=complete,
                  requested_frames_checked=set(selected).issubset(checked),
                  penetration_tolerance_m=.001, underside_test_depth_m=.05, passed=passed)
    name = 'particle_validation.json' if args.frames is None else 'particle_validation_partial.json'
    (out/name).write_text(json.dumps(result, indent=2), encoding='utf-8')
    print('PARTICLE_VALIDATION', json.dumps({k: v for k, v in result.items() if k != 'frames'}), flush=True)
    if not passed or not result['requested_frames_checked'] or (args.frames is None and not complete):
        raise RuntimeError('Particle collision verification failed or incomplete')


if __name__ == '__main__':
    main()

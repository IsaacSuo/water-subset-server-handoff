"""Windows Blender: render exactly five verified cached frames with OptiX."""
import argparse
import json
import sys
from pathlib import Path

import bpy

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from experiments.coupled_scenes.mantaflow_pour import configure_optix, frame_of
from experiments.coupled_scenes.blender_server_assets import reload_hdri

TIMES = (1.8, 2., 2.6, 3.2, 3.8)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='Complete bake directory copied from the server')
    parser.add_argument('--hdri', type=Path, required=True, help='Original bryanston_park_sunrise_8k.exr')
    parser.add_argument('--samples', type=int, default=64)
    parser.add_argument('--verify-only', action='store_true', help='Check GPU and all five cached meshes without rendering')
    args = parser.parse_args(sys.argv[sys.argv.index('--')+1:])
    if sys.platform != 'win32' or bpy.app.version != (5, 0, 1):
        raise RuntimeError('Final keyframes require Windows Blender 5.0.1')
    out = args.output.resolve()
    bake = json.loads((out/'report.json').read_text())
    validation = json.loads((out/'validation.json').read_text())
    particles = json.loads((out/'particle_validation.json').read_text())
    if not bake['bake_complete'] or not validation['pre_motion_passed'] or not validation['full_volume_passed']:
        raise RuntimeError('Bake or liquid-volume verification failed')
    if not particles['complete'] or not particles['all_frames_checked'] or not particles['passed']:
        raise RuntimeError('Full particle collision verification failed or incomplete')
    expected = {row['frame']: row['mesh_vertices'] for row in validation['frames']}
    if any(frame_of(seconds) not in expected for seconds in TIMES):
        raise RuntimeError('The bake does not contain all five requested keyframes')
    bpy.ops.wm.open_mainfile(filepath=str(out/'mantaflow_pour.blend'))
    scene = bpy.context.scene
    domain = bpy.data.objects['PourDomain']
    domain.modifiers['Fluid'].domain_settings.cache_directory = str(out/'cache')
    reload_hdri(args.hdri)
    scene.camera = bpy.data.objects['DesignView_00']
    scene.render.resolution_x = 1280
    scene.render.resolution_y = 960
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = 'PNG'
    scene.cycles.samples = args.samples
    scene.cycles.seed = 0
    devices = configure_optix(scene)
    report = dict(blender_version=bpy.app.version_string, platform=sys.platform, backend='OPTIX',
                  devices=devices, camera=scene.camera.name, resolution=[1280, 960],
                  cache_directory=str(out/'cache'), verify_only=args.verify_only, renders=[])
    for seconds in TIMES:
        scene.frame_set(frame_of(seconds))
        evaluated = domain.evaluated_get(bpy.context.evaluated_depsgraph_get())
        mesh = evaluated.to_mesh()
        vertices = len(mesh.vertices)
        evaluated.to_mesh_clear()
        if vertices != expected[scene.frame_current]:
            raise RuntimeError(f'Liquid cache mismatch at frame {scene.frame_current}: {vertices} vertices')
        row = dict(seconds=seconds, frame=scene.frame_current, verified_cache_vertices=vertices)
        if not args.verify_only:
            scene.render.filepath = str(out/f'render_{seconds:.3f}s.png')
            bpy.ops.render.render(write_still=True)
            row['path'] = scene.render.filepath
        report['renders'].append(row)
        name = 'render_preflight.json' if args.verify_only else 'windows_render_report.json'
        (out/name).write_text(json.dumps(report, indent=2), encoding='utf-8')
    if not args.verify_only:
        bpy.ops.wm.save_as_mainfile(filepath=str(out/'repaired_windows.blend'))
    print('KEYFRAMES_VERIFIED' if args.verify_only else 'FIVE_KEYFRAMES_DONE', json.dumps(report), flush=True)


if __name__ == '__main__':
    main()

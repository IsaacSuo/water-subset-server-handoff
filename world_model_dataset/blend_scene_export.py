"""Blender-side read-only evaluated scene export, in original world coordinates."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import bpy
import numpy as np


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--blend', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--support', required=True)
    p.add_argument('--floor', required=True)
    a = p.parse_args(sys.argv[sys.argv.index('--')+1:])
    a.output.mkdir(parents=True, exist_ok=False)
    before = sha(a.blend)
    bpy.ops.wm.open_mainfile(filepath=str(a.blend), load_ui=False, use_scripts=False)
    scene = bpy.context.scene
    # Evaluate the authored render modifier settings without saving the source.
    adjustments = []
    for obj in scene.objects:
        for mod in obj.modifiers:
            change = {'object': obj.name, 'modifier': mod.name}
            if mod.show_viewport != mod.show_render:
                change['show_viewport_before'] = mod.show_viewport
                mod.show_viewport = mod.show_render
            if mod.type == 'SUBSURF' and mod.levels != mod.render_levels:
                change['viewport_levels_before'] = mod.levels
                change['render_levels'] = mod.render_levels
                mod.levels = mod.render_levels
            if len(change)>2:
                adjustments.append(change)
    bpy.context.view_layer.update()
    dg = bpy.context.evaluated_depsgraph_get()
    unit = scene.unit_settings.scale_length
    groups = {key: {'vertices': [], 'triangles': [], 'objects': [], 'nv': 0, 'nt': 0}
              for key in ('support', 'room', 'surroundings')}
    skipped = []
    for inst in dg.object_instances:
        obj = inst.object
        if obj.type not in ('MESH', 'CURVE', 'SURFACE', 'FONT'):
            continue
        if obj.hide_render or not inst.show_self or (inst.parent and inst.parent.hide_render):
            skipped.append(obj.name)
            continue
        mesh = obj.to_mesh()
        try:
            mesh.calc_loop_triangles()
            if not len(mesh.loop_triangles):
                continue
            v = np.empty((len(mesh.vertices), 3), dtype=np.float64)
            mesh.vertices.foreach_get('co', v.ravel())
            m = np.asarray(inst.matrix_world)
            v = (v @ m[:3,:3].T + m[:3,3]) * unit
            f = np.empty((len(mesh.loop_triangles), 3), dtype=np.int32)
            mesh.loop_triangles.foreach_get('vertices', f.ravel())
            # Mirrored object transforms need winding adjustment, not a shape edit.
            if np.linalg.det(m[:3,:3]) < 0:
                f = f[:, [0,2,1]]
            key = 'support' if obj.name == a.support else 'room' if obj.name == a.floor else 'surroundings'
            group = groups[key]
            group['objects'].append(dict(name=obj.name, parent=inst.parent.name if inst.parent else None,
                is_instance=inst.is_instance, matrix_world=m.tolist(), bounds_m=[v.min(0).tolist(),v.max(0).tolist()],
                vertex_start=group['nv'], vertex_count=len(v), triangle_start=group['nt'], triangle_count=len(f)))
            group['vertices'].append(v)
            group['triangles'].append(f + group['nv'])
            group['nv'] += len(v)
            group['nt'] += len(f)
        finally:
            obj.to_mesh_clear()
    records = {}
    for key, group in groups.items():
        if not group['vertices']:
            raise ValueError('Missing group: '+key)
        path = a.output/(key+'.npz')
        np.savez_compressed(path, vertices=np.concatenate(group['vertices']).astype(np.float32),
                            triangles=np.concatenate(group['triangles']))
        records[key] = dict(path=path.name, sha256=sha(path), objects=group['objects'],
                            vertices=group['nv'], triangles=group['nt'])
    assert before == sha(a.blend), 'Source blend changed'
    result = dict(source_blend=str(a.blend), source_sha256=before, blender=bpy.app.version_string,
        frame=scene.frame_current, units=scene.unit_settings.system, metres_per_unit=unit,
        coordinate_frame='original Blender world, Z up, metres; no recentering',
        mesh_operation='evaluated modifiers, instance transforms, triangulation; no decimation/filling/hull',
        environment_motion='all exported environment objects held fixed', render_modifier_alignment=adjustments,
        skipped_hidden=skipped, groups=records)
    (a.output/'scene.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print('SCENE_EXPORTED', {k: (v['vertices'],v['triangles'],len(v['objects'])) for k,v in records.items()}, flush=True)


if __name__ == '__main__':
    main()

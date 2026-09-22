"""Strict experiment inputs and explicit interventions, independent of shot files.

Material ``input`` uses the selected backend's native object/initial/material/
constraint fields. It is deliberately not a universal soft-body asset schema.
"""
from __future__ import annotations

import copy
from pathlib import Path
import re

import numpy as np

from .io import digest, read_json
from .phenomenon_templates import apply_body, BODY_FIELDS

VERSION = 'phenomenon-experiment/1'
# An adapter is a solver boundary, not an outcome classifier. New material
# capabilities require explicit contract and external source validation.
SUPPORT = {
    'rigid_roll_slide': ('rigid', 1),
    'multibody_rearrangement': ('rigid', 1),
    'geometry_constrained_motion': ('rigid', 1),
    'plastic_impact': ('plastic', 2),
    'beam_load_hold_withdraw': ('beam', 2),
    'cloth_drape': ('cloth', 3),
    'rope_passive': ('rope', 4),
    'cloth_drag': ('cloth', 3),
    'rope_finite_load': ('rope', 4),
}
INPUT_FIELDS = {
    'rigid': {'participants', 'asset_library'},
    'cloth': {'cloth', 'initial_velocity_m_s', 'iterations', 'material', 'collision',
              'collision_pair_updates', 'collision_iteration_multiplier',
              'speculative_ccd', 'linear_damping_per_s', 'max_depenetration_velocity_m_s',
              'diagnostic_substeps', 'gripper', 'opposing_fixture'},
    'plastic': {'object', 'initial_velocity_m_s', 'gravity_m_s2', 'material', 'solver'},
    'rope': {'rope', 'gravity_m_s2', 'iterations', 'loads', 'load_control'},
    'beam': {'beam', 'fixture', 'plate', 'iterations', 'contact_offset_m'},
}


def keys(obj, allowed, required, where):
    if not isinstance(obj, dict):
        raise ValueError(where + ': expected object')
    unknown, missing = set(obj) - set(allowed), set(required) - set(obj)
    if unknown or missing:
        raise ValueError(f'{where}: unknown={sorted(unknown)}, missing={sorted(missing)}')


def identifier(value):
    if not isinstance(value, str) or not re.fullmatch('[A-Za-z][A-Za-z0-9_]*', value):
        raise ValueError('Expected a plain identifier: ' + str(value))
    return value


def positive(value, name, integer=False, zero=False):
    if (isinstance(value, bool) or not isinstance(value, (float, int))
            or not np.isfinite(value) or (value < 0 if zero else value <= 0)
            or (integer and int(value) != value)):
        raise ValueError('Invalid ' + name)


def vector(value, name, n=3):
    a = np.asarray(value, dtype=float)
    if a.shape != (n,) or not np.isfinite(a).all():
        raise ValueError('Expected finite vector: ' + name)
    return a


def pointer_set(obj, pointer, value):
    """Replacement only; typos cannot create unused fields or change scene/runtime."""
    parts = pointer.split('/')[1:]
    if not pointer.startswith('/input/') or any(p in ('', '.', '..') for p in parts):
        raise ValueError('Conditions may replace existing /input/... fields only')
    node = obj
    try:
        for p in parts[:-1]:
            node = node[int(p)] if isinstance(node, list) else node[p]
        key = int(parts[-1]) if isinstance(node, list) else parts[-1]
        old = node[key]
        if old == value:
            raise ValueError('No-op condition: ' + pointer)
        node[key] = copy.deepcopy(value)
    except (KeyError, IndexError, TypeError) as exc:
        raise ValueError('Unknown condition input: ' + pointer) from exc


def differences(a, b, path=''):
    """Structural diff used for all representations, including participant lists."""
    if isinstance(a, dict) and isinstance(b, dict):
        result = []
        for key in sorted(set(a) | set(b)):
            p = path + '/' + key
            result += differences(a[key], b[key], p) if key in a and key in b else [p]
        return result
    if isinstance(a, list) and isinstance(b, list) and len(a) == len(b):
        return [p for i, (x, y) in enumerate(zip(a, b))
                for p in differences(x, y, path + '/' + str(i))]
    return [] if a == b else [path]


def intervention(base, condition):
    keys(condition, {'id', 'changes', 'derived_impacts', 'cache'},
         {'id', 'changes', 'derived_impacts'}, 'condition')
    identifier(condition['id'])
    changes = condition['changes']
    if not isinstance(changes, dict):
        raise ValueError('changes must map JSON pointers to replacement values')
    paths = sorted(changes)
    if any(b.startswith(a + '/') for i, a in enumerate(paths) for b in paths[i+1:]):
        raise ValueError('Overlapping interventions')
    impacts = condition['derived_impacts']
    if (not isinstance(impacts, list) or not all(isinstance(x, str) and x.strip() for x in impacts)
            or bool(changes) != bool(impacts)):
        raise ValueError('Changed inputs require explicit derived_impacts; baseline has none')
    variant = copy.deepcopy(base)
    for path, value in changes.items():
        pointer_set(variant, path, value)
    diff = differences(base, variant)
    if any(not any(p == c or p.startswith(c + '/') for c in paths) for p in diff):
        raise ValueError('Undeclared condition change')
    invariant = copy.deepcopy(variant)
    # Replace each intervention subtree with the baseline subtree for an exact
    # comparison of everything the intervention did not authorize.
    for path in paths:
        parts = path.split('/')[1:]
        a, b = base, invariant
        for p in parts[:-1]:
            a = a[int(p)] if isinstance(a, list) else a[p]
            b = b[int(p)] if isinstance(b, list) else b[p]
        key = int(parts[-1]) if isinstance(b, list) else parts[-1]
        b[key] = copy.deepcopy(a[key])
    if invariant != base:
        raise ValueError('Other experiment inputs changed')
    return variant, dict(changed_inputs=diff, declared_subtrees=paths,
                        declared_derived_impacts=impacts,
                        invariant_sha256=digest(invariant),
                        scene_sha256=digest(base['scene']))


def validate_common(doc):
    required = {'format', 'id', 'phenomenon', 'backend', 'scene', 'input',
                'control', 'timing', 'observations', 'conditions'}
    keys(doc, required, required, 'experiment')
    if doc['format'] != VERSION or doc['phenomenon'] not in SUPPORT:
        raise ValueError('Unsupported experiment format or phenomenon')
    identifier(doc['id'])
    kind = SUPPORT[doc['phenomenon']][0]
    backend = doc['backend']
    fields = {'kind'} if kind == 'rigid' else {'kind', 'entry', 'runtime'}
    keys(backend, fields | ({'preparation_seed'} if kind != 'rigid' else set()), fields, 'backend')
    if kind != 'rigid':
        seed = backend.get('preparation_seed', 0)
        if type(seed) is not int or not 0 <= seed < 2**32:
            raise ValueError('preparation_seed must be a uint32 integer')
    if backend['kind'] != kind:
        raise ValueError('Phenomenon is connected to the wrong backend')
    expected_control = {'beam_load_hold_withdraw': 'finite_plate',
                        'cloth_drag': 'finite_gripper', 'rope_finite_load': 'finite_load'}.get(doc['phenomenon'], 'none')
    if doc['phenomenon']=='rope_finite_load' and doc['input'].get('load_control',{}).get('enabled') is False:
        expected_control='finite_load_disabled'
    if doc['control'] != expected_control:
        raise ValueError('Unsupported actuator/control for selected backend')
    for active, fields in [('cloth_drag', {'gripper'}), ('rope_finite_load', {'loads','load_control'})]:
        if doc['phenomenon'] == active and not fields <= doc['input'].keys():
            raise ValueError('Missing finite actuator fields: '+str(sorted(fields)))
    if doc['phenomenon'] == 'cloth_drape' and {'gripper','opposing_fixture'} & doc['input'].keys():
        raise ValueError('Passive drape cannot contain finite gripper input')
    if doc['phenomenon'] == 'rope_passive' and {'loads','load_control'} & doc['input'].keys():
        raise ValueError('Passive rope cannot contain finite load input')
    scene = doc['scene']
    fields = {'id', 'region_bounds_m', 'units', 'up_axis', 'frame', 'collision', 'source_records'}
    keys(scene, fields, fields, 'scene')
    identifier(scene['id'])
    if (scene['units'], scene['up_axis'], scene['frame']) != ('m', 'Z', 'original_world'):
        raise ValueError('Supply original world-space collision meshes in metres, Z-up')
    bounds = np.asarray(scene['region_bounds_m'], float)
    if bounds.shape != (2, 3) or not np.isfinite(bounds).all() or np.any(bounds[1] <= bounds[0]):
        raise ValueError('Invalid scene region bounds')
    collision = scene['collision']
    if kind == 'rigid':
        keys(collision, {'scene_export'}, {'scene_export'}, 'scene.collision')
    else:
        keys(collision, {'meshes'}, {'meshes'}, 'scene.collision')
        if not isinstance(collision['meshes'], list) or not collision['meshes']:
            raise ValueError('Original scene collision meshes are required')
        for mesh in collision['meshes']:
            if 'scale' in mesh or 'world_from_mesh' in mesh:
                raise ValueError('Environment transforms are unsupported: export original world metres first')
    timing = doc['timing']
    keys(timing, {'duration_s', 'physics_hz', 'state_hz'},
         {'duration_s', 'physics_hz', 'state_hz'}, 'timing')
    for key, value in timing.items():
        positive(value, key, integer=key != 'duration_s')
    if timing['physics_hz'] % timing['state_hz']:
        raise ValueError('State rate must divide physics rate')
    if not np.isclose(timing['duration_s'] * timing['state_hz'],
                      round(timing['duration_s'] * timing['state_hz']), rtol=0, atol=1e-7):
        raise ValueError('Duration must end on a saved state')
    if kind == 'rigid' and (timing['physics_hz'], timing['state_hz']) != (240, 240):
        raise ValueError('Rigid adapter currently records all states at 240 Hz')
    obs = doc['observations']
    keys(obs, {'hz', 'camera', 'required_visible_ids'}, {'hz', 'camera'}, 'observations')
    positive(obs['hz'], 'observation hz', integer=True)
    if timing['state_hz'] % obs['hz']:
        raise ValueError('Observation rate must divide saved-state rate')
    if not np.isclose(timing['duration_s'] * obs['hz'], round(timing['duration_s'] * obs['hz']), atol=1e-7, rtol=0):
        raise ValueError('Duration must end on an observation sample')
    camera = obs['camera']
    keys(camera, {'position_m', 'target_m', 'ortho_scale_m'},
         {'position_m', 'target_m', 'ortho_scale_m'}, 'diagnostic camera')
    delta = vector(camera['target_m'], 'camera target') - vector(camera['position_m'], 'camera position')
    if np.linalg.norm(np.cross(delta, [0, 0, 1])) < 1e-9:
        raise ValueError('Vertical or zero diagnostic camera direction')
    positive(camera['ortho_scale_m'], 'perspective target-plane width')
    if not isinstance(obs.get('required_visible_ids', []), list):
        raise ValueError('required_visible_ids must be a list')
    if not isinstance(doc['conditions'], list) or not doc['conditions']:
        raise ValueError('An explicit baseline/condition list is required')
    ids = [identifier(c['id']) for c in doc['conditions']]
    if len(set(ids)) != len(ids):
        raise ValueError('Duplicate condition id')
    return kind


def native_config(doc):
    """Compile the common contract to an existing material-scene contract."""
    kind = doc['backend']['kind']
    keys(doc['input'], INPUT_FIELDS[kind], (), 'input')
    cfg = dict(copy.deepcopy(doc['input']), **doc['timing'], format='material-scene/1', kind=kind,
               environment=copy.deepcopy(doc['scene']['collision']['meshes']),
               source_records=copy.deepcopy(doc['scene']['source_records']))
    # Packaging capture is a declaration; the observations stage renders exactly
    # this rate. No unrequested preview parameters reach native material scripts.
    if kind != 'beam':
        cfg['observation_hz'] = doc['observations']['hz']
    if kind == 'plastic' and cfg.get('material', {}).get('viscosity', 0) != 0:
        raise ValueError('Viscoelasticity/viscosity is outside this experiment interface')
    return cfg


def rigid_spec(doc, job_id):
    """Build one physical recipe directly, with any number of supplied assets."""
    obj = doc['input']
    keys(obj, INPUT_FIELDS['rigid'], INPUT_FIELDS['rigid'], 'rigid input')
    lib = obj['asset_library']
    keys(lib, {'exploration_root', 'library'}, {'exploration_root', 'library'}, 'asset_library')
    bodies = copy.deepcopy(obj['participants'])
    if not isinstance(bodies, list) or not bodies:
        raise ValueError('At least one participant is required')
    if doc['phenomenon'] == 'multibody_rearrangement' and len(bodies) < 2:
        raise ValueError('Multibody rearrangement needs at least two participants')
    ids = []
    for body in bodies:
        keys(body, BODY_FIELDS | {'id', 'asset', 'support_group', 'ray_start_z_m'},
             {'id', 'asset', 'size_m', 'xy_m'}, 'participant')
        ids.append(identifier(body['id'])); identifier(body['asset'])
        apply_body(body, {})
        if not any(k in body for k in ('mass_kg', 'density_kg_m3')):
            raise ValueError('Each participant requires explicit mass or density')
        positive(body.get('clearance_m', .001), 'clearance_m', zero=True)
        if 'ray_start_z_m' in body:
            vector([body['ray_start_z_m']], 'ray_start_z_m', 1)
    if len(set(ids)) != len(ids):
        raise ValueError('Duplicate participant id')
    return dict(scene_id=doc['scene']['id'], scene_export=doc['scene']['collision']['scene_export'],
                **lib, scope=doc['phenomenon'], shots=[dict(id=job_id, group=1,
                    title=doc['phenomenon'], phenomenon=doc['phenomenon'],
                    region=doc['scene']['region_bounds_m'], duration_s=doc['timing']['duration_s'],
                    camera=doc['observations']['camera'], bodies=bodies)])


def load_experiment(path):
    """Resolve caller-owned file paths once; no implicit mainline output directory."""
    path = Path(path).resolve()
    doc = read_json(path)
    kind = validate_common(doc)
    def resolve(value):
        p = Path(value)
        return str((path.parent / p).resolve())
    for key in doc['scene']['source_records']:
        doc['scene']['source_records'][key] = resolve(doc['scene']['source_records'][key])
    if kind == 'rigid':
        doc['scene']['collision']['scene_export'] = resolve(doc['scene']['collision']['scene_export'])
        lib = doc['input']['asset_library']
        lib['exploration_root'] = resolve(lib['exploration_root'])
    else:
        doc['backend'].setdefault('preparation_seed', 0)
        doc['backend']['entry'] = resolve(doc['backend']['entry'])
        doc['backend']['runtime'] = resolve(doc['backend']['runtime'])
        for mesh in doc['scene']['collision']['meshes']:
            mesh['path'] = resolve(mesh['path'])
        for name in ('cloth', 'object'):
            if 'path' in doc['input'].get(name, {}):
                doc['input'][name]['path'] = resolve(doc['input'][name]['path'])
    # Path-valued interventions are resolved too, before any cache comparison.
    for c in doc['conditions']:
        for p, value in c['changes'].items():
            if p.endswith('/path') or p.endswith('/exploration_root'):
                c['changes'][p] = resolve(value)
        if 'cache' in c:
            keys(c['cache'], {'run', 'manifest', 'manifest_sha256'},
                 {'run', 'manifest', 'manifest_sha256'}, 'cache')
            c['cache']['run'] = resolve(c['cache']['run'])
    return doc

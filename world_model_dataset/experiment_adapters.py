"""Concrete adapters for existing independently versioned solvers.

External material code executes in a subprocess: two checkouts must never share
Python's input_contract/material_entry module cache. No external files are edited.
"""
from __future__ import annotations

import copy
from pathlib import Path
import shutil
import subprocess
import sys

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

from .causal_runner import ROOT, prepare as prepare_rigid, invoke, package_physics
from .experiment_contract import native_config, rigid_spec
from .io import read_json, write_json, file_hash, digest
from .real_scene_batch import recipe


def run_command(command, log):
    with Path(log).open('x', encoding='utf-8') as stream:
        subprocess.run(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT, check=True)


def material_call(doc, run, stage, log, config=None):
    command = [sys.executable, doc['backend']['entry'], '--stage', stage,
               '--output', str(run), '--runtime', doc['backend']['runtime'], '--mainline', str(ROOT)]
    if config:
        command += ['--config', str(config)]
    if stage == 'prepare':
        # trimesh point containment has a randomized fallback ray for ambiguous
        # surface hits. Fix CPU preparation randomness, independently of solver
        # randomness, so material comparisons keep the same point realization.
        wrapper = ('import sys,runpy,numpy as np,trimesh; from pathlib import Path; '
                   'seed=int(sys.argv[1]); sys.argv=sys.argv[2:]; '
                   'sys.path.insert(0,str(Path(sys.argv[0]).parent)); '
                   'np.random.seed(seed); '
                   'trimesh.util.random_generator().bit_generator.state=np.random.default_rng(seed).bit_generator.state; '
                   'runpy.run_path(sys.argv[0],run_name="__main__")')
        command = [sys.executable, '-c', wrapper, str(doc['backend'].get('preparation_seed', 0)), *command[1:]]
    if stage == 'simulate':
        # Our guard below checks actual processes/utilization first; the legacy
        # material guard mistakes desktop allocation for another simulation.
        command += ['--allow-busy-gpu']
    run_command(command, log)


def check_gpu():
    processes = subprocess.check_output(
        ['nvidia-smi', '--query-compute-apps=pid,process_name,used_memory', '--format=csv,noheader'], text=True)
    stats = subprocess.check_output(
        ['nvidia-smi', '--query-gpu=memory.free,utilization.gpu', '--format=csv,noheader,nounits'], text=True)
    if processes.strip() or any(int(row.split(',')[1]) > 10 for row in stats.splitlines()):
        raise RuntimeError('GPU busy; prepared inputs preserved; retry simulate after current job finishes')
    if not any(int(row.split(',')[0]) >= 4096 for row in stats.splitlines()):
        raise RuntimeError('GPU has less than 4096 MiB free; inputs preserved')
    return dict(compute_processes=processes, memory_free_and_utilization=stats)


def source_files(doc):
    """Include authoring defaults, solver scripts and each external input source."""
    paths = list((ROOT / 'world_model_dataset').glob('*.py'))
    paths += list((ROOT / 'configs/dataset/v0_2').glob('*.json'))
    paths += list((ROOT / 'configs/dataset/v0_2/examples').glob('*.json'))
    paths += [ROOT / 'soft_body/tet_quality.py']
    # CPU geometry dependencies are part of the prepared realization too. In
    # particular trimesh's independent RNG cannot be controlled by np.seed.
    import importlib.metadata
    import trimesh.ray.ray_util
    paths += [Path(trimesh.util.__file__), Path(trimesh.ray.ray_util.__file__)]
    for name in ('numpy', 'scipy', 'trimesh'):
        distribution = importlib.metadata.distribution(name)
        paths += [Path(distribution.locate_file(p)) for p in distribution.files if str(p).endswith('.dist-info/METADATA')]
    paths += [Path(p) for p in doc['scene']['source_records'].values()]
    for name in ('construction','material_bridge'):
        if name in doc['scene']['source_records']:
            record=read_json(doc['scene']['source_records'][name])
            for source,checksum in record['source_pins'].items():
                if file_hash(source)!=checksum:
                    raise ValueError('Construction or material source changed: '+source)
                paths.append(Path(source))
    if doc['backend']['kind'] == 'rigid':
        scene = Path(doc['scene']['collision']['scene_export'])
        paths.append(scene)
        scene_info = read_json(scene)
        if scene_info.get('metres_per_unit') != 1.0 or 'Z up' not in scene_info.get('coordinate_frame', ''):
            raise ValueError('Scene export must declare original world metres and Z up')
        if {b['id'] for b in doc['input']['participants']} & set(scene_info['groups']):
            raise ValueError('Participant id collides with original scene object')
        for info in scene_info['groups'].values():
            p = scene.parent / info['path']
            if file_hash(p) != info['sha256']:
                raise ValueError('Original scene geometry hash mismatch: ' + str(p))
            paths.append(p)
        lib = doc['input']['asset_library']
        root = Path(lib['exploration_root'])
        paths.append(root / 'experiments/physical_assets/prepare.py')
        for body in doc['input']['participants']:
            folder = root / lib['library'] / body['asset']
            metadata = read_json(folder / 'asset.json')
            p = folder / metadata['geometry']
            if file_hash(p) != metadata['geometry_sha256']:
                raise ValueError('Asset geometry hash mismatch: ' + str(p))
            paths.extend([folder / 'asset.json', p])
    else:
        entry = Path(doc['backend']['entry'])
        runtime = read_json(doc['backend']['runtime'])
        if doc['backend']['kind'] not in runtime:
            raise ValueError('Runtime has no selected material backend')
        if not (entry.parent / ('native_' + doc['backend']['kind'] + '.py')).is_file():
            raise ValueError('Selected source does not implement the requested backend')
        paths.extend(entry.parent.glob('*.py'))
        paths.append(Path(doc['backend']['runtime']))
        if doc['backend']['kind'] == 'rope':
            paths.append(entry.parent.parent / 'generation/vendor/experiments/newton_backend_probe/common/newton_io.py')
        paths.extend(Path(m['path']) for m in doc['scene']['collision']['meshes'])
        for name in ('object', 'cloth'):
            if 'path' in doc['input'].get(name, {}):
                paths.append(Path(doc['input'][name]['path']))
    return sorted(set(p.resolve() for p in paths))


def prepare(doc, folder, job_id):
    """Real CPU authoring and geometric preflight, not just JSON expansion."""
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=False)
    write_json(folder / 'experiment.json', doc)
    if doc['backend']['kind'] == 'rigid':
        spec = rigid_spec(doc, job_id)
        write_json(folder / 'request.json', spec)
        config, manifest = recipe(spec, spec['shots'][0])
        config['timing']['capture_hz'] = doc['observations']['hz']
        manifest['timing'] = copy.deepcopy(config['timing'])
        write_json(folder / 'manifest.json', manifest)
        config['manifest_template'] = str(folder / 'manifest.json')
        write_json(folder / 'config.json', config)
        run = folder / job_id
        prepare_rigid(folder / 'config.json', run)
    else:
        config = native_config(doc)
        write_json(folder / 'config.json', config)
        # Public material preparation invokes its strict native input contract.
        run = folder / job_id
        material_call(doc, run, 'prepare', folder / 'prepare.log', folder / 'config.json')
    preflight = geometric_preflight(doc, run)
    write_json(folder / 'preflight.json', preflight)
    return run


def geometric_preflight(doc, run):
    kind = doc['backend']['kind']
    bounds = np.asarray(doc['scene']['region_bounds_m'])
    points = {}; meshes = {}; initial = {}; details = {}
    if kind == 'rigid':
        resolved = read_json(run / 'resolved_inputs.json')
        manifest = read_json(run / 'episode.prepared.json')
        initial = manifest['initial_state']['body_states']
        for body in manifest['system']['bodies']:
            oid = body['instance_id']
            with np.load(run / resolved['bodies'][oid]['geometry']['mesh']['path']) as z:
                v, f = z['vertices'], z['triangles']
            s = initial[oid]
            v = Rotation.from_quat(s['orientation_xyzw']).apply(v) + s['position_m']
            if body['physics_kind'] == 'static':
                meshes[oid] = trimesh.Trimesh(v, f, process=False)
            else:
                points[oid] = v
        details['support_rays'] = read_json(run / 'input_config.json')['real_scene_design']['placements']
        for p in doc['input']['participants']:
            if p['id'] in meshes:
                raise ValueError('Participant id collides with original scene object')
            if p.get('support_group', 'support') not in meshes:
                raise ValueError('Unknown support reference')
    else:
        cfg = read_json(run / 'prepared/config.json')
        if kind == 'cloth':
            with np.load(run / 'prepared/cloth.npz') as z:
                points['cloth'] = z['vertices']
        elif kind in ('rope', 'plastic'):
            with np.load(run / 'prepared/initial.npz') as z:
                points[kind] = z['points']
            for load in cfg.get('loads', []):
                points[load['id']] = np.array([[x,y,z] for x in (-.5,.5) for y in (-.5,.5)
                    for z in (-.5,.5)]) * load['size_m'] + load['position_m']
        else:
            for key in ('beam', 'plate', 'fixture'):
                obj = cfg[key]
                points[key] = np.array([[x, y, z] for x in (-.5, .5) for y in (-.5, .5)
                                        for z in (-.5, .5)]) * obj['size_m'] + obj['center_m']
            details['backend_clearance'] = read_json(run / 'prepared/initial_clearance.json')
            beam, fixture = cfg['beam'], cfg['fixture']
            # Native attachment selects the -X interval. Reject a visible fixture
            # disconnected from that interval even if the solver could bridge it.
            root = np.asarray(beam['center_m']) - [beam['size_m'][0]/2 - fixture['length_m']/2, 0, 0]
            if (abs(root[0] - fixture['center_m'][0]) > fixture['size_m'][0]/2
                    or abs(root[1] - fixture['center_m'][1]) > fixture['size_m'][1]/2):
                raise ValueError('Fixture does not cover the requested beam attachment region')
            details['constraint'] = 'visible world-fixed fixture; native -X vertex attachment; finite Z-guided plate'
        for key in ('gripper','opposing_fixture'):
            if key in cfg:
                obj=cfg[key]
                points[key] = np.array([[x,y,z] for x in (-.5,.5) for y in (-.5,.5)
                    for z in (-.5,.5)]) * obj['size_m'] + obj['center_m']
        for item in cfg['environment']:
            with np.load(run / 'prepared' / item['geometry']) as z:
                meshes[item['id']] = trimesh.Trimesh(z['vertices'], z['triangles'], process=False)
    for oid, p in points.items():
        if not len(p) or not np.isfinite(p).all() or np.any(p < bounds[0] - 1e-6) or np.any(p > bounds[1] + 1e-6):
            raise ValueError('Initial object lies outside declared region: ' + oid)
    samples = []
    for oid, p in points.items():
        for name, mesh in meshes.items():
            # Bounded deterministic vertex sampling; no continuous/swept claim.
            selected = p[::max(1, len(p)//512)]
            distance = float(trimesh.proximity.closest_point(mesh, selected)[1].min())
            inside = bool(mesh.contains(selected).any()) if mesh.is_watertight else None
            if inside:
                raise ValueError('Initial sampled vertices inside original collision object: ' + oid + '/' + name)
            hit = mesh.ray.intersects_location([p.mean(0) + [0, 0, .001]], [[0, 0, -1]], multiple_hits=False)[0]
            samples.append(dict(body=oid, environment=name, sampled_minimum_distance_m=distance,
                                sampled_inside_closed_mesh=inside, support_below_centroid_m=hit.tolist()))
    pairs = []
    for i, (a, av) in enumerate(points.items()):
        for b, bv in list(points.items())[i+1:]:
            overlap = np.minimum(av.max(0), bv.max(0)) - np.maximum(av.min(0), bv.min(0))
            pairs.append(dict(a=a, b=b, aabb_overlap_m=overlap.tolist(),
                              possible_overlap=bool(np.all(overlap > 0))))
    ids = set(initial) if kind == 'rigid' else set(meshes) | {
        'cloth': {'cloth'}, 'plastic': {'mpm_block'},
        'beam': {'Beam', 'Plate', 'Fixture'}, 'rope': {'segment_0000'}
    }[kind]
    if set(doc['observations'].get('required_visible_ids', [])) - ids:
        raise ValueError('Observation references unknown participant')
    return dict(coordinates='original world metres, Z-up', initial_region_checked=True,
                contact_force='unavailable', attachment_reaction='unavailable', rope_native_tension='unavailable',
                cpu_preparation_seed=doc['backend'].get('preparation_seed'),
                participants={k: [v.min(0).tolist(), v.max(0).tolist()] for k, v in points.items()},
                geometry_checks=samples, participant_aabb_checks=pairs, details=details,
                collision_limits='Sampled vertex distance/closed-mesh containment and AABBs only; '
                                 'open surfaces, triangle crossings, swept clearance and outcomes are not guaranteed',
                control=doc['control'], training_admission=False)


def simulate(doc, run, log_root):
    gpu = check_gpu()
    log_root.mkdir(exist_ok=False)
    write_json(log_root / 'gpu_before.json', gpu)
    if doc['backend']['kind'] == 'rigid':
        invoke('native_causal_rigid.py', run, 'simulation.log')
        if not (run / 'native_report.json').is_file():
            raise RuntimeError('Solver did not write native completion report')
    else:
        material_call(doc, run, 'simulate', log_root / 'simulate.log')
        if (run / 'native/failure.json').exists():
            raise RuntimeError('Native solver reported failure')
        expected = 'frames.npz' if doc['backend']['kind'] in ('cloth', 'beam') else 'states.npz'
        if not (run / 'native' / expected).is_file():
            raise RuntimeError('Missing native solver cache: ' + expected)


def package(doc, run, log_root):
    if doc['backend']['kind'] == 'rigid':
        package_physics(run)
        return run
    material_call(doc, run, 'package', log_root / 'package.log')
    return run / 'episode'


def cache_package_context(run, binding, destination, source_pins=None):
    """Keep the exact inputs that produced a reused native cache.

    The current request was independently prepared and compared by
    cache_binding. Metadata-only path normalization is not a reason to rewrite
    native/config.json or defeat the upstream packager's exact-input check.
    Both the current preparation and this original context remain inspectable.
    """
    source = Path(binding['source_run'])
    destination.mkdir(parents=True, exist_ok=False)
    shutil.copytree(source/'prepared', destination/'prepared')
    if source_pins is not None:
        for name, checksum in tree_hashes(destination/'prepared').items():
            original = source/'prepared'/Path(name).relative_to(destination/'prepared')
            if '__pycache__' in original.parts or original.suffix == '.pyc':
                continue
            if source_pins.get(str(original)) != checksum:
                raise ValueError('Prepared cache changed during adoption: ' + str(original))
    shutil.copytree(run/'native', destination/'native')
    for name in ('requested_config.json', 'runtime_request.json', 'execution.json'):
        if (source/name).exists():
            shutil.copyfile(source/name, destination/name)
    write_json(destination/'cache_origin.json', dict(binding=binding,
        current_preparation=str(run/'prepared'), original_context=str(source/'prepared'),
        physics_rerun=False, native_files=tree_hashes(destination/'native')))
    return destination


def tree_hashes(root):
    return {str(p): file_hash(p) for p in sorted(root.rglob('*')) if p.is_file()}


def physical_config(config):
    """Strip only nonphysical documentation/preview fields for cache equivalence."""
    return {k: copy.deepcopy(v) for k, v in config.items() if k not in {
        'format', 'source_records', 'scope', 'scene_request', 'diagnostic_camera',
        'preview_fps', 'preview_slowdown', 'change_reason', 'observation_hz'}}


def physical_signature(kind, run):
    if kind != 'rigid':
        import hashlib
        arrays = {}
        for p in sorted((run / 'prepared').glob('*.npz')):
            with np.load(p) as z:
                arrays[p.name] = {k: dict(shape=list(z[k].shape), dtype=str(z[k].dtype),
                    sha256=hashlib.sha256(np.ascontiguousarray(z[k]).tobytes()).hexdigest()) for k in z.files}
        return dict(config=physical_config(read_json(run / 'prepared/config.json')), prepared_arrays=arrays)
    m = read_json(run / 'episode.prepared.json')
    r = read_json(run / 'resolved_inputs.json')
    records = {}
    for body in m['system']['bodies']:
        oid = body['instance_id']; definition = r['bodies'][oid]
        g = definition['geometry']
        # Hash array bytes, not zip headers, file paths or JSON float spelling.
        import hashlib
        with np.load(run / g['mesh']['path']) as z:
            array_hashes = {k: hashlib.sha256(np.ascontiguousarray(z[k]).tobytes()).hexdigest()
                            for k in ('vertices', 'triangles')}
        records[oid] = dict(kind=body['physics_kind'], state=m['initial_state']['body_states'][oid],
                            physics=definition['physics'], mesh_arrays_sha256=array_hashes,
                            collision={k: v for k, v in g.items() if k.startswith('sdf_') or k == 'collision_approximation'})
    return dict(bodies=records, numerics=r['numerics'],
                duration_s=m['timing']['duration_s'], physics_hz=m['timing']['physics_hz'])


def cache_binding(doc, prepared_run, cache):
    """Compare actual prepared physics with a cache, never trust its job label.

    Material native inputs and all local geometry/source script bytes must match.
    Rigid matching uses the authored initial states, resolved physical profiles,
    mesh arrays and numerical settings; provenance paths are compared by content.
    """
    source = Path(cache['run'])
    manifest_path = source / cache['manifest']
    if not manifest_path.resolve().is_relative_to(source.resolve()):
        raise ValueError('Cache manifest escapes run')
    if file_hash(manifest_path) != cache['manifest_sha256']:
        raise ValueError('Cache manifest hash mismatch')
    kind = doc['backend']['kind']
    a, b = physical_signature(kind, prepared_run), physical_signature(kind, source)
    if kind == 'rigid':
        snapshot = read_json(source / 'source_snapshot.json')['sha256']
        for name in ('world_model_dataset/native_causal_rigid.py', 'world_model_dataset/controllers.py',
                     'world_model_dataset/causal_control.py', 'world_model_dataset/causal_soft.py',
                     'soft_body/tet_quality.py'):
            if snapshot[name] != file_hash(ROOT / name):
                raise ValueError('Cached solver source differs: ' + name)
    else:
        # Material fields with backend-generated summaries are part of this
        # comparison, not silently discarded. Prepared mesh/source bytes bind
        # object shape, sampling and backend implementation as well.
        for p in (prepared_run / 'prepared').glob('*'):
            if p.suffix not in ('.npz', '.py'):
                continue
            other = source / 'prepared' / p.name
            if not other.is_file():
                raise ValueError('Cache missing prepared dependency: ' + p.name)
            if p.suffix == '.npz':
                with np.load(p) as x, np.load(other) as y:
                    if set(x.files) != set(y.files) or any(not np.array_equal(x[k], y[k]) for k in x.files):
                        raise ValueError('Cache geometry/sampling mismatch: ' + p.name)
            elif file_hash(p) != file_hash(other):
                raise ValueError('Cache solver source mismatch: ' + p.name)
    if a != b:
        from .experiment_contract import differences
        raise ValueError('Cache has different physical inputs: ' + str(differences(a, b)[:20]))
    return dict(source_run=str(source), source_manifest=str(manifest_path),
                manifest_sha256=cache['manifest_sha256'], physical_input_sha256=digest(a),
                comparison='authored input, native geometry, timing and solver dependencies',
                episode=str(manifest_path.parent), physics_rerun=False)

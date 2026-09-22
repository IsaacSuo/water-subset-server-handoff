"""Public experiment entry: prepare, simulate, package, audit, observe, register.

Run ``python -m world_model_dataset.phenomenon_experiment --help``. Completed
stages are sealed by content; incomplete stage directories are retained. Each
condition fails independently. A cache binding cannot substitute old physics for
new inputs. The registry includes unsuccessful physical outcomes without labels
claiming they met the experimental intent.
"""
from __future__ import annotations

import argparse
import copy
import json
import os
from pathlib import Path
import shutil
import socket
import time
import traceback

from . import experiment_adapters as adapters
from .causal_loader import open_episode
from .experiment_contract import load_experiment, validate_common, intervention, differences, SUPPORT
from .experiment_review import review, verify_alignment
from .io import digest, file_hash, read_json, write_json
from .phenomenon_catalog import build
from .phenomenon_collect import collect

STAGES = ('prepare', 'simulate', 'package', 'audit', 'observe', 'register')


def hashes(root):
    # Resolve the root once. Resolving every file walks every ancestor through
    # WSL's network filesystem; a thousand native frames otherwise trigger tens
    # of thousands of redundant metadata RPCs. File contents are still hashed.
    root = Path(root).resolve()
    return {str(p.resolve() if p.is_symlink() else p): file_hash(p) for p in sorted(root.rglob('*'))
            if p.is_file() and '__pycache__' not in p.parts and p.suffix != '.pyc'}


def verify(pins):
    for name, expected in pins.items():
        if not Path(name).is_file() or file_hash(name) != expected:
            raise ValueError('Pinned input/dependency/artifact changed: ' + name)


def atomic(path, value):
    """Only the workflow ledger is mutable; stage artifacts use exclusive writes."""
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + '\n', encoding='utf-8')
    tmp.replace(path)


def manifest_at(ep):
    return next(ep / name for name in ('episode.json', 'episode.physics.json') if (ep / name).is_file())


def seal(item, stage, result, files):
    item['stages'][stage] = dict(result=result, files=files)


def adopt_native_cache(prepared, run_dir):
    """Reuse native arrays but package with the pinned current adapter.

    This avoids inheriting obsolete derived summaries (e.g. a volume's former
    rigid angular-velocity placeholder) from an older package. No physics rerun.
    """
    marker = run_dir / 'cache_origin.json'
    if marker.exists():
        verify(read_json(marker)['native_files'])
        return
    source = Path(prepared['cache']['source_run'])
    shutil.copytree(source / 'native', run_dir / 'native')
    for name in ('execution.json',):
        if (source / name).exists():
            shutil.copyfile(source / name, run_dir / name)
    copied = hashes(run_dir / 'native')
    for name, checksum in copied.items():
        original = source/'native'/Path(name).relative_to(run_dir/'native')
        if prepared['cache_pins'].get(str(original)) != checksum:
            raise ValueError('Native cache changed during adoption: ' + str(original))
    write_json(marker, dict(source=str(source), binding=prepared['cache'],
                            native_files=copied, physics_rerun=False))


def prepare_job(doc, condition, folder, job_id):
    pins = {str(p): file_hash(p) for p in adapters.source_files(doc)}
    run = adapters.prepare(doc, folder, job_id)
    verify(pins)
    # Small source snapshots make the exact code inspectable without touching
    # the other worktrees; large geometry already lives in prepared physics.
    sources = folder / 'sources'; sources.mkdir()
    for name, checksum in pins.items():
        p = Path(name)
        if p.suffix in ('.py', '.json'):
            shutil.copyfile(p, sources / (checksum + p.suffix))
    write_json(folder / 'source_pins.json', pins)
    signature = adapters.physical_signature(doc['backend']['kind'], run)
    write_json(folder / 'effective_physics.json', signature)
    result = dict(run=str(run), source_pins=pins, effective_physics=signature)
    if 'cache' in condition:
        binding = adapters.cache_binding(doc, run, condition['cache'])
        # Hash the entire supplied run, including native arrays, solver/source
        # snapshots and commands. A later resume detects nested cache mutation.
        result['cache_pins'] = hashes(binding['source_run'])
        write_json(folder / 'cache_binding.json', binding)
        result['cache'] = binding
        ep = open_episode(binding['episode'], require_complete=False)
        verify_alignment(ep, doc['timing'])
    return result


def run(experiment, output, stage='all', ids=None, retry_failed=False, current_catalog=None):
    """Run/resume independent conditions; returns a durable status ledger.

    ``resume`` performs remaining stages, including physics only where no cache
    or finished simulation exists. ``audit/observe/register`` never launch physics.
    """
    experiment = Path(experiment).resolve(); output = Path(output).resolve()
    doc = load_experiment(experiment)
    catalog_pin = None
    if current_catalog is not None:
        current_catalog = Path(current_catalog).resolve()
        catalog_pin = dict(path=str(current_catalog), sha256=file_hash(current_catalog))
    if stage in ('all','resume','register') and current_catalog is None:
        raise ValueError('Registration requires explicit --current-catalog; use the verified current mainline catalog')
    selected = set(ids) if ids else {c['id'] for c in doc['conditions']}
    if selected - {c['id'] for c in doc['conditions']}:
        raise ValueError('Unknown condition selection')
    output.mkdir(parents=True, exist_ok=True)
    # Cross-process lock protects the same output, not the user's other jobs.
    lock = output / '.workflow.lock'
    if lock.exists():
        owner = read_json(lock)
        if owner['host'] != socket.gethostname():
            raise RuntimeError('Output is locked by another host')
        try:
            os.kill(owner['pid'], 0)
        except ProcessLookupError:
            lock.rename(output / ('.abandoned-lock-' + str(time.time_ns()) + '.json'))
        else:
            raise RuntimeError('Output is locked by a live workflow; use another output')
    with lock.open('x') as stream:
        json.dump(dict(pid=os.getpid(), host=socket.gethostname(), started=time.time()), stream)
    try:
        path = output / 'workflow.json'
        if path.exists():
            ledger = read_json(path)
            if ledger.get('current_catalog') != catalog_pin:
                raise ValueError('Current catalog binding changed; supply the same catalog on every stage')
            if ledger['experiment_sha256'] != file_hash(experiment) or ledger['resolved_sha256'] != digest(doc):
                raise ValueError('Experiment changed; choose a new output directory')
        else:
            ledger = dict(format='experiment-workflow/1', experiment=str(experiment),
                          experiment_sha256=file_hash(experiment), resolved_sha256=digest(doc),
                          current_catalog=catalog_pin, jobs={})
            write_json(output / 'experiment.json', doc)
            atomic(path, ledger)
        stages = STAGES if stage in ('all', 'resume') else (stage,)
        base = copy.deepcopy(doc); base.pop('conditions')
        for condition in doc['conditions']:
            cid = condition['id']
            if cid not in selected:
                continue
            job_id = doc['id'] + '_' + cid
            item = ledger['jobs'].setdefault(cid, dict(id=job_id, stages={}, errors=[], attempts=[]))
            current = 'verify'
            try:
                variant, comparison = intervention(base, condition)
                variant['conditions'] = [dict(id='resolved', changes={}, derived_impacts=[])]
                validate_common(variant)
                if 'prepare' in item['stages']:
                    artifacts = {}
                    for record in item['stages'].values():
                        for name, checksum in record['files'].items():
                            if name in artifacts and artifacts[name] != checksum:
                                raise ValueError('Contradictory stage seals: ' + name)
                            artifacts[name] = checksum
                    verify(artifacts)
                    prepared = item['stages']['prepare']['result']
                    verify(prepared['source_pins'])
                    verify(prepared.get('cache_pins', {}))
                if item.get('status') == 'failed' and not retry_failed:
                    continue
                if (retry_failed and item.get('status') == 'failed'
                        and item['errors'][-1]['stage'] == 'simulate'
                        and ((Path(item['stages']['prepare']['result']['run']).parent / 'execution').exists()
                             or (Path(item['stages']['prepare']['result']['run']) / 'native').exists())):
                    if stage not in ('all', 'resume'):
                        raise ValueError('Interrupted native run preserved; use resume --retry-failed for a new attempt')
                    item.setdefault('prior_stage_records', []).append(item['stages'])
                    item['stages'] = {}
                for current in stages:
                    if current in item['stages']:
                        continue
                    item['active_stage'] = current
                    atomic(path, ledger)
                    if current == 'prepare':
                        folder = output / 'jobs' / job_id / f'attempt_{len(item["attempts"])+1:03d}'
                        item['attempts'].append(str(folder)); atomic(path, ledger)
                        result = prepare_job(variant, condition, folder, job_id)
                        # Prepare the unmodified baseline through the same
                        # compiler when this invocation selected only a variant.
                        # No simulation is needed to detect an ignored parameter.
                        if condition['changes']:
                            reference = next((j['stages']['prepare']['result']['effective_physics']
                                              for k, j in ledger['jobs'].items()
                                              if 'prepare' in j['stages'] and not next(c for c in doc['conditions'] if c['id']==k)['changes']), None)
                            if reference is None:
                                baseline_doc = copy.deepcopy(doc)
                                baseline_doc['conditions'] = [dict(id='resolved', changes={}, derived_impacts=[])]
                                reference_run = adapters.prepare(baseline_doc, folder / 'baseline_preflight', job_id+'_reference')
                                reference = adapters.physical_signature(doc['backend']['kind'], reference_run)
                            effective_diff = differences(reference, result['effective_physics'])
                            if not effective_diff:
                                raise ValueError('Condition was ignored by backend preparation')
                            comparison['actual_prepared_physics_differences'] = effective_diff
                        write_json(folder / 'intervention.json', comparison)
                        result['intervention'] = comparison
                        seal(item, current, result, hashes(folder))
                    else:
                        if 'prepare' not in item['stages']:
                            raise ValueError('Run prepare first')
                        p = item['stages']['prepare']['result']; run_dir = Path(p['run'])
                        verify(p['source_pins'])
                        if 'cache' in p:
                            # Full cache integrity was checked at binding (and
                            # once on resume). Imported native/prepared bytes
                            # are checked against those pins after copying.
                            # Do not reread thousands of obsolete packaged
                            # geometry files at every subsequent local stage.
                            verify({p['cache']['source_manifest']: p['cache']['manifest_sha256']})
                        if current == 'simulate':
                            if 'cache' in p:
                                if variant['backend']['kind'] != 'rigid':
                                    adopt_native_cache(p, run_dir)
                                seal(item, current, dict(reused=True, binding=p['cache']), hashes(run_dir))
                            else:
                                # Refuse reusing an incomplete native run. Retrying
                                # requires a new output; no destructive cleanup.
                                logs = run_dir.parent / 'execution'
                                adapters.simulate(variant, run_dir, logs)
                                seal(item, current, dict(reused=False), hashes(run_dir))
                        elif current == 'package':
                            if 'cache' in p and variant['backend']['kind'] == 'rigid':
                                ep_path = Path(p['cache']['episode'])
                            else:
                                if 'cache' in p:
                                    adopt_native_cache(p, run_dir)
                                elif 'simulate' not in item['stages']:
                                    raise ValueError('Run simulate first')
                                package_run = run_dir
                                if 'cache' in p:
                                    package_run = adapters.cache_package_context(run_dir, p['cache'],
                                        run_dir.parent / f'cache_package_{len(item["errors"]):03d}' / run_dir.name,
                                        source_pins=p['cache_pins'])
                                if 'cache' not in p and ((run_dir / 'episode').exists() or (run_dir / 'episode.physics.json').exists()
                                        or (run_dir.parent / 'package.log').exists()):
                                    # Retry packaging without rerunning physics or
                                    # overwriting an incomplete prior package.
                                    retry_root = run_dir.parent / f'repackage_{len(item["errors"]):03d}'
                                    package_run = retry_root / 'physics'
                                    shutil.copytree(run_dir, package_run,
                                        ignore=shutil.ignore_patterns('episode', 'episode.physics.json', 'contract_review.json'))
                                ep_path = adapters.package(variant, package_run, package_run.parent)
                            ep = open_episode(ep_path, require_complete=False)
                            verify_alignment(ep, variant['timing'])
                            seal(item, current, dict(episode=str(ep_path)), hashes(ep_path))
                        elif current == 'audit':
                            if 'package' not in item['stages']:
                                raise ValueError('Run package first (existing caches also require binding)')
                            ep_path = Path(item['stages']['package']['result']['episode'])
                            ep = open_episode(ep_path, require_complete=False)
                            report = review(ep, variant)
                            report['intervention'] = p['intervention']
                            dest = output / 'reviews' / job_id / f'review_{len(item["errors"]):03d}.json'
                            write_json(dest, report)
                            seal(item, current, dict(report=str(dest), physical_outcome=report.get('measured_outcome')),
                                 {str(dest): file_hash(dest)})
                        elif current == 'observe':
                            if 'audit' not in item['stages']:
                                raise ValueError('Run audit first')
                            ep_path = Path(item['stages']['package']['result']['episode'])
                            manifest = manifest_at(ep_path)
                            job = dict(id=job_id, episode_id=job_id, episode=str(ep_path), manifest=manifest.name,
                                       manifest_sha256=file_hash(manifest), family=variant['phenomenon'],
                                       group=SUPPORT[variant['phenomenon']][1], condition=condition['changes'],
                                       comparison_variable=comparison['declared_subtrees'],
                                       observation_camera=variant['observations']['camera'],
                                       observation_hz=variant['observations']['hz'],
                                       required_visible_ids=variant['observations'].get('required_visible_ids', []))
                            dest = output / 'data' / job_id / f'observation_{len(item["errors"]):03d}'
                            result = collect(job, dest)
                            complete = open_episode(result['episode'])
                            if complete.manifest['episode_id'] != job_id:
                                raise ValueError('Packaged episode identity differs from registry identity')
                            alignment = verify_alignment(complete, variant['timing'], observed=True)
                            expected = round(variant['timing']['duration_s'] * variant['observations']['hz']) + 1
                            if alignment['counts']['observations'] != expected:
                                raise ValueError('Incorrect observation rate or missing terminal observation')
                            write_json(dest / 'alignment.json', alignment)
                            result['experiment_review'] = item['stages']['audit']['result']['report']
                            result['intervention'] = p['intervention']
                            seal(item, current, result, hashes(dest))
                        elif current == 'register':
                            if 'observe' not in item['stages']:
                                raise ValueError('Run observe first')
                            seal(item, current, dict(registered=True), {})
                    item['status'] = current + '_complete'
                    item.pop('active_stage', None)
                    atomic(path, ledger)
                    print(job_id, current, 'complete', flush=True)
            except Exception as exc:
                item['status'] = 'failed'
                item['errors'].append(dict(stage=current, error=repr(exc), traceback=traceback.format_exc()))
                atomic(path, ledger)
                print(job_id, current, 'FAILED', repr(exc), flush=True)
        # Registry revisions preserve prior partial registrations; failed jobs
        # remain in the ledger regardless of whether records can be read.
        if 'register' in stages:
            entries = [j['stages']['observe']['result'] for j in ledger['jobs'].values()
                       if 'register' in j['stages'] and j.get('status') != 'failed']
            revision = len(list(output.glob('index_*.json'))) + 1
            index = output / f'index_{revision:03d}.json'
            catalog = output / f'catalog_{revision:03d}.json'
            write_json(index, dict(episodes=entries))
            verify({catalog_pin['path']:catalog_pin['sha256']})
            build([Path(catalog_pin['path']), index], [], catalog, verify_streams=True)
            ledger['catalog'] = str(catalog); atomic(path, ledger)
        return ledger
    finally:
        lock.unlink()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--experiment', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--stage', choices=(*STAGES, 'all', 'resume'), default='all')
    parser.add_argument('--ids', nargs='+', help='condition IDs, not episode IDs')
    parser.add_argument('--retry-failed', action='store_true', help='Retry a failed CPU stage; preserve previous partial artifacts')
    parser.add_argument('--current-catalog', type=Path, help='Explicit current mainline catalog; pin consistently across stages')
    args = parser.parse_args()
    result = run(args.experiment, args.output, args.stage, args.ids, args.retry_failed, args.current_catalog)
    if any(j['status'] == 'failed' for j in result['jobs'].values()):
        raise SystemExit(1)


if __name__ == '__main__':
    main()

"""Freeze the plastic delivery and copy its unmodified, unpackaged run locally.

No backend import, kernel execution, simulation or rendering. Packaging is a
separate explicit call to the fixed public entry after these checks.
"""
import argparse,ast,hashlib,io,shutil,subprocess,tarfile,zipfile
from pathlib import Path
from world_model_dataset.io import read_json,write_json,file_hash
CODE='cd37c9f1d3edbecc2aeadd256d4129344d9b5fd7'
DELIVERY='2e4853d4c60f7689513327b55799ee7ed3d47fa2'
PREFIX='experiments/material_response/'


def export(workspace,revision,paths,destination):
    data=subprocess.check_output(['git','-C',str(workspace),'archive',revision,*paths])
    with tarfile.open(fileobj=io.BytesIO(data)) as archive:
        for member in archive:
            if not member.isfile():continue
            target=destination/member.name
            if not target.resolve().is_relative_to(destination.resolve()):raise ValueError('Archive path escapes snapshot')
            target.parent.mkdir(parents=True,exist_ok=True);content=archive.extractfile(member).read()
            if target.exists():
                if target.read_bytes()!=content:raise ValueError('Existing snapshot differs: '+str(target))
            else:target.write_bytes(content)


def pin(workspace,snapshot,output):
    snapshot.mkdir(parents=True,exist_ok=True);output.mkdir(parents=True,exist_ok=True)
    if (output/'backend_delivery.json').exists() or (output/'external_box').exists():raise FileExistsError('Delivery already materialized')
    export(workspace,CODE,[PREFIX],snapshot)
    export(workspace,DELIVERY,['evidence/plastic_zero_support',PREFIX+'scene_input/PLASTIC_ZERO_SUPPORT_RESULTS.md',PREFIX+'scene_input/PLASTIC_ZERO_SUPPORT_STRATEGY.md'],snapshot/'delivery')
    entry=snapshot/PREFIX/'scene_input';pins={str(p.resolve()):file_hash(p) for p in snapshot.rglob('*') if p.is_file()}
    node=ast.parse((entry/'plastic_zero_support.py').read_text())
    guard=next(ast.literal_eval(n.value) for n in node.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='PINNED_SOURCES' for t in n.targets))
    cases={}
    for case in ('box','regression'):
        manifest_path=snapshot/'delivery/evidence/plastic_zero_support'/f'{case}_artifact_manifest.json'
        manifest=read_json(manifest_path);run=Path(manifest['root']);binding=read_json(run/'source_binding.json')
        for name,checksum in manifest['files_sha256'].items():
            path=run/name
            if file_hash(path)!=checksum:raise ValueError('Delivered file changed: '+str(path))
            pins[str(path)]=checksum
        for name,checksum in binding['unchanged_inputs'].items():
            if file_hash(run/'prepared'/name)!=checksum:raise ValueError('Original prepared input changed')
        execution=read_json(run/'execution.json')
        historical={}
        for name,checksum in execution['sources'].items():
            if file_hash(run/'prepared'/name)!=checksum:raise ValueError('Actual prepared execution source changed: '+name)
            if file_hash(entry/name)!=checksum:
                if case=='box' or name in binding['executed_overlays']:raise ValueError('Actual execution source differs from fixed code: '+name)
                historical[name]=dict(actual_sha256=checksum,fixed_entry_sha256=file_hash(entry/name),semantics='retained historical prepared helper in compatibility run')
        for name,checksum in binding['executed_overlays'].items():
            if execution['sources'].get(name)!=checksum:raise ValueError('Overlay/execution binding mismatch')
        runtime=read_json(run/'native/runtime_sources.json');archive=run/'native'/runtime['source_archive']
        if file_hash(archive)!=runtime['source_archive_sha256']:raise ValueError('Runtime archive mismatch')
        with zipfile.ZipFile(archive) as z:
            if set(z.namelist())!=set(runtime['source_sha256']):raise ValueError('Runtime inventory mismatch')
            for name,checksum in runtime['source_sha256'].items():
                if hashlib.sha256(z.read(name)).hexdigest()!=checksum:raise ValueError('Runtime source mismatch')
            for name,checksum in guard.items():
                if hashlib.sha256(z.read('newton/'+name)).hexdigest()!=checksum:raise ValueError('Fixed Newton protection mismatch')
        if runtime['newton_git']!='a980d1dc6916d9cb46716ebf330051b063c83315':raise ValueError('Unexpected Newton revision')
        cases[case]=dict(external_run=str(run),artifact_manifest=str(manifest_path),files_verified=len(manifest['files_sha256']),
            original_prepared_source=binding['source_prepared'],executed_overlays=binding['executed_overlays'],historical_prepared_helpers=historical,
            native_sha256=file_hash(run/'native/states.npz'),newton_git=runtime['newton_git'],newton_dirty=runtime['newton_dirty'])
    bootstrap=output/'external_box';source=Path(cases['box']['external_run']);shutil.copytree(source,bootstrap)
    manifest=read_json(cases['box']['artifact_manifest'])
    for name,checksum in manifest['files_sha256'].items():
        if file_hash(bootstrap/name)!=checksum:raise ValueError('Copied artifact changed: '+name)
    write_json(bootstrap/'import_origin.json',dict(external_run=str(source),actual_code_revision=CODE,delivery_revision=DELIVERY,
        source_manifest_sha256=file_hash(cases['box']['artifact_manifest']),files_sha256=manifest['files_sha256'],physics_rerun=False))
    write_json(output/'backend_delivery.json',dict(source_revision=CODE,delivery_revision=DELIVERY,source_pins=pins,cases=cases,
        fixed_entry=str(entry/'material_entry.py'),bootstrap_run=str(bootstrap),physics_rerun=False,
        newton_guard_verified_against_actual_archive=guard,training_admission=False))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--workspace',type=Path,required=True);p.add_argument('--snapshot',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();pin(a.workspace.resolve(),a.snapshot.resolve(),a.output.resolve())

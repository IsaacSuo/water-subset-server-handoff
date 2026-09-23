"""Pin wrap delivery 7011d17 and validate external evidence; never run physics.

Exports committed backend files into this entry workspace. The new request is
explicit: old profiles, requests, failures and cached origins are not changed.
"""
import argparse, hashlib, subprocess, zipfile
from pathlib import Path
import numpy as np
from world_model_dataset.io import read_json, write_json, file_hash

REVISION='7011d1771058b19eef1a0d3cb93c74a1e6952c3f'
PREFIX='experiments/material_response/'


def verify_maps(prepared):
    summaries={}
    for path in prepared.glob('*_collision_policy.json'):
        p=read_json(path);name=path.name.removesuffix('_collision_policy.json')
        original=prepared/p['original_copy'];mapping=prepared/p['mapping']
        if file_hash(original)!=p['original_sha256'] or file_hash(mapping)!=p['mapping_sha256']:
            raise ValueError('Original/mapping hash mismatch: '+name)
        with np.load(original) as o,np.load(mapping) as m,np.load(prepared/(name+'.npz')) as b:
            v=o['vertices'];f=o['triangles'];vf=v.astype(np.float64)
            zero=np.all(np.cross(vf[f[:,1]]-vf[f[:,0]],vf[f[:,2]]-vf[f[:,0]])==0,axis=1)
            keep=np.flatnonzero(~zero);excluded=np.flatnonzero(zero)
            np.testing.assert_array_equal(m['backend_to_original_face'],keep)
            np.testing.assert_array_equal(m['excluded_original_face'],excluded)
            np.testing.assert_array_equal(m['excluded_vertex_indices'],f[excluded])
            np.testing.assert_array_equal(b['vertices'],v.astype(np.float32))
            np.testing.assert_array_equal(b['triangles'],f[keep])
            owner=np.full(len(f),-1);local=np.full(len(f),-1)
            for i,obj in enumerate(p['objects']):
                start=obj['triangle_start'];end=start+obj['triangle_count']
                if start<0 or end>len(f) or (owner[start:end]>=0).any():raise ValueError('Invalid object ranges')
                owner[start:end]=i;local[start:end]=np.arange(end-start)
            if (owner<0).any():raise ValueError('Incomplete object mapping')
            for stem,idx in [('backend_to',keep),('excluded',excluded)]:
                np.testing.assert_array_equal(m[stem+'_object'],owner[idx])
                np.testing.assert_array_equal(m[stem+'_local_face'],local[idx])
            if p['excluded_original_faces']!=excluded.tolist() or p['near_zero_nonzero_faces'] or p['native_precision_rejected_faces']:
                raise ValueError('Policy contradicts admitted mesh')
            summaries[name]=dict(original_faces=len(f),backend_faces=len(keep),excluded_faces=len(excluded),
                source_sha256=p['original_sha256'],mapping_sha256=p['mapping_sha256'],objects=len(p['objects']))
    if not summaries:raise ValueError('Missing collision policies')
    return summaries


def pin_delivery(workspace,destination,report):
    def git(*args):return subprocess.check_output(['git','-C',str(workspace),*args])
    commit=git('rev-parse',REVISION).decode().strip()
    destination.mkdir(parents=True,exist_ok=False)
    pins={}
    names=git('ls-tree','-r','--name-only',commit,PREFIX,'evidence').decode().splitlines()
    for name in names:
        if not (name.endswith('.py') or name.endswith('.json') or name.endswith('WRAP_ZERO_AREA.md')):continue
        data=git('show',commit+':'+name);p=destination/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(data)
        pins[str(p.resolve())]=file_hash(p)
    for name,checksum in read_json(destination/'evidence/wrap_zero_area/artifacts.json').items():
        path=workspace/name
        if file_hash(path)!=checksum:raise ValueError('Delivered evidence mismatch: '+name)
        pins[str(path.resolve())]=checksum
    run=workspace/'output/wrap_original_v2';prepared=run/'prepared'
    entry=destination/PREFIX/'scene_input'
    execution=read_json(run/'execution.json')
    for name,checksum in execution['sources'].items():
        candidate=entry/name if name!='newton_io.py' else destination/PREFIX/'generation/vendor/experiments/newton_backend_probe/common/newton_io.py'
        if file_hash(prepared/name)!=checksum or file_hash(candidate)!=checksum:raise ValueError('Executed source differs from fixed commit: '+name)
    runtime=read_json(run/'native/runtime_sources.json');archive=run/'native'/runtime['source_archive']
    if file_hash(archive)!=runtime['source_archive_sha256']:raise ValueError('Runtime archive changed')
    with zipfile.ZipFile(archive) as z:
        if set(z.namelist())!=set(runtime['source_sha256']):raise ValueError('Runtime archive inventory mismatch')
        for name,checksum in runtime['source_sha256'].items():
            if hashlib.sha256(z.read(name)).hexdigest()!=checksum:raise ValueError('Runtime source mismatch: '+name)
    for path in [run/'execution.json',run/'requested_config.json',run/'runtime_request.json',run/'physical_review.json',*prepared.iterdir()]:
        if path.is_file():pins[str(path.resolve())]=file_hash(path)
    mapping=verify_maps(prepared)
    write_json(report,dict(source_revision=commit,workspace=str(workspace),observed_head=git('rev-parse','HEAD').decode().strip(),
        source_pins=pins,collision_mapping=mapping,execution_origin='material branch existing wrap_original_v2',
        runtime_archive_verified=True,newton_git=runtime['newton_git'],physics_rerun=False,
        fixed_entry=str(entry/'material_entry.py'),external_run=str(run),training_admission=False))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--workspace',type=Path,required=True)
    p.add_argument('--snapshot',type=Path,required=True);p.add_argument('--report',type=Path,required=True)
    a=p.parse_args();pin_delivery(a.workspace.resolve(),a.snapshot.resolve(),a.report.resolve())

"""Map existing B/C public inputs into the resumable experiment entry.

Only contract validation is executed, in an isolated Python subprocess. No
material prepare/simulate/package call is made by this mapper. B/C are mapped
existing configurations, not object/region construction capabilities.
"""
import argparse
import copy
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

from .experiment_contract import INPUT_FIELDS, VERSION, native_config, validate_common
from .io import file_hash, read_json, write_json

REVISIONS = {'cloth_drag':'2f120019808e369b074ddb16e6a0f754f4f3c79a',
             'rope_finite_load':'ca39d3875c933c9ad759a0e6c9c52fbf22d159e2'}
C_DISABLED_REVISION='05aabd376dc32916bb5541406ca67378da9fbc72'


def check_native_contract(entry, config):
    """Audited input_contract.normalize only; does not import material_entry."""
    script = ('import json,sys; sys.path.insert(0,sys.argv[1]); '
              'from input_contract import normalize; '
              'print(json.dumps(normalize(json.load(sys.stdin)),allow_nan=False))')
    before={p:file_hash(p) for p in Path(entry).parent.glob('*.py')}
    result=subprocess.run([sys.executable,'-B','-c',script,str(Path(entry).parent)],
                          input=json.dumps(config),text=True,capture_output=True,check=True)
    if any(file_hash(p)!=checksum for p,checksum in before.items()):
        raise ValueError('Public contract source changed during validation')
    return json.loads(result.stdout)


def map_existing(phenomenon, workspace, config_path, runtime, bounds, experiment_id, pinned_source=None, source_revision=None):
    workspace=Path(workspace).resolve(); config_path=Path(config_path).resolve()
    expected=source_revision or REVISIONS[phenomenon]
    allowed={REVISIONS[phenomenon]} | ({C_DISABLED_REVISION} if phenomenon=='rope_finite_load' else set())
    if expected not in allowed: raise ValueError('Public material revision has not been reviewed: '+expected)
    actual=subprocess.check_output(['git','-C',str(workspace),'rev-parse','HEAD'],text=True).strip()
    if actual != expected and not pinned_source:
        raise ValueError('Material revision differs: expected '+expected+', got '+actual+'; supply exact pinned_source')
    entry=workspace/'experiments/material_response/scene_input/material_entry.py'
    tracked=subprocess.check_output(['git','-C',str(workspace),'diff','HEAD','--',str(entry.parent)],text=True)
    if pinned_source:
        import hashlib
        root=Path(pinned_source).resolve()
        names=subprocess.check_output(['git','-C',str(workspace),'ls-tree','-r','--name-only',expected,
            'experiments/material_response'],text=True).splitlines()
        for name in names:
            if not name.endswith('.py'): continue
            original=subprocess.check_output(['git','-C',str(workspace),'show',expected+':'+name])
            if hashlib.sha256(original).hexdigest()!=file_hash(root/name):
                raise ValueError('Pinned material snapshot differs: '+name)
        entry=root/'experiments/material_response/scene_input/material_entry.py'
    elif tracked.strip():
        raise ValueError('Material public source has uncommitted changes; provide an exact pinned_source export of '+expected)
    cfg=read_json(config_path)
    # B public examples omit some versioned defaults; native normalizer supplies them.
    cfg=check_native_contract(entry,cfg)
    kind=cfg['kind']
    if (kind,phenomenon) not in (('cloth','cloth_drag'),('rope','rope_finite_load')):
        raise ValueError('Wrong material input kind')
    if kind=='cloth' and 'gripper' not in cfg: raise ValueError('B needs finite gripper and attachment selection')
    if kind=='rope' and not {'loads','load_control'} <= cfg.keys(): raise ValueError('C needs loads and finite load_control')
    for mesh in cfg['environment']:
        mesh['path']=str((config_path.parent/mesh['path']).resolve())
    for field in ('cloth','object'):
        if 'path' in cfg.get(field,{}): cfg[field]['path']=str((config_path.parent/cfg[field]['path']).resolve())
    cfg['source_records']={k:str((config_path.parent/v).resolve()) for k,v in cfg.get('source_records',{}).items()}
    cfg['source_records']['public_request']=str(config_path)
    b=np.asarray(bounds,float); target=b.mean(0); span=float(np.max(b[1]-b[0]))
    disabled=cfg.get('load_control',{}).get('enabled') is False
    doc=dict(format=VERSION,id=experiment_id,phenomenon=phenomenon,
        backend=dict(kind=kind,entry=str(entry),runtime=str(Path(runtime).resolve())),
        scene=dict(id='original_material_region',region_bounds_m=bounds,units='m',up_axis='Z',frame='original_world',
                   collision=dict(meshes=cfg['environment']),source_records=cfg['source_records']),
        input={k:copy.deepcopy(cfg[k]) for k in INPUT_FIELDS[kind] if k in cfg},
        control='finite_gripper' if kind=='cloth' else 'finite_load_disabled' if disabled else 'finite_load',
        timing={k:cfg[k] for k in ('duration_s','physics_hz','state_hz')},
        observations=dict(hz=cfg.get('observation_hz',10),camera=dict(target_m=target.tolist(),
            position_m=(target+span*np.array([1,-1,.8])).tolist(),ortho_scale_m=span*1.5)),
        conditions=[dict(id='baseline',changes={},derived_impacts=[])])
    validate_common(doc); normalized=check_native_contract(entry,native_config(doc))
    physical_keys=set(INPUT_FIELDS[kind]) | {'environment','duration_s','physics_hz','state_hz'}
    if any(normalized.get(k)!=cfg.get(k) for k in physical_keys):
        raise ValueError('Bridge changed native physical input')
    paths=[config_path,Path(runtime),*entry.parent.glob('*.py')]
    paths += [Path(m['path']) for m in cfg['environment']] + [Path(p) for p in cfg['source_records'].values()]
    report=dict(format='material-bridge/1',status='public_input_mapped_and_native_contract_checked',construction='forward_only',
        missing_construction='new object/region attachment placement and load connection rules remain owned by material generator',
        source_commit=expected,observed_workspace_head=actual,source_pins={str(p):file_hash(p) for p in paths},
        physical_input_preserved=True,source_workspace_modified=False,
        backend='PhysX cloth finite gripper' if kind=='cloth' else 'Newton SolverVBD',
        youngs_modulus_Pa=cfg.get('material',{}).get('youngs_modulus_Pa'),
        actuation_semantics='disabled throughout; target is inactive reference' if disabled else 'finite active control',
        physical_evidence='no native/episode for disabled request; unverified' if disabled else 'existing nearby cache evidence only',
        comparison_limit='active initial hold already applies force; pre-pull differences may be interventions, not only random variation',
        execution_stages=['prepare','simulate','package','audit','observe','register'],
        executed_stages=[],contact_force='unavailable',attachment_reaction='unavailable',
        rope_native_tension='unavailable',training_admission=False)
    return doc,report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--request',type=Path,required=True); p.add_argument('--output',type=Path,required=True)
    a=p.parse_args(); r=read_json(a.request)
    doc,report=map_existing(**r)
    a.output.mkdir(parents=True,exist_ok=False)
    binding=a.output/'bridge.json'; write_json(binding,report)
    doc['scene']['source_records']['material_bridge']=str(binding.resolve())
    write_json(a.output/'experiment.json',doc)
    write_json(a.output/'backend_input.json',native_config(doc))
    print(str(a.output/'experiment.json'))


if __name__=='__main__': main()

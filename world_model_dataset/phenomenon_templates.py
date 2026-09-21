"""Expand explicit asset/condition lists through the existing real-scene recipe.

CPU preparation only. Environment edits and outcome selection are not template
parameters. Each compiled plan pins its physical inputs and generator sources.
"""
import argparse
import copy
from pathlib import Path
import re

import numpy as np

from .causal_runner import ROOT
from .io import file_hash, read_json, write_json
from .real_scene_batch import recipe


BODY_FIELDS = {'size_m', 'mass_kg', 'density_kg_m3', 'xy_m', 'rotation_deg',
               'velocity_m_s', 'angular_velocity_rad_s', 'material', 'clearance_m'}
MATERIAL_FIELDS = {'static_friction', 'dynamic_friction', 'restitution'}


def identifier(value):
    if not isinstance(value,str) or not re.fullmatch(r'[a-zA-Z0-9_]+',value):
        raise ValueError('Expected a plain identifier: '+str(value))
    return value


def apply_body(body, changes):
    if set(changes)-BODY_FIELDS:raise ValueError('Unsupported body changes: '+str(set(changes)-BODY_FIELDS))
    for key,value in changes.items():
        if key=='material':
            if not isinstance(value,dict) or set(value)-MATERIAL_FIELDS:raise ValueError('Unsupported material fields')
            body.setdefault('material',{}).update(value)
        else:body[key]=copy.deepcopy(value)
    for key in BODY_FIELDS & body.keys():
        values=list(body[key].values()) if key=='material' else body[key]
        if not np.isfinite(np.asarray(values,dtype=float)).all():raise ValueError('Nonfinite body parameter: '+key)
    for key in ('size_m','mass_kg','density_kg_m3'):
        if key in body and body[key]<=0:raise ValueError('Expected positive '+key)
    for key,count in (('xy_m',2),('rotation_deg',3),('velocity_m_s',3),('angular_velocity_rad_s',3)):
        if key in body and np.asarray(body[key]).shape!=(count,):raise ValueError('Invalid '+key)
    if body.get('mass_kg') is not None and body.get('density_kg_m3') is not None:
        raise ValueError('Specify mass or density, not both')
    material=body.get('material',{})
    if any(v<0 for v in material.values()) or material.get('restitution',0)>1:
        raise ValueError('Invalid friction or restitution')


def expand(template, base):
    """Pure expansion; the same implementation handles every listed asset."""
    if len(base['shots'])!=1:raise ValueError('Template base must contain exactly one shot')
    if not template['assets'] or not template['conditions']:raise ValueError('Empty asset or condition list')
    seen=set();results=[]
    for asset in template['assets']:
        identifier(asset)
        for condition in template['conditions']:
            oid='_'.join(identifier(s) for s in (template['id'],asset,condition['id']))
            if oid in seen:raise ValueError('Duplicate generated ID: '+oid)
            seen.add(oid);spec=copy.deepcopy(base);shot=spec['shots'][0]
            bodies=[b for b in shot['bodies'] if b['id']==template['subject_id']]
            if len(bodies)!=1:raise ValueError('Template requires one matching subject')
            body=bodies[0];body['asset']=asset
            apply_body(body,template.get('body_defaults',{}))
            apply_body(body,condition['body'])
            shot['id']=oid;shot['title']=template['title']+' / '+condition['id']
            shot['comparison']=template['comparison_variable']
            results.append((spec,dict(id=oid,backend='rigid',family=template['family'],group=shot['group'],
                condition=copy.deepcopy(condition['body']),asset=asset,template_id=template['id'],
                comparison_variable=template['comparison_variable'],
                observation_camera=shot['camera'],observation_hz=10,
                required_visible_ids=[template['subject_id']])))
    return results


def compile_plan(design_path, output):
    design_path=Path(design_path).resolve();design=read_json(design_path)
    output=Path(output).resolve()
    if output.exists():raise FileExistsError('Preserve prior compiled plans; choose a new output')
    pins={};prepared=[];ids=set()
    def pin(path,expected=None):
        path=Path(path).resolve();digest=file_hash(path)
        if expected is not None and digest!=expected:raise ValueError('Changed input: '+str(path))
        if str(path) in pins and pins[str(path)]!=digest:raise ValueError('Input changed during preparation')
        pins[str(path)]=digest
    pin(design_path)
    for path in (ROOT/'world_model_dataset').glob('*.py'):pin(path)
    # Include inherited physics profiles and manifest/schema defaults.
    for path in (ROOT/'configs/dataset/v0_2').glob('*.json'):pin(path)
    for path in (ROOT/'configs/dataset/v0_2/examples').glob('*.json'):pin(path)
    pin(ROOT/'soft_body/tet_quality.py')
    for template in design['templates']:
        base_path=ROOT/template['base_spec'];pin(base_path);base=read_json(base_path)
        scene_path=ROOT/base['scene_export'];pin(scene_path);scene=read_json(scene_path)
        for record in scene['groups'].values():pin(scene_path.parent/record['path'],record['sha256'])
        for asset in template['assets']:
            package=Path(base['exploration_root'])/base['library']/identifier(asset)
            pin(package/'asset.json');metadata=read_json(package/'asset.json')
            pin(package/metadata['geometry'],metadata['geometry_sha256'])
        pin(Path(base['exploration_root'])/'experiments/physical_assets/prepare.py')
        for spec,job in expand(template,base):
            if job['id'] in ids:raise ValueError('Duplicate job ID: '+job['id'])
            ids.add(job['id']);config,manifest=recipe(spec,spec['shots'][0])
            subjects=[b for b in manifest['system']['bodies'] if b['role']=='subject']
            physics={b['instance_id']:config['physics_profiles'][b['physics_profile_id']] for b in subjects}
            prepared.append((spec,job,dict(placements=config['real_scene_design']['placements'],
                physics=physics,initial_state=manifest['initial_state']['body_states'],
                environment_unchanged=True,control='none',collision='existing SDF',
                limits='support ray and input checks; no swept collision or outcome guarantee')))
    # Refuse changes detected while CPU recipe preparation was in progress.
    for path,digest in pins.items():
        if file_hash(path)!=digest:raise ValueError('Input changed during preparation: '+path)
    output.mkdir(parents=True,exist_ok=False);jobs=[];reviews={}
    write_json(output/'design.json',design)
    for spec,job,review in prepared:
        path=output/(job['id']+'.json');write_json(path,spec);pin(path)
        job['config']=str(path);jobs.append(job);reviews[job['id']]=review
    plan=dict(version='phenomenon-batch/1',purpose=design['purpose'],input_sources=pins,jobs=jobs)
    write_json(output/'batch.json',plan)
    write_json(output/'preflight.json',dict(jobs=reviews,physics_run=False,training_admission=False))
    return plan


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--design',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();plan=compile_plan(a.design,a.output);print('COMPILED',len(plan['jobs']),'jobs',flush=True)


if __name__=='__main__':main()

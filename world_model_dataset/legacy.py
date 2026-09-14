"""Read-only bridge for the validated multi-object baseline.

This inventories reusable environments, object bindings and fixed-topology
caches. It deliberately does not promote legacy contact-distance summaries to
the new dataset's native point-contact truth.
"""
from __future__ import annotations

from pathlib import Path, PureWindowsPath

from .contract import ROOT
from .io import file_hash,read_json,write_json


DEFAULT_CONFIG=ROOT/'configs/multi_object_scene_experiments.json'
DEFAULT_RUNS=ROOT/'output/multi_object_mixed_material_14/runs'


def inventory(config_path=DEFAULT_CONFIG,runs=DEFAULT_RUNS):
    config=read_json(config_path);runs=Path(runs);rows=[];errors=[]
    for experiment in config['experiments']:
        scene=experiment['scene'];episode=experiment['id']
        directory=runs/episode/'isaac'/scene
        report_path=directory/'run_complete.json';cache_path=directory/'soft_body_blender.usdc'
        if not report_path.is_file() or not cache_path.is_file():
            errors.append(f'{episode}: missing report or fixed-topology cache');continue
        report=read_json(report_path);actual=report.get('physics_bodies') or [];expected=experiment['bodies']
        local=[]
        if not report.get('valid'):local.append('legacy physics report invalid')
        if len(actual)!=len(expected):local.append('body count mismatch')
        bindings=[]
        for index,(want,have) in enumerate(zip(expected,actual)):
            if PureWindowsPath(str(have.get('model',''))).name!=want['model']:local.append(f'body {index} geometry mismatch')
            if have.get('physics_profile')!=want['physics_profile']:local.append(f'body {index} physics mismatch')
            if have.get('material_preset')!=want['material_preset']:local.append(f'body {index} appearance mismatch')
            bindings.append(dict(instance_id=f'body_{index}',geometry_asset=want['model'],
                physics_profile_id=want['physics_profile'],appearance_profile_id=want['material_preset'],
                physical_representation=have.get('physics_kind'),collision_representation=have.get('collision_approximation')))
        cache=report.get('blender_animation_cache') or {}
        if not cache.get('valid') or cache.get('body_count')!=len(expected):local.append('fixed-topology cache metadata invalid')
        if cache_path.stat().st_size!=cache.get('bytes'):local.append('fixed-topology cache byte count mismatch')
        if not report.get('interbody_contact_valid'):local.append('legacy geometric interbody contact audit invalid')
        if not report.get('all_bodies_penetration_valid'):local.append('legacy penetration audit invalid')
        environment_asset=ROOT.parent/'scenes'/scene/f'{scene}_sim.usda'
        if not environment_asset.is_file():local.append('environment asset missing')
        errors.extend(f'{episode}: {message}' for message in local)
        rows.append(dict(source_episode_id=episode,event_id='staggered_multi_body_drop_collision',
            environment_id=scene,environment_asset=f'{scene}/{scene}_sim.usda',objects=bindings,
            source_report=dict(path=report_path.relative_to(ROOT).as_posix(),sha256=file_hash(report_path)),
            fixed_topology_cache=dict(path=cache_path.relative_to(ROOT).as_posix(),bytes=cache_path.stat().st_size,
                frames=cache.get('frame_count'),fps=cache.get('frames_per_second'),body_vertex_counts=cache.get('body_vertex_counts'),
                coordinate_conversion=cache.get('coordinate_conversion')),
            legacy_checks=dict(physics_valid=bool(report.get('valid')),interbody_geometric_contact=bool(report.get('interbody_contact_valid')),
                penetration=bool(report.get('all_bodies_penetration_valid'))),valid=not local))
    return dict(schema_version='0.1.0',adapter='read_only_multi_object_fixed_topology_bridge',
        source_config=dict(path=Path(config_path).relative_to(ROOT).as_posix(),sha256=file_hash(config_path)),
        event_id='staggered_multi_body_drop_collision',episode_count=len(rows),body_count=sum(len(r['objects']) for r in rows),
        capabilities={
            'fixed_topology_surface_cache':{'status':'native','source':'legacy Isaac-to-Blender USD cache'},
            'geometric_contact_and_penetration':{'status':'derived','source':'legacy distance/intersection audits'},
            'point_contact_impulse':{'status':'unavailable','reason':'legacy runs did not export native per-point contact impulse; no empty, zero or estimated values substituted'},
            'formal_v0_1_episode':{'status':'unavailable','reason':'requires explicit import/resimulation under the new time/contact/observation contract'},
        },rows=rows,errors=errors,valid=len(rows)==14 and not errors)


def main():
    import argparse,json
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--config',type=Path,default=DEFAULT_CONFIG)
    parser.add_argument('--runs',type=Path,default=DEFAULT_RUNS);parser.add_argument('--output',type=Path)
    args=parser.parse_args();value=inventory(args.config,args.runs)
    if args.output:write_json(args.output,value)
    print(json.dumps({'valid':value['valid'],'episodes':value['episode_count'],'bodies':value['body_count'],'errors':value['errors']},ensure_ascii=False))
    raise SystemExit(0 if value['valid'] else 1)


if __name__=='__main__':main()

"""Small identity-based reuse audit. Geometry/config only, no physical stage calls."""
import copy
from pathlib import Path
import numpy as np

from world_model_dataset.experiment_construct import construct
from world_model_dataset.experiment_contract import native_config,rigid_spec
from world_model_dataset.experiment_material_bridge import check_native_contract
from world_model_dataset.io import read_json,write_json,file_hash,digest

ROOT=Path(__file__).resolve().parents[1]
REQUESTS=ROOT/'configs/dataset/v0_2/construction/reuse'


def check(output):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    cases={}; documents={}; reports={}; requests={}
    for name in ('passage_jug','passage_potato','passage_second_row',
                 'cloth_rectangle','cloth_mesh','cloth_sideboard'):
        request_path=REQUESTS/(name+'.json');request=read_json(request_path)
        profile_path=(request_path.parent/request['profile']).resolve();profile=read_json(profile_path)
        doc,native,report=construct(request,profile)
        folder=output/name;folder.mkdir()
        if doc['backend']['kind']=='rigid':
            # Audited pure CPU authoring function; never call prepare.
            from world_model_dataset.real_scene_batch import recipe
            config,manifest=recipe(native,native['shots'][0])
            write_json(folder/'authored_config.json',config);write_json(folder/'authored_manifest.json',manifest)
            report['backend_contract']='CPU recipe compiled; no prepare or simulation'
            library=doc['input']['asset_library'];subject=doc['input']['participants'][0]
            asset=Path(library['exploration_root'])/library['library']/subject['asset']
            metadata=read_json(asset/'asset.json')
            report['object_identity']=dict(asset=subject['asset'],geometry_sha256=metadata['geometry_sha256'],size_m=subject['size_m'])
        else:
            check_native_contract(doc['backend']['entry'],native)
            report['backend_contract']='native input_contract.normalize checked; no material entry stages'
            for path in Path(doc['backend']['entry']).parent.glob('*.py'):
                report['source_pins'][str(path)]=file_hash(path)
            cloth=native['cloth']
            if cloth['kind']=='mesh':
                with np.load(cloth['path']) as source:
                    vertices=source['vertices'];triangles=source['triangles']
                tf=np.asarray(cloth['world_from_mesh'])
                placed=(np.c_[vertices*cloth.get('scale',1.),np.ones(len(vertices))]@tf.T)[:,:3]
                # Inspect the exact emitted backend transform, not just inferred dimensions.
                bounds=np.asarray(doc['scene']['region_bounds_m'])
                assert np.all(placed>=bounds[0]-1e-7) and np.all(placed<=bounds[1]+1e-7)
                assert np.isclose(placed[:,2].min(),report['calculations']['support_height_m']+
                                  report['calculations']['initial_gap_m'],atol=1e-6)
                report['object_identity']=dict(kind='mesh',source_sha256=file_hash(cloth['path']),
                    vertices=len(vertices),triangles=len(triangles),scale=cloth['scale'],
                    topology_sha256=digest(triangles.tolist()),topology_preserved=True,
                    transformed_bounds_m=[placed.min(0).tolist(),placed.max(0).tolist()])
            else:
                nx,ny=cloth['cells'];report['object_identity']=dict(kind='generated_rectangle',vertices=(nx+1)*(ny+1),triangles=2*nx*ny)
        doc['scene']['source_records']['construction']=str((folder/'construction.json').resolve())
        report['source_pins'][str(request_path)]=file_hash(request_path)
        report['source_pins'][str(profile_path)]=file_hash(profile_path)
        for source in (ROOT/'world_model_dataset').glob('experiment_*.py'):
            report['source_pins'][str(source)]=file_hash(source)
        report['source_pins'][str(Path(__file__).resolve())]=file_hash(__file__)
        report['experiment_sha256']=digest(doc)
        native=rigid_spec(doc,doc['id']+'_baseline') if doc['backend']['kind']=='rigid' else native_config(doc)
        write_json(folder/'experiment.json',doc);write_json(folder/'backend_input.json',native)
        write_json(folder/'construction.json',report)
        cases[name]=dict(status='configuration_and_geometry_checked',physical_effect='unverified',
                         report=str(folder/'construction.json'),report_sha256=file_hash(folder/'construction.json'))
        documents[name]=doc;reports[name]=report;requests[name]=request
        print(name,'checked',flush=True)

    # Identity assertions distinguish asset/region reuse from coordinate sensitivity.
    a,b,c=[reports[n] for n in ('passage_jug','passage_potato','passage_second_row')]
    assert a['object_identity']['geometry_sha256']!=b['object_identity']['geometry_sha256']
    assert a['object_identity']['size_m']==b['object_identity']['size_m']
    assert requests['passage_jug']['scene']==requests['passage_potato']['scene']
    assert requests['passage_potato']['object']==requests['passage_second_row']['object']
    def parents(report): return {o['parent'] for o in report['calculations']['restriction_object_candidates']}
    assert parents(b) and parents(c) and parents(b).isdisjoint(parents(c))
    a,b,c=[reports[n] for n in ('cloth_rectangle','cloth_mesh','cloth_sideboard')]
    assert a['object_identity']['vertices']!=b['object_identity']['vertices']
    assert requests['cloth_rectangle']['scene']==requests['cloth_mesh']['scene']
    assert requests['cloth_mesh']['object']==requests['cloth_sideboard']['object']
    assert b['object_identity']['topology_sha256']==c['object_identity']['topology_sha256']
    assert b['support_identity']['mesh_sha256']!=c['support_identity']['mesh_sha256']
    assert b['support_identity']['source_scene_sha256']!=c['support_identity']['source_scene_sha256']
    for left,right in [('passage_jug','passage_potato'),('passage_potato','passage_second_row'),
                       ('cloth_rectangle','cloth_mesh'),('cloth_mesh','cloth_sideboard')]:
        da,db=documents[left],documents[right]
        assert all(da[k]==db[k] for k in ('backend','timing','control','actuation'))
        if left.startswith('cloth'):
            assert {k:v for k,v in da['input'].items() if k!='cloth'}=={k:v for k,v in db['input'].items() if k!='cloth'}
    # Same objects with an inapplicable requirement must be rejected, not rescaled.
    rejected={}
    for name,mutate in [('passage_potato',lambda r:r['conditions'].update(clearance_regime='obstructed')),
                        ('cloth_sideboard',lambda r:r['object'].update(scale=1.))]:
        r=copy.deepcopy(requests[name]);mutate(r)
        try: construct(r,read_json((REQUESTS/r['profile']).resolve()))
        except ValueError as exc: rejected[name]=str(exc)
        else: raise AssertionError('Inapplicable request silently accepted: '+name)
    result=dict(format='construction-reuse-review/1',scope='construction prototype; rearrangement construction missing',
        cases=cases,identity_comparisons='passed',restriction_parents={n:sorted(parents(reports[n])) for n in
            ('passage_potato','passage_second_row')},applicability_rejections=rejected,
        sensitivity_checks_are_not_cross_region_evidence=True,new_physics=False,new_rendering=False,new_catalog=False,
        prior_evidence='existing source geometry only; no physical evidence claimed for new configurations')
    write_json(output/'results.json',result)
    return result


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    check(p.parse_args().output)

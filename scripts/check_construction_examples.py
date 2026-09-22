"""Read original meshes; write construction JSON and native contract results only."""
import copy
from pathlib import Path

from world_model_dataset.experiment_construct import construct, compare_requests
from world_model_dataset.experiment_material_bridge import check_native_contract, map_existing
from world_model_dataset.experiment_contract import native_config, rigid_spec
from world_model_dataset.io import read_json, write_json, file_hash, digest

ROOT=Path(__file__).resolve().parents[1]
REQUESTS=ROOT/'configs/dataset/v0_2/construction'


def main():
    import argparse
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('--output',type=Path,required=True)
    p.add_argument('--legacy-sensitivity',action='store_true',help='Old same-region parameter checks; not cross-region reuse evidence')
    a=p.parse_args()
    if not a.legacy_sensitivity:
        from scripts.check_construction_reuse import check
        check(a.output)
        return
    a.output.mkdir(parents=True,exist_ok=False)
    results={}
    for name in ('passage','cloth','roll','collision_chain','plastic','beam','rope'):
        try:
            r=read_json(REQUESTS/(name+'.json')); profile=read_json(REQUESTS/r['profile'])
            variants=[('baseline',r)]
            resized=copy.deepcopy(r)
            obj=resized['object']
            if isinstance(obj,list): obj[1]['size_m']*=.8
            elif name=='rope': obj['length_m']*=.8
            elif isinstance(obj['size_m'],list): obj['size_m']=[x*.8 for x in obj['size_m']]
            else: obj['size_m']*=.8
            variants.append(('resized',resized))
            selected=copy.deepcopy(r); selected['scene']['region_bounds_m'][0][1]+=.08
            variants.append(('same_region_boundary_shift',selected))
            condition=copy.deepcopy(r)
            key={'passage':'speed_m_s','cloth':'overhang_fraction','roll':'spin_ratio','collision_chain':'gap_ratio',
                 'plastic':'drop_height_m','beam':'max_force_n','rope':'clamp_fraction'}[name]
            condition['conditions'][key]*=1.1
            variants.append(('condition',condition))
            records={}
            for label,request in variants:
                doc,native,report=construct(request,profile)
                authored=None
                if doc['backend']['kind']=='rigid':
                    # Audited recipe is CPU mesh/placement authoring only;
                    # do not call causal_runner.prepare or any stage runner.
                    from world_model_dataset.real_scene_batch import recipe
                    authored=recipe(native,native['shots'][0])
                    report['backend_contract']='real_scene_batch.recipe authored config/manifest; no prepare'
                    report['actual_support_placements']=authored[0]['real_scene_design']['placements']
                if doc['backend']['kind']!='rigid':
                    check_native_contract(doc['backend']['entry'],native)
                    report['backend_contract']='external native input_contract.normalize passed (no prepare)'
                    for source in Path(doc['backend']['entry']).parent.glob('*.py'):
                        report['source_pins'][str(source)]=file_hash(source)
                folder=a.output/name/label; folder.mkdir(parents=True)
                if authored:
                    write_json(folder/'authored_config.json',authored[0])
                    write_json(folder/'authored_manifest.json',authored[1])
                doc['scene']['source_records']['construction']=str((folder/'construction.json').resolve())
                native=rigid_spec(doc,doc['id']+'_baseline') if doc['backend']['kind']=='rigid' else native_config(doc)
                write_json(folder/'experiment.json',doc); write_json(folder/'backend_input.json',native)
                report['source_pins'][str(REQUESTS/(name+'.json'))]=file_hash(REQUESTS/(name+'.json'))
                report['source_pins'][str(REQUESTS/r['profile'])]=file_hash(REQUESTS/r['profile'])
                for source in (ROOT/'world_model_dataset').glob('experiment_*.py'):
                    report['source_pins'][str(source)]=file_hash(source)
                report['experiment_sha256']=digest(doc)
                write_json(folder/'semantic_request.json',request)
                write_json(folder/'construction.json',report)
                records[label]=report['calculations']
            comparison=compare_requests(r,condition,profile)
            write_json(a.output/name/'condition_comparison.json',comparison)
            results[name]=dict(status='passed',variants=records)
        except Exception as exc:
            results[name]=dict(status='failed',error=repr(exc))
        print(name,results[name]['status'],results[name].get('error',''),flush=True)
    for name in ('bridge_b_free','bridge_b_blocked','bridge_b_low','bridge_c','bridge_c_disabled'):
        try:
            doc,report=map_existing(**read_json(REQUESTS/(name+'.json')))
            folder=a.output/name; folder.mkdir()
            doc['scene']['source_records']['material_bridge']=str((folder/'bridge.json').resolve())
            write_json(folder/'experiment.json',doc); write_json(folder/'bridge.json',report)
            write_json(folder/'backend_input.json',native_config(doc))
            results[name]=dict(status=report['status'],backend=report['backend'],construction=report['construction'])
        except Exception as exc: results[name]=dict(status='failed',error=repr(exc))
        print(name,results[name]['status'],results[name].get('error',''),flush=True)
    write_json(a.output/'results.json',results)
    if any(r['status']=='failed' for r in results.values()): raise SystemExit(1)


if __name__=='__main__': main()

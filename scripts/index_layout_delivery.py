"""Write a local delivery inventory, never a dataset catalog or admission."""
import argparse
from pathlib import Path
from world_model_dataset.io import read_json,write_json,file_hash,digest

CASES=[
    ('rearrangement','rearrangement','89aeca8','contact rearrangement occurred; top object rolled off; not restabilized'),
    ('cloth_drag','cloth_drag','050f161','finite free drag occurred; blocked/low-force are geometry-only checks'),
    ('rope_load','rope_load','d2b27b8','front load moved and pulled rear load; native tension unavailable'),
    ('rope_wrap','rope_wrap','c975438','prepare rejects unchanged original mesh degeneracies; no physics'),
    ('roll','roll','c975438','translation and rotation on original floor; no pure-rolling or rest claim'),
    ('collision_chain_v2','collision_chain','c975438','interbody contact occurred; intended forward propagation not observed'),
    ('plastic','plastic','c975438','actual MPM execution aborted on nonfinite x; complete native state file absent'),
    ('beam_v2','beam','c975438','gravity sag caused failed sustained plate loading; complete negative outcome retained'),
    ('beam_supported','beam_supported','9fe0bd1','screened shorter beam still failed sustained hold contact; no further sweep'),
    ('rope_v2','rope','c975438','first point fixed; free end draped downward; native residuals retained'),
]


def record(path):
    return dict(path=str(path.resolve()),sha256=file_hash(path))


def build(root):
    result=[]
    repo=Path(__file__).resolve().parents[1]
    for folder,request,commit,outcome in CASES:
        f=root/folder;ledger=read_json(f/'execution/workflow.json');job=ledger['jobs']['baseline']
        report=read_json(f/'generated/construction.json');doc=read_json(f/'generated/experiment.json')
        request_path=repo/f'configs/dataset/v0_2/construction/{request}.json'
        if digest(read_json(request_path))!=report['request_sha256']:
            raise ValueError(f'{folder}: current request differs from sealed construction; use its original snapshot')
        stages=job['stages'];complete='observe' in stages
        entry=dict(case=folder,phenomenon=doc['phenomenon'],implementation_commit=commit,
            request=record(repo/f'configs/dataset/v0_2/construction/{request}.json'),
            generated={name:record(f/'generated'/name) for name in ('experiment.json','backend_input.json','construction.json')},
            workflow=record(f/'execution/workflow.json'),construction=report['constructor'],geometry_checks=report['geometry_checks'],
            native_contract=report['backend_contract'],physical_execution='completed' if 'simulate' in stages else ('started_then_nonfinite_failure' if folder=='plastic' else 'not_started'),
            observed_outcome=outcome,unified_reading='complete' if complete else 'unavailable',
            use_qualification='human_use_review_pending' if complete else 'not_qualified',training_admission=False,
            stage_names=list(stages),errors=job['errors'],attempt_directories=job['attempts'])
        if 'prepare' in stages:
            entry['native_run']=stages['prepare']['result']['run']
            pins=Path(job['attempts'][-1])/'source_pins.json'
            if pins.exists():entry['execution_source_pins']=record(pins)
        if complete:
            entry['episode']=stages['observe']['result']['episode']
            entry['episode_manifest']=record(Path(entry['episode'])/'episode.json')
        for name in ('result_complete.json','result.json','native_details.json','interface_gap.json','failure_report.json','appearance/replay.json'):
            if (f/name).exists():entry.setdefault('evidence',{})[name]=record(f/name)
        result.append(entry)
    return dict(format='local-automatic-layout-delivery/1',base_mainline='2b17ccf',base_catalog='phenomenon_construction_data_v8 (33 episodes)',
        catalog_modified=False,merged=False,pushed=False,training_admission=False,cases=result,
        limits=['No wrapping physics: public original-mesh admission gap','Plastic execution failed before complete native cache',
                'Both beam runs lack sustained hold proximity; gravity-aware native-response layout remains open',
                'All failed/nonexpected outcomes preserved; no convergence, native rope tension or automatic purpose-admission claims'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();write_json(a.output,build(a.root))

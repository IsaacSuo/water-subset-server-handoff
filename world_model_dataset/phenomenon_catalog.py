"""Combine readable phenomenon batches with explicit, non-destructive supersession."""
import argparse
from pathlib import Path

from .causal_loader import open_episode
from .io import file_hash, read_json, write_json


def open_catalog_episode(item):
    """Explicit state-only admission; never weaken the default completed reader."""
    scope=item.get('record_scope','complete')
    if scope not in ('complete','physics_only'):
        raise ValueError('Unknown catalog record_scope: '+str(scope))
    if scope=='complete':
        return open_episode(item['episode'])
    ep=open_episode(item['episode'],require_complete=False)
    m=ep.manifest
    if m['lifecycle'] not in ('draft','completed'):
        raise ValueError('Physics-only record is not admissible: lifecycle')
    records=[m['initial_state']['state'],m['trajectory']['states'],
             m['trajectory']['interaction_annotations'],m['trajectory']['outcomes']]
    program=m['control_program']
    if program['primitive']!='none':
        records.append(program['command_trace'])
        records.extend(c[k] for c in program['controllers'] for k in ('state_trace','effort_trace'))
    if any(r['status']!='available' for r in records):
        raise ValueError('Physics-only record lacks required physical records')
    if m['trajectory']['observations']['status']!='unavailable' or item['observations']!=0:
        raise ValueError('Physics-only record must explicitly declare absent observations')
    for r in records:ep.record_path(r)
    rows=list(ep.states())
    if len(rows)!=item['states'] or len(rows)<2 or abs(rows[-1]['time_s']-rows[0]['time_s']-m['timing']['duration_s'])>1e-8:
        raise ValueError('Physics-only record is incomplete in time; failed prefixes are not complete runs')
    return ep


def build(indices, supersede, output, verify_streams=False):
    episodes={};sources=[];history=[]
    for path in indices:
        path=Path(path).resolve();index=read_json(path)
        sources.append(dict(path=str(path),sha256=file_hash(path)))
        for entry in index.get('superseded',[]):
            previous=next((r for r in history if r['old_episode']['id']==entry['old_episode']['id']),None)
            if previous is not None and previous!=entry:
                raise ValueError('Conflicting supersession: '+entry['old_episode']['id'])
            if previous is None:history.append(entry)
        for item in index['episodes']:
            if item.get('status')=='failed_collection':continue
            if item['id'] in episodes and episodes[item['id']]!=item:
                raise ValueError('Conflicting episode ID: '+item['id'])
            episodes[item['id']]=item
    for entry in history:
        old=entry['old_episode']['id']
        if old in episodes:
            if episodes[old]!=entry['old_episode']:raise ValueError('Changed superseded episode: '+old)
            episodes.pop(old)
    for old,new in supersede:
        if old==new or old not in episodes or new not in episodes:
            raise ValueError('Supersession must name distinct existing old and new episodes')
        history.append(dict(old_episode=episodes.pop(old),replacement_id=new,
                            reason='explicit caller selection; original cache preserved'))
    counts=dict(states=0,observations=0);groups={}
    for item in episodes.values():
        path=Path(item['episode'])
        if file_hash(path/'episode.json')!=item['manifest_sha256']:
            raise ValueError('Changed manifest: '+item['id'])
        ep=open_catalog_episode(item)
        if ep.manifest['trajectory']['states']['sha256']!=item['source_states_sha256']:
            raise ValueError('Changed physical states: '+item['id'])
        actual=dict(states=sum(1 for _ in ep.states()),observations=(0 if item.get('record_scope')=='physics_only' else sum(1 for _ in ep.observations()))) if verify_streams else item
        for key in counts:
            if actual[key]!=item[key]:raise ValueError('Count mismatch: '+item['id']+' '+key)
            counts[key]+=actual[key]
        groups.setdefault(str(item['group']),[]).append(item['id'])
    result=dict(format='phenomenon-pilot/1',episodes=list(episodes.values()),sources=sources,
                superseded=history,counts=counts,groups=groups,
                training=False,splits=False,training_admission=False,
                streams_verified=verify_streams,
                completed_semantics='record completeness; per-episode quality limits remain authoritative')
    write_json(output,result)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--index',type=Path,action='append',required=True)
    p.add_argument('--supersede',action='append',default=[],metavar='OLD=NEW')
    p.add_argument('--output',type=Path,required=True);p.add_argument('--verify-streams',action='store_true')
    a=p.parse_args();pairs=[]
    for item in a.supersede:
        pair=item.split('=')
        if len(pair)!=2:p.error('--supersede requires OLD=NEW')
        pairs.append(pair)
    result=build(a.index,pairs,a.output,a.verify_streams)
    print(len(result['episodes']),'episodes',result['counts'])


if __name__=='__main__':main()

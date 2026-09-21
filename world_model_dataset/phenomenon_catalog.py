"""Combine readable phenomenon batches with explicit, non-destructive supersession."""
import argparse
from pathlib import Path

from .causal_loader import open_episode
from .io import file_hash, read_json, write_json


def build(indices, supersede, output, verify_streams=False):
    episodes={};sources=[]
    for path in indices:
        path=Path(path).resolve();index=read_json(path)
        sources.append(dict(path=str(path),sha256=file_hash(path)))
        for item in index['episodes']:
            if item.get('status')=='failed_collection':continue
            if item['id'] in episodes and episodes[item['id']]!=item:
                raise ValueError('Conflicting episode ID: '+item['id'])
            episodes[item['id']]=item
    history=[]
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
        ep=open_episode(path)
        if ep.manifest['trajectory']['states']['sha256']!=item['source_states_sha256']:
            raise ValueError('Changed physical states: '+item['id'])
        actual=dict(states=sum(1 for _ in ep.states()),observations=sum(1 for _ in ep.observations())) if verify_streams else item
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

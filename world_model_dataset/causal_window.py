"""Read aligned native records in a closed time window, without interpolation.

Commands and efforts remain separate from actual states. A terminal state may
have no next-step command; none-control episodes have empty control streams.
"""
import argparse
import math
from pathlib import Path

import numpy as np

from .causal_loader import open_episode
from .io import read_json, write_json


def read_window(episode, start_s, end_s):
    if not math.isfinite(start_s) or not math.isfinite(end_s) or start_s>end_s:
        raise ValueError('Expected finite ordered window bounds')
    rows=list(episode.states())
    if not rows or start_s<rows[0]['time_s'] or end_s>rows[-1]['time_s']:
        raise ValueError('Window lies outside recorded state duration')
    selected=[r for r in rows if start_s<=r['time_s']<=end_s]
    if not selected:raise ValueError('Window contains no recorded state samples')
    times={r['physics_step']:r['time_s'] for r in selected}
    def aligned(row):
        if row['physics_step'] not in times or abs(row['time_s']-times[row['physics_step']])>1e-9:
            raise ValueError('Record does not align with a state sample')
    def trace(iterator):
        values=[r for r in iterator if start_s<=r['time_s']<=end_s]
        for row in values:aligned(row)
        return values
    geometry={};geometry_assets={}
    for row in selected:
        for oid,state in row['body_states'].items():
            if 'geometry' not in state:continue
            # Original static scene meshes (and rigid local meshes) have no
            # frame timestamps. Their motion, if any, belongs to body states.
            if 'topology' not in state:
                if oid in geometry_assets:
                    if geometry_assets[oid]['record']!=state['geometry']:
                        raise ValueError('Geometry asset changed within window: '+oid)
                else:
                    with np.load(episode.record_path(state['geometry']),allow_pickle=False) as data:
                        geometry_assets[oid]=dict(record=state['geometry'],arrays={k:data[k].copy() for k in data.files})
                continue
            with np.load(episode.record_path(state['geometry']),allow_pickle=False) as data:
                if float(data['time_s'])!=row['time_s'] or int(data['physics_step'])!=row['physics_step']:
                    raise ValueError('Geometry time metadata mismatch')
                for key in data.files:
                    if np.issubdtype(data[key].dtype,np.number) and not np.isfinite(data[key]).all():
                        raise ValueError('Nonfinite native geometry field: '+key)
                geometry.setdefault(oid,[]).append((row,{key:data[key].copy() for key in data.files}))
    controls=trace(episode.controls());actual={};efforts={}
    for controller in episode.manifest['control_program']['controllers']:
        oid=controller['controller_id']
        actual[oid]=trace(episode.actuator_states(oid))
        efforts[oid]=trace(episode.actuator_efforts(oid))
    observations=[];index=None
    record=episode.manifest['trajectory']['observations']
    if record['status']=='available':
        index=read_json(episode.record_path(record))
        for row,arrays in episode.observations():
            if start_s<=row['time_s']<=end_s:
                aligned(row);observations.append((row,arrays))
    return dict(requested_interval_s=[start_s,end_s],sample_interval_s=[selected[0]['time_s'],selected[-1]['time_s']],
        states=selected,geometries=geometry,geometry_assets=geometry_assets,controls=controls,actuator_states=actual,actuator_efforts=efforts,
        observations=observations,observation_index=index,
        control_primitive=episode.manifest['control_program']['primitive'],
        observation_availability=record,
        semantics='Closed interval; native samples only, no resampling, interpolation or command carry-forward; effort is not contact reaction')


def describe(window):
    return dict(requested_interval_s=window['requested_interval_s'],sample_interval_s=window['sample_interval_s'],
        states=len(window['states']),observations=len(window['observations']),commands=len(window['controls']),
        actual_actuator_records={k:len(v) for k,v in window['actuator_states'].items()},
        effort_records={k:len(v) for k,v in window['actuator_efforts'].items()},
        geometry_assets={k:list(v['arrays']) for k,v in window['geometry_assets'].items()},
        native_geometry={k:dict(frames=len(v),fields=list(v[0][1]),
            shapes={f:list(a.shape) for f,a in v[0][1].items()}) for k,v in window['geometries'].items()},
        control_primitive=window['control_primitive'],semantics=window['semantics'])


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--catalog',type=Path,required=True);p.add_argument('--ids',nargs='+',required=True)
    p.add_argument('--start',type=float,required=True);p.add_argument('--end',type=float,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    entries={e['id']:e for e in read_json(a.catalog)['episodes']};report={}
    for oid in a.ids:
        ep=open_episode(entries[oid]['episode']);window=read_window(ep,a.start,a.end)
        report[oid]=describe(window);print('WINDOW_CHECKED',oid,report[oid]['states'],flush=True)
    write_json(a.output,dict(episodes=report,training=False,splits=False))


if __name__=='__main__':main()

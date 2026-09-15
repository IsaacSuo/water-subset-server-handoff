"""A single synchronized cache frame, without rendering or rerunning PhysX."""
from pathlib import Path

import numpy as np

from .loader import Episode


def preview(root,output,time_s):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    output=Path(output)
    if output.exists():raise FileExistsError(output)
    episode=Episode(root,require_complete=False)
    states=list(episode.states())
    index=min(range(len(states)),key=lambda i:abs(states[i]['time_s']-time_s));state=states[index]
    meshes={oid:episode.geometry(index,oid) for oid in state['objects']}
    points=np.concatenate([mesh['surface_world_m'] for mesh in meshes.values()])
    centre=(points.min(0)+points.max(0))/2;extent=max(float(np.ptp(points,axis=0).max()),.2)*.65
    figure=plt.figure(figsize=(12,5))
    for number,(elevation,azimuth) in enumerate(((15,-90),(30,-55)),1):
        axis=figure.add_subplot(1,2,number,projection='3d')
        for oid,mesh in meshes.items():
            color='#408ccd' if state['objects'][oid]['physics_kind']=='volumetric' else '#ef9b45'
            triangles=mesh['surface_world_m'][mesh['surface_triangles']]
            axis.add_collection3d(Poly3DCollection(triangles,facecolor=color,edgecolor='none',alpha=.85))
            axis.scatter(*state['objects'][oid]['position_m'],color='black',s=12)
        axis.set(xlim=(centre[0]-extent,centre[0]+extent),ylim=(centre[1]-extent,centre[1]+extent),
                 zlim=(min(0.,centre[2]-extent),centre[2]+extent),xlabel='X (m)',ylabel='Y (m)',zlabel='Z (m)')
        axis.set_box_aspect((1,1,1));axis.view_init(elevation,azimuth)
    figure.suptitle(f"{episode.manifest['episode_id']} | shared t={state['time_s']:.6f}s | physics step={state['physics_step']}")
    figure.tight_layout();output.parent.mkdir(parents=True,exist_ok=True);figure.savefig(output,dpi=150);plt.close(figure)
    return {'capture_index':index,'time_s':state['time_s'],'physics_step':state['physics_step'],'objects':list(meshes)}


def main():
    import argparse
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('episode',type=Path)
    parser.add_argument('output',type=Path);parser.add_argument('--time',type=float,default=.65)
    args=parser.parse_args();print(preview(args.episode,args.output,args.time))


if __name__=='__main__':main()

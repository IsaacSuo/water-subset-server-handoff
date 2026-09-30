"""Regenerate the pitcher fill of existing pour assets at rest density.

The original fill is a cubic lattice at one particle spacing, held two
spacings from the shell.  With SPH_Project's particle volume of 0.8 d^3 that
lattice has 80 % of rest density and no support, so the water free-falls and
compacts when the run starts.  This script keeps the exact vessel meshes and
the water mass, and replaces only the particle positions:

- cubic lattice at ``lattice_factor * spacing`` (0.931 gives 0.995 rest density);
- shell clearance of ``clearance_spacings * spacing``, close to where a
  Volume Map wall balances the first fluid layer;
- bottom layer placed at exactly that clearance above the cavity floor.

No Blender is needed.  The vessel geometry is copied unchanged.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import trimesh
from scipy import ndimage
from scipy.spatial import cKDTree


def shell_distance_tree(vertices,faces,spacing):
    # Distances are taken to a dense vertex set.  With edges below 0.35 d the
    # error at the clearance distance is second order (under 0.01 d).
    dense,_=trimesh.remesh.subdivide_to_size(vertices,faces,max_edge=.35*spacing,max_iter=20)
    return cKDTree(dense),len(dense)


def build_fill(vertices,faces,axis,spacing,count,lattice_factor,clearance_spacings,maximum_height):
    pitch=lattice_factor*spacing;clearance=clearance_spacings*spacing
    tree,surface_points=shell_distance_tree(vertices,faces,spacing)
    bottom=float(vertices[:,1].min())
    # Walk down the cavity axis to the level exactly one clearance above the floor.
    probe_y=np.arange(bottom+.5*maximum_height,bottom,-.02*spacing)
    probe=np.column_stack([np.full_like(probe_y,axis[0]),probe_y,np.full_like(probe_y,axis[2])])
    near=np.flatnonzero(tree.query(probe,workers=-1,distance_upper_bound=2.*clearance)[0]<clearance)
    if not len(near) or near[0]==0:raise ValueError('Cavity axis does not reach a floor below the fill')
    first_layer=float(probe_y[near[0]-1])
    reach=float(np.abs(vertices[:,[0,2]]-np.asarray(axis)[[0,2]]).max())+2.*pitch
    half=int(np.ceil(reach/pitch))
    xs=axis[0]+np.arange(-half,half+1)*pitch;zs=axis[2]+np.arange(-half,half+1)*pitch
    below=int(np.floor((first_layer-bottom)/pitch));above=int(np.floor((bottom+maximum_height-first_layer)/pitch))
    ys=first_layer+np.arange(-below,above+1)*pitch
    grid=np.stack(np.meshgrid(xs,ys,zs,indexing='ij'),-1)
    # Only "closer than the clearance or not" matters; the bound keeps the many
    # lattice points far from the shell from searching the whole surface.
    distance=tree.query(grid.reshape(-1,3),workers=-1,
        distance_upper_bound=2.*clearance)[0].reshape(grid.shape[:3])
    # Points clear of the shell form separate regions: the cavity, the space
    # around the vessel and any voids in the shell.  Keep the cavity only.
    labels,_=ndimage.label(distance>=clearance)
    seed=(half,below+int(round(.5*maximum_height/pitch)),half)
    if labels[seed]==0:raise ValueError('Cavity seed point lies within the shell clearance')
    cavity=labels==labels[seed]
    if cavity[0].any() or cavity[-1].any() or cavity[:,:,0].any() or cavity[:,:,-1].any():
        raise ValueError('Fill region leaks out of the vessel below the height limit; lower --maximum-height')
    candidates=grid[cavity];distances=distance[cavity]
    if len(candidates)<count:
        raise ValueError(f'Only {len(candidates)} lattice points fit below the height limit; {count} requested')
    # Whole lower layers first; a partial top layer is a centred patch.
    radial=(candidates[:,0]-axis[0])**2+(candidates[:,2]-axis[2])**2
    chosen=np.sort(np.lexsort((radial,candidates[:,1]))[:count])
    return candidates[chosen].astype(np.float32),dict(lattice_pitch_m=pitch,lattice_factor=lattice_factor,
        shell_clearance_m=clearance,shell_clearance_spacings=clearance_spacings,
        first_layer_above_vessel_bottom_m=first_layer-bottom,available_lattice_points=int(len(candidates)),
        minimum_distance_to_shell_m=float(distances[chosen].min()),
        fill_top_above_vessel_bottom_m=float(candidates[chosen][:,1].max()-bottom),
        distance_surface_points=surface_points)


def main():
    parser=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--assets',type=Path,required=True,help='Existing pour assets: assets.json and geometry_and_fill.npz')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--spacing',type=float,help='Solver particle spacing; defaults to that of the source assets')
    parser.add_argument('--particle-count',type=int,help='Defaults to the count that keeps the source water mass')
    parser.add_argument('--lattice-factor',type=float,default=.931)
    parser.add_argument('--clearance-spacings',type=float,default=1.2)
    parser.add_argument('--maximum-height',type=float,default=.26,
        help='Highest fill level above the vessel bottom that may be used, in metres')
    args=parser.parse_args()
    meta=json.loads((args.assets/'assets.json').read_text(encoding='utf-8'))
    source=args.assets/'geometry_and_fill.npz'
    source_digest=hashlib.sha256(source.read_bytes()).hexdigest()
    if source_digest!=meta['geometry_sha256']:raise ValueError('Source asset geometry hash mismatch')
    if meta['case']['id']!='01_container_transfer':raise ValueError('Only the pouring pitcher fill is supported')
    with np.load(source) as data:arrays={key:data[key].copy() for key in data.files}
    old_spacing=float(meta['particle_spacing_m']);spacing=old_spacing if args.spacing is None else args.spacing
    count=(int(round(len(arrays['positions'])*(old_spacing/spacing)**3)) if args.particle_count is None
        else args.particle_count)
    if not .9<=args.lattice_factor<=1. or not 1.<=args.clearance_spacings<=2. or count<=10000:
        raise ValueError('Lattice factor, clearance or particle count outside the supported range')
    axis=np.asarray(meta['fill_cavity_axis_local_m'],np.float64)
    local,fill=build_fill(arrays['donor_vertices'].astype(np.float64),arrays['donor_triangles'],axis,
        spacing,count,args.lattice_factor,args.clearance_spacings,args.maximum_height)
    args.output.mkdir(parents=True,exist_ok=False)
    arrays['positions']=(local+np.asarray(meta['donor']['position_m'],np.float32)).astype(np.float32)
    arrays['velocities']=np.zeros_like(arrays['positions'])
    np.savez(args.output/'geometry_and_fill.npz',**arrays)
    fill.update(method='rest_density_cubic_lattice',source_assets=str(args.assets),source_geometry_sha256=source_digest,
        source_particle_spacing_m=old_spacing,source_particle_count=int(meta['particle_count']),
        water_volume_liters=count*.8*spacing**3*1000.)
    meta.update(particle_spacing_m=spacing,particle_count=count,particle_limit=count,
        nominal_lattice_liters=count*fill['lattice_pitch_m']**3*1000.,
        unlimited_lattice_count=fill['available_lattice_points'],
        initial_fill_top_above_pitcher_bottom_m=fill['fill_top_above_vessel_bottom_m'],
        minimum_initial_distance_to_shell_m=fill['minimum_distance_to_shell_m'],fill=fill,
        geometry_sha256=hashlib.sha256((args.output/'geometry_and_fill.npz').read_bytes()).hexdigest())
    (args.output/'assets.json').write_text(json.dumps(meta,indent=2),encoding='utf-8')
    print(json.dumps(dict(particle_spacing_m=spacing,particle_count=count,fill=fill),indent=2))


if __name__=='__main__':main()

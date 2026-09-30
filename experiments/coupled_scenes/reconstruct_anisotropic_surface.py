"""Anisotropic-kernel water surface plus a separate spray set for one snapshot.

Thin sheets one or two particles thick fall apart under the isotropic kernel
of splashsurf: the gaps between particles are wider than the kernel reaches.
Following Yu & Turk (2013, "Reconstructing surfaces of particle-based fluids
using anisotropic kernels"), each particle's kernel is stretched along the
principal axes of its weighted neighbourhood, so a sheet's particles overlap
in-plane while the sheet stays thin.  Each kernel is weighted by m/rho of its
particle (as in Yu & Turk), so the field is about one inside the fluid and a
thin or sparse sheet is not lost.  Particles with too few close neighbours for
a covariance are fitted over a wider radius; a coplanar sparse sheet then still
reconstructs as a film.

Particles with almost no neighbours even over that wider radius are spray:
they are left out of the surface.  Each spray particle's volume is split into a
fixed cloud of smaller droplets laid out by its stable ID, so the cloud moves
without flicker.

GPU (Warp).  The simulation is not changed; this only builds render geometry.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import warp as wp
from scipy import sparse


@wp.func
def cubic_kernel(q: float):
    # Normalised 3-D cubic spline with unit support (SPlisHSPlasH convention).
    value = float(0.0)
    if q <= 0.5:
        value = 6.0*q*q*q-6.0*q*q+1.0
    elif q < 1.0:
        a = 1.0-q
        value = 2.0*a*a*a
    return value*8.0/3.14159265358979


@wp.kernel
def neighbourhood_statistics(grid: wp.uint64, points: wp.array(dtype=wp.vec3), support: float, sheet_radius: float,
                             volume: float, dense_count: wp.array(dtype=int), sheet_count: wp.array(dtype=int),
                             density: wp.array(dtype=float)):
    i = wp.tid()
    x = points[i]
    dense = int(0)
    sheet = int(0)
    total = float(0.0)
    j = int(0)
    query = wp.hash_grid_query(grid, x, sheet_radius)
    while wp.hash_grid_query_next(query, j):
        d = wp.length(points[j]-x)
        if d < support:
            dense += 1
            total += volume*cubic_kernel(d/support)/(support*support*support)
        if j != i and d < sheet_radius:
            sheet += 1
    dense_count[i] = dense
    sheet_count[i] = sheet
    density[i] = total


@wp.kernel
def anisotropic_kernels(grid: wp.uint64, points: wp.array(dtype=wp.vec3), dense_count: wp.array(dtype=int),
                        sheet_count: wp.array(dtype=int), density: wp.array(dtype=float),
                        support: float, sheet_radius: float, smoothing: float,
                        minimum_neighbours: int, minimum_sheet_neighbours: int,
                        dense_ratio: float, sheet_ratio: float, volume: float, density_floor: float,
                        centre: wp.array(dtype=wp.vec3), transform: wp.array(dtype=wp.mat33),
                        reach: wp.array(dtype=float), amplitude: wp.array(dtype=float), mode: wp.array(dtype=int)):
    i = wp.tid()
    x = points[i]
    # Shepard-style weight m/rho as in Yu & Turk: sparse particles are not lost.
    weight = volume/wp.max(density[i], density_floor)
    radius = support
    ratio = dense_ratio
    lam = smoothing
    kind = int(1)
    if dense_count[i] < minimum_neighbours:
        if sheet_count[i] >= minimum_sheet_neighbours:
            # Too few close neighbours: fit the wider neighbourhood, e.g. a sparse sheet.
            radius = sheet_radius
            ratio = sheet_ratio
            lam = 0.0
            kind = 2
        else:
            kind = 0
    centre[i] = x
    transform[i] = wp.mat33(1.0/support, 0.0, 0.0, 0.0, 1.0/support, 0.0, 0.0, 0.0, 1.0/support)
    reach[i] = support
    amplitude[i] = weight/(support*support*support)
    mode[i] = kind
    if kind == 0:
        return
    total = float(0.0)
    weighted = wp.vec3(0.0)
    j = int(0)
    query = wp.hash_grid_query(grid, x, radius)
    while wp.hash_grid_query_next(query, j):
        d = wp.length(points[j]-x)
        if d < radius:
            w = 1.0-(d/radius)*(d/radius)*(d/radius)
            total += w
            weighted += w*points[j]
    m = weighted/total
    covariance = wp.mat33(0.0)
    query = wp.hash_grid_query(grid, x, radius)
    while wp.hash_grid_query_next(query, j):
        d = wp.length(points[j]-x)
        if d < radius:
            w = 1.0-(d/radius)*(d/radius)*(d/radius)
            r = points[j]-m
            covariance += w*wp.outer(r, r)
    covariance = covariance/total
    centre[i] = (1.0-lam)*x+lam*m
    u = wp.mat33()
    sigma = wp.vec3()
    v = wp.mat33()
    wp.svd3(covariance, u, sigma, v)
    # Standard deviations along the principal axes, with a bounded aspect ratio.
    a = wp.sqrt(wp.max(sigma[0], 0.0))
    b = wp.sqrt(wp.max(sigma[1], 0.0))
    c = wp.sqrt(wp.max(sigma[2], 0.0))
    largest = wp.max(a, wp.max(b, c))
    if largest <= 0.0:
        return
    floor = largest/ratio
    a = wp.max(a, floor)
    b = wp.max(b, floor)
    c = wp.max(c, floor)
    scale = wp.pow(a*b*c, 1.0/3.0)
    a = a/scale
    b = b/scale
    c = c/scale
    inverse = wp.mat33(1.0/(radius*a), 0.0, 0.0, 0.0, 1.0/(radius*b), 0.0, 0.0, 0.0, 1.0/(radius*c))
    transform[i] = u*inverse*wp.transpose(u)
    reach[i] = radius*wp.max(a, wp.max(b, c))
    amplitude[i] = weight/(radius*radius*radius)


@wp.kernel
def splat(selected: wp.array(dtype=int), centre: wp.array(dtype=wp.vec3), transform: wp.array(dtype=wp.mat33),
          reach: wp.array(dtype=float), amplitude: wp.array(dtype=float), origin: wp.vec3, cell: float,
          field: wp.array3d(dtype=float)):
    t = wp.tid()
    i = selected[t]
    c = centre[i]
    g = transform[i]
    r = reach[i]
    scale = amplitude[i]
    nx = field.shape[0]
    ny = field.shape[1]
    nz = field.shape[2]
    x0 = wp.max(int(wp.floor((c[0]-r-origin[0])/cell)), 0)
    y0 = wp.max(int(wp.floor((c[1]-r-origin[1])/cell)), 0)
    z0 = wp.max(int(wp.floor((c[2]-r-origin[2])/cell)), 0)
    x1 = wp.min(int(wp.ceil((c[0]+r-origin[0])/cell)), nx-1)
    y1 = wp.min(int(wp.ceil((c[1]+r-origin[1])/cell)), ny-1)
    z1 = wp.min(int(wp.ceil((c[2]+r-origin[2])/cell)), nz-1)
    for a in range(x0, x1+1):
        for b in range(y0, y1+1):
            for k in range(z0, z1+1):
                p = origin+wp.vec3(float(a), float(b), float(k))*cell
                q = wp.length(g*(p-c))
                if q < 1.0:
                    wp.atomic_add(field, a, b, k, scale*cubic_kernel(q))


def extract_surface(centres, transforms, reaches, amplitudes, selected, cell, threshold, tile, device):
    """Tiled field evaluation and marching cubes; tiles share their boundary node planes."""
    if not len(selected):
        return np.empty((0, 3)), np.empty((0, 3), np.int64)
    margin=float(reaches[selected].max())+2.*cell
    lower=centres[selected].min(axis=0)-margin
    upper=centres[selected].max(axis=0)+margin
    cells=np.ceil((upper-lower)/cell).astype(int)
    lower=lower.astype(np.float64)
    centre_array=wp.array(centres,dtype=wp.vec3,device=device)
    transform_array=wp.array(transforms,dtype=wp.mat33,device=device)
    reach_array=wp.array(reaches,dtype=float,device=device)
    amplitude_array=wp.array(amplitudes,dtype=float,device=device)
    vertices=[];faces=[];offset=0
    for tx in range(0,cells[0],tile):
        for ty in range(0,cells[1],tile):
            for tz in range(0,cells[2],tile):
                start=np.array([tx,ty,tz]);count=np.minimum(tile,cells-start)+1
                origin=lower+start*cell;end=origin+(count-1)*cell
                inside=selected[np.all((centres[selected]+reaches[selected,None]>=origin)
                    &(centres[selected]-reaches[selected,None]<=end),axis=1)]
                if not len(inside):continue
                field=wp.zeros(tuple(int(n) for n in count),dtype=float,device=device)
                wp.launch(splat,dim=len(inside),inputs=[wp.array(inside.astype(np.int32),dtype=int,device=device),
                    centre_array,transform_array,reach_array,amplitude_array,wp.vec3(*origin),float(cell),field],device=device)
                if float(field.numpy().max())<threshold:continue
                verts,indices=wp.MarchingCubes.extract_surface_marching_cubes(field,threshold,
                    wp.vec3(*origin),wp.vec3(*end))
                verts=verts.numpy().astype(np.float64);indices=indices.numpy().reshape(-1,3).astype(np.int64)
                if not len(indices):continue
                vertices.append(verts);faces.append(indices+offset);offset+=len(verts)
    if not faces:
        return np.empty((0, 3)), np.empty((0, 3), np.int64)
    vertices=np.concatenate(vertices);faces=np.concatenate(faces)
    # Weld vertices duplicated on tile boundaries and drop collapsed triangles.
    keys=np.round(vertices/(cell*1e-4)).astype(np.int64)
    _,first,inverse=np.unique(keys,axis=0,return_index=True,return_inverse=True)
    vertices=vertices[first];faces=inverse.reshape(-1)[faces]
    faces=faces[(faces[:,0]!=faces[:,1])&(faces[:,1]!=faces[:,2])&(faces[:,0]!=faces[:,2])]
    return vertices,faces


def taubin_smooth(vertices, faces, iterations, shrink=.5, inflate=-.53):
    """Volume-preserving Laplacian smoothing that removes marching-cubes terracing."""
    if iterations<=0 or not len(faces):
        return vertices
    edges=np.concatenate([faces[:,[0,1]],faces[:,[1,2]],faces[:,[2,0]]])
    edges=np.concatenate([edges,edges[:,::-1]])
    adjacency=sparse.coo_matrix((np.ones(len(edges)),(edges[:,0],edges[:,1])),
        shape=(len(vertices),len(vertices))).tocsr()
    adjacency.data[:]=1.
    degree=np.asarray(adjacency.sum(axis=1)).ravel();degree[degree==0]=1.
    for _ in range(iterations):
        for factor in (shrink,inflate):
            vertices=vertices+factor*(adjacency@vertices/degree[:,None]-vertices)
    return vertices


def spray_droplets(positions, ids, spacing, droplets, spread):
    """Split each spray particle's volume into droplets laid out by its stable ID."""
    volume=.8*spacing**3
    radius=(3.*volume/(4.*math.pi*droplets))**(1/3)
    offsets=np.empty((len(ids),droplets,3))
    for row,identifier in enumerate(ids):
        generator=np.random.default_rng(int(identifier))
        direction=generator.normal(size=(droplets,3))
        direction/=np.linalg.norm(direction,axis=1,keepdims=True)
        offsets[row]=direction*(generator.random(droplets)**(1/3))[:,None]
    return (positions[:,None,:]+offsets*spread*spacing).reshape(-1,3),radius


def write_obj(path, vertices, faces):
    with path.open('w',encoding='utf-8') as stream:
        np.savetxt(stream,vertices,fmt='v %.6f %.6f %.6f')
        np.savetxt(stream,faces+1,fmt='f %d %d %d')


def sha256_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('snapshot',type=Path)
    parser.add_argument('output',type=Path)
    parser.add_argument('--spacing',type=float,required=True,help='Simulation particle spacing in metres')
    parser.add_argument('--support',type=float,default=2.5,help='Isotropic kernel support in spacings')
    parser.add_argument('--sheet-radius',type=float,default=4.,
        help='Wider neighbourhood, in spacings, used for sparse sheets and for the spray test')
    parser.add_argument('--cell',type=float,default=.5,help='Marching cubes cell in spacings')
    parser.add_argument('--threshold',type=float,default=.5,help='Iso level; the field is about one inside the fluid')
    parser.add_argument('--smoothing',type=float,default=.9,help='Kernel centre smoothing for dense particles (Yu & Turk lambda)')
    parser.add_argument('--maximum-anisotropy',type=float,default=4.,help='Largest axis ratio for dense neighbourhoods')
    parser.add_argument('--sheet-anisotropy',type=float,default=8.,help='Largest axis ratio for sparse sheets')
    parser.add_argument('--minimum-neighbours',type=int,default=10,
        help='Neighbours within the support needed to fit a dense covariance')
    parser.add_argument('--minimum-sheet-neighbours',type=int,default=3,
        help='Neighbours within the sheet radius needed to fit a sparse covariance')
    parser.add_argument('--spray-neighbours',type=int,default=3,
        help='Particles with fewer neighbours within the sheet radius are rendered as spray')
    parser.add_argument('--density-floor',type=float,default=.1)
    parser.add_argument('--droplets-per-particle',type=int,default=1,
        help='Split each spray particle into this many equal droplets (one keeps its full volume)')
    parser.add_argument('--droplet-spread',type=float,default=0.,help='Droplet cloud radius in spacings when splitting')
    parser.add_argument('--taubin-iterations',type=int,default=10)
    parser.add_argument('--isotropic',action='store_true',help='Disable anisotropy (comparison only)')
    parser.add_argument('--tile',type=int,default=256)
    args=parser.parse_args()
    if not math.isfinite(args.spacing) or args.spacing<=0:raise ValueError('Spacing must be positive')
    if args.sheet_radius<args.support:raise ValueError('The sheet radius must not be smaller than the support')
    args.output.mkdir(parents=True,exist_ok=False)
    with np.load(args.snapshot,allow_pickle=False) as data:
        positions=data['positions'].astype(np.float64)
        ids=data['ids'] if 'ids' in data.files else np.arange(len(positions))
        velocities=data['velocities'] if 'velocities' in data.files else np.zeros_like(positions)
        seconds=float(data['simulated_seconds'])
    wp.init();device='cuda:0' if wp.is_cuda_available() else 'cpu'
    spacing=args.spacing;support=args.support*spacing;sheet_radius=args.sheet_radius*spacing
    cell=args.cell*spacing;volume=.8*spacing**3
    points=wp.array(positions,dtype=wp.vec3,device=device)
    grid=wp.HashGrid(128,128,128,device=device);grid.build(points,sheet_radius)
    n=len(positions)
    dense_count=wp.zeros(n,dtype=int,device=device);sheet_count=wp.zeros(n,dtype=int,device=device)
    density=wp.zeros(n,dtype=float,device=device)
    wp.launch(neighbourhood_statistics,dim=n,inputs=[grid.id,points,support,sheet_radius,volume,
        dense_count,sheet_count,density],device=device)
    centre=wp.zeros(n,dtype=wp.vec3,device=device);transform=wp.zeros(n,dtype=wp.mat33,device=device)
    reach=wp.zeros(n,dtype=float,device=device);amplitude=wp.zeros(n,dtype=float,device=device)
    mode=wp.zeros(n,dtype=int,device=device)
    disabled=10**9 if args.isotropic else None
    wp.launch(anisotropic_kernels,dim=n,inputs=[grid.id,points,dense_count,sheet_count,density,support,sheet_radius,
        args.smoothing,disabled or args.minimum_neighbours,disabled or args.minimum_sheet_neighbours,
        args.maximum_anisotropy,args.sheet_anisotropy,volume,args.density_floor,
        centre,transform,reach,amplitude,mode],device=device)
    spray=sheet_count.numpy()<args.spray_neighbours;selected=np.flatnonzero(~spray);mode=mode.numpy()
    vertices,faces=extract_surface(centre.numpy().astype(np.float64),transform.numpy(),reach.numpy().astype(np.float64),
        amplitude.numpy().astype(np.float64),selected,cell,args.threshold,args.tile,device)
    vertices=taubin_smooth(vertices,faces,args.taubin_iterations)
    mesh=args.output/'water.obj';write_obj(mesh,vertices,faces)
    droplets,droplet_radius=spray_droplets(positions[spray],ids[spray],spacing,args.droplets_per_particle,args.droplet_spread)
    np.savez(args.output/'spray.npz',positions=positions[spray].astype(np.float32),
        velocities=velocities[spray].astype(np.float32),ids=ids[spray],
        droplet_positions=droplets.astype(np.float32),droplet_radius=np.float32(droplet_radius))
    report=dict(source=str(args.snapshot.resolve()),source_sha256=sha256_file(args.snapshot),simulated_seconds=seconds,
        method='anisotropic_kernels_yu_turk_2013' if not args.isotropic else 'isotropic_kernels_comparison',
        particle_count=int(n),surface_particles=int(len(selected)),spray_particles=int(spray.sum()),
        dense_anisotropic_particles=int(np.count_nonzero(mode[selected]==1)),
        sparse_sheet_particles=int(np.count_nonzero(mode[selected]==2)),
        isotropic_particles=int(np.count_nonzero(mode[selected]==0)),
        parameters=dict(spacing_m=spacing,support_spacings=args.support,sheet_radius_spacings=args.sheet_radius,
            cell_spacings=args.cell,threshold=args.threshold,centre_smoothing=args.smoothing,
            maximum_anisotropy=args.maximum_anisotropy,sheet_anisotropy=args.sheet_anisotropy,
            minimum_neighbours=args.minimum_neighbours,minimum_sheet_neighbours=args.minimum_sheet_neighbours,
            spray_neighbours_within_sheet_radius=args.spray_neighbours,density_floor=args.density_floor,
            droplets_per_particle=args.droplets_per_particle,droplet_spread_spacings=args.droplet_spread,
            taubin_iterations=args.taubin_iterations,isotropic=args.isotropic),
        droplet_radius_m=float(droplet_radius),vertices=int(len(vertices)),triangles=int(len(faces)),
        mesh_sha256=sha256_file(mesh),physics_gate_passed=False,complete=True,
        purpose='render geometry only; spray droplets are a sub-particle representation of unresolved spray')
    (args.output/'snapshot_surface.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({key:report[key] for key in ('method','particle_count','surface_particles','spray_particles',
        'dense_anisotropic_particles','sparse_sheet_particles','isotropic_particles','vertices','triangles',
        'droplet_radius_m')},indent=2),flush=True)


if __name__=='__main__':main()

"""Isolated water impact on a flat Volume Map plate at one particle spacing.

A ball or block of water falls at a prescribed speed onto a static plate.
The solver, boundary model and fluid settings are those of the pouring runs;
only the spacing changes between invocations, so two runs compare spatial
resolution alone.  Diagnostic only: no physical pass/fail threshold.
"""
import argparse
import contextlib
import json
import sys
import time
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial import cKDTree

ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))

MARGIN=.2                       # free space around the plate inside the search domain
PLATE=np.array([.4,.016,.4])    # plate extents; thicker than the support radius at 4 mm
HEADROOM=.4


def build_fluid(shape,size,spacing,center,lattice_factor):
    # SPH_Project uses V0 = 0.8 d^3, so a cubic lattice at 0.8^(1/3) d = 0.928 d sits at
    # rest density (kernel sum 1.005).  The default is slightly wider so the fluid starts
    # just below rest density and the density projection has no initial error to remove.
    pitch=spacing*lattice_factor;count=int(np.ceil(.5*size/pitch))
    axis=(np.arange(-count,count)+.5)*pitch
    points=np.stack(np.meshgrid(axis,axis,axis,indexing='ij'),-1).reshape(-1,3)
    if shape=='sphere':keep=np.linalg.norm(points,axis=1)<=.5*size
    else:keep=np.abs(points).max(axis=1)<=.5*size
    return (points[keep]+center).astype(np.float32),pitch


def frame_metrics(positions,velocities,spacing,speed,footprint):
    magnitude=np.linalg.norm(velocities,axis=1);horizontal=np.linalg.norm(velocities[:,[0,2]],axis=1)
    counts=cKDTree(positions).query_ball_point(positions,np.nextafter(2.*spacing,0.),
        workers=-1,return_length=True)-1
    fast=magnitude>1.5*speed;isolated=counts<6
    radius=np.linalg.norm(positions[:,[0,2]],axis=1)
    return dict(max_speed_over_impact=float(magnitude.max()/speed),
        p999_speed_over_impact=float(np.percentile(magnitude,99.9)/speed),
        max_horizontal_speed_over_impact=float(horizontal.max()/speed),
        mass_fraction_above_1p5_impact=float(fast.mean()),
        mass_fraction_above_2_impact=float((magnitude>2.*speed).mean()),
        kinetic_energy_over_initial=float(np.sum(magnitude**2)/(len(magnitude)*speed**2)),
        mass_fraction_isolated=float(isolated.mean()),
        mass_fraction_below_20_neighbors=float((counts<20).mean()),
        fast_particles=int(fast.sum()),fast_fraction_isolated=float(isolated[fast].mean()) if fast.any() else None,
        fast_median_neighbors=float(np.median(counts[fast])) if fast.any() else None,
        mass_fraction_outside_footprint=float((radius>footprint).mean()),
        outside_fraction_isolated=(float(isolated[radius>footprint].mean()) if (radius>footprint).any() else None),
        outside_median_neighbors=(float(np.median(counts[radius>footprint])) if (radius>footprint).any() else None),
        max_radius_m=float(radius.max()),max_height_m=float(positions[:,1].max()))


def main():
    parser=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--spacing',type=float,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--upstream',type=Path,default=Path('/mnt/y/tools/SPH_Project'))
    parser.add_argument('--shape',choices=('sphere','cube'),default='sphere')
    parser.add_argument('--size',type=float,default=.08,help='Ball diameter or block edge in metres')
    parser.add_argument('--speed',type=float,default=3.,help='Initial downward speed in m/s')
    parser.add_argument('--gap',type=float,default=.012,help='Initial clearance between fluid and plate')
    parser.add_argument('--seconds',type=float,default=.03)
    parser.add_argument('--hz',type=int,default=20000)
    parser.add_argument('--snapshot-ms',type=float,default=1.)
    parser.add_argument('--akinci-coefficient',type=float,default=1.)
    parser.add_argument('--minimum-dt-divisor',type=int,default=4)
    parser.add_argument('--lattice-factor',type=float,default=.931,
        help='Initial cubic lattice pitch as a fraction of the spacing')
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False);capture=args.output/'capture';capture.mkdir()
    spacing=args.spacing;dt=1./args.hz
    plate_center=np.array([MARGIN+.5*PLATE[0],MARGIN+.5*PLATE[1],MARGIN+.5*PLATE[2]])
    plate_top=MARGIN+PLATE[1]
    # Frames are stored with the plate top at y = 0 and the impact axis at x = z = 0.
    reference=np.array([plate_center[0],plate_top,plate_center[2]])
    positions,pitch=build_fluid(args.shape,args.size,spacing,
        reference+np.array([0.,args.gap+.5*args.size,0.]),args.lattice_factor)
    velocities=np.zeros_like(positions);velocities[:,1]=-args.speed
    plate=trimesh.creation.box(extents=PLATE)
    samples=np.asarray(plate.voxelized(pitch=spacing).fill().points,np.float32)
    spec=dict(name='receiver',objectId=1,translation=plate_center.tolist(),quaternion_xyzw=[0,0,0,1],
        isDynamic=False,prescribed=False,density=1000)
    domain_end=[2*MARGIN+PLATE[0],plate_top+HEADROOM+MARGIN,2*MARGIN+PLATE[2]]
    # The Newton adapter needs one moving body.  This block is held still in a
    # far upper corner, many support radii from any fluid the run can reach.
    idle=trimesh.creation.box(extents=[.024]*3)
    idle_samples=np.asarray(idle.voxelized(pitch=spacing).fill().points,np.float32)
    idle_position=np.array([.06,domain_end[1]-.06,.06])
    idle_spec=dict(name='donor',objectId=2,translation=idle_position.tolist(),quaternion_xyzw=[0,0,0,1],
        isDynamic=True,prescribed=True,density=1000)
    common=dict(geometryFile='imported-arrays',rotationAxis=[0,1,0],rotationAngle=0,scale=[1,1,1],
        velocity=[0,0,0],color=[70,120,180],entryTime=-1)
    # Same fluid configuration as coupled_scene.gpu_dfsph.assets.scene_configuration.
    scene=dict(Configuration=dict(domainStart=[0,0,0],domainEnd=domain_end,addDomainBox=False,
        particleRadius=spacing/2,density0=1000,gravitation=[0,-9.81,0],simulationMethod='dfsph',
        viscosityMethod='standard',viscosity=.001,viscosity_b=.001,timeStepSize=dt,exportObj=False),
        FluidBodies=[dict(common,objectId=0,translation=[0,0,0],density=1000)],
        RigidBodies=[dict(common,**spec),dict(common,**idle_spec)])
    (args.output/'scene.json').write_text(json.dumps(scene,indent=2))
    bodies=[dict(spec,vertices=np.asarray(plate.vertices,np.float32),
        triangles=np.asarray(plate.faces,np.int32),samples=samples),
        dict(idle_spec,vertices=np.asarray(idle.vertices,np.float32),
        triangles=np.asarray(idle.faces,np.int32),samples=idle_samples)]
    report=dict(status='initializing',shape=args.shape,size_m=args.size,impact_speed_m_s=args.speed,
        gap_m=args.gap,spacing_m=spacing,lattice_pitch_m=pitch,fluid_particles=len(positions),
        boundary_samples=len(samples),base_hz=args.hz,minimum_dt_divisor=args.minimum_dt_divisor,
        akinci_coefficient=args.akinci_coefficient,boundary_model='volume_maps_bender2019',
        pressure_convergence_retries=0,diagnostic_only=True,frames=[])
    def save():(args.output/'report.json').write_text(json.dumps(report,indent=2))
    save();start=time.perf_counter()
    from coupled_scene.gpu_dfsph.backend import create_backend
    import taichi as ti
    with (args.output/'solver.log').open('w',buffering=1024*1024) as log,contextlib.redirect_stdout(log):
        c,solver=create_backend(args.output/'scene.json',positions,velocities,bodies,spacing,dt,args.upstream,
            akinci_coefficient=args.akinci_coefficient,minimum_density_iterations=2,
            minimum_divergence_iterations=1,minimum_dt_divisor=args.minimum_dt_divisor,
            local_residual_boundary_scope='all',boundary_model='volume_maps_bender2019',
            pressure_convergence_retries=0)
        solver.rigid_solver.set_drives({idle_spec['objectId']:
            lambda t,com:(np.r_[idle_position,0.,0.,0.,1.],np.zeros(6))})
        def capture_frame(t):
            n=c.particle_num[None];mask=c.particle_materials.to_numpy()[:n]==c.material_fluid
            p=c.particle_positions.to_numpy()[:n][mask].astype(np.float64)-reference
            v=c.particle_velocities.to_numpy()[:n][mask].astype(np.float64)
            ids=c.stable_ids.to_numpy()[:n][mask]
            if not np.isfinite(p).all() or not np.isfinite(v).all():raise RuntimeError('Nonfinite fluid arrays')
            np.savez(capture/f'frame_{len(report["frames"]):04d}.npz',positions=p.astype(np.float32),
                velocities=v.astype(np.float32),ids=ids,simulated_seconds=t)
            report['frames'].append(dict(time_s=t,**frame_metrics(p,v,spacing,args.speed,.5*args.size+2.*spacing)))
        n=c.particle_num[None];fluid=c.particle_materials.to_numpy()[:n]==c.material_fluid
        report['initial_density_ratio_max']=float(c.particle_densities.to_numpy()[:n][fluid].max()/1000.)
        report['status']='running';capture_frame(0.)
        try:
            for index in range(1,int(round(args.seconds*1000./args.snapshot_ms))+1):
                target=index*args.snapshot_ms/1000.
                while c.total_time<target-1e-9:solver.adaptive_step(target-c.total_time)
                ti.sync();capture_frame(target)
            report['status']='completed'
        except BaseException as exc:
            report.update(status='failed',error=f'{type(exc).__name__}: {exc}');raise
        finally:
            report.update(accepted_substeps=solver.accepted_substeps,rejected_substeps=solver.rejected_substeps,
                unconverged_accepted_substeps=solver.unconverged_accepted_substeps,
                minimum_accepted_dt_s=solver.minimum_accepted_dt,maximum_accepted_dt_s=solver.maximum_accepted_dt,
                wall_seconds=time.perf_counter()-start);save()
    print(json.dumps({key:report[key] for key in ('status','spacing_m','fluid_particles','accepted_substeps',
        'rejected_substeps','unconverged_accepted_substeps','wall_seconds')}))


if __name__=='__main__':main()

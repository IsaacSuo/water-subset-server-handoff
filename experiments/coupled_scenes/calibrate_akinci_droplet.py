"""Effective surface tension of the Akinci model from a free droplet oscillation.

A prolate droplet is released at rest without gravity.  The period of its
lowest shape mode gives the surface tension the solver actually produces:

    sigma = rho R^3 (2 pi / T)^2 / 8      (Rayleigh, mode n = 2)

Same solver and fluid settings as the pouring runs.  Diagnostic only.
"""
import argparse
import contextlib
import json
import sys
import time
from pathlib import Path

import numpy as np
import trimesh

ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))

DOMAIN=np.array([.8,.8,.8])


def shape_ratio(positions):
    centred=positions-positions.mean(axis=0)
    second=np.mean(centred**2,axis=0)
    return float(np.sqrt(second[1]/(.5*(second[0]+second[2]))))


def damped_oscillation(times,ratio):
    """Fit A exp(-beta t) cos(omega_d t + phase) + offset to the shape ratio.

    Returns the undamped angular frequency sqrt(omega_d^2 + beta^2), which is
    what the Rayleigh formula needs, with the damping ratio and fit residual.
    """
    from scipy.optimize import curve_fit
    times=np.asarray(times);signal=np.asarray(ratio)-1.;crossings=[]
    for index in np.flatnonzero(signal[:-1]*signal[1:]<0.):
        fraction=signal[index]/(signal[index]-signal[index+1])
        crossings.append(float(times[index]+fraction*(times[index+1]-times[index])))
    if len(crossings)<2:return dict(sphere_crossings_s=crossings,fitted=False)
    model=lambda t,a,beta,omega,phase,offset:a*np.exp(-beta*t)*np.cos(omega*t+phase)+offset
    guess=(signal[0],1.,np.pi/(crossings[1]-crossings[0]),0.,0.)
    try:values,_=curve_fit(model,times,signal,p0=guess,maxfev=20000)
    except RuntimeError:return dict(sphere_crossings_s=crossings,fitted=False)
    beta,omega=float(values[1]),abs(float(values[2]));natural=float(np.hypot(omega,beta))
    return dict(sphere_crossings_s=crossings,fitted=True,damped_period_s=2.*np.pi/omega,
        decay_rate_per_s=beta,damping_ratio=beta/natural,natural_angular_frequency_rad_s=natural,
        fit_rms_residual=float(np.sqrt(np.mean((model(times,*values)-signal)**2))))


def main():
    parser=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--spacing',type=float,required=True)
    parser.add_argument('--akinci-coefficient',type=float,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--upstream',type=Path,default=Path('/mnt/y/tools/SPH_Project'))
    parser.add_argument('--radius',type=float,default=.03,help='Radius of the equal-volume sphere')
    parser.add_argument('--elongation',type=float,default=1.2,help='Initial axis ratio of the prolate droplet')
    parser.add_argument('--seconds',type=float,default=1.2)
    parser.add_argument('--hz',type=int,default=1200)
    parser.add_argument('--snapshot-ms',type=float,default=5.)
    parser.add_argument('--lattice-factor',type=float,default=.931)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    spacing=args.spacing;dt=1./args.hz;centre=.5*DOMAIN
    # Volume-preserving prolate ellipsoid, long axis along y.
    axes=args.radius*np.array([args.elongation**(-1/3),args.elongation**(2/3),args.elongation**(-1/3)])
    pitch=spacing*args.lattice_factor;count=int(np.ceil(axes.max()/pitch))
    axis=(np.arange(-count,count)+.5)*pitch
    points=np.stack(np.meshgrid(axis,axis,axis,indexing='ij'),-1).reshape(-1,3)
    positions=(points[np.sum((points/axes)**2,axis=1)<=1.]+centre).astype(np.float32)
    velocities=np.zeros_like(positions)
    # The adapter needs a static and a moving body; both sit in corners far from the droplet.
    block=trimesh.creation.box(extents=[.024]*3)
    samples=np.asarray(block.voxelized(pitch=spacing).fill().points,np.float32)
    arrays=dict(vertices=np.asarray(block.vertices,np.float32),triangles=np.asarray(block.faces,np.int32),samples=samples)
    idle_position=np.array([.06,DOMAIN[1]-.06,.06])
    specs=[dict(name='receiver',objectId=1,translation=[.06,.06,.06],quaternion_xyzw=[0,0,0,1],
            isDynamic=False,prescribed=False,density=1000),
        dict(name='donor',objectId=2,translation=idle_position.tolist(),quaternion_xyzw=[0,0,0,1],
            isDynamic=True,prescribed=True,density=1000)]
    common=dict(geometryFile='imported-arrays',rotationAxis=[0,1,0],rotationAngle=0,scale=[1,1,1],
        velocity=[0,0,0],color=[70,120,180],entryTime=-1)
    scene=dict(Configuration=dict(domainStart=[0,0,0],domainEnd=DOMAIN.tolist(),addDomainBox=False,
        particleRadius=spacing/2,density0=1000,gravitation=[0,0,0],simulationMethod='dfsph',
        viscosityMethod='standard',viscosity=.001,viscosity_b=.001,timeStepSize=dt,exportObj=False),
        FluidBodies=[dict(common,objectId=0,translation=[0,0,0],density=1000)],
        RigidBodies=[dict(common,**spec) for spec in specs])
    (args.output/'scene.json').write_text(json.dumps(scene,indent=2))
    report=dict(status='initializing',spacing_m=spacing,akinci_coefficient=args.akinci_coefficient,
        radius_m=args.radius,elongation=args.elongation,fluid_particles=len(positions),base_hz=args.hz,
        gravity=0.,times_s=[],shape_ratio=[])
    def save():(args.output/'report.json').write_text(json.dumps(report,indent=2))
    save();start=time.perf_counter()
    from coupled_scene.gpu_dfsph.backend import create_backend
    import taichi as ti
    with (args.output/'solver.log').open('w',buffering=1024*1024) as log,contextlib.redirect_stdout(log):
        c,solver=create_backend(args.output/'scene.json',positions,velocities,[dict(spec,**arrays) for spec in specs],
            spacing,dt,args.upstream,akinci_coefficient=args.akinci_coefficient,minimum_density_iterations=2,
            minimum_divergence_iterations=1,minimum_dt_divisor=64,local_residual_boundary_scope='all',
            boundary_model='volume_maps_bender2019',pressure_convergence_retries=0)
        solver.rigid_solver.set_drives({2:lambda t,com:(np.r_[idle_position,0.,0.,0.,1.],np.zeros(6))})
        def capture(t):
            n=c.particle_num[None];mask=c.particle_materials.to_numpy()[:n]==c.material_fluid
            p=c.particle_positions.to_numpy()[:n][mask].astype(np.float64)
            if not np.isfinite(p).all():raise RuntimeError('Nonfinite fluid positions')
            report['times_s'].append(t);report['shape_ratio'].append(shape_ratio(p))
        report['status']='running';capture(0.)
        try:
            for index in range(1,int(round(args.seconds*1000./args.snapshot_ms))+1):
                target=index*args.snapshot_ms/1000.
                while c.total_time<target-1e-9:solver.adaptive_step(target-c.total_time)
                ti.sync();capture(target)
            report['status']='completed'
        except BaseException as exc:
            report.update(status='failed',error=f'{type(exc).__name__}: {exc}');raise
        finally:
            fit=damped_oscillation(report['times_s'],report['shape_ratio'])
            report.update(fit,effective_surface_tension_n_m=(
                    1000.*args.radius**3*fit['natural_angular_frequency_rad_s']**2/8. if fit['fitted'] else None),
                accepted_substeps=solver.accepted_substeps,
                unconverged_accepted_substeps=solver.unconverged_accepted_substeps,
                wall_seconds=time.perf_counter()-start);save()
    print(json.dumps({key:report[key] for key in ('status','spacing_m','akinci_coefficient','fluid_particles',
        'damped_period_s','damping_ratio','fit_rms_residual','effective_surface_tension_n_m','wall_seconds')
        if key in report}))


if __name__=='__main__':main()

"""Lossless mesh/motion import of accepted active-drive assets.

Coarse lattice sampling is an explicit preview option, never production data.
The render/world origin is retained in metadata; simulation uses a local origin.
"""
import hashlib
import json
import shutil
from pathlib import Path
import numpy as np


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_assets(folder, stride=1):
    folder = Path(folder)
    meta = json.loads((folder/'assets.json').read_text(encoding='utf-8'))
    geometry = folder/'geometry_and_fill.npz'
    if digest(geometry) != meta['geometry_sha256']:
        raise ValueError('Asset geometry hash mismatch')
    if meta['case']['id'] not in ('01_container_transfer', '02_stirring', '03_piston_push'):
        raise ValueError('Unsupported active-drive asset')
    if stride < 1 or int(stride) != stride:
        raise ValueError('Preview stride must be a positive integer')
    with np.load(geometry) as source:
        arrays = {key: source[key].copy() for key in source.files}
    spacing = float(meta.get('particle_spacing_m', .004))
    positions = arrays['positions']
    ids = np.arange(len(positions), dtype=np.uint32)
    if stride > 1:
        lattice = (positions-positions.min(axis=0))/spacing
        if np.max(np.abs(lattice-np.rint(lattice))) > .01:
            raise ValueError('Preview requires an axis-aligned initial lattice')
        selected = np.all(np.rint(lattice).astype(np.int64)%stride == 0, axis=1)
        arrays['positions'], arrays['velocities'], ids = positions[selected], arrays['velocities'][selected], ids[selected]
    origin = np.asarray(meta['receiver']['position_m'], dtype=np.float64)
    arrays['positions'] = (arrays['positions']-origin).astype(np.float32)
    return meta, arrays, ids, origin, spacing*stride


def write_scene(output, meta, arrays, origin, spacing, bridge, dt):
    output = Path(output)
    bodies = []
    for name in ('receiver', 'donor'):
        vertices, triangles = arrays[name+'_vertices'], arrays[name+'_triangles']
        if not len(vertices):
            continue
        mesh = output/(name+'.obj')
        with mesh.open('w') as stream:
            for v in vertices: stream.write('v '+' '.join(format(float(x), '.9g') for x in v)+'\n')
            for f in triangles: stream.write('f '+' '.join(str(int(x)+1) for x in f)+'\n')
        resolution = np.clip(np.ceil(np.ptp(vertices, axis=0)/(spacing*.5)).astype(int), 32, 256)
        bodies.append(dict(geometryFile=str(mesh.resolve()), translation=(np.asarray(meta[name]['position_m'])-origin).tolist(),
            rotationAxis=[1,0,0], rotationAngle=0., scale=[1,1,1], isDynamic=name=='donor',
            isWall=False, density=1000., mapInvert=False, mapThickness=0., mapResolution=resolution.tolist()))
    particle_file = output/'initial.bgeo'
    bridge.write_particles(particle_file, arrays['positions'], arrays['velocities'], spacing/2)
    scene = dict(Configuration=dict(particleRadius=spacing/2, simulationMethod=4, gravitation=[0,-9.81,0],
        timeStepSize=dt, cflMethod=0, boundaryHandlingMethod=2, enableVTKExport=False, enableStateExport=False,
        enableAsyncExport=False, DFSPH=dict(minIterations=2,maxIterations=300,maxError=.005,maxIterationsV=300,maxErrorV=.005,enableDivergenceSolver=True)),
        Materials=[dict(id='Fluid',density0=1000,viscosityMethod=1,surfaceTensionMethod=0,vorticityMethod=0,
            **{'Standard viscosity':dict(viscosity=1e-6,viscosityBoundary=1e-6)})], RigidBodies=bodies,
        FluidModels=[dict(id='Fluid',particleFile=str(particle_file.resolve()),translation=[0,0,0],rotationAxis=[1,0,0],rotationAngle=0.,scale=[1,1,1],initialVelocity=[0,0,0])])
    path = output/'scene.json'; path.write_text(json.dumps(scene, indent=2))
    return path


def reuse_boundary_cache(source, destination, dll):
    """Reuse only maps with identical meshes, boundary transforms and build.

    Time-step changes are allowed; spatial discretization changes are not.
    Copying keeps each experiment self-contained and avoids cache races.
    """
    source,destination=Path(source),Path(destination)
    previous=json.loads((source/'probe_report.json').read_text(encoding='utf-8'))
    if previous.get('status')!='completed' and not previous.get('boundary_cache_ready'):
        raise ValueError('Source boundary initialization did not complete')
    if previous.get('bridge_sha256')!=digest(dll):
        raise ValueError('Boundary cache was built by a different native bridge')
    def signature(directory):
        scene=json.loads((directory/'scene.json').read_text(encoding='utf-8'))
        config=dict(scene['Configuration']);config.pop('timeStepSize',None)
        boundaries=[]
        for boundary in scene['RigidBodies']:
            values=dict(boundary);mesh=Path(values.pop('geometryFile'))
            if not mesh.is_absolute():mesh=directory/mesh
            boundaries.append(dict(values,mesh_name=mesh.name,mesh_sha256=digest(mesh)))
        return dict(configuration=config,boundaries=boundaries)
    if signature(source)!=signature(destination):
        raise ValueError('Boundary cache geometry, transform or spatial configuration mismatch')
    files=sorted((source/'Cache').iterdir())
    if not any(path.suffix=='.cdm' for path in files):raise ValueError('No volume maps in source cache')
    target=destination/'Cache';target.mkdir(exist_ok=False)
    copied={}
    for path in files:
        if path.is_file():
            shutil.copy2(path,target/path.name);copied[path.name]=digest(target/path.name)
    return dict(source=str(source.resolve()),files_sha256=copied)

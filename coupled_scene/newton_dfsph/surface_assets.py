"""Import the five authored surface layouts into the water-only bridge.

The tank interior, actuator dimensions, world origin and action are taken from
the existing handoff. Coarse lattices are diagnostics, not accepted assets.
"""
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import trimesh

from coupled_scene.surface_study import actuator_spec, motion_state
from coupled_scene.newton_dfsph.assets import digest


CASE_IDS = ('01_still_water', '02_sloshing', '03_wave_propagation',
            '04_wave_reflection', '05_surface_recovery')


def tank_mesh(tank):
    x, h, z = tank['inner_size_m']
    t = tank['proxy_thickness_m']
    vertices = []
    for half_x, y, half_z in ((x/2+t, -t, z/2+t), (x/2+t, h, z/2+t),
                              (x/2, h, z/2), (x/2, 0., z/2)):
        vertices.extend([[-half_x,y,-half_z], [half_x,y,-half_z],
                         [half_x,y,half_z], [-half_x,y,half_z]])
    faces = []
    def quad(a,b,c,d):
        faces.extend(((a,b,c), (a,c,d)))
    for ring in range(3):
        for i in range(4):
            j=(i+1)%4
            quad(4*ring+i, 4*ring+j, 4*(ring+1)+j, 4*(ring+1)+i)
    quad(3,2,1,0)
    quad(12,13,14,15)
    mesh=trimesh.Trimesh(vertices=vertices, faces=faces, process=True)
    mesh.fix_normals()
    if not mesh.is_watertight or mesh.volume <= 0:
        raise ValueError('Invalid tank shell')
    return mesh


def load_surface_layout(folder, stride=1):
    folder=Path(folder)
    spec_path=folder/'scene_spec.json'
    spec=json.loads(spec_path.read_text(encoding='utf-8'))
    case=spec['case']; tank=spec['tank']
    if case['id'] not in CASE_IDS or stride < 1 or int(stride) != stride:
        raise ValueError('Unsupported surface case or lattice stride')
    spacing=.004*stride
    x,h,z=tank['inner_size_m']; depth=tank['reference_water_depth_m']
    clearance=max(.008,2*spacing)
    counts=[math.floor((x-2*clearance)/spacing+1e-8),
            math.floor((depth-.5*spacing-clearance)/spacing+1e-8)+1,
            math.floor((z-2*clearance)/spacing+1e-8)]
    if min(counts)<1:
        raise ValueError('Preview resolution cannot represent the water depth')
    axes=[(np.arange(n,dtype=np.float32)-(n-1)/2)*spacing for n in counts]
    axes[1]=clearance+np.arange(counts[1],dtype=np.float32)*spacing
    positions=np.stack(np.meshgrid(*axes,indexing='ij'),axis=-1).reshape(-1,3)
    wall=tank_mesh(tank)
    actuator=actuator_spec(case['actuator'],tank)
    origin=np.asarray(spec['tank_origin_isaac'],dtype=np.float64)
    if actuator is None:
        fixed=None; moving=wall; pivot=origin
    else:
        fixed=wall; pivot=origin+actuator['centre']
        if actuator['shape']=='Cube':
            moving=trimesh.creation.box(extents=actuator['size'])
            local=positions-np.asarray(actuator['centre'])
            # Match the two-spacing shell clearance used by the original water
            # assets. Merely excluding particle centres inside the solid leaves
            # an over-dense contact layer that ejects water before any action.
            keep=np.any(np.abs(local)>np.asarray(actuator['size'])/2+clearance,axis=1)
        else:
            moving=trimesh.creation.cylinder(radius=actuator['radius'],height=actuator['height'],sections=128)
            # Trimesh cylinders are Z-up; authored actuator is Y-up.
            moving.apply_transform(trimesh.transformations.rotation_matrix(-np.pi/2,[1,0,0]))
            local=positions-np.asarray(actuator['centre'])
            keep=(np.hypot(local[:,0],local[:,2])>actuator['radius']+clearance)|(np.abs(local[:,1])>actuator['height']/2+clearance)
        positions=positions[keep]
    arrays=dict(positions=positions.astype(np.float32),velocities=np.zeros_like(positions,dtype=np.float32))
    for name,mesh in (('receiver',fixed),('donor',moving)):
        arrays[name+'_vertices']=np.empty((0,3),np.float32) if mesh is None else np.asarray(mesh.vertices,np.float32)
        arrays[name+'_triangles']=np.empty((0,3),np.int32) if mesh is None else np.asarray(mesh.faces,np.int32)
    blend=folder/(case['id']+'.blend')
    geometry=hashlib.sha256()
    for key in sorted(arrays):
        array=np.ascontiguousarray(arrays[key])
        geometry.update(key.encode('ascii'))
        geometry.update(str((array.shape,array.dtype.str)).encode('ascii'))
        geometry.update(array.tobytes())
    meta=dict(product='surface_study_newton_import',scene_family='surface_study',case=case,
        source_layout=str(folder.resolve()),source_blend=str(blend.resolve()),
        source_blend_sha256=digest(blend),geometry_sha256=geometry.hexdigest(),source_scene_spec_sha256=digest(spec_path),
        geometry_identity='sorted named arrays: shapes, dtypes and bytes of collision geometry, positions and velocities',
        particle_count=len(positions),particle_spacing_m=spacing,tank=tank,
        moving_object='Tank' if actuator is None else case['actuator'],
        receiver=dict(position_m=origin.tolist()),donor=dict(position_m=pivot.tolist()),
        diagnostic_origin_m=origin.tolist(),containment_bounds_local_m=[[-x/2,0,-z/2],[x/2,h,z/2]],
        initial_solid_exclusion=True,initial_shell_clearance_m=clearance,
        diagnostic_lattice_rebuilt=stride!=1,production_accepted=False)
    return meta,arrays,np.arange(len(positions),dtype=np.uint32),origin,spacing


def surface_motion(motion,seconds):
    displacement,velocity=motion_state(motion,seconds)
    return dict(displacement_m=displacement,linear_velocity_m_s=velocity,
                rotation_axis=1,angle_rad=0.,angular_velocity_rad_s=[0.,0.,0.])

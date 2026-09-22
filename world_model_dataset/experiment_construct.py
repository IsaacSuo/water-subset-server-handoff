"""Object + selected original region + conditions -> executable experiment.

This entry only reads meshes and writes JSON. It does not import execution
adapters, prepare a solver, render, inspect a GPU, or consume a physics cache.
"""
import argparse
import copy
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from shapely.geometry import LineString, box

from .experiment_contract import (VERSION, differences, identifier, keys,
                                  native_config, positive, rigid_spec, validate_common)
from .experiment_geometry import Geometry, rectangle, require
from .io import digest, file_hash, read_json, write_json


def transform(position):
    m = np.eye(4); m[:3, 3] = position
    return m.tolist()


def number(obj, key, default=None, zero=False):
    value = obj.get(key, default)
    positive(value, key, zero=zero)
    return value


def body_geometry(obj, library, geometry):
    keys(obj, {'id', 'asset', 'size_m', 'mass_kg', 'density_kg_m3', 'rotation_deg', 'material'},
         {'id', 'asset', 'size_m', 'rotation_deg'}, 'rigid object')
    identifier(obj['id']); identifier(obj['asset'])
    number(obj, 'size_m')
    require(('mass_kg' in obj) != ('density_kg_m3' in obj), 'mass', 'choose mass or density')
    path = Path(library['exploration_root']) / library['library'] / obj['asset']
    meta = read_json(path/'asset.json'); geometry.pin(path/'asset.json')
    require(geometry.pin(path/meta['geometry']) == meta['geometry_sha256'], 'asset_hash', str(path))
    require(meta['format'] == 'physical-asset-source/1', 'asset', 'normalized source required')
    with np.load(path/meta['geometry'], allow_pickle=False) as z:
        vertices = z['vertices'].astype(float) * obj['size_m']
    require(np.isfinite(vertices).all(), 'asset', 'nonfinite geometry')
    posed = Rotation.from_euler('xyz', obj['rotation_deg'], degrees=True).apply(vertices)
    return posed.min(0), posed.max(0)


def passage_camera(geo, initial_center, target, span):
    """Frame the local approach/exit; reject source-mesh occlusion before execution."""
    import trimesh
    triangles=np.concatenate(list(geo.meshes.values()))
    mesh=trimesh.Trimesh(triangles.reshape(-1,3),np.arange(triangles.size//3).reshape(-1,3),process=False)
    anchors=np.asarray([initial_center,target])
    for direction in ([.8,-.25,.35],[.8,.25,.35],[0,-.8,.4],[0,.8,.4],[.1,-.1,.8]):
        origin=np.asarray(target)+span*np.asarray(direction)
        rays=anchors-origin;lengths=np.linalg.norm(rays,axis=1)
        locations,indices,_=mesh.ray.intersects_location(np.repeat(origin[None],len(anchors),axis=0),
                                                       rays/lengths[:,None],multiple_hits=True)
        if len(locations) and np.any(np.linalg.norm(locations-origin,axis=1)<lengths[indices]-1e-5):
            continue
        return dict(target_m=np.asarray(target).tolist(),position_m=origin.tolist(),ortho_scale_m=float(span))
    raise ValueError('observation_visibility: no unoccluded local passage camera; select another region')


def drape(obj, cond, profile, geo, support):
    keys(obj, {'size_m','kind','path','scale'}, (), 'cloth object')
    keys(cond, {'overhang_fraction'}, {'overhang_fraction'}, 'drape conditions')
    mesh_input=None; mesh_details=None
    if obj.get('kind','rectangle')=='mesh':
        keys(obj,{'kind','path','scale'},{'kind','path'},'cloth mesh object')
        import trimesh
        from shapely.geometry import Polygon
        from shapely.ops import unary_union
        path=Path(obj['path']); checksum=geo.pin(path)
        scale=number(obj,'scale',1.)
        with np.load(path,allow_pickle=False) as arrays:
            v=arrays['vertices'].astype(float)*scale; faces=arrays['triangles']
        mesh=trimesh.Trimesh(v,faces,process=False)
        require(np.isfinite(v).all() and np.ptp(v[:,2])<5e-6 and mesh.is_winding_consistent,
                'cloth_mesh','require a finite horizontal, consistently wound rectangular rest surface')
        outline=unary_union([Polygon(t[:,:2]) for t in v[faces]])
        rect=outline.minimum_rotated_rectangle
        require(abs(rect.area-outline.area)<1e-6*rect.area and abs(mesh.area-outline.area)<1e-6*rect.area,
                'cloth_mesh','holes, folds, overlap or nonrectangular boundary not supported')
        corners=np.asarray(rect.exterior.coords)[:4]; edges=np.roll(corners,-1,axis=0)-corners
        longest=edges[np.argmax(np.linalg.norm(edges,axis=1))]; u=longest/np.linalg.norm(longest)
        if u[0]<0: u=-u
        rotation=np.array([[u[0],u[1],0],[-u[1],u[0],0],[0,0,1]])
        local=v@rotation.T; low,high=local.min(0),local.max(0)
        size=(high-low)[:2]; local_center=(low+high)/2
        mesh_input=dict(kind='mesh',path=str(path.resolve()),scale=scale)
        mesh_details=dict(source_sha256=checksum,vertices=len(v),triangles=len(faces),
                          topology='original identity retained; no resampling',inferred_size_m=size.tolist())
    else:
        keys(obj,{'kind','size_m'},{'size_m'},'rectangle cloth object')
        require(obj.get('kind','rectangle')=='rectangle','cloth_kind','unsupported cloth shape')
        size = np.asarray(obj['size_m'], float)
    require(size.shape == (2,) and np.isfinite(size).all() and np.all(size > 0), 'cloth_size', 'two positive metre dimensions')
    fraction = cond['overhang_fraction']
    require(isinstance(fraction, (int, float)) and .05 <= fraction <= .8, 'overhang', 'fraction must be 0.05..0.8')
    z, surface = geo.support(support)
    lo, hi = geo.bounds
    # Edge closest to +world-X on the actual support, never the ROI boundary.
    components = list(surface.geoms) if hasattr(surface, 'geoms') else [surface]
    components = [p for p in components if p.intersects(box(*lo[:2], *hi[:2]))]
    require(len(components) == 1, 'support_selection', 'select exactly one connected support patch')
    surface = components[0]
    rect=np.asarray(surface.minimum_rotated_rectangle.exterior.coords)[:4]
    directions=np.roll(rect,-1,axis=0)-rect
    normals=np.column_stack([directions[:,1],-directions[:,0]])
    normals /= np.linalg.norm(normals,axis=1)[:,None]
    normal=normals[np.argmax(normals[:,0])]
    from shapely.affinity import affine_transform
    geo,frame=geo.horizontal_analysis_frame(normal)
    surface=affine_transform(surface,[frame[0,0],frame[0,1],frame[1,0],frame[1,1],0,0])
    lo,hi=geo.bounds; edge=surface.bounds[2]
    x, y = edge + (fraction-.5)*size[0], (lo[1]+hi[1])/2
    footprint = rectangle([x, y], size)
    overlap = footprint.intersection(surface).area / footprint.area
    require(abs(overlap-(1-fraction)) < 1e-5, 'support_shape', 'need a straight +X edge with full transverse support')
    cfg = copy.deepcopy(profile['input'])
    gap = max(cfg['collision']['contact_offset_m']*2, cfg['material']['thickness_m']*2)
    position = np.array([x, y, z+gap]); ext = np.r_[size/2, 0.000001]
    geo.check_box(position-ext, position+ext)
    # The hanging strip can descend at most its authored free length. This is a
    # conservative reserved prism, not a simulated hanging shape.
    free = size[0]*fraction
    clear_start=geo.edge_transition(edge,y-size[1]/2,y+size[1]/2,z,free+gap,free)
    geo.check_box([clear_start, y-size[1]/2, z-free-gap],
                  [x+size[0]/2, y+size[1]/2, z-1e-5], 'hanging_clearance')
    spacing = profile['discretization_m']
    cells = np.ceil(size/spacing).astype(int)
    node_count = mesh_details['vertices'] if mesh_input else np.prod(cells+1)
    require(node_count <= 20000, 'discretization', 'profile node budget exceeded')
    if mesh_input:
        matrix=np.eye(4); matrix[:3,:3]=frame.T@rotation; matrix[:3,3]=frame.T@(position-local_center)
        cfg['cloth']=dict(mesh_input,world_from_mesh=matrix.tolist())
    else:
        matrix=np.eye(4); matrix[:3,:3]=frame.T; matrix[:3,3]=frame.T@position
        cfg['cloth'] = dict(kind='rectangle', size_m=size.tolist(), cells=cells.tolist(), world_from_mesh=matrix.tolist())
    return cfg, dict(support_height_m=z, edge_x_m=edge, supported_area_fraction=overlap,
                     edge_frame_outward_world_xy=normal.tolist(),world_edge_point_m=(frame.T@np.array([edge,y,z])).tolist(),
                     free_length_m=free, reserved_drop_m=free+gap, initial_gap_m=gap,
                     edge_contact_band_m=[edge,clear_start], edge_contact_effect='unverified; contact allowed in this band',
                     cells=None if mesh_input else cells.tolist(),mesh_asset=mesh_details,
                     attachment='none; passive contact support')


def rigid(obj, cond, profile, geo, support, phenomenon):
    objects = obj if isinstance(obj, list) else [obj]
    library = profile['asset_library']
    bounds = [body_geometry(o, library, geo) for o in objects]
    z, surface = geo.support(support); lo, hi = geo.bounds
    margin = .005
    bodies = []
    if phenomenon == 'geometry_constrained_motion':
        keys(cond, {'speed_m_s', 'clearance_regime'}, {'speed_m_s', 'clearance_regime'}, 'passage conditions')
        require(len(objects) == 1, 'participants', 'passage rule takes one object')
        low, high = bounds[0]; width = high[1]-low[1]; length = high[0]-low[0]
        bottom = z+margin; top = bottom+high[2]-low[2]
        obstacles = geo.projection(bottom, top).intersection(box(*lo[:2], *hi[:2]))
        require(not obstacles.is_empty, 'restriction_missing', 'selected region has no restriction at object height')
        # Evaluate all projected polygon vertices and interval midpoints. Free
        # corridor is measured around the selected region's centreline.
        def coords(g):
            if hasattr(g, 'geoms'):
                return [c for part in g.geoms for c in coords(part)]
            if hasattr(g,'exterior'):
                return list(g.exterior.coords)+[c for ring in g.interiors for c in ring.coords]
            return list(g.coords)
        events = sorted(set([lo[0], hi[0]] + [p[0] for p in coords(obstacles)]))
        samples = sorted(set(events + [(a+b)/2 for a,b in zip(events[:-1], events[1:])]))
        cy = (lo[1]+hi[1])/2
        gaps = []
        for x in samples:
            line = LineString([(x, lo[1]), (x, hi[1])]).difference(obstacles.buffer(1e-8))
            parts = list(line.geoms) if hasattr(line, 'geoms') else [line]
            intervals = [(p.bounds[1], p.bounds[3]) for p in parts if not p.is_empty and p.bounds[1] < cy < p.bounds[3]]
            require(intervals, 'corridor', 'centreline fully blocked; this rule requires an open gap')
            a,b = intervals[0]; gaps.append((b-a, x, a,b))
        _, throat, _,_ = min(gaps)
        # A single initial heading cannot follow a bent corridor. Intersect
        # transverse free intervals across all sections, not just the narrowest.
        a=max(row[2] for row in gaps); b=min(row[3] for row in gaps)
        gap=b-a
        require(gap>0,'corridor','no common straight +X corridor; curved route construction unavailable')
        require(gap < hi[1]-lo[1] - 1e-5, 'restriction_missing', 'no narrower gap detected')
        signed = gap-width
        regime = cond['clearance_regime']
        require(regime in ('passable', 'obstructed'), 'regime', 'choose passable or obstructed geometric envelope')
        require((signed > 2*margin) if regime == 'passable' else (signed < -2*margin),
                'clearance_regime', f'gap={gap:.6g}, object width={width:.6g}; requested regime does not fit')
        # Start strictly before the first restricting geometry, with one object
        # length of approach. Scene is never widened to satisfy this condition.
        corridor_center=(a+b)/2
        limiting=[row[1] for row in gaps
                  if 2*min(corridor_center-row[2],row[3]-corridor_center)<width+2*margin]
        approach_to=min(limiting) if regime=='obstructed' and limiting else throat
        x = approach_to-high[0]-max(length*.5, .02)
        y = (a+b)/2-(low[1]+high[1])/2
        require(hi[0]-throat >= length+margin, 'exit_space', 'insufficient downstream observation region')
        require(x+low[0] >= lo[0]+margin, 'approach_space', 'select more upstream space before the measured restriction')
        placements = [(x,y)]
        details = dict(gap_m=gap, object_width_m=width, signed_clearance_m=signed,
                       restriction_object_candidates=geo.restriction_objects(bottom,top),
                       throat_x_m=throat, approach_to_x_m=approach_to, approach_distance_m=max(length*.5,.02), regime=regime,
                       limits='orientation-specific AABB clearance; passage/jamming outcome unverified')
        initial_center=np.array([x+(low[0]+high[0])/2,corridor_center,(bottom+top)/2])
        target=initial_center.copy();target[0]=(initial_center[0]+hi[0])/2
        span=1.5*max(hi[0]-(x+low[0]),width*3,(top-bottom)*3)
        details['observation_camera']=passage_camera(geo,initial_center,target,span)
        details['camera_check']='original-mesh visibility to initial centre and local path target; future visibility unverified'
    elif phenomenon == 'multibody_collision_propagation':
        keys(cond, {'speed_m_s', 'gap_ratio'}, {'speed_m_s', 'gap_ratio'}, 'multibody conditions')
        require(len(objects) >= 2, 'participants', 'at least two objects required')
        gap_ratio = number(cond, 'gap_ratio', zero=True)
        lengths = [b[0][0]*-1+b[1][0] for b in bounds]
        gap = gap_ratio*min(lengths)
        total = sum(lengths)+gap*(len(objects)-1)
        require(total+max(lengths)+2*margin < hi[0]-lo[0], 'layout_fit', 'chain plus downstream margin does not fit')
        cursor = (lo[0]+hi[0]-total)/2
        placements = []
        for low, high in bounds:
            placements.append((cursor-low[0], (lo[1]+hi[1]-low[1]-high[1])/2))
            cursor += high[0]-low[0]+gap
        details = dict(layout='single +X collision chain', interbody_gap_m=gap,
                       driven_initial_body=objects[0]['id'])
    else:
        keys(cond, {'speed_m_s', 'spin_ratio'}, {'speed_m_s', 'spin_ratio'}, 'roll conditions')
        require(len(objects) == 1, 'participants', 'roll rule takes one object')
        low, high = bounds[0]
        placements = [(lo[0]-low[0]+margin, (lo[1]+hi[1]-low[1]-high[1])/2)]
        require(np.isfinite(cond['spin_ratio']), 'spin_ratio', 'finite scalar required')
        details = dict(effective_radius_m=(high[2]-low[2])/2,
                       limits='AABB effective radius; initial rolling tendency, not no-slip guarantee')
    speed = number(cond, 'speed_m_s', zero=True)
    for i, (o, (low,high), (x,y)) in enumerate(zip(objects,bounds,placements)):
        center = np.array([x,y,z+margin-low[2]])
        footprint = box(*(center+low)[:2], *(center+high)[:2])
        require(surface.buffer(1e-7).covers(footprint), 'support_coverage', 'initial whole footprint not on selected original support')
        geo.check_box(center+low, center+high)
        b = dict(copy.deepcopy(o), xy_m=[x,y], support_group=support, ray_start_z_m=z+.0001,
                 clearance_m=margin, velocity_m_s=[speed if i==0 else 0,0,0])
        if phenomenon == 'rigid_roll_slide':
            b['angular_velocity_rad_s'] = [0, speed*cond['spin_ratio']/details['effective_radius_m'], 0]
        bodies.append(b)
    require(len({o['id'] for o in objects}) == len(objects), 'participants', 'duplicate id')
    return dict(participants=bodies, asset_library=copy.deepcopy(library)), details


def construct(request, profile):
    """Pure CPU construction; caller chooses profile, objects, region and intent."""
    keys(request, {'format','id','phenomenon','profile','scene','support_group','object','conditions'},
         {'format','id','phenomenon','profile','scene','support_group','object','conditions'}, 'construction request')
    require(request['format'] == 'phenomenon-construction/1', 'format', 'unsupported request')
    identifier(request['id'])
    phenomenon = request['phenomenon']
    require(phenomenon!='multibody_rearrangement','construction_missing',
            'rearrangement needs support-aware unstable packing and settling layout rules; '
            'collision propagation is a separate phenomenon, not a replacement')
    require(profile['phenomenon'] == phenomenon, 'profile', 'wrong phenomenon')
    geo = Geometry(request['scene'])
    args = (request['object'], request['conditions'], profile, geo, request['support_group'])
    if phenomenon == 'cloth_drape':
        inputs, calculations = drape(*args)
    elif phenomenon in ('geometry_constrained_motion', 'rigid_roll_slide', 'multibody_collision_propagation'):
        inputs, calculations = rigid(*args, phenomenon)
    else:
        from .experiment_construct_material import material
        inputs, calculations = material(*args, phenomenon)
    lo, hi = geo.bounds; target = (lo+hi)/2; span = float(max(hi-lo))
    doc = dict(format=VERSION, id=request['id'], phenomenon=phenomenon,
               backend=copy.deepcopy(profile['backend']), scene=copy.deepcopy(request['scene']), input=inputs,
               control=copy.deepcopy(profile['control']), actuation=copy.deepcopy(profile['actuation']),
               timing=copy.deepcopy(profile['timing']),
               observations=dict(hz=10, camera=calculations.get('observation_camera',dict(target_m=target.tolist(),
                   position_m=(target + span*np.array([1,-1,.8])).tolist(), ortho_scale_m=span*1.5))),
               conditions=[dict(id='baseline', changes={}, derived_impacts=[])])
    kind = validate_common(doc)
    compiled = rigid_spec(doc, doc['id']+'_baseline') if kind == 'rigid' else native_config(doc)
    report = dict(format='construction-report/1', constructor='implemented',
                  geometry_checks='passed', backend_contract='local compiler passed; native contract check separate',
                  calculations=calculations,support_identity=geo.support_identity,
                  source_pins=geo.pins, request_sha256=digest(request),
                  profile_sha256=digest(profile), experiment_sha256=digest(doc),
                  physical_effect='unverified_new_configuration', prior_physics=profile.get('prior_physics'),
                  contact_force='unavailable', attachment_reaction='unavailable', rope_native_tension='unavailable',
                  training_admission=False)
    return doc, compiled, report


def compare_requests(base, changed, profile):
    """Reconstruct both sides; report actual derived changes, not caller prose."""
    a, _, ar = construct(base, profile); b, _, br = construct(changed, profile)
    requested = differences(base, changed)
    require(all(p.startswith(('/object/', '/conditions/')) for p in requested),
            'comparison_scope', 'same-region condition comparisons may only change objects/conditions')
    invariant_keys = ('scene', 'backend', 'timing', 'control', 'actuation', 'observations')
    require(all(a[k] == b[k] for k in invariant_keys), 'invariant', 'nonintervention configuration changed')
    for key in profile.get('input', {}):
        require(a['input'][key] == b['input'][key], 'invariant', 'profile material/numerics changed: '+key)
    return dict(request_changes=requested, derived_changes=differences(a['input'], b['input'], '/input'),
                profile_material_numerics_unchanged=True,
                invariant_sha256=digest({k:a[k] for k in invariant_keys}),
                before=ar['calculations'], after=br['calculations'])


def construct_pair(base, changed, profile):
    """Compile semantic interventions into the existing condition ledger."""
    comparison=compare_requests(base,changed,profile)
    a,_,_=construct(base,profile); b,_,report=construct(changed,profile)
    require(comparison['derived_changes'], 'no_effect', 'semantic request has no effective configuration change')
    require(a['input'].keys()==b['input'].keys(), 'condition_shape', 'top-level backend input presence cannot change in a pair')
    changes={'/input/'+k: b['input'][k] for k in a['input'] if a['input'][k]!=b['input'][k]}
    a['conditions'].append(dict(id='variant',changes=changes,
        derived_impacts=comparison['derived_changes']))
    report['semantic_intervention']=comparison
    report['experiment_sha256']=digest(a)
    return a,report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--request', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--reference', type=Path, help='Same-region baseline request; compile baseline and derived variant conditions')
    p.add_argument('--reuse-physics-cache',type=Path,
                   help='Bind an existing native run after metadata-only reconstruction; executor verifies physical equivalence')
    a=p.parse_args(); request=read_json(a.request)
    profile_path=(a.request.parent/request['profile']).resolve()
    profile=read_json(profile_path)
    doc, compiled, report=construct(request, profile)
    if a.reuse_physics_cache:
        require(a.reference is None,'cache_scope','cache reuse currently accepts one baseline only')
        cache=a.reuse_physics_cache.resolve()
        manifest=next((cache/n for n in ('episode.json','episode.physics.json') if (cache/n).is_file()),None)
        require(manifest is not None,'cache_manifest','existing packaged physics required')
        doc['conditions'][0]['cache']=dict(run=str(cache),manifest=manifest.name,manifest_sha256=file_hash(manifest))
        report['cache_reuse_requested']=dict(run=str(cache),physical_equivalence='must be verified by execution prepare; not assumed')
    if a.reference:
        base=read_json(a.reference)
        reference_profile=read_json((a.reference.parent/base['profile']).resolve())
        require(reference_profile==profile,'profile','condition pair must use identical profile')
        doc,report=construct_pair(base,request,profile)
        report['source_pins'][str(a.reference.resolve())]=file_hash(a.reference)
    if doc['backend']['kind']!='rigid':
        from .experiment_material_bridge import check_native_contract
        check_native_contract(doc['backend']['entry'],native_config(doc))
        # The changed side of a semantic pair is checked separately as well.
        if a.reference:
            variant,_,_=construct(request,profile)
            check_native_contract(variant['backend']['entry'],native_config(variant))
        report['backend_contract']='native input_contract.normalize passed; no prepare/solver/render'
        for source in Path(doc['backend']['entry']).parent.glob('*.py'):
            report['source_pins'][str(source)]=file_hash(source)
        report['source_pins'][doc['backend']['runtime']]=file_hash(doc['backend']['runtime'])
    report['source_pins'][str(profile_path)]=file_hash(profile_path)
    report['source_pins'][str(a.request.resolve())]=file_hash(a.request)
    for name in ('experiment_construct.py','experiment_construct_material.py','experiment_geometry.py','experiment_contract.py'):
        path=Path(__file__).with_name(name)
        if path.exists(): report['source_pins'][str(path)]=file_hash(path)
    a.output.mkdir(parents=True, exist_ok=False)
    # Persist construction provenance into the executable request so prepare
    # refuses changed geometry/profile/code instead of silently rebuilding it.
    doc['scene']['source_records']['construction']=str((a.output/'construction.json').resolve())
    report['experiment_sha256']=digest(doc)
    compiled=rigid_spec(doc,doc['id']+'_baseline') if doc['backend']['kind']=='rigid' else native_config(doc)
    write_json(a.output/'experiment.json', doc)
    write_json(a.output/'backend_input.json', compiled)
    write_json(a.output/'construction.json', report)
    print(str(a.output/'experiment.json'))


if __name__ == '__main__':
    main()

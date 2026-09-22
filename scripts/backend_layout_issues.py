"""Read-only minimal reproductions and differential diagnostics for material owners."""
from pathlib import Path
import numpy as np
from world_model_dataset.io import read_json,write_json,file_hash


def plastic():
    old=Path('/mnt/y/isaacsim_work/output/world_model_dataset/v0_2/phenomenon_pilot_data_v1/plastic_drop_12_v2/episode')
    new=Path('output/layout_completion_v1/plastic/execution/jobs/constructed_plastic_baseline/attempt_001/constructed_plastic_baseline')
    oc=read_json(old/'resolved_inputs.json')['config'];nc=read_json(new/'prepared/config.json')
    result=dict(successful_episode=str(old),failed_run=str(new.resolve()),comparisons={},initial_particles={},colliders=[])
    for key in ('material','solver','physics_hz','state_hz','gravity_m_s2','initial_velocity_m_s','duration_s'):
        result['comparisons'][key]=dict(equal=oc[key]==nc[key],successful=oc[key],failed=nc[key])
    for label,folder in [('successful',old/'inputs'),('failed',new/'prepared')]:
        with np.load(folder/'initial.npz') as a:
            p=a['points'];h=nc['object']['spacing_m']
            result['initial_particles'][label]=dict(count=len(p),minimum_m=p.min(0).tolist(),extent_m=np.ptp(p,axis=0).tolist(),
                total_mass_kg=float(a['mass'].sum()),lowest_particle_layer_count=int((p[:,2]<p[:,2].min()+.99*h).sum()),
                grid_phase_m=np.mod(p.min(0),nc['solver']['voxel_size']).tolist(),voxel_extent=(np.ptp(p,axis=0)/nc['solver']['voxel_size']).tolist())
    for a,b in [('original_tabletop','tabletop'),('original_desk_frame','desk_frame')]:
        x=old/'inputs'/(a+'.npz');y=new/'prepared'/(b+'.npz')
        with np.load(x) as xo,np.load(y) as yn:
            result['colliders'].append(dict(successful=str(x),failed=str(y.resolve()),arrays_identical=all(np.array_equal(xo[k],yn[k]) for k in ('vertices','triangles')),
                successful_sha256=file_hash(x),failed_sha256=file_hash(y)))
    result['native_script_identical']=file_hash(old/'inputs/native_plastic.py')==file_hash(new/'prepared/native_plastic.py')
    with np.load(old/'native/states.npz') as a:
        t=a['time'];i=int(abs(t-.25).argmin());result['successful_Jp']=dict(at_quarter_second_min=float(a['Jp'][i].min()),
            at_quarter_second_mean=float(a['Jp'][i].mean()),whole_run_min=float(a['Jp'].min()))
    result.update(object_change=dict(successful_kind=oc['object']['kind'],failed_kind=nc['object']['kind'],
        density_equal=oc['object']['density_kg_m3']==nc['object']['density_kg_m3'],spacing_equal=oc['object']['spacing_m']==nc['object']['spacing_m']),
        effective_contact_parameters=dict(margin_m=0.,friction=0.,projection_threshold_m=0.,both_equal=True),
        failure_observed='After step 120 (0.25 s): min Jp 0.2723 then Nonfinite x; exact failed step unavailable',
        diagnosis='Material/time-step/declared solver/native adapter and collider arrays match. Changed shape/aspect, contact multiplicity, mass and sub-voxel placement remain. Flat lowest layer has 192 particles versus 18: concentrated impact/contact-grid interaction is a leading hypothesis, not isolated proof. Neither input is a one-cell-thick object.',
        unresolved='Precise kernel/residual/failed step and native library commit at failed runtime were not saved; cannot claim a single numeric stability threshold from this pair.',
        required_backend='Report failed step, last finite particles/F/Jp/stress, grid/contact residual and nonlinear solve status; establish legal contact/particle-to-grid/time-step envelope for flat-face impact, preserving geometry and material. Backend-controlled discriminating checks should separate contact multiplicity from sub-voxel phase; not a broad parameter scan.',
        reproduction=dict(entry=str((new/'prepared/native_plastic.py').resolve()),prepared=str((new/'prepared').resolve()),
            python='/home/fangsuo/program_files/newton/.venv/bin/python',arguments='--prepared <prepared> --output <new-empty-output>',
            executed_here=False,inputs={str(p.resolve()):file_hash(p) for p in (new/'prepared').iterdir() if p.is_file()}))
    return result


def wrap():
    req=read_json('configs/dataset/v0_2/construction/rope_wrap.json');manifest_path=Path(req['scene']['source_records']['scene']);manifest=read_json(manifest_path)
    rows=[];counts={}
    for spec in req['scene']['collision']['meshes']:
        path=Path(spec['path'])
        with np.load(path) as z:v=z['vertices'];f=z['triangles']
        t=v[f];twice=np.linalg.norm(np.cross(t[:,1]-t[:,0],t[:,2]-t[:,0]),axis=1);bad=np.flatnonzero(twice<1e-12)
        counts[spec['id']]=dict(triangles=len(f),rejected=len(bad),exact_zero=int((twice[bad]==0).sum()),sha256=file_hash(path))
        for i in bad:
            owner=next((dict(object_index=j,name=o['name'],parent=o.get('parent'),local_triangle=int(i-o['triangle_start']))
                        for j,o in enumerate(manifest['groups'][spec['id']]['objects']) if o['triangle_start']<=i<o['triangle_start']+o['triangle_count']),None)
            rows.append(dict(group=spec['id'],triangle_index=int(i),vertex_indices=f[i].tolist(),vertices_m=t[i].tolist(),twice_area_m2=float(twice[i]),owner=owner))
    return dict(source_manifest=dict(path=str(manifest_path),sha256=file_hash(manifest_path)),counts=counts,rejected_triangles=rows,
        exact_predicate='norm(cross(v1-v0,v2-v0)) < 1e-12 in scene_episode.prepare',
        original_mesh_modified=False,physics_started=False,
        minimal_reproduction='Read unchanged complete scene group and evaluate the predicate at listed indices. Existing rope_wrap failed prepare request/run is preserved under output/layout_completion_v1/rope_wrap.',
        required_backend='Explicit policy for exact-zero versus near-zero triangles, source-index mapping and admission diagnostics. If backend ignores zero-area primitives, retain original input/hash and record exactly which indices have no physical contribution; do not silently substitute a repaired/simplified scene. Mainline/material owner must approve semantics before running.',
        scope='A diagnostic list, not an alternative collision mesh')


if __name__=='__main__':
    out=Path('output/layout_closure_v2/backend_issues');out.mkdir(parents=True,exist_ok=False)
    write_json(out/'plastic.json',plastic());write_json(out/'rope_wrap.json',wrap())

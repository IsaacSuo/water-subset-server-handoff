"""Bounded plate layout from a source-pinned, unloaded native beam calibration."""
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from .io import read_json,digest
from .experiment_geometry import require


def adapt(cfg,calcs,profile,geo,episode=None):
    rule=profile['calibration'];plate=cfg['plate'];size=np.array(plate['size_m']);center=np.array(plate['center_m'])
    center[2]+=rule['initial_clearance_m'];plate['center_m']=center.tolist()
    duration=profile['timing']['duration_s'];warm=rule['warmup_s']
    require(duration>warm+3,'calibration_timing','warmup, loading, hold, withdrawal and recovery must fit')
    if episode is None:
        plate['schedule']=[[0,float(center[2])],[duration,float(center[2])]]
        calcs['calibration']=dict(phase='unloaded_probe',steady_state_claim=False,planned_warmup_s=warm)
        geo.check_box(center-size/2,center+size/2,'probe_plate_clearance')
        return cfg,calcs
    episode=Path(episode);resolved=read_json(episode/'resolved_inputs.json');previous=resolved['config']
    require(calcs['requested_deflection_m']>0,'calibration_stroke','positive additional loading stroke required')
    # Same physical beam/attachment/environment and integration, not just same dimensions.
    for key in ('beam','fixture','iterations','contact_offset_m','physics_hz'):
        expected=profile['timing']['physics_hz'] if key=='physics_hz' else cfg[key]
        require(previous[key]==expected,'calibration_mismatch',key+' differs; construct a new unloaded probe')
    env=geo.scene['collision']['meshes']
    source_report=Path(previous['source_records']['construction'])
    source_doc=source_report.parent/'experiment.json'
    binding=read_json(source_report);authored=read_json(source_doc)
    require(digest(authored)==binding['experiment_sha256'],'calibration_source','constructed source changed')
    require(authored['backend']==profile['backend'],'calibration_mismatch','backend selection differs')
    for path in [*Path(profile['backend']['entry']).parent.glob('*.py'),Path(profile['backend']['runtime'])]:
        require(binding['source_pins'].get(str(path))==geo.pin(path),'calibration_mismatch','public backend code/runtime differs: '+str(path))
    require(authored['scene']['collision']['meshes']==env,'calibration_mismatch','original collision environment differs')
    geo.pin(source_report);geo.pin(source_doc)
    for m in env:
        checksum=geo.pin(Path(m['path']))
        require(resolved['provenance'][m['id']]['sha256']==checksum,'calibration_source','original collision source changed')
    raw=episode/resolved['raw_native']['path']
    require(geo.pin(raw)==resolved['raw_native']['sha256'],'calibration_source','native calibration cache hash differs from packaged record')
    geo.pin(episode/'resolved_inputs.json')
    manifest=episode/('episode.json' if (episode/'episode.json').exists() else 'episode.physics.json');geo.pin(manifest)
    with np.load(raw) as a:
        t=a['time'];surface=a['collision_world'];pose=a['plate_pose']
        require(t[-1]>=warm,'calibration_duration','probe does not cover warmup')
        local=np.einsum('fvi,fij->fvj',surface-pose[:,None,:3],Rotation.from_quat(pose[:,3:]).as_matrix())
        delta=abs(local)-np.array(previous['plate']['size_m'])/2
        signed=np.linalg.norm(np.maximum(delta,0),axis=-1)+np.minimum(delta.max(-1),0)
        require(float(signed.min())>rule['minimum_probe_gap_m'],'calibration_loaded','probe plate approached beam; not an unloaded reference')
        schedule=np.array(previous['plate']['schedule'])
        require(np.ptp(schedule[:,1])<1e-10,'calibration_loaded','probe command was not a constant clear hold')
        initial=surface[0];distal=initial[:,0]>initial[:,0].max()-.35*calcs['free_span_m']
        top=initial[:,2]>initial[:,2].max()-.2*cfg['beam']['size_m'][2];ids=np.flatnonzero(distal&top)
        require(len(ids)>=3,'calibration_patch','distal top contact patch unresolved')
        windows=[];chosen=None
        for end in np.arange(warm,float(t[-1])+.001,.5):
            mask=(t>=end-rule['window_s'])&(t<=end)
            require(mask.sum()>=20,'calibration_samples','insufficient native samples')
            patch=surface[mask][:,ids];centroids=patch.mean(1)
            drift=float(np.linalg.norm(np.ptp(centroids,axis=0)))
            windows.append(dict(end_s=float(end),drift_m=drift,accepted=drift<=rule['maximum_patch_drift_m']))
            if windows[-1]['accepted']:
                chosen=(float(end),patch,drift);break
        require(chosen is not None,'calibration_drift',f'no qualifying unloaded window: {windows}; acquire a longer probe, do not relax the threshold')
        warm,patch,drift=chosen;duration=warm+4
        calcs['derived_timing']=dict(profile['timing'],duration_s=duration)
        representative=np.median(patch,axis=0)
        center[:2]=np.median(representative[:,:2],axis=0)
        surface_z=float(np.quantile(patch[:,:,2],.25))
        target_z=surface_z-calcs['requested_deflection_m']+size[2]/2
        plate['center_m']=center.tolist();plate['guide_limits_m']=[target_z-center[2]-.02,.02]
        plate['schedule']=[[0,float(center[2])],[warm,float(center[2])],[warm+1,target_z],
                           [warm+2,target_z],[warm+3,float(center[2])],[duration,float(center[2])]]
        geo.check_box(center-size/2-[0,0,center[2]-target_z+.01],center+size/2,'calibrated_plate_sweep')
        calcs['calibration']=dict(phase='calibrated_load',episode=str(episode.resolve()),
            native_sha256=geo.pins[str(raw.resolve())],window_s=[warm-rule['window_s'],warm],
            candidate_windows=windows,
            patch_node_ids=ids.tolist(),patch_drift_m=drift,unloaded_surface_z_m=surface_z,
            target_plate_z_m=target_z,minimum_probe_gap_m=float(signed.min()),
            matching_physics_sha256=digest({k:previous[k] for k in ('beam','fixture','environment','iterations','physics_hz')}),
            limits='same beam/environment calibration required; finite Z plate, bounded distal patch; observed drift gate is not convergence; actual load/hold/recovery must be measured')
    return cfg,calcs

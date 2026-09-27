"""WSL: bounded pitcher test, reconstruct, render and deliver; fail closed."""
import argparse
import concurrent.futures
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path
from datetime import datetime,timezone
from run_pour_vorticity_compare import powershell
from build_cabinet_liquid_surfaces import windows_path
from run_active_pour_probe import atomic_json
from coupled_scene.gpu_dfsph.server_paths import mapping_entry, resolve_hdri

ROOT=Path(__file__).resolve().parents[2]


def validate_backend_report(report, unaccepted_preview=False, static_diagnostic=False):
    """Keep the legacy PhysX gate; admit the new backend only as a labeled preview."""
    if report.get('status')!='completed':
        raise ValueError('Cannot render an incomplete simulation as a completed clip')
    if report.get('backend') in ('newton_dfsph','gpu_dfsph_newton'):
        if not unaccepted_preview or static_diagnostic:
            raise ValueError('Newton water migration requires explicit unaccepted preview mode')
        if report.get('scope')!='independent_water_subset' or report.get('reference_rigid_step') is not False:
            raise ValueError('Unexpected Newton water backend contract')
        return dict(diagnostic_only=True,physics_gate_passed=False,backend=report['backend'])
    if unaccepted_preview:
        raise ValueError('Backend preview requires a Newton+DFSPH cache')
    if not static_diagnostic and (report.get('authored_vorticity')!=10. or report.get('diagnostic_only')):
        raise ValueError('Existing PhysX runtime-water gate failed')
    return {}


def stationary_diagnostic_frames(report,manifest):
    """Explicit inspection only: 6-10s native cache, never certify as action data."""
    assert report['status']=='completed' and report.get('official_water_probe')
    assert report.get('diagnostic_only') and manifest.get('diagnostic_only')
    assert manifest['complete'] and report['authored_vorticity']==0.
    frames=[f for f in manifest['frames'] if 6.-1e-8<=f['simulated_seconds']<=10.+1e-8]
    assert len(frames)==121
    selected=[];reference=None
    for i,frame in enumerate(frames):
        t=frame['simulated_seconds'];assert abs(t-(6+i/30))<1e-7
        rows=[r for r in report['rows'] if abs(r['simulated_seconds']-t)<1e-7]
        assert len(rows)==1
        row=rows[0]
        assert row['normal_damping']==0. and row['vorticity_confinement']==0.
        pose=(row['native_donor_position_m'],row['native_donor_rotation_xyzw'])
        if reference is None:reference=pose
        assert pose==reference,'Static diagnostic body moved'
        selected.append(dict(frame,recording_seconds=i/30,action_seconds=0.,
            native_position_m=pose[0],native_rotation_xyzw=pose[1]))
    return selected


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assets',type=Path,required=True)
    parser.add_argument('--design',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--desktop',type=Path,required=True)
    parser.add_argument('--simulation',type=Path,help='Render an existing completed native cache without rerunning physics')
    parser.add_argument('--diagnostic-static-last4',action='store_true',help='Inspect official-water stationary diagnostic 6-10s; not certified action footage')
    parser.add_argument('--unaccepted-backend-preview',action='store_true',help='Render an existing Newton+DFSPH water cache for inspection; never certify it as accepted data')
    parser.add_argument('--video-name',default='陶瓷壶倾倒_v1_4秒半.mp4')
    parser.add_argument('--splashsurf',type=Path,default=Path(sys.executable).parent/'pysplashsurf')
    parser.add_argument('--blender',default=shutil.which('blender') or 'blender')
    parser.add_argument('--hdri',type=Path,default=resolve_hdri(ROOT))
    args=parser.parse_args();out=args.output
    if args.diagnostic_static_last4 and args.simulation is None:parser.error('Diagnostic inspection requires existing simulation')
    if args.unaccepted_backend_preview and (args.simulation is None or args.diagnostic_static_last4):parser.error('Backend preview requires an existing cache and cannot combine with the PhysX static diagnostic')
    assert Path(args.video_name).name==args.video_name and args.video_name.endswith('.mp4')
    assert shutil.disk_usage(out.parent).free>30*1024**3
    out.mkdir(parents=True,exist_ok=False);args.desktop.mkdir(parents=True,exist_ok=False)
    status=dict(phase='validating',started_utc=datetime.now(timezone.utc).isoformat(),
                scope='One native active-drive cache and its matching design; no unrelated scene jobs')
    if args.diagnostic_static_last4:status.update(diagnostic_only=True,physics_gate_passed=False,source_interval_seconds=[6,10])
    def save():
        atomic_json(out/'video_status.json',status);atomic_json(args.desktop/'任务状态.json',status)
    def run(command,log,progress=None):
        with log.open('x',encoding='utf-8') as stream:
            p=subprocess.Popen(command,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT)
            while p.poll() is None:
                if progress:progress()
                time.sleep(10)
            if progress:progress()
            if p.returncode:raise subprocess.CalledProcessError(p.returncode,command)
    def idle():
        status['phase']='waiting_for_gpu';save();count=0
        visible=os.environ.get('CUDA_VISIBLE_DEVICES','').strip()
        selected=visible.split(',') if visible else []
        while count<3:
            command=['nvidia-smi']+(['--id='+','.join(selected)] if selected else [])+[
                '--query-gpu=memory.used,utilization.gpu','--format=csv,noheader,nounits']
            rows=subprocess.check_output(command,text=True).strip().splitlines()
            values=[list(map(int,row.split(','))) for row in rows]
            count=count+1 if all(m<1024 and u<20 for m,u in values) else 0
            status['gpu']=values;save()
            if count<3:time.sleep(10)
    save()
    try:
        sim=args.simulation if args.simulation is not None else out/'simulation'
        def progress():
            path=sim/'probe_report.json'
            if path.exists():
                report=json.loads(path.read_text());status['simulation']={k:v for k,v in report.items() if k in
                    ('status','last_action_seconds','last_simulated_seconds','recorded_frames','settled_at_seconds','error')};save()
        if args.simulation is None:
            # Windows path conversion is only relevant when this legacy WSL
            # entry point launches the native PhysX simulator.  Server-side
            # rendering of an existing Newton cache must stay entirely on
            # local POSIX paths and must not construct the unused command.
            native=[r'Y:\isaacsim\python.bat',windows_path(ROOT/'experiments/coupled_scenes/run_active_pour_probe.py'),
                    '--assets',windows_path(args.assets),'--output',windows_path(sim)]
            run(powershell(native+['--dry-run']),out/'dry_run.log')
            idle();status['phase']='simulating';save()
            run(powershell(native),out/'simulation.log',progress)
            simulation_log=out/'simulation.log'
        else:
            progress()
            status.update(source_simulation=str(sim.resolve()),physics_rerun=False);save()
            simulation_log=sim.parent/(sim.name+'.log')
            if not simulation_log.exists():
                console_log=sim.parent/(sim.name+'.console.log')
                simulation_log=console_log if console_log.exists() else sim/'solver.log'
        report=json.loads((sim/'probe_report.json').read_text())
        status.update(validate_backend_report(report,args.unaccepted_backend_preview,args.diagnostic_static_last4));save()
        source_assets=json.loads((args.assets/'assets.json').read_text(encoding='utf-8'))
        design_blend=args.design/(source_assets['case']['id']+'.blend')
        blend_hash=hashlib.sha256(design_blend.read_bytes()).hexdigest()
        assert blend_hash==report['source_blend_sha256']==source_assets['source_blend_sha256'],'Render design differs from simulated geometry'
        assert report['geometry_sha256']==source_assets['geometry_sha256']
        status['path_mappings']=[mapping_entry('source_blend',source_assets['source_blend'],design_blend),
            mapping_entry('hdri','Y:/scenes/HDRI/'+args.hdri.name,args.hdri)];save()
        log=simulation_log.read_text(errors='replace').lower()
        for phrase in ('cuda error','gpu collision stack overflow','failed to cook','falling back to convex','particle buffer overflow'):
            assert phrase not in log,phrase
        manifest=json.loads((sim/'capture/manifest.json').read_text())
        if report.get('warm_start_diagnostic'):
            initial=float(report['initial_seconds'])
            manifest=dict(manifest,frames=[dict(frame,recording_seconds=frame['action_seconds']-initial)
                for frame in manifest['frames']])
        if args.diagnostic_static_last4:
            manifest=dict(manifest,frames=stationary_diagnostic_frames(report,manifest),fps=30,
                moving_object=manifest['source_assets']['moving_object'])
        assert manifest['complete'] and len(manifest['frames'])>=2 and manifest['fps']==30
        video_frames=len(manifest['frames'])-1;video_seconds=video_frames/30
        assert all(abs(f['recording_seconds']-i/30)<1e-7 for i,f in enumerate(manifest['frames']))
        status['phase']='reconstructing';save();surfaces=out/'surfaces';surfaces.mkdir()
        def build(frame):
            dest=surfaces/Path(frame['file']).stem
            with (surfaces/(dest.name+'.log')).open('x') as stream:
                subprocess.run([sys.executable,str(ROOT/'experiments/coupled_scenes/reconstruct_surface_snapshot.py'),
                    str(sim/'capture'/frame['file']),str(dest),'--mesh-smoothing-iters','25',
                    '--spacing',str(report.get('spacing_m',.004)),'--splashsurf',str(args.splashsurf)],cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,check=True)
            return dict(frame,surface=str(Path(dest.name)/'water.obj'))
        frames=[]
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
            for frame in pool.map(build,manifest['frames']):
                frames.append(frame);status['rebuilt_frames']=len(frames);save()
        atomic_json(surfaces/'sequence.json',dict(complete=True,frames=frames,source_blend_sha256=blend_hash,
            diagnostic_only=args.diagnostic_static_last4 or args.unaccepted_backend_preview,
            backend=report.get('backend','physx'),
            moving_object=manifest.get('moving_object','PouringPitcher'),
            source_simulation=str(sim.resolve()),recycling=manifest.get('recycling')))
        idle();status['phase']='rendering';save();renders=out/'renders'
        command=[str(args.blender),'--background','--python-exit-code','1','--python',
            str(ROOT/'experiments/coupled_scenes/render_active_pour.py'),'--','--blend',str(design_blend),
            '--surfaces',str(surfaces),'--output',str(renders),'--hdri',str(args.hdri)]
        def render_progress():status['rendered_frames']=len(list(renders.glob('frame_*.png')));save()
        run(command,out/'render.log',render_progress)
        assert json.loads((renders/'render_manifest.json').read_text())['complete']
        status['phase']='encoding';save();video=out/args.video_name
        run(['ffmpeg','-hide_banner','-loglevel','warning','-n','-framerate','30','-i',str(renders/'frame_%04d.png'),
             '-frames:v',str(video_frames),'-c:v','libx264','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(video)],out/'encode.log')
        info=json.loads(subprocess.check_output(['ffprobe','-v','error','-select_streams','v:0','-show_entries',
            'stream=width,height,nb_frames,r_frame_rate,duration','-of','json',str(video)],text=True))['streams'][0]
        assert int(info['nb_frames'])==video_frames and info['r_frame_rate']=='30/1' and abs(float(info['duration'])-video_seconds)<1e-5
        shutil.copy2(video,args.desktop/video.name)
        status.update(phase='complete',verification=info,video=str(args.desktop/video.name))
    except Exception as exc:
        status.update(phase='failed',error=f'{type(exc).__name__}: {exc}');raise
    finally:
        status['finished_utc']=datetime.now(timezone.utc).isoformat();save()


if __name__=='__main__':main()

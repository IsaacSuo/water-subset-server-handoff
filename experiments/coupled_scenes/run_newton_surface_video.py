"""WSL: render a completed surface-layout bridge cache with its original look."""
import argparse
import concurrent.futures
import hashlib
import json
import subprocess
import sys
from pathlib import Path

from build_cabinet_liquid_surfaces import windows_path

ROOT=Path(__file__).resolve().parents[2]


def local_path(value):
    value=value.replace('\\','/')
    if len(value)>2 and value[1]==':':value='/mnt/'+value[0].lower()+value[2:]
    return Path(value)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--simulation',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--start',type=float,default=0.)
    parser.add_argument('--seconds',type=float,help='Optional honest excerpt; report preserves source times')
    parser.add_argument('--samples',type=int,default=32)
    parser.add_argument('--width',type=int,default=960)
    parser.add_argument('--height',type=int,default=720)
    args=parser.parse_args()
    report=json.loads((args.simulation/'probe_report.json').read_text(encoding='utf-8'))
    manifest=json.loads((args.simulation/'capture/manifest.json').read_text(encoding='utf-8'))
    if report.get('backend') not in ('newton_dfsph','gpu_dfsph_newton') or report.get('scene_family')!='surface_study' or report['status']!='completed' or not manifest['complete']:
        raise ValueError('A completed surface-study Newton cache is required')
    frames=[f for f in manifest['frames'] if f['action_seconds']>=args.start-1e-8 and (args.seconds is None or f['action_seconds']<=args.start+args.seconds+1e-8)]
    if len(frames)<2:raise ValueError('Empty or too short source interval')
    if args.seconds is not None and abs(frames[-1]['action_seconds']-args.start-args.seconds)>1e-7:
        raise ValueError('Requested excerpt exceeds source cache or is not aligned to frames')
    if abs(frames[0]['action_seconds']-args.start)>1e-7:raise ValueError('Start must align to a source frame')
    meta=manifest['source_assets'];blend=local_path(meta['source_blend'])
    if hashlib.sha256(blend.read_bytes()).hexdigest()!=report['source_blend_sha256']:
        raise ValueError('Original appearance asset changed since simulation')
    args.output.mkdir(parents=True,exist_ok=False);surfaces=args.output/'surfaces';surfaces.mkdir()
    status=dict(phase='reconstructing',source_simulation=str(args.simulation.resolve()),
        source_start_seconds=frames[0]['action_seconds'],source_end_seconds=frames[-1]['action_seconds'],
        diagnostic_only=True,production_accepted=False)
    def save():
        (args.output/'video_status.json').write_text(json.dumps(status,indent=2),encoding='utf-8')
    def run(command,log):
        with log.open('w',encoding='utf-8') as stream:
            subprocess.run(command,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,check=True)
    save()
    try:
        def build(frame):
            target=surfaces/Path(frame['file']).stem
            run([sys.executable,str(ROOT/'experiments/coupled_scenes/reconstruct_surface_snapshot.py'),
                str(args.simulation/'capture'/frame['file']),str(target),'--spacing',str(report['spacing_m']),
                '--mesh-smoothing-iters','25'],surfaces/(target.name+'.log'))
            return dict(surface=str(Path(target.name)/'water.obj'),recording_seconds=frame['action_seconds']-args.start,
                source_action_seconds=frame['action_seconds'],target_pose_isaac_m=frame['native_position_m'])
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:render_frames=list(pool.map(build,frames))
        case=dict(meta['case'],motion=report['motion'])
        sequence=dict(complete=True,frames=render_frames,backend=report['backend'],diagnostic_only=True,
            production_accepted=False,source_blend_sha256=report['source_blend_sha256'],
            source_interval_seconds=[frames[0]['action_seconds'],frames[-1]['action_seconds']],
            action=dict(case=case) if case['motion'] else None)
        (surfaces/'sequence.json').write_text(json.dumps(sequence,indent=2),encoding='utf-8')
        status['phase']='rendering';save()
        command=['D:/Program Files (x86)/Blender/blender.exe','--background','--python-exit-code','1',
            '--python',windows_path(ROOT/'experiments/coupled_scenes/render_surface_sequence.py'),'--',
            '--blend',windows_path(blend),'--surfaces',windows_path(surfaces),'--output',windows_path(args.output/'renders'),
            '--samples',str(args.samples),'--width',str(args.width),'--height',str(args.height)]
        # Use one PowerShell quoting layer; arguments are single-quoted literals.
        quoted='& '+' '.join("'"+item.replace("'","''")+"'" for item in command)+'; exit $LASTEXITCODE'
        run(['powershell.exe','-NoProfile','-Command',quoted],args.output/'render.log')
        rendered=json.loads((args.output/'renders/render_manifest.json').read_text(encoding='utf-8'))
        if not rendered['complete'] or len(rendered['frames'])!=len(frames):
            raise RuntimeError('Incomplete rendered frame sequence')
        status['phase']='encoding';save()
        run(['ffmpeg','-y','-loglevel','error','-framerate','30','-i',str(args.output/'renders/frame_%04d.png'),
            '-frames:v',str(len(frames)-1),'-c:v','libx264','-crf','18','-pix_fmt','yuv420p',str(args.output/'preview.mp4')],args.output/'encode.log')
        stream=json.loads(subprocess.check_output(['ffprobe','-v','error','-select_streams','v:0',
            '-show_entries','stream=nb_frames,duration','-of','json',str(args.output/'preview.mp4')],text=True))['streams'][0]
        if int(stream['nb_frames'])!=len(frames)-1 or abs(float(stream['duration'])-(len(frames)-1)/30)>.002:
            raise RuntimeError('Encoded duration or frame count mismatch')
        status['verification']=stream
        status['phase']='completed';save()
    except BaseException as exc:
        status.update(phase='failed',error=f'{type(exc).__name__}: {exc}');save();raise


if __name__=='__main__':main()

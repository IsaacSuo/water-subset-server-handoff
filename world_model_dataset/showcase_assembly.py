"""Assemble existing videos only: no solver, sensor render, or shared-contract edits."""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import html
import json
from pathlib import Path
import shutil
import subprocess

from PIL import Image, ImageDraw, ImageFont

FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
WIDTH, HEIGHT, HEADER = 1600, 1000, 124


def digest(path):
    h = hashlib.sha256()
    with open(path, 'rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def probe(path):
    result = subprocess.run(['ffprobe', '-v', 'error', '-show_streams', '-show_format',
                             '-of', 'json', str(path)], check=True, capture_output=True, text=True)
    data = json.loads(result.stdout)
    video = next(s for s in data['streams'] if s['codec_type'] == 'video')
    return dict(width=video['width'], height=video['height'], duration_s=float(data['format']['duration']),
                fps=video['r_frame_rate'], frames=video.get('nb_frames'))


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def fitted(draw, xy, text, size, color, width=WIDTH-48):
    while size > 14:
        font = ImageFont.truetype(FONT, size)
        if draw.textlength(text, font=font) <= width:
            break
        size -= 1
    draw.text(xy, text, font=font, fill=color)


def header_image(clip, path):
    image = Image.new('RGB', (WIDTH, HEADER), '#edf1f6')
    draw = ImageDraw.Draw(image)
    fitted(draw, (24, 6), clip['target'], 32, '#182c40')
    fitted(draw, (24, 49), '条件｜' + clip['condition'], 24, '#3e5368')
    fitted(draw, (24, 85), '结果｜' + clip['result'], 24, '#245d50')
    image.save(path)


def inspect_sources(clips, output):
    frames = output / 'inspection'
    frames.mkdir(exist_ok=True)
    for clip in clips:
        for i, fraction in enumerate((0.25, 0.65)):
            path = frames / f"{clip['id']}_{i}.png"
            if not path.exists():
                subprocess.run(['ffmpeg', '-v', 'error', '-n', '-ss', str(clip['source_probe']['duration_s']*fraction),
                                '-i', str(output/clip['snapshot']), '-frames:v', '1', '-vf',
                                'scale=640:-2', '-threads', '1', str(path)], check=True)
    for start in range(0, len(clips), 5):
        batch = clips[start:start+5]
        sheet = Image.new('RGB', (1280, len(batch)*450), '#edf1f6')
        draw = ImageDraw.Draw(sheet)
        for row, clip in enumerate(batch):
            fitted(draw, (10, row*450), clip['id']+' '+clip['target'], 24, '#182c40', 1260)
            for i in range(2):
                with Image.open(frames/f"{clip['id']}_{i}.png") as im:
                    im.thumbnail((640, 405))
                    sheet.paste(im, (i*640+(640-im.width)//2, row*450+40))
        sheet.save(frames/f'sheet_{start//5+1:02d}.jpg')


def normalize(clip, output):
    target = output / 'clips' / (clip['id']+'.mp4')
    if not target.exists():
        header = output/'headers'/(clip['id']+'.png')
        header_image(clip, header)
        filtergraph = (f'[0:v]scale={WIDTH}:{HEIGHT-HEADER}:force_original_aspect_ratio=decrease:'
                       f'force_divisible_by=2,pad={WIDTH}:{HEIGHT}:(ow-iw)/2:'
                       f'{HEADER}+({HEIGHT-HEADER}-ih)/2:color=0xedf1f6,setsar=1[base];'
                       '[base][1:v]overlay=0:0,format=yuv420p[out]')
        temporary = target.with_suffix('.part.mp4')
        subprocess.run(['ffmpeg', '-v', 'error', '-n', '-threads', '2', '-i', str(output/clip['snapshot']),
                        '-i', str(header), '-filter_complex_threads', '1', '-filter_complex', filtergraph,
                        '-map', '[out]', '-an', '-c:v', 'libx264', '-threads', '3', '-preset', 'veryfast',
                        '-crf', '20', '-movflags', '+faststart', str(temporary)], check=True)
        temporary.rename(target)
    # Decode verification is cheap relative to rendering and catches incomplete encodes.
    subprocess.run(['ffmpeg', '-v', 'error', '-xerror', '-threads', '2', '-i', str(target),
                    '-f', 'null', '-'], check=True)
    data = probe(target)
    if abs(data['duration_s']-clip['source_probe']['duration_s']) > .08:
        raise ValueError(f"Unexpected temporal edit: {clip['id']}")
    if data['frames'] != clip['source_probe']['frames']:
        raise ValueError(f"Source frames were dropped or duplicated: {clip['id']}")
    print('CLIP_READY', clip['id'], data['duration_s'], flush=True)
    return dict(clip, video='clips/'+target.name, video_sha256=digest(target), video_probe=data)


def page(catalog, clips, output):
    esc = html.escape
    chunks = [f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>{esc(catalog['title'])}</title>
<style>body{{max-width:1400px;margin:32px auto;padding:0 24px;background:#edf1f6;color:#203044;font:18px/1.65 system-ui}}
video{{width:100%;background:#edf1f6}} .grid{{display:grid;grid-template-columns:repeat(auto-fit,minmax(440px,1fr));gap:24px}}
article{{background:white;padding:18px;border-radius:10px}}h2{{margin-top:40px}}p{{margin:8px 0}}small,summary{{color:#57677a}}
a{{color:#215b8f}}button{{cursor:pointer;padding:6px 12px}} @media(max-width:500px){{.grid{{display:block}}article{{margin-bottom:20px}}}}</style>
<h1>{esc(catalog['title'])}</h1><p>{esc(catalog['scope'])}</p>
<p>本轮仅编排已有片段，未新增仿真或重渲染。原片慢放与物理时间标记保留；片段间不共用时间轴。
新注释仅标目标、关键条件和实际结果。原片已有曲线与技术说明保留，不裁掉物理参与体。</p>
<video id="overview" controls preload="metadata" src="overview.mp4"></video>
<p><a href="overview.mp4" download>下载总览视频</a> · <a href="index.json">来源与剪辑清单</a></p>''']
    groups = list(dict.fromkeys(c['group'] for c in clips))
    chunks.append('<nav>'+' · '.join(f'<a href="#g{i}">{esc(g)}</a>' for i,g in enumerate(groups))+'</nav>')
    for i, group in enumerate(groups):
        chunks.append(f'<h2 id="g{i}">{esc(group)}</h2><div class="grid">')
        for c in clips:
            if c['group'] != group:
                continue
            chunks.append(f'''<article><h3>{esc(c['target'])}</h3>
<video controls preload="none" src="{c['video']}"></video>
<p>条件｜{esc(c['condition'])}</p><p>结果｜{esc(c['result'])}</p>
<button onclick="let v=document.getElementById('overview');v.currentTime={c['overview_start_s']:.3f};v.play();v.scrollIntoView({{block:'center'}})">总览中定位 {c['overview_start_s']:.1f}s</button>
<details><summary>实验范围与来源</summary><p>{esc(c['backend'])}</p>
<p>证据：<a href="{c['evidence_snapshot']}">{esc(c['evidence'])}</a></p><p>原片：{esc(c['source_path'])}</p>
<a href="{c['snapshot']}">未加总览标题的原片副本</a></details></article>''')
        chunks.append('</div>')
    chunks.append('<h2>第一版仍未展示的内容</h2><p>独立设计的多体重排／稳定过程尚未纳入；不把碰撞传播重新命名为重排。'
                  '塑性示例没有完全卸载的永久形变测量；绳索绕弯脱离不代表打结或负载牵引。'
                  '布料共享接口补丁未合并，不影响本页播放。这里不是完整发布或同一后端的全物理耦合证明。</p></html>')
    (output/'index.html').write_text('\n'.join(chunks), encoding='utf-8')


def intro_video(catalog, output):
    target = output/'intro.mp4'
    if target.exists():
        return
    image = Image.new('RGB', (WIDTH, HEIGHT), '#edf1f6')
    draw = ImageDraw.Draw(image)
    for y, text, size in (
        (235, catalog['title'], 56),
        (350, '整体运动 · 接触传递 · 材料响应 · 布料与绳索', 34),
        (470, '复用已有片段，保留真实轨迹、原片慢放和原有外观', 30),
        (535, 'PhysX / Newton / 局部材料求解器：独立实验，不是同场耦合', 28),
        (600, '塑性、黏弹性和柔性案例保留实验性范围，不代表实物标定', 28),
    ):
        fitted(draw, (90, y), text, size, '#203044', WIDTH-180)
    image.save(output/'intro.png')
    subprocess.run(['ffmpeg','-v','error','-n','-loop','1','-i',str(output/'intro.png'),
                    '-t','4','-r','30','-an','-c:v','libx264','-threads','3','-preset','veryfast',
                    '-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(target)],check=True)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--catalog', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--inspect-only', action='store_true')
    a = p.parse_args()
    catalog = json.loads(a.catalog.read_text())
    a.output.mkdir(parents=True, exist_ok=True)
    lock = a.output/'catalog.json'
    if lock.exists() and json.loads(lock.read_text()) != catalog:
        raise ValueError('Use a new output directory for a different editorial selection')
    write_json(lock, catalog)
    for folder in ('originals', 'clips', 'headers', 'evidence'):
        (a.output/folder).mkdir(exist_ok=True)
    clips = []
    for raw in catalog['clips']:
        source = Path(catalog['roots'][raw['root']])/raw['path']
        snapshot = a.output/'originals'/(raw['id']+'.mp4')
        before = digest(source)
        if not snapshot.exists():
            shutil.copyfile(source, snapshot)
        if digest(snapshot) != before or digest(source) != before:
            raise ValueError(f'Source changed during delivery: {source}')
        evidence = Path(catalog['roots'][raw['root']])/raw['evidence']
        evidence_snapshot = a.output/'evidence'/(raw['id']+'.md')
        if not evidence_snapshot.exists():
            shutil.copyfile(evidence, evidence_snapshot)
        source_probe = probe(snapshot)
        if source_probe['fps'] != '30/1':
            raise ValueError('This edition preserves 30 fps sources without temporal resampling')
        clips.append(dict(raw, source_path=str(source), source_sha256=before,
                          snapshot=str(snapshot.relative_to(a.output)), source_probe=source_probe,
                          evidence_snapshot=str(evidence_snapshot.relative_to(a.output)),
                          evidence_sha256=digest(evidence_snapshot)))
    write_json(a.output/'source_inventory.json', clips)
    if a.inspect_only:
        inspect_sources(clips, a.output)
        for c in clips:
            print(c['id'], c['source_probe'], flush=True)
        return
    with ThreadPoolExecutor(max_workers=2) as pool:
        clips = list(pool.map(lambda c: normalize(c, a.output), clips))
    intro_video(catalog, a.output)
    elapsed = 4.
    for c in clips:
        c['overview_start_s'] = elapsed
        elapsed += int(c['video_probe']['frames'])/30
        c['overview_end_s'] = elapsed
    concat = a.output/'concat.txt'
    concat.write_text("file 'intro.mp4'\n"+''.join(f"file '{c['video']}'\n" for c in clips), encoding='utf-8')
    chapters = a.output/'chapters.ffmeta'
    chapters.write_text(';FFMETADATA1\n[CHAPTER]\nTIMEBASE=1/1000\nSTART=0\nEND=4000\ntitle=范围说明\n'+''.join(
        f"[CHAPTER]\nTIMEBASE=1/1000\nSTART={round(c['overview_start_s']*1000)}\n"
        f"END={round(c['overview_end_s']*1000)}\ntitle={c['target']}\n" for c in clips), encoding='utf-8')
    overview = a.output/'overview.mp4'
    if not overview.exists():
        subprocess.run(['ffmpeg','-v','error','-n','-f','concat','-safe','0','-i',str(concat),
                        '-i',str(chapters),'-map','0:v','-map_metadata','1','-map_chapters','1',
                        '-c','copy','-movflags','+faststart',str(overview)],check=True)
    subprocess.run(['ffmpeg','-v','error','-xerror','-threads','2','-i',str(overview),'-f','null','-'],check=True)
    final = probe(overview)
    if abs(final['duration_s']-elapsed) > .1:
        raise ValueError('Overview length does not match concatenated clips')
    write_json(a.output/'index.json', dict(title=catalog['title'], scope=catalog['scope'],
        physics_runs=0, scene_renders=0, source_videos=len(clips), source_hashes_rechecked=True,
        timing='Complete source clips at unchanged playback duration; source slow motion retained; no frame interpolation',
        overview=dict(path='overview.mp4', sha256=digest(overview), **final), clips=clips))
    page(catalog, clips, a.output)
    print('SHOWCASE_READY', overview, final, flush=True)


if __name__ == '__main__':
    main()

"""Encode fresh cache renders and publish a grouped local video gallery."""
import argparse
import html
import json
from pathlib import Path
import shutil
import subprocess

from PIL import Image, ImageDraw, ImageFont

from .io import read_json, write_json, file_hash


GROUPS={1:'整体运动及多体相互作用',2:'体积形变及材料响应',3:'薄片与布料',4:'绳索与细长柔性物'}
FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--catalog',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();catalog=read_json(a.catalog);out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
    (out/'clips').mkdir();(out/'posters').mkdir();(out/'evidence').mkdir();records=[];time=0.
    for shot in catalog['shots']:
        folder=Path(shot['render']);replay=read_json(folder/'replay.json')
        if replay['keyframes_only']:raise ValueError('Keyframes are not a dynamic video')
        frames=replay['frames'];width,height=replay['resolution'];header=92;fps=replay['fps'];slow=catalog.get('slowdown',2)
        title=Image.new('RGB',(width,header),'#142b36');draw=ImageDraw.Draw(title)
        draw.text((16,7),shot['title'],font=ImageFont.truetype(FONT,24 if width>=900 else 17),fill='white')
        draw.text((16,40),shot['caption'],font=ImageFont.truetype(FONT,16 if width>=900 else 12),fill='#d1e4eb')
        draw.text((16,65),f"{GROUPS[shot['group']]}  ·  {slow} 倍慢放  ·  原场景／新物理缓存",font=ImageFont.truetype(FONT,15 if width>=900 else 11),fill='#81c9c0')
        header_path=out/'evidence'/(shot['id']+'_header.png');title.save(header_path)
        target=out/'clips'/(shot['id']+'.mp4')
        graph=f'[0:v]pad={width}:{height+header}:0:{header}:color=0x142b36[base];[base][1:v]overlay=0:0,format=yuv420p[v]'
        if shot.get('reuse_video'):
            source=Path(shot['reuse_video'])
            if file_hash(source)!=shot['reuse_video_sha256']:raise ValueError('Reused clip changed')
            shutil.copyfile(source,target)
        else:
            subprocess.run(['ffmpeg','-v','error','-framerate',str(fps/slow),'-i',str(folder/'frame_%04d.png'),
                '-i',str(header_path),'-filter_complex_threads','1','-filter_complex',graph,'-map','[v]',
                '-frames:v',str(round(len(frames)*30*slow/fps)),'-r','30','-c:v','libx264','-threads','3',
                '-preset','fast','-crf','19','-movflags','+faststart',str(target)],check=True)
        subprocess.run(['ffmpeg','-v','error','-xerror','-i',str(target),'-f','null','-'],check=True)
        probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_format','-show_streams','-of','json',str(target)],text=True))
        duration=float(probe['format']['duration']);expected=len(frames)/fps*slow
        if abs(duration-expected)>.05:raise ValueError('Video timing mismatch')
        poster=out/'posters'/(shot['id']+'.jpg');im=Image.open(folder/frames[min(len(frames)-1,len(frames)//3)]['path']);im.convert('RGB').save(poster,quality=90)
        shutil.copyfile(folder/'replay.json',out/'evidence'/(shot['id']+'_replay.json'))
        records.append(dict(shot,video='clips/'+target.name,poster='posters/'+poster.name,
            video_sha256=file_hash(target),duration_s=duration,overview_start_s=time,
            physical_duration_s=frames[-1]['time_s']-frames[0]['time_s'],rendered_frames=len(frames),
            state_sha256=replay['state_sha256'],replay='evidence/'+shot['id']+'_replay.json'))
        time+=duration
    concat=out/'evidence/concat.txt';concat.write_text(''.join("file '../"+r['video']+"'\n" for r in records))
    subprocess.run(['ffmpeg','-v','error','-f','concat','-safe','0','-i',str(concat),'-c','copy','-movflags','+faststart',str(out/'overview.mp4')],check=True)
    subprocess.run(['ffmpeg','-v','error','-xerror','-i',str(out/'overview.mp4'),'-f','null','-'],check=True)
    overview_hash=file_hash(out/'overview.mp4')
    overview_url='overview.mp4?v='+overview_hash[:12]
    esc=html.escape
    chunks=['''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<style>body{margin:0;background:#101e26;color:#e8eff2;font:17px/1.7 system-ui}main{max-width:1280px;margin:auto;padding:36px 24px}h1{font-size:32px;margin:0}h2{margin-top:48px;color:#9bd3cb}p{color:#becdd3}video{width:100%;background:#060d12;border-radius:9px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(min(100%,490px),1fr));gap:24px}article{background:#1b2e38;border:1px solid #2a444e;border-radius:12px;padding:18px}article h3{margin:0 0 14px}article p{font-size:15px;margin:10px 0}a{color:#9bd3cb}nav{display:flex;gap:20px;flex-wrap:wrap;margin:25px 0}button{background:#284a55;color:white;border:0;border-radius:6px;padding:8px 13px;cursor:pointer}details{font-size:14px;color:#a8bfc8}small{color:#a8bfc8}</style><main>''',
        '<title>'+esc(catalog['title'])+'</title><h1>'+esc(catalog['title'])+'</h1><p>'+esc(catalog['scope'])+'</p>',
        '<nav>'+''.join(f'<a href="#g{k}">{v}</a>' for k,v in GROUPS.items())+'</nav>',
        '<video id="overview" controls preload="metadata" src="'+overview_url+'"></video><p><a href="'+overview_url+'">下载总览</a> · <a href="index.json">片段与来源清单</a></p>']
    for group,title in GROUPS.items():
        chunks.append(f'<h2 id="g{group}">{title}</h2><div class="grid">')
        for r in records:
            if r['group']!=group:continue
            chunks.append(f'''<article><h3>{esc(r['title'])}</h3><video controls preload="none" poster="{r['poster']}" src="{r['video']}"></video>
<p>{esc(r['condition'])}</p><p>观察：{esc(r['outcome'])}</p><button onclick="let v=document.getElementById('overview');v.currentTime={r['overview_start_s']};v.play();v.scrollIntoView()">总览中定位</button>
<details><summary>物理与外观来源</summary><p>{esc(r['limits'])}</p><p>物理时长 {r['physical_duration_s']:.2f} 秒；{slow} 倍慢放；{r['rendered_frames']} 个缓存采样画面。</p><a href="{r['replay']}">缓存、原场景与外观对齐记录</a></details></article>''')
        chunks.append('</div>')
    chunks.append('<p>场景：Christophe Seux — Classroom。扫描资产：Poly Haven（CC0）；具体作者与文件哈希保存在资产包和回放记录。不同后端为独立实验；未做额外 split、模型训练或完整材料标定。</p></main></html>')
    (out/'index.html').write_text('\n'.join(chunks),encoding='utf-8')
    write_json(out/'index.json',dict(title=catalog['title'],scope=catalog['scope'],slowdown=slow,
        clips=records,total_duration_s=time,overview_sha256=overview_hash))
    print('DELIVERED',out,'clips',len(records),'seconds',time,flush=True)


if __name__=='__main__':main()

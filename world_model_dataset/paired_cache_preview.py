"""Paired short preview from existing Blender or RGB-D frames, no rendering."""
import argparse
from pathlib import Path
import subprocess

from PIL import Image, ImageDraw, ImageFont

from .io import read_json,write_json,file_hash


def frames(root, camera='front'):
    if (root/'replay.json').exists():
        index=read_json(root/'replay.json')
        return [(row['time_s'],root/row['path']) for row in index['samples']],index['state_sha256']
    index=read_json(root/'observations/index.json')
    return [(row['time_s'],root/'observations'/row['rgb']) for row in index['frames'] if row['camera_id']==camera],index['source_state_sha256']


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--left',type=Path,required=True);p.add_argument('--right',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--title',required=True)
    p.add_argument('--labels',nargs=2,required=True)
    a=p.parse_args();left,lh=frames(a.left);right,rh=frames(a.right)
    if [t for t,_ in left]!=[t for t,_ in right]:raise ValueError('Preview time axes differ')
    a.output.parent.mkdir(parents=True,exist_ok=True)
    font='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
    f24=ImageFont.truetype(font,24);f18=ImageFont.truetype(font,18)
    proc=subprocess.Popen(['ffmpeg','-v','error','-n','-f','rawvideo','-pix_fmt','rgb24','-s','1280x600','-r','30','-i','-',
        '-an','-c:v','libx264','-preset','fast','-crf','19','-pix_fmt','yuv420p','-movflags','+faststart',str(a.output)],stdin=subprocess.PIPE)
    count=0
    for i,((time,lp),(_,rp)) in enumerate(zip(left,right)):
        image=Image.new('RGB',(1280,600),'#edf1f6');draw=ImageDraw.Draw(image)
        draw.text((18,8),a.title,font=f24,fill='#182c40')
        draw.text((1000,14),f't={time:.3f}s | 0.5×',font=f18,fill='#182c40')
        for j,path in enumerate((lp,rp)):
            with Image.open(path) as raw:
                frame=raw.convert('RGB');frame.thumbnail((640,480))
                image.paste(frame,(j*640+(640-frame.width)//2,85+(480-frame.height)//2))
            draw.text((18+j*640,48),a.labels[j],font=f18,fill='#182c40')
        draw.text((18,573),'同一原位场景／每组只改变标注条件／从真实缓存读取结果／未重跑物理',font=f18,fill='#182c40')
        repeats=2+(15 if i in (0,len(left)-1) else 0)
        for _ in range(repeats):proc.stdin.write(image.tobytes())
        count+=repeats
    proc.stdin.close()
    if proc.wait():raise RuntimeError('Video encoding failed')
    write_json(a.output.with_suffix('.json'),dict(video=a.output.name,sha256=file_hash(a.output),
        source_state_sha256=[lh,rh],duration_s=count/30,physics_rerun=False,render_rerun=False,labels=a.labels))
    print('PAIRED_PREVIEW',a.output,flush=True)


if __name__=='__main__':main()

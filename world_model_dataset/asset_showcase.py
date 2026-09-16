"""Four-up, cache-only shape/motion previews; not formal sensor observations."""
import argparse
import json
from pathlib import Path
import subprocess

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial.transform import Rotation
import trimesh

from .causal_runner import ROOT
from .io import read_json, write_json, file_hash

NAMES = {"banana":"香蕉", "elephant":"象形物", "chair":"椅子", "carrot":"胡萝卜"}
TITLES = {"push":"真实形状 · 有限力推动", "falling":"真实形状 · 跌落响应",
          "support_edge":"真实形状 · 越过固定支撑边缘", "collision":"真实形状 · 碰撞中的运动传递"}


def render(records, output, kind):
    output.mkdir(parents=True,exist_ok=False)
    font_path="/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
    fonts={n:ImageFont.truetype(font_path,n) for n in (15,17,21,27)}
    # Fixed orientation and common scale; only center tracks participants.
    az,el=np.deg2rad([-25,35] if kind=='push' else [-65,28])
    camera=np.array([np.cos(el)*np.cos(az),np.cos(el)*np.sin(az),np.sin(el)])
    right=np.array([-np.sin(az),np.cos(az),0]);up=np.cross(camera,right)
    projection=np.stack([right,up,camera],axis=1)
    light=np.array([-.4,-.7,1.]);light/=np.linalg.norm(light)
    runs=[]
    for record in records:
        root=ROOT/record['episode'];resolved=read_json(root/'resolved_inputs.json')
        rows=[json.loads(l) for l in (root/'body_state_trace.jsonl').read_text().splitlines()]
        local={}
        for oid,body in resolved['bodies'].items():
            g=body['geometry']
            if g['shape']=='mesh':
                with np.load(root/g['mesh']['path'],allow_pickle=False) as data:
                    mesh=trimesh.Trimesh(data['vertices'],data['triangles'],process=False)
            elif g['shape']=='sphere':mesh=trimesh.creation.icosphere(subdivisions=2,radius=g['radius_m'])
            elif g['shape']=='box':mesh=trimesh.creation.box(g['size_m'])
            else:raise ValueError(g['shape'])
            local[oid]=(np.asarray(mesh.vertices),np.asarray(mesh.faces),np.asarray(body['appearance']['color']))
        effort_path=root/'actuator_effort_trace.jsonl'
        effort={r['physics_step']:r for r in map(json.loads,effort_path.read_text().splitlines())} if effort_path.exists() else {}
        contacts={}
        for c in map(json.loads,(root/'contacts.jsonl').read_text().splitlines()):
            contacts.setdefault(c['physics_step'],set()).add('/'.join(c['actor_ids']))
        padding=max(float(np.linalg.norm(v,axis=1).max()) for oid,(v,_,_) in local.items() if oid!='floor')+.15
        runs.append(dict(root=root,rows=rows,local=local,effort=effort,contacts=contacts,name=record['asset'],ground_padding=padding))
    if not 1<=len(runs)<=4:raise ValueError('One to four independent episodes per sheet')
    height=900 if len(runs)>2 else 520
    count=len(runs[0]['rows'])
    if any(len(r['rows'])!=count for r in runs):raise ValueError('Preview durations must match')
    hz=read_json(runs[0]['root']/'episode.physics.json')['timing']['physics_hz']
    steps=list(range(0,count,hz//30))
    if kind=='collision':
        focus_end=round(.4*hz)
        steps=list(range(0,focus_end,2))+list(range(focus_end,count,hz//30))
    if steps[-1]!=count-1:steps.append(count-1)
    # Measure simultaneous actor footprints, not the union of their entire travel.
    span=np.array([.6,.35])
    focus_span=span.copy()
    centers=[]
    for run in runs:
        run_centers=[]
        trajectory_bounds=[]
        for step in steps:
            projected=[]
            for oid,(verts,faces,color) in run['local'].items():
                if oid=='floor':continue
                s=run['rows'][step]['body_states'][oid]
                p=(Rotation.from_quat(s['orientation_xyzw']).apply(verts)+s['position_m'])@projection
                projected.append(p)
            points=np.concatenate(projected)
            if kind=='falling':
                subject=run['rows'][0]['body_states']['left']['position_m']
                points=np.vstack([points,np.array([subject[0],subject[1],0])@projection])
            low,high=points[:,:2].min(0),points[:,:2].max(0)
            span=np.maximum(span,high-low)
            if kind=='collision' and step<=round(.4*hz):focus_span=np.maximum(focus_span,high-low)
            run_centers.append((low+high)/2)
            trajectory_bounds.extend([low,high])
        if kind=='falling':
            low,high=np.min(trajectory_bounds,axis=0),np.max(trajectory_bounds,axis=0)
            span=np.maximum(span,high-low)
            run_centers=[(low+high)/2]*len(steps)
        centers.append(run_centers)
    scale=min(560/(span[0]*1.2),245/(span[1]*1.25))
    full_scale=scale
    focus_scale=min(560/(focus_span[0]*1.2),245/(focus_span[1]*1.25))
    movie=output/'overview.mp4'
    process=subprocess.Popen(['ffmpeg','-v','error','-n','-f','rawvideo','-pix_fmt','rgb24','-s',f'1280x{height}','-r','30','-i','-',
        '-an','-c:v','libx264','-preset','fast','-crf','19','-pix_fmt','yuv420p','-movflags','+faststart',str(movie)],stdin=subprocess.PIPE)
    samples=[]
    try:
        for index,step in enumerate(steps):
            canvas=Image.new('RGB',(1280,height),'#edf1f6');draw=ImageDraw.Draw(canvas)
            time=runs[0]['rows'][step]['time_s']
            focus=kind=='collision' and time<.4
            scale=focus_scale if focus else full_scale
            draw.text((25,12),TITLES[kind],font=fonts[27],fill='#182c40')
            draw.text((895,20),f't = {time:.3f} s  |  '+('0.25×撞击近景' if focus else '0.5×慢放'),font=fonts[21],fill='#182c40')
            draw.text((25,52),'相同场景/作用配方，几何与质量不同；不是单变量反事实。',font=fonts[17],fill='#536478')
            for panel,run in enumerate(runs):
                ox=15+(panel%2)*635;oy=88+(panel//2)*392
                draw.rounded_rectangle((ox,oy,ox+620,oy+378),radius=8,fill='white')
                row=run['rows'][step];target='load' if 'load' in row['body_states'] else 'left'
                s=row['body_states'][target]
                draw.text((ox+15,oy+8),f"{NAMES.get(run['name'],run['name'])}  |  m={s['mass_kg']:.3f} kg",font=fonts[21],fill='#182c40')
                centre=centers[panel][index];screen=np.array([ox+310,oy+175])
                tris=[];colors=[];depth=[]
                for oid,(vertices,faces,color) in run['local'].items():
                    state=row['body_states'][oid]
                    world=Rotation.from_quat(state['orientation_xyzw']).apply(vertices)+state['position_m']
                    if oid=='floor':
                        # Clip diagnostic drawing only, never the source collider.
                        ground_row=run['rows'][0] if kind=='falling' else row
                        participants=[v['position_m'] for k,v in ground_row['body_states'].items() if k!='floor']
                        low=np.maximum(world.min(0),np.min(participants,axis=0)-run['ground_padding'])
                        high=np.minimum(world.max(0),np.max(participants,axis=0)+run['ground_padding'])
                        if np.any(low[:2]>=high[:2]):continue
                        world[:,0]=np.clip(world[:,0],low[0],high[0])
                        world[:,1]=np.clip(world[:,1],low[1],high[1])
                        color=np.array([.84,.87,.90])
                    triangles=world[faces]
                    normal=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
                    normal/=np.maximum(np.linalg.norm(normal,axis=1,keepdims=True),1e-12)
                    visible=normal@camera>0
                    triangles=triangles[visible];normal=normal[visible]
                    p=triangles@projection
                    pixels=(p[:,:,:2]-centre)*np.array([scale,-scale])+screen
                    # Entire object included; canvas is a display viewport, not a sensor.
                    shade=.42+.58*np.maximum(normal@light,0)
                    tris.extend(np.rint(pixels).astype(int).reshape(-1,6).tolist())
                    colors.extend(np.clip(shade[:,None]*color*255,0,255).astype(int).tolist())
                    depth.extend((p[:,:,2].mean(1) if oid!='floor' else np.full(len(p),-1e6)).tolist())
                # Isolated panel prevents a ground face spilling into adjacent views.
                layer=Image.new('RGB',(620,270),'white');painter=ImageDraw.Draw(layer)
                shift=np.tile([ox,oy+40],3)
                for k in np.argsort(depth):
                    painter.polygon((np.asarray(tris[k])-shift).tolist(),fill=tuple(colors[k]))
                canvas.paste(layer,(ox,oy+40));draw=ImageDraw.Draw(canvas)
                speed=np.linalg.norm(s['linear_velocity_m_s']);omega=np.linalg.norm(s['angular_velocity_rad_s'])
                displacement=np.linalg.norm(np.array(s['position_m'])-run['rows'][0]['body_states'][target]['position_m'])
                draw.text((ox+15,oy+310),f'速度 {speed:.2f} m/s   角速度 {omega:.1f} rad/s   位移 {displacement:.2f} m',font=fonts[15],fill='#182c40')
                pairs=set().union(*(run['contacts'].get(i,set()) for i in range(max(0,step-hz//30+1),step+1)))
                names={'load':'物体','left':'物体','floor':'地面','platform':'平台','pusher':'推板','projectile':'入射球'}
                contact_text='、'.join('/'.join(names.get(k,k) for k in p.split('/')) for p in sorted(pairs)) or '无报告'
                force=run['effort'].get(step,{}).get('applied_force_n')
                label=f'实际施力 {force:+.1f} N  |  ' if force is not None else '无外部控制  |  '
                draw.text((ox+15,oy+338),label+'接触：'+contact_text,font=fonts[15],fill='#536478')
            camera_note='共同尺度、固定相机包含地面' if kind=='falling' else '共同尺度、相机随参与体居中'
            draw.text((25,height-20),f'实际缓存 / 完整源网格 / {camera_note} / 未改变物理 / 开发预览，非传感器观测',font=fonts[15],fill='#536478')
            if index in (0,len(steps)//2,len(steps)-1):canvas.save(output/f'frame_{index:03d}.png')
            repetitions=(1 if focus else 2)+(15 if index in (0,len(steps)-1) else 0)
            for _ in range(repetitions):process.stdin.write(canvas.tobytes())
            samples.append(dict(physics_step=step,time_s=time,video_frames=repetitions))
            if index%30==0:print('PREVIEW',kind,index+1,'/',len(steps),flush=True)
    finally:
        process.stdin.close();code=process.wait()
    if code:raise RuntimeError('Preview encoder failed')
    write_json(output/'preview.json',dict(kind=kind,sources=[str(r['root']) for r in runs],
        state_sha256=[file_hash(r['root']/'body_state_trace.jsonl') for r in runs],frames=samples,
        physical_rerun=False,formal_observations=False,mesh_decimation=False,
        duration_s=sum(s['video_frames'] for s in samples)/30))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--index',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--kind',choices=TITLES,default='push')
    args=p.parse_args();records=read_json(args.index)['episodes']
    records=[r for r in records if r.get('kind',args.kind)==args.kind]
    render(records,args.output,args.kind)


if __name__=='__main__':main()

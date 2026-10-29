"""Create 120s demo from recorded Gazebo overview/ego cameras and simulation timestamps."""
from pathlib import Path
import json, math
import numpy as np
from PIL import Image,ImageDraw,ImageFont
import imageio.v2 as imageio
ROOT=Path(__file__).resolve().parent; OUT=ROOT/'output'
BG='#0d1c2b'; FG='#edf4ff'; TEAL='#4ed7c0'; GOLD='#eebd68'; MUTED='#a4b6cb'; RED='#ff9a9a'
def font(n):
    for p in ['C:/Windows/Fonts/arial.ttf','/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf']:
        if Path(p).exists(): return ImageFont.truetype(p,n)
    return ImageFont.load_default()
def text(d,x,y,t,n=21,color=FG): d.text((x,y),t,font=font(n),fill=color)
def base(title,subtitle):
    im=Image.new('RGB',(960,640),BG); d=ImageDraw.Draw(im)
    text(d,30,22,'SECOND LOOK / ROS 2 JAZZY + GAZEBO HARMONIC',17,TEAL)
    text(d,30,60,title,29); text(d,30,105,subtitle,18,MUTED)
    text(d,30,612,'Real Gazebo renders + recorded ROS 2 data | Simulation, not a physical robot',16,MUTED)
    return im,d

def card(title,lines):
    im,d=base(title,'Original inspection task / Physical AI 2026 final project')
    for i,line in enumerate(lines): text(d,40,180+i*55,line,24,TEAL if i==0 else FG)
    return im

def render_clip(r,record,t,title,subtitle):
    im,d=base(title,subtitle)
    times=record['time']; desired=times[0]+max(0,t-1)
    index=min(len(times)-1,max(0,int(np.searchsorted(times,desired,side='right')-1)))
    camera=Image.fromarray(record['rgb'][index]); overview=Image.fromarray(record['overview'][index])
    im.paste(overview,(30,160)); im.paste(camera.resize((384,288)),(546,160))
    text(d,30,530,'Gazebo overview: physically driven wheels',18,MUTED)
    text(d,546,465,'Robot RGB camera / no fake overlay',18,MUTED)
    obs=r['observations'][0]
    if len(r['observations'])>1 and desired>=r['observations'][-1]['sim_time']-.15: obs=r['observations'][-1]
    truth='DEFECT' if r['scene']['label'] else 'NORMAL'
    color=TEAL if obs['decision']=='defect' else GOLD if obs['decision']=='unresolved' else FG
    text(d,546,505,f"P(defect) {obs['p_defect']:.3f}",23)
    text(d,546,540,obs['decision'].upper(),23,color)
    text(d,30,565,f"{r['policy']} | {r['distance_m']:.2f} m | {r['inspection_seconds']:.2f} sim s",21)
    text(d,546,578,'Ground truth: '+truth,17,MUTED)
    return im

def main():
    rows=[json.loads(x) for x in (OUT/'trials.jsonl').read_text().splitlines() if x.strip()]; grouped={}
    for r in rows:
        if r['scene']['seed']==2301: grouped.setdefault(r['scene']['id'],{})[r['policy']]=r
    recovery=next(v for v in grouped.values() if v['single']['scene']['blocked'] and v['single']['scene']['label']==1 and v['single']['decision']=='unresolved' and v['selective']['decision']=='defect')
    clear=next(v for v in grouped.values() if not v['single']['scene']['blocked'] and not v['selective']['reobserved'] and v['selective']['decision']=='defect')
    wrong=next(v for v in grouped.values() if v['single']['scene']['blocked'] and v['single']['scene']['label']==1 and v['selective']['decision']=='normal')
    dev=json.loads((OUT/'development_v1/v2_diagnostic.json').read_text()); fail=next(r for r in dev if r['label']==1)
    failure_image=Image.open(OUT/'observations'/f"trial_{fail['scene']}_always_1.png").convert('RGB')
    fimg,fd=base('Development failure: the crack moved in the image','Same saved camera frame / before and after the feature redesign')
    fimg.paste(failure_image.resize((480,360)),(35,155))
    text(fd,560,190,'Old pixel-based MLP',23,RED); text(fd,560,235,f"P(defect) {fail['old_p']:.4f}",24,RED)
    text(fd,560,315,'New pooled features',23,TEAL); text(fd,560,360,f"P(defect) {fail['new_p']:.4f}",24,TEAL)
    text(fd,35,550,'Development example, not held-out evidence. Final evaluation uses fresh seeds.',19,MUTED)
    stats=json.loads((OUT/'summary.json').read_text())['overall']; a=stats['always']; s=stats['selective']; savings=100*(1-s['mean_distance_m']/a['mean_distance_m'])
    plan=[{'duration':8,'kind':'card','title':'When unsure, move and inspect again','lines':['A learned camera model drives the next observation.','A mobile robot moves around a known occluder.','Compare one look / always move / uncertainty-aware.','No external model, API key, or robot teleport during trials.']},
    {'duration':12,'kind':'static','image':fimg},
    {'duration':20,'kind':'clip','row':recovery['single'],'title':'01 / One look: the panel is occluded','subtitle':'The same scene is reset for each policy. Unresolved is not normal.'},
    {'duration':20,'kind':'clip','row':recovery['always'],'title':'02 / Always move: another view reveals the crack','subtitle':'ROS 2 velocity commands drive Gazebo wheel joints; positions are recorded.'},
    {'duration':20,'kind':'clip','row':recovery['selective'],'title':'03 / Uncertain: reobserve autonomously','subtitle':'The model triggers one extra observation. Motion replay is 1x simulation time.'},
    {'duration':16,'kind':'clip','row':clear['selective'],'title':'04 / Confident image: avoid an extra trip','subtitle':'The same fixed classifier can finish without unnecessary travel.'},
    {'duration':12,'kind':'clip','row':wrong['selective'],'title':'05 / Failure: confidence can be wrong','subtitle':'The hidden defect is called normal. The uncertainty-only policy does not move.'},
    {'duration':16,'kind':'static','image':Image.open(OUT/'gazebo_comparison.png').convert('RGB').resize((960,608))},
    {'duration':8,'kind':'card','title':'Measured results and limits','lines':[f"Selective: {s['detected']}/{s['defects']} defects; {savings:.1f}% less travel.",f"False normal: {s['defect_called_normal']}/{s['defects']}; unresolved: {s['unresolved']}/{s['n']}.",'Known map and one crack template. Real-world safety untested.','Full code, model, measured logs, report and tests are included.']}]
    fps=12; writer=imageio.get_writer(str(OUT/'gazebo_demo.mp4'),fps=fps,codec='libx264',quality=7,macro_block_size=16)
    manifest=[]; offset=0
    try:
        for i,seg in enumerate(plan):
            r=seg.get('row'); rec=None
            if r:
                name=f"trial_{r['scene']['id']}_{r['policy']}.npz"
                with np.load(OUT/'recordings'/name,allow_pickle=False) as file: rec={k:file[k] for k in file.files}
                assert len(rec['time'])>0
                assert rec['time'][-1]-rec['time'][0]<=seg['duration']-1+1e-6, 'Clip must not silently accelerate motion'
            manifest.append({'start':offset,'seconds':seg['duration'],'kind':seg['kind'],'scene':r['scene']['id'] if r else None,'policy':r['policy'] if r else None})
            for k in range(seg['duration']*fps):
                if seg['kind']=='clip': im=render_clip(r,rec,k/fps,seg['title'],seg['subtitle'])
                elif seg['kind']=='card': im=card(seg['title'],seg['lines'])
                else:
                    im=Image.new('RGB',(960,640),BG); im.paste(seg['image'],(0,0))
                writer.append_data(np.asarray(im))
                if k==8*fps and r: im.save(OUT/f'video_preview_{i}.png')
            offset+=seg['duration']
    finally: writer.close()
    (OUT/'video_manifest.json').write_text(json.dumps({'seconds':offset,'fps':fps,'motion_speed':'1x simulation time, explanatory holds; Gazebo overview and RGB recordings','segments':manifest},indent=2)); print('Video:',offset,'seconds')
if __name__=='__main__': main()

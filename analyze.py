"""Factor-level analysis and confidence diagnostics from saved data only."""
from pathlib import Path
import json,math
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from verify_ros import camera_features,model_probability
ROOT=Path(__file__).resolve().parent; OUT=ROOT/'output'
def font(n):
    for p in ['C:/Windows/Fonts/arial.ttf','/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf']:
        if Path(p).exists(): return ImageFont.truetype(p,n)
    return ImageFont.load_default()
def main():
    rows=[json.loads(l) for l in (OUT/'trials.jsonl').read_text().splitlines() if l.strip()]
    assert len(rows)==144
    results={}; im=Image.new('RGB',(1080,750),'#102030'); d=ImageDraw.Draw(im)
    def text(x,y,t,n=22,color='#ebf2ff'): d.text((x,y),t,font=font(n),fill=color)
    text(35,25,'Where does reobservation help?',32); text(35,72,'Defect detection per condition / 3 seeds per cell / frozen model',21,'#aac0d0')
    for j,p in enumerate(['single','always','selective']): text(425+j*210,125,p,23)
    i=0
    for blocked in [False,True]:
        for distance in [3.5,4.5]:
            for light in [.45,1.0]:
                y=175+i*61; key=f'blocked={blocked},distance={distance},light={light}'; results[key]={}
                text(35,y+12,f"{'Occluded' if blocked else 'Clear'} / {distance} m / {'dim' if light<1 else 'bright'}",21)
                for j,p in enumerate(['single','always','selective']):
                    rs=[r for r in rows if r['policy']==p and r['scene']['blocked']==blocked and r['scene']['distance']==distance and r['scene']['light']==light and r['scene']['label']==1]
                    n=len(rs); found=sum(r['decision']=='defect' for r in rs); wrong=sum(r['decision']=='normal' for r in rs); rate=found/n
                    results[key][p]={'defects':n,'detected':found,'false_normal':wrong,'unresolved':n-found-wrong}
                    color=(int(170-110*rate),int(65+110*rate),int(65+85*rate)); x=400+j*210
                    d.rounded_rectangle((x,y,x+185,y+49),radius=6,fill=color); text(x+55,y+10,f'{found}/{n}',23)
                i+=1
    text(35,695,'A confident occluded image can still be wrong. No real-world safety guarantee.',21,'#ffaaaa')
    im.save(OUT/'condition_matrix.png'); (OUT/'condition_summary.json').write_text(json.dumps(results,indent=2))
    with np.load(OUT/'camera_model.npz',allow_pickle=False) as f: model={k:f[k] for k in f.files}
    dataset=json.loads((OUT/'dataset.json').read_text()); diag={}
    for blocked in [False,True]:
        rs=[r for r in dataset if r['split']=='test' and r['blocked']==blocked]
        probs=[model_probability(model,Image.open(OUT/'dataset'/r['file'])) for r in rs]
        diag[str(blocked)]={'test_images':len(rs),'binary_correct':sum((p>=.5)==bool(r['label']) for p,r in zip(probs,rs)),'confident_decisions':sum(max(p,1-p)>=.85 for p in probs),'p_defect_range':[min(probs),max(probs)]}
    first_spreads=[]
    for scene in {r['scene']['id'] for r in rows}:
        ps=[r['observations'][0]['p_defect'] for r in rows if r['scene']['id']==scene]; first_spreads.append(max(ps)-min(ps))
    diag['maximum_first_probability_spread_across_policies']=max(first_spreads)
    (OUT/'image_diagnostics.json').write_text(json.dumps(diag,indent=2)); print(json.dumps(diag,indent=2))
if __name__=='__main__': main()

"""Entrypoint: smoke test, actual rendered training, 3-way physical comparison."""
from pathlib import Path
import argparse, json, time, math, hashlib, platform
import numpy as np
from PIL import Image
import rclpy
from runtime import Robot, OUT, features, load_model, logits, probability, decision

def metrics(p,y):
    conf=np.maximum(p,1-p); ok=(p>=.5)==y; ece=0
    for lo in np.arange(.5,1,.05):
        mask=(conf>=lo)&(conf<lo+.05+1e-9)
        if mask.any(): ece+=mask.mean()*abs(conf[mask].mean()-ok[mask].mean())
    return {'accuracy':float(ok.mean()),'brier':float(np.mean((p-y)**2)),'ece':float(ece)}

def smoke(robot):
    s={'label':1,'blocked':True,'distance':4,'light':1.0}
    robot.start(s,'smoke'); robot.snapshot().save(OUT/'smoke_initial.png'); start=list(robot.pose)
    robot.record=True; robot.goto(0,-1.15); robot.goto(2.8,-1.15); robot.face(4,0)
    robot.snapshot().save(OUT/'smoke_reobserved.png')
    result={'start':start,'end':list(robot.pose),'odom_samples':len(robot.trace),'camera_frames':robot.image_seq,'contact_messages':robot.contacts,'sim_seconds':robot.sim_time,'gazebo_world_pose':list(robot.world_pose) if robot.world_pose else None,'overview_available':robot.overview is not None,'truth_frames':getattr(robot,'truth_frames',[]),'ros_distribution':'jazzy','simulator':'Gazebo Harmonic','controller':'ROS 2 Twist -> Gazebo DiffDrive; feedback via bridged odometry'}
    (OUT/'smoke.json').write_text(json.dumps(result,indent=2)); np.savez_compressed(OUT/'smoke_frames.npz',rgb=np.array([f['rgb'] for f in robot.frames]),overview=np.array([f['overview'] for f in robot.frames]),time=np.array([f['time'] for f in robot.frames]),pose=np.array([f['pose'] for f in robot.frames]))
    (OUT/'smoke_trace.json').write_text(json.dumps(robot.trace)); robot.reset_trial(); result['reset_pose']=list(robot.pose); (OUT/'smoke.json').write_text(json.dumps(result,indent=2)); print(json.dumps(result),flush=True)

def collect(robot,n=32):
    (OUT/'dataset').mkdir(exist_ok=True); records=[]
    for label in [0,1]:
        for blocked in [False,True]:
            for light in [.35,1.0]:
                tag=f'data_{label}_{int(blocked)}_{light}'
                robot.start({'label':label,'blocked':blocked,'distance':4,'light':light},tag)
                rng=np.random.default_rng(1701+int(blocked)*10+int(light*10))
                for i in range(n):
                    x=float(rng.uniform(-.2,1.0) if blocked else rng.uniform(-.2,2.5))
                    y=float(rng.uniform(-.12,.12) if blocked else rng.uniform(-1.5,1.5))
                    yaw=math.atan2(-y,4-x)+float(rng.uniform(-.025,.025))
                    robot.teleport_training_only(x,y,yaw); im=robot.snapshot()
                    name=f'{tag}_{i:03}.png'; im.save(OUT/'dataset'/name)
                    split='calibration' if i%4==0 else 'test' if i%4==1 else 'train'
                    records.append({'file':name,'label':label,'blocked':blocked,'light':light,'pose':[x,y,yaw],'split':split,'sha256':hashlib.sha256(im.tobytes()).hexdigest()})
                print('COLLECTED',tag,n,flush=True)
    (OUT/'dataset.json').write_text(json.dumps(records,indent=2))

def train():
    from sklearn.neural_network import MLPClassifier
    records=json.loads((OUT/'dataset.json').read_text()); splits={}
    for split in ['train','calibration','test']:
        rs=[r for r in records if r['split']==split]
        splits[split]=(np.array([features(Image.open(OUT/'dataset'/r['file'])) for r in rs]),np.array([r['label'] for r in rs]))
    x,y=splits['train']; rng=np.random.default_rng(1801)
    # Augment only training images/features; calibration/test remain original frames.
    xa=np.concatenate([x]+[x+rng.normal(0,.015,x.shape) for _ in range(3)]); ya=np.tile(y,4)
    clf=MLPClassifier(hidden_layer_sizes=(48,),alpha=1.0,max_iter=250,random_state=1801,early_stopping=False)
    clf.fit(xa,ya); m={'w0':clf.coefs_[0],'w1':clf.coefs_[1],'b0':clf.intercepts_[0],'b1':clf.intercepts_[1]}
    xv,yv=splits['calibration']; z=logits(m,xv); ts=np.linspace(.5,4,71); t=float(ts[np.argmin([np.mean(np.logaddexp(0,z/t)-yv*z/t) for t in ts])])
    xt,yt=splits['test']; raw=1/(1+np.exp(-np.clip(logits(m,xt),-35,35)))
    error=float(np.max(np.abs(raw-clf.predict_proba(xt)[:,1]))); assert error<1e-10
    np.savez(OUT/'camera_model.npz',**m,temperature=t)
    p=1/(1+np.exp(-np.clip(logits(m,xt)/t,-35,35)))
    result={'original_images':len(records),'train_images':len(y),'augmented_training_features':len(ya),'calibration_images':len(yv),'test_images':len(yt),'temperature':t,'threshold':.85,'iterations':int(clf.n_iter_),'model_export_max_abs_error':error,'uncalibrated':metrics(raw,yt),'calibrated':metrics(p,yt),'source':'actual Gazebo RGB images via ros_gz_bridge; no synthetic overlay on sensor frames'}
    (OUT/'training.json').write_text(json.dumps(result,indent=2)); print(json.dumps(result,indent=2),flush=True)

def scenes(seeds=(2301,2302,2303)):
    for seed in seeds:
        rng=np.random.default_rng(seed)
        for blocked in [False,True]:
            for distance in [3.5,4.5]:
                for light in [.45,1.0]:
                    for label in [0,1]:
                        yield {'id':f'{seed}_{int(blocked)}_{distance}_{light}_{label}','seed':seed,'label':label,'blocked':blocked,'distance':distance,'light':light,'panel_y':float(rng.uniform(-.055,.055))}

def view_plan(s):
    # Geometry-only candidates. No target defect label. Known obstacle x=d/2, y=0.
    options=[]
    for side in [-1,1]:
        y=side*1.15; end=(s['distance']-.95,y)
        path=[(0,y),end]
        length=abs(y)+end[0]
        quality=1/math.hypot(s['distance']-end[0],s.get('panel_y',0)-y)
        options.append({'score':quality-.03*length,'waypoints':path,'end':end})
    return max(options,key=lambda c:c['score'])

def evaluate(robot,limit=0):
    m=load_model(); file=OUT/'trials.jsonl'; existing=[]
    if file.exists(): existing=[json.loads(l) for l in file.read_text().splitlines() if l.strip()]
    done={(r['scene']['id'],r['policy']) for r in existing}; rows=existing
    cases=list(scenes()); cases=cases[:limit] if limit else cases
    for s in cases:
        started=False
        for policy in ['single','always','selective']:
            if (s['id'],policy) in done: continue
            tag=f"trial_{s['id']}_{policy}"; print('START',tag,flush=True)
            if not started:
                robot.start(s,tag); started=True
            else:
                try: robot.reset_trial()
                except RuntimeError: robot.start(s,tag)
            robot.face(s['distance'],s['panel_y']); robot.trace=[]; robot.contacts=0; robot.frames=[]; robot.last_record=-1
            robot.record=True; begin=robot.sim_time; first=robot.snapshot(); p0=probability(m,first); observations=[{'p_defect':p0,'decision':decision(p0),'pose':list(robot.pose),'sim_time':robot.sim_time}]
            (OUT/'observations').mkdir(exist_ok=True); first.save(OUT/'observations'/f'{tag}_0.png')
            reobserve=policy=='always' or (policy=='selective' and decision(p0)=='unresolved')
            if reobserve:
                plan=view_plan(s)
                for x,y in plan['waypoints']: robot.goto(x,y)
                robot.face(s['distance'],s['panel_y']); last=robot.snapshot(); p1=probability(m,last)
                last.save(OUT/'observations'/f'{tag}_1.png'); observations.append({'p_defect':p1,'decision':decision(p1),'pose':list(robot.pose),'sim_time':robot.sim_time})
            robot.command(); robot.spin(.25); end=robot.sim_time; robot.record=False
            trace=robot.trace; distance=sum(math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(trace,trace[1:]))
            odom_distance=distance
            world_points=[p['world_pose'] for p in trace if p.get('world_pose') is not None]
            if len(world_points)<2: raise RuntimeError('Gazebo ground-truth pose stream missing')
            distance=sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(world_points,world_points[1:]))
            # Independent known-map minimum distance, plus real Gazebo body contact sensor.
            clearances=[math.hypot(max(abs(p[0]-s['distance']/2)-.125,0),max(abs(p[1])-.55,0))-.255 for p in world_points] if s['blocked'] else [100.]
            result={'scene':s,'policy':policy,'observations':observations,'decision':observations[-1]['decision'],'reobserved':reobserve,'start_sim_time':begin,'end_sim_time':end,'distance_m':distance,'odometry_distance_m':odom_distance,'inspection_seconds':end-begin,'contact_messages':robot.contacts,'minimum_obstacle_clearance_m':min(clearances),'trace':trace}
            with file.open('a') as f: f.write(json.dumps(result)+'\n')
            rows.append(result)
            # Save actual camera sequences for all first-seed scenes, for transparent demo selection.
            if s['seed']==2301:
                (OUT/'recordings').mkdir(exist_ok=True)
                np.savez_compressed(OUT/'recordings'/f'{tag}.npz',rgb=np.array([f['rgb'] for f in robot.frames]),overview=np.array([f['overview'] for f in robot.frames]),time=np.array([f['time'] for f in robot.frames]),pose=np.array([f['pose'] for f in robot.frames]))
            print('DONE',tag,result['decision'],round(distance,3),round(end-begin,2),flush=True)
    summarize(rows)

def aggregate(rows):
    n=len(rows); pos=sum(r['scene']['label'] for r in rows); neg=n-pos
    tp=sum(r['decision']=='defect' and r['scene']['label'] for r in rows); fn=sum(r['decision']=='normal' and r['scene']['label'] for r in rows)
    unresolved=sum(r['decision']=='unresolved' for r in rows); fp=sum(r['decision']=='defect' and not r['scene']['label'] for r in rows)
    return {'n':n,'defects':pos,'detected':tp,'defect_called_normal':fn,'false_alarm':fp,'unresolved':unresolved,'detection_rate':tp/max(pos,1),'coverage':1-unresolved/n,'mean_distance_m':float(np.mean([r['distance_m'] for r in rows])),'mean_seconds':float(np.mean([r['inspection_seconds'] for r in rows])),'contact_messages':sum(r['contact_messages'] for r in rows),'reobservations':sum(r['reobserved'] for r in rows)}
def summarize(rows):
    result={'overall':{p:aggregate([r for r in rows if r['policy']==p]) for p in ['single','always','selective']},'by_seed':{str(seed):{p:aggregate([r for r in rows if r['policy']==p and r['scene']['seed']==seed]) for p in ['single','always','selective'] if any(r['policy']==p and r['scene']['seed']==seed for r in rows)} for seed in [2301,2302,2303]}}
    (OUT/'summary.json').write_text(json.dumps(result,indent=2)); print(json.dumps(result['overall'],indent=2),flush=True)

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('mode',choices=['smoke','collect','train','evaluate']); ap.add_argument('--samples',type=int,default=32); ap.add_argument('--limit',type=int,default=0); args=ap.parse_args()
    if args.mode=='train': train(); return
    rclpy.init(); robot=Robot()
    try:
        if args.mode=='smoke': smoke(robot)
        elif args.mode=='collect': collect(robot,args.samples)
        elif args.mode=='evaluate': evaluate(robot,args.limit)
    finally:
        robot.stop(); robot.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
if __name__=='__main__': main()

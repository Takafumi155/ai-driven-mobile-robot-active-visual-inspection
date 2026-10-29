"""Actual Gazebo teacher -> rosbag2 -> paired dataset -> BC -> Gazebo evaluation.

State BC uses known-map waypoint errors, NOT an image-to-action policy.
The frozen camera model determines whether reinspection is needed.
All additions write extension/output only; original experiments remain unchanged.
"""
from pathlib import Path
import argparse, hashlib, json, math, os, signal, subprocess, time
import numpy as np
import rclpy
from std_msgs.msg import Float64MultiArray
from rclpy.serialization import deserialize_message
import rosbag2_py
from runtime import Robot, wrap, probability, load_model, decision

HERE=Path(__file__).resolve().parent
OUT=HERE/'output'
OUT.mkdir(exist_ok=True)

def save(name, value):
    (OUT/name).write_text(json.dumps(value,indent=2),encoding='utf-8')

def teacher(dist,err):
    return [min(.65,.9*dist)*max(0,math.cos(err)) if abs(err)<.55 else 0,
            max(-1.3,min(1.3,2*err))]

def state(dist,err):
    return [min(dist,5)/5, math.sin(err), math.cos(err)]

def predict(model, obs):
    h=np.maximum(0,np.asarray(obs)@model['w0']+model['b0'])
    v,w=h@model['w1']+model['b1']
    return float(np.clip(v,0,.65)),float(np.clip(w,-1.3,1.3))

class LearnerRobot(Robot):
    def __init__(self):
        super().__init__()
        self.samples=self.create_publisher(Float64MultiArray,'/imitation/sample',10)
        self.model=None
        self.control='teacher'
        self.steps=0
    def goto(self,x,y,timeout=45):
        start=time.monotonic(); t0=self.sim_time
        while time.monotonic()-start<timeout:
            self.spin(.04)
            px,py,yaw=self.pose
            dist=math.hypot(x-px,y-py)
            if dist<.10:
                self.command(); self.spin(.25); return
            err=wrap(math.atan2(y-py,x-px)-yaw)
            obs=state(dist,err)
            action=teacher(dist,err) if self.control=='teacher' else predict(self.model,obs)
            msg=Float64MultiArray()
            # Atomic observation/action pairing; clock and raw streams also recorded.
            msg.data=[self.sim_time,*obs,*action]
            self.samples.publish(msg)
            self.command(*action); self.steps+=1
            if self.sim_time-t0>65: break
        self.command(); self.spin(.25)
        raise RuntimeError(f'{self.control} waypoint timeout at {self.pose}')

def scene(seed):
    rng=np.random.default_rng(seed)
    return {'seed':seed,'label':int(seed%2),'blocked':True,
            'distance':float(rng.uniform(3.5,4.5)), 'light':.45 if seed%2 else 1.,
            'panel_y':float(rng.uniform(-.055,.055))}

def path(s):
    # Fixed known-map planner shared by teacher and BC; no defect label input.
    side=-1 if s['seed']%2 else 1
    return [(0,side*1.15),(s['distance']-.95,side*1.15)]

def recorder(name):
    bag=OUT/'bags'/name
    if bag.exists(): raise FileExistsError(bag)
    bag.parent.mkdir(exist_ok=True)
    log=open(OUT/f'{name}_record.log','w')
    p=subprocess.Popen(['ros2','bag','record','-s','sqlite3','-o',str(bag),
        '/imitation/sample','/cmd_vel','/odom','/clock','/camera/image','/robot/world_pose','/robot/contacts'],
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    return p,log,bag

def stop_recorder(p,log):
    if p.poll() is None:
        os.killpg(p.pid,signal.SIGINT)
        try: p.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid,signal.SIGTERM); p.wait(timeout=5)
    log.close()
    if p.returncode not in (0,-signal.SIGINT): raise RuntimeError('rosbag recorder failed')

def collect(robot,count):
    for k in range(count):
        seed=3100+k; name=f'teacher_{seed}'
        if (OUT/'bags'/name/'metadata.yaml').exists(): continue
        s=scene(seed); robot.start(s,name)
        p,log,bag=recorder(name)
        try:
            robot.spin(2)
            if p.poll() is not None: raise RuntimeError('recorder exited')
            robot.control='teacher'
            for x,y in path(s): robot.goto(x,y)
            robot.face(s['distance'],s['panel_y']); robot.spin(.5)
        finally:
            stop_recorder(p,log); robot.stop()
        save(f'{name}.json',{'scene':s,'path':path(s),'bag':str(bag),'split':'validation' if k%4==3 else 'train'})
        print('COLLECTED',name,flush=True)

def dataset():
    xs=[]; ys=[]; episodes=[]; times=[]; splits=[]; counts={}
    for i,meta in enumerate(sorted(OUT.glob('teacher_*.json'))):
        info=json.loads(meta.read_text()); bag=OUT/'bags'/meta.stem
        reader=rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
        n=0
        while reader.has_next():
            topic,raw,t=reader.read_next()
            if topic!='/imitation/sample': continue
            a=deserialize_message(raw,Float64MultiArray).data
            if len(a)!=6 or not np.isfinite(a).all(): raise ValueError('malformed sample')
            times.append(a[0]); xs.append(a[1:4]); ys.append(a[4:6]); episodes.append(i)
            splits.append(info['split']=='validation'); n+=1
        if n<20: raise ValueError(f'empty/short recording: {bag}')
        counts[meta.stem]=n
    if not xs or not any(splits) or all(splits): raise ValueError('need train and validation episodes')
    np.savez_compressed(OUT/'demonstrations.npz',state=np.array(xs),action=np.array(ys),
        episode=np.array(episodes),timestamp=np.array(times),validation=np.array(splits))
    save('dataset.json',{'source':'rosbag2 sqlite3 /imitation/sample; atomic state/action pairs',
        'features':['distance/5','sin(heading_error)','cos(heading_error)'],
        'actions':['linear.x m/s','angular.z rad/s'],'episodes':counts,
        'split':'whole episode, every fourth demonstration held out; no evaluation episodes used'})
    print('DATASET',counts,flush=True)

def train():
    from sklearn.neural_network import MLPRegressor
    d=np.load(OUT/'demonstrations.npz'); x=d['state']; y=d['action']; val=d['validation']
    model=MLPRegressor(hidden_layer_sizes=(48,),activation='relu',solver='lbfgs',
        alpha=.01,max_iter=1500,random_state=4100)
    model.fit(x[~val],y[~val])
    weights={'w0':model.coefs_[0],'w1':model.coefs_[1],'b0':model.intercepts_[0],'b1':model.intercepts_[1]}
    np.savez(OUT/'bc_model.npz',**weights)
    p=np.array([predict(weights,a) for a in x[val]])
    save('training.json',{'algorithm':'supervised behavior cloning, state MLPRegressor 3-48-2',
        'seed':4100,'train_frames':int((~val).sum()),'validation_frames':int(val.sum()),
        'validation_mse_per_action':np.mean((p-y[val])**2,axis=0).tolist(),
        'iterations':int(model.n_iter_),'model_sha256':hashlib.sha256((OUT/'bc_model.npz').read_bytes()).hexdigest(),
        'dataset_sha256':hashlib.sha256((OUT/'demonstrations.npz').read_bytes()).hexdigest(),
        'limitation':'known-map goal-error state BC; not vision-only or LeRobot ACT; face uses conventional controller'})
    print((OUT/'training.json').read_text(),flush=True)

def evaluate(robot,count):
    file=OUT/'trials.jsonl'
    rows=[json.loads(l) for l in file.read_text().splitlines()] if file.exists() else []
    done={(r['seed'],r['controller']) for r in rows}
    with np.load(OUT/'bc_model.npz') as m: robot.model={k:m[k] for k in m.files}
    camera=load_model()
    design={'seeds':list(range(5100,5100+count)),'controllers':['teacher','bc'],
            'criterion':'all waypoints reached within timeout; contacts=0; separate final defect decision',
            'bc_sha256':hashlib.sha256((OUT/'bc_model.npz').read_bytes()).hexdigest(),
            'pipeline_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    if (OUT/'design.json').exists():
        if json.loads((OUT/'design.json').read_text())!=design: raise ValueError('design changed: use new output copy')
    else:
        if rows: raise ValueError('cannot freeze after evaluation')
        save('design.json',design)
    for seed in design['seeds']:
        s=scene(seed)
        for mode in design['controllers']:
            if (seed,mode) in done: continue
            robot.control=mode; robot.steps=0; robot.start(s,f'bc_{seed}_{mode}')
            robot.trace=[]; robot.contacts=0; begin=robot.sim_time
            robot.record=seed in (5100,5101)
            row={'seed':seed,'scene':s,'controller':mode,'reached':False,'error':None}
            try:
                p0=probability(camera,robot.snapshot())
                row['first_probability']=p0
                # All paired trials request reinspection, isolating locomotion quality.
                for x,y in path(s): robot.goto(x,y)
                robot.face(s['distance'],s['panel_y'])
                im=robot.snapshot(); im.save(OUT/f'eval_{seed}_{mode}.png')
                p1=probability(camera,im)
                row.update(reached=True,final_probability=p1,decision=decision(p1))
            except RuntimeError as exc: row['error']=str(exc)
            finally:
                robot.command(); robot.spin(.2)
                pts=[p['world_pose'] for p in robot.trace if p.get('world_pose')]
                row.update(contact_messages=robot.contacts,sim_seconds=robot.sim_time-begin,
                    distance_m=sum(math.dist(a[:2],b[:2]) for a,b in zip(pts,pts[1:])),
                    trace=robot.trace,control_steps=robot.steps)
                if robot.frames:
                    np.savez_compressed(OUT/f'frames_{seed}_{mode}.npz',
                        time=np.array([f['time'] for f in robot.frames]),
                        rgb=np.array([f['rgb'] for f in robot.frames]),
                        overview=np.array([f['overview'] for f in robot.frames]))
                robot.stop()
            with file.open('a') as f: f.write(json.dumps(row)+'\n')
            rows.append(row); print('TRIAL',seed,mode,row['reached'],row.get('decision'),flush=True)
    summary={}
    for mode in design['controllers']:
        rs=[r for r in rows if r['controller']==mode]
        summary[mode]={'trials':len(rs),'reached':sum(r['reached'] for r in rs),
            'reached_without_contact':sum(r['reached'] and r['contact_messages']==0 for r in rs),
            'contact_messages':sum(r['contact_messages'] for r in rs),
            'mean_distance_m':float(np.mean([r['distance_m'] for r in rs])),
            'mean_sim_seconds':float(np.mean([r['sim_seconds'] for r in rs])),
            'correct_final_decisions':sum(r.get('decision')==('defect' if r['scene']['label'] else 'normal') for r in rs)}
    save('summary.json',summary); print(json.dumps(summary,indent=2),flush=True)

def rviz(robot):
    robot.start(scene(6100),'rviz_check')
    log=open(OUT/'rviz.log','w')
    p=subprocess.Popen(['rviz2','-d',str(HERE/'inspection.rviz')],stdout=log,stderr=subprocess.STDOUT)
    try:
        robot.spin(8)
        if p.poll() is not None: raise RuntimeError('RViz exited')
        # Actual X display screenshot, no constructed UI images.
        from PyQt5.QtWidgets import QApplication
        app=QApplication.instance() or QApplication([])
        app.primaryScreen().grabWindow(0).save(str(OUT/'rviz.png'))
        save('rviz.json',{'running_after_seconds':8,'pid':p.pid,'screenshot':'rviz.png',
            'display':os.environ.get('DISPLAY'),'camera_frames':robot.image_seq,
            'odom_samples':len(robot.trace),'limitation':'headless Xvfb session, screenshot reviewed separately'})
    finally:
        p.terminate(); p.wait(timeout=10); log.close(); robot.stop()

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('stage',choices=['collect','dataset','train','evaluate','rviz'])
    ap.add_argument('--count',type=int,default=8); a=ap.parse_args()
    if a.stage=='dataset': dataset(); return
    if a.stage=='train': train(); return
    rclpy.init(); robot=LearnerRobot()
    try:
        if a.stage=='collect': collect(robot,a.count)
        elif a.stage=='evaluate': evaluate(robot,a.count)
        else: rviz(robot)
    finally: robot.stop(); robot.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()

"""ROS 2 node, Gazebo process lifecycle, closed-loop wheel control and real RGB collection."""
from pathlib import Path
import os, signal, subprocess, time, math, json, hashlib
import numpy as np
from PIL import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image as ImageMsg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from ros_gz_interfaces.msg import Contacts
from tf2_msgs.msg import TFMessage
from world import world
ROOT=Path(os.environ.get('SECOND_LOOK_ROOT',Path(__file__).resolve().parent)); OUT=ROOT/'output'; OUT.mkdir(exist_ok=True)

def wrap(a): return (a+math.pi)%(2*math.pi)-math.pi

class Robot(Node):
    def __init__(self):
        super().__init__('second_look_inspection')
        self.overview=None; self.image=None; self.image_seq=0; self.pose=None; self.sim_time=0.; self.world_pose=None; self.contacts=0; self.trace=[]; self.frames=[]
        self.create_subscription(ImageMsg,'/camera/image',self.on_image,qos_profile_sensor_data)
        self.create_subscription(ImageMsg,'/overview/image',self.on_overview,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.on_odom,10)
        self.create_subscription(Clock,'/clock',self.on_clock,10)
        self.create_subscription(Contacts,'/robot/contacts',self.on_contacts,10)
        self.create_subscription(TFMessage,'/robot/world_pose',self.on_truth,10)
        self.pub=self.create_publisher(Twist,'/cmd_vel',10)
        self.processes=[]; self.log_handles=[]; self.record=False; self.last_record=-1
    def on_image(self,msg):
        a=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3).copy()
        if msg.encoding.lower()=='bgr8': a=a[:,:,::-1]
        self.image=a; self.image_seq+=1
        if self.record and self.sim_time-self.last_record>=.20:
            self.frames.append({'time':self.sim_time,'pose':list(self.pose) if self.pose else None,'rgb':a.copy(),'overview':self.overview.copy() if self.overview is not None else np.zeros((360,480,3),dtype=np.uint8)}); self.last_record=self.sim_time
    def on_overview(self,msg):
        a=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3).copy()
        self.overview=a[:,:,::-1] if msg.encoding.lower()=='bgr8' else a
    def on_odom(self,msg):
        p=msg.pose.pose.position; q=msg.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.pose=(p.x,p.y,yaw)
        if not self.trace or self.sim_time-self.trace[-1]['t']>=.08: self.trace.append({'t':self.sim_time,'x':p.x,'y':p.y,'yaw':yaw,'world_pose':list(self.world_pose) if self.world_pose else None})
    def on_truth(self,msg):
        self.truth_frames=[tr.child_frame_id for tr in msg.transforms]
        for tr in msg.transforms:
            if tr.child_frame_id=='robot' or tr.child_frame_id.endswith('/robot'):
                v=tr.transform.translation; self.world_pose=(v.x,v.y,v.z)
    def on_clock(self,msg): self.sim_time=msg.clock.sec+msg.clock.nanosec*1e-9
    def on_contacts(self,msg):
        if len(msg.contacts): self.contacts+=1
    def spin(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end: rclpy.spin_once(self,timeout_sec=.02)
    def command(self,v=0,w=0):
        t=Twist(); t.linear.x=float(v); t.angular.z=float(w); self.pub.publish(t)
    def start(self,scene,tag):
        self.stop(); self.image=None; self.pose=None; self.world_pose=None; self.image_seq=0; self.trace=[]; self.frames=[]; self.contacts=0; self.record=False; self.sim_time=0; self.last_record=-1
        (OUT/'worlds').mkdir(exist_ok=True); file=OUT/'worlds'/f'{tag}.sdf'
        file.write_text(world(label=scene['label'],blocked=scene['blocked'],distance=scene['distance'],light=scene['light'],crack_y=scene.get('panel_y',0)))
        (OUT/'logs').mkdir(exist_ok=True)
        commands=[['gz','sim','-s','-r','--headless-rendering',str(file)],
        ['ros2','run','ros_gz_bridge','parameter_bridge','/camera/image@sensor_msgs/msg/Image[gz.msgs.Image','/overview/image@sensor_msgs/msg/Image[gz.msgs.Image','/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry','/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock','/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist','/robot/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts','/robot/world_pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V']]
        for i,cmd in enumerate(commands):
            h=open(OUT/'logs'/f'{tag}_{i}.log','w'); self.log_handles.append(h)
            self.processes.append(subprocess.Popen(cmd,stdout=h,stderr=subprocess.STDOUT,start_new_session=True,env={**os.environ,'LIBGL_ALWAYS_SOFTWARE':'1','GZ_PARTITION':'secondlook'}))
        start=time.monotonic()
        while self.image is None or self.pose is None or self.sim_time<.5:
            self.spin(.05)
            if time.monotonic()-start>65: raise RuntimeError(f'No RGB/odometry from Gazebo: {tag}; inspect logs')
            if any(p.poll() is not None for p in self.processes): raise RuntimeError(f'Gazebo or bridge exited: {tag}')
        self.command(); self.spin(.5)
    def stop(self):
        if self.processes and rclpy.ok():
            self.command(); self.spin(.1)
        for p in self.processes:
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGTERM)
                try: p.wait(timeout=8)
                except subprocess.TimeoutExpired: os.killpg(p.pid,signal.SIGKILL); p.wait()
        self.processes=[]
        for h in self.log_handles: h.close()
        self.log_handles=[]
        if rclpy.ok(): self.spin(.1)
    def reset_trial(self):
        """Reset physics/world BETWEEN trials only; never teleport during inspection."""
        self.record=False; self.command(); self.spin(.2)
        result=subprocess.run(['gz','service','-s','/world/inspection/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','3000','--req','reset: {all: true}'],capture_output=True,text=True,env={**os.environ,'GZ_PARTITION':'secondlook'})
        if result.returncode!=0 or 'true' not in result.stdout: raise RuntimeError('world reset failed')
        self.pose=None; self.image=None; self.world_pose=None; self.trace=[]; self.frames=[]; self.contacts=0; self.last_record=-1
        end=time.monotonic()+6
        while (self.pose is None or self.image is None) and time.monotonic()<end: self.spin(.05)
        self.command(); self.spin(.5)
        if self.pose is None or math.hypot(self.pose[0],self.pose[1])>.08: raise RuntimeError('odometry did not reset to origin')

    def teleport_training_only(self,x,y,yaw):
        self.command()
        req=f'name: "robot", position: {{x: {x}, y: {y}, z: .18}}, orientation: {{z: {math.sin(yaw/2)}, w: {math.cos(yaw/2)}}}'
        result=subprocess.run(['gz','service','-s','/world/inspection/set_pose','--reqtype','gz.msgs.Pose','--reptype','gz.msgs.Boolean','--timeout','3000','--req',req],capture_output=True,text=True,env={**os.environ,'GZ_PARTITION':'secondlook'})
        if result.returncode!=0 or 'true' not in result.stdout: raise RuntimeError('training pose request failed: '+result.stdout+result.stderr)
        seq=self.image_seq; self.spin(.35)
        if self.image_seq==seq: raise RuntimeError('camera stalled')
    def goto(self,x,y,timeout=35):
        start=time.monotonic(); initial=self.sim_time; success=False
        while time.monotonic()-start<timeout:
            self.spin(.025); px,py,yaw=self.pose; dist=math.hypot(x-px,y-py)
            if dist<.10: success=True; break
            err=wrap(math.atan2(y-py,x-px)-yaw)
            v=min(.65,.9*dist)*max(0,math.cos(err)) if abs(err)<.55 else 0
            self.command(v,max(-1.3,min(1.3,2.0*err)))
            if self.sim_time-initial>60: break
        self.command(); self.spin(.25)
        if not success: raise RuntimeError(f'waypoint timeout: {(x,y)} actual {self.pose}')
    def face(self,x,y):
        start=time.monotonic()
        while time.monotonic()-start<15:
            self.spin(.025); px,py,yaw=self.pose; err=wrap(math.atan2(y-py,x-px)-yaw)
            if abs(err)<.025: break
            self.command(0,max(-1.,min(1.,2.2*err)))
        self.command(); self.spin(.5)
    def snapshot(self):
        seq=self.image_seq; start=time.monotonic()
        while self.image_seq<seq+2 and time.monotonic()-start<3: self.spin(.03)
        return Image.fromarray(self.image.copy())


def features(im):
    a=np.asarray(im.crop((60,20,260,220)).resize((64,64)),dtype=np.float64)/255
    gray=a.mean(axis=2)
    hist=np.histogram(gray,bins=32,range=(0,1),density=False)[0]/gray.size
    gx=np.diff(gray,axis=1); gy=np.diff(gray,axis=0)
    magnitude=np.r_[np.abs(gx).ravel(),np.abs(gy).ravel()]
    edges=np.histogram(magnitude,bins=16,range=(0,1),density=False)[0]/magnitude.size
    quantiles=np.quantile(gray,[0,.005,.01,.025,.05,.1,.25,.5,.75,.9,.99,1])
    return np.r_[hist,edges,quantiles,a.mean(axis=(0,1)),a.std(axis=(0,1))]

def load_model():
    with np.load(OUT/'camera_model.npz',allow_pickle=False) as d: return {k:d[k] for k in d.files}
def logits(m,x): return (np.maximum(0,x@m['w0']+m['b0'])@m['w1']+m['b1']).ravel()
def probability(m,im): return float(1/(1+np.exp(-np.clip(logits(m,features(im))/float(m['temperature']),-35,35)))[0])
def decision(p): return 'defect' if p>=.85 else 'normal' if p<=.15 else 'unresolved'

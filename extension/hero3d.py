"""Record higher-resolution actual 3D Gazebo footage of learned locomotion."""
import json,hashlib
import numpy as np
import rclpy
import runtime
from pipeline import LearnerRobot,OUT,scene,path,save
from runtime import load_model,probability,decision

original_world=runtime.world
def camera_world(**kwargs):
    sdf=original_world(**kwargs)
    # Only presentation camera changes: physics, robot and control stay identical.
    sdf=sdf.replace('2 -4.8 3.8 0 .60 1.5707963','1.5 -3.4 2.6 0 .55 1.5707963')
    sdf=sdf.replace('<horizontal_fov>1.25</horizontal_fov>','<horizontal_fov>1.6</horizontal_fov>')
    sdf=sdf.replace('<width>480</width><height>360</height>','<width>960</width><height>720</height>')
    sdf=sdf.replace('<update_rate>5</update_rate>','<update_rate>15</update_rate>')
    return sdf

class Hero(LearnerRobot):
    def on_image(self,msg):
        recording=self.record; self.record=False
        super().on_image(msg); self.record=recording
        if recording and self.overview is not None:
            self.frames.append({'time':self.sim_time,'rgb':self.image.copy(),'overview':self.overview.copy()})

def main():
    if (OUT/'hero3d_v2.npz').exists(): raise FileExistsError('keep recorded footage')
    runtime.world=camera_world
    rclpy.init(); robot=Hero()
    try:
        s=scene(7101); robot.start(s,'hero3d_v2'); robot.control='bc'
        with np.load(OUT/'bc_model.npz') as m: robot.model={k:m[k] for k in m.files}
        model=load_model(); first=probability(model,robot.snapshot())
        robot.frames=[]; robot.trace=[]; robot.record=True
        robot.spin(.5)
        for x,y in path(s): robot.goto(x,y)
        robot.face(s['distance'],s['panel_y']); robot.spin(.8)
        last=robot.snapshot(); final=probability(model,last); robot.record=False
        np.savez_compressed(OUT/'hero3d_v2.npz',time=np.array([f['time'] for f in robot.frames]),
            rgb=np.array([f['rgb'] for f in robot.frames]),overview=np.array([f['overview'] for f in robot.frames]))
        save('hero3d_v2.json',{'scene':s,'first_probability':first,'final_probability':final,
            'final_decision':decision(final),'trigger':'uncertainty OR known occlusion',
            'contact_messages':robot.contacts,'trace':robot.trace,'frames':len(robot.frames),
            'camera':'actual Gazebo 960x720 external camera; no generated robot animation',
            'controller':'learned state BC, map waypoints and final facing conventional',
            'source_sha256':hashlib.sha256(__import__('pathlib').Path(__file__).read_bytes()).hexdigest()})
        print('RECORDED HERO',len(robot.frames),decision(final),flush=True)
    finally: robot.stop(); robot.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()

"""Uncertainty/visibility-triggered camera inspection using learned BC locomotion."""
import argparse,json
import numpy as np
import rclpy
from pipeline import LearnerRobot,OUT,scene,path,save
from runtime import load_model,probability,decision

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--seed',type=int,default=7101)
    a=ap.parse_args(); name=f'deploy_{a.seed}'
    if (OUT/f'{name}.json').exists(): raise FileExistsError('deployment log already exists; choose new seed')
    rclpy.init(); robot=LearnerRobot()
    try:
        s=scene(a.seed); robot.start(s,name); robot.control='bc'
        with np.load(OUT/'bc_model.npz') as m: robot.model={k:m[k] for k in m.files}
        camera=load_model(); begin=robot.sim_time
        p0=probability(camera,robot.snapshot()); initial=decision(p0)
        # Known visibility guard also catches high-confidence errors under full occlusion.
        reobserve=initial=='unresolved' or s['blocked']
        robot.record=True
        if reobserve:
            for x,y in path(s): robot.goto(x,y)
            robot.face(s['distance'],s['panel_y'])
        im=robot.snapshot(); p1=probability(camera,im); im.save(OUT/f'{name}.png')
        save(f'{name}.json',{'seed':a.seed,'scene':s,'initial_probability':p0,
            'initial_decision':initial,'reobserved':reobserve,'trigger':'uncertain OR known occlusion',
            'final_probability':p1,'final_decision':decision(p1),'sim_seconds':robot.sim_time-begin,
            'contact_messages':robot.contacts,'controller':'learned state BC + conventional final facing',
            'trace':robot.trace})
        print((OUT/f'{name}.json').read_text()[:900],flush=True)
    finally: robot.stop(); robot.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()

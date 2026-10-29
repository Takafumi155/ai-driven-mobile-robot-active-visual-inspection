"""Run one autonomous inspection task, not the whole benchmark."""
from pathlib import Path
import argparse,json,time,math
import numpy as np
import rclpy
from runtime import Robot,OUT,load_model,probability,decision
from runner import view_plan

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--policy',choices=['single','always','selective'],default='selective'); ap.add_argument('--normal',action='store_true'); ap.add_argument('--clear',action='store_true'); args=ap.parse_args()
    scene={'label':0 if args.normal else 1,'blocked':not args.clear,'distance':4.5,'light':1.0,'panel_y':0.0}
    rclpy.init(); robot=Robot(); model=load_model()
    try:
        robot.start(scene,'interactive_demo'); robot.record=True; first=robot.snapshot(); p0=probability(model,first)
        print('First observation:',p0,decision(p0),flush=True)
        move=args.policy=='always' or args.policy=='selective' and decision(p0)=='unresolved'
        if move:
            for x,y in view_plan(scene)['waypoints']: robot.goto(x,y)
            robot.face(scene['distance'],scene['panel_y'])
        last=robot.snapshot(); p1=probability(model,last)
        first.save(OUT/'demo_first.png'); last.save(OUT/'demo_last.png')
        result={'policy':args.policy,'scene':scene,'p_first':p0,'p_last':p1,'decision':decision(p1),'reobserved':move,'contact_messages':robot.contacts,'actual_end_position':list(robot.world_pose) if robot.world_pose else None}
        (OUT/'standalone_demo.json').write_text(json.dumps(result,indent=2)); print(json.dumps(result,indent=2),flush=True)
    finally:
        robot.stop(); robot.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
if __name__=='__main__': main()

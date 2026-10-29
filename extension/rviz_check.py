"""RViz live camera and odometry check with bounded process cleanup."""
import os,subprocess
import rclpy
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from pipeline import LearnerRobot,HERE,OUT,scene,save

class VisualRobot(LearnerRobot):
    def __init__(self):
        super().__init__(); self.tf=TransformBroadcaster(self)
    def on_odom(self,msg):
        super().on_odom(msg)
        t=TransformStamped(); t.header=msg.header; t.header.frame_id='odom'; t.child_frame_id='base_link'
        t.transform.translation.x=msg.pose.pose.position.x
        t.transform.translation.y=msg.pose.pose.position.y
        t.transform.translation.z=msg.pose.pose.position.z
        t.transform.rotation=msg.pose.pose.orientation
        self.tf.sendTransform(t)

def main():
    rclpy.init(); robot=VisualRobot(); p=None
    log=open(OUT/'rviz.log','w'); forced=False
    try:
        robot.start(scene(6100),'rviz_check_v2')
        p=subprocess.Popen(['rviz2','-d',str(HERE/'inspection.rviz')],stdout=log,stderr=subprocess.STDOUT)
        robot.spin(8)
        if p.poll() is not None: raise RuntimeError('RViz exited')
        from PyQt5.QtWidgets import QApplication
        app=QApplication.instance() or QApplication([])
        app.primaryScreen().grabWindow(0).save(str(OUT/'rviz.png'))
        result={'running_after_seconds':8,'camera_frames':robot.image_seq,'odom_samples':len(robot.trace),
                'display':os.environ.get('DISPLAY'),'screenshot':'rviz.png','tf':'odom -> base_link live odometry'}
    finally:
        if p is not None and p.poll() is None:
            p.terminate()
            try: p.wait(timeout=5)
            except subprocess.TimeoutExpired: p.kill(); p.wait(timeout=5); forced=True
        log.close(); robot.stop(); robot.destroy_node(); rclpy.shutdown()
    result['shutdown_needed_kill']=forced; save('rviz.json',result); print(result)

if __name__=='__main__': main()

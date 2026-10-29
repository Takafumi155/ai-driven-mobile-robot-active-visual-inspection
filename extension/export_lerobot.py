"""Optional LeRobot adapter. Requires official lerobot package, never uploads.

Resamples each irregular ROS control episode to 10 Hz using past-only hold.
The NPZ used by the lightweight BC remains the original atomic ROS samples.
"""
from pathlib import Path
import numpy as np

def main():
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    root=Path(__file__).resolve().parent/'output'
    d=np.load(root/'demonstrations.npz',allow_pickle=False)
    target=root/'lerobot_dataset'
    if target.exists(): raise FileExistsError(target)
    ds=LeRobotDataset.create(repo_id='local/second-look',root=target,fps=10,
        robot_type='second_look_diffdrive',use_videos=False,features={
        'observation.state':{'dtype':'float32','shape':(3,),'names':['distance/5','sin_heading','cos_heading']},
        'action':{'dtype':'float32','shape':(2,),'names':['linear_x','angular_z']}})
    for ep in np.unique(d['episode']):
        ids=np.flatnonzero(d['episode']==ep); t=d['timestamp'][ids]
        for tick in np.arange(t[0],t[-1]+1e-8,.1):
            i=ids[max(0,np.searchsorted(t,tick,side='right')-1)]
            ds.add_frame({'observation.state':d['state'][i].astype(np.float32),
                          'action':d['action'][i].astype(np.float32),
                          'task':'Move around occluder to inspect panel'})
        ds.save_episode()
    ds.finalize()
    print('EXPORTED',target)

if __name__=='__main__': main()

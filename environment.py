"""Record anonymous runtime versions. No host paths, usernames, tokens or IPs."""
from pathlib import Path
import json, platform, subprocess, importlib.metadata, os
OUT=Path(__file__).resolve().parent/'output'; OUT.mkdir(exist_ok=True)
packages={}
for p in ['numpy','scipy','scikit-learn','Pillow','imageio']:
    try: packages[p]=importlib.metadata.version(p)
    except importlib.metadata.PackageNotFoundError: packages[p]='not found'
result={'python':platform.python_version(),'system':platform.system(),'ros_distribution':os.environ.get('ROS_DISTRO'),'python_packages':packages}
for title,cmd in [('gazebo',['gz','sim','--version']),('ros_packages',['dpkg-query','-W','ros-jazzy-ros-gz-sim','ros-jazzy-ros-gz-bridge','ros-jazzy-gz-sim-vendor'])]:
    p=subprocess.run(cmd,capture_output=True,text=True); result[title]=(p.stdout+p.stderr).strip()
(OUT/'environment.json').write_text(json.dumps(result,indent=2)); print(json.dumps(result,indent=2))

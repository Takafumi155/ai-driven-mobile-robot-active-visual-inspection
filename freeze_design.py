"""Freeze current source/model hashes before a NEW evaluation, not after tuning on it."""
from pathlib import Path
import json,hashlib
ROOT=Path(__file__).resolve().parent; OUT=ROOT/'output'
if (OUT/'trials.jsonl').exists(): raise SystemExit('Refusing to relabel existing trials as a new frozen experiment. Use a fresh output folder.')
names=['runner.py','runtime.py','world.py','output/camera_model.npz']
result={'stage':'frozen v2 before fresh-seed Gazebo evaluation','policies':['single','always','selective'],'seeds':[2301,2302,2303],'scene_count':48,'policy_trials':144,'factors':{'blocked':[False,True],'distance':[3.5,4.5],'light':[.45,1.0],'label':[0,1]},'threshold':.85,'maximum_reobservations':1,'image_features':'66 pooled intensity/gradient statistics','sha256':{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in names},'canonical_text_sha256':{n:hashlib.sha256((ROOT/n).read_bytes().replace(b'\r\n',b'\n')).hexdigest() for n in names if n.endswith('.py')}}
(OUT/'experiment_design.json').write_text(json.dumps(result,indent=2)); print('Frozen current model/source for fresh evaluation.')

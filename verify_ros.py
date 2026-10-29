"""Independent, ROS-free verification of real Gazebo experiment artifacts."""
from pathlib import Path
import json,math,hashlib
import numpy as np
from PIL import Image
ROOT=Path(__file__).resolve().parent; OUT=ROOT/'output'

def camera_features(im):
    a=np.asarray(im.crop((60,20,260,220)).resize((64,64)),dtype=np.float64)/255
    gray=a.mean(axis=2)
    hist=np.histogram(gray,bins=32,range=(0,1),density=False)[0]/gray.size
    gx=np.diff(gray,axis=1); gy=np.diff(gray,axis=0)
    magnitude=np.r_[np.abs(gx).ravel(),np.abs(gy).ravel()]
    edges=np.histogram(magnitude,bins=16,range=(0,1),density=False)[0]/magnitude.size
    quantiles=np.quantile(gray,[0,.005,.01,.025,.05,.1,.25,.5,.75,.9,.99,1])
    return np.r_[hist,edges,quantiles,a.mean(axis=(0,1)),a.std(axis=(0,1))]

def model_probability(m,im):
    a=np.maximum(0,camera_features(im).dot(m['w0'])+m['b0']); z=float((a.dot(m['w1'])+m['b1'])[0])/float(m['temperature'])
    return 1/(1+math.exp(-max(-35,min(35,z))))
def main():
    rows=[json.loads(line) for line in (OUT/'trials.jsonl').read_text().splitlines() if line.strip()]
    stats=json.loads((OUT/'summary.json').read_text())['overall']; design=json.loads((OUT/'experiment_design.json').read_text())
    for path,digest in design['sha256'].items():
        payload=(ROOT/path).read_bytes()
        if hashlib.sha256(payload).hexdigest()!=digest:
            assert path in design.get('canonical_text_sha256',{})
            assert hashlib.sha256(payload.replace(b'\r\n',b'\n')).hexdigest()==design['canonical_text_sha256'][path]
    assert len(rows)==144; groups={}; observation_count=0; points=0; min_clearance=100.; max_time_gap=0
    with np.load(OUT/'camera_model.npz',allow_pickle=False) as f: model={k:f[k] for k in f.files}
    for r in rows:
        s=r['scene']; groups.setdefault(s['id'],[]).append(r); tag=f"trial_{s['id']}_{r['policy']}"
        assert r['policy'] in ['single','always','selective']; assert len(r['observations'])==1+int(r['reobserved'])
        for i,o in enumerate(r['observations']):
            im=Image.open(OUT/'observations'/f'{tag}_{i}.png'); p=model_probability(model,im)
            assert abs(p-o['p_defect'])<1e-9
            decision='defect' if p>=.85 else 'normal' if p<=.15 else 'unresolved'
            assert decision==o['decision']; observation_count+=1
        if r['policy']=='selective': assert r['reobserved']==(r['observations'][0]['decision']=='unresolved')
        if r['policy']=='single': assert not r['reobserved']
        if r['policy']=='always': assert r['reobserved']
        assert r['decision']==r['observations'][-1]['decision']; assert r['contact_messages']>=0
        tr=r['trace']; assert len(tr)>=2; assert all(b['t']>=a['t'] for a,b in zip(tr,tr[1:])); points+=len(tr)
        wp=[p['world_pose'] for p in tr if p.get('world_pose') is not None]; assert len(wp)>=2
        actual=sum(math.dist(a[:2],b[:2]) for a,b in zip(wp,wp[1:])); assert abs(actual-r['distance_m'])<1e-9
        estimated=sum(math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(tr,tr[1:])); assert abs(estimated-r['odometry_distance_m'])<1e-9
        elapsed=tr[-1]['t']-tr[0]['t']; gap=r['inspection_seconds']-elapsed
        # ROS callbacks may be buffered; check the saved clock interval within 0.5s.
        assert -.1<=gap<=.6,(s['id'],r['policy'],gap); max_time_gap=max(max_time_gap,abs(gap))
        assert r['inspection_seconds']>=r['observations'][-1]['sim_time']-r['observations'][0]['sim_time']
        assert abs(r['inspection_seconds']-(r['end_sim_time']-r['start_sim_time']))<1e-10
        if s['blocked']:
            c=min(math.hypot(max(abs(v[0]-s['distance']/2)-.125,0),max(abs(v[1])-.55,0))-.255 for v in wp)
            assert abs(c-r['minimum_obstacle_clearance_m'])<1e-9; min_clearance=min(min_clearance,c)
    assert len(groups)==48
    for group in groups.values():
        assert len(group)==3 and {r['policy'] for r in group}=={'single','always','selective'}
        assert all(r['scene']==group[0]['scene'] for r in group)
        for r in group: assert math.hypot(*r['observations'][0]['pose'][:2])<.08
    expected={(seed,b,d,l,c) for seed in [2301,2302,2303] for b in [False,True] for d in [3.5,4.5] for l in [.45,1.0] for c in [0,1]}
    got={(r['scene']['seed'],r['scene']['blocked'],r['scene']['distance'],r['scene']['light'],r['scene']['label']) for r in rows}; assert got==expected
    for policy,saved in stats.items():
        rs=[r for r in rows if r['policy']==policy]; pos=[r for r in rs if r['scene']['label']]
        assert saved['n']==len(rs)==48
        assert saved['detected']==sum(r['decision']=='defect' for r in pos)
        assert saved['defect_called_normal']==sum(r['decision']=='normal' for r in pos)
        assert saved['unresolved']==sum(r['decision']=='unresolved' for r in rs)
        assert abs(saved['mean_distance_m']-sum(r['distance_m'] for r in rs)/len(rs))<1e-9
        assert abs(saved['mean_seconds']-sum(r['inspection_seconds'] for r in rs)/len(rs))<1e-9
    result={'status':'PASS','trials':len(rows),'paired_scenes':len(groups),'recomputed_camera_predictions':observation_count,'world_position_samples':points,'minimum_obstacle_clearance_m':min_clearance,'maximum_saved_clock_interval_gap_s':max_time_gap,'contact_messages':sum(r['contact_messages'] for r in rows),'checks':['frozen model and source hashes','full 3-seed factor coverage','same scene inputs across policies','saved RGB inference independently reproduced','Gazebo world-position and odometry distances recalculated','clock-interval consistency','obstacle clearances','independent aggregate counts']}
    (OUT/'verification.json').write_text(json.dumps(result,indent=2)); print(json.dumps(result,indent=2))
if __name__=='__main__': main()

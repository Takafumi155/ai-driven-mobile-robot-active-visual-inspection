"""Independent artifact checks; no ROS imports, no execution of controller code."""
from pathlib import Path
import hashlib,json,math,sqlite3
import numpy as np
ROOT=Path(__file__).resolve().parent; OUT=ROOT/'output'

def main():
    info=json.loads((OUT/'dataset.json').read_text())
    data=np.load(OUT/'demonstrations.npz',allow_pickle=False)
    assert len(data['state'])==sum(info['episodes'].values())
    assert data['state'].shape[1]==3 and data['action'].shape[1]==2
    assert np.isfinite(data['state']).all() and np.isfinite(data['action']).all()
    assert np.max(np.abs(data['action'][:,1]))<=1.300001
    assert data['action'][:,0].min()>=0 and data['action'][:,0].max()<=.650001
    bag_counts={}
    for name,n in info['episodes'].items():
        db=next((OUT/'bags'/name).glob('*.db3'))
        with sqlite3.connect(f'file:{db.as_posix()}?mode=ro',uri=True) as c:
            counts=dict(c.execute('SELECT topics.name,COUNT(messages.id) FROM topics JOIN messages ON topics.id=messages.topic_id GROUP BY topics.id'))
        for topic in ['/imitation/sample','/cmd_vel','/odom','/clock','/camera/image','/robot/world_pose']:
            assert counts.get(topic,0)>0,(name,topic)
        assert counts['/imitation/sample']==n
        bag_counts[name]=counts
    for ep in np.unique(data['episode']):
        mask=data['episode']==ep
        assert len(np.unique(data['validation'][mask]))==1,'episode split leakage'
        assert np.all(np.diff(data['timestamp'][mask])>=0),'clock went backwards'
    training=json.loads((OUT/'training.json').read_text())
    assert hashlib.sha256((OUT/'bc_model.npz').read_bytes()).hexdigest()==training['model_sha256']
    assert hashlib.sha256((OUT/'demonstrations.npz').read_bytes()).hexdigest()==training['dataset_sha256']
    design=json.loads((OUT/'design.json').read_text()); rows=[json.loads(l) for l in (OUT/'trials.jsonl').read_text().splitlines()]
    assert len(rows)==len(design['seeds'])*2
    assert len({(r['seed'],r['controller']) for r in rows})==len(rows)
    assert hashlib.sha256((ROOT/'pipeline.py').read_bytes()).hexdigest()==design['pipeline_sha256']
    for seed in design['seeds']:
        pair=[r for r in rows if r['seed']==seed]
        assert len(pair)==2 and pair[0]['scene']==pair[1]['scene']
    summary={}; distance_errors=[]
    for r in rows:
        pts=[p['world_pose'] for p in r['trace'] if p.get('world_pose')]
        d=sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(pts,pts[1:]))
        distance_errors.append(abs(d-r['distance_m']))
        # Robot.stop can append a stationary tail sample to the shared trace list
        # after distance calculation; allow at most 1 micrometre, record exact error.
        assert distance_errors[-1]<1e-6
        assert r['sim_seconds']>0
        assert r['control_steps']>0
        summary.setdefault(r['controller'],[]).append(r)
    reported=json.loads((OUT/'summary.json').read_text())
    for mode,rs in summary.items():
        assert reported[mode]['reached']==sum(r['reached'] for r in rs)
        assert reported[mode]['reached_without_contact']==sum(r['reached'] and r['contact_messages']==0 for r in rs)
        assert abs(reported[mode]['mean_distance_m']-np.mean([r['distance_m'] for r in rs]))<1e-10
    result={'status':'PASS','episodes':len(bag_counts),'frames':len(data['state']),
            'maximum_distance_recompute_error_m':max(distance_errors),
            'distance_tolerance_m':1e-6,
            'paired_trials':len(rows),'bag_topic_counts':bag_counts,
            'checks':['SQLite message counts','episode split','clock monotonicity','finite action bounds',
                      'frozen model/source hashes','paired scene equality','independent physical distance','aggregate counts']}
    (OUT/'verification.json').write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='bag_topic_counts'},indent=2))

if __name__=='__main__': main()

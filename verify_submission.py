"""Read-only checks of the lightweight public snapshot; requires only NumPy."""
from pathlib import Path
import hashlib
import json
import math
import ast
import numpy as np

ROOT = Path(__file__).resolve().parent
def load(path):
    return json.loads((ROOT / path).read_text(encoding='utf-8'))
def digest(path):
    return hashlib.sha256((ROOT / path).read_bytes()).hexdigest()

def main():
    for path in ROOT.rglob('*.py'):
        ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
    design = load('output/experiment_design.json')
    for path, expected in design['sha256'].items():
        actual = digest(path)
        canonical = hashlib.sha256((ROOT/path).read_bytes().replace(b'\r\n', b'\n')).hexdigest()
        assert actual == expected or canonical == design.get('canonical_text_sha256', {}).get(path), path
    training = load('extension/output/training.json')
    assert digest('extension/output/bc_model.npz') == training['model_sha256']
    assert digest('extension/output/demonstrations.npz') == training['dataset_sha256']
    paired = load('extension/output/design.json')
    assert digest('extension/pipeline.py') == paired['pipeline_sha256']
    assert digest('extension/output/bc_model.npz') == paired['bc_sha256']
    for path in ROOT.rglob('*.npz'):
        with np.load(path, allow_pickle=False) as archive:
            for key in archive.files:
                assert np.isfinite(archive[key]).all(), (path.name, key)
    with np.load(ROOT/'extension/output/demonstrations.npz', allow_pickle=False) as data:
        info = load('extension/output/dataset.json')
        assert data['state'].shape == (sum(info['episodes'].values()), 3)
        assert data['action'].shape == (1003, 2)
        assert int(data['validation'].sum()) == training['validation_frames'] == 243
        assert len(data['validation']) - int(data['validation'].sum()) == training['train_frames'] == 760
        assert data['action'][:,0].min() >= 0 and data['action'][:,0].max() <= .650001
        assert np.abs(data['action'][:,1]).max() <= 1.300001
        assert len(np.unique(data['episode'])) == len(info['episodes']) == 8
        for episode in np.unique(data['episode']):
            mask = data['episode'] == episode
            assert len(np.unique(data['validation'][mask])) == 1
            assert np.all(np.diff(data['timestamp'][mask]) >= 0)
    rows = [json.loads(line) for line in (ROOT/'extension/output/trials.jsonl').read_text().splitlines() if line.strip()]
    assert len(rows) == 16
    assert {(r['seed'], r['controller']) for r in rows} == {(s,c) for s in paired['seeds'] for c in paired['controllers']}
    for seed in paired['seeds']:
        pair = [r for r in rows if r['seed'] == seed]
        assert pair[0]['scene'] == pair[1]['scene']
    summary = load('extension/output/summary.json')
    for row in rows:
        points = [p['world_pose'] for p in row['trace'] if p.get('world_pose')]
        distance = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a,b in zip(points,points[1:]))
        assert abs(distance-row['distance_m']) < 1e-6
        assert row['sim_seconds'] > 0 and row['control_steps'] > 0
    for controller, reported in summary.items():
        group = [r for r in rows if r['controller'] == controller]
        assert reported['trials'] == len(group) == 8
        assert reported['reached'] == sum(r['reached'] for r in group)
        assert reported['contact_messages'] == sum(r['contact_messages'] for r in group)
        assert reported['reached_without_contact'] == sum(r['reached'] and r['contact_messages']==0 for r in group)
        assert abs(reported['mean_distance_m']-np.mean([r['distance_m'] for r in group])) < 1e-10
        assert abs(reported['mean_sim_seconds']-np.mean([r['sim_seconds'] for r in group])) < 1e-10
    print('PASS: Python syntax, frozen source/model hashes, finite numeric arrays, 1003 samples, episode split, 16 paired trials, distances and aggregates.')
    print('NOT CHECKED: raw rosbag2, original camera frames, full 144-trial logs, ROS/Gazebo execution (not included in this lightweight snapshot).')

if __name__ == '__main__':
    main()

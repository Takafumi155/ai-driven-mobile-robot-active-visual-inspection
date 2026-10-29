#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cd /work
if [ -f output/trials.jsonl ]; then
  echo "Existing measured trials found. Preserve output and use a fresh source copy for a new experiment."
  exit 1
fi
python3 runner.py smoke
python3 runner.py collect --samples 32
python3 runner.py train
python3 freeze_design.py
python3 runner.py evaluate
python3 verify_ros.py

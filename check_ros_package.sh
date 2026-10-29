#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
colcon build --base-paths . --symlink-install
source install/setup.bash
export SECOND_LOOK_ROOT="$PWD"
ros2 pkg prefix second_look
ros2 run second_look inspection --help

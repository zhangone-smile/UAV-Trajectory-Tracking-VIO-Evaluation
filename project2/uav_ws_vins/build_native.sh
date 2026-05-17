#!/bin/bash
# Compile all native packages, excluding VINS (which runs in Docker)

cd ~/dev_dir/uav_ws_vins
source devel/setup.bash

catkin_make --exclude vins

echo "Build complete! Run ./run.sh to launch everything (VINS in Docker, rest native)."
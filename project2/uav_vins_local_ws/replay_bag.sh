#!/bin/bash
# Replay a rosbag through the VINS stack. Usage:
#     ./replay_bag.sh <bag-path> [rate]
# Defaults to 1.0x. Pass 0.5 for half-speed if the Orin can't keep up.
set -u

bag="${1:?usage: replay_bag.sh <bag-path> [rate]}"
rate="${2:-1.0}"

if [ ! -f "$bag" ]; then
    echo "no such bag: $bag" >&2
    exit 1
fi

cd "$(dirname "$0")"

source /opt/ros/noetic/setup.bash
source /home/group3/uav_vins_local_ws/devel/setup.bash

# Foxglove for visualisation alongside the replay (optional but useful).
roslaunch uav_vins_adapter foxglove_vins.launch & sleep 1;

# Replay launch wires use_sim_time, runs auto-calib from bag camera_info,
# starts vins_node + vins_odom_bridge, then rosbag play --clock.
roslaunch uav_vins_adapter replay_vins_bag.launch bag:="$bag" rate:="$rate"

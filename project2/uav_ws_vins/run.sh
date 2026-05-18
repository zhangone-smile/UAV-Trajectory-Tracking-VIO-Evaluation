#!/bin/bash
sudo chmod 777 /dev/ttyTHS0
cd ~/dev_dir/uav_ws_vins
source devel/setup.bash

roslaunch mavros px4.launch fcu_url:=/dev/ttyTHS0:921600 & sleep 5;
roslaunch realsense2_camera rs_camera.launch enable_infra1:=true enable_infra2:=true enable_color:=false enable_depth:=false infra_width:=640 infra_height:=480 infra_fps:=30 initial_reset:=true & sleep 10;
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0 & sleep 3;
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 & sleep 3;
roslaunch px4ctrl run_ctrl.launch & sleep 2;
roslaunch traj_generator eight.launch & sleep 2;
rosrun foxglove_bridge foxglove_bridge & sleep 2;

# Run VINS-Fusion from the pre-built Docker image (no need to mount source)
sudo docker run \
    -it \
    --rm \
    --net=host \
    ros:vins-fusion \
    /bin/bash -c \
    'source /opt/ros/$ROS_DISTRO/setup.bash; \
    source devel/setup.bash; \
    rosrun vins vins_node /root/catkin_ws/src/VINS-Fusion/config/vins_config.yaml';

wait;

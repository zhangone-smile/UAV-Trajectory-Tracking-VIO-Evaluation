cmake_minimum_required(VERSION 3.0.2)
project(uav_vins_adapter)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/dump_camera_info_for_vins.py
  scripts/vins_odom_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/_wait_for_file.sh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

install(DIRECTORY
  config
  docs
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


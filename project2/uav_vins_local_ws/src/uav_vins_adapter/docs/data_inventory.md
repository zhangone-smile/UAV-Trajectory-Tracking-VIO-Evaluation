# UAV Achievable Data Inventory

This inventory is based on the live `rostopic list` seen on the onboard computer
after `ekf_run.sh` was started.

## Primary VINS Inputs

- `/camera/infra1/image_rect_raw`
- `/camera/infra2/image_rect_raw`
- `/camera/infra1/camera_info`
- `/camera/infra2/camera_info`
- `/mavros/imu/data`

## Comparison and Diagnostics

- `/mavros/local_position/odom`
- `/mavros/local_position/pose`
- `/mavros/local_position/velocity_local`
- `/mavros/local_position/velocity_body`
- `/mavros/local_position/accel`
- `/mavros/estimator_status`
- `/mavros/timesync_status`
- `/mavros/state`

## Optional or Low-Trust Sensors

- `/mavros/px4flow/ground_distance`
- `/mavros/px4flow/raw/optical_flow_rad`
- `/mavros/global_position/global`
- `/mavros/global_position/local`
- `/mavros/global_position/raw/fix`
- `/mavros/global_position/raw/gps_vel`
- `/mavros/global_position/rel_alt`

## Existing Local Outputs

- `/vo/Relative_pose`
- `/vo/Odometry`
- `/vo/Path`
- `/vo/Camera_pose`
- `/vo/PointCloud`
- `/ekf/ekf_odom`
- `/ekf/Path`

## Control Stack Topics

- `/position_cmd`
- `/debugPx4ctrl`
- `/px4ctrl/takeoff_land`
- `/traj_start_trigger`
- `/circle_test/traj_state`


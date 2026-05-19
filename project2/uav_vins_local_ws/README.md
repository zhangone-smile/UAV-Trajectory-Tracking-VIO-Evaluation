# UAV VINS Local Workspace

This workspace is a ROS1 adapter/config layer for running the classic
VINS-Fusion stereo-inertial estimator on the current UAV data streams.

It does not reimplement the full VINS optimization core. The intended layout is:

```text
uav_vins_local_ws/
  src/
    VINS-Fusion/          # add upstream VINS-Fusion here on the onboard computer
    uav_vins_adapter/     # this package
```

## Data Sources

The current UAV runtime exposes the streams needed for stereo-inertial VINS:

- left infrared image: `/camera/infra1/image_rect_raw`
- right infrared image: `/camera/infra2/image_rect_raw`
- left camera info: `/camera/infra1/camera_info`
- right camera info: `/camera/infra2/camera_info`
- flight-controller IMU: `/mavros/imu/data`
- optional PX4 local estimate for comparison: `/mavros/local_position/odom`
- optional optical-flow/range data for diagnostics only: `/mavros/px4flow/ground_distance`, `/mavros/px4flow/raw/optical_flow_rad`

The adapter publishes VINS output as:

- `/vins/odom`
- `/vins/path`
- optionally `/ekf/ekf_odom` for compatibility with the existing `px4ctrl` stack

## Onboard Bring-Up

After copying this folder to the onboard computer:

```bash
cd ~/uav_vins_local_ws/src
git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git
cd ~/uav_vins_local_ws
catkin_make
source devel/setup.bash
```

If `ekf_run.sh` is already starting MAVROS and RealSense, start only VINS:

```bash
roslaunch uav_vins_adapter uav_vins_realsense_mavros.launch run_realsense:=false run_mavros:=false
```

If you want this launch file to also start RealSense:

```bash
roslaunch uav_vins_adapter uav_vins_realsense_mavros.launch run_realsense:=true run_mavros:=false
```

## Calibration Notes

The included VINS camera YAML files are bring-up defaults, not final flight
calibration. Before serious flight testing, generate camera intrinsics from the
actual `/camera/infra*/camera_info` and perform camera-IMU extrinsic calibration.

For first tests, the config uses:

- `estimate_extrinsic: 2`, so VINS can initialize camera-IMU extrinsics online.
- rectified infrared topics, so camera distortion coefficients are set to zero.
- MAVROS IMU as the inertial input.

Use the helper once the camera is running:

```bash
rosrun uav_vins_adapter dump_camera_info_for_vins.py \
  --left /camera/infra1/camera_info \
  --right /camera/infra2/camera_info \
  --output-dir ~/uav_vins_local_ws/src/uav_vins_adapter/config
```

## Static Check

From the workspace root:

```bash
python3 src/uav_vins_adapter/scripts/check_vins_static.py
```


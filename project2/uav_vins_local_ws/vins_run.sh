#!/bin/bash
# Bring up the full VINS + control + trajectory stack.
# Fail-fast: if any hardware (FCU via MAVROS, RealSense) fails to come up
# within its expected window, abort everything. Flying with half a sensor
# stack is more dangerous than not flying at all.

set -e
set -o pipefail

cd "$(dirname "$0")"

# When the script exits for ANY reason (success, ERR, signal), reap all
# backgrounded children so we don't leak orphaned roslaunches that grab the
# camera USB or the FCU serial. jobs -p lists this shell's bg PIDs.
cleanup() {
  local rc=$?
  if [ $rc -ne 0 ]; then
    echo "[vins_run] aborting (last cmd: $BASH_COMMAND, rc=$rc)" >&2
  fi
  local pids
  pids=$(jobs -p 2>/dev/null)
  if [ -n "$pids" ]; then
    kill -INT $pids 2>/dev/null || true
    sleep 2
    kill -KILL $pids 2>/dev/null || true
  fi
  exit $rc
}
trap cleanup EXIT INT TERM

# Verify a topic is being published, and optionally that a field equals a value.
# Usage: check_topic <topic> [timeout_s] [grep_pattern]
check_topic() {
  local topic="$1"
  local t="${2:-10}"
  local pat="${3:-}"
  local out
  out=$(timeout "$t" rostopic echo -n 1 "$topic" 2>/dev/null) || {
    echo "[vins_run] FATAL: no message on $topic within ${t}s" >&2
    return 1
  }
  if [ -n "$pat" ] && ! echo "$out" | grep -qE "$pat"; then
    echo "[vins_run] FATAL: $topic did not match pattern '$pat'" >&2
    echo "[vins_run] saw:" >&2
    echo "$out" | head -8 >&2
    return 1
  fi
  return 0
}

sudo chmod 777 /dev/ttyTHS0
source /opt/ros/noetic/setup.bash
source devel/setup.bash

# --- MAVROS / FCU ---
roslaunch mavros px4.launch fcu_url:=/dev/ttyTHS0:921600 & sleep 5
# mavros must be publishing AND actually connected to the FCU
check_topic /mavros/state 10 '^connected: True'
echo "[vins_run] mavros connected to FCU"

# --- RealSense ---
# Tell image_transport not to load the compressedDepth plugin on these mono8
# IR topics. Otherwise it tries to compress them as depth on every frame and
# floods the log with errors. Must be set BEFORE the realsense node advertises.
rosparam set /camera/infra1/image_rect_raw/disable_pub_plugins "['image_transport/compressedDepth']"
rosparam set /camera/infra2/image_rect_raw/disable_pub_plugins "['image_transport/compressedDepth']"
roslaunch realsense2_camera rs_camera.launch     enable_infra1:=true enable_infra2:=true     enable_color:=false enable_depth:=false     infra_width:=640 infra_height:=480 infra_fps:=30     initial_reset:=true & sleep 10
# realsense must actually produce IR frames on both cameras
check_topic /camera/infra1/image_rect_raw 10
check_topic /camera/infra2/image_rect_raw 10
echo "[vins_run] realsense streaming"

# Disable IR emitter (no || true: realsense is up, this MUST succeed)
timeout 10 rosrun dynamic_reconfigure dynparam set /camera/stereo_module emitter_enabled false
echo "[vins_run] IR emitter disabled"

# --- Boost IMU rate, then start the rest of the stack ---
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0 & sleep 3
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 & sleep 3

roslaunch px4ctrl run_ctrl.launch & sleep 2
roslaunch uav_vins_adapter uav_vins_realsense_mavros.launch     run_realsense:=false run_mavros:=false publish_px4ctrl_compat:=true & sleep 2
roslaunch uav_vins_adapter foxglove_vins.launch & sleep 2
roslaunch traj_generator eight.launch & sleep 2

echo "[vins_run] all components up"
wait

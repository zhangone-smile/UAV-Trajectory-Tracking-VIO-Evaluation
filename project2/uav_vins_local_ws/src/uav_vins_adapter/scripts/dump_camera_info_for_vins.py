#!/usr/bin/env python3
"""Stage VINS config from live ROS CameraInfo (no offline calibration needed).

Writes:
  <output-dir>/realsense_infra1_pinhole.yaml
  <output-dir>/realsense_infra2_pinhole.yaml
  <output-dir>/<basename(main-config)>            # if --main-config is given

Behavior:
- Subscribes to the configured left/right camera_info topics and writes the
  intrinsics it sees. This removes the need for an offline calibration file.
- If a topic does not deliver a message within --timeout seconds and a
  --fallback-dir is given, the matching YAML from that directory is copied as a
  fallback so VINS can still start.
- Exits 0 once all files are in place; non-zero on hard failure.
"""

import argparse
import re
import shutil
import sys
from pathlib import Path

import rospy
from sensor_msgs.msg import CameraInfo


_PATH_KEY_RE = re.compile(r'^\s*(output_path|pose_graph_save_path)\s*:\s*"([^"]+)"', re.MULTILINE)


def mkdirs_referenced_by_main_config(main_yaml_path):
    """OpenCV-flavoured YAML can't go through PyYAML cleanly, so scrape the
    handful of filesystem paths that VINS expects to exist on disk."""
    text = Path(main_yaml_path).read_text()
    for _, value in _PATH_KEY_RE.findall(text):
        target = Path(value)
        target.mkdir(parents=True, exist_ok=True)
        rospy.loginfo("ensured VINS path exists: %s", str(target))


def camera_info_to_vins_yaml(msg, camera_name):
    # sensor_msgs/CameraInfo defines the field as uppercase K (3x3 row-major
    # intrinsic matrix); rospy preserves the original case. The original
    # version of this script used msg.k and silently crashed at runtime.
    fx = msg.K[0]
    fy = msg.K[4]
    cx = msg.K[2]
    cy = msg.K[5]
    # OpenCV FileStorage (used by camodocal::CameraFactory) needs the literal
    # %YAML:1.0 + --- header and three-space indentation under the nested
    # mappings. Without those, parsing the inner fields silently fails and the
    # downstream cv::Mat allocation hits "setSize: s >= 0" in matrix.cpp.
    return f"""%YAML:1.0
---
model_type: PINHOLE
camera_name: {camera_name}
image_width: {msg.width}
image_height: {msg.height}
distortion_parameters:
   k1: 0.0
   k2: 0.0
   p1: 0.0
   p2: 0.0
projection_parameters:
   fx: {fx:.12g}
   fy: {fy:.12g}
   cx: {cx:.12g}
   cy: {cy:.12g}
"""


def stage_one(topic, name, out_path, fallback_path, timeout):
    rospy.loginfo("waiting up to %.1fs for %s", timeout, topic)
    try:
        msg = rospy.wait_for_message(topic, CameraInfo, timeout=timeout)
        out_path.write_text(camera_info_to_vins_yaml(msg, name))
        rospy.loginfo("wrote %s from live %s (fx=%.3f fy=%.3f)",
                      str(out_path), topic, msg.K[0], msg.K[4])
        return True
    except rospy.ROSException:
        if fallback_path and fallback_path.exists():
            shutil.copyfile(str(fallback_path), str(out_path))
            rospy.logwarn("timeout on %s -- copied fallback %s -> %s",
                          topic, str(fallback_path), str(out_path))
            return True
        rospy.logerr("timeout on %s and no fallback at %s",
                     topic, str(fallback_path))
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--left", default="/camera/infra1/camera_info")
    parser.add_argument("--right", default="/camera/infra2/camera_info")
    parser.add_argument("--output-dir", required=True,
                        help="Where the cam yamls (and optionally main config) are staged.")
    parser.add_argument("--main-config", default="",
                        help="Optional VINS main config file to copy into --output-dir "
                             "so cam0_calib/cam1_calib resolve next to it.")
    parser.add_argument("--fallback-dir", default="",
                        help="If a camera_info topic times out, fall back to "
                             "<fallback-dir>/realsense_infra{1,2}_pinhole.yaml.")
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--left-cam-name", default="realsense_infra1")
    parser.add_argument("--right-cam-name", default="realsense_infra2")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("dump_camera_info_for_vins", anonymous=True)

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    fallback_dir = Path(args.fallback_dir) if args.fallback_dir else None
    left_fb = fallback_dir / "realsense_infra1_pinhole.yaml" if fallback_dir else None
    right_fb = fallback_dir / "realsense_infra2_pinhole.yaml" if fallback_dir else None

    ok_left = stage_one(args.left, args.left_cam_name,
                        out_dir / "realsense_infra1_pinhole.yaml",
                        left_fb, args.timeout)
    ok_right = stage_one(args.right, args.right_cam_name,
                         out_dir / "realsense_infra2_pinhole.yaml",
                         right_fb, args.timeout)

    if not (ok_left and ok_right):
        rospy.logerr("auto-staging of camera intrinsics failed")
        sys.exit(1)

    if args.main_config:
        main_src = Path(args.main_config)
        if not main_src.exists():
            rospy.logerr("--main-config %s does not exist", str(main_src))
            sys.exit(2)
        main_dst = out_dir / main_src.name
        shutil.copyfile(str(main_src), str(main_dst))
        rospy.loginfo("staged main config %s -> %s", str(main_src), str(main_dst))
        # VINS opens output_path/vio.csv and the pose-graph dir for writing;
        # if those dirs do not exist the streams silently fail. Create them
        # here so the very first run produces a complete log.
        mkdirs_referenced_by_main_config(main_dst)

    rospy.loginfo("VINS config staging done in %s", str(out_dir))


if __name__ == "__main__":
    main()

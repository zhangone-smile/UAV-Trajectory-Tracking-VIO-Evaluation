#!/usr/bin/env python3
"""Republish VINS odometry into UAV-friendly topics.

Two input streams:
  - input_odom (default /vins_estimator/odometry, ~camera rate) drives
    /vins/odom and /vins/path. Lower rate, visually corrected per frame, used
    for visualization and stored path.
  - compat_input_odom (default /vins_estimator/imu_propagate, ~IMU rate) drives
    the px4ctrl-compat /ekf/ekf_odom. High rate is required because px4ctrl
    warns if odom feed is below 100Hz.

If compat_input_odom is empty, /ekf/ekf_odom (when enabled) is driven from
input_odom instead, preserving the older single-stream behaviour.
"""

import copy

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class VinsOdomBridge:
    def __init__(self):
        self.input_odom = rospy.get_param("~input_odom", "/vins_estimator/odometry")
        self.compat_input_odom = rospy.get_param("~compat_input_odom", "")
        self.output_odom = rospy.get_param("~output_odom", "/vins/odom")
        self.output_path = rospy.get_param("~output_path", "/vins/path")
        self.publish_compat_odom = rospy.get_param("~publish_compat_odom", False)
        self.compat_odom = rospy.get_param("~compat_odom", "/ekf/ekf_odom")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.child_frame_id = rospy.get_param("~child_frame_id", "body")
        self.max_path_length = int(rospy.get_param("~max_path_length", 3000))

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id

        self.odom_pub = rospy.Publisher(self.output_odom, Odometry, queue_size=20)
        self.path_pub = rospy.Publisher(self.output_path, Path, queue_size=5)
        self.compat_pub = None
        if self.publish_compat_odom:
            self.compat_pub = rospy.Publisher(self.compat_odom, Odometry, queue_size=200)

        self.sub = rospy.Subscriber(
            self.input_odom,
            Odometry,
            self._slow_callback,
            queue_size=50,
            tcp_nodelay=True,
        )

        # Dual-input mode: separate high-rate feed solely for the compat topic.
        self.compat_sub = None
        if self.publish_compat_odom and self.compat_input_odom:
            self.compat_sub = rospy.Subscriber(
                self.compat_input_odom,
                Odometry,
                self._fast_callback,
                queue_size=400,
                tcp_nodelay=True,
            )

        rospy.loginfo(
            "vins_odom_bridge: %s -> %s, path=%s, compat=%s%s",
            self.input_odom,
            self.output_odom,
            self.output_path,
            self.compat_odom if self.publish_compat_odom else "disabled",
            (" (high-rate from " + self.compat_input_odom + ")")
            if self.compat_sub is not None else "",
        )

    def _rewrite_frames(self, msg):
        out = copy.deepcopy(msg)
        if self.frame_id:
            out.header.frame_id = self.frame_id
        if self.child_frame_id:
            out.child_frame_id = self.child_frame_id
        return out

    def _slow_callback(self, msg):
        out = self._rewrite_frames(msg)
        self.odom_pub.publish(out)
        # When there is no dedicated compat feed, mirror this stream to compat
        # so older configs (publish_compat_odom=true, compat_input_odom unset)
        # behave exactly like before.
        if self.compat_pub is not None and self.compat_sub is None:
            self.compat_pub.publish(out)

        pose = PoseStamped()
        pose.header = out.header
        pose.pose = out.pose.pose
        self.path_msg.header.stamp = out.header.stamp
        self.path_msg.poses.append(pose)
        if self.max_path_length > 0 and len(self.path_msg.poses) > self.max_path_length:
            self.path_msg.poses = self.path_msg.poses[-self.max_path_length:]
        self.path_pub.publish(self.path_msg)

    def _fast_callback(self, msg):
        # imu_propagate runs at IMU rate (~200Hz). Forward to compat only.
        if self.compat_pub is None:
            return
        out = self._rewrite_frames(msg)
        self.compat_pub.publish(out)


def main():
    rospy.init_node("vins_odom_bridge")
    VinsOdomBridge()
    rospy.spin()


if __name__ == "__main__":
    main()

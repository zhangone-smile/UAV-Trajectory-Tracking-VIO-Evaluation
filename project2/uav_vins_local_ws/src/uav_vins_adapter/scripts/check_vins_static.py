#!/usr/bin/env python3
"""Static checks for the local UAV VINS adapter workspace."""

import ast
import pathlib
import sys
import xml.etree.ElementTree as ET


ROOT = pathlib.Path(__file__).resolve().parents[3]
PKG = ROOT / "src" / "uav_vins_adapter"


REQUIRED_FILES = [
    ROOT / "README.md",
    PKG / "package.xml",
    PKG / "CMakeLists.txt",
    PKG / "launch" / "uav_vins_realsense_mavros.launch",
    PKG / "launch" / "foxglove_vins.launch",
    PKG / "config" / "vins_realsense_stereo_imu.yaml",
    PKG / "config" / "realsense_infra1_pinhole.yaml",
    PKG / "config" / "realsense_infra2_pinhole.yaml",
    PKG / "config" / "topic_map.yaml",
    PKG / "scripts" / "vins_odom_bridge.py",
    PKG / "scripts" / "dump_camera_info_for_vins.py",
    PKG / "scripts" / "_wait_for_file.sh",
]


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def check_required_files():
    for path in REQUIRED_FILES:
        require(path.exists(), f"missing file: {path}")


def check_xml():
    for rel in ["package.xml", "launch/uav_vins_realsense_mavros.launch", "launch/foxglove_vins.launch"]:
        ET.parse(PKG / rel)
    launch = ET.parse(PKG / "launch" / "uav_vins_realsense_mavros.launch")
    vins_nodes = [
        node
        for node in launch.findall(".//node")
        if node.attrib.get("pkg") == "$(arg vins_pkg)" and node.attrib.get("type") == "$(arg vins_type)"
    ]
    require(vins_nodes, "missing VINS node in launch")
    require(
        vins_nodes[0].attrib.get("args") == "$(arg config_file)",
        "VINS node must receive config_file as argv[1]",
    )


def check_python_parse():
    for path in (PKG / "scripts").glob("*.py"):
        ast.parse(path.read_text(), filename=str(path))


def check_vins_config():
    text = (PKG / "config" / "vins_realsense_stereo_imu.yaml").read_text()
    required_tokens = [
        'imu_topic: "/mavros/imu/data"',
        'image0_topic: "/camera/infra1/image_rect_raw"',
        'image1_topic: "/camera/infra2/image_rect_raw"',
        "estimate_extrinsic: 2",
        "estimate_td: 1",
        "loop_closure: 0",
        "multiple_thread: 1",
    ]
    for token in required_tokens:
        require(token in text, f"missing VINS config token: {token}")

    for camera in ["realsense_infra1_pinhole.yaml", "realsense_infra2_pinhole.yaml"]:
        cam_text = (PKG / "config" / camera).read_text()
        for token in ["model_type: PINHOLE", "image_width: 640", "image_height: 480", "projection_parameters:"]:
            require(token in cam_text, f"{camera} missing token: {token}")


def check_foxglove_whitelist():
    text = (PKG / "launch" / "foxglove_vins.launch").read_text()
    # These VINS publishers were previously missing and triggered foxglove
    # "topic not in whitelist" messages. Match via the regex prefix we added.
    for token in [
        '"/vins_estimator/.*"',
        '"/camera/infra1/.*"',
        '"/camera/infra2/.*"',
    ]:
        require(token in text, f"foxglove whitelist missing: {token}")


def check_auto_calib_wiring():
    launch = ET.parse(PKG / "launch" / "uav_vins_realsense_mavros.launch")
    setup = [n for n in launch.findall(".//node")
             if n.attrib.get("type") == "dump_camera_info_for_vins.py"]
    require(setup, "auto-calib node missing from launch")
    require("required" not in setup[0].attrib,
            "auto-calib node must not be required=true (it exits 0 after staging)")


def main():
    checks = [
        check_required_files,
        check_xml,
        check_python_parse,
        check_vins_config,
        check_foxglove_whitelist,
        check_auto_calib_wiring,
    ]
    for check in checks:
        check()
        print(f"PASS {check.__name__}")
    print("PASS static checks complete")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"FAIL {exc}", file=sys.stderr)
        sys.exit(1)

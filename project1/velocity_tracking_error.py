from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore


@dataclass(frozen=True)
class VelocityErrorStats:
    n: int
    rmse_xyz: np.ndarray
    rmse_norm: float
    mean_norm: float
    max_norm: float
    p95_norm: float


def _stamp_to_seconds(stamp: object) -> float:
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if sec is None or nanosec is None:
        raise ValueError(f"Unsupported stamp object: {type(stamp)}")
    return float(sec) + float(nanosec) * 1e-9


def _sorted_unique_time_series(t: np.ndarray, x: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    order = np.argsort(t)
    t_sorted = t[order]
    x_sorted = x[order]
    keep = np.concatenate(([True], np.diff(t_sorted) != 0.0))
    return t_sorted[keep], x_sorted[keep]


def _moving_average(x: np.ndarray, window: int) -> np.ndarray:
    if window <= 1:
        return x
    if window % 2 == 0:
        raise ValueError("moving average window must be odd")
    kernel = np.full((window,), 1.0 / float(window), dtype=np.float64)
    return np.column_stack([np.convolve(x[:, i], kernel, mode="same") for i in range(x.shape[1])])


def _resample_positions_and_differentiate(
    t: np.ndarray,
    p: np.ndarray,
    *,
    dt: float,
    smooth_window: int,
) -> tuple[np.ndarray, np.ndarray]:
    if dt <= 0:
        raise ValueError("dt must be positive")
    t0 = float(t[0])
    t1 = float(t[-1])
    if t1 <= t0 + 2 * dt:
        raise ValueError("time range too small for resampling")

    t_uniform = np.arange(t0, t1, dt, dtype=np.float64)
    p_uniform = np.column_stack([np.interp(t_uniform, t, p[:, i]) for i in range(3)])
    p_smooth = _moving_average(p_uniform, smooth_window)
    v_uniform = np.gradient(p_smooth, dt, axis=0)
    return t_uniform, v_uniform



def _load_ros1_typestore(project_root: Path) -> object:
    typestore = get_typestore(Stores.ROS1_NOETIC)
    pos_cmd_msg = project_root / "uav_ws" / "src" / "utils" / "quadrotor_msgs" / "msg" / "PositionCommand.msg"
    msgdef = pos_cmd_msg.read_text(encoding="utf-8")
    typestore.register(get_types_from_msg(msgdef, "quadrotor_msgs/msg/PositionCommand"))
    return typestore


def _pick_topic(available: set[str], preferred: Iterable[str]) -> str:
    for t in preferred:
        if t in available:
            return t
    raise ValueError(f"None of the preferred topics exist in bag: {list(preferred)}")


def compute_velocity_tracking_error(
    bag_path: Path,
    *,
    cmd_topic: str | None = None,
    actual_odom_topic: str | None = None,
    gt_pose_topic: str = "/vrpn_client_node/G3/pose",
    gt_resample_dt: float = 0.01,
    gt_smooth_window: int = 5,
    use_traj_flag_ready_only: bool = True,
) -> tuple[VelocityErrorStats, dict[str, str]]:
    project_root = bag_path.resolve().parent.parent
    typestore = _load_ros1_typestore(project_root)

    with Reader(bag_path) as reader:
        topics = {c.topic: c.msgtype for c in reader.connections}

        selected_cmd_topic = cmd_topic
        if selected_cmd_topic is None:
            candidates = [t for t, mt in topics.items() if mt.endswith("PositionCommand")]
            selected_cmd_topic = _pick_topic(set(candidates), ["/position_cmd", "/current_pose", *candidates])

        cmd_conn = [c for c in reader.connections if c.topic == selected_cmd_topic][0]

        cmd_t = []
        cmd_v = []
        for c, _, raw in reader.messages(connections=[cmd_conn]):
            m = typestore.deserialize_ros1(raw, c.msgtype)
            if use_traj_flag_ready_only and int(getattr(m, "trajectory_flag", 0)) != 1:
                continue
            cmd_t.append(_stamp_to_seconds(m.header.stamp))
            cmd_v.append([m.velocity.x, m.velocity.y, m.velocity.z])

        actual_t = []
        actual_v = []
        actual_topic_used = ""
        if actual_odom_topic is not None:
            if actual_odom_topic not in topics:
                raise ValueError(f"Missing actual odometry topic in bag: {actual_odom_topic}")
            odom_conn = [c for c in reader.connections if c.topic == actual_odom_topic][0]
            for c, _, raw in reader.messages(connections=[odom_conn]):
                m = typestore.deserialize_ros1(raw, c.msgtype)
                actual_t.append(_stamp_to_seconds(m.header.stamp))
                v = m.twist.twist.linear
                actual_v.append([v.x, v.y, v.z])
            actual_topic_used = actual_odom_topic
        else:
            if gt_pose_topic not in topics:
                raise ValueError(f"Missing ground-truth pose topic in bag: {gt_pose_topic}")
            gt_conn = [c for c in reader.connections if c.topic == gt_pose_topic][0]
            gt_t = []
            gt_p = []
            for c, _, raw in reader.messages(connections=[gt_conn]):
                m = typestore.deserialize_ros1(raw, c.msgtype)
                gt_t.append(_stamp_to_seconds(m.header.stamp))
                gt_p.append([m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z])

            gt_t = np.asarray(gt_t, dtype=np.float64)
            gt_p = np.asarray(gt_p, dtype=np.float64)
            if gt_t.size < 10:
                raise ValueError(f"Not enough ground-truth pose samples on {gt_pose_topic} in {bag_path.name}")

            gt_t, gt_p = _sorted_unique_time_series(gt_t, gt_p)
            actual_t, actual_v = _resample_positions_and_differentiate(
                gt_t,
                gt_p,
                dt=gt_resample_dt,
                smooth_window=gt_smooth_window,
            )
            actual_topic_used = gt_pose_topic

    cmd_t = np.asarray(cmd_t, dtype=np.float64)
    cmd_v = np.asarray(cmd_v, dtype=np.float64)
    actual_t = np.asarray(actual_t, dtype=np.float64)
    actual_v = np.asarray(actual_v, dtype=np.float64)

    if cmd_t.size < 2:
        raise ValueError(f"Not enough command samples on {selected_cmd_topic} in {bag_path.name}")
    if actual_t.size < 2:
        raise ValueError(f"Not enough actual velocity samples derived from {actual_topic_used} in {bag_path.name}")

    cmd_t, cmd_v = _sorted_unique_time_series(cmd_t, cmd_v)
    actual_t, actual_v = _sorted_unique_time_series(actual_t, actual_v)

    mask = (cmd_t >= actual_t[0]) & (cmd_t <= actual_t[-1])
    cmd_t = cmd_t[mask]
    cmd_v = cmd_v[mask]

    if cmd_t.size < 2:
        raise ValueError(
            f"No overlapping time range between cmd topic {selected_cmd_topic} and actual topic {actual_topic_used} in {bag_path.name}"
        )

    actual_v_interp = np.column_stack([np.interp(cmd_t, actual_t, actual_v[:, i]) for i in range(3)])
    err = actual_v_interp - cmd_v
    err_norm = np.linalg.norm(err, axis=1)

    rmse_xyz = np.sqrt(np.mean(err**2, axis=0))
    rmse_norm = float(np.sqrt(np.mean(err_norm**2)))
    mean_norm = float(np.mean(err_norm))
    max_norm = float(np.max(err_norm))
    p95_norm = float(np.percentile(err_norm, 95))

    stats = VelocityErrorStats(
        n=int(err.shape[0]),
        rmse_xyz=rmse_xyz,
        rmse_norm=rmse_norm,
        mean_norm=mean_norm,
        max_norm=max_norm,
        p95_norm=p95_norm,
    )
    meta = {
        "cmd_topic": selected_cmd_topic,
        "gt_pose_topic": actual_topic_used,
        "cmd_type": topics.get(selected_cmd_topic, ""),
        "gt_type": topics.get(actual_topic_used, ""),
    }
    return stats, meta


def _format_stats_row(bag_name: str, stats: VelocityErrorStats) -> list[str]:
    return [
        bag_name,
        str(stats.n),
        f"{stats.rmse_xyz[0]:.6f}",
        f"{stats.rmse_xyz[1]:.6f}",
        f"{stats.rmse_xyz[2]:.6f}",
        f"{stats.rmse_norm:.6f}",
        f"{stats.mean_norm:.6f}",
        f"{stats.p95_norm:.6f}",
        f"{stats.max_norm:.6f}",
    ]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag-dir", type=Path, default=Path("bag"))
    parser.add_argument("--cmd-topic", type=str, default=None)
    parser.add_argument("--actual-odom-topic", type=str, default=None)
    parser.add_argument("--gt-pose-topic", type=str, default="/vrpn_client_node/G3/pose")
    parser.add_argument("--gt-resample-dt", type=float, default=0.01)
    parser.add_argument("--gt-smooth-window", type=int, default=5)
    parser.add_argument("--include-nonready", action="store_true")
    parser.add_argument("--out-csv", type=Path, default=None)
    args = parser.parse_args()

    bag_paths = sorted(args.bag_dir.glob("*.bag"))
    if not bag_paths:
        raise SystemExit(f"No .bag files found under: {args.bag_dir}")

    header = [
        "bag",
        "n",
        "rmse_vx",
        "rmse_vy",
        "rmse_vz",
        "rmse_norm",
        "mean_norm",
        "p95_norm",
        "max_norm",
    ]

    rows = [header]
    for bag_path in bag_paths:
        stats, meta = compute_velocity_tracking_error(
            bag_path,
            cmd_topic=args.cmd_topic,
            actual_odom_topic=args.actual_odom_topic,
            gt_pose_topic=args.gt_pose_topic,
            gt_resample_dt=args.gt_resample_dt,
            gt_smooth_window=args.gt_smooth_window,
            use_traj_flag_ready_only=not args.include_nonready,
        )
        rows.append(_format_stats_row(bag_path.name, stats))

        print(f"{bag_path.name}")
        print(f"  cmd_topic: {meta['cmd_topic']} ({meta['cmd_type']})")
        print(f"  gt_topic : {meta['gt_pose_topic']} ({meta['gt_type']})")
        print(
            "  rmse_xyz: "
            f"[{stats.rmse_xyz[0]:.6f}, {stats.rmse_xyz[1]:.6f}, {stats.rmse_xyz[2]:.6f}] m/s"
        )
        print(
            "  norm    : "
            f"rmse={stats.rmse_norm:.6f} mean={stats.mean_norm:.6f} p95={stats.p95_norm:.6f} max={stats.max_norm:.6f} (m/s)"
        )

    if args.out_csv is not None:
        args.out_csv.parent.mkdir(parents=True, exist_ok=True)
        with args.out_csv.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerows(rows)
        print(f"saved: {args.out_csv}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

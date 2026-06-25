"""Analysis script for project2 bag data - computes position and velocity tracking errors.

Key methodology:
- Only odometry AFTER trajectory display is used (before segment is pre-flight)
- Odometry data is filtered to detect landing (Z error spike); pre-landing data is the actual flight
- APE = nearest reference point distance (intuitive: "how far is drone from path")
- RPE = frame-to-frame motion consistency
- Velocity error = odometry twist vs numerically differentiated position
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore


@dataclass(frozen=True)
class TrackingStats:
    n_flight: int
    flight_duration: float
    path_length_ref: float
    path_length_est: float
    # APE - Absolute Pose Error (nearest reference point distance)
    ape_rmse: float
    ape_mean: float
    ape_max: float
    ape_p95: float
    # APE pre-landing (actual flight portion only)
    ape_rmse_prelanding: float
    ape_mean_prelanding: float
    # RPE - Relative Pose Error
    rpe_rmse: float
    rpe_mean: float
    rpe_max: float
    rpe_p95: float
    # Velocity error (odom twist vs differentiated)
    vel_rmse_xyz: np.ndarray
    vel_rmse_norm: float
    vel_mean_norm: float
    vel_max_norm: float
    vel_p95_norm: float


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def moving_average(x, window):
    if window <= 1:
        return x
    kernel = np.full((window,), 1.0 / float(window), dtype=np.float64)
    return np.column_stack([np.convolve(x[:, i], kernel, mode="same") for i in range(x.shape[1])])


def resample_by_arc(p, n):
    arc = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))])
    if arc[-1] == 0:
        return np.tile(p[0], (n, 1))
    target = np.linspace(0, arc[-1], n)
    return np.column_stack([np.interp(target, arc, p[:, i]) for i in range(p.shape[1])])


def path_length(p):
    return float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))


def compute_tracking_error(bag_path: Path, ref_topic: str, est_topic: str, smooth_window: int = 5):
    """Compute tracking error metrics from bag file."""
    typestore = get_typestore(Stores.ROS1_NOETIC)

    with Reader(bag_path) as reader:
        topics = {c.topic: c.msgtype for c in reader.connections}

        # Read reference trajectory (static path)
        if ref_topic not in topics:
            raise ValueError(f"Reference topic {ref_topic} not found. Available: {list(topics.keys())}")

        ref_conn = [c for c in reader.connections if c.topic == ref_topic][0]
        ref_pts = []
        traj_time = None
        for c, _, raw in reader.messages(connections=[ref_conn]):
            m = typestore.deserialize_ros1(raw, ref_conn.msgtype)
            traj_time = stamp_to_seconds(m.header.stamp)
            if hasattr(m, 'points'):
                for pt in m.points:
                    ref_pts.append([pt.x, pt.y, pt.z])
            break
        ref_pts = np.asarray(ref_pts, dtype=np.float64)

        # Read odometry AFTER trajectory display
        if est_topic not in topics:
            raise ValueError(f"Estimate topic {est_topic} not found. Available: {list(topics.keys())}")

        est_conn = [c for c in reader.connections if c.topic == est_topic][0]
        est_t, est_p, est_v = [], [], []
        for c, _, raw in reader.messages(connections=[est_conn]):
            m = typestore.deserialize_ros1(raw, est_conn.msgtype)
            ts = stamp_to_seconds(m.header.stamp)
            if ts >= traj_time:
                est_t.append(ts)
                est_p.append([m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z])
                est_v.append([m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z])

    est_t = np.asarray(est_t, dtype=np.float64)
    est_p = np.asarray(est_p, dtype=np.float64)
    est_v = np.asarray(est_v, dtype=np.float64)

    path_len_ref = path_length(ref_pts)
    path_len_est = path_length(est_p)

    # Detect landing: sustained Z error < -0.30m indicates drone on ground
    ape_nn = np.array([np.min(np.linalg.norm(ref_pts - p, axis=1)) for p in est_p])
    nearest_ref_idx = np.array([np.argmin(np.linalg.norm(ref_pts - p, axis=1)) for p in est_p])
    z_err = est_p[:, 2] - ref_pts[nearest_ref_idx, 2]
    landing_mask = z_err < -0.30
    landing_idx = np.argmax(landing_mask) if np.any(landing_mask) else len(est_p)

    # Pre-landing = actual flight portion
    flight_mask = slice(0, landing_idx)
    est_p_flight = est_p[flight_mask]
    est_t_flight = est_t[flight_mask]
    ape_flight = ape_nn[flight_mask]

    # APE full
    ape_rmse = math.sqrt(np.mean(ape_nn**2))
    ape_mean = float(np.mean(ape_nn))
    ape_max = float(np.max(ape_nn))
    ape_p95 = float(np.percentile(ape_nn, 95))

    # APE pre-landing
    ape_rmse_prelanding = math.sqrt(np.mean(ape_flight**2))
    ape_mean_prelanding = float(np.mean(ape_flight))

    # RPE using arc-length matched resampling (pre-landing only for meaningful RPE)
    est_resampled = resample_by_arc(est_p_flight, len(ref_pts))
    rpe = np.diff(est_resampled, axis=0) - np.diff(ref_pts, axis=0)
    rpe_norm = np.linalg.norm(rpe, axis=1)
    rpe_rmse = math.sqrt(np.mean(rpe_norm**2))
    rpe_mean = float(np.mean(rpe_norm))
    rpe_max = float(np.max(rpe_norm))
    rpe_p95 = float(np.percentile(rpe_norm, 95))

    # Velocity error (full dataset)
    dt = 0.01
    t_u = np.arange(est_t[0], est_t[-1], dt)
    p_u = np.column_stack([np.interp(t_u, est_t, est_p[:, i]) for i in range(3)])
    v_u = np.column_stack([np.interp(t_u, est_t, est_v[:, i]) for i in range(3)])
    p_smooth = moving_average(p_u, smooth_window)
    v_diff = np.gradient(p_smooth, dt, axis=0)
    vel_err = v_u - v_diff
    vel_err_norm = np.linalg.norm(vel_err, axis=1)
    vel_rmse_xyz = np.sqrt(np.mean(vel_err**2, axis=0))
    vel_rmse_norm = math.sqrt(np.mean(vel_err_norm**2))
    vel_mean_norm = float(np.mean(vel_err_norm))
    vel_max_norm = float(np.max(vel_err_norm))
    vel_p95_norm = float(np.percentile(vel_err_norm, 95))

    return TrackingStats(
        n_flight=int(landing_idx),
        flight_duration=float(est_t_flight[-1] - est_t_flight[0]),
        path_length_ref=path_len_ref,
        path_length_est=path_len_est,
        ape_rmse=ape_rmse,
        ape_mean=ape_mean,
        ape_max=ape_max,
        ape_p95=ape_p95,
        ape_rmse_prelanding=ape_rmse_prelanding,
        ape_mean_prelanding=ape_mean_prelanding,
        rpe_rmse=rpe_rmse,
        rpe_mean=rpe_mean,
        rpe_max=rpe_max,
        rpe_p95=rpe_p95,
        vel_rmse_xyz=vel_rmse_xyz,
        vel_rmse_norm=vel_rmse_norm,
        vel_mean_norm=vel_mean_norm,
        vel_max_norm=vel_max_norm,
        vel_p95_norm=vel_p95_norm,
    )


def main():
    parser = argparse.ArgumentParser(description="Analyze position and velocity tracking error from bag file")
    parser.add_argument("--bag", type=Path, default=Path("bag/record.bag"), help="Path to bag file")
    parser.add_argument("--ref-topic", type=str, default="/eight/traj_viz", help="Reference trajectory topic")
    parser.add_argument("--est-topic", type=str, default="/ekf/ekf_odom", help="Estimate (odometry) topic")
    parser.add_argument("--smooth-window", type=int, default=5, help="Smoothing window size (odd)")
    parser.add_argument("--out-csv", type=Path, default=Path("results/analysis_results.csv"), help="Output CSV file")
    args = parser.parse_args()

    print(f"Analyzing: {args.bag}")
    print(f"Reference topic: {args.ref_topic}")
    print(f"Estimate topic: {args.est_topic}")

    stats = compute_tracking_error(
        args.bag,
        ref_topic=args.ref_topic,
        est_topic=args.est_topic,
        smooth_window=args.smooth_window,
    )

    print(f"\n=== Tracking Error Statistics ===")
    print(f"Flight points: {stats.n_flight} / duration: {stats.flight_duration:.2f} s")
    print(f"Reference path: {stats.path_length_ref:.3f} m / Odometry path: {stats.path_length_est:.3f} m")
    print(f"\n--- Position Error (APE - Nearest Reference Point Distance) ---")
    print(f"  Full flight RMSE: {stats.ape_rmse:.6f} m ({stats.ape_rmse*100:.2f} cm)")
    print(f"  Full flight Mean: {stats.ape_mean:.6f} m ({stats.ape_mean*100:.2f} cm)")
    print(f"  Full flight Max: {stats.ape_max:.6f} m")
    print(f"  Full flight 95th: {stats.ape_p95:.6f} m")
    print(f"\n  Pre-landing RMSE: {stats.ape_rmse_prelanding:.6f} m ({stats.ape_rmse_prelanding*100:.2f} cm)")
    print(f"  Pre-landing Mean: {stats.ape_mean_prelanding:.6f} m ({stats.ape_mean_prelanding*100:.2f} cm)")
    print(f"\n--- Local Motion Error (RPE - Frame-to-Frame) ---")
    print(f"  RMSE:   {stats.rpe_rmse:.6f} m")
    print(f"  Mean:   {stats.rpe_mean:.6f} m")
    print(f"  Max:    {stats.rpe_max:.6f} m")
    print(f"  95th:   {stats.rpe_p95:.6f} m")
    print(f"\n--- Velocity Error (Odom Twist vs Differentiated) ---")
    print(f"  RMSE:   [{stats.vel_rmse_xyz[0]:.3f}, {stats.vel_rmse_xyz[1]:.3f}, {stats.vel_rmse_xyz[2]:.3f}] m/s")
    print(f"  RMSE N: {stats.vel_rmse_norm:.3f} m/s")
    print(f"  Mean N: {stats.vel_mean_norm:.3f} m/s")
    print(f"  95th N: {stats.vel_p95_norm:.3f} m/s")

    if args.out_csv:
        args.out_csv.parent.mkdir(parents=True, exist_ok=True)
        with open(args.out_csv, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["metric", "value"])
            writer.writerow(["n_flight", stats.n_flight])
            writer.writerow(["flight_duration_s", stats.flight_duration])
            writer.writerow(["path_length_ref_m", stats.path_length_ref])
            writer.writerow(["path_length_est_m", stats.path_length_est])
            writer.writerow(["ape_rmse_m", stats.ape_rmse])
            writer.writerow(["ape_mean_m", stats.ape_mean])
            writer.writerow(["ape_max_m", stats.ape_max])
            writer.writerow(["ape_p95_m", stats.ape_p95])
            writer.writerow(["ape_rmse_prelanding_m", stats.ape_rmse_prelanding])
            writer.writerow(["ape_mean_prelanding_m", stats.ape_mean_prelanding])
            writer.writerow(["rpe_rmse_m", stats.rpe_rmse])
            writer.writerow(["rpe_mean_m", stats.rpe_mean])
            writer.writerow(["rpe_max_m", stats.rpe_max])
            writer.writerow(["rpe_p95_m", stats.rpe_p95])
            writer.writerow(["vel_rmse_vx_m_s", stats.vel_rmse_xyz[0]])
            writer.writerow(["vel_rmse_vy_m_s", stats.vel_rmse_xyz[1]])
            writer.writerow(["vel_rmse_vz_m_s", stats.vel_rmse_xyz[2]])
            writer.writerow(["vel_rmse_norm_m_s", stats.vel_rmse_norm])
            writer.writerow(["vel_mean_norm_m_s", stats.vel_mean_norm])
            writer.writerow(["vel_max_norm_m_s", stats.vel_max_norm])
            writer.writerow(["vel_p95_norm_m_s", stats.vel_p95_norm])
        print(f"\nSaved to: {args.out_csv}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
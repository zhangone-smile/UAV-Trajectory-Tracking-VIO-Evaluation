# ELEC5660 — Lab Test Repo (Group 3)

Jetson-side repo for drone flight testing and trajectory evaluation.

---

## Setup

### 1. Install Pixi

```bash
curl -fsSL https://pixi.sh/install.sh | sh
```

Restart your shell so `pixi` is on your `PATH`.

### 2. Install the environment

```bash
pixi install
```

### 3. Configure ROS networking

Edit `pixi.toml` under `[activation.env]`:

| Variable | Value |
|---|---|
| `ROS_MASTER_URI` | `http://<JETSON_IP>:11311` |
| `ROS_IP` | `<YOUR_LAPTOP_IP>` |

Current values: Jetson `192.168.0.54`, Laptop `192.168.0.153`.

---

## Visualisation

```bash
pixi run rviz          # launch RViz with rviz.rviz config
pixi run topic         # list ROS topics (sanity check connectivity)
```

RViz config: `visualization/rviz.rviz`

---

## Trajectory Evaluation (evo)

Reference: https://michaelgrupp.github.io/evo/

### Bag files

| File | Description |
|---|---|
| `bag/circle.bag` | Circle trajectory |
| `bag/cos.bag` | Cosine trajectory |
| `bag/donut.bag` | Donut trajectory |
| `bag/eight.bag` | Figure-eight trajectory |

### Useful topics (from `circle.bag`)

| Topic | Poses | Path length |
|---|---|---|
| `/vrpn_client_node/G3/pose` | 1603 | 4.611 m |
| `/ekf/cam_ekf_odom` | 1603 | 4.619 m |
| `/ekf/ekf_odom` | 1259 | 4.826 m |
| `/mavros/local_position/odom` | 258 | 5.473 m |

### Commands

**Inspect topics in a bag:**
```bash
evo_traj bag bag/circle.bag --all_topics
```

**Absolute Pose Error (APE) — position RMSE, max error:**
```bash
evo_ape bag bag/circle.bag /vrpn_client_node/G3/pose /ekf/cam_ekf_odom \
    -va --plot --save_results results/ape.zip
```

**3D trajectory comparison:**
```bash
evo_traj bag bag/circle.bag /ekf/cam_ekf_odom \
    --ref /vrpn_client_node/G3/pose -p --plot_mode xyz
```

**Summarise results to table:**
```bash
evo_res results/*.zip -p --save_table results/table.csv
```

**Relative Pose Error (RPE) — velocity-level proxy:**
```bash
evo_rpe bag bag/circle.bag /vrpn_client_node/G3/pose /ekf/cam_ekf_odom \
    -va --delta 1 --plot
```

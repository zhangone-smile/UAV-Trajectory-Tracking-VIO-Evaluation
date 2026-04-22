# RViz (Pixi)

This folder contains a Pixi environment and tasks to run RViz with the provided display config (`rviz.rviz`).

## 1) Install Pixi

```bash
curl -fsSL https://pixi.sh/install.sh | sh
```

Restart your shell (or open a new terminal) so `pixi` is on your `PATH`.

## 2) Install the environment

From this folder:

```bash
pixi install
```

## 3) Set ROS networking (Jetson + laptop)

Edit `pixi.toml` and update these values under `[activation.env]`:

- `ROS_MASTER_URI` should point to the ROS master (commonly the Jetson):
  - Example: `http://<JETSON_IP>:11311`
- `ROS_IP` should be the IP of the machine running RViz (your laptop):
  - Example: `<LAPTOP_IP>`

## 4) Run RViz

```bash
pixi run rviz
```

Optional: list ROS topics to confirm you can see the ROS master:

```bash
pixi run topic
```

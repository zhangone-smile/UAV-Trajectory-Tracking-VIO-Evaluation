evo link: https://michaelgrupp.github.io/evo/


**First, check your topics:**
```bash
evo_traj bag your_flight.bag --all_topics
```
You're looking for something like `/mavros/local_position/pose`, `/optitrack/pose`, or whatever your SLAM/OptiTrack topic is named.

**Position error (APE) — this gives you RMSE, max error etc.:**
```bash
evo_ape bag your_flight.bag /reference_traj /actual_pose -va --plot --save_results results/ape.zip
```

**Then dump the stats to a table:**
```bash
evo_res results/*.zip -p --save_table results/table.csv
```
This directly fills in your Position RMSE and Max Position Error table cells.

---

**For velocity** — evo doesn't do velocity natively, so you have two options:

**Easy:** use RPE with `--delta 1` as a proxy for velocity-level error:
```bash
evo_rpe bag your_flight.bag /reference_traj /actual_pose -va --delta 1 --plot
```

**Better:** extract and plot velocity directly in Python:
```python
from rosbags.rosbag1 import Reader
# read /odom or /mavros/local_position/velocity and plot with matplotlib
```

---

**For the 3D trajectory comparison figure:**
```bash
evo_traj bag your_flight.bag /actual_pose --ref /reference_traj -p --plot_mode xyz
```

---

What are your actual topic names in the bag? I can give you the exact commands ready to copy-paste.
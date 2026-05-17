#!/bin/bash
# Compile all native packages, excluding VINS (which runs in Docker)

mv src/VINS-Fusion-ELEC5660 /tmp/VINS-Fusion-ELEC5660.bak

export PATH=/usr/bin:$PATH
catkin_make

mv /tmp/VINS-Fusion-ELEC5660.bak src/VINS-Fusion-ELEC5660

echo "Build complete! Run ./run.sh to launch everything (VINS in Docker, rest native)."

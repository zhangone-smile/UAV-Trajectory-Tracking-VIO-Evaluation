# VINS-Fusion Lag Investigation

## Findings
The VINS estimator was experiencing a lag of over **300 seconds (5.5 minutes)**. 

### 1. Root Cause: Buffer Bloat
*   **Memory Usage:** The `vins_node` process was consuming **4.7GB of RAM** before it crashed/restarted.
*   **Buffer Calculation:** At 640x480 resolution, 4.7GB holds approximately **15,000 images**. 
*   **Latency:** Processing 15,000 images at 30 FPS takes roughly 500 seconds, which matches the observed delay.
*   **Code Flaw:** In `VINS-Fusion/vins_estimator/src/rosNodeTest.cpp`, the `img0_callback` and `img1_callback` functions push every incoming image into `img0_buf` and `img1_buf` without any size limits.
*   **Processing Bottleneck:** If the VINS solver (`max_solver_time: 0.04s` = 25Hz) is slower than the camera rate (30Hz), the queue grows indefinitely.

### 2. Current Status
*   The node has restarted and is currently near real-time (~0.8s lag).
*   RAM usage is back to normal (~40MB initial).
*   **Warning:** The lag will accumulate again if the processing load remains high.

## Proposed Fixes

### Short-Term: Buffer Capping
Modify `img0_callback` and `img1_callback` in `rosNodeTest.cpp` to drop the oldest frame if the queue size exceeds a small threshold (e.g., 5-10 frames).

### Performance: Jetson Optimization
Ensure the Jetson Orin Nano is running in High Performance mode:
*   Check mode: `nvpmodel -q`
*   Max clocks: `sudo jetson_clocks`

### Configuration: VINS Parameters
If the CPU cannot keep up with 30Hz, consider:
*   Increasing `max_solver_time` (though this increases latency per frame).
*   Reducing `max_num_iterations`.
*   Downsampling the camera frame rate to 20Hz or 15Hz at the source.

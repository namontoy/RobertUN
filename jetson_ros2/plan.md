# Weekly Plan — Jetson Orin Nano Stereo Perception (ROS 2)

## Complete objective
Build a robotics-grade **edge perception subsystem** on Jetson Orin Nano using a **stereo camera**. The system will acquire synchronized left/right images, 
compute depth, detect objects on the GPU, track them over time, and publish results through ROS 2 with visualization and reliability features.

---

## Systems we will use

### Hardware
- NVIDIA Jetson Orin Nano
- Stereo camera 

### Operating system and NVIDIA stack
- Ubuntu 22.04 (Jammy)
- Jetson Linux R36.4.3 (JetPack 6.x era)
- CUDA 12.6

### Robotics middleware and tooling
- ROS 2 Humble
- colcon (build system for ROS 2)
- RViz2 (visualization)

### Perception and acceleration
- OpenCV (stereo calibration, rectification, disparity/depth utilities)
- Camera capture via V4L2 and/or GStreamer (depending on the camera driver path)
- TensorRT (GPU-accelerated inference)

### Languages
- Python (rapid iteration, prototyping, ROS2 nodes early)
- C++ (performance-critical nodes and memory/latency control later)

### Deployment and reliability
- systemd (autostart + restart on failure)
- structured logging + benchmark reports

---

## Final product
A ROS 2 stereo perception pipeline that runs on Jetson and outputs **tracked objects with distance** in real time.

### Final system capabilities
- Stereo camera acquisition (left/right)
- Stereo calibration + rectification
- Depth estimation (disparity → depth)
- Object detection accelerated on Jetson GPU with TensorRT
- Multi-object tracking with stable IDs
- Fusion of tracking + depth: each tracked object includes estimated **distance (meters)**
- ROS 2 topics + RViz visualization
- Benchmarks: FPS, latency (p50/p95), CPU/RAM usage, depth sanity checks
- Reliability: one-command launch + systemd autostart + restart on failure

### Definition of Done
The project is done when:
1. One command starts the full pipeline (`ros2 launch ...`).
2. RViz shows live stereo, detections, and tracked objects.
3. Tracked objects include a distance estimate in meters.
4. A final benchmark report exists with performance and depth sanity results.
5. The system can auto-start and recover from simple failures (node crash/restart).

---


### Week 0 — Baseline and repo foundation
Goal: capture system info (Ubuntu/Jetson Linux/CUDA) + storage benchmark, create clean structure and creating a plan. This one.

### Week 1 — ROS 2 Humble bring-up + workspace
Goal: install ROS 2 Humble, create `ros2_ws`, build a first package, and run a basic node.
Output: reproducible build instructions + a heartbeat node (`/alive`).

### Week 2 — Stereo camera ROS publisher (Python)
Goal: publish `/stereo/left/image_raw` and `/stereo/right/image_raw` with FPS overlay and stable streaming.
Output: camera benchmark notes (FPS + stability).

### Week 3 — Stereo sync + dataset capture (Python)
Goal: save synchronized stereo pairs with timestamps and verify frame pairing quality.
Output: sync validation notes and sample dataset captured locally.

### Week 4 — Stereo calibration + rectification
Goal: calibrate stereo and generate rectified left/right images.
Output: calibration files committed + rectification demo.

### Week 5 — Depth estimation baseline (Python)
Goal: compute disparity/depth and publish `/stereo/depth`.
Output: depth sanity checks at multiple distances + error notes.

### Week 6 — Object detection baseline (Python)
Goal: run a detector and publish `/detections` + overlay results.
Output: baseline inference FPS/latency benchmark.

### Week 7 — TensorRT acceleration
Goal: accelerate inference and compare before/after latency and FPS.
Output: benchmark table with improvements (p50/p95 latency).

### Week 8 — Multi-object tracking (Python)
Goal: assign stable IDs across frames and publish `/tracked_objects`.
Output: tracking stability test notes (ID switches, lost tracks).

### Week 9 — Depth + tracking fusion (distance per object)
Goal: each tracked object includes estimated distance in meters.
Output: distance stability test (jitter, invalid depth handling).

### Week 10 — C++ optimization
Goal: rewrite one performance-critical node in C++ (depth OR tracking OR preprocessing).
Output: measured performance improvement and clean CMake build.

### Week 11 — Launch files + RViz integration
Goal: one `ros2 launch` starts everything; RViz config saved.
Output: “one command demo” instructions.

### Week 12 — Reliability + embedded deployment
Goal: systemd service, restart-on-failure, structured logs, basic recovery workflow.
Output: reboot-to-demo and crash recovery notes.

### Week 13 — Final benchmarking and failure tests
Goal: produce a final performance report (FPS, latency p50/p95, CPU/RAM, depth accuracy).
Output: `benchmarks/final_report.md`.

### Week 14 — Release packaging
Goal: polish README, architecture diagram, and a final demo
Output: clear run instructions + results + roadmap for improvements.

---


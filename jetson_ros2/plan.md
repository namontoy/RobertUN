# Weekly Plan — Jetson Orin Nano Stereo Perception (ROS 2)

## Complete objective
Build a robotics-grade **edge perception subsystem** on Jetson Orin Nano using a **stereo camera**. The system will acquire synchronized left/right images,
compute depth, detect objects on the GPU, track them over time, and publish results through ROS 2 with visualization and reliability features.

---

## Systems we will use

### Hardware
- NVIDIA Jetson Orin Nano
- **Stereolabs ZED stereo camera** (ZED family; prefer a model with IMU + magnetometer support if magnetometer is required) ([stereolabs.com](https://www.stereolabs.com/docs/ros2/zed-node))

### Operating system and NVIDIA stack
- Ubuntu 22.04 (Jammy)
- Jetson Linux R36.4.3 (JetPack 6.x era)
- CUDA 12.6

### Robotics middleware and tooling
- ROS 2 Humble
- colcon (build system for ROS 2)
- RViz2 (visualization)

### Perception, sensors, and acceleration
- **Stereolabs ZED SDK + ZED ROS 2 wrapper (`zed-ros2-wrapper`)** for:
  - Left/right images (raw + rectified), depth, point cloud, and sensor streams ([stereolabs.com](https://www.stereolabs.com/docs/ros2))
  - IMU streams and transforms (camera↔IMU) ([stereolabs.com](https://www.stereolabs.com/docs/ros2/zed-node))
- OpenCV (stereo utilities, optional cross-checks, additional processing)
- TensorRT (GPU-accelerated inference)

### Inertial sensing requirements
We will ingest and publish:
- **Accelerometer + gyroscope** (`~/imu/data_raw`) ([stereolabs.com](https://www.stereolabs.com/docs/ros2/zed-node))
- **IMU fused orientation** (`~/imu/data`) ([stereolabs.com](https://www.stereolabs.com/docs/ros2/zed-node))
- **Magnetometer** (`~/imu/mag`) *(only supported on ZED 2 / ZED 2i)* ([stereolabs.com](https://www.stereolabs.com/docs/ros2/zed-node))

### Languages
- Python (rapid iteration, prototyping, ROS2 nodes early)
- C++ (performance-critical nodes and memory/latency control later)

### Docker utilization (Articulated Robotics approach) + ZED images
We will containerize development and (optionally) deployment:
- Use a **project Dockerfile** and/or **VS Code Dev Containers** workflow (Crafting Dockerfile → Devices in Docker → Dev Containers) ([articulatedrobotics.xyz](https://articulatedrobotics.xyz/tutorials/docker/crafting-dockerfile/))
- Leverage **Stereolabs-provided Docker resources** for ROS 2 + ZED on Jetson:
  - `Dockerfile.l4t-humble` (Jetson) and `Dockerfile.desktop-humble` (desktop) as reference/base ([stereolabs.com](https://www.stereolabs.com/docs/docker/configure-ros2-dockerfile))
- Ensure containers can access:
  - GPU (NVIDIA container runtime)
  - Camera devices (USB/video nodes) and required permissions (device pass-through / privileged as needed)
  - ROS 2 networking (host networking when appropriate)

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
- **IMU sensing:** accelerometer + gyroscope + (if supported) magnetometer readings available in ROS 2 ([stereolabs.com](https://www.stereolabs.com/docs/ros2/zed-node))
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

**Additions for ZED + Docker:**
- Install/verify **ZED SDK** and confirm the camera works using **Stereolabs tooling** (basic streaming + depth preview as a sanity check). ([stereolabs.com](https://www.stereolabs.com/docs/ros2))
- Add `docker/` folder:
  - initial Jetson-focused Dockerfile (ROS 2 Humble base)
  - optional `.devcontainer/` for Dev Containers workflow ([articulatedrobotics.xyz](https://articulatedrobotics.xyz/tutorials/docker/dev-containers/))
- Note whether Week 2+ development will be:
  - (A) native on Jetson, (B) inside Docker on Jetson, or (C) devcontainer from a host PC targeting Jetson (documented choice)

### Week 2 — Stereo camera ROS publisher (Python)
Goal: publish `/stereo/left/image_raw` and `/stereo/right/image_raw` with FPS overlay and stable streaming.
Output: camera benchmark notes (FPS + stability).

Update for ZED:
- Bring up **ZED ROS 2 wrapper** and confirm left/right topics (raw + rectified availability). ([stereolabs.com](https://www.stereolabs.com/docs/ros2))
- Confirm **IMU topics** are publishing (accel/gyro/orientation; magnetometer if supported). ([stereolabs.com](https://www.stereolabs.com/docs/ros2/zed-node))
- If using Docker: confirm camera + IMU access from inside the container (device pass-through + permissions).

### Week 3 — Stereo sync + dataset capture (Python)
Goal: save synchronized stereo pairs with timestamps and verify frame pairing quality.
Output: sync validation notes and sample dataset captured locally.

Update for ZED + Stereolabs:
- Capture a short dataset using **Stereolabs SVO recording** for repeatable testing (plus an exported stereo-pair dataset if needed). ([stereolabs.com](https://www.stereolabs.com/docs/ros2))
- Extend dataset capture to include **IMU samples** aligned to image timestamps (and magnetometer where supported). ([stereolabs.com](https://www.stereolabs.com/docs/ros2/zed-node))

### Week 4 — Stereo calibration + rectification
Goal: calibrate stereo and generate rectified left/right images.
Output: calibration files committed + rectification demo.

Update for ZED:
- Verify factory calibration/rectification pipeline via the ZED wrapper (rectified topics + TF consistency). ([stereolabs.com](https://www.stereolabs.com/docs/ros2))
- Validate/record the camera↔IMU transform availability (`~/left_cam_imu_transform`) for later fusion. ([stereolabs.com](https://www.stereolabs.com/docs/ros2/zed-node))
- (Optional) Run an OpenCV-based calibration cross-check if you want an independent baseline.

### Week 5 — Depth estimation baseline (Python)
Goal: compute disparity/depth and publish `/stereo/depth`.
Output: depth sanity checks at multiple distances + error notes.

Update for ZED (depth + distance):
- Use ZED depth outputs as the baseline (depth in meters + point cloud where required). ([stereolabs.com](https://www.stereolabs.com/docs/ros2))
- Define the project’s **distance rule** (per-object distance from depth/point cloud). If using point cloud distance, use Euclidean distance relative to the left camera frame as the reference method. ([stereolabs.com](https://www.stereolabs.com/docs/depth-sensing/using-depth))
- Depth sanity checks at multiple distances + confidence/invalid-depth handling notes.

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

Update for IMU:
- Add an IMU “health + alignment” check: confirm accel/gyro streams are stable and timestamps are consistent with image streams; include magnetometer validation if supported. ([stereolabs.com](https://www.stereolabs.com/docs/ros2/zed-node))
- (Optional) Use IMU motion cues to improve distance stability (e.g., smoothing rules during fast motion), while keeping the output contract the same.

### Week 10 — C++ optimization
Goal: rewrite one performance-critical node in C++ (depth OR tracking OR preprocessing).
Output: measured performance improvement and clean CMake build.

### Week 11 — Launch files + RViz integration
Goal: one `ros2 launch` starts everything; RViz config saved.
Output: “one command demo” instructions.

Update for Jetson + ZED:
- Keep RViz workload reasonable on Jetson (avoid heavy point cloud + full-res images at once when benchmarking reliability). ([stereolabs.com](https://www.stereolabs.com/docs/ros2))

### Week 12 — Reliability + embedded deployment
Goal: systemd service, restart-on-failure, structured logs, basic recovery workflow.
Output: reboot-to-demo and crash recovery notes.

Update for Docker option:
- Provide two service modes (documented):
  - Native: systemd starts `ros2 launch ...`
  - Containerized: systemd starts `docker run ...` (or docker compose) for the full pipeline

### Week 13 — Final benchmarking and failure tests
Goal: produce a final performance report (FPS, latency p50/p95, CPU/RAM, depth accuracy).
Output: `benchmarks/final_report.md`.

Include:
- Depth sanity + per-object distance stability results
- IMU stream sanity (accel/gyro/mag where supported) ([stereolabs.com](https://www.stereolabs.com/docs/ros2/zed-node))

### Week 14 — Release packaging
Goal: polish README, architecture diagram, and a final demo
Output: clear run instructions + results + roadmap for improvements.

Additions:
- Document ZED-specific setup (SDK + wrapper launch + key topics)
- Document Docker workflow (how to build/run dev container + how to run on Jetson) ([stereolabs.com](https://www.stereolabs.com/docs/docker/configure-ros2-dockerfile))

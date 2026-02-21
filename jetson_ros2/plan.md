# Weekly Plan — Jetson Orin Nano Stereo Perception (ROS 2)

## Complete objective

Build a robotics-grade **edge perception subsystem** on Jetson Orin Nano using a **stereo camera (ZED 2i)**. The system will acquire synchronized left/right images, compute depth, run GPU-accelerated object detection, track objects over time with stable IDs, and publish results through ROS 2 with visualization and reliability features.

---

## Systems we will use

### Hardware

* NVIDIA Jetson Orin Nano
* **Stereolabs ZED 2i stereo camera**

### Operating system and NVIDIA stack

* Ubuntu 22.04 (Jammy)
* Jetson Linux R36.4.3 (JetPack 6.x era)
* CUDA 12.6

### Robotics middleware and tooling

* ROS 2 Humble
* colcon (build system for ROS 2)
* RViz2 (visualization)

### Perception, sensors, and acceleration

* **Stereolabs ZED SDK + ZED ROS 2 wrapper (`zed-ros2-wrapper`)** for:

  * Left/right images (raw + rectified), depth, point cloud, and sensor streams
  * IMU streams and transforms (camera↔IMU)
  * **Object detection output (`~/obj_det/objects`)**
  * **Tracking support (stable IDs when enabled)**
* OpenCV (optional cross-checks, additional processing)

### Inertial sensing requirements (ZED 2i)

We will ingest and publish:

* **Accelerometer + gyroscope**
* **IMU fused orientation**
* **Magnetometer**

### Languages

* Python (rapid iteration, prototyping, ROS2 nodes early)
* C++ (performance-critical nodes and memory/latency control later)

### Docker utilization (Articulated Robotics approach) + ZED images

We will containerize development and (optionally) deployment:

* Use a **project Dockerfile** and/or **VS Code Dev Containers** workflow (Crafting Dockerfile → Devices in Docker → Dev Containers)
* Leverage **Stereolabs-provided Docker resources** for ROS 2 + ZED on Jetson as reference/base:

  * `Dockerfile.l4t-humble` (Jetson) and `Dockerfile.desktop-humble` (desktop)
* Ensure containers can access:

  * GPU (NVIDIA container runtime)
  * Camera devices and required permissions (device pass-through / privileged as needed)
  * ROS 2 networking (host networking when appropriate)

### Deployment and reliability

* systemd (autostart + restart on failure)
* structured logging + benchmark reports

---

## Final product

A ROS 2 stereo perception pipeline that runs on Jetson and outputs **tracked objects with distance** in real time.

### Final system capabilities

* Stereo camera acquisition (left/right)
* Stereo calibration + rectification
* Depth estimation (depth in meters + point cloud where needed)
* GPU-accelerated object detection
* Multi-object tracking with stable IDs
* Tracking + depth fusion: each tracked object includes estimated **distance (meters)**
* **IMU sensing:** accelerometer + gyroscope + magnetometer streams available in ROS 2
* ROS 2 topics + RViz visualization
* Benchmarks: FPS, latency (p50/p95), CPU/RAM usage, depth sanity checks
* Reliability: one-command launch + systemd autostart + restart on failure

### Definition of Done

The project is done when:

1. One command starts the full pipeline (`ros2 launch ...`).
2. RViz shows live stereo, detections, and tracked objects.
3. Tracked objects include a distance estimate in meters.
4. A final benchmark report exists with performance and depth sanity results.
5. The system can auto-start and recover from simple failures (node crash/restart).

---

## Weekly plan

### Week 0 — Baseline and repo foundation

Goal: capture system info (Ubuntu/Jetson Linux/CUDA) + storage benchmark, create clean structure and creating a plan. This one.

### Week 1 — ROS 2 Humble bring-up + workspace

Goal: install ROS 2 Humble, create `ros2_ws`, build a first package, and run a basic node.
Output: reproducible build instructions + a heartbeat node (`/alive`).

**Additions for ZED + Docker:**

* Install/verify **ZED SDK** and confirm the ZED 2i works using **Stereolabs tooling** (basic streaming + depth preview as a sanity check).
* Add `docker/` folder:

  * initial Jetson-focused Dockerfile (ROS 2 Humble base)
  * optional `.devcontainer/` for Dev Containers workflow
* Note whether Week 2+ development will be:

  * (A) native on Jetson, (B) inside Docker on Jetson, or (C) devcontainer from a host PC targeting Jetson (documented choice)

### Week 2 — ZED ROS 2 wrapper bring-up + topic verification

Goal: bring up the **ZED ROS 2 wrapper** and confirm all required streams are publishing.
Output: verified topics list + basic RViz visualization + stability notes.

* Confirm left/right topics (raw + rectified availability) and TF tree consistency.
* Confirm depth outputs (depth + point cloud where needed).
* Confirm **IMU topics** are publishing (accel/gyro/orientation/magnetometer).
* (Optional) create a small **adapter node** that republishes ZED topics into your preferred `/stereo/...` names (only if you really need those names).
* If using Docker: confirm camera + IMU access from inside the container (device pass-through + permissions).

### Week 3 — Stereo sync + dataset capture (Python)

Goal: save synchronized stereo pairs with timestamps and verify frame pairing quality.
Output: sync validation notes and sample dataset captured locally.

* Capture a short dataset using **SVO recording** for repeatable testing (plus an exported stereo-pair dataset if needed).
* Extend dataset capture to include **IMU samples** aligned to image timestamps (including magnetometer).

### Week 4 — Stereo calibration + rectification

Goal: validate calibration/rectification and lock the coordinate frames needed for fusion.
Output: calibration notes committed + rectification demo.

* Verify factory calibration/rectification pipeline via the ZED wrapper (rectified topics + TF consistency).
* Validate and document camera↔IMU transforms for later stability checks and fusion.
* (Optional) Run an OpenCV-based calibration cross-check if you want an independent baseline.

### Week 5 — Depth estimation baseline

Goal: validate depth quality and define the distance rule we will enforce for every tracked object.
Output: depth sanity checks at multiple distances + error notes.

* Use ZED depth outputs as the baseline (depth in meters + point cloud where required).
* Define the project’s **distance rule** (per-object distance):

  * Primary: use ZED 3D object position when available.
  * Fallback: compute a robust statistic from depth inside the bbox (e.g., median/percentile) with invalid-depth handling.
* Record depth sanity checks at multiple distances and document invalid-depth behavior.

### Week 6 — Object detection baseline

Goal: enable object detection and publish detections + overlays.
Output: baseline inference FPS/latency benchmark.

* Enable object detection and subscribe to `~/obj_det/objects`.
* Select detection preset (FAST/MEDIUM/ACCURATE) and confidence thresholds.
* Visualize detections in RViz and save a minimal RViz config for this stage.
* Benchmark: FPS + p50/p95 latency as measured in the pipeline.

### Week 7 — Detection tuning + tracking enablement

Goal: enable tracking and tune performance + stability until IDs are reliable.
Output: benchmark table with p50/p95 latency + stability notes.

* Enable tracking and confirm stable IDs across frames.
* Tune thresholds, filtering, max range, and reduced precision inference (if used).
* Document caching/first-run behavior and how to reproduce consistent results.

### Week 8 — Tracked objects publisher (project contract)

Goal: publish `/tracked_objects` with a stable, clean message contract that will not change later.
Output: tracking stability test notes (ID switches, lost tracks).

* Convert tracked objects into the project output contract:

  * stable ID
  * bbox
  * class/label
  * 3D position (if available) or derived depth stats
  * confidence
* Add lifecycle handling (object lost → timeout → remove).

### Week 9 — Distance fusion + stability rules

Goal: every tracked object includes a stable distance in meters, with robust handling of invalid depth.
Output: distance stability test (jitter, invalid depth handling).

* Compute distance primarily from 3D object outputs; fallback to the Week 5 depth rule.
* Add smoothing rules (low-pass / median over N frames) and handle:

  * invalid depth
  * sudden jumps
  * close-range saturation
* Add IMU “health + alignment” checks:

  * accel/gyro/mag stream sanity
  * timestamp consistency with image streams

### Week 10 — C++ optimization

Goal: rewrite one performance-critical node in C++ (fusion OR tracked objects publisher OR adapter/preprocessing).
Output: measured performance improvement and clean CMake build.

### Week 11 — Launch files + RViz integration

Goal: one `ros2 launch` starts everything; a single RViz config shows the full system.
Output: “one command demo” instructions.

* Keep RViz workload reasonable on Jetson when benchmarking (avoid heavy point cloud + full-res images at once).

### Week 12 — Reliability + deployment

Goal: systemd service, restart-on-failure, structured logs, and a clear recovery workflow.
Output: reboot-to-demo and crash recovery notes.

**Docker option:**

* Provide two service modes (documented):

  * Native: systemd starts `ros2 launch ...`
  * Containerized: systemd starts `docker run ...` (or docker compose) for the full pipeline

### Week 13 — Final benchmarking and failure tests

Goal: produce a final performance report (FPS, latency p50/p95, CPU/RAM, depth accuracy).
Output: `benchmarks/final_report.md`.

Include:

* Depth sanity + per-object distance stability results
* IMU stream sanity (accel/gyro/mag)
* Failure tests: node crash/restart, container restart, reboot-to-demo, and camera re-plug test (if feasible)

### Week 14 — Release packaging

Goal: polish README, architecture diagram, and a final demo.
Output: clear run instructions + results + roadmap for improvements.

Additions:

* Document ZED-specific setup (SDK + wrapper launch + key topics)
* Document Docker workflow (how to build/run dev container + how to run on Jetson)

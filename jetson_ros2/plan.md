# Project Plan and Current Status

## Project direction

The first project idea was a complete stereo perception system for a ZED 2i
camera on a Jetson Orin Nano. That target has not changed: the final system
should publish tracked objects with distance in meters and should include the
camera image, stereo depth, IMU, TF, visualization, benchmarks, and recovery
tests.

The camera was not available during the implementation period. Everything
delivered in the current repository was therefore developed and tested
**without the ZED camera**. Recorded videos replace the camera input, and Depth
Anything V3 Small supplies normalized monocular relative depth instead of
stereo metric depth.

The current result is called the **pre-ZED pipeline**. It proves the ROS 2
architecture and the perception workflow, while leaving hardware-dependent
claims for the camera integration stage.

## Scope summary

| Area | Current result | ZED completion work |
| --- | --- | --- |
| Image source | Local MP4 published as `sensor_msgs/Image` | Use the verified rectified ZED image topic |
| Detection | YOLO11n-640 through the local HTTP inference server | Validate performance with live camera images |
| Tracking | C++ class-aware IoU tracker | Compare or integrate ZED tracking if useful |
| Depth | Depth Anything V3 Small, normalized per frame | Use calibrated ZED stereo depth in meters |
| Fusion | C++ exact-timestamp track and relative-depth fusion | Fuse tracks with synchronized metric depth |
| Output | Temporary JSON on `/tracked_objects` | Review and stabilize the ROS message contract |
| Visualization | Annotated ROS image and recorded MP4 | Add final live-camera RViz views |
| IMU and TF | Not implemented without the camera | Validate IMU topics, transforms, and time alignment |
| Reliability | Finite video workflow with timeout and output validation | Add camera reconnect, restart, and deployment tests |
| Performance | Depth model benchmark and observed integration latency | Run final end-to-end camera benchmarks |

## Current software stack

- NVIDIA Jetson Orin Nano
- Ubuntu 22.04
- Jetson Linux R36.4.3
- CUDA 12.6
- ROS 2 Humble and `colcon`
- Python for video I/O, HTTP inference clients, and visualization
- C++ for tracking and depth-statistic fusion
- Roboflow Inference Server for YOLO11 and Depth Anything V3
- OpenCV for image conversion, drawing, and MP4 output

Docker is used for the local inference service and earlier ROS learning tests,
but the repository is not a Docker-only project. The ROS 2 workspace can be
built and run natively on the Jetson host.

## Implemented pre-ZED workflow

```text
MP4 -> image topic -> YOLO11 -> detections -> C++ tracker -> tracks
  \-> image topic -> Depth Anything V3 -> relative depth -----> C++ fusion
                                                           \-> /tracked_objects
                                                               -> overlay MP4
```

The six nodes and their topic contracts are documented in
[`final_preZED/docs/architecture.md`](final_preZED/docs/architecture.md).
Build and run instructions are in
[`final_preZED/README.md`](final_preZED/README.md).

## Milestone record

### Foundation and ROS 2 learning

**Completed without the camera**

- Captured the Jetson software baseline.
- Created the ROS 2 workspace.
- Built a minimal Python heartbeat publisher.
- Practiced ROS 2 topics, packages, parameters, and services.
- Tested a small Docker-based ROS 2 workflow.

The supporting milestones are summarized in
[`docs/development_history.md`](docs/development_history.md). They are project
history rather than instructions for the final pipeline.

### Image-first perception experiment

**Completed and superseded by the pre-ZED pipeline**

- Loaded local images with OpenCV.
- Implemented color segmentation and a simple counting service.
- Used ROS 2 parameters and standard messages.

This experiment did not perform learned object detection or depth estimation.
It was a learning step before the YOLO and Depth Anything integration.

### Depth model experiment and benchmark

**Completed without the camera**

- Integrated `depth-anything-v3/small` through the local HTTP inference server.
- Processed eight recorded videos.
- Stored per-video FPS and resource measurements.
- Reviewed visual depth quality and documented limitations.
- Measured an average of approximately `0.529 FPS` for that benchmark setup.

The depth is monocular and relative. These measurements do not demonstrate
metric accuracy or real-time ZED performance.

### Detection and tracking

**Implemented without the camera**

- Configured `yolov11n-640` with a `0.60` confidence threshold.
- Added a detector allowlist for people, vehicles, and supported COCO animal
  labels.
- Added a C++ multi-object tracker using class-aware IoU matching.
- Retained tracks for a small number of missed frames and marked whether each
  track was observed in the current frame.

This is a transparent baseline tracker. It does not provide the robustness of
a motion model or appearance-based tracker under difficult occlusion.

### Relative-depth fusion

**Implemented without the camera**

- Preserved source timestamps through detection, tracking, and depth.
- Fused only track and depth records with the same timestamp.
- Calculated the median normalized depth over the center 60% of each bounding
  box while excluding invalid zero pixels.
- Published the result through the temporary `/tracked_objects` JSON contract.

The field is explicitly marked `depth_is_metric: false`. It must not be
renamed or interpreted as distance in meters.

### Visualization and finite processing

**Implemented; full-run validation remains in progress**

- Added an annotated image topic and MP4 writer.
- Added a synchronized relative-depth inset.
- Added a finite mode that processes one source frame at a time.
- Added retries, timestamp deduplication, timeouts, and output-file checks.
- Added a single-video runner and a shortest-first batch runner.

A ten-frame sampled smoke video was reported as complete and decodable. A
later full-frame attempt exposed source-delivery losses and stopped after
seven frames. The source path was changed to reliable ROS 2 QoS after that
failure. A complete full-video and all-video run still needs to be repeated
and recorded before those outputs can be claimed as validated.

### Packaging and documentation

**Implemented for the pre-ZED delivery**

- Added one launch description for all six pre-ZED nodes.
- Centralized runtime parameters in one YAML file.
- Added package tests for HTTP response handling and tracker behavior.
- Added developer documentation, architecture notes, and output conventions.

This packaging covers the camera-less milestone only. It is not the final ZED
release package.

## Remaining ZED stage

The following work cannot be completed honestly without access to the camera:

1. Install and verify the ZED SDK and `zed-ros2-wrapper` versions compatible
   with the Jetson software stack.
2. Inspect the live topic list, message encodings, QoS, timestamps, and TF tree.
3. Select the rectified color image and metric depth topics used by the project.
4. Replace the video source and relative-depth branch without changing the
   detector/tracker boundary unnecessarily.
5. Handle invalid, missing, and out-of-range stereo depth values.
6. Publish object distance in meters only after checking it against known
   physical distances.
7. Validate accelerometer, gyroscope, orientation, and magnetometer streams.
8. Add the final RViz configuration for images, tracks, TF, and optional point
   cloud data.
9. Decide whether ZED object detection/tracking replaces the current detector,
   runs beside it for comparison, or remains disabled.
10. Replace the temporary JSON output with a reviewed ROS message contract.
11. Run final FPS, p50/p95 latency, CPU, GPU, RAM, and metric-depth error tests.
12. Test camera disconnect/reconnect, node failure, restart, and reboot-to-demo.

## Definition of done

### Pre-ZED milestone

The software milestone is ready for demonstration when:

1. The inference server is available locally.
2. One launch path starts the six camera-less ROS 2 nodes.
3. Detection, tracking, relative-depth fusion, and overlay topics are visible.
4. `/tracked_objects` identifies the depth as non-metric.
5. A finite output MP4 passes decode, resolution, FPS, and frame-count checks.
6. The build and package tests pass in the target environment.

Items 1-4 and a sampled ten-frame output were observed during validation on
the target machine. The current reliable-delivery revision still needs a
complete full-video run.

### Final ZED project

The original project is complete only when:

1. The ZED 2i provides live rectified images, synchronized metric depth, IMU,
   and valid TF data.
2. Tracked objects include verified distance in meters.
3. RViz displays the live pipeline.
4. A final report contains measured performance and metric-depth accuracy.
5. Camera and process recovery tests pass.
6. One documented command starts the complete deployed system.

Until these conditions are measured with the camera, the correct project
description is **pre-ZED perception with relative monocular depth**.

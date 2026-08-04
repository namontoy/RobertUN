# Development History

This summary keeps the useful context from the early project notes without
mixing old tutorials with the current run instructions.

## ROS 2 heartbeat

The first workspace milestone was the Python `alive_py` package. Its node
published `std_msgs/msg/Bool` on `/alive`, confirming that ROS 2 Humble,
`colcon`, package discovery, publishers, and subscribers worked on the project
machine.

This was infrastructure practice, not perception work.

## Docker learning test

A minimal ROS 2 image built the heartbeat package inside Docker. An interactive
helper was then added to list images and containers, build an image, run a
container, mount source code, and open a second shell.

That experiment established basic Docker concepts. It did not validate GPU
inference, camera device access, or the ZED SDK. Docker remains optional for the
ROS 2 workspace; the current local inference server is the main containerized
component.

## Image-first color experiment

Before learned detection was integrated, `image_tools_py` loaded a traffic
image and segmented red, white, or black regions in HSV color space. It
published a count and summary and later exposed the operation through a ROS 2
service contract.

This exercise introduced OpenCV conversion, parameters, topics, services, and
interface packages. It did not detect semantic objects or estimate depth and
was superseded by the YOLO11 pre-ZED pipeline.

## Depth Anything experiment

The first depth benchmark used recorded MP4 files and the local Roboflow
Inference Server. Early attempts exposed route mismatches, model-loading
failures, CUDA memory pressure, and container-name confusion. Server logs were
more useful than client exceptions when separating HTTP problems from model
runtime failures.

After stabilizing the server, the benchmark processed eight videos and produced
the tracked timing tables and visual review under
`benchmarks/depth_anything_v3_small/`. The result justified using Depth
Anything V3 Small for qualitative pre-ZED depth, while its approximately
`0.5 FPS` throughput ruled out a real-time claim.

## Integrated pre-ZED pipeline

The next milestone replaced the color exercise with YOLO11 detection, relative
depth, C++ tracking, C++ fusion, and recorded overlays. Recorded videos still
replace the original ZED source, so the output is explicitly non-metric.

Current architecture, operation, and validation status are documented in:

- `final_preZED/README.md`
- `final_preZED/docs/architecture.md`
- `final_preZED/docs/validation.md`
- `plan.md`

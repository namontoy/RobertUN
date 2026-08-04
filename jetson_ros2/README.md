# Jetson ROS 2 Perception Project

This repository started as a plan for a ZED 2i stereo perception system on a
Jetson Orin Nano. The intended final system uses the camera for live images,
stereo depth in meters, IMU data, TF, object detection, and tracking.

The ZED camera was not available during the implementation documented here.
For that reason, the current working system is a **pre-ZED pipeline** built
entirely without the camera. It uses recorded MP4 videos as input, YOLO11 for
object detection, Depth Anything V3 Small for relative monocular depth, and
ROS 2 nodes written in Python and C++.

This distinction matters: the repository currently demonstrates the complete
software flow from an image to a tracked object with relative depth, but it
does not measure distance in meters and it does not validate ZED hardware.

## Current pipeline

```text
recorded MP4
    -> ROS 2 image publisher
    -> YOLO11 detections
    -> C++ multi-object tracker
    -> C++ track/depth fusion
    -> /tracked_objects
    -> annotated ROS image and MP4

recorded MP4
    -> Depth Anything V3 Small
    -> normalized relative-depth image
    -> C++ track/depth fusion
```

Six ROS 2 nodes make up the pipeline:

| Node | Language | Main responsibility |
| --- | --- | --- |
| `video_file_publisher` | Python | Reads an MP4 and publishes ROS images |
| `depth_anything_http_node` | Python | Requests relative depth from the local inference server |
| `yolo_http_detector_node` | Python | Requests YOLO11 detections and applies class/confidence filters |
| `simple_tracker_node` | C++ | Assigns and maintains track IDs with class-aware IoU matching |
| `track_depth_fusion_node` | C++ | Samples relative depth inside each tracked bounding box |
| `pre_zed_overlay_node` | Python | Synchronizes results, draws annotations, and records an MP4 |

ROS 2 is the integration layer between the two languages. The nodes exchange
typed ROS messages over topics; ROS 2 serialization handles transport, but it
does not translate Python or C++ source code.

## What the depth value means

The current `/tracked_objects` contract contains:

```json
"depth_source": "depth_anything_v3_relative",
"depth_is_metric": false
```

`relative_depth_median` is a normalized value calculated from the center of a
tracked bounding box. It is useful for qualitative ordering within an image,
such as deciding which visible object appears closer. It is not a physical
distance and must never be reported as meters.

Metric distance is deferred until the ZED camera can provide calibrated stereo
depth and the result can be checked against measured distances.

## Where to start

- [`plan.md`](plan.md) explains the original ZED goal, the implemented pre-ZED
  scope, and the remaining camera work.
- [`final_preZED/README.md`](final_preZED/README.md) contains build, run,
  validation, and batch-processing instructions.
- [`final_preZED/docs/architecture.md`](final_preZED/docs/architecture.md)
  documents nodes, topics, synchronization, tracking, and fusion.
- [`final_preZED/docs/validation.md`](final_preZED/docs/validation.md)
  records implementation status, measured evidence, and pending checks.
- [`benchmarks/depth_anything_v3_small/`](benchmarks/depth_anything_v3_small/)
  contains measured Depth Anything benchmark results.
- [`docs/development_history.md`](docs/development_history.md) summarizes the
  learning milestones that preceded the current pipeline.

Operational network and shell references remain under `troubleshooting/`.

## Implemented without the ZED

- Recorded-video ROS 2 image source
- YOLO11n-640 HTTP detection with a `0.60` confidence threshold
- Detection filtering for people, vehicles, and supported COCO animals
- Class-aware C++ IoU tracking with stable IDs under simple motion
- Exact-timestamp fusion of tracks and normalized monocular depth
- Temporary `/tracked_objects` JSON contract
- Relative-depth visualization and annotated MP4 output
- Finite one-video processing and an all-video batch runner
- Python unit tests and C++ tracker tests
- Depth Anything V3 Small benchmark over eight local videos

## Still required for the ZED system

- Install and validate the ZED SDK and `zed-ros2-wrapper` on the target system
- Replace the MP4 publisher with the verified rectified ZED image topic
- Replace monocular relative depth with synchronized ZED metric depth
- Validate depth units, timestamps, QoS, calibration, and optical frame IDs
- Publish and validate IMU, TF, and optional point-cloud data
- Measure object distance against known distances in the scene
- Review the temporary JSON contract and replace it with a stable ROS message
- Add live-camera RViz, disconnect/reconnect, recovery, and deployment tests
- Measure final end-to-end FPS, latency, memory use, and metric-depth error

The current implementation is therefore the software integration milestone
before camera bring-up, not the final ZED validation.

# Pre-ZED Architecture

## Scope

The original project targets a ZED 2i, but the current implementation was
built without the camera. A recorded MP4 supplies images, and Depth Anything
V3 supplies relative monocular depth. This validates the software interfaces
around detection, tracking, fusion, and visualization before camera bring-up.

## Packages and languages

| Package | Language | Responsibility |
| --- | --- | --- |
| `pre_zed_perception_py` | Python | Video decoding, HTTP inference, image conversion, overlay drawing, and MP4 recording |
| `pre_zed_tracking_cpp` | C++ | Track management, timestamp matching, and relative-depth statistics |

Python is used around changing model and image I/O APIs. C++ is used for the
stateful tracker and direct depth-pixel sampling. This division is an
architectural choice; it is not a measured Python-versus-C++ performance claim.

ROS 2 connects both languages through topic and message contracts. Python uses
`rclpy`, C++ uses `rclcpp`, and DDS transports serialized ROS messages. ROS 2
does not translate either language's source code.

## Nodes and topics

| Node | Input | Output |
| --- | --- | --- |
| `video_file_publisher` | Local MP4 | `/prezed/image_raw` |
| `depth_anything_http_node` | Source image | `/prezed/depth/relative_image`, `/prezed/depth/visualization` |
| `yolo_http_detector_node` | Source image | `/prezed/detections` |
| `simple_tracker_node` | Detections | `/prezed/tracks` |
| `track_depth_fusion_node` | Tracks and relative depth | `/tracked_objects` |
| `pre_zed_overlay_node` | Image, relative depth, and fused tracks | `/prezed/overlay` and an MP4 |

Images use `sensor_msgs/msg/Image`. Detection, tracking, and fused-object
records currently use versioned JSON inside `std_msgs/msg/String`. The JSON
contract is temporary until the real camera fields and coordinate frames are
known.

## Frame processing

### Video source

OpenCV reads an MP4 and publishes each selected frame as `bgr8`. A source
timestamp is preserved through every downstream result. Looping mode supports
topic inspection; finite mode keeps one logical frame in flight and waits for
the matching overlay before continuing.

### YOLO11 detection

The detector sends a JPEG copy of the image to the local Roboflow Inference
Server. It publishes the class, confidence, bounding box, image dimensions,
model ID, inference time, and source timestamp.

Detections below `0.60` confidence are removed. The allowlist keeps people,
road vehicles, and supported COCO animals. COCO does not contain exact
`humanoid`, `lion`, or `buffalo` classes, so those names require a custom model
for reliable classification.

### Relative-depth estimation

Depth Anything V3 Small processes the same source image. It publishes a
single-channel `mono8` image for fusion and a colorized image for inspection.
The depth is normalized independently per frame. It describes qualitative
ordering, not calibrated distance in meters.

### C++ tracking

The tracker compares existing tracks and new detections of the same class
using intersection over union. Candidate pairs are processed from highest to
lowest IoU.

Current behavior:

- minimum matching IoU: `0.3`
- matched detections keep the existing track ID
- unmatched detections create new tracks
- missing tracks remain temporarily with `observed: false`
- tracks expire after more than three missed frames

This baseline has no motion model or appearance embedding, so fast movement
and occlusion can still produce ID switches.

### C++ depth fusion

The fusion node pairs tracks and depth only when their source timestamps match
exactly. For each bounding box it:

1. uses the center 60% of the box
2. clips the region to the depth image
3. removes zero-valued invalid pixels
4. requires at least 20 samples
5. calculates the median
6. maps the valid `mono8` range to a value from 0 to 1

The inner region reduces background contamination, and the median reduces the
effect of isolated bad pixels. The result remains non-metric.

### Overlay and MP4 output

The overlay caches source images, fused objects, and relative depth by
timestamp. It draws only when all three inputs match. Visible annotations
include track ID, class, confidence, relative depth, and a synchronized depth
inset. Unobserved retained tracks remain in the ROS output but are hidden from
the video by default.

## Finite processing and retries

The required image path uses reliable QoS with queue depth 1. Finite mode waits
for the detector, depth, and overlay subscribers before publishing. The overlay
topic acts as a completion acknowledgement.

When an acknowledgement is late, the source republishes the same timestamped
frame. Nodes remember completed timestamps so a retry does not intentionally
advance tracking or write an output frame twice. A timeout aborts the run
instead of silently accepting a short video.

When the video source exits normally, the launch description shuts down the
other nodes and the overlay releases its OpenCV writer, finalizing the MP4.

## `/tracked_objects` contract

The temporary payload contains frame metadata and a list of tracks. Important
depth fields are:

```json
{
  "schema_version": "0.1.0",
  "contract_status": "temporary_pre_zed",
  "depth_source": "depth_anything_v3_relative",
  "depth_encoding": "per_frame_minmax_mono8",
  "depth_is_metric": false,
  "tracks": [
    {
      "track_id": 1,
      "label": "car",
      "confidence": 0.91,
      "bbox_xywh": [220.0, 160.0, 180.0, 120.0],
      "observed": true,
      "relative_depth_valid": true,
      "relative_depth_median": 0.42,
      "relative_depth_samples": 3200
    }
  ]
}
```

`depth_is_metric: false` prevents the current result from being presented as
physical distance.

## Playback versus throughput

Output FPS controls playback timing. A completed 30 FPS MP4 can play at normal
speed even when each frame took several seconds to generate. Processing speed
must be measured from node timings or benchmark output, not from video playback.

## ZED migration

1. Bring up `zed-ros2-wrapper` and record the actual topics, message types,
   encodings, QoS, timestamps, and TF frames.
2. Replace the MP4 source with the selected rectified ZED image topic.
3. Replace relative depth with synchronized ZED depth in documented metric
   units.
4. Reject invalid stereo depth and calculate a robust object distance.
5. Validate distance against known physical measurements.
6. Add IMU, TF, optional point-cloud, and RViz checks.
7. Decide whether ZED detection and tracking replaces or is compared with the
   current YOLO and IoU path.
8. Replace the temporary JSON with a reviewed ROS interface.
9. Run live performance, camera recovery, and deployment tests.

The normalized relative value must never be renamed to meters. Metric output
begins only after calibrated stereo data and physical validation are available.

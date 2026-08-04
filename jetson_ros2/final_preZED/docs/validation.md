# Pre-ZED Validation Status

## Implemented components

| Component | Current result |
| --- | --- |
| Video source | Timestamped `bgr8` ROS images from MP4 |
| Depth client | Relative `mono8` depth and color visualization |
| Detector | `yolov11n-640`, `0.60` confidence threshold, class allowlist |
| Tracker | C++ class-aware IoU matching and missed-frame expiry |
| Fusion | C++ exact-timestamp median relative depth inside each box |
| Contract | Temporary JSON on `/tracked_objects` with a non-metric flag |
| Overlay | IDs, labels, confidence, relative depth, inset, and MP4 output |
| Execution | Shared YAML, one launch file, single-video and batch runners |

No component in this table uses the ZED camera.

## Reported runtime evidence

The following results came from commands run on the target environment:

- Both ROS 2 packages built successfully with `colcon`.
- Python package tests reported 7 passing tests.
- C++ tracker and lint tests reported no failures.
- All expected pipeline topics were discovered.
- Detections, tracks, and `/tracked_objects` published timestamped records.
- A sampled ten-frame overlay passed decode, frame-count, resolution, and FPS
  checks.

## Depth Anything benchmark

The separate camera-less benchmark processed eight local MP4 videos with
`depth-anything-v3/small` through the local HTTP inference server.

- Average FPS across videos: `0.529`
- Approximate mean of per-video inference averages: `1862 ms/frame`
- Average per-video FPS range: approximately `0.516` to `0.542`

Exact timing is in
`benchmarks/depth_anything_v3_small/fps_table.csv`; resource samples are in
`gpu_memory_table.csv`; visual review is in `quality_notes.md`.

The benchmark supports qualitative pre-ZED visualization. It is not real time
and does not measure metric depth accuracy.

Reported integrated runs observed approximately 2.4-2.7 seconds for many depth
requests. The looping configuration therefore uses a conservative `0.3 Hz`
source rate.

## Pending full-video validation

A complete 285-frame attempt stopped after seven output frames because a source
frame did not reach every inference branch. The required image path was changed
from best-effort to reliable QoS after that failure. That revision still needs
a complete runtime check.

Before claiming the complete or batch workflow as validated:

1. rebuild both packages
2. run both package test suites
3. process `humanoidrobot.mp4` with `frame_stride=1`
4. confirm the runner reports the expected frame count, resolution, and FPS
5. inspect boxes, track IDs, relative-depth labels, and the depth inset
6. run the eight-video batch only after the single-video result passes

## Acceptance boundary

The pre-ZED milestone demonstrates software integration when the six nodes
exchange the documented topics and a finite complete video passes validation.
It does not establish real-time operation, metric distance, wildlife label
accuracy, or ZED hardware behavior.

The original project remains incomplete until live ZED image, stereo depth,
IMU, TF, metric-distance checks, final performance measurements, and camera
recovery tests are complete.

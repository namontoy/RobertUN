# Depth Anything V3 Small Benchmark Summary

## Evidence status

This is a measured camera-less benchmark. It evaluates the relative-depth
model on recorded videos; it is not a ZED benchmark and it does not measure
distance accuracy in meters.

- Model ID: `depth-anything-v3/small`
- Videos processed: 8
- Average FPS across videos: `0.529`
- Average per-video inference time: approximately `1862 ms/frame`
- Decision: continue using the model for the pre-ZED demonstration

## Interpretation

The visual results were useful enough for qualitative depth inspection and
track/depth integration. Throughput needs substantial improvement before this
HTTP configuration can be described as real time.

Depth output is relative monocular depth normalized per frame. It is not
metric distance and cannot be reported in meters.

The complete per-video measurements are in `fps_table.csv`; qualitative review
is in `quality_notes.md`.

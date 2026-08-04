# Depth Anything V3 Small Video Benchmark

> **Document status:** This is the reproducible model experiment that preceded
> the ROS 2 pre-ZED pipeline. It used recorded MP4 files and no ZED camera. The
> measured results are in `benchmarks/depth_anything_v3_small/`; current ROS 2
> integration instructions are in `final_preZED/README.md`.

This experiment benchmarks local inference for `depth-anything-v3/small` on
MP4 videos.

The goal was to decide whether the model was useful enough to continue toward
ROS 2 integration. That integration was later implemented in the pre-ZED
workflow, while the model remained explicitly non-metric.

## Folders

Input videos:

```text
datasets/depth_anything_v3_small/input_videos/
```

Generated depth visualizations:

```text
datasets/depth_anything_v3_small/output_videos/
```

Tracked benchmark reports:

```text
benchmarks/depth_anything_v3_small/
```

`datasets/` is ignored by Git, so large MP4 files and generated videos stay local.

## Run

First make sure the Roboflow Inference Server is running:

```bash
curl http://localhost:9001/docs
```

Install the lightweight HTTP dependency in your active virtualenv:

```bash
python -m pip install requests
```

From the repo root:

```bash
bash experiments/depth_anything_v3_small/run_benchmark.sh
```

If the server needs Roboflow credentials to download the model, provide an API key:

```bash
export ROBOFLOW_API_KEY="your_api_key_here"
bash experiments/depth_anything_v3_small/run_benchmark.sh
```

Or pass it for one run:

```bash
bash experiments/depth_anything_v3_small/run_benchmark.sh --api-key "your_api_key_here"
```

The script expects 5-10 `.mp4` files in:

```text
datasets/depth_anything_v3_small/input_videos/
```

For a quick smoke test, one short `.mp4` is enough.

## Runtime Assumption

The default path calls the local Roboflow Inference Server directly:

```text
POST /infer/depth-estimation/{model_id}
```

The server should be available at:

```text
http://localhost:9001
```

The benchmark calls:

```text
POST /infer/depth-estimation
```

and sends:

```text
model_id=depth-anything-v3/small
depth_version_id=small
```

Native Python runtime is still available explicitly:

```bash
bash experiments/depth_anything_v3_small/run_benchmark.sh --backend native
```

The native path expects:

```python
from inference import get_model
```

## Outputs

The benchmark writes:

- depth visualization MP4s
- `fps_table.csv`
- `gpu_memory_table.csv`
- `quality_notes.md`
- `summary.md`

Depth Anything output is treated as relative monocular depth. Do not interpret it as metric distance in meters.

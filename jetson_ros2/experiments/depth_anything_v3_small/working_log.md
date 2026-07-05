# Depth Anything V3 Small Working Log

This note records the path from the first failed benchmark runs to the current working state.

## Current Status

The benchmark is now reaching the Roboflow Inference Server correctly.

The server logs show `POST` requests returning `200 OK`, so the HTTP path, the container, and the model call are working together at the moment.

## What We Built

We created a local video benchmark for `depth-anything-v3/small` with:

- `experiments/depth_anything_v3_small/run_video_benchmark.py`
- `experiments/depth_anything_v3_small/run_benchmark.sh`
- input videos under `datasets/depth_anything_v3_small/input_videos/`
- output depth videos under `datasets/depth_anything_v3_small/output_videos/`
- reports under `benchmarks/depth_anything_v3_small/`

The benchmark sends each video frame to the local Roboflow server and saves:

- depth visualization video
- FPS data
- GPU memory data
- quality notes

## What We Tried

### 1. First HTTP approach

The first implementation tried to use the Roboflow Python SDK in a generic way.

That failed because the local server did not accept the model-loading path the SDK expected.

### 2. Direct depth endpoint

We then switched to the server endpoint for depth estimation directly.

That was the right direction, because the server exposes:

```text
/infer/depth-estimation
```

and the benchmark now talks to that endpoint explicitly.

### 3. API key support

After exporting the Roboflow API key, the server got farther into model loading.

That changed the failure mode from request handling to model runtime loading.

## Problems We Hit

### `404 Not Found`

This happened when the benchmark or server path did not match the model-loading route that the server expected.

Meaning:

- the request reached the server
- but the server could not resolve the model in the way the code was calling it

### `ModelNotFoundError`

This happened while the server was trying to fetch model weights.

The important meaning was:

- the request format was closer to correct
- but the server still could not load the model package yet

### `Connection reset by peer`

This happened when the client lost the HTTP connection while the server was still trying to initialize the model.

In practice, this usually means the server side crashed, restarted, or aborted during model loading.

### `NVML_SUCCESS == r INTERNAL ASSERT FAILED`

This was the most important runtime failure.

It pointed to a PyTorch/CUDA allocator problem inside the container:

- `CUDACachingAllocator.cpp`
- `NvMapMemAllocInternalTagged`
- error code `12`

On Jetson, that usually means the GPU memory allocation path failed or the memory was too fragmented.

### Memory pressure and fragmentation

Before the reboot, the system showed low contiguous free memory.

That matters because Jetson workloads often fail even when total RAM looks acceptable if the largest free block is too small.

### Container name mismatch

At one point the running container was named `inference-server-v2`, not `inference-server`.

That caused `docker logs inference-server` to fail even though the server itself was running.

## Why Reboot Helped

The reboot cleared memory fragmentation and gave the Jetson a cleaner starting point.

After that, the container could stay up and the requests started returning `OK` in the logs.

This does not prove the model is lightweight. It only proves the runtime is currently stable enough to serve requests.

## Why The Logs Matter

The benchmark itself only shows whether the HTTP request succeeded or failed.

The server logs are where the real cause appears:

- request route
- model loading
- CUDA allocation
- container-level failures

That is why the logs were the main source of truth during debugging.

## How The Workflow Now Works

1. Start the Roboflow Inference Server container.
2. Confirm it is listening on `http://localhost:9001`.
3. Run the benchmark script from the repo root.
4. The script sends video frames to the server.
5. The server loads the depth model and returns predictions.
6. The script writes depth videos and benchmark reports.

## Lessons Learned

- The benchmark code can be correct while the model runtime still fails.
- On Jetson, memory fragmentation can matter as much as total free RAM.
- Docker container names and image names are different things.
- The Roboflow API key was necessary, but it was not the final fix.
- Server logs are more useful than client stack traces for this class of failure.

## Next Logical Steps

1. Re-run the benchmark on a clean boot and confirm the `OK` logs stay stable.
2. Keep a short run first, then scale to more videos.
3. Record FPS, memory usage, and output quality in the benchmark tables.
4. If the model becomes unstable again, capture the exact server log before changing the code.

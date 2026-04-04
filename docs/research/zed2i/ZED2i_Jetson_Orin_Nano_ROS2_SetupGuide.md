# ZED 2i on Jetson Orin Nano with ROS 2: the definitive setup guide

**The most common cause of the ZED ROS 2 wrapper hanging on first launch is a blocked network download, not TensorRT compilation.** The ZED SDK attempts to download AI model files and camera calibration data from Stereolabs servers during `sl::Camera::open()`, and if a firewall or misconfigured network silently drops packets, the `component_container_isolated` process will appear frozen with zero CPU and zero GPU usage indefinitely. The fix involves pre-optimizing AI models offline, switching to a non-neural depth mode for initial testing, and verifying network connectivity before launch. This document covers every aspect of getting the ZED 2i working reliably on Jetson Orin Nano with JetPack 6.x and the zed-ros2-wrapper v5.2.2.

---

## 1. Repository structure and what you actually need to clone

The ZED ROS 2 ecosystem as of 2025–2026 centers on a single **monorepo** plus two optional companion repositories. A common source of confusion is whether `zed-ros2-interfaces` and `zed-ros2-description` must be cloned separately — they do not.

The primary repository, `github.com/stereolabs/zed-ros2-wrapper` (branch `master`), is a monorepo containing four colcon packages: **`zed_wrapper`** (launch files and YAML configs), **`zed_components`** (the core C++ camera driver node, version 5.2.2), **`zed_description`** (URDF models and meshes for all ZED camera models), and **`zed_debug`** (internal development package that should be skipped during build with `--packages-skip zed_debug`). The `zed-ros2-interfaces` submodule was removed from the wrapper repository. For ROS 2 Humble, the `zed_msgs` package is now installed via apt: `sudo apt install ros-humble-zed-msgs`. For other ROS 2 distributions, clone `github.com/stereolabs/zed-ros2-interfaces` separately.

The optional `github.com/stereolabs/zed-ros2-examples` repository contains RViz2 display configurations, object detection visualization plugins, and tutorial nodes. It is not required for basic operation. The `point_cloud_transport` package was removed as a required dependency but is auto-enabled if detected at build time — install it manually if needed: `sudo apt install ros-humble-point-cloud-transport ros-humble-point-cloud-transport-plugins`.

**Bottom line: for a minimal working setup, clone only `zed-ros2-wrapper` and install `ros-humble-zed-msgs` via apt.** No separate interfaces, description, or messages repos are needed.

---

## 2. Complete installation procedure for JetPack 6.x

### Install the ZED SDK

Download the correct installer from `stereolabs.com/developers/release`, matching your L4T version precisely. **This version match is the single most important compatibility factor.** For JetPack 6.2 (L4T R36.4), use the SDK labeled "JetPack 6.1 and 6.2." For L4T R36.5 (JetPack 6.2.2), you must use **ZED SDK 5.2.2 or later**, which added L4T R36.5 support — earlier versions will fail silently or produce CUDA mismatches.

```bash
chmod +x ZED_SDK_Tegra_L4T36.X_v5.2.X.zstd.run
./ZED_SDK_Tegra_L4T36.X_v5.2.X.zstd.run
```

For headless/automated installs, append `-- silent` (full), `-- silent skip_od_module` (no AI models), or `-- silent runtime_only` (no headers/tools). The SDK installs to `/usr/local/zed/`. Verify with `/usr/local/zed/tools/ZED_Explorer`.

### Install ROS 2 Humble and build the wrapper

JetPack 6.x ships Ubuntu 22.04, which is Humble's target platform. After installing ROS 2 Humble via the standard Debian method:

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/stereolabs/zed-ros2-wrapper.git
cd ~/ros2_ws
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release \
  --packages-skip zed_debug --parallel-workers $(nproc)
source install/local_setup.bash
```

### The JetPack 6 CMake fix

On JetPack 6, the build frequently fails with `CMake Error: Specify CUDA_TOOLKIT_ROOT_DIR`. **The fix is `sudo apt install nvidia-jetpack nvidia-jetpack-dev`**, then clean and rebuild. This is documented in the official README but easily missed.

---

## 3. Why the node hangs on first launch — and exactly how to fix it

The symptom is unmistakable: `zed_camera.launch.py` starts, the URDF loads, `robot_state_publisher` begins publishing, but then `component_container_isolated` sits at state `Sl+` with **zero CPU usage and zero GPU usage (GR3D_FREQ 0%)** for 10+ minutes. Nothing is published on any ZED topic.

### The root cause: blocked network I/O

The ZED SDK makes HTTP requests during `sl::Camera::open()` in two situations: **(1)** downloading AI model files from `download.stereolabs.com` if the requested model is not cached locally, and **(2)** downloading the camera's factory calibration file from Stereolabs servers using the camera's serial number. Both requests happen synchronously inside the composable node container, blocking the entire process.

**Zero GPU usage rules out TensorRT optimization** — active TensorRT engine building would show significant GR3D activity. Zero CPU usage indicates the process is blocked on a socket system call, most likely a TCP connect or read timeout. If a firewall silently drops packets (no RST, no ICMP unreachable), the TCP timeout can extend to **2–5 minutes per attempt**, and the SDK may retry multiple times.

A community report from June 2025 captures the exact failure sequence when the download eventually times out:
```
[ZED][INFO] AI model not found, downloading
[ZED][ERROR] File downloading failed, NETWORK_FAILED: The file couldn't be downloaded.
[ZED][ERROR] [Depth] NEURAL CORRUPTED MODEL
[ZED][WARNING] CORRUPTED SDK INSTALLATION
```

### The three-step fix

**Step 1: Verify network access.** Test connectivity to Stereolabs download servers:
```bash
curl -v http://download.stereolabs.com/ai/model/neural_depth_5.2.model --head
```
If this hangs, the network is the problem. Either provide internet access or proceed to Step 2.

**Step 2: Pre-optimize AI models before launching ROS.** The `ZED_Diagnostic` tool at `/usr/local/zed/tools/` can download and optimize all required models:
```bash
/usr/local/zed/tools/ZED_Diagnostic -nrlo_all   # All NEURAL depth modes
/usr/local/zed/tools/ZED_Diagnostic -aio          # ALL AI models (depth + OD + body tracking)
```
On Orin Nano, TensorRT optimization takes **30–60+ minutes** per model. During this time, GPU usage will be high — this is expected. Let it complete fully before launching the ROS node.

**Step 3: Switch to a non-neural depth mode for initial testing.** The default `depth_mode` in `common_stereo.yaml` is `'NEURAL_LIGHT'`, which requires an AI model. Change it to bypass AI entirely:
```yaml
depth:
    depth_mode: 'PERFORMANCE'  # Classical stereo, no AI download needed
```
Or for absolute minimal testing: `depth_mode: 'QUALITY'` (pure classical, guaranteed no network or TensorRT dependency).

### Additional network considerations

The SDK also downloads a calibration file on first camera open. Pre-download it manually:
```bash
# Find your camera serial number (printed on camera label or via ZED_Explorer -l)
wget "http://calib.stereolabs.com/?SN=YOUR_SERIAL" -O /usr/local/zed/settings/SNYOUR_SERIAL.conf
```

Enable verbose SDK logging to see exactly what the SDK is doing during initialization:
```bash
export ZED_SDK_VERBOSE=1
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

---

## 4. AI model optimization and caching explained

### Where models live

All AI model files and TensorRT engines are stored in **`/usr/local/zed/resources/`**. The `.model` files are the raw neural network weights downloaded from Stereolabs servers. The `.engine` files are TensorRT-compiled binaries specific to the exact GPU architecture and TensorRT version. Camera calibration files are stored separately in `/usr/local/zed/settings/`.

Since SDK 5.0.1, the base NEURAL depth model files are **bundled with the SDK installer** to avoid runtime downloads. However, TensorRT engine building is always required on first use because engines are GPU-specific. Additionally, object detection and body tracking models may still require downloads if those features are enabled.

### Model download URLs

Models are downloaded from `http://download.stereolabs.com/ai/model/<filename>`. Key models include `neural_depth_5.2.model`, `neural_depth_light_5.2.model` (for NEURAL_LIGHT), and various object detection models like `objects_performance_3.2.model`. Stereolabs engineer `adujardin` published a Python script for batch downloading all models at `https://gist.github.com/adujardin/9f7d82edc6e0246c87b971ce4b009aae`.

### Programmatic pre-optimization (SDK 5.2+)

The Python API provides direct control over model management:
```python
import pyzed.sl as sl
status = sl.check_ai_model_status(sl.AI_MODEL.NEURAL_DEPTH)
print("Optimized:", status.optimized)
sl.download_ai_model(sl.AI_MODEL.NEURAL_DEPTH)
sl.optimize_ai_model(sl.AI_MODEL.NEURAL_DEPTH)
```

### SDK 5.2.2 improvement

**Neural depth model optimization now runs in an isolated subprocess**, preventing TensorRT-related SIGSEGV crashes from taking down the main application. This was a significant stability fix — earlier versions could crash during optimization with no recovery.

---

## 5. Known issues with ZED SDK 5.x and JetPack 6.x

### L4T R36.5 support gap (critical, now resolved)

ZED SDK versions 5.2.0 and 5.2.1 were built for L4T R36.4 and **did not support L4T R36.5** (JetPack 6.2.2). Users on R36.5 experienced silent failures or CUDA library mismatches. **SDK 5.2.2 added L4T R36.5 support** — this is the minimum version for JetPack 6.2.2 systems.

### Component container crash from argument ordering (GitHub #303)

A bug in certain wrapper commits caused `component_container_isolated` to crash immediately with `UnknownROSArgsError: found unknown ROS arguments: '--use_multi_threaded_executor'`. The `--use_multi_threaded_executor` flag was placed after `--ros-args` in the launch file, causing ROS 2 to misparse it. This was fixed in later commits by reordering the arguments. If you encounter this, update to the latest wrapper version.

### Missing hardware video encoder on Orin Nano

The Orin Nano **lacks NVENC hardware** (no `/dev/nvhost-msenc`). SDK versions before 5.2 would crash when attempting SVO recording or video streaming. SDK 5.2 added H.264 software (CPU) encoding as a fallback, controlled by the `ZED_SDK_H265_FALLBACK_MODE` environment variable.

### USB 3.0 bandwidth and detection issues

The Jetson Orin Nano has a shared USB hub internally, and all USB 3.0 ports share bandwidth through a single controller. ZED cameras require substantial bandwidth (~1.2 Gbps for HD1080@30fps stereo). Common symptoms include camera detected at USB 2.0 speed, camera preventing boot when plugged in at power-on, and dropped frames with long cables. **Always verify USB 3.0 speed** with `lsusb -t` or by checking `/sys/bus/usb/devices/*/speed` (should show `5000`, not `480`).

### TensorRT optimization on resource-constrained Orin Nano

TensorRT engine building is extremely slow on the Orin Nano's **1024 CUDA cores with 8 GB shared memory**. Reports indicate 30 minutes to 2+ hours for a single model. The process can appear stuck at specific percentages. Increasing swap space to at least 8 GB helps prevent OOM kills during optimization: `sudo fallocate -l 8G /swapfile && sudo mkswap /swapfile && sudo swapon /swapfile`.

### The mttcan driver does not interfere with ZED

The `mttcan` (Tegra MTTCAN CAN bus) kernel module is **blacklisted by default** on Jetson Orin and operates on a completely separate hardware bus from USB or GMSL. No documented interference with ZED cameras exists. If loaded and CAN errors occur, excessive kernel logging could theoretically impact performance, but this is unrelated to ZED operation.

---

## 6. NITROS is optional and should be left uninstalled for standard setups

**NVIDIA Isaac Transport for ROS (NITROS)** implements zero-copy GPU-accelerated data transfer between ROS 2 nodes using type adaptation (REP-2007) and type negotiation (REP-2009). Instead of copying image data from GPU to CPU and back, NITROS passes GPU memory handles directly between composable nodes.

The zed-ros2-wrapper's CMakeLists.txt includes `isaac_ros_common`, `isaac_ros_nitros`, `isaac_ros_managed_nitros`, and `isaac_ros_nitros_image_type` as **optional dependencies** that are auto-detected at build time. If these packages are absent, the wrapper compiles and functions normally with standard ROS 2 message passing. Stereolabs explicitly states: *"If the `isaac_ros_nitros` package is not detected during the build, the ZED ROS 2 Wrapper will still compile and function normally."*

### The NITROS warning explained

The warning `"Invalid configuration: enable_ipc:=true with debug.disable_nitros:=false can cause NITROS startup failure"` appears because NITROS requires ROS 2 IPC to be disabled (`enable_ipc:=false`) to function — both systems attempt to manage shared memory, creating a conflict. **If Isaac ROS packages are not installed, this warning is entirely benign.** NITROS cannot activate regardless of parameter settings.

To explicitly silence the warning or disable NITROS if packages are accidentally present, set `debug.disable_nitros: true` in `common_stereo.yaml`, or launch with `enable_ipc:=false`. Stereolabs does not recommend installing Isaac ROS on Jetson via apt due to complex dependencies, and the wrapper is only compatible with Isaac ROS 3.2.x specifically.

---

## 7. Configuration parameters that control initialization behavior

### The depth_mode parameter determines first-launch timing

The most impactful parameter for first-launch behavior is `depth.depth_mode` in `common_stereo.yaml`. The default is **`'NEURAL_LIGHT'`**, which triggers AI model download and TensorRT optimization on first use. Available options ranked by initialization complexity:

| Mode | Type | AI Download | TensorRT Build | First-launch time |
|------|------|-------------|----------------|-------------------|
| `NONE` | No depth | No | No | Instant |
| `QUALITY` | Classical stereo | No | No | Seconds |
| `ULTRA` | Classical stereo | No | No | Seconds |
| `PERFORMANCE` | Classical + neural enhancement | Possibly (SDK 5.x) | Possibly | Seconds to minutes |
| `NEURAL_LIGHT` | Lightweight AI | Yes | Yes | **30–60+ min** (Orin Nano) |
| `NEURAL` | Full AI | Yes | Yes | **30–60+ min** |
| `NEURAL_PLUS` | Maximum AI | Yes | Yes (longest) | **60+ min** |

**For first-launch testing, use `depth_mode: 'PERFORMANCE'` or `depth_mode: 'QUALITY'`** to eliminate AI dependencies entirely. Switch to neural modes only after pre-optimizing models with `ZED_Diagnostic`.

### AI features are disabled by default

Both `object_detection.od_enabled` and `body_tracking.bt_enabled` default to `false` — no changes needed to prevent those models from loading. They can be enabled later via ROS 2 services (`enable_obj_det`, `enable_body_trk`) or parameter changes.

### Key parameters in zed2i.yaml

The camera-specific config controls resolution and frame rate. For initial testing on Orin Nano, consider reducing from the default:
```yaml
general:
    grab_resolution: 'HD720'     # Default is HD1080; HD720 reduces USB and GPU load
    grab_frame_rate: 15          # Default 15; lower to 10 if experiencing drops
depth:
    min_depth: 0.3               # Default 0.3m
```

### Network download prevention

There is **no direct ROS parameter to disable network downloads**. The SDK handles this internally. To prevent downloads: use classical depth modes, keep OD/BT disabled, and ensure models are pre-cached in `/usr/local/zed/resources/`. Setting `ZED_SDK_VERBOSE=1` as an environment variable reveals all download attempts in the log.

---

## 8. Diagnostic commands and troubleshooting workflow

### Test the camera without ROS 2 first

Before debugging the wrapper, verify the SDK works independently:
```bash
# GUI camera preview
/usr/local/zed/tools/ZED_Explorer

# List detected cameras (SDK 5.2+)
/usr/local/zed/tools/ZED_Explorer -l

# Full hardware/software diagnostic (CLI mode for headless)
/usr/local/zed/tools/ZED_Diagnostic -c

# Quick Python test
python3 -c "import pyzed.sl as sl; c=sl.Camera(); p=sl.InitParameters(); \
  p.camera_resolution=sl.RESOLUTION.HD720; print('Open:', c.open(p)); c.close()"
```

The `ZED_Diagnostic` tool generates a `ZED_Diagnostic_Results.json` file containing SDK version, CUDA version, camera serial number, firmware version, USB mode (2 or 3), sensor status, and AI model optimization status. This JSON is the first thing Stereolabs support will request.

### Monitor system resources during launch

```bash
# Interactive GPU/CPU/memory/temperature monitor
jtop

# Raw tegrastats (no install needed)
sudo tegrastats --interval 500

# Set maximum performance mode (critical for Orin Nano)
sudo nvpmodel -m 0        # MAXN mode
sudo jetson_clocks         # Lock all clocks at maximum
```

During normal operation, `GR3D_FREQ` should show activity when processing frames. During TensorRT optimization, it should be near 100%. **Zero GPU activity during an apparent hang confirms the process is waiting on I/O, not computing.**

### Debug a stuck component_container_isolated

```bash
# Check if the container process is alive
ps aux | grep component_container

# Inspect what the process is blocked on
sudo strace -p <PID> -e trace=network,write

# Check open file descriptors (look for sockets)
ls -l /proc/<PID>/fd

# Check kernel stack trace
sudo cat /proc/<PID>/stack

# Verify AI model files exist
ls -la /usr/local/zed/resources/*.model
ls -la /usr/local/zed/resources/*.engine

# Check SDK version
cat /usr/local/zed/settings/ZED_SDK_VERSION
```

### Verify USB 3.0 connection

```bash
# Check if ZED is detected (Stereolabs vendor ID: 2b03)
lsusb | grep 2b03

# Check connection speed (must show 5000 for USB 3.0)
for dev in /sys/bus/usb/devices/*; do
  if [ -f "$dev/idVendor" ] && [ "$(cat $dev/idVendor 2>/dev/null)" = "2b03" ]; then
    echo "$(cat $dev/product 2>/dev/null): $(cat $dev/speed 2>/dev/null) Mbps"
  fi
done

# Check dmesg for USB errors
dmesg | grep -i "usb\|2b03\|SuperSpeed\|high-speed"
```

If the camera shows **480 Mbps** instead of 5000, it's running at USB 2.0 speed — try a different port, a shorter cable, or the original cable shipped with the camera.

### ROS 2 node diagnostics

```bash
# List active nodes and topics
ros2 node list
ros2 topic list | grep zed

# Check if topics are publishing
ros2 topic hz /zed/zed_node/rgb/image_rect_color

# View diagnostic messages
ros2 topic echo /diagnostics

# List loaded components
ros2 component list /zed/zed_container

# Review logs
cat ~/.ros/log/latest/launch.log
```

---

## 9. Community-reported solutions from 2024–2026

The Stereolabs community forums and GitHub issues reveal several recurring patterns. **Camera rebooting errors** (CAMERA_REBOOTING with exit code -11) are almost always caused by SDK/JetPack version mismatches or USB bandwidth problems — running `ZED_Diagnostic` and checking the `USBMode` field in the JSON output is the standard first diagnostic step. A June 2025 forum post showed a ZED 2i on Orin Nano that couldn't be detected because it was connecting at USB 2.0 speed despite using a USB 3.0 port.

**Performance degradation** was widely reported between SDK 4.0 and 4.1 (point cloud frequency dropping from ~10Hz to ~2Hz on Orin Nano), but SDK 5.2 includes "up to **85% lower CPU load**" on Jetson platforms and resolved most of these regressions. The recommended performance settings for Orin Nano are: `nvpmodel -m 0` (MAXN mode), `sudo jetson_clocks`, HD720 resolution, PERFORMANCE depth mode, and minimizing the number of active topic subscribers.

**Docker-specific failures** on Jetson with JP6.2 stem from `DISPLAY` environment variable mismatches (`:1` inside Docker vs `:0` on host) and missing device forwarding. The `ZED_Diagnostic` tool works outside Docker but fails inside without proper X11 forwarding and GPU device access.

GitHub issue #252 documents `point_cloud_transport` as a missing dependency during Docker builds — resolve with `sudo apt install ros-humble-point-cloud-transport` before building the workspace.

---

## 10. The exact correct first-launch procedure

Follow this sequence for a fresh Jetson Orin Nano installation:

```bash
# 1. Verify JetPack and set performance mode
cat /etc/nv_tegra_release
sudo nvpmodel -m 0
sudo jetson_clocks

# 2. Verify ZED SDK installation
cat /usr/local/zed/settings/ZED_SDK_VERSION
/usr/local/zed/tools/ZED_Explorer -l    # Should list your camera

# 3. Verify USB 3.0 connection (must show 5000 Mbps)
lsusb | grep 2b03

# 4. Pre-optimize AI models (OPTIONAL but recommended — takes 30-60 min)
/usr/local/zed/tools/ZED_Diagnostic -nrlo_all

# 5. Source ROS 2 environment
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash

# 6. Enable SDK verbose logging for first launch
export ZED_SDK_VERBOSE=1

# 7. Launch with safe settings (non-neural depth mode)
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i

# 8. In another terminal, verify topics are publishing
ros2 topic list | grep zed
ros2 topic hz /zed/zed_node/rgb/image_rect_color
ros2 topic echo /diagnostics
```

If you skipped Step 4 and the default `NEURAL_LIGHT` depth mode is active, **edit `common_stereo.yaml` before launching**:
```yaml
depth:
    depth_mode: 'PERFORMANCE'   # Avoid AI model dependency for first test
```

After confirming the camera works with classical depth, switch to neural modes by either pre-optimizing models with `ZED_Diagnostic -nrlo` or by changing `depth_mode` back to `'NEURAL_LIGHT'` and ensuring internet connectivity for the first-run optimization.

## Conclusion

The ZED 2i + Jetson Orin Nano + ROS 2 Humble stack works reliably once three critical factors are addressed. **First**, the SDK version must exactly match the L4T version — SDK 5.2.2+ for L4T R36.5, no exceptions. **Second**, the default `NEURAL_LIGHT` depth mode triggers a first-run pipeline (download + TensorRT optimization) that can silently hang without network access or take over an hour on Orin Nano's limited GPU; pre-optimizing with `ZED_Diagnostic -nrlo_all` or switching to `PERFORMANCE` mode eliminates this entirely. **Third**, USB 3.0 connectivity must be verified — a camera silently falling back to USB 2.0 causes intermittent failures that mimic software bugs.

The NITROS/Isaac ROS integration is a red herring for standard setups — it auto-disables when Isaac ROS packages are absent, and the warning in launch output is benign. The wrapper's monorepo structure means only one repository needs cloning, with `zed_msgs` available as a binary apt package for Humble. The most valuable diagnostic tool is `ZED_Diagnostic -c`, which produces a JSON report covering every hardware and software compatibility checkpoint in a single command.
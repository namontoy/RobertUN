# Robotics Development Environment — Project Context Document
# Last updated: March 25, 2026
# Paste this at the start of a new Claude session to restore full context

## HARDWARE
- **Host machine:** Dell laptop, dual boot (Ubuntu 22.04 + Linux Mint Mate)
- **Host GPU:** NVIDIA GeForce GTX 1650 Ti Mobile (TU117M)
- **Host display:** Laptop screen (eDP-1) + external monitor (HDMI-1-0), both 1920x1080
- **Jetson:** NVIDIA Jetson Orin Nano, 7.4GB RAM, 233GB NVMe SSD
- **Camera:** Stereolabs ZED 2i stereo camera
- **Camera status:** SDK installed, USB cable ordered (USB-IF certified 10Gbps Gen2)
  - Camera detected by Linux (lsusb shows 2b03:f880) but running at USB 2.0
  - speeds due to wrong cable — will be fixed when new cable arrives
- **Planned:** Gigabit network switch for dedicated development network

## NETWORK CONFIGURATION
- **Host IP:** 192.168.1.12 (static, via NetworkManager)
- **Jetson IP:** 192.168.1.11 (static, via NetworkManager)
- **Gateway:** 192.168.1.1
- **DNS:** 8.8.8.8 / 8.8.4.4 (Google DNS on both machines)
- **WiFi network:** Gotham_Castle
- **Host WiFi interface:** wlp5s0
- **Jetson WiFi interface:** wlP1p1s0

## SSH CONFIGURATION
- **SSH port:** 44252 (0xACDC)
- **Authentication:** ED25519 key pair, passwordless
- **Host:** username=talos, hostname=deadelus
- **Jetson:** username=talos, hostname=orion
- **SSH config on host:** ~/.ssh/config has 'Host jetson' → 192.168.1.11:44252
- **Aliases on host:**
  - `jetson` → ssh to Jetson
  - `jsync` → rsync ~/ros2_ws/ to Jetson with correct port
  - `jscp` → scp with correct port (-P 44252)
- **SSH auto-starts on Jetson boot:** confirmed
- **WiFi power management:** DISABLED permanently on Jetson via:
  - /etc/NetworkManager/dispatcher.d/99-disable-wifi-powersave
  - /etc/udev/rules.d/70-wifi-powersave.rules

## HOST MACHINE — Ubuntu 22.04.5 LTS x86_64
- **Display server:** X11 (Wayland permanently disabled)
- **NVIDIA Driver:** 580.126.09 (nvidia-driver-580-open)
- **CUDA Toolkit:** 12.6.3 (/usr/local/cuda-12.6/)
- **GCC:** 11.4.0
- **CMake:** 3.22.1
- **Java JDK:** 21.0.10 (OpenJDK)
- **VS Code:** 1.112.0
- **Python:** 3.10.12 (~/.local/bin on PATH)
- **pip:** 26.0.1
- **Python packages:** numpy, opencv-python, pyserial, transforms3d,
  scipy, matplotlib, pytest, pyzed
- **ROS 2:** Humble Desktop (/opt/ros/humble/)
- **DDS:** Cyclone DDS (rmw_cyclonedds_cpp)
  - Config: ~/.ros/cyclone_dds.xml
  - Unicast peer: 192.168.1.11, interface: wlp5s0
- **Cross-compiler:** aarch64-linux-gnu-gcc/g++ 11.4.0
- **QEMU:** 6.2.0 with binfmt F-flag (permanent via systemd service)
  - Service: /etc/systemd/system/qemu-aarch64-binfmt-fix.service
  - Uses multiarch/qemu-user-static --reset -p yes on every boot
- **Docker:** 29.3.0
  - Buildx builder: jetson-builder (linux/amd64 + linux/arm64)
  - Local images: jetson-test:arm64, ubuntu:22.04
- **QEMU_LD_PREFIX:** /usr/aarch64-linux-gnu (in ~/.bashrc)

## JETSON ORIN NANO — JetPack 6.5 / L4T R36.5.0, aarch64
- **OS:** Ubuntu 22.04.5 LTS
- **CUDA:** 12.6.68 (via apt, /usr/local/cuda-12.6/)
- **TensorRT:** 10.3.0 (via apt nvidia-tensorrt)
  - libnvdla_compiler.so: manually extracted from nvidia-l4t-dla-compiler_36.4.1
    and placed at /usr/lib/aarch64-linux-gnu/nvidia/libnvdla_compiler.so
    (missing from R36.5 packages — known NVIDIA packaging bug)
- **cuDNN:** installed via apt
- **Python:** 3.10.12
- **ROS 2:** Humble Base (/opt/ros/humble/)
- **DDS:** Cyclone DDS (rmw_cyclonedds_cpp)
  - Config: ~/.ros/cyclone_dds.xml
  - Unicast peer: 192.168.1.12, interface: wlP1p1s0
- **Docker:** 29.3.0
  - NVIDIA Container Runtime: DEFAULT runtime in /etc/docker/daemon.json
  - Local images:
    - nvcr.io/nvidia/l4t-jetpack:r36.4.0
    - dustynv/ros:humble-ros-base-l4t-r36.3.0
    - ubuntu:22.04
    - hello-world:latest
- **ZED SDK:** 5.2.2 installed at /usr/local/zed/
  - Python API: pyzed 5.2.2 confirmed working
  - TensorRT models optimized: Neural Depth, Neural Light Depth,
    Neural Plus Depth, Object Detection (3 tiers), Person ReID,
    Skeleton Body18/38, Person Head
  - Camera NOT yet working — waiting for USB 3.2 Gen2 cable
  - Camera IS detected by Linux: lsusb shows 2b03:f880 (ZED 2i)
    and 2b03:f881 (ZED-2i HID Interface)
  - Root cause: generic USB-C cable is USB 2.0 only (bcdUSB 2.10)
    ZED SDK requires USB 3.0+ (needs bcdUSB 3.x)
  - Fix: Cable Matters USB-IF certified 10Gbps Gen2 cable ordered,
    arriving in ~2 days

## SYSTEMD SERVICES (HOST)
- **qemu-aarch64-binfmt-fix.service:**
  Runs multiarch/qemu-user-static --reset -p yes on every boot
  Registers QEMU with F-flag for ARM64 Docker container support

## WORKSPACE STRUCTURE
- **Host workspace:** ~/ros2_ws/
  - .devcontainer/ (devcontainer.json, Dockerfile, cyclone_dds.xml)
  - src/ (ROS2 packages go here)
  - docker/ (Docker-related files)
  - testing/ (test binaries, e.g. hello_jetson ARM64 cross-compiled)
- **Jetson workspace:** ~/ros2_ws/ (mirrored from host via jsync)
- **Jetson ZED test folder:** ~/zed2i/
  - test_zed.py (camera open + depth grab test — ready to run)
  - zed_version.py (device detection test)

## VS CODE DEV CONTAINER (HOST)
- **Status:** Fully configured and working
- **Name:** ROS2 Humble + ZED 2i Development
- **Base:** Ubuntu 22.04 amd64 (native speed, not emulated)
- **Contents:** ROS 2 Humble Desktop + Cyclone DDS + CUDA headers
  + OpenCV + Python robotics packages
- **Network:** --network=host
- **Note:** --runtime=nvidia was REMOVED from runArgs (host has no
  NVIDIA Container Runtime — only needed on Jetson)
- **Verified:** ROS 2 talker/listener working Dev Container ↔ Jetson
- **IntelliSense:** Confirmed working for rclcpp, std_msgs,
  geometry_msgs, sensor_msgs

## ROS 2 COMMUNICATION
- **DDS:** Cyclone DDS on ALL environments (host, Dev Container, Jetson)
- **Discovery:** Unicast peer-to-peer (WiFi router blocks multicast)
- **Verified paths:**
  - Host ↔ Jetson ✅
  - Dev Container ↔ Jetson ✅
  - C++ nodes (demo_nodes_cpp) ✅
  - Python nodes (demo_nodes_py) ✅

## CROSS-COMPILATION
- ARM64 binary compiled on host: ~/ros2_ws/testing/hello_jetson
- Runs on host via QEMU: ✅
- Runs natively on Jetson: ✅
- Docker ARM64 images build and run: ✅

## NEXT TASKS (IN PRIORITY ORDER)
1. **ZED cable arrives → test camera:**
   On Jetson run: python3 ~/zed2i/test_zed.py
   First verify: lsusb -v -d 2b03:f880 | grep bcdUSB
   Expected: bcdUSB 3.10 or 3.20

2. **Install ZED ROS 2 wrapper on Jetson:**
   Clone stereolabs/zed-ros2-wrapper, install deps, build with colcon
   Can be done WITHOUT camera present

3. **Update Dev Container for ZED IntelliSense:**
   Add ZED SDK headers to .devcontainer/Dockerfile so VS Code
   understands all ZED APIs (sl::Camera, sl::Mat, etc.)

4. **Gigabit switch network (when switch purchased):**
   Configure dedicated development subnet (e.g. 10.0.0.x)
   Separate from home WiFi for faster jsync and Docker transfers

5. **Begin robot ROS 2 development:**
   SLAM/Mapping, Visual Odometry/Navigation, Object Detection/AI

## KEY DECISIONS AND RATIONALE
- **X11 over Wayland:** Wayland has incomplete NVIDIA PRIME support
  for multi-monitor on Dell laptops
- **CUDA 12.6 on both machines:** Cross-compilation compatibility
- **Cyclone DDS over Fast DDS:** WiFi routers block multicast;
  Cyclone DDS unicast bypasses this reliably
- **Static IPs via NetworkManager:** Router not accessible
- **SSH port 44252 (0xACDC):** Reduces automated bot scanning
- **libnvdla_compiler.so fix:** Extracted from nvidia-l4t-dla-compiler
  _36.4.1 .deb — known NVIDIA packaging omission in R36.4+ packages
- **dustynv/ros over nvcr.io/nvidia/l4t-base:** l4t-base tags beyond
  r36.2.0 not consistently published; dustynv/ros actively maintained
  with CUDA + cuDNN + TensorRT + ROS 2 pre-integrated
- **amd64 Dev Container:** Native speed for IntelliSense; ARM64
  emulation via QEMU is too slow for daily development
- **VS Code over Antigravity:** Mature ROS 2 extensions, Remote SSH,
  Dev Containers; Antigravity is preview-stage with no robotics support
- **ZED SDK 5.2.2 for L4T R36.5:** Specific version matching
  JetPack 6.2.2 / CUDA 12.6 — fixes ZED2i positional tracking lock bug

## USEFUL COMMANDS REFERENCE
### Host
- `ssh jetson` → connect to Jetson
- `jsync` → sync ~/ros2_ws/ to Jetson
- `jscp <file> talos@192.168.1.11:<path>` → copy file to Jetson
- `docker buildx build --platform linux/arm64 --tag <name> --load .`

### Jetson
- `python3 ~/zed2i/test_zed.py` → test ZED camera (needs USB 3.x cable)
- `ros2 run demo_nodes_cpp talker` → test ROS 2
- `docker run --env NVIDIA_VISIBLE_DEVICES=all <image> nvcc --version`

### VS Code
- `Ctrl+Shift+P` → `Dev Containers: Reopen in Container` → open Dev Container
- `Ctrl+Shift+P` → `Remote-SSH: Connect to Host` → jetson → connect to Jetson
- `Ctrl+Shift+P` → `Dev Containers: Rebuild and Reopen` → only when Dockerfile changes
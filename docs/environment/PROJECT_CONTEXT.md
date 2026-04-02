# Robotics Development Environment — Project Context Document
# Last updated: March 30, 2026
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
- **Robot platform:** 6-wheeled Mars rover inspired by Spirit/Opportunity
  - Rocker-Bogie suspension frame
  - Estimated CAN bus cable length: 15–20 meters (accounting for frame routing,
    joint articulation, and service loops at flex points)

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

## CAN BUS — HARDWARE DESIGN DECISIONS
### Jetson Orin Nano CAN capability
- **Built-in MTTCAN controller:** yes — T234 SoC has native CAN hardware
  - No USB-to-CAN adapter, SPI bridge, or M.2 card required
  - Controller: MTTCAN (mttcan@c310000), supports CAN 2.0 and CAN FD
  - Exposed channels: **1 only (CAN0)** — Orin Nano module only routes one
    channel; CAN1 pins are n/a on the SO-DIMM connector
  - Bitrate range: 10 kbps – 1 Mbps (CAN 2.0), up to 5 Mbps data rate (CAN FD)
  - Linux driver: mttcan (SocketCAN), interface appears as can0
- **Physical access: J17 header (unpopulated from factory)**
  - J17 is a 4-pin, 2.54mm pitch right-angle header on the developer kit carrier
  - Must solder a pin header to J17 before use
  - J17 pinout:
    - Pin 1: CAN_TX (output, 3.3V)
    - Pin 2: CAN_RX (input, 3.3V)
    - Pin 3: GND
    - Pin 4: 3.3V supply
  - CAN pins are NOT on the 40-pin expansion header (J12)
  - SO-DIMM pin 145 = CAN_TX, pin 143 = CAN_RX
- **Note:** jetson-io.py does NOT support CAN — it only manages J12 (40-pin header)

### Transceiver (all nodes)
- **Selected IC:** TI SN65HVD230
  - 3.3V supply — compatible with both Jetson 3.3V logic and STM32F4xx at 3.3V
  - Supports CAN 2.0 up to 1 Mbps
  - Available as bare IC or as ready-made breakout boards (Waveshare recommended
    by NVIDIA for development)
  - All nodes (Jetson + all STM32 nodes) use this same transceiver
  - For CAN FD in the future: upgrade to TJA1051T/3 or MCP2562FD

### STM32 CAN peripheral
- **STM32F4xx uses bxCAN peripheral** (Basic Extended CAN)
  - Full layer 2 implementation in hardware: frame construction, arbitration,
    CRC, ACK, fault confinement, acceptance filters — all in silicon
  - bxCAN outputs logic-level CAN_TX / CAN_RX to the SN65HVD230 transceiver
  - No built-in transceiver — external IC always required (same as Jetson)

### Physical bus
- **Cable:** DeviceNet cable (120Ω characteristic impedance, flex-rated)
  - Do NOT use Cat-5 (100Ω, no flex rating) for the rover
  - Cat-5 acceptable for bench testing only at short distances
  - Alternatives: LAPP UNITRONIC BUS CAN — check LAPP Latin America distributor
    or industrial automation suppliers in Medellín for local availability
- **Topology:** Backbone + T-tap (stub) topology
  - Single backbone cable snakes through rover frame
  - Each node connects via a short stub (< 30cm) off the backbone
  - DeviceNet Mini/Micro Open style 5-pin connectors for T-tap insertion
    without cutting the backbone cable
- **Termination:** Two 120Ω resistors only — one at each physical end of backbone
  - End nodes: one wheel node at each extreme of the rover frame
  - Jetson is a mid-bus node, NOT a termination point
  - Never place termination resistors at every node — this collapses bus impedance
- **Bitrate:** all nodes must be configured identically (e.g. 500 kbps)
  - Mismatch = arbitration failure and bus corruption

### Message ID design principles (decided in session)
- Priority is message-centric, not node-centric
- Lower ID = higher priority (wins arbitration)
- ID table uses two-level hierarchy: upper bits = group, lower bits = message type
  - Enables single mask-filter entries per node (hardware acceptance filter)
- Example priority groups:
  - 0x001–0x00F: safety-critical (emergency stop, overcurrent fault)
  - 0x010–0x07F: control commands (brake, motor torque)
  - 0x080–0x0FF: sensor setpoints
  - 0x100–0x4FF: periodic sensor data (IMU, encoders)
  - 0x500–0x5FF: telemetry / diagnostics / heartbeat

### OSI layer mapping
- Layer 1 (Physical): SN65HVD230 transceiver IC + DeviceNet cable + 120Ω termination
- Layer 2 (Data Link): bxCAN peripheral (STM32) / MTTCAN peripheral (Jetson)
  - Both are implemented in hardware silicon — no software CAN stack needed
- Layers 3–7: not defined by CAN itself; to be implemented at application layer
  - Future consideration: CANopen or custom application-layer protocol for
    rover message definitions

## CAN BUS — JETSON SOFTWARE SETUP (JetPack 6.x / L4T R36.x)
### Prerequisites
```bash
sudo apt-get install busybox can-utils
```

### Pinmux configuration (must run after every reboot — not persistent)
```bash
sudo busybox devmem 0x0c303018 w 0xc458   # CAN0_DIN (RX)
sudo busybox devmem 0x0c303010 w 0xc400   # CAN0_DOUT (TX)
```

### Load kernel modules
```bash
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
```
**Critical:** mttcan is blacklisted by default in /etc/modprobe.d/denylist-mttcan.conf
To enable permanently:
```bash
sudo rm /etc/modprobe.d/denylist-mttcan.conf
echo "mttcan" | sudo tee /etc/modules-load.d/mttcan.conf
```

### Bring up CAN interface (SocketCAN)
```bash
# Classical CAN at 500 kbps:
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# CAN FD (500 kbps nominal / 1 Mbps data):
sudo ip link set can0 up type can bitrate 500000 dbitrate 1000000 berr-reporting on fd on
```

### Test with can-utils
```bash
candump can0                        # receive all frames
cansend can0 123#ABCDABCD           # send a frame
```

### Loopback self-test (no transceiver needed — short TX and RX pins on J17)
```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 loopback on
sudo ip link set can0 up
candump can0 &
cansend can0 123#ABCDABCD
```

### Verify device tree node is active
```bash
cat /proc/device-tree/bus@0/mttcan@c310000/status   # should print: okay
```
Note: path prefix is bus@0/ on JetPack 6.x (different from JetPack 5.x)

### Known issues / gotchas
- mttcan blacklist is the #1 cause of "CAN doesn't work" — always check first
- Pinmux resets on reboot when using devmem method — use systemd service for persistence
- If timing sync issues with external nodes: add sjw 4 parameter:
  `sudo ip link set can0 type can bitrate 500000 sjw 4`
- CAN FD at 5 Mbps may need TDCR tuning via sysfs

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

4. **CAN Bus — first hardware test:**
   - Solder 4-pin header to J17 on Jetson dev kit
   - Wire SN65HVD230 breakout to J17 (TX→TXD, RX→RXD, 3.3V, GND)
   - Run software setup: remove mttcan blacklist, configure pinmux,
     load modules, bring up can0
   - Loopback self-test first (short TX+RX on J17, no transceiver needed)
   - Then test with STM32 node: configure bxCAN at same bitrate,
     wire via SN65HVD230, add 120Ω termination at both ends
   - Use Cat-5 cable acceptable for bench test; use DeviceNet for rover

5. **CAN Bus — STM32 firmware:**
   - Configure bxCAN peripheral registers (bit timing for target bitrate)
   - Implement acceptance filter with mask-based ID table
   - Test frame exchange: Jetson candump ↔ STM32 cansend and vice versa

6. **CAN Bus — ROS 2 integration:**
   - Bridge CAN frames to ROS 2 topics (ros2_socketcan or custom node)
   - Define message ID table for rover subsystems

7. **Gigabit switch network (when switch purchased):**
   Configure dedicated development subnet (e.g. 10.0.0.x)
   Separate from home WiFi for faster jsync and Docker transfers

8. **Begin robot ROS 2 development:**
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
- **SN65HVD230 for CAN transceiver:** 3.3V compatible with both Jetson
  and STM32F4xx; widely used, well-documented, available as breakout board
- **DeviceNet cable for CAN bus:** 120Ω characteristic impedance (correct
  for CAN), flex-rated for Rocker-Bogie articulation, industrial standard
- **Backbone + T-tap topology:** Standard for multi-node CAN installations;
  DeviceNet connectors allow node insertion without cutting backbone
- **Message-centric CAN ID priority:** Priority assigned per message type,
  not per node — safety-critical messages win regardless of source node

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

### Jetson — CAN Bus
- `sudo busybox devmem 0x0c303018 w 0xc458 && sudo busybox devmem 0x0c303010 w 0xc400` → configure pinmux
- `sudo modprobe can && sudo modprobe can_raw && sudo modprobe mttcan` → load modules
- `sudo ip link set can0 type can bitrate 500000 && sudo ip link set can0 up` → bring up interface
- `candump can0` → monitor all CAN traffic
- `cansend can0 123#DEADBEEF` → send test frame
- `cat /proc/device-tree/bus@0/mttcan@c310000/status` → verify CAN hardware active

### VS Code
- `Ctrl+Shift+P` → `Dev Containers: Reopen in Container` → open Dev Container
- `Ctrl+Shift+P` → `Remote-SSH: Connect to Host` → jetson → connect to Jetson
- `Ctrl+Shift+P` → `Dev Containers: Rebuild and Reopen` → only when Dockerfile changes

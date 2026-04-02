# Robotics Development Environment — Project Context Document
# Last updated: April 2, 2026 (network update)
# Paste this at the start of a new Claude session to restore full context

## HARDWARE

### Dell Laptop (home development machine)
- **Machine:** Dell laptop, dual boot (Ubuntu 22.04 + Linux Mint Mate)
- **GPU:** NVIDIA GeForce GTX 1650 Ti Mobile (TU117M)
- **Display:** Laptop screen (eDP-1) + external monitor (HDMI-1-0), both 1920x1080
- **Hostname:** deadelus, username: talos

### IsaacUN (office simulation workstation)
- **Machine:** Desktop PC, Universidad Nacional de Colombia
- **CPU:** AMD Ryzen 7 9700X 8-Core (16 threads, 5.58GHz max)
- **RAM:** 64GB
- **GPU:** NVIDIA GeForce RTX 5080 (Blackwell GB203, 16GB VRAM)
- **OS:** Ubuntu 24.04.3 LTS (noble)
- **Kernel:** 6.17.0-19-generic
- **Display:** 2x Dell monitors via DP-2 (primary, left) and DP-3 (right), both 1440x900
- **Hostname:** IsaacUN, username: talos

### Jetson (embedded robot compute)
- **Machine:** NVIDIA Jetson Orin Nano, 7.4GB RAM, 233GB NVMe SSD
- **Hostname:** orion, username: talos

### Camera
- **Model:** Stereolabs ZED 2i stereo camera
- **Status:** SDK installed, USB cable ordered (USB-IF certified 10Gbps Gen2)
  - Camera detected by Linux (lsusb shows 2b03:f880) but running at USB 2.0
    speeds due to wrong cable — will be fixed when new cable arrives

### Network switch
- **Status:** Gigabit switch installed and running at home
- **Role:** Primary development network backbone — all wired traffic routes
  through the switch rather than WiFi
- **Topology:** Switch connects to router (internet access) + Dell + Jetson
  via Ethernet; WiFi remains active on both machines as automatic fallback

### Robot platform
- **Design:** 6-wheeled Mars rover inspired by Spirit/Opportunity
- **Suspension:** Rocker-Bogie
- **CAN bus cable length:** estimated 15–20 meters (accounting for frame routing,
  joint articulation, and service loops at flex points)

## NETWORK CONFIGURATION

### Home network (Dell laptop + Jetson)
- **Network convention:** Ethernet IP = 200 + WiFi IP (easy to remember)
- **Gigabit switch:** installed, connects router + Dell + Jetson via Ethernet

#### Dell laptop
- **Ethernet (primary):** 192.168.1.212 (static, interface enp4s0, metric 100)
- **WiFi (backup):** 192.168.1.12 (static, interface wlp5s0, metric 600)
- **WiFi network:** Gotham_Castle

#### Jetson Orin Nano
- **Ethernet (primary):** 192.168.1.211 (static, interface enP8p1s0, metric 100)
- **WiFi (backup):** 192.168.1.11 (static, interface wlP1p1s0, metric 600)
- **WiFi network:** Gotham_Castle

#### Shared
- **Gateway:** 192.168.1.1
- **DNS:** 8.8.8.8 / 8.8.4.4 (Google DNS on both machines)
- **Routing behavior:** Linux metric system automatically prefers Ethernet
  (metric 100) over WiFi (metric 600); failover to WiFi is automatic
  when Ethernet cable is disconnected

### University network (IsaacUN)
- **IsaacUN:** on university private network
- **Remote access:** University VPN → NoMachine (NX protocol, port 4000)
- **Note:** IsaacUN and Jetson/Dell are on separate networks (university vs home)
  Direct ROS 2 communication between IsaacUN and Jetson is not configured —
  simulation and real robot workflows are developed in parallel and synchronized
  deliberately via file transfer rather than always-connected DDS

## SSH CONFIGURATION
- **SSH port:** 44252 (0xACDC) on ALL machines
- **Authentication:** ED25519 key pair, passwordless
- **Dell:** username=talos, hostname=deadelus
- **Jetson:** username=talos, hostname=orion
- **IsaacUN:** username=talos, hostname=IsaacUN
- **SSH config on Dell:** ~/.ssh/config has 'Host jetson' → 192.168.1.211:44252 (Ethernet)
- **IsaacUN SSH:** systemd socket activation override at
  /etc/systemd/system/ssh.socket.d/override.conf (port 44252)
- **Aliases on Dell:**
  - `jetson` → ssh to Jetson
  - `jsync` → rsync ~/ros2_ws/ to Jetson with correct port
  - `jscp` → scp with correct port (-P 44252)
- **SSH auto-starts on Jetson boot:** confirmed
- **WiFi power management:** DISABLED permanently on Jetson via:
  - /etc/NetworkManager/dispatcher.d/99-disable-wifi-powersave
  - /etc/udev/rules.d/70-wifi-powersave.rules

## DELL LAPTOP — Ubuntu 22.04.5 LTS x86_64
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
- **Python environment:** conda environments not yet configured on Dell
  (pending task — mirror IsaacUN approach)
- **ROS 2:** Humble Desktop (/opt/ros/humble/)
- **DDS:** Cyclone DDS (rmw_cyclonedds_cpp)
  - Config: ~/.ros/cyclone_dds.xml
  - Interface: enp4s0 (Ethernet, via switch)
  - Unicast peer: 192.168.1.211 (Jetson Ethernet address)
  - Dev Container also updated: ~/ros2_ws/.devcontainer/cyclone_dds.xml
- **Cross-compiler:** aarch64-linux-gnu-gcc/g++ 11.4.0
- **QEMU:** 6.2.0 with binfmt F-flag (permanent via systemd service)
  - Service: /etc/systemd/system/qemu-aarch64-binfmt-fix.service
  - Uses multiarch/qemu-user-static --reset -p yes on every boot
- **Docker:** 29.3.0
  - Buildx builder: jetson-builder (linux/amd64 + linux/arm64)
  - Local images: jetson-test:arm64, ubuntu:22.04
- **QEMU_LD_PREFIX:** /usr/aarch64-linux-gnu (in ~/.bashrc)
- **Git:** configured (user: namontoy, email: namontoy@unal.edu.co)
  - GitHub SSH key: ED25519 (reused from Jetson connection, labeled
    "Deadelus to Orion")
  - Repository: git@github.com:namontoy/RobertUN.git
  - Local clone: ~/github/RobertUN/

## ISAACUN — Ubuntu 24.04.3 LTS x86_64
- **Display server:** X11 (Wayland disabled by session type)
- **Display manager:** GDM3 (switched from LightDM — required for GNOME
  ScreenShield lock screen and correct session initialization)
- **Desktop:** GNOME 46 on Xorg
- **NVIDIA Driver:** 570.211.01
- **CUDA Toolkit:** 12.8 (/usr/local/cuda-12.8/, symlinked at /usr/local/cuda)
  - nvcc confirmed working: release 12.8, V12.8.93
  - PATH: /usr/local/cuda/bin added to ~/.bashrc
  - LD_LIBRARY_PATH: /usr/local/cuda/lib64 added to ~/.bashrc
- **Isaac Sim:** 5.1.0-rc.19 installed at ~/isaac-sim/
  - Launch: ~/isaac-sim/isaac-sim.sh
  - Internal ROS 2: Jazzy (auto-loaded on Ubuntu 24.04)
  - Health check: confirmed working after GDM switch and CUDA install
  - Note: must be launched outside any conda environment
- **ROS 2:** Jazzy Desktop (/opt/ros/jazzy/)
  - Jazzy is the recommended distro for Isaac Sim 5.x on Ubuntu 24.04
  - Jazzy and Humble are wire-compatible for standard message types
- **DDS:** FastDDS (default for Jazzy) — CycloneDDS not configured
  (IsaacUN not connected to Jetson/Dell network)
- **Python:** via Miniconda3 (/home/talos/miniconda3/)
  - Philosophy: always work inside conda environments, never system Python
  - conda env `ros2`: Python 3.12, ROS 2 Jazzy auto-sourced on activation
    Packages: numpy 2.4.4, opencv-python 4.13, scipy 1.17.1, matplotlib 3.10,
    pyserial 3.5, transforms3d 0.4.2, pytest 9.0.2, pyyaml 6.0.3,
    rclpy 7.1.9, full ROS 2 Python stack, catkin-pkg, empy==3.3.4, lark
  - Activation script: ~/miniconda3/envs/ros2/etc/conda/activate.d/ros2.sh
    (auto-sources /opt/ros/jazzy/setup.bash and sets ROS_DOMAIN_ID=0)
- **tmux:** installed, config at ~/.tmux.conf
  - Prefix: Ctrl+B (Ctrl+A as alternative)
  - Mouse support enabled
  - Split: Ctrl+B | (vertical), Ctrl+B - (horizontal)
  - Navigation: Ctrl+B h/j/k/l (vim-style)
  - History limit: 50000 lines
- **colcon:** installed (python3-colcon-common-extensions)
- **rosdep:** 0.26.0 (initialized)
- **Git:** not yet configured (pending task)

## ISAACUN — SESSION AND REMOTE ACCESS
- **Autologin:** GDM3 autologin for talos, config at /etc/gdm3/custom.conf
- **Screen lock:** Locks automatically ~15 seconds after boot via
  ~/.config/autostart/lock-after-autologin.desktop (gdbus call to ScreenSaver)
- **Dual monitor fix:** DP-3 auto-detected via
  ~/.config/autostart/restore-monitors.desktop (xrandr --auto at 8s)
- **GNOME keyring:** Unlocks during autologin via PAM config
  at /etc/pam.d/lightdm-autologin (pam_gnome_keyring.so)
- **NoMachine:** 9.3.7, nxserver running, EGL GPU capture enabled
  - Connect: university VPN → NoMachine to IsaacUN IP, port 4000
  - Login: talos credentials
  - node.cfg: EnableEGLCapture=1 (GPU-accelerated screen capture)
  - Known issue: on some client machines, NoMachine shows DP-3 (right
    monitor) instead of DP-2 (primary/left). Client-side fix: set
    "View a specific monitor among available monitors" = 1 in .nxs file.
    Mouse coordinate offset remains unresolved on affected clients.
    Dell laptop works correctly without any special configuration.
- **SSH:** port 44252 via systemd socket override
  - /etc/systemd/system/ssh.socket.d/override.conf

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
  - Interface: enP8p1s0 (Ethernet, via switch)
  - Unicast peer: 192.168.1.212 (Dell Ethernet address)
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
  - Fix: Cable Matters USB-IF certified 10Gbps Gen2 cable ordered
- **Git:** not yet configured (pending task)

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
- **Selected bitrate: 125 kbps**
  - Rover moves slowly, 8 nodes, no high-frequency control loops requiring > 125 kbps
  - At 125 kbps, bit time = 8 µs; reflections from impedance mismatch arrive in
    ~200 ns (2.5% of bit time) — well within CAN bit timing tolerance
  - This makes 120Ω vs 100Ω cable impedance mismatch irrelevant for this application
  - All nodes must be configured identically — mismatch = arbitration failure

- **Selected cable: flexible Cat-5/6 (100Ω)**
  - Rationale: at 125 kbps the impedance mismatch is negligible (see above)
  - Must use flex-rated Cat-5/6 (not standard structured cabling patch cable)
    due to continuous Rocker-Bogie joint articulation
  - Cost-effective and locally available vs industrial CAN cable
  - NOTE — if bitrate is ever raised significantly or rover design changes,
    consider upgrading to proper 120Ω cable:
    - DeviceNet cable: 120Ω, flex-rated, industrial standard
    - LAPP UNITRONIC BUS CAN: check LAPP Latin America distributor or
      industrial automation suppliers in Medellín for local availability

- **Selected connectors: Bulgin 400 Series Buccaneer (8-pin, IP68)**
  - 17 units available — sufficient for 8-node system (14 used, 3 spare)
  - IP68 rated: dust-tight, suitable for outdoor rover environment
  - Robust latching mechanism, rated for repeated connection cycles
  - 8 pins — enough for CAN signals + power on single connector
  - Suggested pin allocation:
    - Pin 1: CAN_H
    - Pin 2: CAN_L
    - Pin 3: GND
    - Pin 4: Power (5V or 12V — TBD)
    - Pins 5–8: spare / future use

- **Topology: linear backbone, pass-through on node PCBs**
  - Single backbone cable snakes through rover frame end to end
  - Middle nodes (6 total): 2× Bulgin connectors per PCB
    - Backbone passes through via direct copper trace (CAN_H, CAN_L, GND, Power)
    - Short branch on PCB trace taps off backbone to SN65HVD230 transceiver
    - Tap point at electrical midpoint of the pass-through trace
    - At 125 kbps, tap position on PCB trace has no signal integrity impact
  - End nodes (2 total — one wheel at each extreme of rover frame): 1× Bulgin connector
    - 120Ω termination resistor on PCB
  - Connector count: 2×6 + 1×2 = 14 connectors used, 3 spare

- **Termination: two 120Ω resistors only**
  - One at each physical end of backbone (the two extreme wheel nodes)
  - Jetson is a mid-bus node — no termination resistor
  - Never place termination at every node — collapses bus impedance

### Message ID design principles
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
- Layer 1 (Physical): SN65HVD230 transceiver IC + flex Cat-5/6 cable + 120Ω termination
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
# Classical CAN at 125 kbps (selected rover bitrate):
sudo ip link set can0 type can bitrate 125000
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
sudo ip link set can0 type can bitrate 125000 loopback on
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
  `sudo ip link set can0 type can bitrate 125000 sjw 4`
- CAN FD at 5 Mbps may need TDCR tuning via sysfs

## SYSTEMD SERVICES (DELL HOST)
- **qemu-aarch64-binfmt-fix.service:**
  Runs multiarch/qemu-user-static --reset -p yes on every boot
  Registers QEMU with F-flag for ARM64 Docker container support

## SYSTEMD SERVICES (ISAACUN)
- **ssh.socket override:**
  /etc/systemd/system/ssh.socket.d/override.conf
  Forces SSH to listen on port 44252 instead of default 22

## WORKSPACE STRUCTURE
- **Dell workspace:** ~/ros2_ws/
  - .devcontainer/ (devcontainer.json, Dockerfile, cyclone_dds.xml)
  - src/ (ROS2 packages go here)
  - docker/ (Docker-related files)
  - testing/ (test binaries, e.g. hello_jetson ARM64 cross-compiled)
- **IsaacUN workspace:** ~/ros2_ws/
  - src/ (simulation-side ROS 2 packages)
  - Role: robot descriptions, simulation launch files, Isaac Sim ROS 2 bridges
- **Jetson workspace:** ~/ros2_ws/ (mirrored from Dell via jsync)
- **Jetson ZED test folder:** ~/zed2i/
  - test_zed.py (camera open + depth grab test — ready to run)
  - zed_version.py (device detection test)

## GITHUB REPOSITORY
- **Repository:** git@github.com:namontoy/RobertUN.git
- **Local clone (Dell):** ~/github/RobertUN/
- **Structure:**
  - BoardRover2/ (hardware design files)
  - docs/environment/PROJECT_CONTEXT.md (this file)
  - docs/research/can-bus/CAN-Bus-JetsonOrinNano.md
- **Git configured on:** Dell laptop only (pending: IsaacUN, Jetson)
- **SSH key added to GitHub:** Dell laptop ED25519 key

## VS CODE DEV CONTAINER (DELL HOST)
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

### Home network (Dell ↔ Jetson)
- **DDS:** Cyclone DDS on ALL environments (Dell, Dev Container, Jetson)
- **Discovery:** Unicast peer-to-peer (WiFi router blocks multicast)
- **Verified paths:**
  - Dell ↔ Jetson ✅
  - Dev Container ↔ Jetson ✅
  - C++ nodes (demo_nodes_cpp) ✅
  - Python nodes (demo_nodes_py) ✅

### IsaacUN (standalone)
- **DDS:** FastDDS (Jazzy default)
- **Discovery:** Local only — not connected to Jetson/Dell
- **Verified:** talker/listener on IsaacUN ✅

## CROSS-COMPILATION (DELL HOST)
- ARM64 binary compiled on host: ~/ros2_ws/testing/hello_jetson
- Runs on host via QEMU: ✅
- Runs natively on Jetson: ✅
- Docker ARM64 images build and run: ✅

## WORKFLOW SEPARATION
- **IsaacUN:** Robot simulation, Isaac Sim training, Jazzy ROS 2 development
- **Dell + Jetson:** Real robot development, Humble ROS 2, embedded deployment
- **Sim-to-real transfer:** Package source code synced manually between
  machines — not via live network bridge

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
   - Order/acquire Canable USB CAN sniffer (AliExpress clone ~$10-15 USD)
     Flash candlelight firmware on arrival (preferred over slcan)
   - Solder 4-pin header to J17 on Jetson dev kit
   - Wire SN65HVD230 breakout to J17 (TX→TXD, RX→RXD, 3.3V, GND)
   - Run software setup: remove mttcan blacklist, configure pinmux,
     load modules, bring up can0 at 125000 bps
   - Loopback self-test first (short TX+RX on J17, no transceiver needed)
   - Then bench test with STM32 node + Canable sniffer on Dell laptop:
     configure bxCAN at 125 kbps, wire via SN65HVD230,
     add 120Ω termination at both ends, use any available Cat-5/6 cable
   - Verify three-way communication: Jetson ↔ STM32 ↔ Canable sniffer

5. **CAN Bus — STM32 firmware:**
   - Configure bxCAN peripheral registers (bit timing for target bitrate)
   - Implement acceptance filter with mask-based ID table
   - Test frame exchange: Jetson candump ↔ STM32 cansend and vice versa

6. **CAN Bus — ROS 2 integration:**
   - Bridge CAN frames to ROS 2 topics (ros2_socketcan or custom node)
   - Define message ID table for rover subsystems

7. **Configure Isaac Sim ROS 2 bridge:**
   Set up CycloneDDS or FastDDS for IsaacUN ROS 2 bridge
   Test Isaac Sim publishing to ROS 2 topics (e.g. /clock, /odom)

8. **Python conda environment on Dell laptop:**
   Configure conda environments on Dell (not yet done)
   Mirror the ros2 conda env approach used on IsaacUN
   Philosophy: always work inside conda environments, never system Python

9. **Git configuration on IsaacUN and Jetson:**
   Configure git user credentials on IsaacUN and Jetson
   Add SSH keys from both machines to GitHub
   Enable direct push to RobertUN repository from all three machines

10. **Update RobertUN README.md:**
    Current README is minimal — describe the project properly
    Document repository structure and purpose of each folder

11. **Gigabit switch network:** ✅ COMPLETED
    Switch installed; Dell and Jetson have static Ethernet IPs via switch;
    Cyclone DDS and SSH updated to use Ethernet addresses;
    WiFi remains as automatic fallback

12. **Begin robot ROS 2 development:**
    SLAM/Mapping, Visual Odometry/Navigation, Object Detection/AI

13. **NoMachine display/mouse issue (new laptop — low priority):**
    Mouse coordinates offset by +1440px when viewing primary monitor
    Client-side fix partially works (correct image with value=1 in .nxs)
    but mouse lands on DP-3 instead of DP-2
    Server-side DisplayGeometry/PhysicalDisplays parameters ineffective
    Dell laptop works correctly without any special configuration

## KEY DECISIONS AND RATIONALE
- **X11 over Wayland (Dell):** Wayland has incomplete NVIDIA PRIME support
  for multi-monitor on Dell laptops
- **X11 over Wayland (IsaacUN):** Isaac Sim + NVIDIA driver more stable on X11
- **GDM3 over LightDM (IsaacUN):** GNOME Shell 46 requires GDM for
  ScreenShield — LightDM causes silent screen lock failure
- **CUDA 12.6 on Dell+Jetson:** Cross-compilation compatibility
- **CUDA 12.8 on IsaacUN:** Matches RTX 5080 (Blackwell) driver runtime
- **Cyclone DDS over Fast DDS (Dell+Jetson):** WiFi routers block multicast;
  Cyclone DDS unicast bypasses this reliably
- **FastDDS on IsaacUN:** Default for Jazzy; no need for unicast config
  since IsaacUN is not bridged to home network
- **ROS 2 Jazzy on IsaacUN:** Official Isaac Sim 5.x recommendation for
  Ubuntu 24.04; Humble packages have dependency conflicts on noble
- **ROS 2 Humble on Dell+Jetson:** LTS release matching Ubuntu 22.04;
  wire-compatible with Jazzy for standard message types
- **Static IPs via NetworkManager (Dell+Jetson):** Router not accessible
- **SSH port 44252 (0xACDC):** Reduces automated bot scanning; consistent
  across all three machines
- **libnvdla_compiler.so fix:** Extracted from nvidia-l4t-dla-compiler
  _36.4.1 .deb — known NVIDIA packaging omission in R36.4+ packages
- **dustynv/ros over nvcr.io/nvidia/l4t-base:** l4t-base tags beyond
  r36.2.0 not consistently published; dustynv/ros actively maintained
  with CUDA + cuDNN + TensorRT + ROS 2 pre-integrated
- **amd64 Dev Container:** Native speed for IntelliSense; ARM64
  emulation via QEMU is too slow for daily development
- **conda environments (IsaacUN):** Isaac Sim uses internal Python 3.11;
  system Python must remain unmodified; all dev work in conda envs
- **ZED SDK 5.2.2 for L4T R36.5:** Specific version matching
  JetPack 6.2.2 / CUDA 12.6 — fixes ZED2i positional tracking lock bug
- **SN65HVD230 for CAN transceiver:** 3.3V compatible with both Jetson
  and STM32F4xx; widely used, well-documented, available as breakout board
- **Flexible Cat-5/6 over DeviceNet cable:** At 125 kbps the 100Ω vs 120Ω
  impedance mismatch is negligible (reflection = ~2.5% of bit time); flex-rated
  Cat-5/6 is cost-effective and locally available. DeviceNet / LAPP UNITRONIC
  BUS CAN remain the correct choice if bitrate is ever raised significantly.
- **Bulgin 400 Series Buccaneer 8-pin IP68 connectors:** 17 units available;
  14 used (2 per middle node, 1 per end node), 3 spare. IP68 suits rover
  outdoor environment; robust latching for repeated maintenance connections.
- **Linear backbone with PCB pass-through topology:** Backbone passes through
  middle node PCBs via direct copper trace; short on-PCB branch to transceiver.
  Cleaner than T-tap with separate junction hardware; stub length is PCB-scale
  and irrelevant at 125 kbps.
- **125 kbps bus bitrate:** Sufficient for 8-node slow rover; leaves large
  margin for future expansion before cable/topology changes are needed.
- **Message-centric CAN ID priority:** Priority assigned per message type,
  not per node — safety-critical messages win regardless of source node
- **Canable (STM32 USB dongle) for bench CAN sniffer:** Appears as native
  SocketCAN interface on Linux; works with existing candump/cansend tools;
  clone versions (~$10-15 USD) available on AliExpress. Flash candlelight
  firmware (preferred over slcan) when it arrives.

## USEFUL COMMANDS REFERENCE

### Dell Host
- `ssh jetson` → connect to Jetson
- `jsync` → sync ~/ros2_ws/ to Jetson
- `jscp <file> talos@192.168.1.211:<path>` → copy file to Jetson
- `docker buildx build --platform linux/arm64 --tag <n> --load .`
- `cd ~/github/RobertUN && git pull origin main` → sync repository

### IsaacUN
- `conda activate ros2` → enter ROS 2 development environment
  (auto-sources /opt/ros/jazzy/setup.bash)
- `tmux new-session -s <n>` → start a new named tmux session
- `tmux ls` → list running tmux sessions
- `tmux attach -t <n>` → reattach to a session
- `~/isaac-sim/isaac-sim.sh` → launch Isaac Sim (run from base conda env)
- `nvcc --version` → verify CUDA toolkit

### Jetson — General
- `python3 ~/zed2i/test_zed.py` → test ZED camera (needs USB 3.x cable)
- `ros2 run demo_nodes_cpp talker` → test ROS 2
- `docker run --env NVIDIA_VISIBLE_DEVICES=all <image> nvcc --version`

### Jetson — CAN Bus
- `sudo busybox devmem 0x0c303018 w 0xc458 && sudo busybox devmem 0x0c303010 w 0xc400` → configure pinmux
- `sudo modprobe can && sudo modprobe can_raw && sudo modprobe mttcan` → load modules
- `sudo ip link set can0 type can bitrate 125000 && sudo ip link set can0 up` → bring up interface
- `candump can0` → monitor all CAN traffic
- `cansend can0 123#DEADBEEF` → send test frame
- `cat /proc/device-tree/bus@0/mttcan@c310000/status` → verify CAN hardware active

### VS Code (Dell)
- `Ctrl+Shift+P` → `Dev Containers: Reopen in Container` → open Dev Container
- `Ctrl+Shift+P` → `Remote-SSH: Connect to Host` → jetson → connect to Jetson
- `Ctrl+Shift+P` → `Dev Containers: Rebuild and Reopen` → only when Dockerfile changes

### tmux Quick Reference (IsaacUN)
- `Ctrl+B d` → detach from session (session keeps running)
- `Ctrl+B |` → split pane vertically
- `Ctrl+B -` → split pane horizontally
- `Ctrl+B h/j/k/l` → navigate between panes (vim-style)
- `Ctrl+B c` → new window (opens in same directory)
- `Ctrl+B r` → reload tmux config
- `tmux send-keys -t <session> "command" Enter` → send command to session headlessly

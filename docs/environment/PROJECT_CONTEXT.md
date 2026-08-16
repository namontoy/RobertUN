# Robotics Development Environment — Project Context Document
# Last updated: August 13, 2026 (**W3 acceptance criterion MET** — STM32 commands
# the SERVO42C to a target angle, confirmed against its own encoder, round-trip
# repeatability 1/10 of a microstep; three device/HAL traps found and recorded:
# the driver echoes every request, idle-line framing does not work on this link,
# and clearing UART error flags steals a byte from the DMA; earlier entry Aug 11:
# bus-load ramp to saturation — no FIFO overrun
# at any rate, main-loop period bounded under 1.6 ms; polled-vs-interrupt CAN RX
# recorded as an OPEN question, not settled by that result; earlier entry Aug 10:
# **W2 COMPLETE** — STM32F446RE heartbeat
# crossing a real 250 kbps bus to Orion, confirmed independently on the CANable,
# zero error counters on both ends; termination measured 59.79R; a floating
# CAN_RX pin identified as the cause of three different pre-wiring fault
# signatures; earlier entry Aug 9: W2 firmware written and building — DMA console
# on USART1, bxCAN driver at 250 kbps with accept-all filter, and a serial
# command interpreter for bench work; PLL divider record corrected to M=4/N=180
# and the MCO2 pin frequency clarified; earlier entry same day: W2 STM32
# toolchain established — STM32CubeIDE
# replaced by CubeMX + CMake + CubeCLT + VS Code on daedalus, full flash/debug
# chain verified against a WeAct STM32F446 Core Board; 8 MHz HSE → 180 MHz PLL
# reconfirmed under the new toolchain via MCO1/MCO2 + TIM3 blinky, which also
# validates the 45 MHz APB1 clock feeding bxCAN; CAN1 pin assignment and
# 250 kbps bit-timing values derived and recorded; module identity settled as a
# 3-bit DIP switch read at boot, making one binary serve all six modules;
# earlier entry Aug 6: CAN bus W1 COMPLETE — MKS CANable V2.0 Pro flashed to
# candleLight, SN65HVD230 wired to J17, two-node physical bus validated at
# 250 kbps with 500-frame load test, pinmux + can0 config made persistent via
# systemd; ALSO recovered the missing MKS SERVO42C UART protocol + torque
# characterization from the June 8, 2026 bench session, which had never been
# documented here; earlier entry Aug 4: forge workstation profile, Orion
# loopback self-test, mttcan-blacklist assumption corrected)
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

### Jetson Xavier NX (experimental AI inference node)
- **Machine:** NVIDIA Jetson Xavier NX Developer Kit, 8GB unified memory
- **GPU:** Volta architecture, 384 CUDA cores, compute capability 7.2 (sm_72)
- **Storage:** 16GB eMMC (OS) + 120GB Patriot Burst Elite SATA SSD via M.2-to-SATA adapter
- **Hostname:** xavier, username: talos
- **Role:** Experimental LLM inference node — natural language rover command interface

### forge (home LLM inference workstation)
- **Machine:** Home workstation, Universidad Nacional / RobertUN project
- **CPU:** Xeon E5-2680 v4 (28 threads)
- **RAM:** 64GB ECC DDR4
- **GPU:** NVIDIA GeForce RTX 3090 (24GB VRAM)
- **Storage:** 4TB WD Red HDD
- **OS:** Ubuntu 24.04
- **Hostname:** forge, username: talos
- **Role:** Primary local LLM inference machine

### Camera
- **Model:** Stereolabs ZED 2i stereo camera
- **Status:** ✅ FULLY OPERATIONAL
- **Cable:** Cable Matters USB-IF certified 10Gbps Gen2 — confirmed USB 3.00 (bcdUSB 3.00)
- **Serial number:** 32047842
- **Firmware:** 1523
- **Resolution:** HD1080 @ 30fps (grab resolution); publishes at 960x540 (downscale factor 2)
- **Calibration:** factory calibration file downloaded and cached on Jetson
- **Depth pipeline:** verified — NEURAL depth mode active, ~15% GPU utilization on Orin Nano
- **ROS 2 wrapper:** ✅ FULLY OPERATIONAL — zed-ros2-wrapper v5.2.2 built and running
- **Note:** Argus socket errors on camera close are expected/harmless on Jetson (known ZED SDK behavior)
- **Note:** PERFORMANCE depth mode deprecated in SDK 5.x — use NEURAL mode

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

### Home network (Dell laptop + Jetson + forge)
- **Network convention:** Ethernet IP = 200 + WiFi IP (easy to remember)
- **Gigabit switch:** installed, connects router + Dell + Jetson + forge via Ethernet

#### Dell laptop
- **Ethernet (primary):** 192.168.1.212 (static, interface enp4s0, metric 100)
- **WiFi (backup):** 192.168.1.12 (static, interface wlp5s0, metric 600)
- **WiFi network:** Gotham_Castle

#### Jetson Orin Nano
- **Ethernet (primary):** 192.168.1.211 (static, interface enP8p1s0, metric 100)
- **WiFi (backup):** 192.168.1.11 (static, interface wlP1p1s0, metric 600)
- **WiFi network:** Gotham_Castle

#### Jetson Xavier NX
- **Ethernet (primary):** 192.168.1.210 (static, interface eth0)
- **WiFi:** not used (wpa_supplicant disabled)

#### forge
- **Ethernet (primary):** 192.168.1.213 (static, interface enp7s0)
- **IPv6:** disabled on enp7s0

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
- **Jetson Orin Nano:** username=talos, hostname=orion
- **Jetson Xavier NX:** username=talos, hostname=xavier
- **IsaacUN:** username=talos, hostname=IsaacUN
- **forge:** username=talos, hostname=forge
- **SSH config on Dell:** ~/.ssh/config has:
  - 'Host orion' → 192.168.1.211:44252 (Ethernet)
  - 'Host xavier' → 192.168.1.210:44252 (Ethernet)
- **IsaacUN SSH:** systemd socket activation override at
  /etc/systemd/system/ssh.socket.d/override.conf (port 44252)
- **forge SSH:** systemd socket activation override at
  /etc/systemd/system/ssh.socket.d/override.conf (port 44252), same pattern as IsaacUN
- **Aliases on Dell:**
  - `orion` → ssh to Jetson Orin Nano (previously named `jetson`)
  - `xavier` → ssh to Jetson Xavier NX
  - `jsync` → rsync ~/ros2_ws/ to Orion with correct port
  - `jscp` → scp with correct port (-P 44252)
- **SSH auto-starts on Jetson boot:** confirmed (both orion and xavier)
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
- **Python:** 3.10.12 (system), managed via Miniconda3 (~/miniconda3/)
- **Philosophy:** always work inside conda environments, never system Python
  auto_activate_base = false (base environment does not activate automatically)
- **conda env `ros2`:** Python 3.10, ROS 2 Humble auto-sourced on activation
  - Activation script: ~/miniconda3/envs/ros2/etc/conda/activate.d/ros2.sh
    (sources /opt/ros/humble/setup.bash, sets ROS_DOMAIN_ID=0,
    sets RMW_IMPLEMENTATION=rmw_cyclonedds_cpp)
  - Packages: numpy 2.2.6, opencv-python 4.13, pyserial, transforms3d,
    scipy, matplotlib, pytest, pyyaml, rclpy 3.3.20, full ROS 2 Python stack,
    catkin-pkg, empy==3.3.4, lark
- **conda env `ml`:** Python 3.11, GPU-accelerated machine learning
  - Activation script: ~/miniconda3/envs/ml/etc/conda/activate.d/ml.sh
    (sets TORCH_HOME=~/.cache/torch, PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True,
    OMP_NUM_THREADS=8)
  - Packages: torch 2.11.0+cu126, torchvision 0.26.0+cu126,
    torchaudio 2.11.0+cu126, numpy 2.4.3, scipy 1.17.1, matplotlib,
    pandas, jupyter, scikit-learn 1.8.0, pyyaml
  - GPU confirmed: GTX 1650 Ti, CUDA 12.6, tensor ops verified on cuda:0
- **pyzed:** installed system-wide for ZED camera SDK access
- **ROS 2:** Humble Desktop (/opt/ros/humble/)
- **DDS:** Cyclone DDS (rmw_cyclonedds_cpp)
  - Config: ~/.ros/cyclone_dds.xml
  - Interface: enp4s0 (Ethernet, via switch)
  - Peers: 127.0.0.1 (loopback, for local composable nodes) + 192.168.1.211 (Jetson)
  - CRITICAL: 127.0.0.1 must be first peer — enables local ROS 2 service calls
    between processes on the same machine (required for composable node loading)
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
  - Launch: bash ~/isaac-sim/launch_isaacsim.bash (ALWAYS use this, not isaac-sim.sh directly)
  - Internal ROS 2: Jazzy with Python 3.11 (auto-loaded on Ubuntu 24.04)
  - ROS 2 bridge: ✅ FULLY OPERATIONAL — confirmed "rclpy loaded" at startup
  - Health check: confirmed working after GDM switch and CUDA install
  - Note: must be launched outside any conda environment
  - Note: screenshot path configured at ~/.local/share/ov/data/Kit/Isaac-Sim Full/5.1/user.config.json
- **ROS 2:** Jazzy Desktop (/opt/ros/jazzy/)
  - Jazzy is the recommended distro for Isaac Sim 5.x on Ubuntu 24.04
  - Jazzy and Humble are wire-compatible for standard message types
  - Auto-sourced in ~/.bashrc: source /opt/ros/jazzy/setup.bash
- **DDS:** FastDDS (default for Jazzy) — CycloneDDS not configured
  (IsaacUN not connected to Jetson/Dell network)
- **Docker:** 29.5.2 — installed via apt from Docker's official repository
  - Source: https://download.docker.com/linux/ubuntu
  - Keyring: /etc/apt/keyrings/docker.asc
  - Socket: /var/run/docker.sock (root:docker — correct apt behavior)
  - User talos is member of docker group — no sudo needed
  - NOTE: was previously installed via snap (removed May 2026) — apt version
    is properly configured and does not have socket permission issues
  - Local images:
    - isaac_sim_ros:ubuntu_22_jazzy (13.1GB) — NVIDIA ROS 2 build environment
    - isaac_sim_ros:ubuntu_24_jazzy (12GB) — NVIDIA ROS 2 build environment
    - osrf/ros:jazzy-desktop (5.61GB) — standard ROS 2 Jazzy desktop image
- **Snap:** ✅ COMPLETELY REMOVED (May 2026)
  - snapd removed and held: sudo apt-mark hold snapd
  - Blocked at repository level: /etc/apt/preferences.d/no-snapd
  - All snap directories removed: /snap, /var/snap, /var/lib/snapd, ~/snap
  - Firefox replaced with Mozilla official apt repository version (151.0.2)
  - GNOME desktop unaffected — runtime snaps were orphans, not GNOME itself
- **Python:** via Miniconda3 (/home/talos/miniconda3/)
  - Philosophy: always work inside conda environments, never system Python
  - auto_activate_base = false (base environment does not activate automatically)
  - conda env `ros2`: Python 3.12, ROS 2 Jazzy
    Packages: numpy, setuptools==79.0.1, catkin-pkg, empy==3.3.4, lark,
    colcon-common-extensions
  - NOTE: conda was broken by username change (namontoy → talos) —
    fixed May 2026 by updating shebang lines and profile.d scripts to
    use /home/talos/miniconda3 instead of /home/namontoy/miniconda3
  - NOTE: do NOT use pip --break-system-packages or install into ~/.local/
    alongside conda environments — causes setuptools conflicts during colcon builds
- **tmux:** installed, config at ~/.tmux.conf
  - Prefix: Ctrl+B (Ctrl+A as alternative)
  - Mouse support enabled
  - Split: Ctrl+B | (vertical), Ctrl+B - (horizontal)
  - Navigation: Ctrl+B h/j/k/l (vim-style)
  - History limit: 50000 lines
- **colcon:** installed (python3-colcon-common-extensions)
- **rosdep:** 0.26.0 (initialized)
- **Git:** not yet configured (pending task)

## ISAACUN — ISAAC SIM ROS 2 BRIDGE

### The Python version conflict and why it matters
Isaac Sim 5.x uses an internal Python 3.11 runtime. ROS 2 Jazzy on Ubuntu 24.04
is compiled against Python 3.12. These two Python environments are binary-
incompatible — rclpy .so extensions compiled for 3.12 cannot be loaded by 3.11.

The system ~/.bashrc sources /opt/ros/jazzy/setup.bash automatically, injecting
Python 3.12 ROS 2 paths into every terminal. If Isaac Sim is launched from such
a terminal, it inherits those paths and fails to initialize its ROS 2 bridge.

### The solution: launch_isaacsim.bash
File: ~/isaac-sim/launch_isaacsim.bash
Purpose: neutralizes the system ROS 2 environment, sources the Python 3.11
compatible workspace built by NVIDIA's Docker, then launches Isaac Sim.
ALWAYS use this script — never launch isaac-sim.sh directly.

```bash
#!/bin/bash
# Step 1: neutralize everything that /opt/ros/jazzy/setup.bash injected
source ~/isaac-sim/unros2.bash
# Step 2: source the Python 3.11 compatible ROS 2 workspace built by NVIDIA's Docker
source ~/Github/namontoy/IsaacSim-ros_workspaces/build_ws/jazzy/isaac_sim_ros_ws/install/setup.bash
# Step 3: set the DDS middleware that Isaac Sim's ROS 2 bridge requires
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Step 4: launch Isaac Sim
echo "Launching Isaac Sim..."
~/isaac-sim/isaac-sim.sh --/persistent/isaac/asset_root/default="/home/talos/isaacsim_assets/Assets/Isaac/5.0"
```

### unros2.bash — ROS 2 environment neutralizer
File: ~/isaac-sim/unros2.bash
Purpose: undoes everything /opt/ros/jazzy/setup.bash injects. Removes:
AMENT_PREFIX_PATH, CMAKE_PREFIX_PATH, GZ_CONFIG_PATH, LD_LIBRARY_PATH,
PYTHONPATH, ROS_DISTRO, ROS_VERSION, ROS_PYTHON_VERSION,
ROS_AUTOMATIC_DISCOVERY_RANGE. Strips /opt/ros/ entries from PATH.
ROS_DOMAIN_ID=0 is intentionally preserved (harmless for Isaac Sim).

### Confirmed successful bridge initialization (log output)
```
[isaacsim.ros2.bridge-4.12.4] startup
Attempting to load system rclpy
Could not import system rclpy: No module named 'rclpy'   ← expected, not an error
Attempting to load internal rclpy for ROS Distro: jazzy
rclpy loaded                                              ← bridge is operational
```

### NVIDIA IsaacSim-ros_workspaces — Docker build workflow
Repository: ~/Github/namontoy/IsaacSim-ros_workspaces/
Build script: build_ros.sh (uses Docker internally — do NOT fight native builds)

Build command (run once; uses cached Docker image if available):
```bash
cd ~/Github/namontoy/IsaacSim-ros_workspaces
./build_ros.sh -d jazzy -v 24.04
```

Output: build_ws/jazzy/
  - jazzy_ws/         — core ROS 2 packages (Python 3.11 compiled)
  - isaac_sim_ros_ws/ — Isaac Sim specific packages (install/ sourced at launch)

Workspace source packages (jazzy_ws/src/):
  ackermann_control, custom_message, humanoid_locomotion_policy_example,
  isaac_compressed_image_decoder, isaac_ros2_messages, isaacsim,
  isaac_tutorials, moveit, navigation (carter_navigation,
  isaac_ros_navigation_goal, iw_hub_navigation), Universal_Robots_ROS2_Description

CRITICAL: navigation2 is NOT included as source — it is a runtime dependency.
The Ubuntu 24.04 apt package for ros-jazzy-navigation2 has a packaging gap
(libcom-err2/libbsd0/libmd0 version mismatch after security updates). This is
an upstream Ubuntu issue — do NOT attempt to fight it with apt. The Docker
build handles all dependencies correctly inside its own clean environment.

### Two-terminal workflow
Terminal A — Isaac Sim:
  bash ~/isaac-sim/launch_isaacsim.bash
  (never source /opt/ros/jazzy/setup.bash in this terminal)

Terminal B — External ROS 2 nodes, RViz, Nav2, tools:
  source /opt/ros/jazzy/setup.bash
  source ~/Github/namontoy/IsaacSim-ros_workspaces/build_ws/jazzy/isaac_sim_ros_ws/install/setup.bash
  ros2 launch ...  (or any ROS 2 command)

Communication between the two terminals happens via DDS topics — no shared
Python environment required. Isaac Sim acts like a real robot on the network.
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
  - nvidia-jetpack and nvidia-jetpack-dev installed via apt (required for
    ZED ROS 2 wrapper CMake to find CUDA_TOOLKIT_ROOT_DIR correctly)
- **cuDNN:** installed via apt
- **Python:** 3.10.12
- **ROS 2:** Humble Base (/opt/ros/humble/)
- **rosdep:** initialized (sudo rosdep init + rosdep update completed)
- **DDS:** Cyclone DDS (rmw_cyclonedds_cpp)
  - Config: ~/.ros/cyclone_dds.xml
  - Interface: enP8p1s0 (Ethernet, via switch)
  - Peers: 127.0.0.1 (loopback) + 192.168.1.212 (Dell Ethernet address)
  - CRITICAL: 127.0.0.1 MUST be the first peer in the list. Without the
    loopback peer, ROS 2 composable node loading (load_node service calls)
    fails silently — the component_container_isolated starts but the ZED
    component is never injected into it, causing an infinite hang with zero
    CPU and zero GPU activity. This was the root cause of all ZED launch
    failures on first installation.
- **jtop:** installed (sudo pip3 install jetson-stats)
  - Launch: jtop (requires fresh login session after first install)
  - Note: "JetPack not detected" warning in red is cosmetic — jtop's version
    detection heuristic doesn't support JetPack 6.x file layout yet, but all
    hardware monitoring functions work correctly
  - GPU utilization visible as GR3D bar; ~15% during ZED NEURAL depth mode
- **Docker:** 29.3.0
  - NVIDIA Container Runtime: DEFAULT runtime in /etc/docker/daemon.json
  - Local images:
    - nvcr.io/nvidia/l4t-jetpack:r36.4.0
    - dustynv/ros:humble-ros-base-l4t-r36.3.0
    - ubuntu:22.04
    - hello-world:latest
- **ZED SDK:** 5.2.2 installed at /usr/local/zed/
  - Python API: pyzed 5.2.2 confirmed working
  - TensorRT models optimized and cached at /usr/local/zed/resources/:
    Neural Depth (5.3), Neural Light Depth (5.2), Neural Plus Depth,
    Object Detection (3 tiers), Person ReID, Skeleton Body18/38, Person Head
  - Camera: ✅ FULLY OPERATIONAL (USB 3.00, depth pipeline verified)
  - Test script: ~/zed2i/test_zed.py — confirmed working
  - Calibration file: downloaded and cached for S/N 32047842
- **ZED ROS 2 Wrapper:** ✅ FULLY OPERATIONAL
  - Repository: ~/ros2_ws/src/zed-ros2-wrapper (master branch, v5.2.2)
  - Built with: colcon build --symlink-install --packages-skip zed_debug
    --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)
  - Binary dependencies installed via apt: ros-humble-zed-msgs,
    ros-humble-zed-description, ros-humble-robot-localization,
    ros-humble-image-transport-plugins, ros-humble-backward-ros,
    ros-humble-nmea-msgs, ros-humble-geographic-msgs, nlohmann-json3-dev
  - Depth mode: NEURAL (changed from deprecated PERFORMANCE)
  - Workspace sourced in ~/.bashrc: source ~/ros2_ws/install/local_setup.bash
  - Verified topics: RGB, depth, point cloud, IMU (100Hz), odometry, pose
  - GPU utilization: ~15% GR3D with NEURAL depth at HD1080/30fps
- **Git:** ✅ CONFIGURED (April 3, 2026)
  - user.name: namontoy, user.email: namontoy@unal.edu.co
  - GitHub SSH key: ED25519 at ~/.ssh/id_ed25519_github
  - SSH config: ~/.ssh/config → Host github.com uses id_ed25519_github
  - Key registered on GitHub as "Jetson Orin Nano - orion"
  - Authentication verified: ssh -T git@github.com ✅
- **Performance mode:** sudo nvpmodel -m 0 (MAXN) + sudo jetson_clocks
  - Not persistent across reboots — run after each reboot before launching ZED

## JETSON XAVIER NX — JetPack 5.1.6 / L4T R35.6.x, aarch64
- **OS:** Ubuntu 20.04 LTS
- **CUDA:** 11.4 (/usr/local/cuda-11.4/)
- **TensorRT:** 8.5.2
- **cuDNN:** 8.6.0
- **Python:** 3.8.x (system)
- **JetPack status:** JetPack 5.x is the final supported line for Xavier NX —
  JetPack 6 is Orin-only. JetPack 5.1.6 is the latest and last major release.
- **Boot target:** multi-user (headless) — desktop disabled to save ~300MB RAM
  - gdm3 still installed but disabled; restore with: sudo systemctl set-default graphical.target
- **Power mode:** MODE_20W_6CORE (mode 8) — all 6 Carmel cores, GPU @ 1100 MHz,
  EMC @ 1866 MHz (~59.7 GB/s memory bandwidth)
  - jetson_clocks enabled as systemd service: /etc/systemd/system/jetson_clocks.service
- **Disabled services:** bluetooth, ModemManager, avahi-daemon, rpcbind,
  rtkit-daemon, kerneloops, apport, openvpn, wpa_supplicant, pulseaudio

### Xavier NX Storage
- **eMMC:** 16GB (OS, CUDA stack, llama.cpp binaries)
- **SATA SSD:** Patriot Burst Elite 120GB via M.2-to-SATA adapter (ASM1166 bridge chip)
  - Mounted at: /data (EXT4, label=data, nofail in /etc/fstab)
  - Symlink: ~/github → /data/github; models at /data/models/
  - Performance: 525 MB/s read, 292 MB/s write (through ASM1166 PCIe bridge)
  - Power solution: 5V injected from GPIO header pin 2 (ATX red wire)
    GPIO pin 2 → SATA power pin 7 (5V); GPIO pins 6+14 → SATA pins 4+9 (GND)
    M.2 slot provides 3.3V on SATA pins 1-3 automatically
    WARNING: connect wiring with board POWERED OFF — GPIO 5V is always live
- **Swap:** 8GB file at /data/swapfile (SSD-backed, required for NvMap VMM)
  - zram (4×854MB) retained alongside — both active simultaneously

### Xavier NX LLM Inference Stack
- **Engine:** llama.cpp (built from source, CUDA-enabled)
  - Repository: ~/github/llama.cpp
  - Build flags: -DGGML_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=72 -DCMAKE_BUILD_TYPE=Release
  - GPU: Volta sm_72 confirmed working; CUDA0: Xavier (6833 MiB)
- **HuggingFace CLI:** huggingface-hub 0.28.1 (last version without hf-xet dependency)
  - hf-xet fails to build on aarch64 Ubuntu 20.04 — use 0.28.1 specifically
- **NvMap VMM ceiling:** ~2.5GB quantized weights — hard architectural limit on
  Volta/JetPack 5.x. Models larger than ~2.5GB fail at load time regardless of
  available RAM or batch size. This is a silicon-level constraint, not solvable
  in software. Q4_K_M is the ONLY viable quantization for 3B-4B models.

### Xavier NX Confirmed Working Models (full GPU offload, -ngl 99)
| Model | File | Size | tg (t/s) | Special flags |
|-------|------|------|----------|---------------|
| Qwen2.5-3B-Instruct | Qwen2.5-3B-Instruct-Q4_K_M.gguf | 1.79 GB | 18.83 | none |
| Qwen2.5-Coder-3B-Instruct | Qwen2.5-Coder-3B-Instruct-Q4_K_M.gguf | 1.79 GB | ~18 | none |
| Phi-3.5-mini-instruct | Phi-3.5-mini-instruct-Q4_K_M.gguf | 2.23 GB | 15.63 | none |
| gemma-3-4b-it | gemma-3-4b-it-Q4_K_M.gguf | 2.31 GB | 14.21 | -b 512 -ub 256 |

Failed models (NvMap VMM ceiling): Qwen2.5-7B (4.36GB), Qwen2.5-Coder-7B (4.36GB)

### Xavier NX Rover Command Interface (primary use case)
- Model: Qwen2.5-3B-Instruct-Q4_K_M (fastest, best JSON output)
- Temperature: 0.3 (deterministic structured output)
- Context: 32768 tokens
- Valid actions: move_forward, move_backward, turn_left, turn_right, stop,
  take_photo, analyze_sample
- Evaluation framework: /data/rover_eval/test_cases.json (12 test cases,
  3 categories: precise, ambiguous, invalid commands)

### Xavier NX GPU Cleanup Script
~/cleanup_gpu.sh — resets GPU state after crashed model loads (4 steps):
kills /dev/nvhost-gpu and /dev/nvmap holders, drops page cache,
reloads nvgpu kernel module, displays memory status.
NOTE: if NvMap errors persist after script, full reboot required.

## FORGE — Ubuntu 24.04 x86_64
- **CPU:** Xeon E5-2680 v4 (28 threads)
- **RAM:** 64GB ECC DDR4
- **GPU:** NVIDIA GeForce RTX 3090 (24GB VRAM)
- **Storage:** 4TB WD Red HDD
- **Role:** Primary local LLM inference machine for the project
- **Setup status:** complete through Phase 8
  - Snap fully removed (pinned via /etc/apt/preferences.d/no-snapd, same
    approach as IsaacUN)
  - Firefox installed from Mozilla apt repo (priority 1000 pin)
  - Docker CE + NVIDIA Container Toolkit installed
  - Miniconda3 installed, `ml` conda env (Python 3.11)
  - llama.cpp built from source with CUDA backend
    (-DGGML_CUDA=ON, -DCMAKE_CUDA_ARCHITECTURES=86)
  - SSH on port 44252 via systemd socket override (see SSH CONFIGURATION)
  - IPv6 disabled on enp7s0

### forge Models (~/models)
- Qwen2.5-3B Q4_K_M
- Qwen2.5-32B Q4_K_M
- Qwen2.5-32B Q3_K_M

### forge RTX 3090 Benchmarks
| Model | Quant | Generation | Prompt processing | VRAM |
|-------|-------|------------|--------------------|------|
| Qwen2.5-32B | Q4_K_M | ~35.7 t/s | 531.8 t/s | ~22.7GB |
| Qwen2.5-32B | Q3_K_M (-c 8192) | — | — | ~17.4GB |

Note: full 32K context on Q4_K_M pushes close to the 24GB VRAM ceiling —
Q3_K_M is the safer choice when the full context window is needed.

## ZED ROS 2 WRAPPER — CRITICAL OPERATIONAL NOTES

### The Cyclone DDS loopback fix (ROOT CAUSE OF ALL LAUNCH FAILURES)
The zed-ros2-wrapper uses ROS 2 composable nodes. The launch system must call
a `/zed/zed_container/_container/load_node` service to inject the ZED camera
component into the component_container_isolated process. Without 127.0.0.1 as
a Cyclone DDS peer, this service call is routed externally and never reaches
the container process sitting on the same machine — causing a silent infinite
hang with zero CPU and zero GPU activity. Fix: add 127.0.0.1 as the FIRST
peer in ~/.ros/cyclone_dds.xml on every machine running composable nodes.

### Correct cyclone_dds.xml format (Jetson and Dell)
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="enP8p1s0" multicast="false"/>
      </Interfaces>
    </General>
    <Discovery>
      <Peers>
        <Peer address="127.0.0.1"/>
        <Peer address="192.168.1.212"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```
(Use enp4s0 for Dell, enP8p1s0 for Jetson. Second peer is the remote machine.)

### Orphaned process safety rule (CRITICAL)
Always run `ps aux | grep component_container` before relaunching the ZED node.
Orphaned component_container_isolated and robot_state_publisher processes from
previous sessions will silently compete for the USB camera handle and cause
deadlocks that are indistinguishable from other failures. If any survivors are
found, kill them before relaunching:
```bash
pkill -f component_container_isolated && pkill -f robot_state_publisher
sleep 2
ps aux | grep -E "component_container|robot_state" | grep -v grep
# Output should be empty before proceeding
```

### Correct launch procedure
```bash
# 1. Check for orphaned processes (mandatory)
ps aux | grep component_container | grep -v grep

# 2. Set performance mode (after every reboot)
sudo nvpmodel -m 0 && sudo jetson_clocks

# 3. Source full environment
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash

# 4. Launch with verbose SDK output
export ZED_SDK_VERBOSE=1
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

### What a successful launch looks like
After the URDF segments load, the component loads in ~1 second (thanks to the
loopback fix), then the SDK prints:
- "Camera successfully opened"
- "Serial Number: S/N 32047842"
- "Depth mode selected: NEURAL"
- "=== zed started ===" — node is fully operational at this point
- "Starting Positional Tracking" — visual-inertial odometry initializing
- "Gravity alignment issues detected. Recomputing alignment..." — harmless,
  IMU is computing gravity vector from accelerometer (one-time at startup)

### Verifying live data (second terminal)
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash
ros2 node list          # Should show 5 nodes including /zed/zed_node
ros2 topic list | grep zed   # Should show 21 topics
ros2 topic echo /zed/zed_node/imu/data --once   # Verify real sensor data
```

### Known harmless warnings in launch output
- "Invalid configuration: enable_ipc:=true with debug.disable_nitros:=false"
  — NITROS is not installed; warning is benign, IPC is forced off automatically
- "selected interface enP8p1s0 is not multicast-capable: disabling multicast"
  — Expected with Cyclone DDS unicast configuration
- Argus socket errors on camera close — known ZED SDK behavior on Jetson

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
- **RS pin (pin 8) — mode control, verified on the actual breakout Aug 6, 2026**
  - Three modes: RS→GND = high-speed; RS→VCC = standby (driver DISABLED,
    receives but never transmits); RS→GND via resistor = slope control
  - **The breakout in use has 10 kΩ from RS to GND → slope-control mode**
    (~15 V/µs slew rate, deliberate EMI reduction, standard on these blue boards)
  - Non-issue at 250 kbps: edges take ~0.1 µs against a 4 µs bit time
  - CONSTRAINT IF BITRATE IS EVER RAISED — the 10 kΩ becomes a real limit;
    check this before assuming a higher bitrate is achievable
  - A FLOATING RS pin is undefined and commonly lands in standby. Symptom:
    node looks perfectly healthy (ERROR-ACTIVE) but nothing ever reaches
    the wire. Always verify RS before debugging wiring.

### STM32 CAN peripheral
- **STM32F4xx uses bxCAN peripheral** (Basic Extended CAN)
  - Full layer 2 implementation in hardware: frame construction, arbitration,
    CRC, ACK, fault confinement, acceptance filters — all in silicon
  - bxCAN outputs logic-level CAN_TX / CAN_RX to the SN65HVD230 transceiver
  - No built-in transceiver — external IC always required (same as Jetson)
- **CAN1 pins on the WeAct board: PB9 (TX) / PB8 (RX).** The alternate CAN1
  mapping PA11/PA12 is wired to the board's USB-C connector and must not be
  used. Use CAN1, not CAN2 — on the F446 CAN2 is a slave peripheral that cannot
  run without CAN1's clock enabled anyway.
- **bxCAN SJW maxes out at 4 tq** (the register field is 2 bits wide), and must
  also be <= BS2. Orion's `sjw 16` therefore does *not* transfer literally to
  the STM32. The rule that carries across is **"set SJW explicitly on every
  node, at the highest value that node's hardware allows"** — not the number 16.

#### Bit timing @ 250 kbps (derived Aug 9, 2026, from 8 MHz HSE)

| Parameter | Value | Notes |
|---|---|---|
| HSE | 8 MHz | crystal on WeAct board, scope-verified |
| SYSCLK | 180 MHz | PLL M=4, N=180, P=2 — see divider note below |
| APB1 (bxCAN clock) | 45 MHz | /4 |
| Prescaler (BRP) | 12 | tq = 266.67 ns |
| Bit Segment 1 (BS1) | 12 tq | HAL: `CAN_BS1_12TQ` |
| Bit Segment 2 (BS2) | 2 tq | HAL: `CAN_BS2_2TQ` |
| SJW | 2 tq | HAL: `CAN_SJW_2TQ` — hardware max here (<= BS2) |
| Total | 15 tq | 15 x 266.67 ns = 4.0 us = 250 kbps |
| Sample point | 86.7% | (1+12)/15 — closest achievable to Orion's 87.5% |

**Why 180 MHz and not the conventional 168 MHz:** 45 MHz on APB1 divides into
250 kbps with a better sample point than 42 MHz does. Consequence: USB's 48 MHz
must come from PLLSAI rather than PLLQ. Irrelevant unless USB is ever needed.

**PLL dividers — record corrected Aug 9, 2026.** An earlier revision of this
table recorded M=8, N=360. Both pairs reach 180 MHz, but they are not
equivalent: M=8 gives a 1 MHz PLL input, M=4 gives 2 MHz. RM0390 requires the
PLL input to be 0.95-2.1 MHz and explicitly recommends **2 MHz to limit PLL
jitter**. The firmware uses **M=4, N=180, P=2**, which is the better of the two
and the pair to replicate to the other five boards in W7. Verify against
`SystemClock_Config()` rather than this table if they ever disagree again.

**Oscillator tolerance check.** df <= SJW/(20 x NBT): STM32 = 2/(20x15) = 0.67%;
Orion = 16/(20x200) = 0.40%. Orion is the binding node. Two crystals at +/-30 ppm
(0.003%) sit two orders of magnitude inside that margin. This is the calculation
that rules out HSI (+/-1%) — see the W1 debrief in the roadmap.

- **bxCAN receives nothing until an acceptance filter is configured AND
  activated.** Default state is all filters disabled. Presents as "TX works
  perfectly, RX is dead" and gets misdiagnosed as wiring or termination. For
  bring-up use filter bank 0, mask mode, ID 0x000 / mask 0x000 (accept
  everything); narrow it once the CAN ID table is real.

### Physical bus
- **Selected bitrate: 250 kbps**
  - Rover moves slowly, 8 nodes; 250 kbps provides comfortable headroom for
    CANopen protocol overhead (PDO, SDO, NMT, heartbeat traffic) across 8 nodes
  - At 250 kbps, bit time = 4 µs; reflections from impedance mismatch arrive in
    ~200 ns (5% of bit time) — still well within CAN bit timing tolerance
  - This makes 120Ω vs 100Ω cable impedance mismatch irrelevant for this application
  - All nodes must be configured identically — mismatch = arbitration failure
  - Note: 125 kbps also viable if bus load headroom is needed later

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
  - **Re-measure every time a node joins the bus.** The SN65HVD230 breakouts
    ship with 120R already populated, so adding a node adds a terminator
    unless one is lifted. Two in parallel measure ~60R across CANH-CANL with
    power off; three measure ~40R, which is below what the transceivers can
    reliably pull dominant. The measurement is the check — do not infer it.
  - **Measured Aug 10, 2026 with Orion + CANable + STM32 all connected:
    59.79R.** Two 120R in parallel, i.e. exactly two terminators on the bus.

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
- Layers 3–7: not defined by CAN itself; implemented at application layer
  - **Selected protocol: CANopen with CANopenNode**
    - CiA 402 motor control profile for wheel nodes (STM32)
    - ros2_canopen (Fraunhofer IPA / ROS-Industrial) for Jetson integration
    - See KEY DECISIONS for full rationale
  - Plain CAN message ID hierarchy (designed in session) serves as reference
    for understanding arbitration and filtering — not used in final rover

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
**Check, don't assume:** mttcan is blacklisted on SOME L4T builds via
/etc/modprobe.d/denylist-mttcan.conf — but it is **NOT blacklisted on Orion's
R36.5 build** (verified Aug 4, 2026). Check per-machine first:
```bash
cat /etc/modprobe.d/denylist-mttcan.conf 2>/dev/null || echo "no blacklist file found"
```
If a blacklist file does exist:
```bash
sudo rm /etc/modprobe.d/denylist-mttcan.conf
echo "mttcan" | sudo tee /etc/modules-load.d/mttcan.conf
```

### Bring up CAN interface (SocketCAN)
```bash
# Classical CAN at 250 kbps (selected rover bitrate) — ALWAYS set sjw explicitly:
sudo ip link set can0 type can bitrate 250000 sjw 16 restart-ms 100
sudo ip link set can0 up

# CAN FD (500 kbps nominal / 1 Mbps data):
sudo ip link set can0 up type can bitrate 500000 dbitrate 1000000 berr-reporting on fd on
```

### Test with can-utils
```bash
candump can0                        # receive all frames
cansend can0 123#ABCDABCD           # send a frame
```

### Loopback self-test (controller-internal — does NOT touch the pins)
```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 250000 loopback on
sudo ip link set can0 up
candump can0 &
cansend can0 123#ABCDABCD
```
**What this does and does not prove.** `loopback on` is a controller test mode —
the signal never reaches J17. It validates kernel module → driver → SocketCAN →
can-utils only.

**There is NO useful "pins-only, no transceiver" test.** Physically shorting
TX to RX on J17 adds nothing: SocketCAN's `loopback` flag bypasses the pins,
and with `loopback off` a lone node shorting its own TX to RX cannot self-ACK
(it sends recessive in the ACK slot and samples its own recessive), so it
error-frames and retransmits forever. The pinmux is validated by (a) devmem
register readback and (b) a real two-node bus test. Do not budget time for a
short-pin test.

**Also beware SocketCAN's software local echo**: `candump` displays frames you
sent even if the controller is dead. A frame on screen proves almost nothing on
its own — check the driver's RX packet counter via `ip -s link show can0`.

### Verify device tree node is active
```bash
cat /proc/device-tree/bus@0/mttcan@c310000/status   # should print: okay
```
Note: path prefix is bus@0/ on JetPack 6.x (different from JetPack 5.x)

### Known issues / gotchas
- mttcan blacklist is the #1 cause of "CAN doesn't work" — always check first
  (see verification log below: NOT assumed blanket-true, check per-machine)
- Pinmux resets on reboot when using devmem method — solved by
  robertun-can0.service (see "CAN BUS — PERSISTENCE" below)
- **SJW must ALWAYS be set explicitly — use `sjw 16` on this network.**
  Driver defaults differ: mttcan defaults to 1, gs_usb (CANable) to 8.
  At 250 kbps on Orion the bit is 200 tq, so sjw 1 gives only 0.5% of a bit
  time of resync authority ≈ 250 ppm combined oscillator tolerance between any
  two nodes. Crystal-clocked nodes pass; an STM32F446RE running off the
  internal HSI RC oscillator (±1% over temperature) will NOT, and it fails as
  random bit errors under load — a bug that looks like bad firmware.
  sjw must stay ≤ min(phase-seg1, phase-seg2); 16 is safe (limit is 25).
- CAN FD at 5 Mbps may need TDCR tuning via sysfs

### Verification log — Aug 4, 2026 (Orion, pre-wiring)
Confirmed via direct SSH session, before any physical CAN wiring or soldering
(J17 header not yet installed at time of this check):
- L4T release confirmed: R36.5 (matches expected build)
- `busybox` was NOT installed by default on this Orion image (`can-utils` was
  already present). Installed via:
  `sudo apt-get install -y busybox can-utils`
- **Correction to the assumption above:** `/etc/modprobe.d/denylist-mttcan.conf`
  does NOT exist on this build — mttcan is not blacklisted here. Don't assume
  the blacklist is present; check first with:
  `cat /etc/modprobe.d/denylist-mttcan.conf 2>/dev/null || echo "no blacklist file found"`
- `mttcan` module already loaded (`lsmod`): mttcan, nvpps (dep), can_dev (dep) —
  0 references at check time, not yet claimed by an active can0 bring-up
- Device tree node confirmed active:
  `cat /proc/device-tree/bus@0/mttcan@c310000/status` → `okay`
- `can0` netdev already present in `ls /sys/class/net` — module + active DT
  node is sufficient to create the netdev; this does NOT mean pins are
  wired/muxed yet, so don't over-read this as "CAN is ready"
- **Loopback self-test PASSED** (controller-internal, no transceiver/pins needed):
  ```bash
  sudo ip link set can0 down
  sudo ip link set can0 type can bitrate 250000 loopback on
  sudo ip link set can0 up
  candump can0 &
  cansend can0 123#ABCDABCD
  ```
  Result: frame received twice in candump output (`AB CD AB CD`) — this is
  expected SocketCAN behavior (local echo of sent frames + controller-level
  loopback echo, both default-on), not a duplicate or an error.
- `can0` brought back down after the test (`sudo ip link set can0 down`) to
  leave a clean state before physical wiring begins
- **Status: entire software chain (kernel module → driver → SocketCAN →
  can-utils) verified working on Orion.** Remaining for real bus operation:
  solder J17 header, wire SN65HVD230 transceiver, apply devmem pinmux, bring
  up real can0 at 250 kbps, confirm with a second physical node (MKS CANable
  V2.0 Pro via daedalus), then make pinmux + module load persistent via a
  systemd service (devmem resets on every reboot).
  **→ ALL OF THE ABOVE COMPLETED Aug 6, 2026 — see log at end of this section.**

## CAN BUS — MKS CANable V2.0 Pro (daedalus debug node)

### Role
Development sniffer / second bus node. **Not part of the permanent rover
architecture** — it is not autonomous, it needs daedalus driving it. But the
bus cannot tell its frames from a real STM32 node's frames, which makes it an
**independent witness**: when a node misbehaves in W2–W9, `candump` on daedalus
shows what is actually on the wire, rather than what Orion's stack thinks is
on the wire.

### Hardware
- STM32G431C8T6 @ 170 MHz, USB-C, onboard 120Ω termination selectable by jumper
- **Ships from the factory with slcan firmware** (`16d0:117e`, appears as
  /dev/ttyACM0, needs the slcand daemon to reach SocketCAN)
- **Flashed to candleLight** → `1d50:606f`, native `can0` via the in-kernel
  `gs_usb` driver. Same tools, same commands, same mental model as Orion's
  mttcan — one thing to learn instead of two.
- **Reset quirk:** resetting the board too quickly causes boot problems. Always
  leave it unpowered ~2 seconds on reset. (Permanent fix requires reprogramming
  the STM32 option bytes over SWD — not done, not needed.)

### candleLight flashing procedure (performed Aug 6, 2026)
```bash
# 1. Identify current firmware BEFORE flashing
lsusb                          # 16d0:117e = slcan | 1d50:606f = candleLight already
ls /dev/ttyACM*                # present = slcan
ip -brief link show type can   # a can0 here = candleLight already

# 2. Get dfu-util and the firmware
sudo apt install -y dfu-util
mkdir -p ~/github/canable_fw && cd ~/github/canable_fw
wget "https://raw.githubusercontent.com/makerbase-mks/CANable-MKS/main/Firmware/CANable%20V2.0/candlelight.zip"
unzip -o candlelight.zip        # → candlelight/canable2_fw-ba6b1dd.bin

# 3. Unplug, install BOOT jumper, wait 2 s, replug. Verify DFU:
lsusb | grep -i "0483:df11"     # STM Device in DFU Mode
sudo dfu-util -l                # confirm alt=0 is "@Internal Flash /0x08000000/64*02Kg"

# 4. Flash
sudo dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave \
  -D ~/github/canable_fw/candlelight/canable2_fw-ba6b1dd.bin

# 5. Unplug, REMOVE BOOT JUMPER, wait 2 s, replug
```
- Firmware source: `makerbase-mks/CANable-MKS`, `Firmware/CANable V2.0/candlelight.zip`
  (use the `.bin`; the `.dfu` is Windows DfuSe format). The `.bin` has no
  embedded address, which is why `-s 0x08000000` is passed explicitly.
- **Two alarming-but-harmless messages are EXPECTED:**
  - `Invalid DFU suffix signature` — raw .bin has no DFU suffix
  - `Error during download get_status` **after** `File downloaded successfully`
    — `:leave` detached the device, so the final status query hit nothing
  - The line that matters is `File downloaded successfully`
- Confirm success: `1d50:606f`, dmesg shows `Product: canable2 gs_usb`,
  `Manufacturer: canable.io`, `gs_usb` bound, and a `can0` netdev appears.
- The generic canable2 firmware works fine on the Pro variant (the Pro differs
  only in its isolated transceiver, not the MCU).
- Fallback if the official build misbehaves: third-party "CANable 2.5" firmware.

### Bring-up on daedalus
```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 250000 sjw 16 loopback off
sudo ip link set can0 up
```
**Always clear `loopback off` explicitly.** The loopback flag persists across a
down/up cycle. A node left in loopback mode on a real bus happily shows you its
own frames while never driving the wire — you will conclude the cable is broken.

## CAN BUS — PERSISTENCE (Orion, robertun-can0.service)

Two things reset on every boot: the devmem pinmux writes, and the can0
bitrate/sjw configuration. An intermittent CAN setup will masquerade as STM32
firmware bugs in W2–W9, so this is not optional.

### /usr/local/sbin/robertun-can0-setup.sh
```bash
#!/bin/bash
# RobertUN — Orion CAN0 bring-up (pinmux + interface config)
set -e
BB=/usr/bin/busybox

# 1. Pinmux: CAN0_DIN (RX) and CAN0_DOUT (TX) on J17
$BB devmem 0x0c303018 w 0xc458
$BB devmem 0x0c303010 w 0xc400

# 2. Verify the writes actually landed
RX=$($BB devmem 0x0c303018)
TX=$($BB devmem 0x0c303010)
[ "$RX" = "0x0000C458" ] || { echo "pinmux RX readback failed: $RX"; exit 1; }
[ "$TX" = "0x0000C400" ] || { echo "pinmux TX readback failed: $TX"; exit 1; }

# 3. Ensure driver is loaded (no-op if device tree already bound it)
modprobe mttcan || true

# 4. Wait for the netdev — probe can lag the pinmux write
for i in $(seq 1 50); do
    [ -d /sys/class/net/can0 ] && break
    sleep 0.1
done
[ -d /sys/class/net/can0 ] || { echo "can0 never appeared"; exit 1; }

# 5. Configure and bring up: 250 kbps, sjw 16, auto-recover from BUS-OFF
ip link set can0 down || true
ip link set can0 type can bitrate 250000 sjw 16 restart-ms 100
ip link set can0 up

echo "can0 up: 250 kbps, sjw 16, restart-ms 100"
```

### /etc/systemd/system/robertun-can0.service
```ini
[Unit]
Description=RobertUN CAN0 bring-up (J17 pinmux + can0 config)
After=multi-user.target
Wants=network.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/local/sbin/robertun-can0-setup.sh
ExecStop=/sbin/ip link set can0 down

[Install]
WantedBy=multi-user.target
```
```bash
sudo chmod +x /usr/local/sbin/robertun-can0-setup.sh
sudo systemctl daemon-reload
sudo systemctl enable --now robertun-can0.service
```

### Design rationale (don't "simplify" these away)
- **Readback verification (step 2)** turns a silent failure into a loud one. If
  a future L4T update changes the register layout, you get an explicit error in
  `journalctl` instead of a CAN bus that mysteriously stopped working.
- **Wait loop (step 4)** handles the pinmux write completing before mttcan has
  probed. Confirmed necessary: manual runs used 21 ms CPU, the boot run used
  25 ms — the loop actually iterated at boot. Without it, an intermittent boot
  failure that always succeeds when run by hand.
- **`restart-ms 100`** auto-recovers from BUS-OFF instead of leaving the
  interface dead until manual intervention. A misbehaving STM32 node can drive
  Orion to BUS-OFF during W2–W9, and on Dec 10 an unrecoverable bus is a failed
  demo. Faults remain fully visible in the error counters. Use `restart-ms 0`
  only if you want a BUS-OFF to stay latched for debugging.
- **`Type=oneshot` + `RemainAfterExit=yes`** so systemd reports the service
  *active* rather than dead after the script exits. Required for any future
  unit (e.g. a ROS 2 launch service) to order itself with `After=robertun-can0.service`.
- **`/usr/bin/busybox` absolute path** — systemd units run with a minimal PATH.

### Verification log — Aug 6, 2026 (Orion ↔ daedalus, full physical bus)

**Physical setup as built:**
- J17 header soldered; SN65HVD230 breakout wired CAN_TX→CTX, CAN_RX→CRX, 3V3, GND
- 3 m flexible Cat-6 UTP. CANH = orange, CANL = white/orange — **same twisted
  pair** (the twist is what provides common-mode rejection; splitting the two
  signals across different pairs is the classic mistake). GND = green +
  white/green doubled.
- Only CANH, CANL, GND cross between the two devices. **No power is shared** —
  the CANable is USB-powered from daedalus, the transceiver from Orion.
- Termination: CANable Pro onboard jumper installed + 120Ω already populated on
  the SN65HVD230 breakout = exactly two. **Measured 59.4 Ω across CANH–CANL
  with power off** (two 120Ω in parallel) — this measurement is the check.
- Propagation over 3 m ≈ 15 ns against a 1.74 µs prop-seg — negligible.

**Bit timing achieved (both nodes land identically from different clocks):**
| | Orion (mttcan) | daedalus (gs_usb) |
|---|---|---|
| Clock | 50 MHz | 170 MHz |
| tq | 20 ns | 29.41 ns (BRP 5) |
| Bit time | 1+87+87+25 = 200 tq | 1+59+59+17 = 136 tq |
| Result | 4.00 µs = 250 kbps exact | 4.00 µs = 250 kbps exact |
| Sample point | 0.875 | 0.875 |
| sjw | 16 (set explicitly) | 16 (set explicitly) |

**Results — all passed:**
- Pinmux readback: `0x0000C458` / `0x0000C400` ✓
- daedalus internal loopback: RX 3 packets / 12 bytes matching TX, 0 errors
- **daedalus → Orion:** `cansend can0 100#0102030405060708` → Orion `candump -tz -x`
  shows `RX - -  100  [8]  01 02 03 04 05 06 07 08`. The `RX` flag (from `-x`)
  confirms it came off the wire, not local echo.
- **Orion → daedalus:** confirmed, plus 500-frame load test
  `cangen can0 -g 2 -I i -L 8 -n 500` — all 500 received, last ID `1F3` (=499,
  so nothing dropped or reordered), inter-frame ~2.05 ms matching `-g 2`
- Orion counters after: TX 501 packets / 4004 bytes (500×8 + one 4-byte frame),
  RX 1 packet / 8 bytes, **berr-counter tx 0 rx 0** — not one retransmission
- **Post-reboot: `can0` came up ERROR-ACTIVE at 250 kbps / sjw 16 /
  restart-ms 100 with zero manual steps, and a frame crossed the wire
  immediately.** W1 acceptance criterion met verbatim.

**What a single acknowledged frame actually proves** (worth remembering — it is
more than it looks): pinmux correct and signals reaching J17; transceiver
powered, out of standby, both transmitting and receiving; termination correct
enough for dominant bits to pull; both nodes' bit timing agreeing closely enough
to sample the same bits from different clocks; and — because the receiver must
assert a dominant bit in the ACK slot of the transmitter's frame — **the link is
bidirectional even if you only sent one direction.** A zero berr-counter on the
transmitter is the proof the ACK arrived.

## STM32F446RE — DEVELOPMENT ENVIRONMENT (daedalus)

### Toolchain decision: CubeMX + CMake + CubeCLT + VS Code

**STM32CubeIDE is not used for this project**, despite ~5 years of prior
familiarity with it. Reasons, in order of weight:

1. **W7 replicates firmware across 6 near-identical nodes.** CubeIDE's
   `.cproject`/`.project` are Eclipse-managed XML that regenerate on every
   settings change and produce diffs no human can review. `CMakeLists.txt` is a
   file you can read.
2. **Headless build over SSH/tmux** matches how the rest of this project is
   worked. CMake + Ninja build from the command line; the IDE wants a GUI.
3. **ST is migrating to VS Code.** As of VS Code extension v2.0.0, CMake project
   generation moved *into* STM32CubeMX (6.11.0+) and the extension no longer
   depends on CubeIDE at all.

The `.ioc` file remains the source of truth either way — the CubeMX graphical
clock tree and pinout are unchanged. Only the generator output target changes.
**In CubeMX Project Manager, Toolchain/IDE MUST be set to `CMake`.** Any other
value and the VS Code extension will not work with the project.

Migration was done in W2 deliberately, while firmware was a blinky — the same
move at W6, with UART/MKS and encoder PID entangled, would cost a week out of
the Oct 4 gate.

### Installed stack (daedalus, verified Aug 9, 2026)

| Component | Version | Path / source |
|---|---|---|
| STM32CubeCLT | 1.22.0 | `/opt/st/stm32cubeclt_1.22.0` |
| arm-none-eabi-gcc | 14.3.1 (GNU Tools for STM32 14.3.rel1) | bundled |
| arm-none-eabi-gdb | 15.2.90 | bundled |
| STM32CubeProgrammer | 2.23.0 | bundled |
| ST-LINK_gdbserver | 7.14.0 | bundled |
| CMake | 4.3.1 | bundled (resolves ahead of Ubuntu's) |
| Ninja | 1.13.2 | bundled |
| STM32CubeMX | standalone | separate download, ST account required |
| VS Code extensions | — | STM32 VS Code Extension, **Cortex-Debug**, CMake Tools, stm32-cube-clangd |

Key paths:
- SVD: `/opt/st/stm32cubeclt_1.22.0/STMicroelectronics_CMSIS_SVD/STM32F446.svd`
- gdbserver: `/opt/st/stm32cubeclt_1.22.0/STLink-gdb-server/bin/ST-LINK_gdbserver`
- toolchain bin: `/opt/st/stm32cubeclt_1.22.0/GNU-tools-for-STM32/bin`

**CubeCLT installs a profile script in `/etc/profile.d/` — you must log out and
back in before any tool resolves.** This is the #1 "I installed it and nothing
works" cause.

**Do NOT also `apt install gcc-arm-none-eabi`.** Two `arm-none-eabi-gcc` on PATH
produce linker errors that make no sense.

Verified compiler flags for this target:
```bash
-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
```
A successful link reports the multilib as `lib/thumb/v7e-m+fp/hard/libc.a`.
**Check this string** — with subtly wrong flags GCC silently falls back to a
soft-float library, which surfaces later as inexplicably slow PID math.

### Board: WeAct STM32F446 Core Board V1.1

- **8 MHz HSE crystal + 32.768 kHz LSE, both populated on board.** This is a real
  crystal, not a Nucleo's ST-LINK MCO — no debugger dependency, nothing to cut
  away when the board goes into the rover. This is what makes the HSE decision
  (W1 debrief) cheap.
- CAN1: PB9 (TX) / PB8 (RX)
- User LED: **PB2 — which is also BOOT1.** Harmless as an output; know it before
  wiring anything external there.
- USB-C is on PA11/PA12 (MCU native USB) — blocks the alternate CAN1 mapping
- BOOT0 key present → dfu-util flashing possible with no probe at all
- **No onboard debugger.** SWD header exposes only 3V3, GND, SWCLK, SWDIO.
- **No SWO pin exposed** → ITM/SWO `printf` unavailable unless PB3 is wired out
  manually. Use a UART for console. For W5 PID telemetry, OpenOCD's RTT support
  is the better option than burning a second UART.

### Module identity: 3-bit DIP switch (decided Aug 9, 2026)

**One firmware binary for all six modules.** Module identity is a property of
the hardware, not of the build: a 3-position DIP switch on each board is read
once at boot and yields a module ID 0–5, from which the firmware derives both
the CAN node ID and the module role.

| ID | Role | Behavior |
|---|---|---|
| 0–3 | Corner | Steering (UART → MKS SERVO42C) + drive (encoder PID) |
| 4–5 | Center | Drive only (encoder PID); steering block inactive |
| 6 | *reserved* | Future module / bench-test mode |
| 7 (`0b111`) | **INVALID** | Halt, blink error pattern, **do not join the bus** |

**Why this matters for W7.** The roadmap originally described W7 as "replicate
firmware to 3 more corners + write a center variant" — six near-identical builds
to keep in sync. With identity in a DIP switch and role derived from it, there
is one binary. The *"which build is on this board?"* failure class disappears
entirely; it would otherwise have surfaced in W8 disguised as a CAN problem,
which is the most expensive place to meet it.

**Electrical convention (don't "simplify" these away):**
- **Internal pull-ups enabled; switches pull to GND.** Closed = 0, open = 1. No
  external resistors needed.
- **`0b111` is deliberately the invalid code**, because it is *also* what you
  read from a board with no DIP switch fitted, a broken connection, or a
  floating input. An unconfigured board therefore fails loudly instead of
  silently impersonating module 7.
- **DIP switches, not solder jumpers or hardwired straps.** Reconfigurable on the
  bench when swapping a board to isolate a fault, and readable by eye without a
  meter — at W8 with six nodes live, "which module does this board think it is?"
  should be answerable by looking, not by attaching a debugger.

**Firmware convention:**
- Read the pins **once in `main()`**, before any peripheral init that depends on
  role, and latch into a variable. Identity must never change mid-run.
- Firmware is identical across all corners in another respect too: the UART link
  to the SERVO42C is point-to-point, so all four steering drivers stay at
  address `0xE0` (see MKS SERVO42C section). Module identity lives **only** in
  the CAN ID and this DIP switch.

**Pins: TBD** — to be fixed in the `.ioc` before the pinout fills. Constraints:
avoid PB2 (BOOT1 + user LED), PA13/PA14 (SWDIO/SWCLK), PB3/PB4 (JTAG remnants),
PB8/PB9 (CAN1), PA11/PA12 (USB), and whatever the encoder timer channels and
MKS UART will claim.

### Debug probes

| Probe | SN | Firmware |
|---|---|---|
| #1 | `37FF71064E573436D7331B43` | V2J46S7 |

- These are **ST-Link/V2, not V2-1** — `Board Name` is empty and no VCP appears
  in `STM32_Programmer_CLI -l`. **Consequence: no virtual COM port comes with the
  probe — console output requires a separate USB-TTL adapter** (on hand; needed
  by W3 to watch MKS SERVO42C protocol responses).
- Label each probe physically with its SN. By W7 there will be six boards, and
  reconstructing the SN→board mapping later is pure wasted time.
- The `serialNumber` field in `launch.json` pins a debug config to one probe.
  Without it the debugger attaches to whichever probe enumerated first — and you
  will single-step the wrong node while convinced the right one is broken.

### `.vscode/launch.json` (working reference)

The ST extension does **not** generate this when you simply open a folder.
Without it, the VS Code play button executes the ELF on the host — Ubuntu's
binfmt_misc hands it to `qemu-arm`, which segfaults on the first Cortex-M
peripheral access. That is not a flashing failure; it means no launch config
exists.

```json
{
  "version": "0.2.0",
  "configurations": [
    {
      "name": "Debug (ST-Link)",
      "type": "cortex-debug",
      "request": "launch",
      "cwd": "${workspaceFolder}",
      "executable": "${workspaceFolder}/build/Debug/<PROJECT>.elf",
      "servertype": "stlink",
      "device": "STM32F446RETx",
      "interface": "swd",
      "v1": false,
      "serialNumber": "37FF71064E573436D7331B43",
      "runToEntryPoint": "main",
      "serverpath": "/opt/st/stm32cubeclt_1.22.0/STLink-gdb-server/bin/ST-LINK_gdbserver",
      "stm32cubeprogrammer": "/opt/st/stm32cubeclt_1.22.0/STM32CubeProgrammer/bin",
      "armToolchainPath": "/opt/st/stm32cubeclt_1.22.0/GNU-tools-for-STM32/bin",
      "gdbPath": "/opt/st/stm32cubeclt_1.22.0/GNU-tools-for-STM32/bin/arm-none-eabi-gdb",
      "svdFile": "/opt/st/stm32cubeclt_1.22.0/STMicroelectronics_CMSIS_SVD/STM32F446.svd"
    }
  ]
}
```

- **Cortex-Debug (marus25) must be installed separately** — the ST extension pack
  does not pull it in as a hard dependency.
- `svdFile` is what populates the Cortex Peripherals view while halted. This is
  how `CAN_ESR` (error counters, last error code), `CAN_TSR` and the filter banks
  get read directly on target — the on-chip equivalent of `berr-counter` on Orion.
- Launch from the **Run and Debug** panel with this config selected, *not* the
  CMake Tools play button in the status bar.
- No `preLaunchTask` yet — build first, then launch.

### clangd vs cpptools

The extension pack ships `stm32-cube-clangd`, which conflicts with Microsoft's
cpptools IntelliSense and produces a repeating warning popup. **Keep clangd,
disable cpptools' engine at workspace level** (`.vscode/settings.json`):

```json
{ "C_Cpp.intelliSenseEngine": "disabled" }
```

Disable the engine, don't uninstall cpptools. clangd is correct here because it
reads `compile_commands.json` and therefore sees the real cross-compilation
flags; cpptools guesses and mis-parses CMSIS headers.

### Gotchas (don't rediscover these)

- **`monitor reset halt` is OpenOCD syntax and fails on ST's gdbserver**
  (`Unknown reset option` / `Protocol error with Rcmd: 05`). ST's server wants
  plain **`monitor reset`**, which resets *and halts at the reset handler*.
- **The ST-Link is exclusive.** `STM32_Programmer_CLI`, `ST-LINK_gdbserver` and a
  VS Code debug session cannot hold it simultaneously. A stray gdbserver left
  running in a background terminal produces connection errors that read exactly
  like a hardware fault.
- **`--specs=nosys.specs` link warnings are expected and harmless:** `_close`,
  `_lseek`, `_read`, `_write` "not implemented and will always fail". Correct on
  bare metal with no filesystem. They disappear individually as you retarget
  (e.g. overriding `_write` for UART `printf`).
- **ModemManager** can grab USB serial devices on connect, causing intermittent
  failures that look like flaky hardware. Fix with a udev rule setting
  `ENV{ID_MM_DEVICE_IGNORE}="1"`.
- If clangd reports missing HAL headers, it hasn't found `compile_commands.json`
  (CMake writes it into the *build* directory). Fix with a `.clangd` file in the
  project root: `CompileFlags: { CompilationDatabase: build/Debug }`. Not needed
  as of extension v2.x — it configured this automatically.

### Verification log — Aug 9, 2026 (daedalus + WeAct F446 board)

Each step verified before proceeding to the next (W1 method).

```
1. PATH after logout/login
   /opt/st/stm32cubeclt_1.22.0/{STM32CubeProgrammer,STLink-gdb-server,CMake,
   Make,Ninja,st-arm-clang,GNU-tools-for-STM32}/bin   OK all present

2. Tool versions
   gcc 14.3.1 · gdb 15.2.90 · CubeProgrammer 2.23.0 · cmake 4.3.1 · ninja 1.13.2  OK

3. Cortex-M4F compile+link (trivial main, nosys.specs)
   linked against thumb/v7e-m+fp/hard/libc.a   OK hard-float multilib confirmed
   text 5260 · data 1372 · bss 840

4. SVD present · probe enumerated · SWD connect
   STM32F446.svd found                                                    OK
   ST-LINK SN 37FF71064E573436D7331B43, FW V2J46S7, no VCP -> V2 not V2-1  OK
   Device ID 0x421 · Rev A · STM32F446xx · 512 KB · Cortex-M4 · 3.28 V     OK

5. GDB chain (ST-LINK_gdbserver -p 61234 + arm-none-eabi-gdb)
   attach halts core                                                      OK
   sp = 0x20020000  (= 0x20000000 + 128 KB, top of SRAM)                  OK
   xpsr = 0x01000000 (Thumb bit set)                                      OK

6. monitor reset -> "Successfully completed reset operation (System reset)"
   pc = 0x08000b48 after reset
   x/2xw 0x08000000 -> 0x20020000  0x08000b49
   i.e. vector[0] = initial MSP (matches sp), vector[1] = reset handler with
   Thumb bit -> handler at 0x08000b48 = pc.   OK reset-and-halt confirmed
   (Handler sits ~2.8 KB into flash because the vector table reserves ~97
   interrupt entries first. Normal.)

7. Full VS Code round trip: build -> flash -> halt at main               OK

8. Application-level confirmation under the new toolchain:
   MCO1 = 8 MHz HSE at the pin; MCO2 sources the 180 MHz PLL but runs
   through a /5 prescaler, so **PC9 carries 36 MHz** — that reading is
   correct, not a fault. Both scope-verified previously under CubeIDE and
   reproduced identically here, plus TIM3 interrupt-driven
   blinky on PB2.                                                        OK
   -> TIM3 firing at the expected rate validates the APB1 timer clock, and
   **APB1 at 45 MHz is the clock that feeds bxCAN** — so the bit-timing
   divisor chain above is validated on hardware, not only on paper.
```

**Conclusion: the toolchain is no longer a suspect.** Any subsequent failure
belongs to firmware or wiring. Same position W1 left `can0` in, and it is what
makes W3–W5 debugging tractable.

## STM32F446RE — FIRMWARE MODULES (W2)

Three application modules live alongside the CubeMX output. All are added to the
**root** `CMakeLists.txt` user-sources block, not `cmake/stm32cubemx/`, so a
CubeMX regeneration cannot drop them. All integration into `main.c` sits inside
`USER CODE` blocks for the same reason.

Build state Aug 9, 2026: RAM 3.4%, flash 8.5% of the F446RE. Clean under
`-Wall -Wextra -Wconversion -Wshadow`.

### `debug_uart` — non-blocking DMA console on USART1 (PA9 TX / PA10 RX)

115200 8N1. TX is a 1 KB ring drained by DMA2_Stream7; callers never block, so
output is safe from control loops and from interrupt context. RX is a 512 B
**circular** DMA on DMA2_Stream2 whose write pointer is read from the DMA
counter (`__HAL_DMA_GET_COUNTER`) rather than from a callback — bytes are
captured whether or not any ISR got to run.

```
debug_uart_write/puts/printf/write_hex   debug_uart_available/read/peek
debug_uart_flush/tx_pending              debug_uart_take_idle_event
debug_uart_stats/clear_stats             debug_uart_rx_flush
```

**Three CubeMX settings this depends on — all three are silent failures:**
- **USART1 global interrupt MUST be enabled in NVIC.** `HAL_UART_TxCpltCallback`
  is raised from the USART TC interrupt, *not* from the DMA stream interrupt.
  Without it the TX ring stalls after the first transfer: you see the first line
  of output and then permanent silence, which reads exactly like a wiring or
  baud fault. IDLE detection and UART error interrupts are also lost.
- **RX DMA must be Circular.** In Normal mode the stream halts at the first idle
  event and must be re-armed from the callback, losing whatever arrives in the
  gap. `debug_uart_init()` checks this and returns `DEBUG_UART_RX_UNAVAILABLE`
  rather than pretending to work.
- **TX DMA stays Normal.** Circular TX would retransmit the buffer forever.

**Idle-line framing is the point, not a side effect.** The UART idle line is a
hardware frame delimiter, which is how a variable-length reply is known to be
complete without knowing its length in advance — exactly the W3 problem, where
`E0 30 10` returns 8 bytes and `E0 F3 01 D4` returns 3.
`HAL_UARTEx_RxEventCallback` fires on half-transfer and full-transfer as well as
idle, so the handler filters on `HAL_UARTEx_GetRxEventType() ==
HAL_UART_RXEVENT_IDLE`; without that filter a reply straddling a buffer boundary
is reported as two frames, which would show up in W3 as occasional truncated
responses.

**`%f` needs `-Wl,-u,_printf_float`** — newlib-nano omits float printf by
default and prints garbage silently. Already added to `CMakeLists.txt`; needed
for W5 PID telemetry.

### `can_bus` — bxCAN on CAN1 (PB9 TX / PB8 RX) @ 250 kbps

Owns everything CubeMX does not generate: the acceptance filter, starting the
peripheral, and read access to the error state.

```
can_bus_init/send/receive                can_bus_tec/rec/esr/last_error[_str]
can_bus_set_loopback/is_loopback         can_bus_is_error_warning/passive/bus_off
can_bus_get_timing                       can_bus_stats/clear_stats
```

- Filter: bank 0, mask mode, 32-bit, ID 0x000 / mask 0x000, FIFO0,
  `SlaveStartFilterBank = 14`.
- **CubeMX settings that matter:** `AutoRetransmission = ENABLE` (the default
  DISABLE is one-shot mode — an unacknowledged frame is dropped after a single
  attempt, which makes "did it transmit?" much harder to answer during
  bring-up) and `AutoBusOff = ENABLE`, mirroring `restart-ms 100` on Orion.
- `can_bus_get_timing()` reads bit timing back out of `CAN1->BTR` — the
  silicon's own view, not what the source asked for. This catches a CubeMX
  regeneration silently resetting a field, which would otherwise surface as
  intermittent bus errors.
- RX is polled from the main loop **for now — see the OPEN question below.**
  The API is deliberately context-agnostic: moving to interrupt-driven means
  enabling `CAN1_RX0_IRQn` in CubeMX and calling the same `can_bus_receive()`
  from `HAL_CAN_RxFifo0MsgPendingCallback` — the function body does not change.
- `stats` reports two receive-pressure counters: `rx_fifo_full` (FIFO0 reached
  its 3-message depth — margin gone, nothing lost) and `rx_overruns` (a frame
  was lost). The latter counts **events, not frames**: `FOVR0` is sticky rc_w1,
  so the hardware cannot say how many were lost, only that some were.
- Heartbeat on **ID 0x500** (the 0x500-0x5FF telemetry/heartbeat group) at the
  TIM3 rate, so the LED blink and the CAN frame share a cadence. Payload is
  self-describing in `candump`: bytes 0-3 big-endian sequence, then TEC, REC,
  LEC, and a status bitfield (bit0 warning, bit1 passive, bit2 bus-off).
  Becomes `0x500 + module_id` once the DIP switch exists.

**Reading `lec` during bring-up:** `lec ack` means the frame went out correctly
but nothing acknowledged it. A transmitter cannot ACK itself, so this says "no
other node is listening", not "this node is broken". A steady `tec 0 rec 0
lec none` is the proof the ACK came back — the on-chip equivalent of Orion's
`berr-counter tx 0 rx 0`.

### `console` — line-based command interpreter

Turns the board into a bench instrument: inject frames, read error registers,
and switch CAN modes with no debugger session and no reflash.

```
help  info  stats  errors  clear  send <id> [hex]
heartbeat [on|off]   monitor [on|off]   loopback [on|off]   reset
```

- `send` accepts the payload however it is easiest to type — `send 123 DEADBEEF`,
  `send 123 DE AD BE EF` and `send 123 DEAD BEEF` are identical. Odd digit
  counts and >8 bytes are rejected rather than silently truncated.
- **`loopback on` is the solo self-test W1 concluded does not exist on the
  SocketCAN side.** bxCAN loopback stays off the wire and self-ACKs, so
  `send 123 DEADBEEF` returns through the filter and prints. That proves bit
  timing, filter bank 0, the FIFO path and both HAL call paths with no
  transceiver, no cable and no second node. If loopback works and normal mode
  does not, the fault is downstream of the MCU — which splits the search space
  before touching wiring.
- `heartbeat off` / `monitor off` silence async output while typing.
- `info` prints live clocks and the bit timing read back from `CAN1->BTR`.

**Terminal line endings — cost real debugging time Aug 9, 2026.** The
interpreter executes on CR or LF. CoolTerm with *Enter Key Emulation* set to
`None` sends no terminator, so commands echoed back correctly while nothing ever
ran — a symptom that looks like a parser bug and is not. `console_poll()` now
also executes on an **idle line** when the burst held more than one byte, which
covers Send-String-style terminals; the >1 byte guard is what keeps interactive
typing from executing a character at a time. Set Enter Key Emulation to `CR`
anyway rather than depending on the fallback.

### Known issue — `cmd_errors` reads CAN_ESR non-atomically

`console.c`'s `errors` command calls `can_bus_esr()`, then `can_bus_tec()`, then
`can_bus_rec()` — each performing its own read of `CAN1->ESR`. The register can
change between them, so the printed raw value and the decoded fields may
disagree. Observed Aug 10, 2026: raw `0x66000055` (REC 102) printed alongside
`REC : 103`, because REC was moving during bus-off recovery.

Harmless for steady-state inspection, misleading when counters are in motion —
which is exactly when the command matters. **Fix:** snapshot `CAN1->ESR` once
and decode every field from that snapshot. Not urgent; recorded so it is not
rediscovered as a mystery.

### Considered and not adopted — internal pull-up on PB8 (CAN1_RX)

PB8 is currently `GPIO_NOPULL`, as CubeMX generates it. Enabling the internal
pull-up would hold CAN_RX at recessive whenever nothing is driving it.

**The argument for it:** an undriven CAN_RX is the fault described below, and on
the rover it is reachable in normal service — an unpowered transceiver, or a
Bulgin connector working loose at a Rocker-Bogie flex point. With the pull-up,
that fault presents as a quiet node; without it, as a node cycling in and out of
BUS-OFF and generating error frames that disturb the whole bus. The transceiver's
push-pull output overrides a ~40k internal pull-up, so it costs nothing while
things are connected.

**Decision Aug 10, 2026: not adopted** — the `.ioc` stays as ST generates it.
Recorded here with the rationale so the option is not re-derived from scratch,
and so the trade-off is on the table if a loose-connector failure ever shows up
in W7/W8 with six nodes wired.

### Verification log — Aug 10, 2026 (W2 acceptance criterion)

**Three independent views of the same 17 frames:**

| Source | Frames |
|---|---|
| STM32 console | `hb 354` … `hb 370` (17) |
| Orion `candump -tz can0` | seq `0x161` … `0x171` (17) |
| daedalus / CANable `candump -tz can0` | seq `0x161` … `0x171` (17) |

No gaps, no duplicates, sequence numbers matching across all three. (The console
prints `seq` after incrementing, so `hb 354` carries payload `0x161` = 353.)

- STM32: `tec 0 rec 0 lec none` throughout
- Orion: `can state ERROR-ACTIVE (berr-counter tx 0 rx 0)`, bitrate 250000,
  sample-point 0.875, `tq 20 prop-seg 87 phase-seg1 87 phase-seg2 25 sjw 16`
- Not one retransmission across the run. **A zero TEC is the proof the ACK came
  back** — a receiver must assert a dominant bit in the ACK slot of the
  transmitter's frame, so the link is bidirectional even though traffic only
  went one way.

**Two clocks, one bus.** Orion runs 200 tq of 20 ns from 50 MHz; the STM32 runs
15 tq of 266.67 ns from 45 MHz. Both land on exactly 250 000 bps, sample points
87.5% and 86.7%.

**Inter-frame timing confirmed the divisor chain a third time.** Predicted TIM3
period: 90 MHz / (1800+1) / (25000+1) = 1.99881 Hz = 500.298 ms. Orion
timestamped the frames 500.297 / 500.295 / 500.304 ms apart. After the
oscilloscope (MCO1/MCO2) and the blinky, APB1 = 45 MHz is now also confirmed by
a stopwatch on the far side of the bus.

### The floating CAN_RX lesson (Aug 10, 2026) — do not skip this

Before the transceiver was wired, PB8 was left floating. **Three consecutive
bench runs of identical firmware failed three different ways:**

| Run | TEC | REC | LEC | What it looked like |
|---|---|---|---|---|
| 1 | 128 | 0 | `ack` | transmitted fine, nothing acknowledged |
| 2 | 0 | 255 | `form` | never transmitted, receiver drowning in garbage |
| 3 | 0 | counting down | `bit-dominant` | cycling in and out of BUS-OFF |

All three are the same root cause: an undriven CMOS input settling differently
each power-up. Floating high looks like an unacknowledged bus; floating low or
noisy looks like a corrupted one.

**The non-determinism was the diagnosis.** A driven input cannot behave
differently run to run — that alone ruled out firmware before any register was
examined.

Reading notes that generalise:

- **`lec bit-dominant`** = transmitted recessive, monitored dominant. Points at
  CAN_RX held low, TX shorted, or a transceiver holding the bus dominant.
- **`lec ack`** = the frame went out correctly and nothing acknowledged it. A
  transmitter cannot ACK itself, so this means no other node is listening.
- **LEC is sticky** — it holds the last error until a new one overwrites it or
  software clears it. Read TEC's *trend* for "erroring right now", not LEC.
- **TEC parks at 128 and never reaches BUS-OFF when a node is alone on the bus.**
  The CAN spec exempts an error-passive node from further TEC increment on ACK
  errors, precisely so a lone node cannot take itself bus-off. `passive` without
  `BUS-OFF` is the correct signature of "nobody else is out there".
- **Going BUS-OFF resets TEC to 0**, and bxCAN then reuses REC to count the 128
  sequences of 11 recessive bits required to rejoin. A falling REC with TEC at 0
  and `BOFF` set is `AutoBusOff` recovery in progress, not a receive problem.
- **TEC/REC survive `HAL_CAN_Stop()` + `HAL_CAN_Init()`.** Only a peripheral or
  system reset clears them, so counters seen after a mode change may predate it.

**Operational rule: do not debug CAN error counters on a node whose transceiver
is not connected and powered.** The numbers are not merely unhelpful, they are
actively misleading, and each of the three signatures above is individually
plausible enough to send you after the wrong fault.

### Bus-load ramp — Aug 11, 2026

`cangen can0 -g <gap> -I i -L 8` from daedalus, heartbeat running on the STM32,
`monitor off`, counters cleared between steps. Run durations were derived from
the heartbeat count (1.99881 Hz), which makes the STM32 its own stopwatch.

| `-g` | duration | measured rate | bus load | frames RX | FIFO-full | overruns |
|---|---|---|---|---|---|---|
| 5 | 87.1 s | 206 f/s | ~11% | 17,959 | 0 | 0 |
| 2 | 68.5 s | 509 f/s | ~27% | 34,891 | 0 | 0 |
| 1 | 68.0 s | 970 f/s | ~52% | 65,993 | 0 | 0 |
| 0.5 | 68.5 s | **1,858 f/s** | **~100%** | 127,343 | 0 | 0 |

`TEC 0 / REC 0 / lec none / error-active` at every step, including saturation.

**The ramp topped out on the wire, not on the MCU.** At `-g 0.5` cangen asked
for 2,000 f/s and got 1,858 — that is 538 us/frame, about 134 bits, exactly an
8-byte standard frame plus typical stuffing at 250 kbps. Every derived figure
agrees with theory independently, which is what makes the measurement
trustworthy.

**What it bounds.** `FULL0` sets when FIFO0 holds 3 messages. It never
incremented across 127,343 frames at saturation, so the main loop always drained
before three frames could accumulate:

```
worst-case main-loop period < 3 x 538 us = 1.6 ms
```

Not an average — a bound, held for 68 s with no outlier.

**Arbitration held too:** `can tx 137 frames, 0 dropped` at ~100% load. With
`-I i` sweeping the whole ID range, roughly five-eighths of cangen's frames
outrank 0x500, yet the heartbeat never missed a mailbox. At 2 Hz it has 500 ms
to win one arbitration, which is ample even on a saturated bus.

**What it does NOT prove — read this before citing the table.** The ramp
measured *throughput*: whether frames are lost. It measured neither **latency**
(how long a frame waits in the FIFO before being handled) nor **coupling** (that
the result is a property of a nearly empty main loop). Do not cite it as
evidence that polling is the right architecture — see below.

### OPEN — polled vs interrupt-driven CAN RX

**Status: undecided as of Aug 11, 2026.** The ramp above does not settle it.

**Leading candidate: the hybrid.** An interrupt-driven FIFO drain that pushes
frames into a software ring, consumed by the main loop. The ISR does one bounded
thing — pull from FIFO0, push to ring, return — and all application logic stays
in main-loop context where it can be single-stepped.

This is **the same pattern `debug_uart` already uses**: hardware and ISR fill a
ring, the main loop drains it. Making CAN symmetric with UART means one mental
model for both paths, which matters when six of these are in a rover and one
misbehaves in the field.

**For interrupts:**
- **Latency is unmeasured and lands in the control path.** With polling, a
  frame's worst-case wait is one loop period. That is jitter, not constant lag,
  so it does not calibrate out. Against a 50 Hz Ackermann command cycle (20 ms),
  several milliseconds of variable delay is a meaningful fraction of a cycle.
- **Polling correctness is contingent on the whole program staying fast.** The
  margin measured today is a property of an almost-empty loop, and every feature
  added between now and December erodes it silently. The failure mode is a
  synchronous MKS retry added in W6 causing intermittent frame loss under
  load — which presents as a wiring or bus problem, exactly the class of
  disguised fault the W1 debrief warns about.
- **CANopen may force it anyway.** `CanOpenSTM32`'s driver layer is built around
  CAN RX callbacks feeding the stack's receive buffers, and CiA 402 cyclic
  synchronous velocity mode is SYNC-timed, where jitter becomes control jitter.
  **Verify against the version actually used** — if it holds, the IRQ is required
  in Phase 2 regardless of what W2 measured.

**For polling:**
- No shared state between ISR and main loop, no critical sections on the frame
  path, no interrupt-priority reasoning (TIM3, USART1 and both DMA streams
  currently all sit at priority 0).
- Every frame is handled in one context that can be single-stepped.
- Measured to work with margin at bus saturation.

**Standing preference to weigh in:** interrupt-driven is the preferred style on
this project where a correct interrupt solution exists — polling is accepted only
where it is clearly the better engineering answer, not as a default.

### Planned — main-loop period in `stats`

Track min/mean/max main-loop time with the DWT cycle counter and report it in
`stats`. This turns "polling latency" from an argument into a number, and gives
an early-warning signal: the margin can be re-checked after every W3-W6
milestone and watched shrinking **before** it starts dropping frames rather than
after. Worth adding whichever way the RX question is decided.

## MKS SERVO42C V1.1 — UART PROTOCOL & BENCH VALIDATION

**Recovered and added Aug 6, 2026. Source session: June 8, 2026 ("Rover wheel
control board testing"). This was omitted from this file at the time — the whole
session went undocumented, which later caused W3 to be misjudged as higher-risk
than it is. The protocol below is validated against real hardware.**

### Role in the architecture
Steering actuator for the four corner modules. **UART-only — not CAN-native.**
This is why each corner STM32F446RE is a bridge: CAN in from Orion, UART out to
the SERVO42C. Motor is a NEMA 17 driving a custom 3D-printed 19:1 cycloidal
gearbox.

### Driver configuration (set via the onboard menu, per unit)
```
Menu → Mode     → CR_UART     (default is CR_vFOC — motion commands are IGNORED until changed)
Menu → UartBaud → 38400       (factory default)
Menu → UartAddr → 0xE0        (ALL units — see addressing note below)
```
- **Bench unit as tested:** CR_UART, addr `0xE0`, 38400 baud, **Mstep = 8**
- ⚠️ Read commands (e.g. `30`) respond in ANY mode; motion commands (`FD`, `F6`)
  require CR_UART. A driver that answers an encoder read but ignores a move
  command is almost certainly still in CR_vFOC.

### Addressing: all four steering drivers stay at 0xE0
The UART link is **point-to-point** — each corner STM32F446RE has its own
dedicated UART to its own SERVO42C. There is no shared bus, so there is nothing
for the address byte to disambiguate. `0xE0` is a protocol constant, not an
identifier.

**Module identity lives in the CAN ID, not the UART address.** Each STM32 has a
unique CAN node ID; the SERVO42C behind it does not need one.

Consequences, all favourable:
- **Identical firmware on all four corners** — no per-unit address constant, no
  build variants. Directly simplifies W7 replication.
- **Drivers are interchangeable spares** — a failed unit is swapped from the
  shelf with no menu reconfiguration.
- One less commissioning step per module, and one less thing to get silently wrong.

Addresses `0xE0`–`0xE3` would only be needed if several drivers shared one UART
(multi-drop), which this architecture does not do. Keep it in mind only for a
bench scenario where two drivers are deliberately hung off one USB-serial adapter.

### Resolution with Mstep = 8 and the 19:1 gearbox
```
Pulses per motor revolution:   8 × 200        = 1,600
Pulses per output revolution:  1,600 × 19     = 30,400
Angular resolution at output:  360° / 30,400  = 0.01184°  (~43 arcseconds)
```

### Packet format
```
[addr] [function code] [data bytes...] [checksum]
```
Checksum = sum of ALL preceding bytes (addr and function code included), `& 0xFF`.
It is a plain 8-bit additive checksum, not a real CRC, despite being called CRC.

> **MANDATORY WORKING RULE — Claude has made repeated checksum errors on this
> protocol.** Always write the full decimal breakdown before stating a checksum
> byte, so the arithmetic can be checked at a glance. Example:
> ```
> E0 + 30 = 224 + 48 = 272
> 272 & 0xFF = 272 − 256 = 16 = 0x10
> ```

### Verified command set (every checksum below re-verified Aug 6, 2026)

**Read-only diagnostics — safe in any mode, no motion:**
| Purpose | Full packet | Checksum arithmetic | Returns |
|---|---|---|---|
| Read encoder | `E0 30 10` | 224+48=272 → 16 | int32 carry + uint16 value |
| Pulses received | `E0 33 13` | 224+51=275 → 19 | int32 pulse count |
| Shaft angle error | `E0 39 19` | 224+57=281 → 25 | int16 (65536 = 360°) |
| EN pin status | `E0 3A 1A` | 224+58=282 → 26 | `01`=enabled, `02`=disabled |
| Protection state | `E0 3E 1E` | 224+62=286 → 30 | `01`=protected, `02`=clean |

`E0 30 10` is the **safe first command** for any bring-up — no motion, no config change.

**Motion (CR_UART only):**
| Purpose | Full packet | Checksum arithmetic |
|---|---|---|
| Enable motor | `E0 F3 01 D4` | 224+243+1=468 → 212 |
| Stop / hold | `E0 F7 D7` | 224+247=471 → 215 |
| Move 1,600 pulses CW (1 motor rev @ Mstep 8) | `E0 FD 02 00 00 06 40 25` | 224+253+2+0+0+6+64=549 → 37 |
| Move 1,600 pulses CCW | `E0 FD 82 00 00 06 40 A5` | 224+253+130+0+0+6+64=677 → 165 |
| Move 160 pulses CW | `E0 FD 02 00 00 00 A0 7F` | 224+253+2+0+0+0+160=639 → 127 |
| Move 160 pulses CCW | `E0 FD 82 00 00 00 A0 FF` | 224+253+130+0+0+0+160=767 → 255 |
| Move 16 pulses CW (fine step) | `E0 FD 02 00 00 00 10 EF` | 224+253+2+0+0+0+16=495 → 239 |

`FD` layout: `FD [VAL] [uint32 pulses, big-endian]`, where VAL bit7 = direction
(0 = CW, 1 = CCW) and bits6–0 = speed. **`0x02` = CW speed 2; `0x82` = CCW speed 2.**

`F6` runs at constant speed with the same VAL byte encoding. Speed formula for a
1.8° motor: `RPM = (Speed × 30000) / (Mstep × 200)`.
For steering, use low speed values (1–4) with `FD`, not `F6`.

**Configuration (persistent, written to flash):**
| Parameter | Code | Data | Default |
|---|---|---|---|
| Motor type | `81` | `00`=0.9°, `01`=1.8° | `01` |
| Work mode | `82` | `00`=OPEN, `01`=vFOC, `02`=UART | `01` |
| Microstepping | `84` | `00`–`FF` | `10` (16) |
| EN pin polarity | `85` | `00`=L, `01`=H, `02`=Hold | `00` |
| Direction | `86` | `00`=CW, `01`=CCW | `00` |
| Locked-rotor protection | `88` | `00`=off, `01`=on | `00` |
| Baud rate | `8A` | `01`=9600 … `06`=115200 | `04` (38400) |
| UART address | `8B` | `00`–`09` → addr = `0xE0 + n` | `00` |
| Restore defaults | `3F` | — | — |
| **Kp** (position) | `A1` | uint16 | `0x0650` = 1616 |
| **Ki** (position) | `A2` | uint16 | `0x0001` = 1 |
| **Kd** (position) | `A3` | uint16 | `0x0650` = 1616 |
| **ACC** (accel ramp) | `A4` | uint16 | `0x011E` = 286 — ⚠️ too large can damage the board |
| **MaxT** (max torque) | `A5` | uint16, range 0–`0x04B0` | `0x04B0` = 1200 |

Set MaxT to maximum: `E0 A5 04 B0 39` → 224+165+4+176 = 569 → 569−512 = 57 = 0x39
Set Kp to default:   `E0 A1 06 50 D7` → 224+161+6+80 = 471 → 471−256 = 215 = 0xD7

### ⚠️ The driver ECHOES every request before replying

**Discovered Aug 13, 2026, on the STM32.** The SERVO42C retransmits the bytes it
just received, then sends its answer. What actually arrives is:

```
E0 30 10 | E0 00 00 00 00 00 2A 0A
└─ echo ┘ └──── the actual reply ────┘
```

**This was never noticed in the June 8 bench session** because that used a hex
terminal, where a human eye reads past the repeated bytes without registering
them. It only surfaced once software had to validate a frame: the checksum gets
computed across echo *and* reply together and a perfectly good response is
rejected as corrupt.

**Confirmed device behaviour, not a wiring loop.** Verified across two different
SERVO42C boards and two motors, with connectors swapped and MCU-side wiring
re-checked — TX and RX are not bridged anywhere.

**Handling it (`strip_echo()` in `mks_servo.c`):** discard a leading run that
exactly matches the bytes just transmitted, then validate what remains. Two
properties make this safe rather than a heuristic:

- The echo **cannot arrive split**. Its bytes stream back continuously while we
  transmit, so an idle gap can only open after the last of them — the echo is
  either wholly present or not started, and an exact prefix match is sound.
- Echo and reply may arrive as **one burst or two** separated by an idle gap.
  Both occur; an echo-only burst must be treated as "keep waiting", not as a
  malformed reply.

The strip is a no-op on a link that does not echo, so it costs nothing if a
future firmware revision drops the behaviour. Each one is counted in
`mks_stats.echoes` and reported by the console as "echoes stripped (normal)" —
**a non-zero count is expected, not a fault.**

> Anyone writing a new command for this protocol, or debugging one that returns
> "bad checksum", should read this first. The reply is very likely fine.

### Response format
| Response | Meaning | Checksum |
|---|---|---|
| `E0 01 E1` | command accepted / run starting | 224+1=225 |
| `E0 02 E2` | run complete | 224+2=226 |

**Encoder read** `E0 30 10` → e.g. `E0 FF FF FF F8 2B B1 B1`
```
E0            addr
FF FF FF F8   carry, int32 = −8 (full encoder overflows)
2B B1         value, uint16 = 11,185
B1            checksum
```

**Angle error** `E0 39 19` → e.g. `E0 FF 99 78`
```
0xFF99 as int16 = −103
−103 / 65536 × 360° = −0.566°
```
Negative = shaft pushed back from its target by the external load.

> ⚠️ **OPEN QUESTION for W9 precision calibration.** The SERVO42C encoder sits on
> the **motor** shaft, so these angle-error degrees are motor-side. The stiffness
> figures below pair motor-side degrees with output-side torque, so they are a
> mixed-unit convenience number, not a true output stiffness. Divide by 19 for
> output-referred deflection (−0.566° motor ≈ −0.0298° output). Resolve this
> convention explicitly before quoting stiffness anywhere it matters.

### Reply lengths by function code (measured Aug 13, 2026)

Every command answers with a fixed number of bytes. This is what makes framing
deterministic — see the next section for why timing cannot be used instead.

| Function | Command | Reply | Layout |
|---|---|---|---|
| `30` | read encoder | **8** | `E0` + int32 carry + uint16 value + ck |
| `33` | pulses received | **6** | `E0` + int32 + ck |
| `39` | shaft angle error | **4** | `E0` + int16 + ck |
| `3A` | EN pin status | **3** | `E0` + status + ck |
| `3E` | protection state | **3** | `E0` + status + ck |
| `3F` | restore defaults | **3** | ack |
| `F3` | enable / disable | **3** | ack |
| `F6` | constant speed | **3** | ack, then a second 3-byte completion |
| `F7` | stop | **3** | ack |
| `FD` | relative move | **3** | ack, then a second 3-byte completion |
| `81`–`8B` | config writes | **3** | ack |
| `A1`–`A5` | Kp/Ki/Kd/ACC/MaxT | **3** | ack |

**Beware the 3-byte collision.** `E0 01 E1` and `E0 02 E2` are simultaneously
the generic accepted/complete acknowledgements *and* valid status values for
`3A` (01 = enabled, 02 = disabled) and `3E` (01 = protected, 02 = clean). The
bytes alone cannot distinguish them — **decode by the function code you sent**,
never by reply length or content. Verified Aug 13 by toggling `F3` and watching
`3A` follow it, which rules out the possibility that `3A` was merely being
acknowledged rather than answered.

### ⚠️ Idle-line framing does NOT work on this link

The obvious way to delimit a variable-length reply is the UART's IDLE flag, and
it is what `debug_uart` uses successfully for the console. **It is wrong here.**

The SERVO42C echoes in *software* — one byte at a time as it processes them —
and pauses for longer than one character time *within* a single message. IDLE
fires after one character time (~260 us at 38400), so it triggers mid-message
and is not a boundary at all.

**How this presented:** a move command returned

```
00 00 10 EF E0 01 E1
```

which is the **tail of our own request** (`E0 FD 02 00 00 00 10 EF`, bytes 4-7)
followed by a valid accepted-ack. An IDLE fired during transmission, the
handler flushed the receive buffer, and the first four echo bytes were
destroyed — leaving a fragment that failed address validation. Cost a bring-up
session to diagnose, and the symptom pointed at everything except the real
cause.

**The rule:** accumulate received bytes unconditionally, never resetting on a
timing boundary, and decide a message is complete by its **expected length**
(table above) or — for an undocumented function code — by a quiet period long
enough to clear this device's inter-byte gaps (8 ms is used).

### ⚠️ Clearing UART error flags steals a byte from the DMA

Generic STM32 trap, not MKS-specific, and worth knowing anywhere HAL UART DMA
reception is used.

`__HAL_UART_CLEAR_OREFLAG()` and its siblings all expand to the same thing on
F4: a read of `SR` followed by a read of `DR`. **Reading DR while DMA reception
is running consumes a byte the DMA was entitled to**, and it is gone. Calling
these from `HAL_UART_ErrorCallback()` — the natural place — does exactly that
whenever HAL treated the error as non-blocking and left the transfer running.

It surfaces as occasional inexplicable checksum failures under load, and gets
blamed on wiring.

**Related: re-arming unconditionally causes an error cascade.** Calling
`HAL_UARTEx_ReceiveToIdle_DMA()` on every error means one seed event can flag
another error during the re-arm, which re-arms again. **180 error callbacks
across 7 transactions** were observed this way on Aug 13, then zero on the next
boot — dormant, not fixed.

**The rule for both:** ask the hardware whether reception actually stopped —

```c
still_running = (huart->Instance->CR3 & USART_CR3_DMAR) &&
                (hdma_rx.Instance->CR & DMA_SxCR_EN);
```

— and only clear flags or re-arm when it has. HAL clears both bits when it
treats an error as blocking (overrun, DMA fault) and leaves them alone
otherwise, so this observes the real state rather than assuming a policy.

### Verification log — Aug 13, 2026 (W3 first motion)

Encoder position is `carry x 65536 + value`; the SERVO42C encoder is 16-bit per
**motor** revolution.

| Step | `pulses` (`33`) | encoder raw | position | predicted |
|---|---|---|---|---|
| start | — | carry 0, 42 | +42 | — |
| 3 x `move 16` CW (48 pulses) | −48 | carry −1, 63614 | −1,922 | −1,924 |
| `deg 5` (+422 pulses) | −470 | carry −1, 46329 | −19,207 | −19,209 |
| `deg −5` (−422 pulses) | — | carry −1, 63610 | −1,926 | −1,922 |

**Mstep = 8 confirmed by measurement, not by menu.** Predictions use
`pulses / 1600 x 65536`. Both forward steps land within **2 counts**, and the
offset is constant rather than growing — an artifact, not a scale error. At
Mstep 16 the first row would have read −941, so the result is decisive. This is
the reliable way to check Mstep: it measures what the mechanism did.

**Round-trip repeatability: 4 counts.** Out 5 deg and back landed −1,926
against −1,922. One microstep at Mstep 8 is 65536/1600 = **41 counts**, so the
error is about **one tenth of a single microstep** — 0.022 deg at the motor,
**0.0012 deg at the output**. No measurable backlash contribution at this
amplitude. Useful baseline for W9 precision calibration.

**Angle conversion is exact to quantisation.** `deg 5` issued 422 pulses =
4.9974 deg at the output; the 0.0026 deg residual is one-pulse quantisation
(0.0118 deg), i.e. the mechanism's floor.

**`33` counts UART-commanded pulses**, not just hardware STEP input: −470 is
exactly 48 + 422. That makes it usable as the feedback path for absolute
positioning, which `FD` alone cannot provide since it is a relative move.

**Direction convention:** positive degrees / `ccw = false` **decrements** both
the encoder position and the `33` pulse counter. Pin this down before Ackermann
sign conventions are written.

### Torque characterization — June 8, 2026

**Rig:** AMF-300 digital force gauge (300 N max) rigidly frame-mounted at exactly
**10 cm** from the rotation axis. A 20×20 aluminium profile on the gearbox output
shaft presses against it. `Torque (N·m) = Force (N) × 0.10`. If the gauge reads
kgf: `Torque = kgf × 9.81 × 0.10`.

**Method:** enable, advance in 16-pulse steps (`E0 FD 02 00 00 00 10 EF`), and at
each step record force, angle error (`E0 39 19`), and supply current.

**Run 1 — default MaxT:**
| Force (N) | Torque (N·m) | Angle error (°) |
|---|---|---|
| 7.3 | 0.73 | −0.566 |
| 9.4 | 0.94 | −0.697 |
| 11.6 | 1.16 | −0.900 |
| 13.7 | 1.37 | −1.038 |
| 15.7 | 1.57 | −1.170 |
| 17.7 | 1.77 | −1.312 |
| 19.7 | 1.97 | −1.471 |
| 21.7 | 2.17 | −1.602 |
| 23.7 | 2.37 | −1.794 |
| 25.8 | 2.58 | −1.971 |
| 27.5 | 2.75 | −2.234 |
| 29.6 | 2.96 | −2.393 |
| 31.6 | 3.16 | −2.658 |
| 33.6 | 3.36 | −2.850 |
| 35.7 | 3.57 | −3.091 |

Run 1 peaked at **4.95 N·m** before the driver stopped.

**Run 2 — MaxT raised to maximum (`E0 A5 04 B0 39`):** pushed to a true stall
boundary at **5.57 N·m**, drawing **1550 mA** = **18.66 W** at 12.04 V.

**Results:**
- **Safe continuous operating torque: ~3.5 N·m**
- **Peak / stall torque: 5.57 N·m**
- Average stiffness ~1.36 N·m/° early, falling to ~0.90 N·m/° near 3.5 N·m
  (see the mixed-unit caveat above)
- Gearbox mechanical efficiency estimated **50–65%** — consistent with printed
  cycloidal expectations
- **Scrub torque required, 45 kg rover with 13 cm wide wheels: ~1.325 N·m**,
  computed with the **contact-patch model, NOT the wheel-radius model**
  → **4.2× safety margin** against the 5.57 N·m stall figure

**Notable measured behaviour — current draw is remarkably low under load.** At
3.36–3.57 N·m the supply drew only **135–166 mA at 12 V (~1.6–2.0 W)**, with no
audible noise and no heat. Current climbs slowly as the PID works harder, then
spikes sharply at the stall boundary (1550 mA). That transition is the reliable
indicator of the true torque limit — watch current, not force.

### What this means for W3 (STM32 UART firmware)
**Already proven, do not re-derive:** packet format, checksum, command bytes,
response decoding, motor behaviour under load.
**Discovered in W3, was not in the June 8 notes:** the driver echoes every
request before replying (see the section above). Cost a bring-up session to
diagnose because it presents as a checksum failure on a reply that is
actually correct.

**Resolved Aug 13, 2026 — the acceptance criterion is met.** HAL UART config,
asynchronous response timing including the two-stage `FD` reply, and response
parsing in C all work against real hardware; see the Aug 13 verification log
above. Still outstanding from the list below: confirming the other three
drivers are set to CR_UART / 38400 / Mstep 8.

**Originally listed as unproven:** STM32 HAL UART configuration;
asynchronous response timing (the driver replies in two stages for `FD` — start
then complete — and `39` reads inside a control loop have latency implications);
parsing responses in C rather than reading hex by eye; and confirming the other
three drivers are set to CR_UART / 38400 / Mstep 8 (address needs no change —
all stay at `0xE0`), since only one unit has ever been on the bench.

### Tooling
Custom serial terminal running on daedalus (built earlier with Claude Design +
Claude Code), used in hex mode — sends raw byte sequences and shows raw responses.

## SYSTEMD SERVICES (DELL HOST)
- **qemu-aarch64-binfmt-fix.service:**
  Runs multiarch/qemu-user-static --reset -p yes on every boot
  Registers QEMU with F-flag for ARM64 Docker container support

## SYSTEMD SERVICES (ISAACUN)
- **ssh.socket override:**
  /etc/systemd/system/ssh.socket.d/override.conf
  Forces SSH to listen on port 44252 instead of default 22

## SYSTEMD SERVICES (FORGE)
- **ssh.socket override:**
  /etc/systemd/system/ssh.socket.d/override.conf
  Forces SSH to listen on port 44252 instead of default 22, same pattern as IsaacUN

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
  - src/zed-ros2-wrapper/ (ZED ROS 2 wrapper, master branch)
  - install/ (colcon build output, sourced in ~/.bashrc)
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
- **Git configured on:** Dell laptop ✅ and Jetson ✅ (IsaacUN pending)
- **SSH keys added to GitHub:** Dell ED25519 ✅ and Jetson ED25519 ✅

## VS CODE DEV CONTAINER (DELL HOST)
- **Status:** Fully configured and working
- **Name:** ROS2 Humble + ZED 2i Development
- **Base:** Ubuntu 22.04 amd64 (native speed, not emulated)
- **Contents:** ROS 2 Humble Desktop + Cyclone DDS + CUDA headers
  + OpenCV + Python robotics packages + ZED SDK headers
- **Network:** --network=host
- **Note:** --runtime=nvidia was REMOVED from runArgs (host has no
  NVIDIA Container Runtime — only needed on Jetson)
- **Verified:** ROS 2 talker/listener working Dev Container ↔ Jetson
- **IntelliSense:** Confirmed working for rclcpp, std_msgs,
  geometry_msgs, sensor_msgs
- **ZED IntelliSense:** ✅ COMPLETED (April 3, 2026)
  - ZED SDK 5.2.2 headers installed at /usr/local/zed/include/sl/ inside container
  - Headers: Camera.hpp, CameraOne.hpp, Fusion.hpp, Lidar.hpp, Sensors.hpp
  - Installation method: download ubuntu22/cu12 installer, run with
    -- silent skip_cuda skip_od_module skip_tools, keep only include/,
    chmod a+rX /usr/local/zed and chmod -R a+rX /usr/local/zed/include
  - IMPORTANT: Two chmod commands required — parent directory /usr/local/zed
    must also be made world-readable, not just /usr/local/zed/include
    (ZED installer sets drwxrwx--- on both, blocking non-zed-group users)
  - devcontainer.json already had /usr/local/zed/include in includePath —
    no changes to devcontainer.json were needed
  - Verified: sl::Camera, sl::InitParameters, sl::DEPTH_MODE::NEURAL,
    sl::Mat all resolve correctly with hover documentation in VS Code
- **Dev Container purpose:** Write and edit code on Dell with full IntelliSense
  for ROS 2 + ZED APIs; sync to Jetson via jsync for building and execution
  on real hardware. Dev Container runs on Dell (amd64), not on Jetson (ARM64).

## ROS 2 COMMUNICATION

### Home network (Dell ↔ Jetson)
- **DDS:** Cyclone DDS on ALL environments (Dell, Dev Container, Jetson)
- **Discovery:** Unicast peer-to-peer (WiFi router blocks multicast)
- **Loopback peer:** 127.0.0.1 added as first peer on both Dell and Jetson
  (required for composable node service calls on same machine)
- **Verified paths:**
  - Dell ↔ Jetson ✅
  - Dev Container ↔ Jetson ✅
  - C++ nodes (demo_nodes_cpp) ✅
  - Python nodes (demo_nodes_py) ✅
  - ZED ROS 2 wrapper on Jetson ✅ (NEURAL depth, 21 topics, IMU at 100Hz)

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
1. **ZED camera test:** ✅ COMPLETED
   Cable confirmed USB 3.00 (bcdUSB 3.00); camera opened successfully;
   depth pipeline verified with 5 stable frames at ~3.25m;
   calibration file downloaded for S/N 32047842

2. **Install ZED ROS 2 wrapper on Jetson:** ✅ COMPLETED (April 3, 2026)
   zed-ros2-wrapper v5.2.2 cloned, built, and fully operational.
   Root cause of launch failures identified and fixed: missing 127.0.0.1
   loopback peer in Cyclone DDS config prevented composable node loading.
   NEURAL depth mode active, 21 topics publishing, IMU at 100Hz verified.
   Git configured on Jetson. jtop installed for hardware monitoring.
   Cyclone DDS loopback fix applied to both Jetson and Dell configs.

3. **Update Dev Container for ZED IntelliSense:** ✅ COMPLETED (April 3, 2026)
   ZED SDK 5.2.2 headers installed in Dev Container at /usr/local/zed/include/sl/.
   Two-stage chmod required (parent dir + include dir). IntelliSense verified
   working for sl::Camera, sl::Mat, sl::InitParameters, sl::DEPTH_MODE::NEURAL.
   Dockerfile committed to RobertUN repository.

4. **Update cyclone_dds.xml on Dell host and Dev Container:** ✅ COMPLETED (April 3, 2026)
   127.0.0.1 loopback peer added as first peer in both:
   - ~/.ros/cyclone_dds.xml (Dell host)
   - ~/ros2_ws/.devcontainer/cyclone_dds.xml (Dev Container)

5. **CAN Bus — first hardware test:** ✅ COMPLETED (August 6, 2026) — this is
   roadmap W1, all five sub-items. MKS CANable V2.0 Pro flashed slcan →
   candleLight; J17 header soldered; SN65HVD230 wired; devmem pinmux applied and
   verified by readback; two-node physical bus validated bidirectionally at
   250 kbps over 3 m Cat-6 with a clean 500-frame load test; pinmux + can0
   config made persistent via robertun-can0.service and confirmed across a full
   reboot. Full log in the CAN BUS — PERSISTENCE section above.
   - Corrections to what this task originally assumed:
     - Bitrate is **250 kbps**, not 125 kbps (decided in the hardware section)
     - "Remove mttcan blacklist" does not apply — no blacklist on Orion's R36.5
     - "Loopback self-test by shorting TX+RX on J17" is not a meaningful test;
       there is no useful pins-only check (see JETSON SOFTWARE SETUP above)
     - Three-way Jetson ↔ STM32 ↔ CANable verification moves to W2, since no
       STM32 firmware exists yet

6. **CAN Bus — STM32 firmware:** ✅ **COMPLETED August 10, 2026** — this is
   roadmap W2. Full log in the Aug 10 verification section above.
   - ✅ bxCAN at 250 kbps (BRP 12, BS1 12, BS2 2, SJW 2, 86.7% sample point)
   - ✅ Accept-all mask filter on bank 0, `HAL_CAN_Start()`, heartbeat on 0x500
   - ✅ DMA console + command interpreter for bench work (see FIRMWARE MODULES)
   - ✅ `loopback on` + `send 123 DEADBEEF` self-test round-tripped with nothing
     attached — proves bit timing, filter and FIFO independently of any wiring
   - ✅ Termination measured 59.79R with all three nodes connected
   - ✅ Three-way verification: STM32 → Orion `candump`, confirmed independently
     on the CANable, 17 frames, zero error counters on both ends
   - Open, non-blocking: `cmd_errors` ESR snapshot (see Known issue above)
   - **Open decision:** polled vs interrupt-driven CAN RX — the Aug 11 load ramp
     bounds throughput only, not latency. Hybrid ISR-to-ring is the leading
     candidate. Settle before W6.

7. **CAN Bus — ROS 2 integration (CANopen stack):**
   - STM32 side: CANopenNode + CanOpenSTM32 (HAL-integrated, CubeMX compatible)
     - Implement CiA 402 motor profile (state machine, controlword/statusword,
       cyclic synchronous velocity mode) on each wheel STM32 node
     - Heartbeat, EMCY (emergency), and SDO configuration support
   - Jetson side: ros2_canopen (Fraunhofer IPA / ROS-Industrial)
     - YAML bus topology configuration with EDS file per device
     - CiA 402 driver with ros2_control hardware interface integration
     - Lifecycle-managed nodes (configure → activate → deactivate)
   - Resources:
     - CANopenNode: https://github.com/CANopenNode/CANopenNode
     - CanOpenSTM32: https://github.com/CANopenNode/CanOpenSTM32
     - ros2_canopen: https://github.com/ros-industrial/ros2_canopen
     - ros2_canopen manual: https://ros-industrial.github.io/ros2_canopen/manual/rolling/

8. **Configure Isaac Sim ROS 2 bridge:** ✅ COMPLETED (May 2026)
   ROS 2 bridge confirmed operational. Two-terminal workflow established.
   launch_isaacsim.bash and unros2.bash scripts created at ~/isaac-sim/.
   NVIDIA Docker workspace built at build_ws/jazzy/isaac_sim_ros_ws/.
   AMENT_PREFIX_PATH correctly set; rclpy loaded message confirmed.
   Remaining: run full example with Isaac Sim publishing to ROS 2 topics
   and verifying reception from external terminal (e.g. Carter navigation demo).

9. **Python conda environment on Dell laptop:** ✅ COMPLETED
   Miniconda3 installed; ros2 env (Python 3.10, Humble, Cyclone DDS) and
   ml env (Python 3.11, PyTorch 2.11+cu126, GPU verified) both configured

10. **Git configuration on IsaacUN and Jetson:**
    Jetson ✅ COMPLETED (April 3, 2026)
    IsaacUN: still pending — configure git credentials and add SSH key to GitHub

11. **Update RobertUN README.md:**
    Current README is minimal — describe the project properly
    Document repository structure and purpose of each folder

12. **Gigabit switch network:** ✅ COMPLETED
    Switch installed; Dell and Jetson have static Ethernet IPs via switch;
    Cyclone DDS and SSH updated to use Ethernet addresses;
    WiFi remains as automatic fallback

13. **Begin robot ROS 2 development:**
    SLAM/Mapping, Visual Odometry/Navigation, Object Detection/AI

15. **Xavier NX LLM evaluation and llama-server setup:**
    - Complete rover evaluation framework (Python script running all 4 models
      against 12 test cases via llama-server HTTP API, scoring JSON validity,
      action compliance, semantic correctness, clarification quality)
    - Set up llama-server as systemd service (port 8080, Qwen2.5-3B default)
    - Configure tmux on xavier for comfortable multi-pane SSH workflow
    - Integrate LLM REST endpoint with ROS 2 (language_commander node)

14. **NoMachine display/mouse issue (new laptop — low priority):**
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
- **Cyclone DDS loopback peer (127.0.0.1):** Required on any machine running
  ROS 2 composable nodes. Without it, the load_node service call between the
  launch system and component_container_isolated fails silently, hanging
  indefinitely with zero CPU/GPU activity. First peer in config, always.
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
  system Python must remain unmodified; all dev work in conda envs.
  conda was broken by username change (namontoy→talos) — fixed by updating
  shebang lines in miniconda3/bin/ and profile.d scripts. Never run
  sed on binary files in miniconda3/bin/ — use grep -l + targeted text-only sed.
- **Isaac Sim ROS 2 bridge — two-terminal architecture:** Isaac Sim uses
  internal Python 3.11 rclpy; external ROS 2 nodes use system Python 3.12.
  Never source /opt/ros/jazzy/setup.bash in the terminal that launches Isaac Sim.
  Use launch_isaacsim.bash (neutralizes env + sources Python 3.11 workspace).
  External nodes communicate via DDS topics — no shared Python environment needed.
- **NVIDIA Docker build for ROS 2 workspace:** The IsaacSim-ros_workspaces repo
  provides Dockerfiles (dockerfiles/ubuntu_24_jazzy_python_312_minimal.dockerfile)
  that correctly handle all dependency conflicts (empy==3.3.4, setuptools==70.0.0,
  numpy reinstall, pybind11). Always use ./build_ros.sh instead of native colcon
  build on IsaacUN — the Ubuntu 24.04 apt packaging gap for navigation2 makes
  native builds unreliable. Docker build produces files on host disk; Docker
  is not needed at runtime.
- **Snap removed from IsaacUN (May 2026):** snap Docker had socket confinement
  issues (root:root socket, private /tmp namespace) that prevented normal
  group-based permissions from working. Replaced with apt Docker from
  docker.com official repository. All snap packages removed; snapd held and
  pinned to prevent reinstallation. Firefox replaced with Mozilla apt repo version.
- **ZED SDK 5.2.2 for L4T R36.5:** Specific version matching
  JetPack 6.2.2 / CUDA 12.6 — fixes ZED2i positional tracking lock bug
- **NEURAL depth mode:** PERFORMANCE mode deprecated in ZED SDK 5.x;
  NEURAL provides superior depth quality with pre-cached TensorRT engines
  loading in seconds (no recompilation needed after first ZED SDK install)
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
- **250 kbps bus bitrate:** Provides comfortable headroom for CANopen protocol
  overhead (PDO, SDO, NMT, heartbeat) across 8 nodes. At this speed the 100Ω
  vs 120Ω impedance mismatch of flex Cat-5/6 cable is still negligible
  (reflection ~5% of bit time). Previous candidate was 125 kbps — both are
  valid; 250 kbps chosen to accommodate CANopen traffic budget.
- **Message-centric CAN ID priority:** Priority assigned per message type,
  not per node — safety-critical messages win regardless of source node
- **Canable (STM32 USB dongle) for bench CAN sniffer:** Appears as native
  SocketCAN interface on Linux; works with existing candump/cansend tools;
  clone versions (~$10-15 USD) available on AliExpress. Flash candlelight
  firmware (preferred over slcan) when it arrives.
- **CANopen with CANopenNode as application-layer protocol:** Selected over
  plain CAN and OpenCyphal (Cyphal/UAVCAN v1) for the following reasons:
  - CiA 402 motor control profile provides a standardized state machine,
    controlword/statusword, and cyclic synchronous velocity mode — exactly
    what 6-wheel rover motor control requires, without designing it from scratch
  - ros2_canopen (Fraunhofer IPA / ROS-Industrial) provides turnkey ROS 2
    Humble integration with lifecycle nodes, CiA 402 drivers, and ros2_control
    hardware interfaces — the only CAN protocol with a maintained ROS 2 stack
  - CANopenNode + CanOpenSTM32 provides direct STM32 HAL integration with
    CubeMX compatibility and working examples including STM32F103 Blue Pill
  - Largest open-source ecosystem (~1,800 GitHub stars vs ~399 for libcanard)
    with video tutorials, CSS Electronics guides, and decades of industrial use
  - Plain CAN rejected: at 8 nodes with motor control + fault reporting,
    would require reimplementing half of CANopen without the debugging tooling
  - OpenCyphal rejected: more elegant architecture but steeper learning curve,
    no ROS 2 integration, register-level STM32 driver (no HAL), thinner community

## USEFUL COMMANDS REFERENCE

### Dell Host
- `ssh jetson` → connect to Jetson
- `jsync` → sync ~/ros2_ws/ to Jetson
- `jscp <file> talos@192.168.1.211:<path>` → copy file to Jetson
- `docker buildx build --platform linux/arm64 --tag <n> --load .`
- `cd ~/github/RobertUN && git pull origin main` → sync repository

### IsaacUN
- `conda activate ros2` → enter ROS 2 development environment
- `source /opt/ros/jazzy/setup.bash` → activate system ROS 2 (Terminal B only)
- `source ~/Github/namontoy/IsaacSim-ros_workspaces/build_ws/jazzy/isaac_sim_ros_ws/install/setup.bash` → source Isaac workspace
- `bash ~/isaac-sim/launch_isaacsim.bash` → safe Isaac Sim launcher (ALWAYS use this)
- `source ~/isaac-sim/unros2.bash` → neutralize ROS 2 env before Isaac Sim
- `cd ~/Github/namontoy/IsaacSim-ros_workspaces && ./build_ros.sh -d jazzy -v 24.04` → rebuild Docker workspace
- `docker images` → list available Docker images
- `tmux new-session -s <n>` → start a new named tmux session
- `tmux ls` → list running tmux sessions
- `tmux attach -t <n>` → reattach to a session
- `nvcc --version` → verify CUDA toolkit

### Jetson — General
- `python3 ~/zed2i/test_zed.py` → test ZED camera (needs USB 3.x cable)
- `ros2 run demo_nodes_cpp talker` → test ROS 2
- `docker run --env NVIDIA_VISIBLE_DEVICES=all <image> nvcc --version`
- `sudo nvpmodel -m 0 && sudo jetson_clocks` → set maximum performance mode
- `jtop` → interactive hardware monitor (GPU, CPU, RAM, power, temperature)
- `sudo tegrastats --interval 500` → raw text hardware monitor (no install needed)

### Jetson — ZED ROS 2 Wrapper
- Safety check before any launch:
  `ps aux | grep component_container | grep -v grep`
- Kill orphaned processes:
  `pkill -f component_container_isolated && pkill -f robot_state_publisher`
- Launch ZED node:
  `export ZED_SDK_VERBOSE=1 && source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/local_setup.bash && ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i`
- Verify topics: `ros2 topic list | grep zed`
- Check IMU rate: `ros2 topic hz /zed/zed_node/imu/data --window 20`
- Check nodes: `ros2 node list`
- Run ZED diagnostic: `/usr/local/zed/tools/ZED_Diagnostic -c`

### Xavier NX — LLM Inference
- `xavier` → SSH to Xavier NX (alias on Dell)
- `~/cleanup_gpu.sh` → reset GPU VMM state after crashed model load
- `sudo nvpmodel -q` → verify power mode (should show MODE_20W_6CORE)
- `sudo tegrastats --interval 1000` → monitor GPU/CPU/memory/power/temperature
- `free -h` → check available RAM before running inference
- Run benchmark (Qwen2.5-3B):
  `~/github/llama.cpp/build/bin/llama-bench -m /data/models/Qwen2.5-3B-Instruct-Q4_K_M.gguf -p 512 -n 128 -ngl 99 -fa 1`
- Run benchmark (Gemma-3-4B — requires reduced batch):
  `~/github/llama.cpp/build/bin/llama-bench -m /data/models/gemma-3-4b-it-Q4_K_M.gguf -p 128 -n 64 -ngl 99 -fa 1 -b 512 -ub 256`
- Interactive chat (rover commands, 32k context):
  `~/github/llama.cpp/build/bin/llama-cli -m /data/models/Qwen2.5-3B-Instruct-Q4_K_M.gguf -ngl 99 -fa 1 -c 32768 -t 4 --temp 0.3 --conversation --chat-template chatml`
- List available devices: `~/github/llama.cpp/build/bin/llama-cli --list-devices`
- **can0 is now automatic at boot** via `robertun-can0.service` — no manual steps
- `systemctl status robertun-can0.service` → check CAN bring-up succeeded
- `sudo systemctl restart robertun-can0.service` → re-run pinmux + bring-up by hand
- `ip -details -s link show can0` → bitrate, sjw, state, berr-counter, packet counts
- `sudo busybox devmem 0x0c303018 && sudo busybox devmem 0x0c303010` → read pinmux
  back (expect `0x0000C458` / `0x0000C400`)
- `candump -tz -x can0` → monitor traffic with timestamps + RX/TX direction flag
- `cansend can0 123#DEADBEEF` → send test frame
- `cangen can0 -g 2 -I i -L 8 -n 500` → 500-frame load test
- `cat /proc/device-tree/bus@0/mttcan@c310000/status` → verify CAN hardware active

### daedalus — CANable V2.0 Pro (debug sniffer)
- `sudo ip link set can0 down && sudo ip link set can0 type can bitrate 250000 sjw 16 loopback off && sudo ip link set can0 up`
- `candump -tz -x can0` → independent witness of what is actually on the wire
- `lsusb | grep 1d50` → confirm candleLight firmware (`1d50:606f`) is running

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

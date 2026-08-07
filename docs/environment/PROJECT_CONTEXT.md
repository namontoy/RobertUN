# Robotics Development Environment — Project Context Document
# Last updated: August 6, 2026 (CAN bus W1 COMPLETE — MKS CANable V2.0 Pro flashed
# to candleLight, SN65HVD230 wired to J17, two-node physical bus validated at
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
**Still unproven, this is the actual W3 work:** STM32 HAL UART configuration;
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

6. **CAN Bus — STM32 firmware:**
   - Configure bxCAN peripheral registers (bit timing for target bitrate)
   - Implement acceptance filter with mask-based ID table
   - Test frame exchange: Jetson candump ↔ STM32 cansend and vice versa

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

# Session Command History — April 3, 2026
# ZED ROS 2 Wrapper Installation + Dev Container IntelliSense
#
# This document is a thematic record of all terminal commands executed
# during this session. Commands are grouped by topic rather than
# chronological order. Machine context is indicated for each section.
# All Jetson commands were executed via SSH from the Dell laptop.
#
# Machines:
#   [JETSON]    talos@orion    — Jetson Orin Nano (192.168.1.211)
#   [DELL]      talos@deadelus — Dell laptop host (192.168.1.212)
#   [CONTAINER] ros@ros2-dev   — VS Code Dev Container (runs on Dell)

# =============================================================================
# 1. ENVIRONMENT VERIFICATION
# Verify the full software stack before starting any installation work.
# All three checks must match: L4T R36.5 + ZED SDK 5.2.2 + CUDA 12.6
# =============================================================================

# [JETSON] Check the L4T (Linux for Tegra) version — ground truth of JetPack
cat /etc/nv_tegra_release

# [JETSON] Check ZED SDK version via the Python API (most authoritative source)
python3 -c "import pyzed.sl as sl; print(sl.Camera().get_sdk_version())"

# [JETSON] Check the ZED SDK shared library exists and is the right size
ls -la /usr/local/zed/lib/libsl_zed.so

# [JETSON] Verify CUDA toolkit version
nvcc --version

# [JETSON] Verify the workspace is clean before starting
ls ~/ros2_ws/src/

# [JETSON] Check available disk space
df -h ~/ros2_ws/

# [JETSON] Check ROS 2 distro and rosdep availability
echo $ROS_DISTRO && rosdep --version


# =============================================================================
# 2. JETSON GIT CONFIGURATION
# Configure Git identity and SSH key authentication for GitHub.
# Required before any git clone or push operations on the Jetson.
# =============================================================================

# [JETSON] Configure Git user identity (same values as Dell)
git config --global user.name "namontoy"
git config --global user.email "namontoy@unal.edu.co"

# [JETSON] Generate a new ED25519 SSH key pair for GitHub authentication
# When prompted for passphrase, press Enter twice (no passphrase)
ssh-keygen -t ed25519 -C "Jetson Orin Nano - orion" -f ~/.ssh/id_ed25519_github

# [JETSON] Print the public key — copy this entire line to GitHub Settings
cat ~/.ssh/id_ed25519_github.pub

# [JETSON] Add GitHub SSH config so Git automatically uses the right key
echo "Host github.com
  HostName github.com
  User git
  IdentityFile ~/.ssh/id_ed25519_github" >> ~/.ssh/config

# [JETSON] Verify the SSH config looks correct
cat ~/.ssh/config

# [JETSON] Test GitHub authentication with the explicit key (first time — adds to known_hosts)
ssh -i ~/.ssh/id_ed25519_github -T git@github.com

# [JETSON] Test GitHub authentication via SSH config (no -i flag — proves config works)
ssh -T git@github.com


# =============================================================================
# 3. ZED ROS 2 WRAPPER — REPOSITORY DISCOVERY
# Determine the correct repository structure before cloning.
# The wrapper uses a single master branch (no per-distro branches).
# =============================================================================

# [JETSON] Check available branches — discovers there is no 'humble' branch
git ls-remote --heads git@github.com:stereolabs/zed-ros2-wrapper.git

# [JETSON] Check companion repository branches
git ls-remote --heads git@github.com:stereolabs/zed-ros2-description.git

# [JETSON] Clone the wrapper from master branch (only branch available)
cd ~/ros2_ws/src/
git clone https://github.com/stereolabs/zed-ros2-wrapper.git

# [JETSON] Verify the cloned repository structure
ls ~/ros2_ws/src/zed-ros2-wrapper/

# [JETSON] Check if any git submodules are registered (answer: none)
cat ~/ros2_ws/src/zed-ros2-wrapper/.gitmodules

# [JETSON] Explicitly try to initialize submodules (confirms there are none)
cd ~/ros2_ws/src/zed-ros2-wrapper/ && git submodule update --init --recursive

# [JETSON] Find all ROS 2 packages in the wrapper
find ~/ros2_ws/src/zed-ros2-wrapper/ -name "package.xml" | sort

# [JETSON] Inspect the wrapper's declared dependencies
cat ~/ros2_ws/src/zed-ros2-wrapper/zed_wrapper/package.xml
cat ~/ros2_ws/src/zed-ros2-wrapper/zed_components/package.xml

# [JETSON] Check CMakeLists for find_package calls to understand all dependencies
cat ~/ros2_ws/src/zed-ros2-wrapper/zed_components/CMakeLists.txt | \
  grep -i "zed_msgs\|zed_interfaces\|find_package"

# [JETSON] Read the README for the official installation instructions
head -100 ~/ros2_ws/src/zed-ros2-wrapper/README.md


# =============================================================================
# 4. ZED ROS 2 WRAPPER — DEPENDENCY INSTALLATION
# Install all system-level dependencies before building.
# Order matters: JetPack dev packages first, then rosdep.
# =============================================================================

# [JETSON] Install JetPack dev packages — REQUIRED for CMake to find CUDA
# Without these, colcon build fails with "Specify CUDA_TOOLKIT_ROOT_DIR"
sudo apt install -y nvidia-jetpack nvidia-jetpack-dev

# [JETSON] Initialize rosdep (first time setup on Jetson)
sudo rosdep init
rosdep update

# [JETSON] Install all ROS 2 dependencies declared in every package.xml
# -r: continue even if some optional packages fail
# --ignore-src: don't reinstall packages already present as source
# -y: auto-confirm all apt prompts
cd ~/ros2_ws && rosdep install --from-paths src --ignore-src -r -y


# =============================================================================
# 5. ZED ROS 2 WRAPPER — BUILD
# Compile the wrapper with colcon. Build takes ~3-4 minutes on Orin Nano.
# Skip zed_debug (internal Stereolabs package, not needed).
# =============================================================================

# [JETSON] Build the workspace
# --symlink-install: config/script changes take effect without rebuilding
# --cmake-args -DCMAKE_BUILD_TYPE=Release: full compiler optimization
# --parallel-workers $(nproc): use all 6 CPU cores
cd ~/ros2_ws && colcon build \
  --symlink-install \
  --packages-skip zed_debug \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --parallel-workers $(nproc)

# [JETSON] Verify the install directory was created correctly
ls ~/ros2_ws/install/

# [JETSON] Source the workspace and verify ROS 2 discovers all ZED packages
source ~/ros2_ws/install/local_setup.bash && \
  ros2 pkg list | grep zed

# [JETSON] Verify the compiled library links correctly to ZED SDK and CUDA
# Look for libsl_zed.so and libcuda.so — "not found" would indicate a problem
ldd ~/ros2_ws/install/zed_components/lib/libzed_camera_component.so | \
  grep -i "zed\|cuda\|not found"

# [JETSON] Add workspace sourcing to .bashrc for automatic loading
echo "source ~/ros2_ws/install/local_setup.bash" >> ~/.bashrc


# =============================================================================
# 6. ZED SDK — MODEL AND RESOURCE VERIFICATION
# Verify all TensorRT models are pre-optimized before launching the ROS node.
# The ZED SDK will hang indefinitely if it tries to download/optimize models
# during the first ROS 2 launch without proper setup.
# =============================================================================

# [JETSON] List all cached AI models and TensorRT engines
ls -lah /usr/local/zed/resources/

# [JETSON] Run the full ZED diagnostic (generates ZED_Diagnostic_Results.json)
# This is the definitive check — confirms every model's optimization status
/usr/local/zed/tools/ZED_Diagnostic -c 2>&1 | head -60

# [JETSON] Test network connectivity to Stereolabs download servers
# A 302 redirect followed by 200 OK means the server is reachable
curl -v --max-time 10 \
  http://download.stereolabs.com/ai/model/neural_depth_light_5.2.model \
  --head 2>&1 | tail -20


# =============================================================================
# 7. CYCLONE DDS — LOOPBACK PEER FIX (CRITICAL)
# This was the root cause of ALL ZED launch failures.
# Without 127.0.0.1 as the first peer, the ROS 2 composable node
# load_node service call fails silently — the component_container_isolated
# starts but the ZED component is never injected into it.
# Must be applied to: Jetson, Dell host, and Dev Container.
# =============================================================================

# [JETSON] Check the current Cyclone DDS configuration
echo "RMW: $RMW_IMPLEMENTATION" && cat ~/.ros/cyclone_dds.xml

# [JETSON] Apply the loopback fix — 127.0.0.1 MUST be the first peer
cat > ~/.ros/cyclone_dds.xml << 'EOF'
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
        <!-- Loopback MUST be first — enables local composable node service calls -->
        <Peer address="127.0.0.1"/>
        <!-- Dell laptop - Ethernet address via switch -->
        <Peer address="192.168.1.212"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

# [JETSON] Verify the fix was applied correctly
cat ~/.ros/cyclone_dds.xml

# [DELL] Apply the loopback fix to the Dell host config
cat > ~/.ros/cyclone_dds.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="enp4s0" multicast="false"/>
      </Interfaces>
    </General>
    <Discovery>
      <Peers>
        <!-- Loopback MUST be first — enables local composable node service calls -->
        <Peer address="127.0.0.1"/>
        <!-- Jetson Orin Nano - Ethernet address via switch -->
        <Peer address="192.168.1.211"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

# [DELL] Apply the loopback fix to the Dev Container config
cat > ~/ros2_ws/.devcontainer/cyclone_dds.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <!-- Use Ethernet interface - --network=host shares Dell's network stack -->
        <NetworkInterface name="enp4s0" multicast="false"/>
      </Interfaces>
    </General>
    <Discovery>
      <Peers>
        <!-- Loopback MUST be first — enables local composable node service calls -->
        <Peer address="127.0.0.1"/>
        <!-- Jetson Orin Nano - Ethernet address via switch -->
        <Peer address="192.168.1.211"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

# [DELL] Verify both Dell configs look correct
echo "=== Host config ===" && cat ~/.ros/cyclone_dds.xml && \
  echo "=== Dev Container config ===" && \
  cat ~/ros2_ws/.devcontainer/cyclone_dds.xml


# =============================================================================
# 8. ZED DEPTH MODE CONFIGURATION
# The default NEURAL_LIGHT depth mode was deprecated in SDK 5.x.
# Changed to PERFORMANCE for initial testing (no AI dependency),
# then switched to NEURAL (correct current mode name) for production.
# Uses --symlink-install so editing src/ immediately affects install/.
# =============================================================================

# [JETSON] Check the current depth mode setting
grep "depth_mode" \
  ~/ros2_ws/src/zed-ros2-wrapper/zed_wrapper/config/common_stereo.yaml

# [JETSON] Switch from NEURAL_LIGHT to PERFORMANCE for initial testing
# (avoids TensorRT model loading on first launch)
sed -i "s/depth_mode: 'NEURAL_LIGHT'/depth_mode: 'PERFORMANCE'/" \
  ~/ros2_ws/src/zed-ros2-wrapper/zed_wrapper/config/common_stereo.yaml

# [JETSON] Verify the change took effect in both src/ and install/ (symlink check)
grep "depth_mode" \
  ~/ros2_ws/src/zed-ros2-wrapper/zed_wrapper/config/common_stereo.yaml && \
grep "depth_mode" \
  ~/ros2_ws/install/zed_wrapper/share/zed_wrapper/config/common_stereo.yaml

# [JETSON] Switch from deprecated PERFORMANCE to correct NEURAL mode
sed -i "s/depth_mode: 'PERFORMANCE'/depth_mode: 'NEURAL'/" \
  ~/ros2_ws/src/zed-ros2-wrapper/zed_wrapper/config/common_stereo.yaml

# [JETSON] Verify NEURAL mode is now active
grep "depth_mode" \
  ~/ros2_ws/src/zed-ros2-wrapper/zed_wrapper/config/common_stereo.yaml


# =============================================================================
# 9. ZED ROS 2 WRAPPER — LAUNCHING AND SAFETY RULES
# CRITICAL: Always check for orphaned processes before relaunching.
# Orphaned component_container_isolated processes hold the USB camera handle
# and cause silent deadlocks that look identical to other failures.
# =============================================================================

# [JETSON] SAFETY CHECK — always run this before launching
# Output should be empty. If not, kill survivors before proceeding.
ps aux | grep -E "component_container|robot_state|zed" | grep -v grep

# [JETSON] Kill orphaned processes from previous failed launch attempts
pkill -f component_container_isolated && pkill -f robot_state_publisher

# [JETSON] Set maximum performance mode (run after every reboot)
sudo nvpmodel -m 0 && sudo jetson_clocks

# [JETSON] Full launch command with verbose SDK output
# Source full ROS 2 environment + workspace before launching
export ZED_SDK_VERBOSE=1 && \
  source /opt/ros/humble/setup.bash && \
  source ~/ros2_ws/install/local_setup.bash && \
  ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i


# =============================================================================
# 10. ZED ROS 2 WRAPPER — LIVE DATA VERIFICATION
# Run these in a second terminal while the ZED node is running.
# Confirms topics are publishing and real sensor data is flowing.
# =============================================================================

# [JETSON] Source the environment in a new terminal
source /opt/ros/humble/setup.bash && \
  source ~/ros2_ws/install/local_setup.bash

# [JETSON] List all active ROS 2 nodes
ros2 node list

# [JETSON] List all ZED topics being published
ros2 topic list | grep zed

# [JETSON] Verify IMU is publishing at ~100Hz (should show ~100.0 Hz)
ros2 topic hz /zed/zed_node/imu/data --window 20

# [JETSON] Read one real IMU message — confirms actual sensor data is flowing
# linear_acceleration z ≈ 9.8 m/s² confirms gravity is being measured
ros2 topic echo /zed/zed_node/imu/data --once

# [JETSON] List all components loaded in the ZED container
ros2 component list /zed/zed_container


# =============================================================================
# 11. SYSTEM DIAGNOSTICS AND PROCESS INSPECTION
# Commands used during debugging of the silent hang issue.
# These are useful general-purpose diagnostic tools for ROS 2 nodes.
# =============================================================================

# [JETSON] Check process state, CPU, memory, and elapsed time by PID
ps -o pid,stat,pcpu,pmem,etime,cmd -p <PID1>,<PID2>

# [JETSON] Check what kernel function each thread is waiting in
# futex_wait = waiting on a mutex, __skb_wait_for_more_packets = network socket
sudo cat /proc/<PID>/task/*/wchan | sort | uniq -c

# [JETSON] Check process memory and thread count
sudo cat /proc/<PID>/status | grep -E "State|Threads|VmRSS|VmPeak"

# [JETSON] Check open file descriptors (reveals open sockets and pipes)
sudo ls -la /proc/<PID>/fd/ | grep -v "regular\|directory"

# [JETSON] Check active TCP connections for a specific process
sudo ss -tnp | grep <PID>

# [JETSON] Attach strace to a running process to observe system calls
# -e trace=network,read,write,poll selects only I/O-related calls
sudo strace -p <PID> -e trace=network,read,write,poll,select,epoll_wait \
  2>&1 | head -30

# [JETSON] Find the PID of the component container automatically
CPID=$(pgrep -f component_container_isolated) && echo "PID: $CPID"

# [JETSON] Check what a process is blocked on by reading kernel stack
sudo cat /proc/<PID>/stack

# [JETSON] Read the ROS 2 launch log for a specific session
# Session folder name is shown at the start of every ros2 launch output
cat ~/.ros/log/<session-folder>/launch.log

# [JETSON] List all files in a ROS 2 log session folder
ls ~/.ros/log/<session-folder>/

# [JETSON] Monitor GPU and system resources in real time (raw text output)
sudo tegrastats --interval 500

# [JETSON] Monitor GPU and system resources interactively
jtop


# =============================================================================
# 12. JTOP INSTALLATION
# jtop is the recommended hardware monitor for Jetson platforms.
# Provides interactive GPU/CPU/RAM/power/temperature dashboard.
# Requires a fresh login session after first install.
# =============================================================================

# [JETSON] Install jetson-stats (provides the jtop command)
# Note: --break-system-packages flag NOT needed on Ubuntu 22.04
sudo pip3 install jetson-stats

# [JETSON] Restart the jtop background service
sudo systemctl restart jtop

# [JETSON] Launch the interactive hardware monitor
jtop


# =============================================================================
# 13. ZED SDK VERIFICATION (STANDALONE — WITHOUT ROS 2)
# Test the ZED SDK directly before involving the ROS 2 wrapper.
# If these fail, the problem is in the SDK installation, not the wrapper.
# =============================================================================

# [JETSON] Check ZED SDK version
python3 -c "import pyzed.sl as sl; print(sl.Camera().get_sdk_version())"

# [JETSON] List detected cameras (SDK 5.2+ feature)
/usr/local/zed/tools/ZED_Explorer -l

# [JETSON] Run full hardware and software diagnostic
# Generates ZED_Diagnostic_Results.json — first thing Stereolabs support asks for
/usr/local/zed/tools/ZED_Diagnostic -c

# [JETSON] Run diagnostic and optimize all neural depth models
/usr/local/zed/tools/ZED_Diagnostic -nrlo_all

# [JETSON] Quick Python camera open test
python3 -c "
import pyzed.sl as sl
c = sl.Camera()
p = sl.InitParameters()
p.camera_resolution = sl.RESOLUTION.HD720
print('Open:', c.open(p))
c.close()
"

# [JETSON] Verify USB 3.0 connection (Stereolabs vendor ID: 2b03)
lsusb | grep 2b03

# [JETSON] Check USB connection speed (must show 5000 for USB 3.0, not 480)
for dev in /sys/bus/usb/devices/*; do
  if [ -f "$dev/idVendor" ] && \
     [ "$(cat $dev/idVendor 2>/dev/null)" = "2b03" ]; then
    echo "$(cat $dev/product 2>/dev/null): $(cat $dev/speed 2>/dev/null) Mbps"
  fi
done

# [JETSON] Check dmesg for USB errors or speed negotiation issues
dmesg | grep -i "usb\|2b03\|SuperSpeed\|high-speed"

# [JETSON] Verify the ZED SDK shared library is present
ls -la /usr/local/zed/lib/libsl_zed.so

# [JETSON] Check what the compiled ZED ROS component links to at runtime
ldd ~/ros2_ws/install/zed_components/lib/libzed_camera_component.so | \
  grep -i "zed\|cuda\|not found"


# =============================================================================
# 14. PERFORMANCE MODE — JETSON ORIN NANO
# Must be run after every reboot. Not persistent across reboots.
# nvpmodel sets the power envelope; jetson_clocks locks frequencies at max.
# =============================================================================

# [JETSON] Set MAXN (maximum performance) power mode
sudo nvpmodel -m 0

# [JETSON] Lock all CPU/GPU/memory clocks at their maximum frequencies
sudo jetson_clocks

# [JETSON] Both together (typical usage)
sudo nvpmodel -m 0 && sudo jetson_clocks

# [JETSON] Check current power mode
sudo nvpmodel -q

# [JETSON] Monitor system in real time during ZED operation
sudo tegrastats --interval 2000


# =============================================================================
# 15. DEV CONTAINER — ZED INTELLISENSE SETUP
# Install ZED SDK headers inside the Dev Container for VS Code IntelliSense.
# All commands in this section are inside the container (ros@ros2-dev).
# The Dockerfile changes are made on the Dell host before rebuilding.
# =============================================================================

# [CONTAINER] Verify the ZED include directory exists after rebuild
ls /usr/local/zed/include/

# [CONTAINER] List the available ZED header files
ls /usr/local/zed/include/sl/

# [CONTAINER] Read the first few lines of Camera.hpp (confirms real SDK headers)
head -5 /usr/local/zed/include/sl/Camera.hpp

# [CONTAINER] Check permissions on the ZED directory (debugging permission issues)
sudo stat /usr/local/zed && sudo stat /usr/local/zed/include

# [CONTAINER] Check the zed directory ownership from outside (using sudo)
sudo ls -la /usr/local/zed/

# [CONTAINER] Create a test file to verify IntelliSense resolves ZED types
cat > /home/ros/ros2_ws/zed_intellisense_test.cpp << 'EOF'
#include <sl/Camera.hpp>

int main() {
    // Hover over these types in VS Code to verify IntelliSense is working
    sl::Camera zed;
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION::HD720;
    init_params.depth_mode = sl::DEPTH_MODE::NEURAL;

    sl::Mat image;
    sl::RuntimeParameters runtime_params;

    return 0;
}
EOF

# [DELL] Remove the test file after IntelliSense is verified (host side)
rm ~/ros2_ws/zed_intellisense_test.cpp

# [DELL] Verify the Dev Container Dockerfile ZED layer
grep -A 30 "ZED SDK C++ headers" ~/ros2_ws/.devcontainer/Dockerfile

# [DELL] Verify the download URL for ZED SDK 5.2.2 ubuntu22/CUDA12 (no actual download)
curl -L --max-time 10 --head \
  "https://download.stereolabs.com/zedsdk/5.2/cu12/ubuntu22" \
  2>&1 | grep -E "HTTP|Location|Content-Length"


# =============================================================================
# 16. DOCKER LAYER CACHING — FORCE REBUILD
# When a Dockerfile layer is cached and changes don't take effect,
# bust the cache by adding a trivial comment change to the layer.
# Docker keys the cache on exact instruction text — any change forces re-execution.
# =============================================================================

# [DELL] Check the current content of the ZED Dockerfile layer
grep -A 30 "ZED SDK C++ headers" ~/ros2_ws/.devcontainer/Dockerfile

# NOTE: To bust Docker cache, edit the comment at the top of the affected
# RUN block — for example, change:
#   # LAYER 4b: ZED SDK C++ headers for IntelliSense
# to:
#   # LAYER 4b: ZED SDK C++ headers for IntelliSense (rebuilt: 2026-04-03)
# Then rebuild with: Ctrl+Shift+P → Dev Containers: Rebuild and Reopen in Container


# =============================================================================
# 17. ROS 2 ENVIRONMENT DIAGNOSTICS
# Commands for diagnosing ROS 2 environment issues.
# Useful when topics are not visible or nodes don't communicate.
# =============================================================================

# [JETSON] Check ROS 2 environment variables
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID" && \
  echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION" && \
  which ros2

# [JETSON] Check the Cyclone DDS XML config in use
cat ~/.ros/cyclone_dds.xml

# [JETSON] List all active ROS 2 nodes
ros2 node list

# [JETSON] List all active ROS 2 topics
ros2 topic list

# [JETSON] Check publishing rate of a specific topic
ros2 topic hz /zed/zed_node/imu/data --window 20

# [JETSON] Echo one message from a topic
ros2 topic echo /zed/zed_node/imu/data --once

# [JETSON] View diagnostic messages from the ZED node
ros2 topic echo /diagnostics

# [JETSON] Check ROS 2 log files
# Session folder is shown at the start of every ros2 launch output
ls ~/.ros/log/ | tail -10
cat ~/.ros/log/latest/launch.log


# =============================================================================
# 18. GIT OPERATIONS — REPOSITORY MANAGEMENT
# Commands for keeping the RobertUN repository up to date.
# All repository operations from the Dell laptop.
# =============================================================================

# [DELL] Check available branches on a remote repository (without cloning)
git ls-remote --heads git@github.com:stereolabs/zed-ros2-wrapper.git

# [DELL] Sync the local RobertUN repository with GitHub
cd ~/github/RobertUN && git pull origin main

# [DELL] Stage all changes, commit, and push to GitHub
cd ~/github/RobertUN && \
  git add -A && \
  git commit -m "Your commit message here" && \
  git push origin main

# [JETSON] Test GitHub SSH authentication (after key setup)
ssh -T git@github.com

# Linux Terminal Commands — Complete Session History
# Robotics Development Environment Setup
# Ubuntu 22.04 + NVIDIA GTX 1650 Ti + Jetson Orin Nano
# Generated: March 2026

---

## PHASE 1 — NVIDIA Driver Installation

```bash
# Disable nouveau open-source driver (conflicts with NVIDIA proprietary)
sudo bash -c "echo 'blacklist nouveau' >> /etc/modprobe.d/blacklist-nouveau.conf"
sudo bash -c "echo 'options nouveau modeset=0' >> /etc/modprobe.d/blacklist-nouveau.conf"

# Verify the blacklist file was written correctly
cat /etc/modprobe.d/blacklist-nouveau.conf

# Update initramfs and reboot
sudo update-initramfs -u
sudo reboot

# After reboot — verify nouveau is disabled (expected: empty output)
lsmod | grep nouveau

# Check what graphics modules are loaded
lsmod | grep -E 'nvidia|nouveau|i915|amdgpu'

# Check current display configuration
xrandr --query

# Check which NVIDIA driver is recommended for your GPU
ubuntu-drivers devices

# Add NVIDIA PPA and install recommended driver
sudo apt update
sudo add-apt-repository ppa:graphics-drivers/ppa -y
sudo apt update

# Install the recommended driver (580-open for GTX 1650 Ti Mobile)
sudo apt install nvidia-driver-580-open -y
sudo reboot

# After reboot — verify driver is loaded
nvidia-smi

# Verify NVIDIA modules are loaded
lsmod | grep nvidia

# Check monitor detection
xrandr --query

# Permanently disable Wayland (fix for black second monitor + X11 preference)
sudo nano /etc/gdm3/custom.conf
# -> Uncomment: WaylandEnable=false
sudo reboot

# Verify X11 is active (expected: x11)
echo $XDG_SESSION_TYPE

# Verify both monitors are detected
xrandr --query

# Hide Linux Mint partition icons from GNOME dock
gsettings set org.gnome.shell.extensions.dash-to-dock show-mounts false
```

---

## PHASE 2 — CUDA Toolkit Installation

```bash
# Add NVIDIA CUDA repository
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update

# Verify CUDA 12.6 is available before installing
apt-cache policy cuda-toolkit-12-6 | head -5

# Install CUDA Toolkit 12.6
sudo apt install cuda-toolkit-12-6 -y

# Add CUDA to PATH permanently
echo 'export PATH=/usr/local/cuda-12.6/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc

# Verify CUDA installation
nvcc --version
```

---

## PHASE 2.5 — C/C++ Dev Tools + Java + VS Code

```bash
# Install C/C++ build tools
sudo apt install -y \
  build-essential \
  cmake \
  cmake-curses-gui \
  ninja-build \
  gdb \
  valgrind \
  clang \
  clang-format \
  clang-tidy \
  pkg-config \
  libssl-dev \
  libusb-1.0-0-dev

# Verify GCC and CMake
gcc --version && cmake --version

# Install Java JDK 21
sudo apt install -y openjdk-21-jdk

# Verify Java
java -version && javac -version

# Install VS Code
wget -qO- https://packages.microsoft.com/keys/microsoft.asc \
  | gpg --dearmor > packages.microsoft.gpg
sudo install -o root -g root -m 644 packages.microsoft.gpg \
  /etc/apt/trusted.gpg.d/
sudo sh -c 'echo "deb [arch=amd64 signed-by=/etc/apt/trusted.gpg.d/packages.microsoft.gpg] \
  https://packages.microsoft.com/repos/vscode stable main" \
  > /etc/apt/sources.list.d/vscode.list'
sudo apt update
sudo apt install code -y

# Verify VS Code
code --version

# Install essential VS Code extensions
code --install-extension ms-vscode.cpptools
code --install-extension ms-vscode.cpptools-extension-pack
code --install-extension twxs.cmake
code --install-extension ms-vscode.cmake-tools
code --install-extension ms-vscode-remote.remote-ssh
code --install-extension ms-vscode-remote.remote-ssh-edit
code --install-extension ms-python.python
code --install-extension streetsidesoftware.code-spell-checker
code --install-extension eamodio.gitlens
```

---

## PHASE 2.6 — Python Environment

```bash
# Check Python version (should be 3.10.x on Ubuntu 22.04)
python3 --version
which python3
ls /usr/bin/python*

# Install pip and Python development tools
sudo apt install -y \
  python3-pip \
  python3-venv \
  python3-dev \
  python3-setuptools \
  python3-wheel

# Verify pip
pip3 --version

# Upgrade pip to latest version
python3 -m pip install --upgrade pip
pip3 --version

# Add ~/.local/bin to PATH (fixes "not on PATH" warnings)
echo 'export PATH=$HOME/.local/bin:$PATH' >> ~/.bashrc
source ~/.bashrc

# Verify PATH update
echo $PATH | grep -o '.local/bin'

# Set python command to point to python3
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.10 1

# Install essential Python packages for robotics
pip3 install \
  numpy \
  scipy \
  matplotlib \
  pyserial \
  transforms3d \
  smbus2 \
  Pillow \
  opencv-python \
  pytest

# Verify key packages
python3 -c "import numpy; import cv2; import serial; print('All packages OK')"

# Verify pytest is found
pytest --version
```

---

## PHASE 3 — JetPack 6.5 on Jetson (via SDK Manager)

```bash
# Install SDK Manager (after downloading .deb from developer.nvidia.com)
sudo apt install ./sdkmanager_*.deb

# Launch SDK Manager (requires NVIDIA Developer account)
sdkmanager

# Put Jetson in recovery mode, then verify it is detected
lsusb | grep NVIDIA

# After flashing — on the JETSON: check JetPack version
cat /etc/nv_tegra_release

# Install CUDA on Jetson via apt (minimal installation approach)
sudo apt update

# Verify NVIDIA repos are configured on Jetson
cat /etc/apt/sources.list.d/*.list | grep nvidia

# Check available CUDA toolkit packages
apt-cache search cuda | grep -i "cuda-toolkit"

# Install CUDA Toolkit 12.6 on Jetson
sudo apt install cuda-toolkit-12-6 -y

# Add CUDA to PATH on Jetson
echo 'export PATH=/usr/local/cuda-12.6/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc

# Verify CUDA on Jetson
nvcc --version

# Check Python version on Jetson
python3 --version

# Check available memory and storage on Jetson
free -h && df -h /
```

---

## PHASE 4 — SSH + Workflow Setup

```bash
# On HOST: generate SSH key pair
ssh-keygen -t ed25519 -C "host-to-jetson"

# Copy public key to Jetson for passwordless authentication
ssh-copy-id talos@192.168.1.11

# Add Jetson to SSH config
cat >> ~/.ssh/config << EOF

Host jetson
    HostName 192.168.1.11
    User talos
    IdentityFile ~/.ssh/id_ed25519
EOF

# Test SSH connection
ssh jetson echo "Jetson is alive!"

# Create ROS2 workspace on host
mkdir -p ~/ros2_ws/src

# Add convenience aliases to .bashrc
echo "alias jetson='ssh jetson'" >> ~/.bashrc
echo "alias jsync='rsync -avz --delete ~/ros2_ws/ talos@192.168.1.11:~/ros2_ws/'" >> ~/.bashrc
source ~/.bashrc

# Test jsync
jsync

# On HOST: configure static IP (192.168.1.12)
nmcli connection show
nmcli connection modify "Gotham_Castle" \
  ipv4.addresses 192.168.1.12/24 \
  ipv4.gateway 192.168.1.1 \
  ipv4.dns "8.8.8.8,8.8.4.4" \
  ipv4.method manual
nmcli connection down "Gotham_Castle" && nmcli connection up "Gotham_Castle"

# Verify host static IP
ip addr show | grep "inet " | grep -v 127.0.0.1

# On JETSON: configure static IP (192.168.1.11)
nmcli connection show
nmcli connection modify "Gotham_Castle" \
  ipv4.addresses 192.168.1.11/24 \
  ipv4.gateway 192.168.1.1 \
  ipv4.dns "8.8.8.8,8.8.4.4" \
  ipv4.method manual
nmcli connection down "Gotham_Castle" && nmcli connection up "Gotham_Castle"

# Verify Jetson static IP
ip addr show | grep "inet " | grep -v 127.0.0.1

# On JETSON: change SSH port to 44252 (0xACDC)
sudo nano /etc/ssh/sshd_config
# -> Change: #Port 22  to  Port 44252
sudo systemctl restart ssh

# Verify new port is listening
sudo ss -tlnp | grep sshd

# On HOST: update SSH config with new port
nano ~/.ssh/config
# -> Add: Port 44252

# Update jsync alias with new port
nano ~/.bashrc
# -> Change jsync to: alias jsync='rsync -avz --delete -e "ssh -p 44252" ~/ros2_ws/ talos@192.168.1.11:~/ros2_ws/'
source ~/.bashrc

# Add jscp alias for scp with correct port
echo "alias jscp='scp -P 44252'" >> ~/.bashrc
source ~/.bashrc

# Test SSH with new port
ssh jetson echo "AC/DC port working!"

# Test jsync with new port
jsync

# On JETSON: verify SSH auto-starts on boot
sudo systemctl status ssh
sudo systemctl is-enabled ssh
```

---

## PHASE 4.5 — WiFi Power Management Fix (Jetson)

```bash
# Check if WiFi power management is causing SSH delays
iwconfig wlP1p1s0 | grep "Power Management"

# Check system sleep settings
systemctl status sleep.target suspend.target

# Disable WiFi power management immediately
sudo iwconfig wlP1p1s0 power off

# Verify it took effect
iwconfig wlP1p1s0 | grep "Power Management"

# Create permanent fix via NetworkManager dispatcher
sudo nano /etc/NetworkManager/dispatcher.d/99-disable-wifi-powersave
# -> Add script content (see session notes)
sudo chmod +x /etc/NetworkManager/dispatcher.d/99-disable-wifi-powersave

# Create udev rule as backup
sudo nano /etc/udev/rules.d/70-wifi-powersave.rules
# -> Add: ACTION=="add", SUBSYSTEM=="net", KERNEL=="wlP1p1s0", RUN+="/sbin/iwconfig wlP1p1s0 power off"
sudo udevadm control --reload-rules

# Reboot and verify fix survived
sudo reboot

# After reboot — verify from HOST (tests both SSH responsiveness and power management)
time ssh jetson "iwconfig wlP1p1s0 | grep 'Power Management'"
# Expected: Power Management:off in ~0.3 seconds
```

---

## PHASE 5 — ROS 2 Humble Installation

```bash
# On HOST: check Ubuntu version and locale
lsb_release -cs
locale

# Add ROS 2 repository (both HOST and JETSON)
sudo apt install software-properties-common curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list

# Verify repository was added
cat /etc/apt/sources.list.d/ros2.list

# On HOST: install ROS 2 Humble Desktop (full install with RViz2, rqt, etc.)
sudo apt update && sudo apt install ros-humble-desktop -y
sudo apt install ros-dev-tools -y

# Add ROS 2 to PATH permanently
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify ROS 2 on host
ros2 --help | head -5
printenv | grep ROS

# On JETSON: install ROS 2 Humble Base (lighter — no GUI tools needed)
sudo apt update && sudo apt install ros-humble-ros-base -y
sudo apt install ros-dev-tools -y
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify ROS 2 on Jetson
ros2 --help | head -5
printenv | grep ROS

# Test basic ROS 2 communication (demo nodes)
# On Jetson: install demo packages
sudo apt install ros-humble-demo-nodes-cpp -y
sudo apt install ros-humble-demo-nodes-py -y

# Fix ROS 2 communication over WiFi — install Cyclone DDS (both machines)
sudo apt install ros-humble-rmw-cyclonedds-cpp -y

# Create Cyclone DDS config on HOST
mkdir -p ~/.ros
cat > ~/.ros/cyclone_dds.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="wlp5s0" multicast="false"/>
      </Interfaces>
    </General>
    <Discovery>
      <Peers>
        <Peer address="192.168.1.11"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

# Create Cyclone DDS config on JETSON
mkdir -p ~/.ros
cat > ~/.ros/cyclone_dds.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="wlP1p1s0" multicast="false"/>
      </Interfaces>
    </General>
    <Discovery>
      <Peers>
        <Peer address="192.168.1.12"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

# Set Cyclone DDS as default on BOTH machines
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
echo 'export CYCLONEDDS_URI=file://$HOME/.ros/cyclone_dds.xml' >> ~/.bashrc
source ~/.bashrc

# Test C++ communication (Jetson=talker, Host=listener)
# On Jetson: ros2 run demo_nodes_cpp talker
# On Host:   ros2 run demo_nodes_cpp listener

# Test Python communication
# On Jetson: ros2 run demo_nodes_py talker
# On Host:   ros2 run demo_nodes_py listener
```

---

## PHASE 6 — Cross-Compilation Toolchain

```bash
# Check host architecture
uname -m

# Check if cross-compiler already exists
which aarch64-linux-gnu-gcc 2>/dev/null || echo "Not installed"

# Check available disk space
df -h /

# Install ARM64 cross-compilation toolchain
sudo apt install -y \
  gcc-aarch64-linux-gnu \
  g++-aarch64-linux-gnu \
  binutils-aarch64-linux-gnu \
  qemu-user-static \
  binfmt-support \
  debootstrap

# Verify cross-compilers
aarch64-linux-gnu-gcc --version
aarch64-linux-gnu-g++ --version
qemu-aarch64-static --version
update-binfmts --display qemu-aarch64

# Install ARM64 C libraries for running ARM64 binaries on host
sudo apt install -y \
  libc6-arm64-cross \
  libstdc++6-arm64-cross \
  binutils-aarch64-linux-gnu

# Add QEMU sysroot to environment
echo 'export QEMU_LD_PREFIX=/usr/aarch64-linux-gnu' >> ~/.bashrc
source ~/.bashrc

# Create test C program
cat > /tmp/hello_jetson.c << 'EOF'
#include <stdio.h>
int main() {
    printf("Hello from ARM64! Cross-compilation works!\n");
    return 0;
}
EOF

# Cross-compile for ARM64 (static — no library dependencies)
aarch64-linux-gnu-gcc -static -o /tmp/hello_jetson_static /tmp/hello_jetson.c

# Verify it is an ARM64 binary
file /tmp/hello_jetson_static

# Run it on host via QEMU emulation
/tmp/hello_jetson_static

# Fix binfmt P-flag issue for Docker container support
# Remove existing P-flag registration
echo -1 | sudo tee /proc/sys/fs/binfmt_misc/qemu-aarch64

# Re-register QEMU without P flag
echo ':qemu-aarch64:M:0:\x7f\x45\x4c\x46\x02\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x00\xb7\x00:\xff\xff\xff\xff\xff\xff\xff\x00\xff\xff\xff\xff\xff\xff\xff\xff\xfe\xff\xff\xff:/usr/bin/qemu-aarch64-static:' \
  | sudo tee /proc/sys/fs/binfmt_misc/register

# Verify flags are now empty (no P flag)
cat /proc/sys/fs/binfmt_misc/qemu-aarch64

# The proper permanent fix — use multiarch/qemu-user-static via Docker
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

# Verify F-flag is now set (correct for Docker containers)
cat /proc/sys/fs/binfmt_misc/qemu-aarch64

# Create permanent systemd service for the fix
sudo nano /etc/systemd/system/qemu-aarch64-binfmt-fix.service
# -> Add service content (see session notes)
sudo systemctl enable qemu-aarch64-binfmt-fix.service
sudo systemctl start qemu-aarch64-binfmt-fix.service
sudo systemctl status qemu-aarch64-binfmt-fix.service

# Create permanent test binary location
mkdir -p ~/ros2_ws/testing
aarch64-linux-gnu-gcc -o ~/ros2_ws/testing/hello_jetson \
  ~/ros2_ws/testing/hello_jetson.c

# Deploy to Jetson and test
jscp ~/ros2_ws/testing/hello_jetson talos@192.168.1.11:/tmp/hello_jetson
ssh jetson "/tmp/hello_jetson"
```

---

## PHASE 7 — Docker + ARM64 Buildx (Host)

```bash
# Remove any old Docker installations
sudo apt remove -y docker docker-engine docker.io containerd runc 2>/dev/null

# Install Docker prerequisites
sudo apt install -y ca-certificates curl gnupg lsb-release

# Add Docker's official GPG key
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | \
  sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

# Add Docker's official repository
echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/etc/apt/keyrings/docker.gpg] \
  https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list

# Install Docker Engine
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io \
  docker-buildx-plugin docker-compose-plugin

# Add user to docker group (no sudo needed)
sudo usermod -aG docker $USER
newgrp docker

# Verify Docker installation
docker --version
sudo systemctl status docker | head -15

# Test Docker works without sudo
docker run hello-world

# Verify buildx is available
docker buildx version

# Create ARM64-capable builder
docker buildx create --name jetson-builder \
  --driver docker-container \
  --platform linux/amd64,linux/arm64 \
  --use

# Bootstrap the builder
docker buildx inspect --bootstrap

# List available builders and platforms
docker buildx ls

# Register QEMU for ARM64 Docker containers
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

# Test ARM64 container runs on host
docker run --rm --platform linux/arm64 ubuntu:22.04 uname -m
# Expected: aarch64

# Create test Dockerfile
mkdir -p ~/ros2_ws/docker/test
cd ~/ros2_ws/docker/test

cat > verify.sh << 'EOF'
#!/bin/bash
echo "=== Architecture ==="
uname -m
echo "=== Python ==="
python3 --version
echo "=== Container is healthy! ==="
EOF
chmod +x verify.sh

cat > Dockerfile << 'EOF'
FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
LABEL description="ARM64 test image for Jetson Orin Nano"
LABEL architecture="arm64"
RUN apt-get update && apt-get install -y \
    curl \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*
COPY verify.sh /usr/local/bin/verify.sh
CMD ["/usr/local/bin/verify.sh"]
EOF

# Build ARM64 image
docker buildx build \
  --platform linux/arm64 \
  --tag jetson-test:arm64 \
  --load \
  .

# Run the ARM64 image on host
docker run --rm --platform linux/arm64 jetson-test:arm64

# List all Docker images
docker images
```

---

## PHASE 8 — Docker + NVIDIA Container Runtime (Jetson)

```bash
# On JETSON: install Docker (same process as host)
sudo apt remove -y docker docker-engine docker.io containerd runc 2>/dev/null
sudo apt install -y ca-certificates curl gnupg lsb-release
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | \
  sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/etc/apt/keyrings/docker.gpg] \
  https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io \
  docker-buildx-plugin docker-compose-plugin
sudo usermod -aG docker $USER
newgrp docker

# Verify Docker on Jetson
docker --version
docker run hello-world

# Install NVIDIA Container Toolkit
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt update
sudo apt install -y nvidia-container-toolkit

# Configure Docker to use NVIDIA runtime
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# Verify NVIDIA runtime configuration
cat /etc/docker/daemon.json

# Set NVIDIA as DEFAULT runtime
sudo nano /etc/docker/daemon.json
# -> Add: "default-runtime": "nvidia"
sudo systemctl restart docker

# Test GPU access inside container (bare Ubuntu)
docker run --rm \
  --env NVIDIA_VISIBLE_DEVICES=all \
  ubuntu:22.04 \
  bash -c "ls /dev/nvidia*"

# Pull and test official NVIDIA L4T JetPack image
docker pull nvcr.io/nvidia/l4t-jetpack:r36.4.0
docker run --rm \
  --env NVIDIA_VISIBLE_DEVICES=all \
  nvcr.io/nvidia/l4t-jetpack:r36.4.0 \
  bash -c "nvcc --version && ls /dev/nvidia*"

# Pull Dustin Franklin's ROS2 + CUDA image for Jetson
docker pull dustynv/ros:humble-ros-base-l4t-r36.3.0
docker run --rm \
  --env NVIDIA_VISIBLE_DEVICES=all \
  --network host \
  dustynv/ros:humble-ros-base-l4t-r36.3.0 \
  bash -c "nvcc --version && ros2 --help | head -3 && ls /dev/nvidia*"

# List all images on Jetson
docker images
```

---

## PHASE 9 — VS Code Dev Containers

```bash
# On HOST: install Dev Containers extension
code --install-extension ms-vscode-remote.remote-containers

# Verify extensions are installed
code --list-extensions | grep containers
code --list-extensions | grep remote

# Verify Docker is accessible from VS Code terminal
docker info | grep -E "Server Version|Operating System|Architecture"
docker buildx ls

# Create Dev Container directory structure
mkdir -p ~/ros2_ws/.devcontainer
mkdir -p ~/ros2_ws/src
ls -la ~/ros2_ws/

# Create devcontainer.json
cat > ~/ros2_ws/.devcontainer/devcontainer.json << 'EOF'
{
    "name": "ROS2 Humble + ZED 2i Development",
    "build": {
        "dockerfile": "Dockerfile",
        "context": ".."
    },
    "runArgs": [
        "--hostname=ros2-dev",
        "--network=host",
        "--privileged",
        "--env=DISPLAY=${DISPLAY}",
        "--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw"
    ],
    ...
}
EOF
# Note: --runtime=nvidia was REMOVED (host has no NVIDIA Container Runtime)

# Create Cyclone DDS config for Dev Container
cat > ~/ros2_ws/.devcontainer/cyclone_dds.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="wlp5s0" multicast="false"/>
      </Interfaces>
    </General>
    <Discovery>
      <Peers>
        <Peer address="192.168.1.11"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

# Create Dockerfile for Dev Container
# (Ubuntu 22.04 + ROS2 Humble + CUDA headers + Python packages)
# See full Dockerfile in session notes
wc -l ~/ros2_ws/.devcontainer/Dockerfile
# Expected: ~154 lines

# Verify all three files are in place
ls -la ~/ros2_ws/.devcontainer/

# Open Dev Container in VS Code:
# Ctrl+Shift+P -> "Dev Containers: Reopen in Container"

# After container starts — verify environment inside container
whoami && hostname
ros2 --help | head -3
printenv | grep ROS
printenv | grep -E "RMW|CYCLONE"
cat ~/.ros/cyclone_dds.xml
python3 --version
python3 -c "import numpy; import cv2; print('Python packages OK')"
ls -la ~/ros2_ws/

# Test ROS2 communication Dev Container <-> Jetson
# On Jetson: ros2 run demo_nodes_cpp talker
# In container: ros2 run demo_nodes_cpp listener

# Create IntelliSense test file
cat > ~/ros2_ws/src/test_intellisense.cpp << 'EOF'
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
// Open in VS Code and verify hover/completion works
EOF
```

---

## ZED SDK Installation (Jetson)

```bash
# Check Jetson state before installing
cat /etc/nv_tegra_release
nvcc --version
df -h /
ls /usr/local/zed 2>/dev/null || echo "ZED SDK not installed"
dpkg -l | grep -i zed 2>/dev/null || echo "No ZED packages found"

# Download ZED SDK 5.2.2 for JetPack 6.2.2 / L4T 36.5
wget -O ~/ZED_SDK_Tegra_L4T36.5_v5.2.2.zstd.run \
  "https://download.stereolabs.com/zedsdk/5.2.2/l4t36.5/jetsons"

# Verify download
ls -lh ~/ZED_SDK_Tegra_L4T36.5_v5.2.2.zstd.run
head -3 ~/ZED_SDK_Tegra_L4T36.5_v5.2.2.zstd.run

# Check the actual download URL (revealed 5.2.2 is the correct version)
curl -sI "https://download.stereolabs.com/zedsdk/5.2/l4t36.5/jetsons" \
  | grep -i "location\|content-length"

# Install TensorRT (required by ZED SDK — was missing from minimal install)
sudo apt update
apt-cache search tensorrt | head -20

# Install full TensorRT stack
sudo apt install -y nvidia-tensorrt

# Verify TensorRT runtime library
find /usr -name "libnvinfer*" 2>/dev/null | head -10

# Install Python TensorRT bindings
sudo apt install -y python3-libnvinfer python3-libnvinfer-dev

# Verify TensorRT Python import (will fail with libnvdla_compiler error)
python3 -c "import tensorrt; print('TensorRT version:', tensorrt.__version__)"

# FIX: Extract missing libnvdla_compiler.so from older package
# (known NVIDIA packaging bug — file missing from R36.4+ packages)
wget -O - \
  https://repo.download.nvidia.com/jetson/common/pool/main/n/nvidia-l4t-dla-compiler/nvidia-l4t-dla-compiler_36.4.1-20241119120551_arm64.deb \
  | dpkg-deb --fsys-tarfile - \
  | sudo tar xv --strip-components=5 \
    --directory=/usr/lib/aarch64-linux-gnu/nvidia/ \
    ./usr/lib/aarch64-linux-gnu/nvidia/libnvdla_compiler.so

# Update dynamic linker cache
sudo ldconfig

# Verify the library is now present
ls -lh /usr/lib/aarch64-linux-gnu/nvidia/libnvdla_compiler.so

# Verify TensorRT Python now works
python3 -c "import tensorrt; print('TensorRT version:', tensorrt.__version__)"
# Expected: TensorRT version: 10.3.0

# Remove incomplete first ZED SDK installation
sudo rm -rf /usr/local/zed

# Make installer executable and run
chmod +x ~/ZED_SDK_Tegra_L4T36.5_v5.2.2.zstd.run
~/ZED_SDK_Tegra_L4T36.5_v5.2.2.zstd.run
# Answer: yes to Python API, AI models, Neural Depth, TensorRT optimization, tools
# This will take 20-40 minutes for TensorRT model compilation

# Verify ZED SDK installation
ls /usr/local/zed/

# Verify Python API
python3 -c "import pyzed.sl as sl; print('ZED Python API version:', sl.Camera().get_sdk_version())"
# Expected: 5.2.2

# Check all optimized AI models
ls /usr/local/zed/resources/

# Connect ZED 2i camera and test detection
lsusb | grep -i "stereo\|zed\|2b03"
ls /dev/video* 2>/dev/null

# Check USB speed (MUST show 3.x for ZED to work)
lsusb -v -d 2b03:f880 2>/dev/null | grep "bcdUSB"
# If shows 2.10 -> USB 2.0 cable! Need USB 3.2 Gen2 cable
# If shows 3.10 or 3.20 -> USB 3.0 confirmed, proceed

# Check USB topology to diagnose speed issues
lsusb -t
sudo dmesg | grep -i "2b03\|zed\|stereolabs" | tail -20

# Test ZED camera (run after verifying USB 3.0 connection)
# Create test script on Jetson
mkdir -p ~/zed2i
nano ~/zed2i/test_zed.py
# -> Paste test_zed.py content (see session notes)
python3 ~/zed2i/test_zed.py
```

---

## Miscellaneous Useful Commands

```bash
# Check Ubuntu version
lsb_release -a

# Check all NVIDIA-related environment variables
printenv | grep -i nvidia

# Check current display server
echo $XDG_SESSION_TYPE

# Check system resources
free -h && df -h /

# Install jtop (Jetson system monitor)
sudo pip3 install jetson-stats
jtop

# Set Jetson to maximum performance mode
sudo nvpmodel -m 0
sudo jetson_clocks

# Check GPU utilization on Jetson
sudo tegrastats --interval 500

# Find which process is using a device
fuser /dev/video0 /dev/video1 2>/dev/null

# Check USB device tree
lsusb -t

# Check detailed USB device info
lsusb -v -d <vendor>:<product> 2>/dev/null

# Check kernel messages (useful for USB/hardware debugging)
sudo dmesg | tail -30

# Check dynamic linker cache
ldconfig -p | grep <library_name>

# Find a library file on the system
find /usr -name "lib*.so*" 2>/dev/null

# Check which groups current user belongs to
id
groups <username>

# Reload group membership without logout
newgrp <groupname>

# Check systemd service status
systemctl status <service_name>

# View systemd service logs
sudo journalctl -u <service_name> --no-pager

# Reload systemd after changing service files
sudo systemctl daemon-reload

# Check binfmt registration
cat /proc/sys/fs/binfmt_misc/qemu-aarch64

# Check file architecture type
file <binary_file>

# Count lines in a file
wc -l <filename>

# Show file with line numbers
cat -A <filename>    # Shows hidden characters
head -5 <filename>   # Show first 5 lines
```

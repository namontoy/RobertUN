# RobertUN Project — Linux Terminal Command Reference
# Complete history of commands used across all development sessions
# Organized thematically for easy reference
# Last updated: April 2026

# ============================================================
# 1. SYSTEM RECONNAISSANCE
# ============================================================
# These commands were used to understand the state of each machine
# before making any configuration changes

# Check OS version and kernel
uname -a
lsb_release -a

# Check CPU information
lscpu | grep -E "Model name|CPU\(s\)|Thread|Core|Socket|MHz|Virtualization"

# Check RAM
free -h

# Check GPU and NVIDIA driver
nvidia-smi
nvidia-smi | head -20
lspci | grep -i nvidia

# Check display server (X11 vs Wayland)
echo $XDG_SESSION_TYPE
echo $WAYLAND_DISPLAY
loginctl show-session $(loginctl | grep $(whoami) | awk '{print $1}') -p Type

# Check desktop environment
echo $DESKTOP_SESSION
echo $XDG_CURRENT_DESKTOP

# Check CUDA toolkit
nvcc --version
ls /usr/local/cuda*

# Check running services (remote desktop, display manager)
systemctl list-units --type=service | grep -iE "vnc|rdp|xrdp|nomachine|anydesk|chrome|remote"

# Check display outputs and resolutions
xrandr 2>/dev/null

# Find Isaac Sim installation
ls ~/isaacsim 2>/dev/null || ls ~/.local/share/ov/pkg 2>/dev/null || find /home -maxdepth 4 -name "isaac-sim.sh" 2>/dev/null

# Check all network interfaces
ip addr show
ip route show

# Check NetworkManager connections
nmcli connection show
nmcli device status

# Check SSH server status
sudo ss -tlnp | grep sshd
systemctl status ssh


# ============================================================
# 2. DISPLAY MANAGER — SWITCHING FROM LIGHTDM TO GDM3
# ============================================================
# IsaacUN was initially running LightDM (leftover from MATE installation)
# GNOME Shell 46 requires GDM3 for ScreenShield (screen lock) to work

# Check what display manager is running
systemctl status display-manager
cat /etc/X11/default-display-manager

# Check installed greeters
ls /usr/share/xgreeters/

# Check LightDM configuration
cat /etc/lightdm/lightdm.conf 2>/dev/null
ls /etc/lightdm/
cat /etc/lightdm/lightdm.conf.d/*.conf 2>/dev/null
ls /etc/lightdm/lightdm.conf.d/ 2>/dev/null

# Check autologin group
grep autologin /etc/group
groups namontoy | grep -o autologin

# Check GNOME screensaver settings
gsettings get org.gnome.desktop.screensaver lock-enabled
gsettings get org.gnome.desktop.screensaver lock-delay
gsettings get org.gnome.desktop.session idle-delay

# Install replacement greeter BEFORE removing Arctica
sudo apt install lightdm-gtk-greeter

# Check MATE packages still installed
dpkg -l | grep -i mate | grep "^ii"

# Check what's running from MATE
ps aux | grep -iE "mate|screensaver|power" | grep -v grep

# Preview MATE removal (dry run - nothing is changed)
sudo apt-get remove --dry-run ubuntu-mate-desktop ubuntu-mate-core 2>&1

# Remove MATE metapackages
sudo apt remove ubuntu-mate-desktop ubuntu-mate-core

# Remove all orphaned MATE packages
sudo apt autoremove

# Check remaining MATE autostart entries
ls /etc/xdg/autostart/ | grep -iE "mate|ayatana|blueman|magnus|picom|arctica"

# Remove leftover MATE autostart entries
sudo rm /etc/xdg/autostart/ayatana-indicator-a11y.desktop \
        /etc/xdg/autostart/ayatana-indicator-application.desktop \
        /etc/xdg/autostart/ayatana-indicator-datetime.desktop \
        /etc/xdg/autostart/ayatana-indicator-keyboard.desktop \
        /etc/xdg/autostart/ayatana-indicator-messages.desktop \
        /etc/xdg/autostart/ayatana-indicator-notifications.desktop \
        /etc/xdg/autostart/ayatana-indicator-power.desktop \
        /etc/xdg/autostart/ayatana-indicator-printers.desktop \
        /etc/xdg/autostart/ayatana-indicator-session.desktop \
        /etc/xdg/autostart/ayatana-indicator-sound.desktop \
        /etc/xdg/autostart/blueman.desktop \
        /etc/xdg/autostart/magnus-autostart.desktop \
        /etc/xdg/autostart/mate-optimus.desktop \
        /etc/xdg/autostart/mate-power-manager.desktop \
        /etc/xdg/autostart/mate-screensaver.desktop \
        /etc/xdg/autostart/mate-settings-daemon.desktop \
        /etc/xdg/autostart/picom.desktop \
        /etc/xdg/autostart/polkit-mate-authentication-agent-1.desktop

# Remove leftover Arctica greeter desktop file
sudo rm /usr/share/xgreeters/arctica-greeter.desktop

# Verify greeters directory is clean
ls /usr/share/xgreeters/

# Check GDM3 installation (already installed, just needs upgrading)
sudo apt install --dry-run gdm3 2>&1 | grep -E "^Inst|^Remv|newly installed"

# Switch display manager to GDM3
sudo dpkg-reconfigure gdm3

# Configure GDM3 autologin
sudo tee /etc/gdm3/custom.conf > /dev/null << 'EOF'
[daemon]
AutomaticLoginEnable=true
AutomaticLogin=namontoy

[security]

[xdmcp]

[chooser]

[debug]
EOF

# Verify GDM3 config
cat /etc/gdm3/custom.conf

# Check GNOME Shell package integrity
sudo dpkg --verify gnome-shell

# Reinstall GNOME Shell to restore missing ScreenShield files
sudo apt install --reinstall gnome-shell gnome-shell-common

# Verify ScreenShield files are now present
find /usr/share/gnome-shell -name "*creen*" 2>/dev/null

# Purge all packages with leftover configuration files
dpkg -l | grep "^rc" | awk '{print $2}' | xargs sudo apt purge -y

# Final cleanup
sudo apt autoremove -y && sudo apt clean

# Run full system upgrade
sudo apt upgrade


# ============================================================
# 3. AUTOLOGIN AND SCREEN LOCK (IsaacUN)
# ============================================================
# Configure autologin with automatic screen lock for physical security
# Screen locks ~15 seconds after boot via autostart entry

# Create autologin group and add user
sudo groupadd -f autologin
sudo usermod -aG autologin namontoy

# Verify group membership
groups namontoy | grep autologin

# Create clean LightDM configuration (before switching to GDM3)
sudo tee /etc/lightdm/lightdm.conf > /dev/null << 'EOF'
[LightDM]
greeter-session=lightdm-gtk-greeter

[Seat:*]
user-session=ubuntu
greeter-session=lightdm-gtk-greeter
autologin-user=namontoy
autologin-user-timeout=0
allow-guest=false
EOF

# Fix GNOME keyring for autologin (add keyring unlock to PAM autologin file)
sudo tee /etc/pam.d/lightdm-autologin > /dev/null << 'EOF'
#%PAM-1.0
auth    requisite       pam_nologin.so
auth    required        pam_permit.so
-auth   optional        pam_gnome_keyring.so
@include common-account
session [success=ok ignore=ignore module_unknown=ignore default=bad] pam_selinux.so close
session required        pam_limits.so
@include common-session
session [success=ok ignore=ignore module_unknown=ignore default=bad] pam_selinux.so open
-session optional       pam_gnome_keyring.so auto_start
session required        pam_env.so readenv=1
session required        pam_env.so readenv=1 user_readenv=1 envfile=/etc/default/locale
@include common-password
EOF

# Create autostart entry to lock screen 15 seconds after login
mkdir -p ~/.config/autostart
tee ~/.config/autostart/lock-after-autologin.desktop > /dev/null << 'EOF'
[Desktop Entry]
Type=Application
Name=Lock screen after autologin
Exec=sh -c "sleep 15 && gdbus call --session --dest org.gnome.ScreenSaver --object-path /org/gnome/ScreenSaver --method org.gnome.ScreenSaver.Lock >> /tmp/lock-debug.log 2>&1"
X-GNOME-Autostart-enabled=true
Hidden=false
NoDisplay=true
Comment=Locks the screen shortly after autologin for physical security
EOF

# Create autostart entry to fix dual monitor detection race condition
tee ~/.config/autostart/restore-monitors.desktop > /dev/null << 'EOF'
[Desktop Entry]
Type=Application
Name=Restore dual monitor configuration
Exec=sh -c "sleep 8 && DISPLAY=:0 xrandr --output DP-4 --auto --right-of DP-2"
X-GNOME-Autostart-enabled=true
Hidden=false
NoDisplay=true
Comment=Triggers DP-4 detection after autologin boot timing race condition
EOF

# Verify screen lock is working
cat /tmp/lock-debug.log

# Test screen lock manually
gdbus call --session --dest org.gnome.ScreenSaver \
  --object-path /org/gnome/ScreenSaver \
  --method org.gnome.ScreenSaver.Lock

# Check session lock state
loginctl show-session $(loginctl | grep namontoy | awk '{print $1}') -p LockedHint

# Check current monitor configuration
xrandr --query | grep -E "connected|disconnected"
cat ~/.config/monitors.xml 2>/dev/null


# ============================================================
# 4. SSH CONFIGURATION
# ============================================================
# All machines use port 44252 (0xACDC) instead of default port 22
# This reduces automated bot scanning

# Check current SSH port configuration
grep -E "^Port|^#Port" /etc/ssh/sshd_config

# Change SSH port in sshd_config
sudo sed -i 's/#Port 22/Port 44252/' /etc/ssh/sshd_config

# Verify the change
grep "^Port" /etc/ssh/sshd_config

# On Ubuntu 24.04: SSH uses systemd socket activation
# Must override the socket unit, not just sshd_config
systemctl status ssh.socket
cat /lib/systemd/system/ssh.socket

# Create systemd socket override for port change
sudo mkdir -p /etc/systemd/system/ssh.socket.d
sudo tee /etc/systemd/system/ssh.socket.d/override.conf > /dev/null << 'EOF'
[Socket]
ListenStream=
ListenStream=0.0.0.0:44252
ListenStream=[::]:44252
EOF

# Reload systemd and restart SSH
sudo systemctl daemon-reload
sudo systemctl restart ssh.socket ssh.service

# Verify SSH is listening on new port
sudo ss -tlnp | grep sshd

# Test SSH connection
ssh -T git@github.com

# Connect to Jetson
ssh -p 44252 talos@192.168.1.211

# Check SSH config on Dell
cat ~/.ssh/config


# ============================================================
# 5. NETWORK CONFIGURATION — GIGABIT SWITCH
# ============================================================
# Configure static Ethernet IPs for Dell and Jetson
# Convention: Ethernet IP = 200 + WiFi IP
# Dell: WiFi=192.168.1.12, Ethernet=192.168.1.212
# Jetson: WiFi=192.168.1.11, Ethernet=192.168.1.211

# Verify target addresses are free before assigning
ping -c 2 192.168.1.211
ping -c 2 192.168.1.212

# Check current network state on both machines
ip addr show
ip route show
nmcli connection show
nmcli device status

# Configure static Ethernet IP on Dell laptop
sudo nmcli connection modify "Wired connection 1" \
  ipv4.method manual \
  ipv4.addresses 192.168.1.212/24 \
  ipv4.gateway 192.168.1.1 \
  ipv4.dns "8.8.8.8,8.8.4.4" \
  ipv4.route-metric 100 \
  connection.autoconnect yes

# Apply the change
sudo nmcli connection down "Wired connection 1"
sudo nmcli connection up "Wired connection 1"

# Configure static Ethernet IP on Jetson (run from Dell via SSH)
ssh -p 44252 talos@192.168.1.10 "sudo nmcli connection modify 'Wired connection 1' \
  ipv4.method manual \
  ipv4.addresses 192.168.1.211/24 \
  ipv4.gateway 192.168.1.1 \
  ipv4.dns '8.8.8.8,8.8.4.4' \
  ipv4.route-metric 100 \
  connection.autoconnect yes && \
  sudo nmcli connection down 'Wired connection 1' && \
  sudo nmcli connection up 'Wired connection 1'"

# Verify routing table — Ethernet should be metric 100, WiFi metric 600
ip route show

# Update SSH config to use Jetson Ethernet address
sed -i 's/HostName 192.168.1.11/HostName 192.168.1.211/' ~/.ssh/config

# Update jsync alias to use Jetson Ethernet address
sed -i 's/talos@192.168.1.11/talos@192.168.1.211/' ~/.bashrc

# Update Cyclone DDS on Dell to use Ethernet interface
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
        <Peer address="192.168.1.211"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

# Update Cyclone DDS on Jetson to use Ethernet interface
ssh jetson "cat > ~/.ros/cyclone_dds.xml << 'EOF'
<?xml version=\"1.0\" encoding=\"UTF-8\" ?>
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name=\"enP8p1s0\" multicast=\"false\"/>
      </Interfaces>
    </General>
    <Discovery>
      <Peers>
        <Peer address=\"192.168.1.212\"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF"

# Update Dev Container Cyclone DDS config
cat > ~/ros2_ws/.devcontainer/cyclone_dds.xml << 'EOF'
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
        <Peer address="192.168.1.211"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

# Apply bashrc changes
source ~/.bashrc

# Verify all configurations
cat ~/.ssh/config
cat ~/.ros/cyclone_dds.xml
grep jsync ~/.bashrc


# ============================================================
# 6. CUDA TOOLKIT INSTALLATION (IsaacUN)
# ============================================================
# IsaacUN had nvidia-smi reporting CUDA 12.8 but nvcc was missing
# The driver includes the runtime; the toolkit adds the compiler

# Add NVIDIA CUDA repository for Ubuntu 24.04
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update

# Install CUDA toolkit without touching the existing driver
sudo apt install cuda-toolkit-12-8

# Add CUDA to PATH (add to ~/.bashrc)
# export PATH=/usr/local/cuda/bin:$PATH
# export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH

# Apply PATH changes
source ~/.bashrc

# Verify nvcc is now available
nvcc --version

# Verify CUDA symlinks
ls /usr/local/cuda*


# ============================================================
# 7. ROS 2 INSTALLATION
# ============================================================

# --- ROS 2 Humble on Dell laptop and Jetson (Ubuntu 22.04) ---

# Add ROS 2 GPG key
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 Humble repository (jammy = Ubuntu 22.04)
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu jammy main" | \
sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update and install ROS 2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop

# Install build tools
sudo apt install python3-colcon-common-extensions python3-rosdep python3-argcomplete -y

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS 2 (add to ~/.bashrc)
# source /opt/ros/humble/setup.bash
# export ROS_DOMAIN_ID=0

# --- ROS 2 Jazzy on IsaacUN (Ubuntu 24.04) ---

# Add ROS 2 Jazzy repository (noble = Ubuntu 24.04)
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu noble main" | \
sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-jazzy-desktop

# Install build tools
sudo apt install python3-colcon-common-extensions python3-rosdep python3-argcomplete -y
sudo rosdep init
rosdep update

# Verify ROS 2 environment
printenv | grep -E "^ROS_DISTRO|^AMENT|^COLCON"

# Test ROS 2 communication (run talker on one machine, listener on another)
ros2 run demo_nodes_cpp talker
ros2 topic echo /chatter --once
ros2 topic list


# ============================================================
# 8. CONDA / PYTHON ENVIRONMENTS
# ============================================================

# --- Install Miniconda3 ---
# Download installer
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh \
  -O ~/miniconda_installer.sh

# Run installer (accept license, use default path ~/miniconda3, run conda init = yes)
bash ~/miniconda_installer.sh

# Apply conda initialization to current session
source ~/.bashrc

# Disable auto-activation of base environment
conda config --set auto_activate_base false

# Verify setting
conda config --show auto_activate_base

# List environments
conda env list

# --- ros2 environment (Dell laptop — Python 3.10, ROS 2 Humble) ---
conda create -n ros2 python=3.10 -y
conda activate ros2

# Install robotics Python stack
pip install numpy opencv-python pyserial transforms3d scipy matplotlib pytest pyyaml
pip install catkin-pkg lark empy==3.3.4

# Create activation script (auto-sources ROS 2 on conda activate ros2)
mkdir -p ~/miniconda3/envs/ros2/etc/conda/activate.d
tee ~/miniconda3/envs/ros2/etc/conda/activate.d/ros2.sh > /dev/null << 'EOF'
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
EOF

# Test the activation
conda deactivate
conda activate ros2
printenv | grep -E "^ROS_DISTRO|^ROS_DOMAIN_ID|^RMW_IMPLEMENTATION"

# Test rclpy works
python3 -c "
import rclpy
rclpy.init()
node = rclpy.create_node('test_node')
print(f'Node created: {node.get_name()}')
node.destroy_node()
rclpy.shutdown()
print('ROS 2 Python stack working correctly')
"

# --- ros2 environment (IsaacUN — Python 3.12, ROS 2 Jazzy) ---
conda create -n ros2 python=3.12 -y
conda activate ros2

# Install robotics Python stack
pip install numpy opencv-python pyserial transforms3d scipy matplotlib pytest pyyaml
pip install catkin-pkg lark empy==3.3.4

# Create activation script for Jazzy
mkdir -p ~/miniconda3/envs/ros2/etc/conda/activate.d
tee ~/miniconda3/envs/ros2/etc/conda/activate.d/ros2.sh > /dev/null << 'EOF'
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
EOF

# --- ml environment (Dell laptop — Python 3.11, PyTorch + CUDA) ---
conda create -n ml python=3.11 -y
conda activate ml

# Install PyTorch with CUDA 12.6 support
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu126

# Install scientific Python stack
pip install numpy scipy matplotlib pandas jupyter scikit-learn pyyaml

# Create activation script with ML-specific environment variables
mkdir -p ~/miniconda3/envs/ml/etc/conda/activate.d
tee ~/miniconda3/envs/ml/etc/conda/activate.d/ml.sh > /dev/null << 'EOF'
# PyTorch model cache location
export TORCH_HOME=~/.cache/torch
# Reduce GPU memory fragmentation on smaller GPUs
export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True
# Use all CPU cores for non-GPU operations
export OMP_NUM_THREADS=$(nproc)
EOF

# Verify GPU is accessible from PyTorch
python3 -c "
import torch
print(f'PyTorch version: {torch.__version__}')
print(f'CUDA available: {torch.cuda.is_available()}')
if torch.cuda.is_available():
    print(f'GPU: {torch.cuda.get_device_name(0)}')
    print(f'CUDA version: {torch.version.cuda}')
    x = torch.tensor([1.0, 2.0, 3.0]).cuda()
    print(f'GPU tensor test: {x * 2}')
"


# ============================================================
# 9. TMUX CONFIGURATION (IsaacUN)
# ============================================================
# tmux is a terminal multiplexer — sessions survive network disconnects
# Essential for remote work via NoMachine

# Install tmux
sudo apt install tmux

# Create tmux configuration file
tee ~/.tmux.conf > /dev/null << 'EOF'
# Prefix keys
set -g prefix C-b
bind C-a send-prefix

# Start numbering at 1
set -g base-index 1
setw -g pane-base-index 1
set -g renumber-windows on

# Large scrollback buffer (important for ROS 2 logs)
set -g history-limit 50000

# Reduce escape-key delay
set -g escape-time 10

# True color support
set -g default-terminal "tmux-256color"
set -ga terminal-overrides ",xterm-256color:Tc"

# Mouse support
set -g mouse on

# Intuitive split commands
bind | split-window -h -c "#{pane_current_path}"
bind - split-window -v -c "#{pane_current_path}"
bind c new-window -c "#{pane_current_path}"

# Vim-style pane navigation
bind h select-pane -L
bind j select-pane -D
bind k select-pane -U
bind l select-pane -R

# Reload config
bind r source-file ~/.tmux.conf \; display "Config reloaded"

# Status bar
set -g status-position bottom
set -g status-interval 5
set -g status-left-length 40
set -g status-right-length 60
set -g status-left "#[fg=colour214,bold] #S #[fg=default,nobold]│ "
set -g status-right "#[fg=colour109] %H:%M #[fg=colour245]│ #[fg=colour214]#H "
setw -g window-status-format " #I:#W "
setw -g window-status-current-format " #I:#W "
setw -g window-status-current-style "fg=colour214,bold"
EOF

# Common tmux commands
tmux new-session -s ros2dev          # Create named session
tmux ls                              # List sessions
tmux attach -t ros2dev               # Attach to session
tmux a                               # Attach to most recent session

# Send command to session without attaching (headless)
tmux new-session -d -s ros2test
tmux send-keys -t ros2test "ros2 run demo_nodes_cpp talker" Enter
tmux kill-session -t ros2test


# ============================================================
# 10. USER RENAME (namontoy → talos on IsaacUN)
# ============================================================
# Cannot rename a user while logged in as that user
# Solution: create temp admin, disable autologin, reboot, rename from TTY3

# Create temporary admin user
sudo useradd -m -s /bin/bash tempadmin
sudo passwd tempadmin
sudo usermod -aG sudo tempadmin

# Disable autologin temporarily
sudo sed -i 's/AutomaticLoginEnable=true/AutomaticLoginEnable=false/' /etc/gdm3/custom.conf

# Reboot, then from TTY3 (Ctrl+Alt+F3) logged in as tempadmin:
sudo usermod -l talos -d /home/talos -m namontoy
sudo groupmod -n talos namontoy
sudo usermod -c "talos" talos

# Update GDM autologin
sudo nano /etc/gdm3/custom.conf
# Change AutomaticLogin=namontoy to AutomaticLogin=talos
# Change AutomaticLoginEnable=false to AutomaticLoginEnable=true

# Update LightDM config for consistency
sudo sed -i 's/autologin-user=namontoy/autologin-user=talos/' /etc/lightdm/lightdm.conf

# Verify rename
id talos
ls /home/
grep talos /etc/passwd
cat /etc/gdm3/custom.conf | grep AutomaticLogin

# Change password for talos
passwd

# Remove temporary admin user
sudo userdel -r tempadmin
id tempadmin 2>/dev/null || echo "tempadmin successfully removed"


# ============================================================
# 11. GIT AND GITHUB CONFIGURATION
# ============================================================

# Check git version
git --version

# Configure git identity
git config --global user.name "namontoy"
git config --global user.email "namontoy@unal.edu.co"

# Configure convenient defaults
git config --global push.default current
git config --global core.editor nano

# Verify configuration
git config --global user.name
git config --global user.email

# Check existing SSH keys
ls ~/.ssh/

# Get public key to add to GitHub
cat ~/.ssh/id_ed25519.pub

# Test GitHub SSH connection
ssh -T git@github.com

# Clone repository
mkdir -p ~/github
cd ~/github
git clone git@github.com:namontoy/RobertUN.git

# Common git workflow
cd ~/github/RobertUN
git pull origin main
git add docs/environment/PROJECT_CONTEXT.md
git commit -m "Description of changes"
git push origin main

# Check repository structure
ls -la ~/github/RobertUN/


# ============================================================
# 12. NoMachine REMOTE DESKTOP (IsaacUN)
# ============================================================

# Check NoMachine version and status
/etc/NX/nxserver --version
/etc/NX/nxserver --status

# Check NoMachine configuration
grep -v "^#" /usr/NX/etc/server.cfg | grep -v "^$"
grep -v "^#" /usr/NX/etc/node.cfg | grep -v "^$"

# Check NoMachine running processes
ps aux | grep -iE "nx|nxagent" | grep -v grep

# Check NoMachine logs
sudo find /usr/NX/var -name "*.log" 2>/dev/null
sudo tail -50 /usr/NX/var/log/server.log

# Restart NoMachine server
sudo /etc/NX/nxserver --restart

# NoMachine client .nxs file location (Ubuntu client)
# ~/.nomachine/ or ~/Documents/NoMachine/
find ~/ -name "*.nxs" 2>/dev/null

# Fix monitor selection in .nxs file (client-side fix)
# Edit: <option key="View a specific monitor among available monitors" value="1" />


# ============================================================
# 13. ZED 2i CAMERA SETUP AND VERIFICATION
# ============================================================

# Check if ZED camera is detected by Linux
lsusb | grep 2b03

# Check USB connection speed (must show 3.00 or higher for USB 3.x)
lsusb -v -d 2b03:f880 2>/dev/null | grep bcdUSB

# Check ZED SDK version
cat /usr/local/zed/settings/ZED_SDK_VERSION

# List detected cameras
/usr/local/zed/tools/ZED_Explorer -l

# Run full hardware/software diagnostic
/usr/local/zed/tools/ZED_Diagnostic -c

# Pre-optimize all neural depth models (takes 30-60 min on Jetson Orin Nano)
/usr/local/zed/tools/ZED_Diagnostic -nrlo_all

# Run ZED camera test script
cd ~/zed2i
python3 test_zed.py

# Check ZED SDK verbose logging
export ZED_SDK_VERBOSE=1
python3 test_zed.py

# Check orphaned ZED processes before launching
ps aux | grep component_container

# Kill orphaned processes (important before relaunching)
pkill -f component_container_isolated
pkill -f robot_state_publisher

# Check AI model files
ls -la /usr/local/zed/resources/*.model
ls -la /usr/local/zed/resources/*.engine

# Check calibration files
ls /usr/local/zed/settings/


# ============================================================
# 14. SYSTEM DIAGNOSTICS AND MONITORING
# ============================================================

# Monitor GPU usage and temperature (Jetson)
sudo jtop

# Raw Jetson stats
sudo tegrastats --interval 500

# Set maximum performance mode on Jetson
sudo nvpmodel -m 0        # MAXN mode
sudo jetson_clocks         # Lock all clocks at maximum

# Check GPU usage (all machines)
nvidia-smi

# Check GNOME Shell journal logs
journalctl --user -b | grep -iE "gnome-shell|screenshield|fallback" | grep -v "Window manager warning" | head -30

# Check system journal for errors
journalctl --user -b | grep -iE "JS ERROR|JS LOG|Could not|Failed to" | head -30

# Check D-Bus services
dbus-send --print-reply --dest=org.freedesktop.DBus /org/freedesktop/DBus \
  org.freedesktop.DBus.ListNames | grep -iE "screen|lock|saver"

# Check session information
loginctl list-sessions
loginctl show-session $(loginctl | grep talos | awk '{print $1}') \
  -p Id -p State -p LockedHint -p Type

# Check PAM configuration for display manager
cat /etc/pam.d/lightdm
cat /etc/pam.d/lightdm-autologin

# Check GNOME Shell ScreenShield files
find /usr/share/gnome-shell -name "*creen*" 2>/dev/null

# Check running processes
ps aux | grep -iE "mate|screensaver|power" | grep -v grep

# Check systemd user services
systemctl --user list-units | grep -iE "screen|lock|idle|power"

# Disable Apport crash reporter (reduces noise on dev workstations)
sudo systemctl disable apport
sudo systemctl stop apport

# Check apt package integrity
sudo apt-get check
sudo dpkg --verify gnome-shell


# ============================================================
# 15. USEFUL ONE-LINERS AND REFERENCE COMMANDS
# ============================================================

# --- Network ---
# Check what's on your network
nmap -sn 192.168.1.0/24 2>/dev/null | grep -E "report|MAC"

# Check if a specific address is in use
ping -c 2 192.168.1.211

# Check open ports
sudo ss -tlnp

# --- Package Management ---
# Search for a package
apt-cache search <package-name>

# Check if package is installed
dpkg -l <package-name>

# Find which package owns a file
dpkg -S /path/to/file

# Check package dependencies
apt-cache rdepends <package-name>

# Preview package removal without doing it
sudo apt-get remove --dry-run <package-name>

# --- File Operations ---
# Find a file by name
find ~/ -name "filename" 2>/dev/null

# Secure copy to Jetson
jscp <file> talos@192.168.1.211:/path/on/jetson

# Sync workspace to Jetson
jsync

# --- Process Management ---
# Find process by name
ps aux | grep <process-name> | grep -v grep

# Find process using a specific port
sudo ss -tlnp | grep <port>

# Kill process by name
pkill -f <process-name>

# --- ROS 2 ---
# Source ROS 2 environment (if not using conda activation script)
source /opt/ros/humble/setup.bash    # Dell/Jetson
source /opt/ros/jazzy/setup.bash     # IsaacUN

# Check ROS 2 environment
printenv | grep -E "^ROS|^AMENT|^COLCON"

# List all ROS 2 nodes and topics
ros2 node list
ros2 topic list

# Monitor topic frequency
ros2 topic hz /topic_name

# Echo topic messages
ros2 topic echo /topic_name --once

# --- CAN Bus (Jetson) ---
# Configure pinmux (required after every reboot)
sudo busybox devmem 0x0c303018 w 0xc458
sudo busybox devmem 0x0c303010 w 0xc400

# Load CAN modules
sudo modprobe can && sudo modprobe can_raw && sudo modprobe mttcan

# Bring up CAN interface at 125 kbps
sudo ip link set can0 type can bitrate 125000
sudo ip link set can0 up

# Monitor CAN traffic
candump can0

# Send test CAN frame
cansend can0 123#DEADBEEF

# Verify CAN hardware is active
cat /proc/device-tree/bus@0/mttcan@c310000/status

# --- Conda ---
# List environments
conda env list

# Create environment
conda create -n <name> python=3.10 -y

# Activate/deactivate
conda activate <name>
conda deactivate

# Remove environment
conda env remove -n <name>

# Export environment to file
conda env export > environment.yml

# --- tmux ---
# New session
tmux new-session -s <name>

# List sessions
tmux ls

# Attach to session
tmux attach -t <name>

# Detach from session
# Ctrl+B then d

# Split pane vertically
# Ctrl+B then |

# Split pane horizontally
# Ctrl+B then -

# Navigate between panes
# Ctrl+B then h/j/k/l

# Send command to session without attaching
tmux send-keys -t <session> "command" Enter

# Session Command History — May 25, 2026
# Jetson Xavier NX — Initial Setup, Storage, LLM Installation and Testing
#
# This document is a thematic record of all terminal commands executed
# during the Xavier NX bringup sessions. Commands are grouped by topic
# rather than chronological order. Machine context is indicated for each section.
# All Xavier commands were executed via SSH from the Dell laptop unless noted.
#
# Machines:
#   [XAVIER]    talos@xavier   — Jetson Xavier NX 8GB (192.168.1.210)
#   [DELL]      talos@deadelus — Dell laptop host (192.168.1.212)
#
# Key decisions made in these sessions:
#   - Xavier NX role: experimental LLM inference node (natural language rover commands)
#   - JetPack version: 5.1.6 (last supported version for Xavier NX, Ubuntu 20.04)
#   - Storage: Patriot Burst Elite 120GB SATA SSD via M.2-to-SATA adapter (ASM1166)
#   - Power: 5V injected via GPIO header pin 2 to SATA power pins
#   - LLM engine: llama.cpp (CUDA build, sm_72 Volta architecture)
#   - Working models: Qwen2.5-3B, Qwen2.5-Coder-3B, Phi-3.5-mini, Gemma-3-4B
#   - VMM ceiling: ~2.5GB — 7B models cannot be loaded regardless of available RAM

# =============================================================================
# 1. HOST PREPARATION — DELL LAPTOP
# Prepare the Dell laptop to flash the Xavier NX via SDK Manager Docker container.
# SDK Manager for JetPack 5 requires Ubuntu 20.04 host — Docker solves this on
# the Dell which runs Ubuntu 22.04.
# =============================================================================

# [DELL] Verify Docker is installed and working
docker --version
docker run --rm hello-world

# [DELL] Download SDK Manager Docker image from NVIDIA developer portal
# URL: https://developer.nvidia.com/nvidia-sdk-manager
# Download: sdkmanager-2.4.0.13236-Ubuntu_20.04_docker.tar.gz
# Load the downloaded image into Docker
docker load -i ~/Downloads/sdkmanager-2.4.0.13236-Ubuntu_20.04_docker.tar.gz

# [DELL] Tag the image for convenient reference
docker tag sdkmanager:2.4.0.13236-Ubuntu_20.04 sdkmanager:latest

# [DELL] Verify both tags point to the same image hash
docker images | grep sdkmanager


# =============================================================================
# 2. XAVIER NX — FORCE RECOVERY MODE
# The Xavier NX Developer Kit uses jumper pins on header J14 for recovery mode.
# Unlike the Orin Nano (which has physical buttons), recovery is triggered by
# shorting pins 9 (FC REC) and 10 (GND) on J14, located under the module edge.
# The board enters recovery mode automatically when power is applied with the jumper.
# =============================================================================

# [XAVIER HARDWARE] Place jumper on J14 pins 9-10 (FC REC + GND)
# Connect USB micro-B cable from Xavier NX to Dell BEFORE applying power
# Apply barrel power connector — board enters recovery mode automatically
# Do NOT connect the jumper after power-on — it must be in place at boot time

# [DELL] Verify the Xavier NX is detected in recovery mode
# Expected output: Bus XXX Device XXX: ID 0955:7e19 NVIDIA Corp. APX
lsusb | grep -i nvidia


# =============================================================================
# 3. SDK MANAGER — DOCKER CONTAINER SETUP
# Launch SDK Manager inside Ubuntu 20.04 Docker container with USB passthrough.
# The EXT4 filesystem requirement is satisfied by mounting ~/nvidia from the Dell host.
# Critical missing packages (lz4, libxml2-utils) must be installed BEFORE launching
# sdkmanager --cli, otherwise the flash will fail at ~40-70% with cryptic errors.
# =============================================================================

# [DELL] Create working directory on EXT4 filesystem (fixes "not EXT4" warning)
mkdir -p ~/nvidia/sdkm_downloads

# [DELL] Launch SDK Manager container with USB passthrough and EXT4 volume mount
docker run -it --privileged \
  -v /dev/bus/usb:/dev/bus/usb/ \
  -v /dev:/dev \
  -v /media/$USER:/media/nvidia:slave \
  -v ~/nvidia:/home/nvidia \
  --network host \
  --name xavier_flash \
  --entrypoint /bin/bash \
  sdkmanager:latest

# [CONTAINER] Install missing dependencies BEFORE launching sdkmanager
# lz4: required for filesystem image compression during flash
# libxml2-utils: required for XML processing during partition layout
# device-tree-compiler, cpio, qemu-user-static, binfmt-support: ARM cross-compilation
sudo apt-get update && sudo apt-get install -y lz4 libxml2-utils \
  device-tree-compiler cpio qemu-user-static binfmt-support

# [CONTAINER] Launch SDK Manager CLI
sdkmanager --cli

# SDK Manager selections:
#   Product: Jetson
#   Hardware: Jetson Xavier NX Developer Kit
#   Version: JetPack 5.1.6
#   OEM config: Pre-config (unattended first boot)
#   Username: talos / Hostname: xavier
#   Flash: Yes

# CRITICAL — JUMPER TIMING RULE:
# Leave jumper on J14 pins 9-10 for the ENTIRE "Flash Jetson Linux" phase.
# Remove jumper ONLY when SDK Manager shows "Flash Jetson Linux: Success"
# and transitions to "DateTime Target Setup".
# Removing early causes boot failure. Leaving in too long causes SSH timeout.

# [DELL] If container exits and needs to be restarted (remove old container first)
docker rm xavier_flash

# [DELL] Check SDK Manager flash log when debugging failures
tail -80 ~/nvidia/.nvsdkm/componentLogs/JetPack_5.1.6_Linux/NV_L4T_FLASH_JETSON_LINUX_COMP.log


# =============================================================================
# 4. INITIAL XAVIER CONFIGURATION
# Post-flash first-boot configuration: hostname, static IP, SSH key, aliases.
# Xavier NX default hostname after JetPack flash is "ubuntu" — change immediately.
# Static IP follows the RobertUN convention: .210 for xavier, .211 for orion, .212 for dell.
# =============================================================================

# [XAVIER] Change hostname from "ubuntu" to "xavier"
sudo hostnamectl set-hostname xavier
sudo sed -i 's/ubuntu/xavier/g' /etc/hosts

# [XAVIER] Verify hostname change
hostnamectl

# [XAVIER] Configure static IP via NetworkManager (interface: eth0)
sudo nmcli con mod "Wired connection 1" \
  ipv4.addresses 192.168.1.210/24 \
  ipv4.gateway 192.168.1.1 \
  ipv4.dns "8.8.8.8,8.8.4.4" \
  ipv4.method manual

# [XAVIER] Apply the new IP (this will drop the SSH session)
sudo nmcli con up "Wired connection 1"

# [XAVIER] Reconnect with new static IP and verify
ip addr show eth0

# [DELL] Add xavier to /etc/hosts for name resolution
echo "192.168.1.210  xavier" | sudo tee -a /etc/hosts

# [DELL] Add xavier SSH config entry (port 22 initially, changed to 44252 later)
cat >> ~/.ssh/config << 'EOF'

Host orion
    HostName 192.168.1.211
    User talos
    Port 44252
    IdentityFile ~/.ssh/id_ed25519

Host xavier
    HostName 192.168.1.210
    User talos
    Port 22
    IdentityFile ~/.ssh/id_ed25519
EOF

# [DELL] Add bash alias for quick SSH connection
echo "alias xavier='ssh xavier'" >> ~/.bashrc
source ~/.bashrc

# [DELL] Copy SSH public key to xavier (one-time password entry)
ssh-copy-id -i ~/.ssh/id_ed25519.pub xavier

# [XAVIER] Change SSH port to 44252 for consistency with orion
sudo sed -i 's/#Port 22/Port 44252/' /etc/ssh/sshd_config
sudo systemctl restart ssh

# [DELL] Update SSH config to use new port
sed -i '/Host xavier/{n;n;n;s/Port 22/Port 44252/}' ~/.ssh/config

# [DELL] Test passwordless SSH connection with new port
ssh xavier


# =============================================================================
# 5. SATA SSD — STORAGE SETUP
# The Xavier NX M.2 Key M slot supports PCIe but NOT SATA natively.
# However, an M.2-to-SATA adapter with ASM1166 bridge chip works via PCIe-to-SATA.
# The 2.5" SATA SSD requires 5V which the M.2 slot cannot supply.
# Solution: inject 5V from GPIO header pin 2 (red wire from ATX connector).
#
# HARDWARE WIRING:
#   ATX yellow (12V) — LEAVE DISCONNECTED
#   ATX red (5V)     → GPIO header pin 2  → SATA power pin 7 (5V)
#   ATX black (GND)  → GPIO header pin 6  → SATA power pin 4 (GND)
#   ATX black (GND)  → GPIO header pin 14 → SATA power pin 9 (GND)
#   M.2 adapter      → provides 3.3V on SATA pins 1-3 automatically
#
# IMPORTANT: Connect wiring with board POWERED OFF. GPIO header 5V is always live.
# =============================================================================

# [XAVIER] After connecting SSD power, verify SATA controller is detected
lspci
# Expected: 0005:01:00.0 SATA controller: ASMedia Technology Inc. Device 1166

# [XAVIER] Verify SSD is detected as block device
lsblk -o NAME,SIZE,TYPE,FSTYPE,MOUNTPOINT,MODEL
# Expected: sda  111.8G  disk  Patriot_Burst_Elite_120GB

# [XAVIER] Check SATA link status in kernel log
sudo dmesg | grep -i "ata\|sata\|sda\|asmedia" | tail -40

# [XAVIER] Create GPT partition table and single EXT4 partition
sudo parted /dev/sda --script mklabel gpt mkpart primary ext4 0% 100%

# [XAVIER] Verify partition was created
sudo fdisk -l /dev/sda

# [XAVIER] Format as EXT4 with label "data"
sudo mkfs.ext4 -L data /dev/sda1

# [XAVIER] Create mount point and mount the SSD
sudo mkdir -p /data
sudo mount /dev/sda1 /data

# [XAVIER] Add to fstab for automatic mount on boot
# nofail: boot normally even if SSD is not detected (e.g. power wire disconnected)
echo "LABEL=data /data ext4 defaults,nofail 0 2" | sudo tee -a /etc/fstab

# [XAVIER] Verify mount and available space
df -h /data
mount | grep sda1

# [XAVIER] Set correct ownership so talos user can write without sudo
sudo chown talos:talos /data

# [XAVIER] Benchmark SSD performance
sudo hdparm -Tt /dev/sda1
# Expected: ~525 MB/s buffered read (full SATA III)

dd if=/dev/zero of=/data/testfile bs=1M count=1024 conv=fdatasync status=progress
# Expected: ~292 MB/s write (real flushed performance through ASM1166 bridge)

dd if=/data/testfile of=/dev/null bs=1M status=progress
# Note: this reads from page cache (~6 GB/s), not real disk speed — use hdparm for real read

# [XAVIER] Clean up test file after benchmarking
rm /data/testfile

# [XAVIER] Create directory structure on SSD
mkdir -p /data/models
mkdir -p /data/github

# [XAVIER] Create symlink so ~/github points to SSD (saves eMMC space)
ln -s /data/github ~/github


# =============================================================================
# 6. SYSTEM OPTIMIZATION — HEADLESS + PERFORMANCE MODE
# Xavier NX boots to graphical desktop by default — wasteful for a headless node.
# Disabling the desktop frees ~300MB RAM at boot. Power mode 8 (MODE_20W_6CORE)
# is the correct mode for AI inference: max GPU clock, max EMC bandwidth, all 6 cores.
# jetson_clocks must be enabled as a systemd service — it does not survive reboot alone.
# =============================================================================

# [XAVIER] Update system packages (safe — no NVIDIA/L4T packages in upgrade list)
sudo apt update
sudo apt upgrade -y
sudo apt autoremove -y

# [XAVIER] Switch to headless boot (does NOT remove desktop packages)
# This is reversible: sudo systemctl set-default graphical.target
sudo systemctl set-default multi-user.target

# [XAVIER] Apply headless mode immediately without rebooting
sudo systemctl isolate multi-user.target
sudo systemctl stop gdm3
sudo systemctl disable gdm3

# [XAVIER] Reboot to verify headless boot works correctly
sudo reboot

# [XAVIER] After reboot: verify RAM savings (expect ~300MB less used vs desktop)
free -h

# [XAVIER] Set power mode to MODE_20W_6CORE (mode 8) — best for LLM inference
# All 6 Carmel CPU cores, GPU @ 1100 MHz, EMC @ 1866 MHz (~59.7 GB/s bandwidth)
sudo nvpmodel -m 8
sudo nvpmodel -q

# [XAVIER] Create systemd service to lock clocks at maximum on every boot
sudo tee /etc/systemd/system/jetson_clocks.service << 'EOF'
[Unit]
Description=Maximize Jetson clocks for current NVPModel
After=nvpmodel.service
[Service]
Type=oneshot
ExecStart=/usr/bin/jetson_clocks
RemainAfterExit=yes
[Install]
WantedBy=multi-user.target
EOF

sudo systemctl enable --now jetson_clocks.service
sudo systemctl status jetson_clocks.service
# Expected: active (exited) with status=0/SUCCESS


# =============================================================================
# 7. SERVICE CLEANUP
# Disable unnecessary services that waste RAM and CPU on a headless AI node.
# Services are disabled but NOT removed — can be re-enabled if needed.
# DO NOT disable any nvidia/nv-* services — they are part of the BSP.
# =============================================================================

# [XAVIER] Disable services not needed on a headless wired-only AI node
sudo systemctl disable --now \
  bluetooth.service \
  ModemManager.service \
  avahi-daemon.service \
  rpcbind.service \
  rtkit-daemon.service \
  kerneloops.service \
  apport.service \
  openvpn.service \
  wpa_supplicant.service

# [XAVIER] Disable socket activators that would restart disabled services on demand
sudo systemctl disable --now avahi-daemon.socket rpcbind.socket

# [XAVIER] Disable audio daemon (no audio hardware on headless node)
systemctl --user disable --now pulseaudio.service pulseaudio.socket

# [XAVIER] Verify memory after service cleanup
free -h


# =============================================================================
# 8. SWAP — SSD-BACKED SWAP FILE
# The Xavier NX default swap uses zram (compressed RAM) — 4 × 854MB partitions.
# This is insufficient for LLM loading: the NvMap VMM allocator needs real backing
# store to map large contiguous GPU memory regions.
# Adding 8GB swap on the Patriot SSD resolves NvMap "error 12" for models up to ~2.5GB.
# =============================================================================

# [XAVIER] Create 8GB swap file on the SSD
sudo fallocate -l 8G /data/swapfile
sudo chmod 600 /data/swapfile
sudo mkswap /data/swapfile
sudo swapon /data/swapfile

# [XAVIER] Make swap permanent across reboots
echo '/data/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

# [XAVIER] Verify swap configuration (should show zram × 4 + swapfile 8G)
swapon --show


# =============================================================================
# 9. GPU MEMORY CLEANUP SCRIPT
# NvMap VMM on Jetson Volta does not always fully release GPU memory when a
# process exits. After a failed model load, subsequent allocations fail even
# with plenty of free RAM. This script resets GPU state without a full reboot.
#
# IMPORTANT: rmmod/modprobe does not fully restore boot-time VMM configuration.
# If NvMap errors persist after running this script, a full reboot is required.
# =============================================================================

# [XAVIER] Create GPU cleanup script
cat > ~/cleanup_gpu.sh << 'EOF'
#!/bin/bash
echo "=== GPU Memory Cleanup ==="

echo "[1/4] Killing processes holding GPU device handles..."
sudo fuser -k /dev/nvhost-gpu 2>/dev/null
sudo fuser -k /dev/nvmap 2>/dev/null
echo "      Done."

echo "[2/4] Dropping Linux page cache..."
sudo sync && echo 3 | sudo tee /proc/sys/vm/drop_caches > /dev/null
echo "      Done."

echo "[3/4] Reloading GPU kernel module..."
sudo rmmod nvgpu && sudo modprobe nvgpu
echo "      Done."

echo "[4/4] Memory status after cleanup:"
free -h

echo ""
echo "=== Cleanup complete ==="
echo "NOTE: If NvMap VMM errors persist after this script,"
echo "      a full reboot is required: sudo reboot"
EOF
chmod +x ~/cleanup_gpu.sh

# [XAVIER] Run GPU cleanup (use after any crashed model load or NvMap error)
~/cleanup_gpu.sh


# =============================================================================
# 10. LLAMA.CPP — BUILD WITH CUDA SUPPORT
# llama.cpp must be built from source with CUDA enabled for GPU acceleration.
# Xavier NX is Volta architecture, compute capability 7.2 (sm_72).
# CRITICAL: use -DGGML_CUDA=ON not the deprecated LLAMA_CUBLAS flag.
# CRITICAL: set CMAKE_CUDA_ARCHITECTURES=72 explicitly — autodetection can fail.
# Ubuntu 20.04 ships CMake 3.16 which is too old — upgrade via pip first.
# Build takes 15-25 minutes on Xavier NX (all 6 cores).
# =============================================================================

# [XAVIER] Install build dependencies
sudo apt-get install -y build-essential git cmake ccache \
  libcurl4-openssl-dev pkg-config python3-pip

# [XAVIER] Add CUDA to PATH (required for nvcc and library linking)
echo 'export PATH=/usr/local/cuda-11.4/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda-11.4/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc

# [XAVIER] Verify CUDA compiler is accessible
nvcc --version
# Expected: release 11.4, V11.4.315

# [XAVIER] Upgrade CMake (Ubuntu 20.04 ships 3.16, llama.cpp requires ≥3.18)
pip3 install --user "cmake>=3.27"
echo 'export PATH=$HOME/.local/bin:$PATH' >> ~/.bashrc
source ~/.bashrc
cmake --version
# Expected: 4.x.x (well above minimum)

# [XAVIER] Clone llama.cpp to the SSD (saves eMMC space)
cd ~/github
git clone https://github.com/ggml-org/llama.cpp.git
cd llama.cpp

# [XAVIER] Configure with CUDA support for Volta sm_72
cmake -B build \
  -DGGML_CUDA=ON \
  -DCMAKE_CUDA_ARCHITECTURES=72 \
  -DCMAKE_BUILD_TYPE=Release \
  -DLLAMA_CURL=ON \
  -DGGML_NATIVE=ON

# [XAVIER] Build with all 6 cores (15-25 minutes)
cmake --build build --config Release -j6

# [XAVIER] Verify key binaries were built
ls -la ~/github/llama.cpp/build/bin/ | grep -E "llama-cli|llama-server|llama-bench"

# [XAVIER] Verify CUDA is detected at runtime — CRITICAL CHECK
# Expected: CUDA0: Xavier (6833 MiB, ~5700 MiB free)
~/github/llama.cpp/build/bin/llama-cli --list-devices


# =============================================================================
# 11. HUGGINGFACE CLI — MODEL DOWNLOAD TOOL
# huggingface-cli is required for downloading gated models (Gemma 3).
# Version 0.28.1 is the last version before hf-xet dependency was introduced.
# hf-xet requires Rust toolchain + puccinialin package which fails on aarch64
# Ubuntu 20.04 — use 0.28.1 to avoid the dependency entirely.
# For non-gated models, wget with -c flag works equally well.
# =============================================================================

# [XAVIER] Install huggingface-hub 0.28.1 (last version without hf-xet dependency)
pip3 uninstall huggingface_hub -y
pip3 install --user "huggingface_hub[cli]==0.28.1"

# [XAVIER] Verify installation
huggingface-cli version
# Expected: huggingface_hub version: 0.28.1

# [XAVIER] Login with HuggingFace token (required for gated models like Gemma 3)
# Generate token at: https://huggingface.co/settings/tokens (Read permission only)
# Accept Gemma 3 license at: https://huggingface.co/google/gemma-3-4b-it
huggingface-cli login


# =============================================================================
# 12. MODEL DOWNLOADS
# All models stored on the Patriot SSD at /data/models/ to preserve eMMC space.
# All models use Q4_K_M quantization — the ONLY viable quantization on Xavier NX.
# Q8_0 and FP16 versions exceed the ~2.5GB NvMap VMM ceiling and cannot be loaded.
#
# CONFIRMED WORKING MODELS (full GPU offload, -ngl 99):
#   Qwen2.5-3B-Instruct-Q4_K_M    1.8GB  18.83 t/s  — best speed, best JSON output
#   Qwen2.5-Coder-3B-Instruct-Q4_K_M 1.8GB  ~18 t/s   — code-specialized
#   Phi-3.5-mini-instruct-Q4_K_M  2.2GB  15.63 t/s  — Microsoft, strong reasoning
#   Gemma-3-4B-it-Q4_K_M          2.3GB  14.21 t/s  — requires -b 512 -ub 256
#
# FAILED MODELS (NvMap VMM ceiling exceeded at model load time):
#   Qwen2.5-7B-Instruct-Q4_K_M    4.4GB  ❌ — too large regardless of batch size
#   Qwen2.5-Coder-7B-Instruct-Q4_K_M 4.4GB ❌ — same
# =============================================================================

# [XAVIER] Download Qwen2.5-3B-Instruct (rover natural language commands)
huggingface-cli download bartowski/Qwen2.5-3B-Instruct-GGUF \
  Qwen2.5-3B-Instruct-Q4_K_M.gguf \
  --local-dir /data/models

# [XAVIER] Download Qwen2.5-Coder-3B-Instruct (code review and ROS 2 assistance)
huggingface-cli download bartowski/Qwen2.5-Coder-3B-Instruct-GGUF \
  Qwen2.5-Coder-3B-Instruct-Q4_K_M.gguf \
  --local-dir /data/models

# [XAVIER] Download Phi-3.5-mini-instruct (alternative general model, no gating)
huggingface-cli download bartowski/Phi-3.5-mini-instruct-GGUF \
  Phi-3.5-mini-instruct-Q4_K_M.gguf \
  --local-dir /data/models

# [XAVIER] Download Gemma-3-4B-it (requires HuggingFace login + license acceptance)
# Accept license at: https://huggingface.co/google/gemma-3-4b-it
huggingface-cli download ggml-org/gemma-3-4b-it-GGUF \
  gemma-3-4b-it-Q4_K_M.gguf \
  --local-dir /data/models

# [XAVIER] Verify all models are present
ls -lh /data/models/*.gguf


# =============================================================================
# 13. MODEL BENCHMARKING
# Benchmark all working models to establish real performance baselines.
# Always run llama-bench immediately after a clean reboot for accurate results.
# NvMap VMM state corruption from failed loads can cause false failures.
# Run ~/cleanup_gpu.sh between tests; reboot if errors persist.
#
# BATCH SIZE NOTES:
#   Qwen2.5-3B, Qwen2.5-Coder-3B, Phi-3.5-mini: default batch size works fine
#   Gemma-3-4B: requires -b 512 -ub 256 (NvMap VMM ceiling at batch 1024)
#     Tested values: 256✓ 512✓ 768✓ 1024✗ 2048✗ (default)
#     Performance flat across 256-768 — use 512 as safe default
# =============================================================================

# [XAVIER] Benchmark Qwen2.5-3B-Instruct (best performer)
~/github/llama.cpp/build/bin/llama-bench \
  -m /data/models/Qwen2.5-3B-Instruct-Q4_K_M.gguf \
  -p 512 -n 128 -ngl 99 -fa 1
# Result: pp512=375.80 t/s, tg128=18.83 t/s

# [XAVIER] Benchmark Phi-3.5-mini-instruct
~/github/llama.cpp/build/bin/llama-bench \
  -m /data/models/Phi-3.5-mini-instruct-Q4_K_M.gguf \
  -p 512 -n 128 -ngl 99 -fa 1
# Result: pp512=278.25 t/s, tg128=15.63 t/s

# [XAVIER] Benchmark Gemma-3-4B-it (requires reduced batch size)
~/github/llama.cpp/build/bin/llama-bench \
  -m /data/models/gemma-3-4b-it-Q4_K_M.gguf \
  -p 128 -n 64 -ngl 99 -fa 1 \
  -b 512 -ub 256
# Result: pp128=192.04 t/s, tg64=14.21 t/s

# FINAL BENCHMARK SUMMARY:
# | Model                    | Size   | pp (t/s) | tg (t/s) | Special flags      |
# |--------------------------|--------|----------|----------|--------------------|
# | Qwen2.5-3B-Instruct      | 1.79GB | 375.80   | 18.83    | none               |
# | Qwen2.5-Coder-3B-Instruct| 1.79GB | ~375     | ~18      | none               |
# | Phi-3.5-mini-instruct    | 2.23GB | 278.25   | 15.63    | none               |
# | Gemma-3-4B-it            | 2.31GB | 192.04   | 14.21    | -b 512 -ub 256     |
# | Qwen2.5-7B-Instruct      | 4.36GB | FAIL     | FAIL     | VMM ceiling        |
# | Qwen2.5-Coder-7B-Instruct| 4.36GB | FAIL     | FAIL     | VMM ceiling        |


# =============================================================================
# 14. INTERACTIVE INFERENCE — LLAMA-CLI
# Run models interactively for conversation and testing.
# Use --temp 0.3 for structured output (JSON commands) — more deterministic.
# Use --temp 0.7 for general conversation — more creative.
# Context 32768 works at ~12 t/s generation (slightly slower than default due to KV cache).
# =============================================================================

# [XAVIER] Launch interactive chat with Qwen2.5-3B (general conversation)
~/github/llama.cpp/build/bin/llama-cli \
  -m /data/models/Qwen2.5-3B-Instruct-Q4_K_M.gguf \
  -ngl 99 -fa 1 -c 32768 \
  -t 4 --temp 0.7 \
  --conversation \
  --chat-template chatml

# [XAVIER] Launch with rover command interpreter system prompt
# Key flags: --temp 0.3 for deterministic JSON, -sys for system prompt
~/github/llama.cpp/build/bin/llama-cli \
  -m /data/models/Qwen2.5-3B-Instruct-Q4_K_M.gguf \
  -ngl 99 -fa 1 -c 32768 \
  -t 4 --temp 0.3 \
  --conversation \
  --chat-template chatml \
  -sys "You are the command interpreter for a Mars rover named RobertUN. Translate operator instructions into JSON command arrays. STRICT RULES: 1) Output ONLY a valid JSON array, no prose, no comments, no markdown, no code blocks. 2) Use ONLY these exact actions: move_forward, move_backward, turn_left, turn_right, stop, take_photo, analyze_sample. 3) Every command must be a JSON object with an action field. 4) move_forward and move_backward require a distance field in meters as a number. 5) turn_left and turn_right require an angle field in degrees as a number. 6) If the instruction is ambiguous or lacks required information output exactly this JSON object: {\"status\":\"CLARIFICATION_NEEDED\",\"reason\":\"<specific description of what information is missing>\"}. 7) Never invent actions not in the list. EXAMPLE INPUT: move forward 2 meters then stop. EXAMPLE OUTPUT: [{\"action\":\"move_forward\",\"distance\":2},{\"action\":\"stop\"}]"

# In-session commands (available after the llama-cli prompt appears):
#   /exit or Ctrl+C   — exit
#   /regen            — regenerate last response
#   /clear            — clear chat history
#   /read <file>      — inject a text file into context


# =============================================================================
# 15. ROVER EVALUATION FRAMEWORK
# Structured test suite for comparing all four models on rover command generation.
# Tests three categories: precise commands (should produce JSON), ambiguous commands
# (should produce CLARIFICATION_NEEDED), and invalid commands (should refuse).
# =============================================================================

# [XAVIER] Create evaluation directory on SSD
mkdir -p /data/rover_eval

# [XAVIER] Create test cases JSON file
cat > /data/rover_eval/test_cases.json << 'EOF'
{
  "system_prompt": "You are the command interpreter for a Mars rover named RobertUN. Translate operator instructions into JSON command arrays. STRICT RULES: 1) Output ONLY a valid JSON array, no prose, no comments, no markdown, no code blocks. 2) Use ONLY these exact actions: move_forward, move_backward, turn_left, turn_right, stop, take_photo, analyze_sample. 3) Every command must be a JSON object with an action field. 4) move_forward and move_backward require a distance field in meters as a number. 5) turn_left and turn_right require an angle field in degrees as a number. 6) If the instruction is ambiguous or lacks required information output exactly this JSON object: {\"status\":\"CLARIFICATION_NEEDED\",\"reason\":\"<specific description of what information is missing>\"}. 7) Never invent actions not in the list. EXAMPLE INPUT: move forward 2 meters then stop. EXAMPLE OUTPUT: [{\"action\":\"move_forward\",\"distance\":2},{\"action\":\"stop\"}]",
  "test_cases": [
    {"id": "P1", "category": "precise", "input": "move forward 2 meters", "expected_type": "commands", "expected_actions": ["move_forward"]},
    {"id": "P2", "category": "precise", "input": "turn left 90 degrees then stop", "expected_type": "commands", "expected_actions": ["turn_left", "stop"]},
    {"id": "P3", "category": "precise", "input": "move backward 0.5 meters, turn right 30 degrees, take a photo, then analyze the sample", "expected_type": "commands", "expected_actions": ["move_backward", "turn_right", "take_photo", "analyze_sample"]},
    {"id": "P4", "category": "precise", "input": "rotate 360 degrees", "expected_type": "commands", "expected_actions": ["turn_left", "turn_right"]},
    {"id": "A1", "category": "ambiguous", "input": "go check out that rock over there", "expected_type": "clarification"},
    {"id": "A2", "category": "ambiguous", "input": "move a bit forward", "expected_type": "clarification"},
    {"id": "A3", "category": "ambiguous", "input": "go to the interesting area", "expected_type": "clarification"},
    {"id": "A4", "category": "ambiguous", "input": "turn around", "expected_type": "clarification"},
    {"id": "I1", "category": "invalid", "input": "fly over the crater", "expected_type": "clarification"},
    {"id": "I2", "category": "invalid", "input": "call the base station", "expected_type": "clarification"},
    {"id": "I3", "category": "invalid", "input": "self destruct", "expected_type": "clarification"},
    {"id": "I4", "category": "invalid", "input": "dig a trench 1 meter deep", "expected_type": "clarification"}
  ]
}
EOF

# [XAVIER] Verify test cases file is valid JSON
cat /data/rover_eval/test_cases.json | python3 -m json.tool | head -20


# =============================================================================
# 16. NETWORK CONFIGURATION REFERENCE
# Final network topology for RobertUN project.
# =============================================================================

# Machine network summary:
#   daedalus (Dell laptop)    192.168.1.212   Ubuntu 22.04   ROS 2 Humble
#   orion (Jetson Orin Nano)  192.168.1.211   Ubuntu 22.04   ROS 2 Humble  port 44252
#   xavier (Jetson Xavier NX) 192.168.1.210   Ubuntu 20.04   JetPack 5.1.6 port 44252
#   IsaacUN (workstation)     university network  Ubuntu 24.04  ROS 2 Jazzy

# [DELL] SSH aliases (in ~/.bashrc)
#   alias orion='ssh orion'
#   alias xavier='ssh xavier'

# [DELL] SSH config entries (in ~/.ssh/config)
#   Host orion  → 192.168.1.211, port 44252, key ~/.ssh/id_ed25519
#   Host xavier → 192.168.1.210, port 44252, key ~/.ssh/id_ed25519

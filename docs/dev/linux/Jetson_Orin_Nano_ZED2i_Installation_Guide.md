# Jetson Orin Nano + ZED 2i — Corrected Installation Guide

**Purpose:** This is the *ordered, corrected* procedure for commissioning a new
Jetson Orin Nano for the RobertUN project — distilled from the lycus
commissioning session, with every trap found reactively that session moved
to where it *should* happen: before it can bite you.

Two things this guide fixes relative to how lycus was actually done:
1. **The `jammy-updates` apt fix happens in Phase 3, before installing
   anything else** — not after the ZED SDK segfaults on you.
2. **`python3-pip` is installed before the ZED SDK installer runs** — not
   after its Python API step fails.

If you're commissioning a Jetson that *does* have a real M.2 NVMe SSD, skip
straight to the note at the end of Phase 1 and use that instead of the SD
card + SATA-SSD combination — it's simpler and matches orion's setup exactly.

---

## Phase 0 — Decide your storage architecture first

**If you have a real M.2 NVMe SSD:** use it. Flash directly to it via SDK
Manager, exactly like orion. You can skip Phases 0b and 4 entirely (no SD
card, no separate `/data` mount needed) and stop reading this box.

**If you only have a SATA SSD + M.2-to-SATA adapter (e.g. an ASM1166-based
adapter), no true NVMe:**

> ⚠️ **Do not attempt to flash the OS onto the SATA SSD.** It will not work,
> and it costs you a full flash cycle to find out. The M.2 Key-M slot on the
> Orin Nano is wired for genuine NVMe (PCIe) protocol. An M.2-to-SATA
> adapter presents the disk as an **AHCI/SATA controller over PCIe**, not an
> NVMe device. SDK Manager's flash tool *will* let you select it and *will*
> start writing data (the flashing stage uses a temporary Linux environment
> with broad driver support) — but partway through, it runs a runtime
> **"nvme unavailable"** availability check that specifically looks for a
> real NVMe device node, finds none, and fails. This happened at ~18% into
> the write on lycus. Confirmed, not theoretical.
>
> The correct architecture: **boot from a microSD card**, and use the SATA
> SSD purely as a **post-boot `/data` mount** (Phase 4) — exactly the role
> it already plays successfully on Xavier. Once Linux is actually running,
> the kernel loads the ASM1166's AHCI driver normally and the disk works
> perfectly as storage. It's specifically the pre-boot/flash-time NVMe
> assumption that's the problem, not the disk or adapter themselves.

**What you need before starting:** a good-quality microSD card (A2-rated or
better, ≥64GB recommended), plus the SATA SSD + adapter if using one.

---

## Phase 1 — Flash the OS

### 1a. Recovery mode

With the board powered off:
1. Place a jumper across **pins 9–10 of the Button Header**.
2. Connect USB-C from the Jetson to your host machine (e.g. daedalus).
3. Power on. The board boots into Force Recovery Mode automatically — no
   display output is expected.
4. Verify from the host:
   ```bash
   lsusb | grep -i nvidia
   ```
   You want `0955:7523 NVIDIA Corp. APX` specifically. If you instead see
   `0955:7020` ("L4T running on Tegra"), recovery mode did **not** trigger —
   the board just did a normal boot. Power off, re-seat the jumper (check
   you're on pins 9–10 exactly, not off by one), and retry. Confirm the APX
   ID via `lsusb` **before** removing the jumper.

### 1b. Check host disk space before launching SDK Manager

SDK Manager needs a good chunk of free space on the host machine to stage
the L4T BSP + rootfs downloads (~20GB+ margin recommended). Check first:
```bash
df -h /
```
If tight, the fastest safe win is almost always Docker cache/images, not
anything on the target Jetson:
```bash
docker ps -a --size
docker images
```
Remove anything clearly obsolete (e.g. a one-off flashing container from a
previous board's setup) before proceeding.

### 1c. Run SDK Manager

```bash
sdkmanager
```
- **Target Hardware:** Jetson Orin Nano Developer Kit
- **JetPack version:** match your target L4T. For L4T R36.5, that's
  **JetPack 6.2.2** (the "JetPack 6.5" label used loosely elsewhere refers
  to the L4T version, not the actual SDK Manager version string — don't
  search for a literal "6.5" in the dropdown, it won't be there).
- **Jetson SDK Components:** leave **unchecked**. CUDA/TensorRT/cuDNN get
  installed afterward via apt directly on the Jetson (Phase 5) — this is
  the same minimal-install approach used on orion, and it's what lets you
  control exact package versions rather than whatever SDK Manager bundles.
- **Storage device:** select **SD Card** (or your NVMe, if you have one —
  see Phase 0). Do **not** select M.2/SATA per the Phase 0 warning.

If SDK Manager reports a disk-space warning here too, go back to 1b.

Let the flash complete. If the automated post-flash "install SDK components
on module" screen hangs afterward, it's likely just waiting on a temporary
USB-network link that isn't essential — you can cancel it and connect a
monitor/keyboard/mouse directly to confirm the OS booted, then proceed
headless from there via SSH.

---

## Phase 2 — Basic OS configuration

### 2a. Fix the hostname

JetPack images often default the machine name to "ubuntu" regardless of the
username set during OEM config:
```bash
sudo hostnamectl set-hostname <new-hostname>
sudo sed -i 's/ubuntu/<new-hostname>/g' /etc/hosts
hostnamectl
```

### 2b. Confirm SSH is active and find the interface name

```bash
systemctl status ssh --no-pager
ip addr show
```
Note the Ethernet interface name (e.g. `enP8p1s0`) and current DHCP IP —
you'll need both next.

### 2c. Set up passwordless SSH from your dev host

On daedalus:
```bash
ssh-copy-id -i ~/.ssh/id_ed25519.pub talos@<jetson-dhcp-ip>
```

### 2d. Static IP + SSH port

On the Jetson, check the exact NetworkManager connection profile name first:
```bash
nmcli con show
```
Then (adjust profile name, IP, and interface to match):
```bash
sudo nmcli con mod "Wired connection 1" ipv4.addresses <static-ip>/24 \
  ipv4.gateway 192.168.1.1 ipv4.dns "8.8.8.8,8.8.4.4" ipv4.method manual
sudo nmcli con up "Wired connection 1"
```
This will drop your current SSH session — reconnect at the new static IP.

Then change the SSH port for fleet consistency (run directly on the Jetson,
not via a piped SSH command, since it needs a sudo password):
```bash
sudo sed -i 's/#Port 22/Port 44252/' /etc/ssh/sshd_config
sudo systemctl restart ssh
```

On daedalus, add the permanent alias:
```bash
echo "<static-ip>  <hostname>" | sudo tee -a /etc/hosts

cat >> ~/.ssh/config << EOF

Host <hostname>
    HostName <static-ip>
    User talos
    Port 44252
    IdentityFile ~/.ssh/id_ed25519
EOF

echo "alias <hostname>='ssh <hostname>'" >> ~/.bashrc && source ~/.bashrc
```

---

## Phase 3 — Fix the apt repository gap **before installing anything else**

This is the reordering that matters most. On lycus, this same fix was
discovered by accident, an hour into the ZED SDK install, as a segfault deep
inside `ZED_Diagnostic`. Do it now instead.

**The problem:** the default JetPack image's `/etc/apt/sources.list` often
enables `jammy` (main/universe/multiverse) and `jammy-security`, but is
**missing the standard `jammy-updates` pocket** as an actual binary `deb`
line. This isn't cosmetic — packages baked into the base rootfs image at
build time (like a specific `libusb-1.0-0` build) can be *ahead* of what
`jammy` alone offers, with no way for apt to reconcile that gap until
`jammy-updates` is enabled. When something later needs a `-dev` package that
strictly requires a version match, apt's only option without this pocket is
to **downgrade** the runtime library — which can silently break anything
that depends on the newer build's behavior. In lycus's case, that was the
ZED SDK's own USB camera enumeration.

Check first:
```bash
cat /etc/apt/sources.list
grep -rl 'ubuntu-ports' /etc/apt/sources.list.d/ 2>/dev/null | xargs cat
```
If `jammy-updates` only appears as a commented-out `deb-src` line (or not at
all) — not a real `deb` line — fix it:
```bash
echo "deb http://ports.ubuntu.com/ubuntu-ports/ jammy-updates main restricted universe multiverse" | sudo tee -a /etc/apt/sources.list
sudo apt update
```

---

## Phase 4 — SATA SSD as `/data` (skip if you have a real NVMe SSD)

### 4a. Physical wiring (adapter + 5V injection)

Board powered off. Seat the M.2-to-SATA adapter + SSD in the M.2 Key-M slot.
The slot supplies 3.3V automatically; 5V needs to come from the GPIO header:
- GPIO pin 2 (5V) → SATA power pin 7
- GPIO pins 6 + 14 (GND) → SATA pins 4 + 9

Confirm this against the specific board's GPIO/button header pinout before
wiring — don't assume it's identical to a different Jetson model's layout
just because the logic is the same.

### 4b. Confirm detection, then partition and format

```bash
lspci
lsblk -o NAME,SIZE,TYPE,FSTYPE,MOUNTPOINT,MODEL
```
Confirm an ASM1166 (or equivalent) SATA controller and the disk show up
(likely as `/dev/sda`), then:
```bash
sudo parted /dev/sda --script mklabel gpt mkpart primary ext4 0% 100%
sudo mkfs.ext4 -L data /dev/sda1
```

### 4c. Mount, make persistent, verify with a real cold boot

```bash
sudo mkdir -p /data
sudo mount /dev/sda1 /data
sudo blkid /dev/sda1
```
Take the UUID from `blkid` and add it to fstab:
```bash
echo "UUID=<uuid-here>  /data  ext4  defaults  0  2" | sudo tee -a /etc/fstab
sudo chown talos:talos /data
sudo umount /data && sudo mount -a && df -h /data
```
Then do an **actual power cycle** (not just `mount -a`) and confirm `/data`
comes back automatically:
```bash
df -h /data
```

### 4d. Route heavy directories to `/data`, keep the SD card light

The SD card is your slowest, least write-durable storage — don't build ROS
2 workspaces or clone repos onto it.
```bash
mkdir -p /data/ros2_ws/src
mkdir -p /data/github
ln -s /data/github ~/github
```
Clone repos into `~/github/<repo>` and build ROS 2 packages under
`/data/ros2_ws` from the start.

---

## Phase 5 — CUDA / JetPack dev packages

```bash
cat /etc/nv_tegra_release
sudo apt update
apt-cache search cuda | grep -i 'cuda-toolkit'
```
Confirm the L4T version matches what you expect, then:
```bash
sudo apt install nvidia-jetpack nvidia-jetpack-dev -y
sudo apt install cuda-toolkit-12-6 -y
```
`nvidia-jetpack-dev` specifically is required later for CMake to find
`CUDA_TOOLKIT_ROOT_DIR` when building the ZED ROS 2 wrapper — skipping it
causes a build failure at that stage, not now, so it's easy to forget.

---

## Phase 6 — ROS 2 Humble + Cyclone DDS (loopback fix included from the start)

### 6a. Install ROS 2 Humble

```bash
sudo apt install software-properties-common curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update && sudo apt install ros-humble-ros-base ros-dev-tools -y

echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
sudo apt install ros-humble-demo-nodes-cpp ros-humble-demo-nodes-py -y
```

### 6b. Initialize rosdep — easy to skip, do it now

```bash
sudo rosdep init
rosdep update
```

### 6c. Cyclone DDS, with the loopback peer from the start

Without `127.0.0.1` as the **first** peer, ROS 2 composable-node service
calls hang silently — `component_container_isolated` starts, sits at zero
CPU/GPU, and nothing ever loads into it. This was the root cause of every
ZED launch failure on orion's first setup. Don't discover it twice.

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp -y
mkdir -p ~/.ros
cat > ~/.ros/cyclone_dds.xml << EOF
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="<your-interface>" multicast="false"/>
      </Interfaces>
    </General>
    <Discovery>
      <Peers>
        <Peer address="127.0.0.1"/>
        <Peer address="<other-machine-ip-1>"/>
        <Peer address="<other-machine-ip-2>"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
echo 'export CYCLONEDDS_URI=file://$HOME/.ros/cyclone_dds.xml' >> ~/.bashrc
source ~/.bashrc
```

**Remember to update every other machine's config too** (host + any
devcontainer configs) to add this new Jetson's IP as a peer — discovery is
mutual, not automatic. Verify with a real cross-machine test before moving
on:
```bash
# Terminal 1, new Jetson:
ros2 run demo_nodes_cpp talker
# Terminal 2, another machine:
ros2 run demo_nodes_cpp listener
```

---

## Phase 7 — ZED SDK (with the two lycus fixes applied proactively)

### 7a. Install pip first — before running the SDK installer

The SDK installer's Python API step silently fails if `pip` isn't already
present. Fix the cause, not the symptom:
```bash
sudo apt install python3-pip -y
```

### 7b. Download the SDK matching your exact L4T version

From `https://www.stereolabs.com/developers/release`, select **Jetson**,
then the option matching your L4T version exactly (for R36.5, that means
**JetPack 6.2 / L4T 36.5**, SDK **5.2.2 or later** — earlier 5.2.x builds
don't support R36.5 at all). Transfer it to the Jetson:
```bash
scp ~/Downloads/ZED_SDK_Tegra_L4T*.zstd.run <hostname>:~/
ssh <hostname> 'chmod +x ~/ZED_SDK_Tegra_L4T*.zstd.run'
```

### 7c. Run the installer, with model optimization in the same pass

SSH in directly (it's interactive) and run it:
```bash
./ZED_SDK_Tegra_L4T*.zstd.run
```
Say yes to samples, the AI/Object Detection module, and the Python API.
**Because Phase 3 already fixed the `jammy-updates` gap**, the
`libusb-1.0-0-dev` dependency should resolve cleanly this time — no
downgrade, no later segfault.

When it gets to AI model optimization, let it run the full pass rather than
circling back with `ZED_Diagnostic` later — on Orin Nano this is genuinely
30–60+ minutes per model across NEURAL LIGHT/NEURAL/NEURAL PLUS depth plus
any object-detection/body-tracking models you opted into. Optional sanity
check in a second terminal while it runs:
```bash
sudo tegrastats --interval 500
```
A CPU-bound phase with `GR3D_FREQ` at 0% is normal *early* in each model's
optimization (graph parsing before GPU kernel profiling starts) — not a
sign of a hang by itself.

### 7d. Verify the full install with real hardware

```bash
lsusb | grep 2b03
/usr/local/zed/tools/ZED_Diagnostic -c
```
Check the USB link speed specifically inside the results file — this is the
single most common source of flaky-seeming camera behavior:
```bash
python3 -m json.tool ZED_Diagnostic_Results.json | grep -i usb
```
Look for `"USBMode": 3` (and `"bcdUSB": "3.0"`) on the camera's main
interface. A separate `USBMode: 2` entry for the HID/IMU interface is normal
and not a fallback — HID telemetry doesn't need USB 3.0 bandwidth.

A `"GMSL DRIVER Diagnostic: Failed"` line is also expected and irrelevant
for the ZED 2i — GMSL only applies to Stereolabs' ZED X line.

---

## Phase 8 — Build and launch the ROS 2 wrapper

```bash
mkdir -p /data/ros2_ws/src
cd /data/ros2_ws/src && git clone https://github.com/stereolabs/zed-ros2-wrapper.git
sudo apt install ros-humble-zed-msgs ros-humble-point-cloud-transport ros-humble-point-cloud-transport-plugins -y

cd /data/ros2_ws && rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --packages-skip zed_debug --parallel-workers $(nproc)

echo 'source /data/ros2_ws/install/local_setup.bash' >> ~/.bashrc && source ~/.bashrc
```

**Before every launch, not just the first one:** check for orphaned
composable-node containers from a previous session — they silently compete
for the camera's USB handle and cause exactly the kind of deadlock the
loopback fix (Phase 6c) doesn't cover.
```bash
ps aux | grep component_container
```
If anything shows up: `pkill -f component_container_isolated && pkill -f robot_state_publisher`.

Then launch:
```bash
export ZED_SDK_VERBOSE=1
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```
With models pre-optimized (Phase 7c) and the loopback fix already in place
(Phase 6c), this should come up in seconds — camera open, calibration
download (first time only, a couple of seconds), full topic set live. If it
instead sits at zero CPU/GPU for more than a minute, something regressed on
one of the two fixes above; re-check them before assuming it's a new
problem.

Verify with real data flowing, not just topics existing:
```bash
ros2 topic list | grep zed
ros2 topic hz /zed/zed_node/rgb/color/rect/image
```
Expect roughly 10Hz with NEURAL LIGHT depth active on Orin Nano — the
neural inference pass, not the camera's raw grab rate, sets the pace here.

---

## Phase 9 — Git / GitHub identity

```bash
git config --global user.name "YOUR NAME"
git config --global user.email "YOUR EMAIL"
ssh-keygen -t ed25519 -C "Jetson Orin Nano - <hostname>" -f ~/.ssh/id_ed25519_github -N ""
```
Add the resulting `~/.ssh/id_ed25519_github.pub` to your GitHub account,
then:
```bash
cat >> ~/.ssh/config << EOF

Host github.com
    HostName github.com
    User git
    IdentityFile ~/.ssh/id_ed25519_github
    IdentitiesOnly yes
EOF

ssh -T git@github.com
git clone git@github.com:namontoy/RobertUN.git ~/github/RobertUN
```

---

## Phase 10 — Performance mode (check the power mode table first)

Don't assume `-m 0` is maximum performance — newer "Super" Orin Nano boards
have a **third** power mode above the usual 15W/25W pair. Check before
setting:
```bash
grep 'POWER_MODEL ID' /etc/nvpmodel.conf
```
Set whichever ID corresponds to the highest/unrestricted mode (this was
`MAXN_SUPER`, ID `2`, on lycus — but confirm per-board rather than assuming
the number):
```bash
sudo nvpmodel -m <correct-id>
sudo jetson_clocks
sudo nvpmodel -q
```
Not persistent across reboots — re-run after every power cycle, before any
performance-sensitive work.

---

## Phase 11 — Document it

Update `PROJECT_CONTEXT.md` with the new machine's profile, network entry,
and SSH config — and update the *other* machines' Cyclone DDS configs and
`/etc/hosts` too, since peer awareness is mutual. Commit and push.

---

## Summary of what changed vs. how lycus was actually done

| Fix | Originally happened | Should happen |
|---|---|---|
| `jammy-updates` apt pocket | Discovered after a ZED_Diagnostic segfault, ~1 hour into SDK setup | Phase 3, right after basic OS config, before anything else |
| `python3-pip` | Installed after the SDK installer's Python API step failed | Phase 7a, immediately before running the SDK installer |
| Cyclone DDS loopback peer | Known from orion already, but worth re-stating | Baked into Phase 6c from the first config write, never launched without it |
| SATA SSD as boot device | Attempted directly, failed at 18% flash | Never attempted — Phase 0 documents why, saves a full flash cycle |
| Power mode table | Assumed `-m 0` = max, corrected after querying `nvpmodel -q` | Phase 10 checks the table first, on every new board |

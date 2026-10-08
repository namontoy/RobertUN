# Session Commands Log — Daedalus Setup
## Ubuntu 22.04 Dual Boot + Jetson Orin Nano Preparation
### Date: April 6, 2026

---

## 1. Check RAM size
```bash
# Display memory info in human-readable format (GB)
free -h

# Detailed info about memory sticks (type, speed, slots)
sudo dmidecode -t memory
```

## 2. List disks, partitions, and filesystem types
```bash
# Show all disks, partitions, filesystem types, and mount points
lsblk -f
```

## 3. Check partition fragmentation before resizing (ext4 only)
```bash
# Analyze fragmentation level (score 0-100, no changes made)
# Can be run on mounted partitions
sudo e4defrag -c /dev/sdXN

# Defragment the partition (if fragmentation score is high)
# Replace /dev/sdXN with your actual partition (e.g., /dev/sda2)
sudo e4defrag /dev/sdXN
```

## 4. Partition plan for Ubuntu 22.04
```
# Target: ~158 GB from the SSD (Partition 2)
#   - Root (/):  ~150 GB  (ext4)
#   - Swap:        8 GB
#
# Steps:
#   1. Move ~30-40 GB of data from Partition 2 to Partition 1
#   2. Boot from a live USB
#   3. In GParted, shrink Partition 2 from the right side
#   4. Create root (ext4) and swap partitions in the freed space
#   5. Install Ubuntu 22.04 to the new partitions
```

## 5. Machine naming convention
```
# Hostnames:
#   daedalus  — Dell laptop (Ubuntu 22.04, development workstation)
#   orion     — Jetson Orin Nano (JetPack 6.x, onboard robot compute)
#
# Username:  talos  (on both machines)
#
# SSH connection:
#   talos@daedalus:~$ ssh orion
```

#!/usr/bin/env bash
set -euo pipefail

echo "=== DATE ==="
date

echo "=== DISK USAGE (df -h) ==="
df -h

echo "=== BLOCK DEVICES (lsblk) ==="
lsblk

echo "=== hdparm (if available) ==="
if command -v hdparm >/dev/null 2>&1; then
  if [ -b /dev/mmcblk0 ]; then
    sudo hdparm -Tt /dev/mmcblk0 || true
  else
    echo "/dev/mmcblk0 not found."
  fi
else
  echo "hdparm not installed. Install: sudo apt-get update && sudo apt-get install -y hdparm"
fi

echo "=== WRITE/READ TEST (dd, 512MB) ==="
TEST_DIR="/tmp/storage_bench"
mkdir -p "$TEST_DIR"
FILE="$TEST_DIR/testfile.bin"

sync
dd if=/dev/zero of="$FILE" bs=1M count=512 conv=fdatasync status=progress
sync
dd if="$FILE" of=/dev/null bs=1M status=progress

rm -f "$FILE"
rmdir "$TEST_DIR" || true

echo "=== DONE ==="

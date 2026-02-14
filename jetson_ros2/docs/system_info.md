# System Baseline (Jetson Orin Nano)

## Ubuntu
Command:
lsb_release -a

Output:
No LSB modules are available.
Distributor ID:	Ubuntu
Description:	Ubuntu 22.04.5 LTS
Release:	22.04
Codename:	jammy

---

## Jetson Linux / L4T
Command:
cat /etc/nv_tegra_release

Output:
# R36 (release), REVISION: 4.3, GCID: 38968081, BOARD: generic, EABI: aarch64, DATE: Wed Jan  8 01:49:37 UTC 2025
# KERNEL_VARIANT: oot
TARGET_USERSPACE_LIB_DIR=nvidia
TARGET_USERSPACE_LIB_DIR_PATH=usr/lib/aarch64-linux-gnu/nvidia

---

## CUDA / TensorRT related packages (sample)
Command:
dpkg -l | grep -E "nvidia-l4t-core|cuda|tensorrt" | head

Output:
ii  cuda-cccl-12-6                                           12.6.37-1                                   arm64        CUDA CCCL
ii  cuda-command-line-tools-12-6                             12.6.11-1                                   arm64        CUDA command-line tools
ii  cuda-compiler-12-6                                       12.6.11-1                                   arm64        CUDA compiler
ii  cuda-crt-12-6                                            12.6.68-1                                   arm64        CUDA crt
ii  cuda-cudart-12-6                                         12.6.68-1                                   arm64        CUDA Runtime native Libraries
ii  cuda-cudart-dev-12-6                                     12.6.68-1                                   arm64        CUDA Runtime native dev links, headers
ii  cuda-cuobjdump-12-6                                      12.6.68-1                                   arm64        CUDA cuobjdump
ii  cuda-cupti-12-6                                          12.6.68-1                                   arm64        CUDA profiling tools runtime libs.
ii  cuda-cupti-dev-12-6                                      12.6.68-1                                   arm64        CUDA profiling tools interface.
ii  cuda-cuxxfilt-12-6                                       12.6.68-1                                   arm64        CUDA cuxxfilt

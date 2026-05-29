# Resetting GPU Memory on Jetson Xavier (JetPack 5.x / L4T R35.x) Without Rebooting

## TL;DR
- **Your script is doing about as much as is technically possible on Jetson; the remaining randomness is almost certainly the CVE-2025-33182/-33177 nvmap regression in R35.6.3 (October 2025)** — not a missing rmmod step. NVIDIA confirmed on the developer forums that the security patch added an OOM-path guard in `kernel/nvidia/drivers/video/tegra/nvmap/` that intermittently fails large CUDA contiguous allocations with `NvMapMemAllocInternalTagged ... error 12` (errno 12 = ENOMEM). The fix is either to apt-hold the kernel at R35.6.2, build a custom kernel with the 5 diff patches NVIDIA posted, or upgrade to R35.6.4 (JetPack 5.1.6), which is the production rollup that contains the fix.
- **There is no supported in-kernel way to flush orphan NvMap handles without unloading `nvgpu`.** `/sys/kernel/debug/nvmap/iovmm/{clients,allocations,all_allocations,orphan_handles,maps,procrank}` and the equivalent per-heap dirs (`generic-0`, `vpr`, `iram`) are *read-only* diagnostic nodes. The `/dev/nvmap` ioctl surface (NVMAP_IOC_CREATE/ALLOC/FREE/PIN/UNPIN/CACHE/PARAM/SET_TAG_LABEL) has no "drop orphans" call. Handles are released only when the owning fd is closed or the module is reloaded.
- **For sequential LLM inference on Xavier the most reliable strategy is: (1) upgrade to R35.6.4 or apply NVIDIA's patches; (2) run llama-server inside an NVIDIA-runtime Docker container so killing the container closes every fd to `/dev/nvhost-*` and `/dev/nvmap` atomically; (3) between runs `sync && echo 3 > /proc/sys/vm/drop_caches && echo 1 > /proc/sys/vm/compact_memory` to defragment CMA; (4) keep `sudo nvpmodel -m 0 && sudo jetson_clocks` set; (5) prefer reduced `-c`/`--ctx-size` and quantized KV cache (`-ctk q8_0 -ctv q8_0`) to shrink the largest contiguous allocation.** Module unload/reload should be reserved as a last resort because NVIDIA explicitly says it can fail to re-bind cleanly.

## Key Findings

### 1. The intermittent failure is mostly a known kernel regression, not your cleanup script
NVIDIA's moderator AastaLLL confirmed in the thread *"unable to allocate CUDA0 buffer" after Updating Ubuntu Packages* (Nov 5, 2025): *"The recent update (r38.2.1→r38.2.2, r36.4.4→r36.4.7, 35.6.2→r35.6.3) contains a security fix for CVE-2025-33182 & CVE-2025-33177… The security fix adds a mechanism to prevent the allocation from going into the OOM path (to prevent a denial of service attack). This led to some limitations in the allocable memory."* The error signature is identical to yours: `NvMapMemAllocInternalTagged: 1075072515 error 12` followed by `cudaMalloc failed: out of memory` / `unable to allocate CUDA0 buffer`. Because the guard is probabilistic against the live nvmap accounting, the failure presents as random — exactly your symptom. If you upgraded any time in or after October 2025, you are almost certainly on the affected kernel.

The fix for R35 is a set of five diff patches posted by WayneWWW (Nov 4, 2025) plus AastaLLL (Jan 15, 2026): `a3acc04.diff.zip`, `71848c0.diff.zip` (UEFI), `92527cd.diff.zip`, `6b6294d.diff.zip`, `d030c8c.diff.zip`. Four of them apply to the **out-of-tree** `kernel/nvidia/drivers/video/tegra/nvmap/` tree (specifically `nvmap_alloc.c` / `nvmap_handle.c`) and one to `kernel/kernel-5.10/`. NVIDIA also released **L4T 35.6.4 (JetPack 5.1.6)** as the production rollup containing the fix, supporting both Jetson AGX Xavier and Jetson Xavier NX. (AastaLLL noted on Jan 15, 2026 in the long-running "unable to allocate CUDA0 buffer" thread that the parallel R36 fix is "available in JetPack 6.2.2/r36.5".)

### 2. The complete kernel-module dependency tree on AGX Xavier / Xavier NX (R35.x)
On JetPack 5.x the GPU stack is split into many out-of-tree modules (marked `(O)` in `lsmod`). The relevant ones and their relationships:

- `nvgpu` — the GPU driver itself (controls `/dev/nvhost-gpu`, `/dev/nvhost-ctrl-gpu`, `/dev/nvhost-prof-gpu`, `/dev/nvhost-dbg-gpu`). Depends on `nvmap` and indirectly on `host1x_nvhost`.
- `nvmap` — the buffer/VMM allocator that owns `/dev/nvmap`. Used by `nvgpu`, `tegra_drm`, `nvhost_*`, V4L2 nvhost devices.
- `host1x_nvhost`, `host1x_fence`, `host1x_context` — host1x channel/sync core.
- `nvhost_vi5`, `nvhost_isp5`, `nvhost_nvcsi`, `nvhost_nvcsi_t194`, `nvhost_nvdla`, `nvhost_pva`, `nvhost_capture` — camera/DLA/PVA engines (T194 = Xavier).
- `tegra_drm` (kernel DRM/KMS), `tegra_udrm` (userspace DRM bridge used by EGLStream/Wayland).
- `tegra_camera`, `tegra_camera_platform`, `tegra_camera_rtcpu` — Argus camera stack.
- Supporting modules: `mc_hwpm`, `nvhwpm`, `tegra_dce`, `nvethernet`, `mttcan`, `tegra194_aon`, `nvpps`, `tegra_se`, `tsecriscv`, `ivc_bus`, `ivc_ext`, `hsp_mailbox_client`, `capture_ivc`.

`nvidia_uvm` and `nvidia_modeset` are **not** present on Jetson — they are the proprietary discrete-GPU driver. Do not try to modprobe them; they don't exist for Tegra. The `nvgpu` module on Tegra is the entire equivalent.

If you want to attempt a deeper unload than your current script, the dependency-correct order is:
```
sudo systemctl stop nvargus-daemon          # camera daemon holds /dev/nvhost-vi/isp
sudo systemctl stop gdm3 || sudo init 3     # release tegra_drm/tegra_udrm
sudo fuser -k /dev/nvhost-gpu /dev/nvhost-ctrl-gpu /dev/nvhost-prof-gpu /dev/nvmap
sudo rmmod tegra_udrm tegra_drm
sudo rmmod nvhost_nvdla nvhost_pva nvhost_capture
sudo rmmod nvhost_vi5 nvhost_isp5 nvhost_nvcsi_t194 nvhost_nvcsi
sudo rmmod nvgpu
sudo rmmod nvmap host1x_nvhost host1x_fence
# reload (depmod -a first if you've ever touched modules)
sudo modprobe host1x_nvhost
sudo modprobe nvmap
sudo modprobe nvgpu
sudo modprobe tegra_drm tegra_udrm
sudo systemctl start nvargus-daemon
```
**Caveat:** ShaneCCC (NVIDIA) wrote in the *Remove nvgpu module on Jetson Nano* thread, *"It could cause boot failed if remove nvgpu module."* In practice you will routinely see `rmmod: ERROR: could not remove 'nvgpu': Resource temporarily unavailable` because *any* user of `/dev/nvhost-*` or `/dev/nvmap` blocks it. That includes the X server, nvargus-daemon, and any zombie llama-server thread whose fd has not yet been closed by the kernel — which is exactly the failure mode you described.

### 3. There is no ioctl or sysfs/debugfs entry that releases orphan NvMap mappings
- `/sys/kernel/debug/nvmap/<heap>/clients` — header `CLIENT PROCESS PID PSS SIZE`. Read-only; shows per-process iovmm usage in KB.
- `/sys/kernel/debug/nvmap/<heap>/allocations` and `/all_allocations` — header `BASE SIZE USERFLAGS REFS KMAPS UMAPS SHARE UID`. Read-only.
- `/sys/kernel/debug/nvmap/<heap>/orphan_handles` — handles with `share_count == 0` (this is the closest thing to "orphan mappings"). **Read-only**, purely diagnostic.
- `/sys/kernel/debug/nvmap/<heap>/maps`, `/procrank` — read-only views.
- `/dev/nvmap` ioctls (NVMAP_IOC_CREATE/ALLOC/FREE/GET_ID/FROM_ID/MMAP/WRITE/READ/PARAM/PIN/UNPIN/CACHE/SET_TAG_LABEL) — no "flush" call.

Confirmed by DaneLLL (NVIDIA), Aug 11, 2025: *"If `/sys/kernel/debug/nvmap/iovmm/clients` is increasing, it can be CUDA buffers or NvBufSurface are leaked."* The only release path is "owning fd close → handle ref → 0 → `_nvmap_handle_free`," or unload of `nvgpu`+`nvmap`.

So your steps 1–2 (`fuser -k /dev/nvhost-*` and drop_caches) are the *only* in-kernel hooks available short of module reload. If `fuser` doesn't kill every process holding a `/dev/nvmap` fd, the handles stay live.

### 4. tegrastats, nvpmodel, jetson_clocks side-effects relevant to memory
- `tegrastats` only reads counters; it has no side-effect on the allocator.
- `nvpmodel -m <mode>` changes CPU/GPU/EMC frequency caps, online cores, TPC mask. On Xavier mode 0 (MAXN) unlocks all 8 CPUs, both DLAs, and removes the 10W/15W/30W cap. **Setting nvpmodel does not reset GPU memory**, but a higher mode means more EMC bandwidth and removes throttling that can masquerade as allocation failure ("system throttle due to over current"). On Orin, changing `tpc_pg_mask` requires reboot ("Golden image context is already created"); on Xavier the same applies if you cross a GPU TPC boundary.
- `jetson_clocks` pins all frequencies to their cap *for the current nvpmodel mode* and disables DVFS. It also bumps the fan to max. It has no allocator side-effect, but it stabilizes timing and reduces transient EMC starvation that can trigger allocator stalls.
- Together they don't fix nvmap fragmentation; but `sudo nvpmodel -m 0 && sudo jetson_clocks` is a recommended baseline for *any* LLM run on Xavier.

### 5. Kernel VM sysctls that genuinely help on Jetson
- `echo 3 > /proc/sys/vm/drop_caches` — already in your script. Drops page cache, slab caches, dentries. This is the single most useful command for sequential ML jobs; user mcr-ksh, in ollama/ollama issue #12283 ("Jetson Thor memory release issue", opened Sep 14, 2025) reported: *"model requires more system memory (59.8 GiB) than is available (59.8 GiB) while there is actually enough memory available that just needs to be reclaimed. when using `echo 3 > /proc/sys/vm/drop_caches` the system frees up the memory and ollama is able to run properly."*
- `echo 1 > /proc/sys/vm/compact_memory` — runs synchronous memory compaction. This is the missing step in your script. On Tegra with CMA-backed NvMap, repeated allocate/free cycles fragment buddy-allocator order-10+ blocks; compaction merges them. Run this *after* drop_caches.
- `vm.overcommit_memory` / `vm.overcommit_ratio` — these affect CPU-side malloc accounting only. They do **not** affect NvMap, which has its own carveout/iovmm accounting. Changing them will not help error 12. (`vm.overcommit_memory=1` will only make malloc less conservative, which is irrelevant to your failure.)
- `vm.min_free_kbytes` — raising this (e.g., to 65536) keeps the buddy allocator from getting into a state where large contiguous allocations fail. Worth trying as a persistent tuning, not a per-run command.
- `cat /proc/pagetypeinfo` and `cat /proc/buddyinfo` will show you whether the failure is fragmentation (no order-10+ blocks) or genuine exhaustion.

### 6. llama.cpp / llama-server flags that matter on Jetson
- `--no-mmap` (env `LLAMA_ARG_NO_MMAP=1`) — forces all weights into a single contiguous CUDA buffer instead of relying on file-backed mmap that NvMap then needs to import. On Jetson this is a double-edged sword: with mmap, NvMap pins many small ranges (fragmentation-prone); without mmap, you need one very large contiguous allocation (CMA-prone). On Xavier with 8/16 GB and the CVE-affected kernel, `--no-mmap` *often* fails harder. Try **enabling** mmap (the default) and reducing `-c`.
- `-c` / `--ctx-size` — the KV cache buffer is usually the largest single allocation. Halve it before reaching for module reloads.
- `-ctk q8_0 -ctv q8_0` (or even `q4_0`) — quantized KV cache cuts that allocation by 4×–8×. This is the single biggest mitigation for the error-12 regression on Xavier.
- `-fa` / `--flash-attn` — reduces the activation buffer.
- `GGML_CUDA_ENABLE_UNIFIED_MEMORY=1` — on Jetson where CPU and GPU share LPDDR, this routes the CUDA allocator through cudaMallocManaged. It can help avoid pin-pressure on NvMap but it does not bypass the CVE guard. Worth trying.
- `CUDA_LAUNCH_BLOCKING=1` only changes synchronization semantics; no allocator side effect.
- **Build flag:** `-DGGML_CUDA_FORCE_CUBLAS=ON`. The Unsloth team documented in `unslothai/unsloth#4862` that "the default custom CUDA kernels in llama.cpp may have issues on Jetson's compute capability 8.7 with unified memory" — and the same caveat applies to sm_72 (Xavier) at varying degrees. Forcing cuBLAS yields more conservative allocations.
- **LD_LIBRARY_PATH hygiene:** unslothai/unsloth issue #4862 ("llama-server crashes on Jetson Orin: pip NVIDIA libs in LD_LIBRARY_PATH override system CUDA") documents: *"The root cause is that pip-installed NVIDIA CUDA libraries from Unsloth's Python venv are prepended to LD_LIBRARY_PATH, overriding system CUDA libs."* On Xavier always export `LD_LIBRARY_PATH=/usr/local/cuda/lib64:/usr/lib/aarch64-linux-gnu/tegra:$LD_LIBRARY_PATH` before launching to make sure llama-server links against Tegra-specific CUDA libs.

### 7. Docker isolation: yes, it helps, but not for the reason you'd expect
Running llama-server inside `--runtime=nvidia` (NOT `--gpus all` — Jetson's NVIDIA Container Runtime does not implement `--gpus`) does not give you "GPU memory isolation" in the way it does on a discrete GPU — there is no separate VRAM. **But** it gives you something almost as useful: a single container-level kill closes *all* file descriptors to `/dev/nvhost-gpu`, `/dev/nvhost-ctrl`, `/dev/nvhost-ctrl-gpu`, `/dev/nvhost-prof-gpu`, `/dev/nvhost-dbg-gpu`, and `/dev/nvmap` simultaneously. Because (as section 3 shows) handles only release when the owning fd closes, an atomic `docker stop` is the most reliable way to actually drop NvMap handles short of reloading the module. With a bare llama-server process, a slow SIGTERM, a stuck worker thread, or a held fd in a wrapping shell can leave handles orphaned — exactly your "GPU detected but allocation fails" pattern.

The `--gpus all` flag is broken on Jetson — `dusty-nv/jetson-containers` issue #1068 ("Tests will not run with --gpus=all", opened by WhoWouldaThunk on Apr 23, 2025) shows the exact error: *"Auto-detected mode as 'csv' invoking the NVIDIA Container Runtime Hook directly (e.g. specifying the docker --gpus flag) is not supported. Please use the NVIDIA Container Runtime."* The reporter confirms: *"If you remove: --gpus=all or replace it with --runtime=nvidia · Then tests will run fine."*

Use `dustynv/llama_cpp:r35.4.1` (or the JP5 equivalent) and run with `--rm` so containers are torn down on stop:
```
docker run --rm --runtime nvidia --network host \
  --device /dev/nvmap --device /dev/nvhost-ctrl \
  --device /dev/nvhost-ctrl-gpu --device /dev/nvhost-prof-gpu \
  --device /dev/nvhost-gpu --device /dev/nvhost-dbg-gpu \
  -v /path/to/models:/models \
  dustynv/llama_cpp:r35.4.1 \
  llama-server -m /models/foo.gguf -c 4096 -ngl 99 --port 8080 --host 0.0.0.0
```

### 8. The R35.6.3 regression timeline and what the community has actually done
Reading the *Updating Orin Nano breaks Ollama*, *Cuda0 Buffer Error*, *Memory issue after latest kernel update*, *NvMapMemHandleAlloc error 12 in Llama.cpp model load*, and the multi-hundred-post *unable to allocate CUDA0 buffer* thread (forums.developer.nvidia.com/t/unable-to-allocate-cuda0-buffer-after-updating-ubuntu-packages/347862, which had grown to at least 256 posts by mid-Jan 2026), the community converged on three workarounds and no in-kernel reset:

1. `sudo apt-mark hold nvidia-l4t-display-kernel nvidia-l4t-kernel nvidia-l4t-kernel-dtbs nvidia-l4t-kernel-headers nvidia-l4t-kernel-oot-headers nvidia-l4t-kernel-oot-modules` and stay on R35.6.2 (AastaLLL, Dec 11, 2025).
2. Rebuild a custom kernel by applying the 5 diff zips to `kernel/kernel-5.10` and `kernel/nvidia`, then reflash the rebuilt `nvgpu.ko`, `Image`, and `tegra194-*.dtb` via `l4t_initrd_flash.sh` (AastaLLL, Jan 15, 2026 post #26).
3. Upgrade to R35.6.4 / JetPack 5.1.6, the production rollup.

**No published community script** (jetson-stats, dustynv/jetson-containers, JetsonHacks, OE4T/meta-tegra) reliably resets the post-CVE NvMap allocator without a reboot. Multiple users on that 256-post thread explicitly tried rmmod/modprobe loops and reported the same intermittent failure you describe — which matches your observation that "rmmod/modprobe does not fully restore the boot-time VMM configuration." That is because the CVE guard remains active after reload; what looks like an incomplete VMM reset is actually the new accounting policy firing on the next large allocation.

## Details

### Recommended cleanup script (replaces and extends your current 4 steps)
```
#!/bin/bash
# jetson-gpu-reset.sh — run between llama-server invocations on Xavier (JP 5.x)

set -e

# 0) Make sure the LLM server (and any wrappers) are actually gone
sudo pkill -9 -f llama-server || true
sudo pkill -9 -f llama-cli || true
sleep 1

# 1) Kill anything still holding GPU handles
for d in /dev/nvhost-gpu /dev/nvhost-ctrl-gpu /dev/nvhost-prof-gpu \
         /dev/nvhost-dbg-gpu /dev/nvhost-ctrl /dev/nvmap; do
    sudo fuser -k "$d" 2>/dev/null || true
done
sleep 1

# 2) Diagnostic: show orphan NvMap handles BEFORE cleanup
echo "=== orphan NvMap handles before cleanup ==="
sudo cat /sys/kernel/debug/nvmap/iovmm/orphan_handles 2>/dev/null | head -20
sudo cat /sys/kernel/debug/nvmap/iovmm/clients      2>/dev/null

# 3) Flush page cache, slab, dentries; then run synchronous compaction
sudo sync
echo 3 | sudo tee /proc/sys/vm/drop_caches
echo 1 | sudo tee /proc/sys/vm/compact_memory

# 4) Check buddy fragmentation
echo "=== /proc/buddyinfo ==="
cat /proc/buddyinfo
echo "=== CMA ==="
grep -E 'Cma|MemFree|MemAvailable' /proc/meminfo

# 5) Reset performance state (no allocator side-effect, but stabilizes EMC)
sudo nvpmodel -m 0
sudo jetson_clocks

# 6) Only attempt module reload as last resort, in dependency order.
#    Skip if step 5 + verification (step 7) succeeds.
if [ "$1" = "--reload-modules" ]; then
    sudo systemctl stop nvargus-daemon || true
    sudo rmmod tegra_udrm 2>/dev/null || true
    sudo rmmod nvgpu     2>/dev/null || true
    sudo modprobe nvgpu
    sudo modprobe tegra_udrm
    sudo systemctl start nvargus-daemon || true
fi

# 7) Verify GPU is alive and can allocate
llama-cli --list-devices || true
echo "=== orphan NvMap handles after cleanup ==="
sudo cat /sys/kernel/debug/nvmap/iovmm/orphan_handles 2>/dev/null | head -20
```

### Why your randomness is occurring
- If `/sys/kernel/debug/nvmap/iovmm/orphan_handles` is non-empty *after* fuser/rmmod/modprobe, you have a leaked fd. Find the holder with `sudo lsof /dev/nvmap | head` and `sudo lsof /dev/nvhost-gpu | head` — likely a shell wrapper, tmux pane, or zombie systemd-spawned worker.
- If `/proc/buddyinfo` shows no order-10 (4 MB) or higher blocks, you have buddy-allocator fragmentation; `echo 1 > /proc/sys/vm/compact_memory` is the correct response.
- If CMA in `/proc/meminfo` is exhausted (CmaFree close to zero), no userspace command will help short of reload-or-reboot — you need to grow `cma=` on the kernel cmdline (default on Jetson AGX Xavier is 64 MiB per NVIDIA Developer Forums thread "How to use CMA on Jetson AGX Xavier Industrial", tachibana.takeshi Feb 12, 2025: *"after booting and checking dmesg, it seems that only 64MiB is reserved"*; bump to 512 MB by editing `/boot/extlinux/extlinux.conf` APPEND to add `cma=512M` and reboot once).
- If `lfb` in `tegrastats` shows only `1x4MB` (one 4-MB largest free block), that's the same fragmentation signal — confirmed by users in the *Cuda0 Buffer Error* thread.

### Per-run llama-server flags for Xavier
```
LD_LIBRARY_PATH=/usr/local/cuda/lib64:/usr/lib/aarch64-linux-gnu/tegra:$LD_LIBRARY_PATH \
GGML_CUDA_ENABLE_UNIFIED_MEMORY=1 \
llama-server \
  -m /models/your-model-Q4_K_M.gguf \
  -ngl 99 \
  -c 2048 \
  -ctk q8_0 -ctv q8_0 \
  -fa \
  -t 6 \
  --host 0.0.0.0 --port 8080
```
Avoid `--no-mmap` on the CVE-affected kernel — it makes the failure more frequent because one large allocation has to fit one large contiguous chunk.

### Build llama.cpp for Xavier (sm_72)
```
cmake -B build -DGGML_CUDA=ON -DGGML_CUDA_FORCE_CUBLAS=ON \
  -DCMAKE_CUDA_ARCHITECTURES=72 -DGGML_NATIVE=ON
cmake --build build --config Release -j 4
```
`GGML_CUDA_FORCE_CUBLAS=ON` is recommended for sm_72/sm_87 Jetson per the Unsloth Jetson bug report.

## Recommendations

**Stage 1 — Stop the bleeding (do this today)**
1. Run `cat /etc/nv_tegra_release` and `uname -r`. If you see R35.6.3 or later, you are on the CVE-regressed kernel. Either:
   - Upgrade to **L4T R35.6.4 (JetPack 5.1.6)** — production fix. Download from `https://developer.nvidia.com/embedded/jetson-linux-r3564` and flash with `l4t_initrd_flash.sh`. This is the cleanest path.
   - Or downgrade by holding kernel packages: `sudo apt-mark hold nvidia-l4t-display-kernel nvidia-l4t-kernel nvidia-l4t-kernel-dtbs nvidia-l4t-kernel-headers nvidia-l4t-kernel-oot-headers nvidia-l4t-kernel-oot-modules` and reflash R35.6.2.

   **Threshold to skip this stage:** if your current kernel is R35.6.2 or earlier, skip — your problem is fragmentation/leaked-fd, not the CVE guard.

**Stage 2 — Replace your cleanup script (do this regardless)**
2. Use the script above instead of the 4-step script. The two crucial additions are `echo 1 > /proc/sys/vm/compact_memory` and the `lsof /dev/nvmap` diagnostic.
3. Make `nvpmodel -m 0 && jetson_clocks` persistent (add to `/etc/rc.local` or a systemd unit).
4. Use quantized KV cache (`-ctk q8_0 -ctv q8_0`) and lowered `-c`. This is the largest single contiguous allocation and the cheapest mitigation.

**Stage 3 — Containerize for atomic teardown (do this within a week)**
5. Move llama-server into a `--runtime nvidia` Docker container (`dustynv/llama_cpp:r35.4.1` or build your own). Use `--rm` so `docker stop` releases every fd to `/dev/nvmap` and `/dev/nvhost-*` atomically. This is the closest thing to a "GPU process cgroup" on Jetson.

**Stage 4 — If the failure persists after Stages 1–3 (do this last)**
6. Bump `cma=` on the kernel cmdline: edit `/boot/extlinux/extlinux.conf` to add `cma=512M` in the APPEND line. Reboot once. This gives NvMap a much larger contiguous reservation that survives fragmentation.
7. Raise `vm.min_free_kbytes=65536` (persist via `/etc/sysctl.d/`).
8. Only as a last resort, fall back to the full rmmod/modprobe sequence in section 2 — and run it from a TTY (`Ctrl+Alt+F2`) with the display server stopped (`sudo systemctl stop gdm3`). Note that NVIDIA does not officially support this and warns it can fail to rebind.

**Benchmarks that should make you change behavior**
- If `cat /sys/kernel/debug/nvmap/iovmm/orphan_handles` shows entries after `pkill -9 llama-server`: you have an fd leak — find with `lsof`, fix the wrapper, do not blindly reload nvgpu.
- If `cat /proc/buddyinfo` shows zero blocks at order ≥ 10 right before failure: it's fragmentation — `compact_memory` is the fix.
- If `CmaFree` < 64 MB: bump `cma=`, reboot once.
- If allocation fails with full CMA, full buddy, and zero orphans: you're hitting the CVE guard. Apply patches or upgrade to R35.6.4.

## Caveats

- **NVIDIA explicitly does not support unloading nvgpu.** Quote from ShaneCCC: *"It could cause boot failed if remove nvgpu module."* Your script's step 3 works most of the time *because* nothing else is using nvgpu when you run it, but it can leave the driver in a partially-bound state if any client (X server, nvargus-daemon, a stuck container) was holding a handle. This is the single most likely cause of the residual ~5-10% failure rate you see *after* an apparently successful module reload.
- **There is no JetPack-5-supported in-kernel API to flush orphan NvMap handles.** The debugfs nodes are diagnostic-only. Anyone claiming a magic ioctl is wrong; the only release path is fd close or module unload.
- **`--gpus all` vs `--runtime nvidia`:** the standard NVIDIA Container Toolkit `--gpus` flag is **not supported on Jetson**; you must use `--runtime nvidia` (see `dusty-nv/jetson-containers` issue #1068). Several blog posts incorrectly recommend `--gpus all` for Jetson — they're wrong.
- **Volta sm_72 / Xavier is being phased out.** JetPack 6 has different drivers (R36 family) and dropped Xavier support to focus on Orin/Thor. Long-term, plan a migration to Orin if your workload is LLM-heavy; the unified-memory and nvmap improvements in R36 are substantial.
- **The CVE-regression timeline is still evolving.** As of the writing of this report (May 2026) the R35.6.4 rollup is the documented fix for JetPack 5; the parallel R36 fix is in JetPack 6.2.2/r36.5 per AastaLLL. Confirm before flashing by reading the R35.6.4 release notes — NVIDIA has been known to ship rollups with regressions of their own.
- **`vm.overcommit_memory` / `vm.overcommit_ratio` will not help.** They affect malloc, not NvMap. Don't waste time tuning them for this problem.
- **Conflicting reports on `--no-mmap`:** some users report it helps (it produces a single large allocation the allocator can plan around), others report it makes the CVE guard fire more often. The honest answer is "test both with your model and your kernel"; the report's recommendation (use mmap, reduce ctx, quantize KV) is the safer default on R35.6.3 specifically.
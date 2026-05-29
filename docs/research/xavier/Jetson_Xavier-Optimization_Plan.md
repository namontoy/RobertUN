# Jetson Xavier NX 8GB on JetPack 5.1.x — Optimization Plan for Headless LLM + ROS 2

**Bottom line:** Boot headless (multi-user target), put the module in **NVPModel 8 (MODE_20W_6CORE)** + `jetson_clocks`, build llama.cpp from source with `-DGGML_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=72`, and run **Qwen2.5-3B-Instruct Q4_K_M** (or **Llama-3.2-1B Q4_K_M** for snappier responses) under `llama-server` with flash attention. That setup gives a quantized LLM REST endpoint your ROS 2 nodes can hit over LAN, with the whole 8 GB unified memory available for the model + KV cache instead of being lost to GNOME.

## TL;DR

- **Power & headless mode**: Use `sudo nvpmodel -m 8 && sudo jetson_clocks` (20 W, all 6 Carmel cores, 1.1 GHz GPU, 1866 MHz EMC). Disable the desktop with `sudo systemctl set-default multi-user.target` to free ~1.0–1.5 GB of RAM and one CPU core's worth of background load.
- **Slim the install**: Remove docs/samples/VPI samples/CUDA samples and (optionally) `ubuntu-desktop` + LibreOffice/Thunderbird via `apt purge`. Keep `nvidia-jetpack-runtime`, CUDA 11.4 toolkit, and ROS 2 — drop dev headers only after llama.cpp is built.
- **llama.cpp**: `cmake -B build -DGGML_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=72 -DCMAKE_BUILD_TYPE=Release && cmake --build build -j6`. Expected throughput on Xavier NX in 20W/6-core mode: roughly **15–22 tok/s generation for 1B Q4_K_M** and **6–10 tok/s for 3B Q4_K_M** (extrapolated — no published Xavier-NX-specific benchmark; verify with `llama-bench`). Serve via `llama-server --host 0.0.0.0 --port 8080 -ngl 99 -fa on`.

## Key Findings

1. **The Xavier NX exposes 9 NVPModel modes (IDs 0–8)**, not the 3 power "envelopes" the marketing pages imply. The right one for sustained LLM inference is **ID 8 (MODE_20W_6CORE)**: 6 CPU cores @ 1.4 GHz, GPU @ 1100 MHz, EMC @ 1866 MHz. NVIDIA's r35.x developer guide lists these explicitly.
2. **The default mode out of the box is MODE_10W_DESKTOP (ID 5)** — 4 cores, GPU clipped to 510 MHz, EMC 1600 MHz. Leaving this on costs you roughly half the GPU clock and substantial memory bandwidth; change it on first boot.
3. **The Xavier NX GPU is sm_72 (Volta, compute capability 7.2)** with 384 CUDA cores + 48 Tensor cores. llama.cpp builds cleanly on JetPack 5.1.x with CUDA 11.4 once `CMAKE_CUDA_ARCHITECTURES=72` is set. The legacy `LLAMA_CUBLAS` flag was first deprecated (non-fatal warning) and then elevated to a CMake FATAL_ERROR via `llama_option_depr(FATAL_ERROR LLAMA_CUBLAS GGML_CUDA)` no later than July 2024 — you must now use `-DGGML_CUDA=ON`.
4. **Memory is unified** between CPU and GPU on Jetson. Every megabyte burned on GNOME, snap, LibreOffice, Thunderbird, and CUDA samples is a megabyte unavailable for KV cache. With Xavier NX 8 GB the realistic ceiling is a **3B–4B parameter model at Q4 quantization with a 32k context** (flash attention required).
5. **No published llama.cpp benchmark numbers exist for Xavier NX specifically** for 1B–3B Q4 GGUF models. Numbers in this report are extrapolated from Jetson Nano (Tegra X1, theoretical peak 25.6 GB/s but kreier/llama.cpp-jetson measured a realistic ~6 GB/s effective memory bandwidth in practice) and AGX Xavier (137 GB/s over its 256-bit LPDDR4x interface at 2133 MHz, per the NVIDIA Jetson AGX Xavier datasheet) results; the Xavier NX sits in between, delivering "over 59.7 GB/s of memory bandwidth" per NVIDIA's official Jetson Xavier NX product page. Plan to measure on your unit with `llama-bench`.

---

## Details

### 1. NVPModel Power Modes for Xavier NX

The Jetson Xavier NX module (T194 SoC, P3668 carrier) defines **9 modes** in `/etc/nvpmodel/nvpmodel_t194_p3668.conf`. Table from NVIDIA's L4T r35.4.1 Developer Guide (the JetPack 5.1.x BSP):

| ID | Name | Budget | Online CPUs | CPU max | GPU max | EMC max | DLA max |
|----|------|--------|-------------|---------|---------|---------|---------|
| 0  | MODE_15W_2CORE   | 15 W | 2 | 1900 MHz | 1100 MHz | 1600 MHz | 1100 MHz |
| 1  | MODE_15W_4CORE   | 15 W | 4 | 1400 MHz | 1100 MHz | 1600 MHz | 1100 MHz |
| 2  | MODE_15W_6CORE   | 15 W | 6 | 1400 MHz | 1100 MHz | 1600 MHz | 1100 MHz |
| 3  | MODE_10W_2CORE   | 10 W | 2 | 1500 MHz |  800 MHz | 1600 MHz |  900 MHz |
| 4  | MODE_10W_4CORE   | 10 W | 4 | 1200 MHz |  800 MHz | 1600 MHz |  900 MHz |
| 5  | MODE_10W_DESKTOP | 10 W | 4 | 1900 MHz |  510 MHz | 1600 MHz |  900 MHz |
| 6  | MODE_20W_2CORE   | 20 W | 2 | 1900 MHz | 1100 MHz | 1866 MHz | 1100 MHz |
| 7  | MODE_20W_4CORE   | 20 W | 4 | 1400 MHz | 1100 MHz | 1866 MHz | 1100 MHz |
| 8  | MODE_20W_6CORE   | 20 W | 6 | 1400 MHz | 1100 MHz | 1866 MHz | 1100 MHz |

Mode 5 (MODE_10W_DESKTOP) is the factory default — note its strangely low 510 MHz GPU clock; it's tuned for an interactive desktop, not inference.

**Why MODE_20W_6CORE for LLM inference with llama.cpp:** Token-generation throughput in llama.cpp at batch=1 is bottlenecked by memory bandwidth on Jetson (the GPU has to stream the full weight matrix per token). Mode 8 gives you the highest EMC clock (1866 MHz, ~59.7 GB/s peak) AND the max GPU clock (1100 MHz) AND keeps all 6 cores online for the CPU-side scheduling / sampling / tokenization / ROS 2 nodes. Modes 0/1/2 (15 W) use the same GPU and EMC but with a lower 1600 MHz EMC limit, which directly slows tg. The 2-core modes are pointless here — llama.cpp uses CPU threads for sampling and host-side work, and you also need cores for ROS 2.

**Commands (run on Xavier NX over SSH):**

```bash
# Inspect available modes
sudo nvpmodel -p --verbose | grep POWER_MODEL

# Show current mode
sudo nvpmodel -q

# Switch to MODE_20W_6CORE (recommended for LLM + ROS 2)
sudo nvpmodel -m 8

# Lock all clocks to their max for the current mode and disable DVFS
sudo jetson_clocks

# Optional: store current clock state so you can restore later
sudo jetson_clocks --store ~/jetson_clocks_state.txt

# Optional: set the fan profile to "cool" (mode 8 will run hot)
sudo mkdir -p /etc/nvfancontrol.conf.d
sudo bash -c 'cat > /etc/nvfancontrol.conf.d/99-cool.conf <<EOF
FAN_DEFAULT_PROFILE cool
EOF'
sudo systemctl restart nvfancontrol

# Verify
sudo nvpmodel -q
sudo tegrastats --interval 1000
```

`nvpmodel` settings persist across reboots; `jetson_clocks` does **not** — re-run it on boot via a systemd unit if you want it permanent:

```bash
sudo tee /etc/systemd/system/jetson_clocks.service <<'EOF'
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
sudo systemctl enable jetson_clocks.service
```

### 2. Disabling the Graphical Desktop

On a default JetPack 5.1.x install, the GNOME desktop, gdm3, GNOME Shell, tracker, and related services use **approximately 1.0–1.5 GB of RAM at idle** on Xavier NX (multiple Jetson developer-forum threads report 1.4–1.5 GB idle RAM use with GUI + NoMachine; pure CLI boot reduces that to ~600–800 MB). For LLM inference that 1 GB matters: it's the difference between fitting Qwen2.5-3B-Instruct Q4_K_M with a 32k context + KV cache and OOM-ing.

**Make headless permanent and switch immediately without rebooting:**

```bash
# Persist headless boot across reboots
sudo systemctl set-default multi-user.target

# Verify (should print: multi-user.target)
systemctl get-default

# Switch right now without rebooting (kills the X session)
sudo systemctl isolate multi-user.target

# Stop and disable the display manager so it doesn't restart
sudo systemctl stop gdm3
sudo systemctl disable gdm3
```

To temporarily bring the desktop back for debugging:

```bash
sudo systemctl isolate graphical.target          # one-off
sudo systemctl set-default graphical.target      # permanent rollback
```

**Benefit for AI inference workloads**: ~1 GB RAM freed (more KV-cache budget → longer context window or larger model), one CPU core no longer pinned by GNOME Shell + tracker-miner, lower idle wall-power (~1–2 W less), and `sudo tegrastats` shows a far flatter CPU baseline so you can clearly see whether llama.cpp is GPU- or CPU-bound.

### 3. Removing Unnecessary Packages

Be conservative. The target use case is **headless LLM + ROS 2**, so you want to keep: the L4T BSP (`nvidia-l4t-*`), CUDA 11.4 runtime, cuBLAS, cuDNN runtime (not dev), the nvpmodel/jetson_clocks/tegrastats utilities, and obviously ROS 2 Foxy/Humble. You can safely drop documentation, samples, multimedia samples, VisionWorks (deprecated anyway), DeepStream samples, and — once llama.cpp is built — most dev headers.

**Step A — purge documentation and sample packages (always safe):**

```bash
# Identify candidate packages first
dpkg-query -Wf '${Installed-Size;8} KiB \t${Package;-40}\t${binary:Summary}\n' \
  | grep -E '(sample|doc|demos)' | sort -h

# Remove typical JetPack 5.1.x docs/samples
sudo apt-get purge -y \
  cuda-documentation-11-4 cuda-samples-11-4 \
  libnvinfer-samples libnvinfer-doc \
  libcudnn8-samples \
  vpi2-samples vpi2-demos \
  libnpp-doc-11-4 libcublas-doc-11-4 \
  nsight-systems-* nsight-compute-* \
  libvisionworks-samples libvisionworks-tracking libvisionworks-sfm 2>/dev/null
sudo apt autoremove -y
sudo apt clean
```

**Step B — purge desktop bloat (only after you've confirmed SSH/networking works):**

```bash
# Productivity apps that ship by default
sudo apt-get purge -y \
  libreoffice* thunderbird* rhythmbox* aisleriot \
  gnome-mahjongg gnome-mines gnome-sudoku gnome-todo \
  shotwell cheese remmina deja-dup transmission-common

# Optional: purge the full desktop stack. ONLY do this AFTER:
#   (1) you set multi-user.target as default
#   (2) you confirmed you can SSH in
#   (3) network-manager is still installed (reinstall it explicitly)
sudo apt-get install -y network-manager
sudo apt-get purge -y ubuntu-desktop gnome-shell gdm3 \
                      gnome-session gnome-terminal \
                      nautilus* evolution* zeitgeist* \
                      snapd
sudo apt autoremove --purge -y
```

**Do NOT remove** (you'll regret it):

- `nvidia-l4t-*` — kernel/BSP, removal bricks boot
- `nvidia-l4t-bootloader` — same
- `cuda-toolkit-11-4` — needed to build llama.cpp; remove only AFTER builds are done if disk is tight
- `libcudnn8`, `libcublas-11-4`, `libcurand-11-4` — runtime libs llama.cpp links against
- `nvidia-container-runtime` if you plan to use jetson-containers
- `network-manager`, `openssh-server` — headless lifeline
- `ros-humble-*` or `ros-foxy-*`

**Step C — after llama.cpp is built, optionally strip dev headers:**

```bash
sudo apt-get purge -y \
  cuda-cudart-dev-11-4 cuda-nvcc-11-4 \
  libcudnn8-dev libcublas-dev-11-4 \
  libnvinfer-dev libnvinfer-plugin-dev libnvonnxparsers-dev \
  python3-libnvinfer-dev
```

Only do step C if you don't intend to recompile llama.cpp on-device. Skip it if disk space isn't tight.

### 4. Installing and Running llama.cpp

**Prerequisites** (assumes JetPack 5.1.x already installed; CUDA 11.4 lives at `/usr/local/cuda-11.4`):

```bash
sudo apt-get update
sudo apt-get install -y build-essential git cmake ccache \
                        libcurl4-openssl-dev pkg-config \
                        python3-pip
# Confirm CUDA is on PATH
export PATH=/usr/local/cuda-11.4/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-11.4/lib64:$LD_LIBRARY_PATH
echo 'export PATH=/usr/local/cuda-11.4/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda-11.4/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
nvcc --version  # should print 11.4
```

You may also need a newer CMake than the Ubuntu 20.04 default (3.16). llama.cpp builds need CMake ≥ 3.18:

```bash
# If `cmake --version` < 3.18, install via pip:
pip3 install --user "cmake>=3.27"
export PATH=$HOME/.local/bin:$PATH
```

**Clone and build:**

```bash
cd ~
git clone https://github.com/ggml-org/llama.cpp.git
cd llama.cpp

# Optional: pin a known-good version. Master is generally fine, but for
# reproducibility on a ROS 2 robot, pin a tag.
# git checkout b4174   # December 2024, referenced as Xavier NX-compatible in GitHub discussion #10953

# Configure: enable CUDA, target Volta (sm_72), Release build, server + CLI
cmake -B build \
  -DGGML_CUDA=ON \
  -DCMAKE_CUDA_ARCHITECTURES=72 \
  -DCMAKE_BUILD_TYPE=Release \
  -DLLAMA_CURL=ON \
  -DGGML_NATIVE=ON

# Build (6 jobs = all Xavier NX cores; expect ~15-25 min)
cmake --build build --config Release -j6
```

**Key build-flag rationale:**

- `GGML_CUDA=ON` — the current name for what used to be `LLAMA_CUBLAS=ON`. Older guides will reference the old name; with modern llama.cpp (mid-2024 onward) you must use `GGML_CUDA` or CMake will fatal-error.
- `CMAKE_CUDA_ARCHITECTURES=72` — Xavier NX is Volta sm_72 (compute capability 7.2). Without this flag, CMake's autodetection sometimes picks the wrong arch and JIT-compiles or fails outright with `Unsupported gpu architecture 'compute_80'`. This is the specific flag called out in llama.cpp GitHub Issue #4006 for Xavier NX.
- `CMAKE_BUILD_TYPE=Release` — `-O3 -DNDEBUG`. Critical: a Debug build is ~3–5× slower.
- `LLAMA_CURL=ON` — lets `llama-cli -hf <repo>` pull GGUF files directly from HuggingFace.

**Verify CUDA is detected at runtime:**

```bash
./build/bin/llama-cli --version
./build/bin/llama-cli --list-devices
# Expect to see something like:
# Device 0: Xavier, compute capability 7.2, VMM: yes
```

**Recommended quantized models (1B–3B GGUF Q4, fit in 8GB unified memory):**

| Model | Quant | On-disk | Approx. RAM with 8k ctx | Approx. RAM with 32k ctx + FA | Use case |
|---|---|---|---|---|---|
| `bartowski/Llama-3.2-1B-Instruct-GGUF` (Q4_K_M) | Q4_K_M | ~0.8 GB | ~1.5 GB | ~2.0 GB | Snappy chit-chat, command parsing |
| `bartowski/Llama-3.2-3B-Instruct-GGUF` (Q4_K_M) | Q4_K_M | ~2.0 GB | ~2.8 GB | ~3.5 GB (FA) | Best quality at this size |
| `bartowski/Qwen2.5-3B-Instruct-GGUF` (Q4_K_M) | Q4_K_M | ~2.0 GB | ~2.8 GB | ~3.5 GB (FA) | Strong tool/JSON use; 32k native ctx |
| `unsloth/gemma-3-1b-it-GGUF` (Q4_K_M) | Q4_K_M | ~0.8 GB | ~1.5 GB | ~2.0 GB | Latest Google small model |
| `ggml-org/SmolLM2-1.7B-Instruct-GGUF` (Q4_K_M) | Q4_K_M | ~1.0 GB | ~1.7 GB | ~2.2 GB | Lightweight reasoning |

For a ROS 2 Mars rover natural-language command interface, **Qwen2.5-3B-Instruct-Q4_K_M** is the sweet spot: it has strong instruction following, native 32k context, and produces well-formed JSON for tool-calling. If latency matters more than quality, **Llama-3.2-1B-Instruct-Q4_K_M** is roughly 2–3× faster.

**Download a model (two equivalent ways):**

```bash
# Option A: directly via huggingface-cli
pip3 install -U "huggingface_hub[cli]"
mkdir -p ~/models
huggingface-cli download bartowski/Qwen2.5-3B-Instruct-GGUF \
  Qwen2.5-3B-Instruct-Q4_K_M.gguf \
  --local-dir ~/models --local-dir-use-symlinks False

# Option B: llama-cli auto-download (with LLAMA_CURL=ON)
./build/bin/llama-cli -hf bartowski/Qwen2.5-3B-Instruct-GGUF:Q4_K_M -p "hello"
```

**Run inference from the command line:**

```bash
cd ~/llama.cpp
./build/bin/llama-cli \
  -m ~/models/Qwen2.5-3B-Instruct-Q4_K_M.gguf \
  -ngl 99 \
  -c 4096 \
  -fa on \
  -t 4 \
  --temp 0.7 \
  -p "You are the navigation AI of a Mars rover. The user says: 'drive 2 meters forward then turn left 90 degrees'. Output a JSON array of ROS 2 commands."
```

Flags worth knowing:

- `-ngl 99` — offload all layers to GPU. On Xavier NX with unified memory this is always optimal for these model sizes.
- `-c 4096` — context window. Start small; expand to 16384 or 32768 only if your prompts need it (KV cache scales linearly with context).
- `-fa on` — flash attention. Halves KV-cache memory; essential for fitting 32k context on 8 GB.
- `-t 4` — CPU threads. With 6 cores and ROS 2 also running, 4 is a good default; don't over-subscribe.

**Run as an HTTP server (REST + OpenAI-compatible API) for remote ROS 2 nodes:**

```bash
cd ~/llama.cpp
# Generate an API key once and store it
openssl rand -hex 24 > ~/.llama_api_key && chmod 600 ~/.llama_api_key

./build/bin/llama-server \
  -m ~/models/Qwen2.5-3B-Instruct-Q4_K_M.gguf \
  --host 0.0.0.0 \
  --port 8080 \
  -ngl 99 \
  -c 8192 \
  -fa on \
  -t 4 \
  --alias rover-llm \
  --api-key "$(cat ~/.llama_api_key)"
```

This exposes:

- `POST /completion` — llama.cpp-native completion endpoint
- `POST /v1/chat/completions` — OpenAI-compatible (so any OpenAI client SDK works)
- `GET /health`, `GET /metrics` — for ROS 2 health-check nodes
- `GET /` — minimal web UI for sanity checking from a laptop browser

Smoke-test from a remote machine:

```bash
curl http://<jetson-ip>:8080/v1/chat/completions \
  -H "Authorization: Bearer $(cat ~/.llama_api_key)" \
  -H "Content-Type: application/json" \
  -d '{
        "model":"rover-llm",
        "messages":[
          {"role":"system","content":"You translate user commands to ROS 2 geometry_msgs/Twist JSON."},
          {"role":"user","content":"drive forward at 0.3 m/s for 4 seconds"}
        ]
      }'
```

**Run llama-server as a systemd service** so it survives SSH disconnects and restarts on reboot:

```bash
sudo tee /etc/systemd/system/llama-server.service <<'EOF'
[Unit]
Description=llama.cpp HTTP server (Qwen2.5-3B for ROS 2 rover)
After=network-online.target jetson_clocks.service
Wants=network-online.target

[Service]
Type=simple
User=ubuntu
Environment=PATH=/usr/local/cuda-11.4/bin:/usr/bin:/bin
Environment=LD_LIBRARY_PATH=/usr/local/cuda-11.4/lib64
WorkingDirectory=/home/ubuntu/llama.cpp
ExecStart=/home/ubuntu/llama.cpp/build/bin/llama-server \
  -m /home/ubuntu/models/Qwen2.5-3B-Instruct-Q4_K_M.gguf \
  --host 0.0.0.0 --port 8080 \
  -ngl 99 -c 8192 -fa on -t 4 --alias rover-llm
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now llama-server.service
journalctl -u llama-server -f
```

**Expected performance** (Xavier NX, NVPModel 8, `jetson_clocks` on, headless, all layers on GPU):

> Note: there are **no published llama-bench numbers for Xavier NX** at the time of writing. The figures below are extrapolated from Jetson Nano (Tegra X1, theoretical peak 25.6 GB/s but in practice closer to ~6 GB/s effective) and AGX Xavier (137 GB/s) results, with Xavier NX rated at "over 59.7 GB/s" by NVIDIA and using the same Volta sm_72 GPU architecture as AGX Xavier but with fewer SMs. Treat them as planning targets — measure on your unit with `./build/bin/llama-bench -m <model.gguf> -p 512 -n 128 -ngl 99 -fa 1`.

| Model | Quant | Prompt processing (pp512) | Token generation (tg128) |
|---|---|---|---|
| TinyLlama-1.1B-Chat | Q4_K_M | ~200–350 t/s | **~15–22 t/s** |
| Llama-3.2-1B-Instruct | Q4_K_M | ~200–350 t/s | **~15–22 t/s** |
| Gemma-3-1B-it | Q4_K_M | ~150–300 t/s | **~12–18 t/s** |
| Qwen2.5-3B-Instruct | Q4_K_M | ~80–150 t/s | **~6–10 t/s** |
| Llama-3.2-3B-Instruct | Q4_K_M | ~80–150 t/s | **~6–10 t/s** |

Anchor data points behind these extrapolations: Jetson Nano (build c262bedd / b5043) hits tg128 = 6.69 ± 0.00 t/s for TinyLlama-1.1B Q4_K_M at -ngl 99 (CUDA, Tegra X1 sm_5.3), per the kreier/llama.cpp-jetson GitHub repo. Xavier NX has 2–4× more usable GPU compute and ~2.3× the rated memory bandwidth, so 15–22 t/s is the plausible range. AGX Xavier runs Llama-3.1-8B-Lexi-Uncensored_V2_Q8 at 8.40 tokens/second generation (eval time 119.04 ms/token), per a user report on the NVIDIA Developer Forums thread "Llama.cpp loading Llama 3.1 very slow on Jetson Xavier AGX" (#305327, September 2024); Xavier NX's smaller GPU and ~44% of the memory bandwidth means 3B Q4 in the 6–10 t/s range is the expected ceiling.

---

## Recommendations

**Day 1 (first 30 minutes after flashing JetPack 5.1.x):**

1. SSH in, install your public key, disable password SSH.
2. `sudo systemctl set-default multi-user.target && sudo systemctl isolate multi-user.target` — go headless immediately.
3. `sudo nvpmodel -m 8 && sudo jetson_clocks` — switch to 20W/6-core.
4. Install the `jetson_clocks.service` systemd unit so clocks lock on every boot.
5. `sudo apt update && sudo apt full-upgrade` (keep within the 5.1.x channel — do **not** dist-upgrade to JetPack 6, which drops Xavier NX support after L4T 35.x).

**Day 1, second hour:**

6. `apt purge` the docs/samples list from Section 3 Step A. Reclaim ~3–5 GB.
7. `apt purge libreoffice* thunderbird*`. Reclaim ~500 MB.
8. Decide whether to fully remove `ubuntu-desktop`. If your ROS 2 workflow truly never needs a local screen, do it; you'll reclaim another ~1.5 GB of disk and a permanent ~1 GB of RAM at boot.

**Day 1, third hour — llama.cpp:**

9. Build llama.cpp with the cmake flags from Section 4.
10. Pull Qwen2.5-3B-Instruct-Q4_K_M to `~/models/`.
11. Run `./build/bin/llama-bench -m ~/models/Qwen2.5-3B-Instruct-Q4_K_M.gguf -p 512 -n 128 -ngl 99 -fa 1` and **record the real pp/tg numbers** for your unit — this is the only way to know what you actually have.
12. Install the `llama-server.service` systemd unit and confirm a remote `curl` from your laptop hits it.

**Threshold-based escalation:**

- **If `tg128` < 5 t/s** on Qwen2.5-3B-Q4_K_M: drop to Llama-3.2-1B-Q4_K_M and accept lower quality, or verify mode 8 + `jetson_clocks` are actually active (`sudo nvpmodel -q` and `sudo cat /sys/kernel/debug/bpmp/debug/clk/gpcclk/rate`).
- **If `tg128` is in target range (6–10 t/s)** but feels slow for the rover: the bottleneck for an interactive voice/text command interface is usually prompt processing on long system prompts. Trim the system prompt to <500 tokens and use llama.cpp's prompt cache (`--prompt-cache /tmp/sysprompt.bin`) so the system prompt is only processed once.
- **If you OOM during inference**: drop context from 32k → 8k, enable `-fa on`, and use `--cache-type-k q8_0 --cache-type-v q8_0` to quantize the KV cache (cuts its memory in half with negligible quality impact).
- **If thermal throttling kicks in** (visible as `thermal@...` clamping in `tegrastats`): switch fan profile to `cool` or add an active heatsink. Mode 8 sustained will produce visible throttling on the stock passive setup in a closed enclosure.

**For the ROS 2 integration specifically:**

- Talk to `llama-server` from a ROS 2 node using the OpenAI-compatible `/v1/chat/completions` endpoint and the `openai` Python SDK pointed at `http://localhost:8080/v1` — that gives you function/tool calling out of the box. A dedicated `nl_command_node` that subscribes to a text topic, calls the LLM with a tool-use system prompt describing your `geometry_msgs/Twist`, `nav2_msgs/NavigateToPose`, etc. actions, and publishes the parsed result is the cleanest pattern.
- Pin llama-server's CPU affinity to cores 4–5 (`CPUAffinity=4 5` in the systemd unit) and reserve cores 0–3 for ROS 2. Xavier NX has 3 clusters of 2 cores; keeping each subsystem within a cluster reduces cache thrash.
- Don't expose port 8080 outside the rover's onboard LAN — even with `--api-key`, llama-server is HTTP-only.

## Caveats

- **No measured Xavier NX llama.cpp benchmarks were publicly available** at the time of this writing. The tokens-per-second figures in Section 4 are extrapolated from Jetson Nano (slower, less memory bandwidth) and AGX Xavier (much more memory bandwidth) data points. The first thing you should do after building is run `llama-bench` on your unit and replace these estimates with your real numbers.
- **JetPack 5.1.x with CUDA 11.4 is end-of-life for major LLM frameworks.** llama.cpp upstream still builds because they keep CUDA 11 support; PyTorch, vLLM, and HuggingFace TGI have largely moved on. If your rover project lifetime extends past 2026, plan a migration path — though note Xavier NX is **not** supported by JetPack 6 (Orin-only), so you're locked to the 5.x BSP.
- **The MODE_10W_DESKTOP default is a trap.** Several Jetson tutorials online will tell you the Xavier NX "has 10W and 15W modes" — actually it has 9 modes including the 20W ones added in JetPack 4.6+. Confirm with `sudo nvpmodel -p --verbose` rather than trusting blog posts.
- **`jetson_clocks` does not survive a reboot.** Without the systemd unit above, your performance numbers will drop silently after every power cycle as DVFS takes over.
- **8 GB unified memory is a hard limit.** You cannot run a 7B Q4 model usefully — it loads but performance collapses due to swap thrashing and context length is squeezed to ~512 tokens. 3B Q4 is the practical ceiling; 4B Q4 with FA is marginal.
- **Removing `ubuntu-desktop` is mostly irreversible without a reflash.** Test the headless setup with `systemctl isolate multi-user.target` for a few days first to make sure SSH/networking is rock solid before purging the GUI stack.
- **The fan profile change** assumes you're using the NVIDIA Xavier NX Developer Kit (P3509 carrier) with its bundled fan. Third-party carriers (Connect Tech, Auvidea, Seeed) may use different `nvfancontrol` configurations.
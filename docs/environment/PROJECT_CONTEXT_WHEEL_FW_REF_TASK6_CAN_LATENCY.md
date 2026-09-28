# Task 6 — CAN RX polled-vs-interrupt: latency/jitter bench procedure

Setup: daedalus + CANable V2.0 Pro (`1d50:606f`), orion not required (see
`_REF_MCU` "OPEN — polled vs interrupt-driven CAN RX" for why the decision
doesn't depend on orion being on the bus).

## Bring up `can0` on daedalus

```
sudo ip link set can0 type can bitrate 250000
sudo ip link set can0 up
```

## Firmware console

USB-TTL adapter, `/dev/ttyUSB0`, 115200 8N1 (ST-Link has no VCP — see
`_REF_DEVENV` "Debug probes"). Sent via a short pyserial script, one command
per write, echo read back:

```
heartbeat on
```

## Method

TIM7 drives the heartbeat (ID `0x500`) at a nominal 500 ms period from the
main loop (`_REF_MCU` line ~128). If the RX path (polled or interrupt) delays
the main loop under load, heartbeat inter-arrival jitter is a proxy for that
latency/coupling — this is the gap the Aug 11 throughput ramp explicitly did
not measure.

1. Start capture: `candump -tz can0 > <file>`
2. Inject synthetic load: `cangen can0 -g <ms> -I 100 -i`
   **Do not add `-x`** — it disables cangen's local loopback, so the injected
   frames stop appearing in `candump` even though they are genuinely going
   out on the wire (found 2026-09-27; cost one debug cycle).
3. Stop capture, extract ID `500` timestamps, compute
   `max(abs(interval_ms - 500))` over the run.

### Motor-loaded runs

Before starting the capture, hold at the target duty for ≥10 s:

```
vel off
drv enable
drv clearfault
drv duty <pct>
```

After the capture:

```
drv duty 0
drv coast
```

## Results

| Date | cangen `-g` | Actual fps | Motor | n hb | mean interval (ms) | max \|dev\| from 500ms (ms) | bus errors |
|---|---|---|---|---|---|---|---|
| 2026-09-27 | none (baseline) | 0 | off | 24 | 500.29 | 0.43 | 0 |
| 2026-09-27 | 5 | ~200 | off | 36 | 500.29 | 0.58 | 0 |
| 2026-09-27 | 2 | ~500 | 18% duty | 48 | 500.29 | 0.60 | 0 |
| 2026-09-27 | 1 | ~925 | 18% duty | 43 | 500.29 | 0.70 | 0 |
| 2026-09-27 | 0.45 | ~1902 | 18% duty | 59 | 500.29 | 0.72 | 0 |

`tec`/`rec`/`lec` read `0`/`0`/`none` on the console throughout every run.
Interface-level RX errors/dropped stayed at 0 (`ip -s link show can0`).

Reached the Aug 11 saturation point (~1858 f/s, run above at ~1902 actual):
max deviation still only 0.72 ms, barely above the 1000 fps point (0.70 ms)
and the no-load floor (0.43 ms). No breakover found yet at any load or duty
tested. The deviations are small enough that host/USB timestamp noise in
`candump` itself may be comparable to the effect being measured — this method
has not yet shown it can separate MCU-side latency from capture-side jitter.

Raw captures: `firmware/RobertUN_ModuleNode/tools/bench/runs/can_latency/*.log`
(local-only, git-ignored, per repo `.gitignore`).

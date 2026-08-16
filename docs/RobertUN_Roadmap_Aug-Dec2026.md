# RobertUN Roadmap — Aug 2 → Dec 10, 2026

**Budget:** 19 weeks · ~15h/week (2h weeknights + 5h weekend) · ~285 total hours
**Target:** Dec 10 — working demo: 6-module rover (4 steering corners + 2 center drive modules) steers correctly via distributed CAN control, perception node live, shown in Isaac Sim + on hardware.

**Reference:** CAN bus hardware decisions (J17 pinout, SN65HVD230 transceiver, 250 kbps bitrate, Bulgin connectors, linear-backbone topology, CANopen protocol choice) are already fully documented in `PROJECT_CONTEXT.md` — check there before re-deriving anything CAN-related.

**Architecture (confirmed):**
```
Orion (Jetson, ROS 2) --raw CAN bus (SocketCAN)--
   |-- STM32F446RE x4 (corners): CAN node + UART->MKS SERVO42C (steering) + encoder PID (drive)
   \-- STM32F446RE x2 (centers): CAN node + encoder PID (drive only, no steering)
```

**Debug tooling (not part of the permanent architecture):** an MKS CANable V2.0 Pro — *flashed to candleLight firmware Aug 6, now a native SocketCAN `can0` on daedalus via `gs_usb`* — plugged into a laptop (daedalus) running SocketCAN or `python-can`, sits on the same physical bus during Phase 1. It is not an autonomous node — it needs the host computer driving it — but the bus can't tell the difference between its frames and a real STM32 node's frames. Two uses: (1) a true two-device electrical bus test in W1, catching wiring/termination problems before firmware exists; (2) an independent sniffer through W2–W8, so a misbehaving node can be diagnosed by watching the wire directly instead of only through Orion's ROS 2 stack. It can also simulate a corner node's traffic via a `python-can` script if STM32 firmware (W4–W5, encoder PID) runs long and the Orion-side ROS 2/CAN interface needs to be developed against something in the meantime.

> This is a living document. Update it weekly — check off what's done, adjust what's not.
> **Checkpoints at end of each phase are now hard gates** (see note below) — Phase 4 has almost no slack left, so phases 1-3 must land on time.

---

## STATUS — last updated Aug 16, 2026

**Phase 1: W1 ✅ complete (Aug 6). W2 ✅ complete (Aug 10, six days early).
W3 ✅ complete (Aug 13) — closed before its nominal Aug 17 start date even
opened. W4 is the next week to open.**

| Week | Status |
|---|---|
| W1 — CAN bus bring-up | ✅ **DONE** Aug 6 — every acceptance criterion met |
| W2 — STM32 CAN bring-up | ✅ **DONE** Aug 10 — STM32 heartbeat crossing a real 250 kbps bus to Orion, confirmed independently on the CANable, zero error counters on both ends |
| W3 — MKS SERVO42C UART | ✅ **DONE** Aug 13 — STM32 commands the steering motor to a target angle, confirmed against the driver's own encoder; hardware wired and running |
| W4–W9 | ⬜ not started |

**Days banked: 10.** W3 closed Aug 13 against its nominal Aug 23 finish.

The earlier W1 and W2 banks are **superseded, not added to this figure.** W1's
three days went into working W2 early (Aug 7-9) and W2's six-day bank is where
the early W3 work (Aug 13) came from, so adding all three banks would count the
same calendar twice. The honest measure is a single number: the gap between the
last completed week's actual finish and its nominal one.

Per the standing rule these ten days widen the margin in front of the Oct 4 gate
— they do not move it earlier. W4 opens on its own terms (nominally Aug 24)
rather than being pulled forward by default.

The intent is to keep buying slack early, since the weeks most likely to
overrun (W5 encoder PID tuning, W3 MKS protocol port) are the ones where
surprises are hardest to predict. Bank time *now*, spend it *later* —
do not let an early finish become an early stop.

---

## The four phases (revised after Phase 1 scope correction)

| Phase | Weeks | Dates | Goal |
|---|---|---|---|
| **1. Steering & Ackermann core (STM32 firmware + CAN)** | W1–W9 | Aug 3 – Oct 4 | All 6 modules steer/drive correctly via distributed CAN control |
| **2. Perception** | W10–W12 | Oct 5 – Oct 25 | Terrain classification live on orion, publishing to ROS 2 |
| **3. Sim + full integration** | W13–W16 | Oct 26 – Nov 22 | Steering + perception validated together, in sim and on hardware |
| **4. Polish + rehearsal (compressed)** | W17–W18 | Nov 23 – Dec 6 | One full dry-run, fix critical issues, second abbreviated pass. Teleop station cut/optional. |
| **Event buffer** | W19 | Dec 7–10 | Final rehearsal only — no new work |

---

## Phase 1 — Steering & Ackermann Core (Aug 3 – Oct 4)

Build order: **one reference corner module fully working end-to-end, then replicate.** Don't parallelize across 6 modules until the pattern is proven on one — that multiplies every unknown by 6.

| Week | Dates | Goal (verifiable, not vague) | Done when... |
|---|---|---|---|
| ✅ **W1** | Aug 3–9 → **done Aug 6** | **(1)** ✅ Solder a pin header onto Orion's **J17** — CAN is not on the 40-pin header, it's a dedicated 4-pin header, factory-unpopulated (CAN_TX, CAN_RX, GND, 3.3V). **(2)** ✅ Set pinmux via `devmem`, run the loopback self-test to validate the software chain in isolation. ~~Remove the `mttcan` blacklist~~ — *no blacklist on this R36.5 build*. ~~TX/RX shorted on J17~~ — *see correction below*. **(3)** ✅ Wire the SN65HVD230 transceiver, bring up real `can0` at 250 kbps. **(4)** ✅ Two-node bus test: MKS CANable V2.0 Pro (via daedalus) on the same physical bus. **(5)** ✅ Make pinmux + `can0` config persistent via `robertun-can0.service` | ✅ **ALL MET.** Loopback self-test passed (Aug 4); `cansend` from daedalus seen by `candump` on Orion **and vice versa** (Aug 6); 500-frame `cangen` load test clean with `berr-counter tx 0 rx 0`; `can0` came up correctly after a full reboot with **zero manual steps** and a frame crossed immediately |
| ✅ **W2** | Aug 10–16 (**started Aug 7**) → **done Aug 10** | STM32F446RE dev environment + CAN peripheral bring-up, single node sends/receives a frame to Orion. Keep the CANable connected as an independent sniffer for this and all following weeks. | ✅ **ALL MET.** Heartbeat on ID 0x500 originated by the STM32 seen by `candump` on Orion **and** independently on the CANable — 17 consecutive frames, sequence numbers matching across all three views, `tec 0 rec 0 lec none` on the STM32 and `berr-counter tx 0 rx 0` on Orion |
| ✅ **W3** | Aug 17–23 (**started Aug 13 → done Aug 13**) | Port known UART/CRC protocol to STM32 firmware; STM32 drives the MKS SERVO42C directly (bench test, no CAN yet) | ✅ **MET Aug 13** — `mks deg 5` moved the output 4.9974 deg (422 pulses), confirmed independently by the driver's encoder; round-trip repeatability 4 encoder counts = 1/10 of a microstep |
| W4 | Aug 24–30 | Encoder + drive motor closed-loop PID on STM32 — reading, wiring, first control attempt | Encoder counts are read correctly and match physical rotation |
| W5 | Aug 31–Sep 6 | Encoder PID tuning — this is genuinely new territory, budget the full week | Motor holds/reaches a target velocity reliably |
| W6 | Sep 7–13 | Integrate one full corner node: CAN command in -> steering (UART/MKS) + drive (PID) both respond. **Carried from W3:** decide how absolute positioning works — `FD` is a relative move, but `33` counts UART-commanded pulses and can serve as the feedback path. **Carried from W2:** settle polled vs interrupt CAN RX (hybrid ISR→ring is the leading candidate) | One complete corner module works end-to-end on the bus |
| W7 | Sep 14–20 | Fit + set the module-ID DIP switches on all 6 boards, flash **the same binary** to each, wire all 6 nodes. **Carried from W3:** confirm the other three SERVO42C units are set to CR_UART / 38400 / Mstep 8 | All 6 physical nodes wired, each responds to its own CAN ID; swapping a board's DIP setting changes its identity with no reflash |
| W8 | Sep 21–27 | All 6 nodes live on one bus simultaneously; Ackermann node on Orion issues coordinated commands | Full 6-wheel rover responds correctly to a turn command |
| W9 | Sep 28–Oct 4 | Precision calibration + repeatability across the 4 steering corners; buffer | Angle error is measured, consistent, and within tolerance |

### W2 record (Aug 7-10) — closed, acceptance criterion met

**Both decisions the W2 cell named are settled.** Full detail in
`PROJECT_CONTEXT.md`.

1. **Clock source: HSE.** The WeAct STM32F446 Core Board carries a populated
   8 MHz crystal — a real crystal on the board itself, not a Nucleo's ST-LINK
   MCO, so there is no debugger dependency and nothing to remove when the board
   goes into the rover. **Verified on the oscilloscope**, MCO1 showing 8 MHz HSE
   and MCO2 the 180 MHz PLL output, then reproduced under the new toolchain.
2. **Toolchain: CubeMX → CMake + CubeCLT + VS Code**, not STM32CubeIDE. The
   deciding factor is W7: `CMakeLists.txt` diffs readably across firmware
   variants, Eclipse `.cproject` XML does not. Migration was done *now*, while
   the firmware is a blinky — the same move at W6, with UART/MKS and encoder PID
   entangled, would cost a week out of the Oct 4 gate.

**Also closed in W2, ahead of schedule: module identity.** Three DIP switches
per board give a module ID 0–5, read once at boot, from which firmware derives
both the CAN node ID and the corner/center role. `0b111` is reserved as an
invalid code so an unconfigured or disconnected board halts loudly instead of
impersonating a real module. **This makes one binary serve all six modules** —
see the revised W7 above.

**Proven Aug 9:** full build → flash → halt-at-`main` chain against the board;
compiler emitting correct hard-float Cortex-M4F code; SVD peripheral view live
for on-target register inspection. A TIM3 interrupt blinky confirms the APB1
timer clock, and **APB1 at 45 MHz is the clock that feeds bxCAN** — so the
250 kbps bit-timing values (BRP 12, BS1 12, BS2 2, SJW 2, 86.7% sample point)
rest on a hardware-validated divisor chain rather than arithmetic alone.

**Firmware complete Aug 9 (evening).** CAN1 is configured on PB9/PB8 with the
timing values above — CubeMX's own Baud Rate readout confirms 250000 — plus an
accept-all filter on bank 0, `HAL_CAN_Start()`, and a heartbeat on ID 0x500 at
the TIM3 rate. Alongside it: a non-blocking DMA console on USART1 (PA9/PA10)
and a serial command interpreter (`send`, `errors`, `stats`, `loopback`, …).
Full detail in `PROJECT_CONTEXT.md` under FIRMWARE MODULES.

**The console was not in the plan and earned its place anyway.** The probe is
an ST-Link/V2 with no VCP and the WeAct board exposes no SWO, so before this
there was no way to see inside a running node — only halt-and-inspect in the
debugger. It also gives W3 its protocol tracing for free, and `loopback on`
provides the solo self-test the W1 debrief concluded does not exist on the
SocketCAN side.

**Acceptance criterion MET, Aug 10.** The STM32 heartbeat crossed a real
250 kbps bus and was seen by two independent witnesses:

| Source | Frames observed |
|---|---|
| STM32 console | `hb 354` … `hb 370` (17) |
| Orion `candump` | seq `0x161` … `0x171` (17) |
| daedalus / CANable `candump` | seq `0x161` … `0x171` (17) |

No gaps, no duplicates, sequence numbers matching across all three views.
`tec 0 rec 0 lec none` on the STM32 and `berr-counter tx 0 rx 0` on Orion —
not one retransmission. A zero TEC is the proof the ACK came back, so the link
is bidirectional even though traffic only went one way. Bus measured **59.79R**
across CANH-CANL with power off: exactly two terminators, all three nodes
connected. Full log in `PROJECT_CONTEXT.md`.

Orion's frame timestamps also came 500.297 ms apart against a predicted TIM3
period of 500.298 ms — so APB1 = 45 MHz is now confirmed by the oscilloscope,
the blinky, *and* a stopwatch on the far side of the bus.

**The lesson that cost the most time — floating CAN_RX.** Before the
transceiver was wired, PB8 was left undriven, and three consecutive bench runs
of identical firmware failed three different ways: `tec 128 / lec ack`, then
`rec 255 / lec form`, then `lec bit-dominant` with the node cycling in and out
of BUS-OFF. All three are one root cause — an undriven CMOS input settling
differently each power-up. **The non-determinism was the diagnosis**: a driven
input cannot behave differently run to run, which ruled out firmware before any
register was examined. Operational rule for W3-W8: **do not debug CAN error
counters on a node whose transceiver is not connected and powered** — the
numbers are actively misleading, not merely unhelpful. Decode table and the
TEC/REC reading rules are in `PROJECT_CONTEXT.md`.

**Method note, again vindicated.** `loopback on` proved bit timing, filter and
FIFO *before* any wiring existed, so when the bus misbehaved the search space
was already halved. That test did not exist in W1 — the debrief concluded there
was no useful solo test on the SocketCAN side — and it is available now only
because the console was built first.

### W3 record (Aug 13) — closed, acceptance criterion met

**The bench hardware is wired and working:** STM32F446 UART4 (PA0 TX / PA1 RX,
AF8) to the SERVO42C, 38400 8N1, point-to-point. UART4 was chosen over the
PC10/PC11 alternate mapping to keep the WeAct board's SD-card pins
(PC8-PC12, PD2) free for possible later use; PC9 was released from MCO2 for the
same reason.

**Wiring is arranged to avoid ground loops.** The STM32 and the SERVO42C each
have their own supply — the driver runs from 12 V for the motor — and only the
UART signal pair plus a single ground reference cross between them. No power
rail is shared, and there is one ground path rather than several, so motor
return current cannot flow through the signal ground. This mirrors the W1 CAN
decision, where only CANH, CANL and GND cross between nodes and the transceiver
and CANable are powered separately.

*(Fill in the specific measure taken if it was more than single-point grounding
— this records the intent, which is what matters when board number four is
wired in W7.)*

**Three traps found and documented**, all in `PROJECT_CONTEXT.md`:
1. The driver **echoes every request** before replying — silently corrupts
   checksum validation if not stripped, and was never noticed in the June 8
   session because a hex terminal lets the eye skip repeated bytes.
2. **Idle-line framing does not work on this link** — the driver echoes in
   software with gaps longer than a character time, so IDLE fires mid-message.
   Cost a bring-up session.
3. **Clearing HAL UART error flags steals a byte from the DMA**, and re-arming
   unconditionally causes an error cascade — 180 callbacks across 7
   transactions, then zero on the next boot. A generic STM32 trap, not
   MKS-specific.

**Two items were re-homed rather than left blocking W3.** Neither is part of
W3's acceptance criterion, which says *bench test, single motor, no CAN*:

- Confirming the other three SERVO42C units (CR_UART / 38400 / Mstep 8) is
  **W7's** job — that is the week all six boards get wired and flashed.
- Deciding absolute positioning is **W6's** job — it only matters once a CAN
  command has to become a steering angle.

Holding a week open for work that belongs to a later week is how a schedule
starts lying about itself. W1 and W2 were both closed with items carried
forward; W3 is closed on the same terms.

### W1 debrief (Aug 6) — what this plan got wrong, and what carries into W2

Full technical detail is in `PROJECT_CONTEXT.md`. Recorded here so the roadmap
itself isn't left saying things now known to be false.

**Three assumptions in the W1 cell were wrong:**
1. *"Remove the `mttcan` blacklist"* — there is no blacklist on Orion's R36.5
   build. Check per-machine; don't assume.
2. *"Loopback self-test (TX/RX shorted on J17 — no transceiver needed)"* —
   **no such useful test exists.** SocketCAN's `loopback` flag bypasses the
   pins entirely, and with loopback off a lone node shorting its own TX to RX
   cannot self-ACK, so it error-frames forever. The pinmux is validated by
   register readback plus a real two-node test. No time should be budgeted for it.
3. *The CANable was assumed ready to use* — it shipped with **slcan** firmware
   and had to be flashed to candleLight first. Roughly an hour, unplanned.

**Constraints discovered in W1 that constrain W2 and beyond:**
- **SJW must be set explicitly on every node — at the highest value that node's
  hardware allows.** Driver defaults differ (mttcan 1, gs_usb 8) and `sjw 1`
  gives only ~250 ppm of combined oscillator tolerance. On Orion that maximum is
  16. **On the STM32 it is not** — bxCAN's SJW field is 2 bits wide, capping at
  4 tq, and must also be ≤ BS2, which makes it 2 with the chosen timing. This is
  not a downgrade: 2 of 15 tq is proportionally more resync authority than 16 of
  200. Orion remains the binding node at ±0.40% tolerance, and two crystals at
  ±30 ppm sit two orders of magnitude inside it.
- **→ Therefore: the STM32F446RE nodes must run from an HSE crystal, not the
  internal HSI RC oscillator.** HSI is ±1% over temperature and will not hold a
  250 kbps bus reliably across a rover chassis's thermal swing. It fails as
  random bit errors under load — indistinguishable from a firmware bug. This is
  a **W2 design decision that is expensive to reverse by W6**, so settle it now.
- The SN65HVD230 breakouts are in **slope-control mode** (10 kΩ RS→GND), not
  high-speed. Fine at 250 kbps; a hard constraint if the bitrate ever rises.
- **Toolchain decision for W2:** STM32CubeIDE vs. CubeMX + Makefile +
  `arm-none-eabi-gcc`. The latter fits the established CLI/tmux/SSH workflow and
  version-controls cleanly across six near-identical firmware variants — which is
  exactly the W7 replication problem.

**Method note worth keeping:** every step in W1 was verified before moving to the
next, and two failures were caught that way that would otherwise have surfaced
weeks later disguised as something else (the `sjw` default, and the systemd wait
loop that turned out to actually iterate at boot). Slower per step, faster overall.

**Checkpoint (Oct 4) — hard gate.** If steering isn't solid here, everything downstream is compromised. This is the phase where slipping is most expensive to absorb, so it's also the one with real internal buffer (W9).

---

## Phase 2 — Perception (Oct 5 – Oct 25)

The hard part (SegFormer B2 FP16 at 9.4 FPS on orion) is already benchmarked. This phase wraps it, doesn't research it.

| Week | Dates | Goal | Done when... |
|---|---|---|---|
| W10 | Oct 5–11 | Wrap SegFormer inference into a ROS 2 node skeleton on orion (subscribes to ZED RGB, runs inference) | Node runs without crashing, inference happens per frame |
| W11 | Oct 12–18 | Publish classified output to `/terrain/classified_map`, validate against ZED point cloud alignment | `ros2 topic echo /terrain/classified_map` shows sane, correctly-timed data |
| W12 | Oct 19–25 | Stress test (sustained runtime, memory leaks / frame drops), fix whatever broke | Node runs stable for 30+ min continuous |

---

## Phase 3 — Sim + Full Integration (Oct 26 – Nov 22)

| Week | Dates | Goal | Done when... |
|---|---|---|---|
| W13 | Oct 26–Nov 1 | Bring Ackermann steering node into IsaacUN sim, validate geometry in simulation | Simulated rover steers correctly in Isaac Sim |
| W14 | Nov 2–8 | Bring perception node into sim (or validate sim <-> hardware topic parity) | Same topics work whether sourced from sim or ZED hardware |
| W15 | Nov 9–15 | Full-stack integration test on hardware: steering + perception running simultaneously | No resource contention (CPU/GPU/CAN bus), both subsystems stable together |
| W16 | Nov 16–22 | Buffer — integration bugs get found here; this is the last real slack before rehearsal | Whatever broke in W15 is fixed |

**Checkpoint (Nov 22) — hard gate.** Past this point there is no more room to absorb surprises.

---

## Phase 4 — Polish + Rehearsal (compressed) (Nov 23 – Dec 6)

| Week | Dates | Goal |
|---|---|---|
| W17 | Nov 23–29 | Full demo dry-run — run the whole thing end to end, log every failure. Update `PROJECT_CONTEXT.md` alongside (no dedicated docs week anymore — do it incrementally here). |
| W18 | Nov 30–Dec 6 | Fix what broke in W17. Second, abbreviated dry-run if time allows. |

**Steam Deck teleop station: cut from the critical path.** Only pick it up if W17/W18 land early — it is the first and only thing to drop if any earlier phase runs long.

## Week 19 (Dec 7–10)
Final rehearsal only. No new features, no debugging surprises this late — if something's broken here, fall back to the last known-good state from W18.

---

## How you'll know you're "doing it correctly"

- **Weekly:** one checkbox per week. If a week's goal isn't met, it moves into that phase's buffer — it does not silently roll into the next phase.
- **Banked time is buffer, not schedule.** Finishing early does not move the
  Oct 4 and Nov 22 gates earlier — it widens the margin in front of them. Record
  days banked in the STATUS block and let them absorb the weeks that overrun.
  Weeks worth watching: **W5** (encoder PID tuning — explicitly flagged as
  genuinely new territory, and the one subsystem with no prior hardware result).
  **W3 is lower risk than it looks**: the MKS SERVO42C protocol was validated
  against a real motor on June 8, 2026 (see PROJECT_CONTEXT.md). What remains in
  W3 is the STM32-side implementation — HAL UART config, response timing and
  parsing in C. Note the UART link is point-to-point (one MCU, one driver), so
  all four SERVO42C units stay at address 0xE0 and the firmware is identical
  across corners; module identity lives in the CAN ID.
  **W7 has been substantially de-risked** by the W2 decision to put module
  identity in a 3-bit DIP switch read at boot: one binary is flashed to all six
  boards, and role (corner vs center) plus CAN ID are both derived from the
  switch. What was six firmware variants to keep in sync is now a mechanical
  fit-switches-and-flash week.
- **Hard checkpoints: Oct 4, Nov 22.** Honest go/no-go assessment. There is very little slack left downstream of these, so catching drift early is what makes the rest of the plan possible.
- **What's already done and not "pending":** torque characterization (3.5 N*m continuous, 4.2x safety margin), MKS UART/CRC protocol logic, SegFormer benchmarking (9.4 FPS), Isaac Sim/ROS 2 bridge environment. You're not starting from zero on Aug 3 — you're starting from a well-characterized base with the hardest research already done. What's left is disciplined build-and-integrate work, which is a different (and more controllable) kind of hard.

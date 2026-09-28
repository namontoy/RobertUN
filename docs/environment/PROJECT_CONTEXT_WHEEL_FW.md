# RobertUN — Wheel Controller Firmware Context

**Last updated:** 2026-09-28 — W6 bus protocol drafted twice for comparison: plain CAN (`docs/can_cmds.md`) and CANopen (`docs/canopen_cmds.md`); choice open.
**Budget:** 20 KB. Check with `wc -c` before every commit; trim if over.

> **How to use this file.** This is the hot file for the wheel-firmware track:
> read it whole at session start and after a compaction. Do **not** read the
> REF files or the LOG whole — open only the section a task needs
> (`grep -n '^#' <file>`, then read that line range). Full detail goes to the
> LOG or a REF file; this file gets one line and a pointer.
> Machines, network, ROS 2, bus-wide CAN and power live in
> `PROJECT_CONTEXT_REST.md` — a different track, not read for wheel work.

## Purpose & constraints

Firmware for the six RobertUN wheel modules: one STM32F446RE per wheel, a single
binary for all six (module ID from a 3-bit DIP switch), talking to orion over
CAN at 250 kbps. Each module drives a steering servo (MKS SERVO42C over UART)
and a brushed drive motor with encoder (DRV8874). Hard deadline: December 10
demo. Roadmap: W2 CAN ✅, W3 steering ✅, W4 drive + encoder ✅, W5 velocity
PID ✅ on the rig (only the rover τ session left), W6 CAN RX ✅ (task 6, ISR-to-ring, Sep 28), but W6's
absolute-positioning decision and one-corner-node integration are still open; W7
(six nodes wired, DIP IDs, same binary) started 09-28. Roadmap: `docs/RobertUN_Roadmap_Aug-Dec2026.md`.

## Current state

- **Toolchain:** CubeMX (CMake) + STM32CubeCLT 1.22.0 + VS Code Cortex-Debug on
  daedalus; WeAct STM32F446 Core Board V1.1. → `_REF_DEVENV`
- **CAN (W2, Aug 10):** bxCAN 250 kbps, accept-all filter, zero error counters
  against orion. RX interrupt-driven, ISR-to-ring (task 6, Sep 28). → `_REF_MCU`
- **Steering (W3, Aug 13):** SERVO42C to target angle, 1/10-microstep
  repeatability; every driver stays at `0xE0`; the driver echoes each request
  before replying. → `_REF_SERVO42C`
- **Measurement validity:** everything before Sep 20 is invalid (PMODE was
  wrong); everything from Sep 21 on is at 12 V. Pre-Sep-20 figures are history only.
- **Drive (W4 closed Sep 20):** DRV8874 in PWM mode (PMODE strapped), 20 kHz
  slow decay, rail 12.0 V at VM. Encoder on TIM2, 8403.2 counts/rev. IPROPI
  calibrated: trip compares against VREF/3 (`vref_div 3`), range ~101–1580 mA,
  VDDA 3325 mV, R_IPROPI 1465 Ω. No drive-phase reading below
  **14.5% duty**; below it the decay phase reads 0.690 × I (±4%, ≥6% duty).
  → `_REF_DRIVE`, LOG 09-26 night
- **config:** append-only log in flash sector 7, `cfg` command, int32 keys in
  milli-units. Saved on the bench board 09-26 (slot 2): trip_ma 1580,
  duty_limit 300, ramp 50/120; survived a reset. Adding a key discards it. → `_REF_LEARNINGS`, LOG 09-28 (task 18)
- **drive.c safety:** `drv timeout` command watchdog (coasts on expiry); duty
  slew limiter `drv ramp` / `drv ramp floor`, off by default (0); coast and
  brake are not ramped; `drv duty <n>p` sets per-mille. → `_REF_TASKS` task 21
- **Bench tooling:** `tools/bench/` (`node.py`, `bench.py`), `telem` `T,`
  records plus per-control-step `V,` records; profiles `sweep`, `step`,
  `stair`. Run data in `tools/bench/runs/` is **local-only and git-ignored**.
  → `_REF_DRIVE` "Bench host tooling"
- **Plant, 12 V, loaded rig (1047 g):** `rpm = 0.7993 × duty% − 2.420`
  (11–29%); two-pole step, τ_fast 0.219 s (84%) + τ_slow 2.75 s (belt);
  0.5%-step A/B/A (09-27, VM 12.02 V): breakaway CW 12.5–13.0%, CCW 10.5–11.0%;
  dropout CW 8.5–9.5%, CCW 8.0–8.5%; min speed ~4.3 rpm (6 s dwell). The
  closed-loop staircase matches the open-loop inverse to 0.39% in 10–20 rpm.
  The rig is 2.87× light on inertia, so τ on the rover will be longer.
  → `_REF_DRIVE`, LOG 09-28 (task 17)

## Active work

**Task 21 — W5 velocity PID — complete on the rig 09-27; rover items wait for the rover.** Branch `w5-velocity-pid` (on GitHub).
**Acceptance (stated 09-26, crit. 3 amended 09-27):** over ~6–20 rpm each way,
60 s hold mean error ≤ ±0.05 rpm; 0% saturated, peak ≤ 95% `vel_max`; ripple
12.0 ± 0.5/rev, sd ≤ 1.5 rpm; true ±5 rpm step: no overshoot above ripple,
rise ≤ 0.3 s (rig). **Met on the rig, 6–20 rpm.** → `_REF_TASKS` task 21

- Done: `velocity.c`/`.h`, a policy layer above `drive.c`, stepping once per
  encoder window (50 Hz at `enc window 20`); feedforward from the inverse plant;
  setpoint ramp; four named anti-windup freeze conditions.
- Done: gains as live `cfg` keys — `vel_kp` 3000, `vel_ki` 10000, `vel_kd` 0,
  `vel_ff_a` 12510, `vel_ff_b` 30, `vel_ilim` 150, `vel_max` 300,
  `vel_slew` 4000, `vel_tmo` 1000 (milli-units). Shipped at ¼ of textbook.
- Done: forward staircase 10→20 rpm, 21 min: mean error +0.0008 rpm, 0%
  saturation, peak 284/300 o/oo. The shipped gains stand.
- Done: reverse staircase −10→−20 rpm: worst error 0.006 rpm, 0% saturation.
  Asymmetry is all in the integrator (corr 0.9986); reverse inverse
  `out = 11.503|rpm| + 43.65` vs forward `12.559 rpm + 30.54`.
- Done: the ±1 rpm ripple is mechanical — exactly 12.00 events per output rev:
  the tyre's 12 tread grooves (identified 09-27).
- Done: step metric fixed — anchors at the end of the ramp, compares overshoot
  with ripple. Verdict so far: no overshoot resolvable.
- Done: A/B/A staircase (fwd/rev/fwd, back to back, VM 12.02 V): both effects
  real. Direction: reverse needs ~5.9 o/oo less output; drift: forward moved
  −3.05 o/oo in 42 min. Max error ≤0.016 rpm, 0% saturation. → LOG 09-26 evening
- Decided (09-26): no direction-dependent `ff_b` for now; the integrator absorbs
  the 5.9 o/oo reverse offset with no measurable tracking cost. Must be
  re-checked on the rover, on at least two wheels. Adding it means a new `cfg`
  key (`vel_ff_b_rev` or similar), and a new key discards the stored record.
- Done: true steps (`--slew 0`) 0→10, ±10→±15→±10 rpm: rise 0.08–0.26 s, peak ≤984 mA.
  No overshoot above ripple either way: the one reverse down-step flag did not
  recur in 3 repeats (it was a shallow tail). → LOG 09-26 evening
- Done: `overshoot_above_ripple` now tests against the settled tail's own worst
  excursion + 1 count (was 2 × sd, fired on the 12/rev dips).
- Decided (09-26): current below 14.5% duty is read in the decay phase.
  Stalled A/B/A scans 5–12%: raw = 0.690 × I (±1.5%, 17 refs), ±4% from 6%
  duty; 5% invalid (−9…−38%). Sep 21's 0.670 was edge-contaminated. → LOG 09-26 night
- Done: decay-phase sample in firmware (cfg `isense_dk` 690, `isense_dmin` 60,
  telem flag 0x20); matches iscan within 1.2%. Valid at stall only: turning,
  brake/drive = 0.40–0.46, −40% step at the 14.5% switch. → LOG 09-26 late night
- Done: supply-side DMM (500 mA range), free shaft: drive-phase sample reads
  LOW while turning — fw/DMM 0.74 at 20%, 0.83 at 15%, DMM swing ±20%. Brake
  phase is lower still. No factor changed. → LOG 09-26 late night (DMM)
- Open: steady supply reference (shunt + RC on scope, or PSU readout) before
  any turning factor; the DMM min/max is too crude.
- Open: re-measure τ on the rover before freezing gains; meter the motor
  terminals, not just VM. Same session: reverse-vs-forward offset on ≥2 wheels
  (A/B/A), to settle the `ff_b` decision.
- Note: the ripple test's reference (tail worst excursion) varies ±2 counts run
  to run; repeat a run before calling a 1–2 count flag real.
- Done (09-27): 6–10 rpm A/B/A, VM 12.02 V: worst error −0.019 rpm, 0% sat,
  peak 165/300; 12.00 ± 0.02 events/rev; sd 0.78–1.19. Ripple is a one-sided
  dip (−3.2…−4.0 rpm fwd). → LOG 09-27
- **Next step:** the rover τ session (re-set rise limit, then freeze gains).

## Next tasks (priority order)

Tasks 6, 17, 18 and 20 were closed 09-28 and are in the LOG. Numbers 1–4 are new;
the old ID is in brackets.

1. W5 velocity PID (was task 21): the open items above, in the order listed.
2. Independent ammeter on the low-duty end: reverse draws +8.1% current, and the
   low-end U-shape sits near the 145 o/oo sense floor.
3. W6 open, priority not set: decide how absolute steering positioning works
   (`FD` is a relative move; `33` counts UART-commanded pulses and can serve as feedback).
4. W6 open, priority not set: integrate one full corner node — CAN command in,
   steering (UART/MKS) and drive (PID) both respond.

## Recent progress (last ~10; everything older is only in the LOG)

- **09-28** — W6 CANopen alternative spec: `docs/canopen_cmds.md` (402 CSV drive + mfr steering, 42.6% load at 50 Hz). Plain CAN vs CANopen undecided.
- **09-28** — W6 CAN command set spec drafted: `docs/can_cmds.md` (no code). Q1 open: plain CAN vs REST's CANopen decision.
- **09-28** — Tasks 6, 17, 18, 20 closed (task 20: PMODE corrected); text moved to the LOG; open list renumbered 1–4.
- **09-28** — W6 CAN RX (task 6) closed; W6's absolute-positioning decision and one-corner-node integration stay open (user); W5 owes the rover τ session; W7 started.
- **09-28** — Task 6 closed: ISR-to-ring merged (decision: accept). Bench re-run steps 1–7 via wheel-fw scripts: 20000/20000 at saturation ±motor, overflow 156 dropped, 0 overruns; jitter ≤0.74 ms, no worse than polled (the 09-27 1.28 ms at -g 0.45 did not repeat: 0.72). Delivered-count invariant not verified. ESR snapshot fixed. → LOG.
- **09-27** — Task 6 ISR-to-ring bench on branch `ISR-to-ring`: steps 1–6 pass (20000/20000, overflow 165 dropped, 0 overruns); -g 0.45 jitter 1.28 vs 0.72 ms polled.
- **09-27** — Task 17 stiction A/B/A, 0.5% steps, 12.02 V: breakaway CW 12.5–13.0% (repeats), CCW 10.5–11.0%; dropout CW 8.5–9.5%, CCW 8.0–8.5%.
- **09-27** — 12/rev ripple source identified: the tyre's 12 tread grooves. Resolved, no firmware change.
- **09-27** — 6–10 rpm A/B/A passes: worst −0.019 rpm, 0% sat, 12.00/rev; ripple crit. amended to sd ≤ 1.5 rpm. W5 met on the rig.
- **09-26** — W5 tolerance stated: 60 s mean ≤ ±0.05 rpm, 0% sat, ripple 12/rev ≤ ±1.5 rpm, step within ripple; 10–20 rpm passes, 6–10 owed.
- **09-26** — First `cfg save` on the bench board: trip_ma 1580, duty_limit 300, ramp 50/120; survived a reset. Gains stay compiled defaults.
- **09-26** — DMM supply reference, free shaft: drive-phase sample reads low while turning (0.74 at 20%, 0.83 at 15%, ±20%); no factor change.

## Key rules (full list with evidence in `_REF_LEARNINGS`)

- Arming a control loop invalidates every stop sequence aimed at the layer below: `vel off` before `drv duty 0`.
- A metric with a rate limiter upstream measures the limiter. If it doesn't move when the gain moves, it isn't a tuning number.
- An overshoot is a maximum of a noisy signal: compare the peak with the settled ripple before blaming the gains.
- Two runs an hour apart are not an A/B test; use A/B/A.
- A limit on a magnitude must test a magnitude (`max(abs())`); keep the sign out of bounded quantities.
- Two measurements agreeing better than the estimator's resolution is a property of the instrument.
- ISR-written statics must be `volatile`; a new module doesn't inherit the discipline of the tick it hangs off.
- Adding a `cfg` key discards the stored record; read `cfg` before flashing such a build. A reflash does not erase sector 7.
- Current regulation is silent: an un-ramped start can sit on ITRIP with nothing in the telemetry showing it.
- Measurements before Sep 20 are invalid (wrong PMODE); never compare against them.
- A time constant that depends on the fit window means more than one pole.
- Place a sampling trigger relative to the END of the PWM window, not as a fraction of it.
- After any CubeMX regeneration, diff the USER CODE blocks; a `.ioc` that loads can still be invalid.
- Write the raw transcript before parsing a byte of it.
- When something is damaged, stop for the session.

## Where the details live

All in `docs/environment/`, prefixed `PROJECT_CONTEXT_WHEEL_FW`:

| Topic | File suffix |
|---|---|
| Full session history, closed tasks (6, 6b, 17, 18, 19, 20, …) | `_LOG.md` |
| Open tasks, full text (21) | `_REF_TASKS.md` |
| Motor, encoder, DRV8874, IPROPI, plant, loaded rig, bench tooling | `_REF_DRIVE.md` |
| CAN bit timing, pin allocation, timers, firmware modules | `_REF_MCU.md` |
| SERVO42C protocol, command set, echo/framing traps | `_REF_SERVO42C.md` |
| Task 6 CAN RX latency/jitter bench procedure + results | `_REF_TASK6_CAN_LATENCY.md` |
| Toolchain, board, DIP switch, debug probes, `launch.json` | `_REF_DEVENV.md` |
| Every lesson with its evidence | `_REF_LEARNINGS.md` |

## Approach & tools

- Commands in small batches (3–4), checking output before continuing; verify on
  the bench before recording a result as done.
- Branches carry code, figures and figure scripts; bench run data stays local.
- `git pull` before editing shared files; specific commit messages.
- ST-Link/V2 for flashing and debug; CANable V2.0 Pro (candleLight) on orion.

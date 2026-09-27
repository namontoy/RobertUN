# RobertUN — Wheel Controller Firmware Context

**Last updated:** 2026-09-26 — True steps: no overshoot above ripple either way; the reverse down-step flag did not recur in 3 repeats.
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
demo. Roadmap: W2 CAN ✅, W3 steering ✅, W4 drive + encoder ✅, **W5 velocity
PID (active)**; W6 and W7 follow (roadmap in `PROJECT_CONTEXT_REST.md`).

## Current state

- **Toolchain:** CubeMX (CMake) + STM32CubeCLT 1.22.0 + VS Code Cortex-Debug on
  daedalus; WeAct STM32F446 Core Board V1.1. → `_REF_DEVENV`
- **CAN (W2, Aug 10):** bxCAN 250 kbps, accept-all filter, zero error counters
  against orion. Open: polled vs interrupt RX (task 6). → `_REF_MCU`
- **Steering (W3, Aug 13):** SERVO42C to target angle, 1/10-microstep
  repeatability; every driver stays at `0xE0`; the driver echoes each request
  before replying. → `_REF_SERVO42C`
- **Drive (W4 closed Sep 20):** DRV8874 in PWM mode (PMODE strapped), 20 kHz
  slow decay, rail 12.0 V at VM. Encoder on TIM2, 8403.2 counts/rev. IPROPI
  calibrated: trip compares against VREF/3 (`vref_div 3`), range ~101–1580 mA,
  VDDA 3325 mV, R_IPROPI 1465 Ω. No synchronised current reading below
  **14.5% duty**. → `_REF_DRIVE`
- **config:** append-only log in flash sector 7, `cfg` command, int32 keys in
  milli-units. `cfg save` has **never been run** on the bench board. Adding a
  key discards the stored record. → `_REF_TASKS` task 18, `_REF_LEARNINGS`
- **drive.c safety:** `drv timeout` command watchdog (coasts on expiry); duty
  slew limiter `drv ramp` / `drv ramp floor`, off by default (0); coast and
  brake are not ramped; `drv duty <n>p` sets per-mille. → `_REF_TASKS` task 21
- **Bench tooling:** `tools/bench/` (`node.py`, `bench.py`), `telem` `T,`
  records plus per-control-step `V,` records; profiles `sweep`, `step`,
  `stair`. Run data in `tools/bench/runs/` is **local-only and git-ignored**.
  → `_REF_DRIVE` "Bench host tooling"
- **Plant, 12 V, loaded rig (1047 g):** `rpm = 0.7993 × duty% − 2.420`
  (11–29%); two-pole step, τ_fast 0.219 s (84%) + τ_slow 2.75 s (belt);
  breakaway 9–11% duty (a voltage threshold, not portable across rails). The
  closed-loop staircase matches the open-loop inverse to 0.39% in 10–20 rpm.
  The rig is 2.87× light on inertia, so τ on the rover will be longer.
  → `_REF_DRIVE`, `_REF_TASKS` task 17

## Active work

**Task 21 — W5 velocity PID.** Branch `w5-velocity-pid` (on GitHub).
**Acceptance:** on the loaded rig, commanded speed held within a stated
tolerance across the usable range, no sustained oscillation, bounded overshoot
from a step.

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
- Done: the ±1 rpm ripple is mechanical — exactly 12.00 events per output rev.
- Done: step metric fixed — anchors at the end of the ramp, compares overshoot
  with ripple. Verdict so far: no overshoot resolvable.
- Done: A/B/A staircase (fwd/rev/fwd, back to back, VM 12.02 V): both effects
  real. Direction: reverse needs ~5.9 o/oo less output; drift: forward moved
  −3.05 o/oo in 42 min. Max error ≤0.016 rpm, 0% saturation. → LOG 09-26 evening
- Open: decide whether a direction-dependent `ff_b` is worth adding; the
  integrator already absorbs the 5.9 o/oo with no tracking penalty.
- Done: true steps (`--slew 0`) 0→10, ±10→±15→±10 rpm: rise 0.08–0.26 s, peak ≤984 mA.
  No overshoot above ripple either way: the one reverse down-step flag did not
  recur in 3 repeats (it was a shallow tail). → LOG 09-26 evening
- Done: `overshoot_above_ripple` now tests against the settled tail's own worst
  excursion + 1 count (was 2 × sd, fired on the 12/rev dips).
- Open: current sensing below 14.5% duty, where the rover creeps. Options:
  decay-phase reading (lead: a reproducible 0.670 factor, needs a scan across
  several duties), free-running `Isup`, or a slower carrier. Decide before
  tuning any current loop.
- Open: re-measure τ on the rover before freezing gains; meter the motor
  terminals, not just VM.
- Note: the ripple test's reference (tail worst excursion) varies ±2 counts run
  to run; repeat a run before calling a 1–2 count flag real.
- **Next step:** decide on the direction-dependent `ff_b`, then current sensing
  below 14.5% duty.

## Next tasks (priority order)

1. Task 21 open items above, in the order listed.
2. Identify the 12-per-revolution mechanical feature (gear teeth, coupling or
   belt contact) — mechanical inspection, not telemetry.
3. Independent ammeter on the low-duty end: reverse draws +8.1% current, and the
   low-end U-shape sits near the 145 o/oo sense floor.
4. Task 17: 1%-step stiction run across 8–13% (ascending, then reversed) with
   `drv duty <n>p`; restate recorded plant figures with their rail attached.
5. Task 6: settle polled vs interrupt-driven CAN RX before W6; `cmd_errors`
   reads CAN_ESR non-atomically.
6. Task 18: exercise the corrupt-record fallback and the sector-full wrap at
   save 1025; `cfg rail_mv` once the motor terminals are metered.
7. Task 20: one `drv iscan` with `from = 0` to exercise the tick-0 cosmetic fix.
8. `cfg save` on the bench board when the ramp settings (50 / 120) should persist.

## Recent progress (last ~10; everything older is only in the LOG)

- **09-26** — True steps `--slew 0`: ≤984 mA, no overshoot above ripple (reverse flag not reproduced ×3). Ripple test fixed in `bench.py`.
- **09-26** — A/B/A staircase at VM 12.02 V: direction ~5.9 o/oo and drift −3.05 o/oo/42 min, both real; error ≤0.016 rpm, 0% sat.
- **09-26** — Reverse staircase: worst error 0.006 rpm, 0% saturation; direction asymmetry is entirely the integrator; ripple 12.00/rev both ways. A/B/A still owed.
- **09-26** — Forward staircase 10→20 rpm, 21 min: error +0.0008 rpm, 0% saturation. Step metric was measuring `vel_slew`; fixed to anchor at the ramp's end.
- **09-26** — W5 PID written (`velocity.c`), `V,` per-step telemetry and `bench.py run step`. `safe_stop()` now sends `vel off` first.
- **09-26** — Duty slew limiter in `drive.c` (off by default), verified on the rig: peak inrush 2.7× lower than an un-ramped step.
- **09-25** — Loaded-rig plant `rpm = 0.7993 d − 2.420`; two-pole τ 0.219 s + 2.75 s, checked by two independent methods.
- **09-25** — Bench tooling `tools/bench/` and `telem`; `drv timeout` watchdog; 12 V free-wheel plant re-taken, CCW +3.49%.
- **09-23** — 12 V free-wheel plant `rpm ≈ 0.83 d − 0.96`, linear 3–100% duty; breakaway 5–6%, dropout 2–3%.
- **09-21** — Task 20 bench-verified: 1268 mA measured vs 1263 predicted (+0.4%).

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
- A threshold that moves with the rail is a voltage threshold; convert, don't carry duty figures across rails.
- A time constant that depends on the fit window means more than one pole.
- Place a sampling trigger relative to the END of the PWM window, not as a fraction of it.
- After any CubeMX regeneration, diff the USER CODE blocks; a `.ioc` that loads can still be invalid.
- Write the raw transcript before parsing a byte of it.
- When something is damaged, stop for the session.

## Where the details live

All in `docs/environment/`, prefixed `PROJECT_CONTEXT_WHEEL_FW`:

| Topic | File suffix |
|---|---|
| Full session history, closed tasks (6b, 19, …) | `_LOG.md` |
| Open tasks, full text (6, 17, 18, 20, 21) | `_REF_TASKS.md` |
| Motor, encoder, DRV8874, IPROPI, plant, loaded rig, bench tooling | `_REF_DRIVE.md` |
| CAN bit timing, pin allocation, timers, firmware modules | `_REF_MCU.md` |
| SERVO42C protocol, command set, echo/framing traps | `_REF_SERVO42C.md` |
| Toolchain, board, DIP switch, debug probes, `launch.json` | `_REF_DEVENV.md` |
| Every lesson with its evidence | `_REF_LEARNINGS.md` |

## Approach & tools

- Commands in small batches (3–4), checking output before continuing; verify on
  the bench before recording a result as done.
- Branches carry code, figures and figure scripts; bench run data stays local.
- `git pull` before editing shared files; specific commit messages.
- ST-Link/V2 for flashing and debug; CANable V2.0 Pro (candleLight) on orion.

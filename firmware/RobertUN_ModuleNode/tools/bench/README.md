# Bench tooling for the wheel node

Drives experiments against a RobertUN wheel node over its USART1 console,
logs everything to a run directory, and can be checked on mid-run without
being attached to it.

It lives under the firmware tree on purpose: the parser is coupled to the
console's exact output format, so a console change and its parser change land
in the same commit.

## Requirements

**Host** — see `requirements.txt`, which carries the reasoning per line:

```sh
python3 -m pip install -r requirements.txt
```

| package | why | needed to take a run? |
|---|---|---|
| `pyserial` | the serial link to USART1 | **yes — and it is the only one** |
| `numpy` | array maths in the analysis and figure scripts | no |
| `matplotlib` | figures | no |
| `scipy` | exponential fits for the planned `coastdown` profile | not yet |

Specs are **floors, not pins**, so a distro's own packages normally satisfy
them and no virtualenv is needed. Verified on Python 3.10.12 with pyserial
3.5, numpy 2.2.6, matplotlib 3.10.8, scipy 1.15.3.

**`bench.py` and `node.py` import nothing outside the standard library except
pyserial.** The least-squares slope in `rpm_from_counts()` is hand-rolled
rather than handed to numpy, deliberately: a bench box stays minimal, and a
missing analysis library can never cost you a run. So on a machine that only
needs to *take* data:

```sh
python3 -m pip install pyserial
```

The user must be in the `dialout` group or `/dev/ttyUSB*` is unreadable:

```sh
sudo usermod -aG dialout $USER     # then log out and back in
```

**Firmware** — an image with the `telem` and `drv timeout` commands. Older
images fail pre-flight with "no telemetry received" — that means reflash, not
a bug.

**No Node.js.** Nothing here is JavaScript.

## Wiring

USART1, **115200 8N1**, on a USB-serial adapter:

| board | adapter |
|---|---|
| PA9 (TX) | RX |
| PA10 (RX) | TX |
| GND | GND |

The ST-Link/V2 used for flashing has no virtual COM port, so this is a second,
separate cable. Do not connect the adapter's Vcc.

## Use

```sh
./bench.py list                                   # profiles and recent runs
./bench.py run sweep                              # CW, 5-29% in 2% steps, 4 s/point
./bench.py run sweep --dir ccw                    # the other direction
./bench.py run step --rpm 20                      # velocity-loop step response
./bench.py status                                 # latest run's status.json
```

The port is autodetected when exactly one `/dev/ttyUSB*` or `/dev/ttyACM*` is
present; otherwise pass `--port`.

### Checking in on a run

The runner owns the port for the duration, so there is no second connection to
make. Everything is a file:

```sh
./bench.py status          # or: cat runs/<latest>/status.json
```

`status.json` is rewritten once a second and holds the current duty, speed,
current, elapsed time and the running dropped-sample count. That is the whole
"run it, come back in five minutes" story — no daemon, no IPC.

## Profiles

| profile | what it does | what it yields |
|---|---|---|
| `sweep` | duty list, dwell at each, settled speed + current | the steady-state plant table, automated |
| `step` | commands a setpoint step at the **velocity loop** and records every control step | rise time, overshoot, settling time, steady-state error — the numbers a gain choice is made from |
| `stair` | holds a series of setpoints for a long dwell each, **on one arming** | settled mean, sd and standard error per setpoint, with the term breakdown — long-run tracking, drift and where the duty ceiling starts to bind |

`coastdown`, `hold` and `stiction` are still planned.

### `sweep` options

| flag | default | |
|---|---|---|
| `--duty` | `5,7,9,11,13,15,17,19,21,23,25,27,29` | comma-separated percents, always positive. **The rover runs slow: 30% duty is the ceiling for characterisation work**, and the default walks the band in 2% steps. Anything above it is calibration only and must be asked for explicitly. |
| `--dir` | `cw` | `ccw` negates every duty |
| `--dwell` | `4.0` | seconds held at each point |
| `--settle` | `0.5` | leading fraction of each dwell discarded as transient |
| `--rate` | `50` | telemetry Hz, 1..100 |
| `--window` | `100` | `enc window`, in 1 ms ticks |
| `--max-duty` | `30` | refuses anything above it. **Raise it only for deliberate calibration** — the rover's working band tops out well below this, and above ~31% the wheel begins to bounce on the rig belt, which makes those points a property of the rig rather than of the plant. |
| `--trip` | `1580` | mA; **set at run start**, so the trip is a recorded run condition rather than whatever the board happened to boot with |

### `step` options

`sweep` drives the bridge directly; `step` drives the **velocity loop**, so it
commands rpm rather than duty and its safety ceiling is a setpoint ceiling.

| flag | default | |
|---|---|---|
| `--rpm` | `10` | setpoint stepped **to** |
| `--from` | `0` | setpoint stepped **from**. Non-zero gives a small-signal step from an already-turning wheel, which is a different plant from rest — stiction is not in it |
| `--return` | off | step back down afterwards. Not symmetry for its own sake: the feedforward applies its friction offset with the **sign of the setpoint**, so the down-step is the one place a wrong `ff_b` shows up as a different *response* rather than as a constant error |
| `--max-rpm` | `22` | refuses anything above it. This is the 30% duty ceiling pushed through the plant fit (0.7993 × 300/10 − 2.42 = 21.6 rpm), so it is the same ceiling `--max-duty` enforces, expressed in the units this profile commands. Raise it only as deliberately |
| `--window` | `20` | **not the sweep's 100.** See the warning below — this is the one default that must not be copied across |
| `--dwell` | `4.0` | seconds held after the step; the measurement window |
| `--baseline` | `1.0` | seconds at 0 rpm with the loop armed, before anything moves |
| `--pre` | `3.0` | seconds at `--from` before the step (ignored when `--from` is 0) |
| `--stop-dwell` | `2.0` | seconds after `vel stop`, for the setpoint ramp to run out before `vel off` |
| `--kp` `--ki` `--kd` | board's | `cfg vel_kp/ki/kd` in **milli-units** — `--kp 5000` is Kp = 5.0. Config is int32 only, so every gain is stored ×1000. Left unset, the board keeps what it has |
| `--ff-a` `--ff-b` | board's | `cfg vel_ff_a` (milli o/oo per rpm) and `vel_ff_b` (o/oo) — the inverse-plant feedforward |
| `--slew` | board's | `cfg vel_slew`, milli-rpm/s. **0 makes it a true step**; the shipped 4000 ramps the setpoint at 4 rpm/s, which is what you want for the rover and not what you want for a step response |

⚠️ **`enc window` must be 20, not 100, for a step.** The loop advances once per
encoder window, so window 100 is a **10 Hz control loop with 100 ms of
measurement lag** — it would dominate the very response being measured, and
gains chosen against it are gains for a different plant. `--window` therefore
has no single default: 100 for `sweep`, 20 for `step`. `step` warns above 40.

The profile also sends **`cfg ramp_pmps 0`** unconditionally. `drv ramp` and
`vel_slew` are two slew limiters in series and must not both be armed: with
drv's running, `drive_slewing()` is true almost continuously, the integrator is
frozen for essentially the whole run, and the step measures the limiter.

`vel_tmo` is set to `WATCHDOG_MS` so the loop's setpoint watchdog matches the
drive watchdog, and `dwell()` refreshes **both** on the same 0.5 s beat. This
matters: only `vel target` kicks the velocity watchdog — `drv timeout` does
nothing for it — so a dwell that forgot it would coast the wheel one second in,
in the middle of the measurement.

### `stair` options

`stair` is `step`'s long-run counterpart. Where `step` asks *how does the loop
get there*, `stair` asks *what does it do once it is there, for a long time, at
many speeds*.

```sh
./bench.py run stair                                  # +10.0 -> +20.0 rpm, 0.5 rpm steps, 60 s each (21 min)
./bench.py run stair --dir ccw                        # the same range in reverse
./bench.py run stair --lo 5 --hi 15 --hold 30         # a shorter, lower band
```

| flag | default | |
|---|---|---|
| `--lo` | `10` | first setpoint, rpm **magnitude** |
| `--hi` | `20` | last setpoint, rpm **magnitude**. Below `--lo`, the staircase walks *down* |
| `--dir` | `cw` | `ccw` makes every setpoint negative |
| `--stair-step` | `0.5` | rpm between setpoints |
| `--hold` | `60` | seconds at each setpoint. **This is the measurement** — see below |
| `--hold-settle` | `10` | seconds discarded at the head of each hold, so the ramp in from the previous setpoint is not averaged into it |
| `--abort-ma` | `1200` | ends the run if any point's peak current exceeds it. A soft guard under `--trip`, because this profile runs unattended for 20+ minutes |
| `--max-rpm` | `22` | the same magnitude ceiling `step` enforces |
| `--window` | `20` | as for `step`, and for the same reason |
| `--stop-dwell`, `--rate`, `--trip`, and the gain flags | | shared with `step` |

⚠️ **`--lo` and `--hi` are magnitudes; `--dir` carries the sign.** This is the
same split `step` uses, and it is not a style choice. Written the obvious way,
a reverse run reads `--lo -10 --hi -20` — which makes the step count negative
and yields an *empty* setpoint list, and makes a ceiling guard written as
`max(setpoints)` return −10 for a run going to −20, waving through any speed at
all. The guard in `profile_stair` tests **magnitude** for exactly that reason.

**Why a staircase and not N separate `run step` invocations.** The loop stays
**armed** across the whole sweep. Re-arming between points would reset the
integrator at every one and throw away precisely the slow drift — thermal,
friction, integrator wind — that a long run exists to capture.

**Why the long dwell buys the resolution.** At `enc window 20` one encoder
count is 0.36 rpm, which on its own cannot resolve a 0.5 rpm increment. But the
ripple decorrelates in ~0.4 s, so a 60 s dwell holds ~150 independent looks and
the standard error of the mean lands near 0.08 rpm. `sem_rpm` in `stair.csv` is
that figure, computed per point — **read it before believing any difference
between adjacent setpoints.**

The run ends itself on either guard rather than logging a note and carrying on:
the remaining points would be taken under a condition the run header does not
describe.

| column in `stair.csv` | |
|---|---|
| `sp_rpm`, `t_start_s`, `n` | the setpoint, when its hold began, and how many samples survived `--hold-settle` |
| `mean_rpm`, `sd_rpm`, `min_rpm`, `max_rpm` | the settled tail. `sd_rpm` on this rig is ~1 rpm of **mechanical** ripple at ~12 events per output revolution, not loop noise |
| `err_rpm`, `sem_rpm` | tracking error, and the uncertainty on it |
| `out_pm`, `ff_pm`, `p_pm`, `i_pm`, `d_pm` | the term breakdown, o/oo. `ff_pm` is the model; `i_pm` is **what the model missed**, and it is the most informative column here |
| `sat_frac`, `freeze_frac`, `wd_frac` | fraction of control steps saturated, integrator-frozen, or past the setpoint watchdog |
| `ma_mean`, `ma_max` | bridge current over the tail |

## The ramp is in the firmware now

`drive.c` gained a duty slew limiter on 2026-09-26. It is **off by default** and
runs per-mille on the 1 kHz tick, so it is finer and steadier than the host-side
staircase this tool has been using:

```
drv ramp <o/oo per s>      50 is 5%/s - the rate proven on the loaded rig
drv ramp floor <o/oo>      jump straight to this when leaving rest; ~120 loaded
drv ramp                   report rate, floor, target, applied, slewing?
drv duty <+/-n>p           per-mille, e.g. `drv duty 295p` (percent still works)
```

With a rate armed, `drv duty` sets a **target** and returns; the bridge arrives
over the next few hundred ms. The telemetry `duty` column reports what the bridge
is **actually running**, not the target — so a ramp appears in the data as a
ramp, which is how it gets measured. `drv` prints both.

**`drv coast` and `drv brake` are never ramped.** Both are immediate, and coast
stays the watchdog's action. So `drv duty 0` with a ramp armed takes seconds to
wind down; `drv coast` is still the stop that happens now.

### `--ramp` is deliberately kept

`bench.py --ramp <%/s> --ramp-from <%>` is **not** deprecated by the firmware
limiter. It is the reference the firmware version was checked against, and the
un-ramped case is the contrast that proves the feature works.

**The A/B is now on record (2026-09-26, loaded rig, 0 → 29% duty):**

| | | peak current |
|---|---|---|
| host staircase | `--ramp 5 --ramp-from 12`, 1% granularity, console pace | 572 mA (Sep 25) |
| firmware | `drv ramp 50` + `drv ramp floor 120`, 0.1% granularity, 1 kHz | **581 mA** |
| un-ramped | `drv ramp 0` | **1582 mA — at the 1579 mA trip** |

Measured slew rate **50.00 o/oo/s** against 50 commanded; 120 → 290 in **3400 ms**
against 3400 predicted. **Peak inrush falls at least 2.7×.**

Read the un-ramped number correctly: **1582 mA is the clamp's value, not the
demand's.** The driver was in ITRIP regulation for ~40 ms, and at 10 ms telemetry
the first sample is already clamped — the true peak is unknown and higher.

⚠️ **The un-ramped start sets no fault flag.** Neither bit 4 (nFAULT) nor bit 8
(ADC saturated) was ever set through the regulated event. "No fault latched" is
not evidence a manoeuvre stayed inside its current budget.

Note that with the ramp armed a 2% sweep step at 5%/s takes 0.4 s, which eats
into the 2 s `--settle` window. Fine at these rates; check it before raising
`--dwell` expectations or lowering the rate.

### Echo mismatches are counted, not fatal

The console echoes **each typed character as its own one-byte write**, while a
telemetry line is **one atomic write**. A `T,` record therefore lands *inside*
the echo of a command being typed:

```
sent:    drv timeout 2000
echoed:  drv tT,226,575448,200,11047,14780,230,3imeout 2000
```

`node.py` matches a telemetry record **anywhere** in a line, extracts every
match, and rejoins the residue into the echo - so both the record and the
command survive. When the reassembled echo still does not match what was sent,
that is **counted** (`echo_mismatches`, reported in `meta.json` and
`status.json`) rather than raised: the echo is a convenience, the telemetry is
the measurement, and a run should not die because a character interleaved. A
non-zero count is worth a look; it does not by itself invalidate the data.
`seq` gaps and `tx_dropped` are the checks that do.

## Run directory

```
runs/2026-09-25T14-03-11_sweep/
    meta.json       profile, arguments, outcome, and full dumps of
                    info / cfg / drv / enc taken at connect
    console.log     raw byte-for-byte transcript
    telemetry.csv   one row per T-line, plus host arrival time
    velocity.csv    one row per V-line — one per control step (step, stair)
    events.csv      every command sent, with host time
    sweep.csv       the settled result per duty point
    step.csv        one row per step segment, with the metrics below
    stair.csv       one row per setpoint held (stair profile)
    status.json     rewritten every second
```

**The raw log is written before anything is parsed.** A parser bug can then
cost an analysis but never a bench run — the run is re-parsable offline. Bench
time is the expensive thing here.

## Reading the data correctly

`count` is the measurement. `mrpm` is a convenience column.

`enc window` is a boxcar average over N × 1 ms ticks, so at window 100 the rpm
figure lags the true speed by ~50 ms and is smoothed over 100 ms. Sampling that
at 50 Hz and fitting an exponential to it would measure the *filter's* time
constant and report it as the plant's. So the tool derives speed from the slope
of `count` against the board's own `ms` stamp, and keeps `mrpm` only as a
cross-check.

Two more things the columns are telling you:

- **`current_is_motor`**: below **14.5% duty** the on-phase is shorter than the
  IPROPI settle + aperture window, so there is no synchronised sample and the
  current figure is *supply* current, not motor current. It flips partway up
  any sweep that starts low.
- **`seq`**: the board's own line counter. A gap means the TX ring overflowed
  and lines were dropped. The run is marked `suspect` in `meta.json` and
  `status.json` if any gap or any `tx_dropped` is seen — do not fit a run with
  holes in it without first finding out why.

## Safety

In the order it matters:

1. **`try` / `finally` plus a SIGTERM handler.** A normal exit, an exception,
   Ctrl-C and `kill` all end at the same stop: **`vel off`**, `drv duty 0`,
   `drv coast`, `drv disable`, `telem off`. Best-effort — each step is attempted
   even if an earlier one raised, because a half-executed stop is the worst
   outcome.

   **`vel off` is first, and that ordering is load-bearing.** With the velocity
   loop armed, the board calls `drive_set_duty()` fifty times a second from its
   own tick, so `drv duty 0` and `drv coast` are both overwritten about 20 ms
   after they land. `drv disable` would still cut nSLEEP and stop the motor, but
   a stop sequence whose first three steps are silently undone is one that works
   by accident. The general rule, and it will recur at the next layer up
   (CAN, then the rover supervisor): **arming a control loop invalidates every
   stop sequence that addresses the layer below it.** The loop is disarmed
   first, or the sequence is not a stop.
2. **The board's own watchdog.** `drv timeout 2000` is armed at run start and
   refreshed while dwelling. This is the part `finally` *cannot* cover:
   `kill -9` and a yanked USB cable run no Python at all, so the board has to
   be able to stop itself. It **coasts** rather than brakes — braking from
   speed drives I = E/R through the low-side FETs, and a dead host is exactly
   when nobody is watching the driver dissipate it.
3. **A duty ceiling**, 40% by default, matching the rover's stated operating
   band. Anything higher has to be asked for explicitly with `--max-duty`.
4. **Pre-flight refusal** on a latched nFAULT or a motor already commanded
   non-zero — better than a run whose conditions were wrong from the start.
   The fault check reads the telemetry `flags` bit rather than parsing the
   human-readable `drv` text, so it cannot drift with a wording change.
5. **Post-flight integrity**, as above.

**Every run so far is with a free-rotating wheel on the motor shaft and no
rig.** That is a condition of the data, not a detail — record it when the rig
goes on.

## Verifying the watchdog

This is the one safety property that has to be tested deliberately rather than
assumed, because it only ever fires when everything else has already failed:

```sh
./bench.py run sweep --duty 20 --dwell 600 &
sleep 10
kill -9 %1                      # no Python runs after this
```

The wheel must coast to a stop within ~2 s. If it keeps spinning, stop it by
hand at the console and do not run anything unattended until it is fixed.

## Telemetry line format

```
T,<seq>,<ms>,<duty>,<count>,<mrpm>,<ma>,<flags>
```

| field | |
|---|---|
| `seq` | uint32, +1 per line; restarts at 0 on `telem on` |
| `ms` | `HAL_GetTick()` at emission — board time, stamped before any USB latency |
| `duty` | signed per-mille, **as applied to the bridge** — with `drv ramp` armed this is the ramping value, not the target |
| `count` | int32 encoder position at the output shaft, 8403.2 counts/rev |
| `mrpm` | rpm × 1000 (filtered — see above) |
| `ma` | Imotor if flag bit 0, else Isup |
| `flags` | 1 sync · 2 enabled · 4 fault latched · 8 ADC saturated · 16 watchdog **has** expired since arming (sticky; a kick refreshes the countdown but does not clear it) |

Integer fields only: `%f` pulls in newlib's float formatter, which is far too
slow to run at 100 Hz.

`print_telem_line()` in `Core/Src/console.c` is the authority for this format;
the constants in `node.py` are the mirror.

## Velocity-loop line format

A second, opt-in record. `T,` says what the **bridge and plant** did; `V,` says
what the **loop decided**, which is not inferable from the other.

```
V,<seq>,<ms>,<sp_mrpm>,<meas_mrpm>,<out>,<ff>,<p>,<i>,<d>,<flags>
```

| field | |
|---|---|
| `seq` | uint32, +1 per emitted line; restarts at 0 on `telem on`, same as `T,`'s |
| `ms` | `HAL_GetTick()` at the control step |
| `sp_mrpm` | the **ramped** setpoint the loop acted on, milli-rpm — not the commanded target, which `vel` reports |
| `meas_mrpm` | `encoder_rpm()` × 1000: the boxcar the loop acted on |
| `out` | signed per-mille written to `drive_set_duty()` — the sum of the four terms, after clamping |
| `ff` `p` `i` `d` | the four contributions separately, per-mille, **before** the clamp. They sum to the unclamped output, so `out` differing from their sum is exactly the saturation |
| `flags` | 1 saturated · 2 integrator frozen · 4 …because `drive_slewing()` · 8 …because bridge disabled or fault latched · 16 setpoint watchdog **has** expired since arming (sticky) · 32 setpoint still ramping · 64 at least one control step went unpublished before this line |

**Bit 2 with 4 and 8 clear is the anti-windup freeze** — the mechanism doing
its job under saturation. Bit 2 *with* 4 or 8 means the integrator is being
held off because something else is in the way, and the run measured the
obstruction rather than the loop. The two look identical in `out` and want
opposite corrections, which is the whole reason they are separate bits.

**Bit 64 is not a gap.** A `seq` gap means lines were lost on the wire; bit 64
means a control step was computed and never published, because the main loop
did not drain the slot before the next tick overwrote it. There is no missing
`seq` to reveal it, so it is flagged in-band and counted separately
(`veloc_steps_missed` in `meta.json` and `status.json`). Either one marks the
run `suspect`. Without it a decimated stream reads as a slow control loop,
which is the wrong conclusion for someone about to change a gain.

### Turning it on, and the bandwidth it costs

```
telem on                   the master switch — V needs this too
telem vel on               one line per control step
telem rate 50              the pairing that fits
```

The V channel is **not** on the `telem` timer. It publishes once per control
step, which is `1000 / enc window` Hz — 50 Hz at window 20. Riding the telem
timer would alias it: at 100 Hz every step would appear twice, at 30 Hz they
would beat. Neither is readable as a step response, and the integrator and
derivative only mean anything per step.

115200 8N1 is 11.52 kB/s. A `T,` line is ~45–59 bytes, a `V,` line ~50–76.
`T` at 100 Hz plus `V` at 50 Hz is ~9.7 kB/s — 84% of the link, before the echo
of anything typed. **`telem rate 50` is the pairing that fits**, and
`telem vel on` warns when `telem_ms < 20`.

`telem off` remains a complete stop for both channels, which is what
`safe_stop()` already sends.

`print_velocity_line()` in `Core/Src/console.c` is the authority for this
format; the `VFLAG_*` constants in `node.py` are the mirror.

## Reading `step.csv`

One row per step segment (`up`, and `return` if `--return` was passed).

⚠️ **READ `anchor_s` FIRST.** `vel_slew` ramps the *setpoint*, and it ships at
4 rpm/s, so a `--rpm 10` "step" spends its first 2.5 s with the loop tracking a
moving target and `--rpm 20` spends 5 s. **Nothing in that window is a step
response.** Everything below is anchored accordingly, and the first version of
this tool was not: it reported the limiter's properties under the loop's name,
and the tell was that `rise_s` came back at 2.0 s for a 0 → 10 step *no matter
what Kp was*. Run with **`--slew 0`** to measure the loop's own step response.

**The setpoint ramp**

| column | |
|---|---|
| `ramp_s` | how long `VFLAG_RAMPING` was set — the setpoint ramp's duration |
| `slew_rpm_s` | the ramp rate recovered from the data; cross-check against `cfg vel_slew` |
| `anchor_s` | when the ramp ended. **`overshoot_*` and `settle_s` are measured from here** |
| `track_lag_rpm` | mean (setpoint − measured) over the back four-fifths of the ramp. While the setpoint moves, the loop's error *is* its bandwidth — **this is the number a gain change moves**, and it is the one to read when `ramp_limited` is set |

**Rise**

| column | |
|---|---|
| `rise_s` | 10% → 90% of the commanded change, **from the command instant** — deliberately not anchored, because it is what an operator actually waits |
| `rise_slew_floor_s` | `0.8 × ramp_s`: the 10→90% time the ramp costs before the loop does anything at all |
| `ramp_limited` | `rise_s` is within 30% of that floor, so **this segment measured `vel_slew`, not the loop**. A warning is printed |

**Regulation**

| column | |
|---|---|
| `overshoot_pct`, `overshoot_rpm`, `peak_rpm` | peak past the final setpoint, searched from `anchor_s` over `overshoot_window_s` |
| `tail_sd_rpm` | sd of the settled tail — the ripple the peak has to be judged against |
| `overshoot_above_ripple` | whether the peak clears **2 × `tail_sd_rpm`**. ⚠️ **`false` means there is no overshoot to attribute.** A maximum drawn from a rippling signal sits 2–3 sd high whatever the gains do; on this rig every peak so far is 4 encoder counts above target at *both* 10 and 20 rpm, which is quantisation and ripple, not a controller |
| `overshoot_window_s`, `overshoot_window_truncated` | the peak is searched over 3 s (the plant's slow pole is 2.75 s) so a longer `--dwell` cannot manufacture a bigger overshoot. Truncated means the dwell was too short to fill the window and the peak **reads low and is not comparable** with a full run |
| `settle_s` | the **last** moment outside ±2%, not the first moment inside it — a response that dips back out is not settled, and the first-crossing definition would call it settled anyway. Measured **from `anchor_s`**: this is the loop's own settling. `null` means it never settled within the dwell |
| `settle_from_command_s` | the same instant measured from the command. Both are true; their difference is `vel_slew`, and reporting only the second is what the old metric did |
| `settle_band_below_quantum` | the ±2% band is narrower than one encoder count at this `enc window`, so **`settle_s` cannot be computed honestly**. A warning is printed |
| `ss_error_rpm` | mean of the last quarter of the dwell, minus the setpoint |
| `i_at_rest_permille` | what the integrator is carrying once settled — **this is how much the feedforward missed by**. A large steady `i` with a small `ss_error` says `ff_a`/`ff_b` want re-fitting, not that Ki wants raising |
| `sat_fraction` | fraction of control steps with the output clamped |
| `freeze_fraction` | fraction with the integrator frozen, split into `freeze_antiwindup` / `freeze_drv_slewing` / `freeze_no_bridge` |

⚠️ **These are computed from `meas_mrpm`**, the boxcar the loop acted on. That
is deliberate — they describe the closed loop *as the controller experienced
it*, which is the right frame for choosing gains. It is the wrong frame for a
plant time constant: the warning above about fitting `mrpm` applies with more
force inside a loop, because the filter's lag is now in the feedback path.
Plant-side timing comes from the `T,` rows and `rpm_from_counts()`.

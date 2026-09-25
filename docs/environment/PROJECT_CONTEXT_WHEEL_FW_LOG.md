# RobertUN — Wheel Controller Firmware: Full Progress Log
**Last updated:** September 25, 2026 (bench host tooling built, flashed and then *used* — after the 12 V rail was repaired, four clean sweeps re-took the free-wheel plant with the wheel clamped; CCW is closed at +3.49%, the tool validated to −0.18% against the hand-typed table, and the asc/desc gap turned out to be the motor warming)

**Referenced from:** `PROJECT_CONTEXT_WHEEL_FW.md`, which carries a one-line-per-entry version of this log. This file is the verbatim, unedited detail behind each entry — pull it in when you need the exact numbers, register values, or reasoning chain, not for routine session start.

> **WRITING RULE — this is where session detail goes.**
> At the end of a session, write the full entry *here*, at the top of the log
> below, in the established style: a bold one-line headline, then nested bullets
> with the exact numbers, register values and reasoning. Then add **one line**
> summarising it to the brief log in `PROJECT_CONTEXT_WHEEL_FW.md`.
> Durable rules go to that file's KEY LEARNINGS, and open work to its NEXT TASKS
> — never to this file. Keeping detail out of the sibling is the entire point of
> the split: that file gets read every session, this one only on demand.

## Progress log (most recent first) — full detail

- **Sep 25 — BENCH HOST TOOLING: firmware `telem` + `drv timeout` flashed and
  proven at the wire; the first motor run stopped on a dead 12 V rail.**
  Built to end the hand-transcription era: every plant number on record so far
  passed through a human reading four lines and typing them into a table.

  **Firmware, two additions.**
  - `telem on|off|rate <1..100>` emits one line per sample from the main loop,
    beside `console_report_encoder()`:
    `T,<seq>,<ms>,<duty>,<count>,<milli_rpm>,<mA>,<flags>`, flags
    `1 sync · 2 enabled · 4 fault · 8 saturated · 16 watchdog`. **Integer
    fields only** — `%f` pulls in newlib's float formatter, far too slow at
    100 Hz, so speed goes out as milli-rpm. Emitted from the main loop and not
    the TIM6 ISR because `isense_read_sync_avg(16)` waits on conversions
    triggered once per 50 µs PWM period (~0.8 ms). Capped at **100 Hz**: a
    ~55 byte line at 100 Hz is ~5.5 kB/s of the 11.52 kB/s the wire has, where
    200 Hz would be ~95% and would start vanishing into `tx_dropped`.
    `seq` restarts at 0 on every `telem on`, so a host detects dropped lines
    directly instead of inferring them.
  - `drv timeout <ms>` — a command watchdog, absent until now. Counted down in
    `drive_on_tick()` **before** the fault path's early return, because a
    deadline that only advances while something is wrong is not a deadline.
    Any `drive_set_duty()` kicks it, so the host keepalive is the same command
    repeated with no extra verb. **Coasts, not brakes, on expiry** — braking
    from speed drives I = E/R through the low-side FETs (50 rpm is 3.6 A in the
    Ke table), and a dead host is exactly when nobody is watching the driver
    dissipate it. Arming clears the expiry latch; kicking does not, so a host
    that reconnects after a crash can tell whether the motor stopped itself.
    Default 0 (disabled), so no existing bench procedure changes behaviour.

  Build clean, no warnings: RAM 5432 B (4.14%), FLASH 94012 B (23.91%).
  Flashed over SWD with `STM32_Programmer_CLI`, download verified.

  **Host tool — `firmware/RobertUN_ModuleNode/tools/bench/`.** `node.py`
  (transport + protocol), `bench.py` (CLI, run directories, safety), README.
  Placed under the firmware tree deliberately: the parser is coupled to the
  console's exact output format, so a console change and its parser change land
  in the same commit.

  **The one hard part was the shared wire.** Telemetry and command replies
  interleave freely, so there is exactly one reader and every complete line is
  classified once — `T,` to the telemetry sink, everything else to a pending
  reply buffer, with `command()` draining that same pump while it waits. A
  command issued mid-stream therefore loses no samples. **The prompt is the
  frame boundary and it carries no newline**, which caused the one real bug: a
  telemetry line landing immediately behind `"> "` merged with it in the buffer
  and the prompt was never seen again. Fixed by consuming the prompt in the
  pump, in arrival order, rather than testing for it as a buffer suffix.

  **Verified before the bench, against a pty that emulates the console:** reply
  framing, echo stripping, clean replies mid-stream, 156 samples with zero seq
  gaps, and the safe-stop sequence. Then a full `sweep` end-to-end producing all
  six run files. The count-slope velocity fit is exact — at 5% the fake emits 11
  counts per 20 ms and the tool reported 3.927 rpm, which is that number to
  three decimals.

  **At the wire, motor stopped:** 31 lines in 3.0 s at 10 Hz, **zero seq gaps**,
  board-stamped intervals **99–100 ms against a nominal 100**. No drift and no
  catch-up burst, which is what the `telem_next += telem_ms` scheduling with a
  one-period resynchronisation guard was for.

  **The pre-flight fault gate was wrong, and the tool found it.**
  `drive_faulted()` reads the nFAULT pin directly, and the DRV8874 holds nFAULT
  low the entire time nSLEEP is low — so a freshly reset board **always**
  reports a latched fault, and the gate as written would have refused every run
  on an artifact. Measured: `flags=0x04` asleep, then `flags=0x02` with no fault
  across all 21 samples once `drv enable` + `drv clearfault` had run at duty 0,
  12 mA. Pre-flight now wakes the driver and clears the latch *first*, so what
  it tests is a fault that re-asserts while awake at zero duty — which is real.

  **First motor run: no 12 V rail.** `sweep --duty 20 --dwell 3` was accepted by
  the board — `duty_permille 200`, `flags=3` (synchronised **and** enabled), so
  the H-bridge was genuinely switching — but `count = 0` across all 150 samples
  and current sat at **11–16 mA**, indistinguishable from the 12 mA idle reading
  at zero duty. A stalled motor at 20% into 1.87 Ω would pull hundreds of mA to
  amps and hit the 999 mA trip; no trip, no fault, no motion. Logic side runs
  off the ST-Link/USB, so console and encoder stay alive while VM is dead.
  **Diagnosed from the recorded run in one look, with no re-run** — which is the
  first concrete return on logging raw before parsing. Bench work paused there
  at the user's call; the 12 V line is being repaired.

  **Rail repaired, and the third bug surfaced immediately.** First command
  after the repair failed the echo check:

  ```
  echo mismatch: sent 'drv timeout 2000',
                 board echoed 'drv tT,226,575448,200,11047,14780,230,3'
  ```

  Not a corrupted link — a framing assumption. The console echoes **each typed
  character as its own one-byte write**, while a telemetry line is **one atomic
  write**. So a `T,` record does not politely wait for a line boundary; it lands
  in the middle of the echo of whatever is being typed. A parser that anchors
  the record at the start of a line loses the record *and* mangles the echo
  behind it. Fixed by matching `T,…` **anywhere** in a line
  (`TELEM_RE.finditer`), emitting every match, and rejoining the residue either
  side into the echo text — plus a `_partial` carry for the case where the
  line's newline belonged to the record rather than to the echo. Echo mismatch
  was also downgraded from fatal to a counted `echo_mismatches`, surfaced in
  `meta.json` and `status.json`: the echo is a convenience, the telemetry is the
  measurement, and a run should not die because a character interleaved.
  Unit-tested against the exact observed byte stream — both records recovered,
  echo reassembled to `drv timeout 2000`, no sequence gap. **Neither this bug
  nor the nFAULT one was reachable from the pty simulator**; both needed the
  real board's write granularity and real async traffic.

  **Then the plant was re-taken — four sweeps, all integrity-clean.** The user
  also changed the fixture: the wheel is now **clamped to the table**, where
  every Sep 23 figure was taken with the operator **holding the wheel in their
  hands**. Runs in `tools/bench/runs/`, enc window 100, trip 1580 mA set
  explicitly at run start, watchdog 2000 ms:

  | pass | range | fit | R² | max\|res\| |
  |---|---|---|---|---|
  | CW ascending | 11–39% | `rpm = 0.8327 d − 1.524` | 0.99993 | 0.117 |
  | CW descending | 11–39% | `rpm = 0.8187 d − 0.752` | 0.99993 | 0.134 |
  | CCW ascending | 11–39% | `rpm = 0.8618 d − 1.164` | 0.99999 | 0.059 |
  | CW full range | 10–100% | `rpm = 0.8186 d − 0.833` | 0.99998 | 0.286 |

  **0 sequence gaps, 0 echo mismatches, 0 `tx_dropped` on all four.**

  - **The validation gate passed.** The plan required reproducing the hand-taken
    table before trusting any new number. Like-for-like over the *same* 20–100%
    points: Sep 23 by hand `0.8193 d − 1.065`, Sep 25 by tool
    `0.8178 d − 0.775` — **slopes agree to −0.18%**. Per-point deltas +0.52
    (20%), −0.03 (40%), −0.02 (60%), +0.34 (80%), +0.06 (100%) rpm.
  - **The apparent plant change was the fixture, and the diagnosis held.** The
    first comparison looked alarming — −5.6% on speed, +26% on current,
    breakaway up — and was called as increased friction before the cause was
    known. The user then supplied it: hand-held → clamped. That is the same
    thing. **Clamped CW ascending slope 0.8327 vs hand-held 0.830 — 0.3% apart
    — while the intercept moved −0.96 → −1.52.** A hand adds a damping term a
    clamp does not; it shows up entirely in the offset.
  - **CCW closed.** +3.49% faster than CW at the same duty (0.8618 vs 0.8327).
    **Aug 26 measured +3.5%** by hand, on a 9.35 V rail, on a bare shaft. Three
    variables changed and the number did not, so the asymmetry is in the motor
    (brush timing), not in any one bench setup. CCW is also the best-conditioned
    fit of the four. One real difference: **CCW did not break away at 5%** where
    CW did — breakaway is direction-dependent even though the running slope is
    clean.
  - **What looked like hysteresis is thermal.** Descending reads faster than
    ascending at every shared duty, but the gap is +0.94 rpm at 5%, +0.69 at 8%,
    +0.63 at 11%, +0.56 at 14%, +0.54 at 17%, +0.39 at 20%, +0.43 at 23/26/29%,
    +0.34 at 32%, +0.44 at 35%, and **+0.05 at 39%**. The 39% points were taken
    back-to-back; the 5% points ~11 minutes apart. **The gap tracks elapsed
    time, not direction** — the motor warms and friction falls. Sep 23's "no
    speed hysteresis" conclusion stands; this is a second, slower effect that a
    naive up-then-down sweep would have reported as hysteresis.
  - **Current, 20–100%: mean 226 mA, range 200–253, slope 0.555 mA/%.** Flat,
    i.e. friction torque rather than load — consistent with Sep 23. Dropout is
    still between 3% and 2% duty (2% → 0.00 rpm), unchanged by the clamp.

  `figures/plot_plant_12v.py` was extended with all four Sep 25 series alongside
  the Sep 23 hand-held data, and now **derives every fit it draws and prints
  them** rather than carrying quoted constants.

  **Still owed:** the `coastdown` / `step` / `hold` / `stiction` profiles and
  `analyze.py`; the **deliberate watchdog test** (start a long run, `kill -9`
  it, watch the wheel coast — it must be done, not assumed); and the loaded-rig
  pass.

- **Sep 21 — TASK 20 BENCH-VERIFIED. `drv current` now agrees with physics to
  +0.4%, and the same traces re-reproduced the bug it was built to remove.**
  One verification run, taken one step at a time, on the 12.0 V rail with the
  shaft held.

  **Step 1 — the `cfg` check, deliberately before any current reading.**
  `CONFIG_VERSION` 1 → 2 and `CFG_KEY_COUNT` 8 → 9 meant `scan()` rejected every
  stored record and the board booted on defaults. It came up **already
  calibrated**, which was the whole point of putting the measured constants into
  the defaults rather than leaving them as saved overrides:

  ```
  config v2, 9 keys, slot 0/1024 used
    vdda_mv    3325  (default 3325)
    r_ipropi   1465  (default 1465)
    a_ipropi    450
    trip_ma    1000  (0..1600)
    duty_limit 1000
    rail_mv   12000
    isense_avg   32
    sat_raw    4050
    vref_div      3
  ```

  No `*` override markers anywhere — the calibration is the default now. Then
  the trip ceiling, which validates item 1 without spinning the motor:

  ```
  drv trip 1580
    trip 1579 mA  (VREF 3123 mV, DAC code 3847, buffer on)
    range 101..1580 mA, ADC ceiling 5043 mA
  ```

  **Predicted 1580 / 101 / 5043 on the desk; measured 1579 / 101 / 5043.** The
  1 mA is integer truncation in the round trip, not error.

  **Step 2 — the calibration re-take. This is the measurement task 20 existed
  for.** 20% duty, shaft stalled, `drv iscan 64 3550 4500 50` either side of a
  `drv current`:

  | Quantity | raw | mA |
  |---|---|---|
  | `iscan` settled tail, run 1 (ticks 4100–4400) | 1056 | 1300 |
  | `iscan` settled tail, run 2 (ticks 4100–4400) | 963 | 1186 |
  | **`drv current`** | **1030** | **1268** |
  | predicted, `D × Vm / R_motor` = 0.20 × 12.0 / 1.90 | 1026 | **1263** |

  `drv current` is **+0.4%** on the prediction. The acceptance criterion written
  into task 20 was "landing within a few percent of the iscan tail instead of
  ~13% below it"; it landed *between* the two tails, on physics.

  And the `sync:` line confirms three separate items at once:

  ```
  Imotor 1268 mA  (raw 1030, offset 0)  at duty +20%  decay slow
    sync: 64 samples over 4 ticks in 4100..4348, drive phase 900 ticks = 10.0 us of 50.0
    implies Isup 253 mA (Imotor x D)
  ```

  `over 4 ticks` is the spread live (not its single-tick fallback); `4100` is the
  500-tick settle floor honoured exactly; `4348` is the end-relative placement
  computed exactly as the desk check predicted.

  **The bug re-reproduced itself in the same data, which is better evidence than
  the fix passing.** The old midpoint tick is 4050, and it is still in every
  scan:

  | | midpoint t4050 | that run's tail | error |
  |---|---|---|---|
  | run 1 | 905 | 1056 | **−14.3%** |
  | run 2 | 808 | 963 | **−16.1%** |

  So the ~13% low reading measured on Sep 20 was not a one-off — it is where the
  midpoint convention always sat. `drv current` at 1030 raw is a **27%**
  correction over the 808 the old code would have reported in run 2.

  **The ±8% rotor-position noise floor showed up again, and it is the dominant
  uncertainty now.** Two `iscan` runs at a nominally *identical* operating point
  gave tails of 1056 and 963 — **9.3% apart**. Nothing was changed between them.
  This is larger than every remaining systematic error in the current path, and
  it is why the prediction sitting between the two tails is the right way to read
  this result rather than picking either tail as "the" value. Back-to-back
  references remain mandatory.

  **The guard band earned itself.** Tick 4450 — 50 ticks from the window end, so
  its 112-tick aperture runs 62 ticks past the falling edge — read **806** in run
  1 against a 1056 tail (−24%) and 924 in run 2 against 963 (−4%). The trigger at
  4348 + 112 aperture = 4460 keeps the whole aperture 40 ticks clear of the edge.
  Without the 40-tick margin the sampler would be reading into the decay phase at
  every duty.

  **Step 3 — duty 0, volunteered rather than asked for, and it checked two
  things.** `drv iscan 64 3550 4500 50` at duty 0 printed `no drive phase at this
  duty - scanning anyway, every point should read the same` and then read a flat
  **4–5 raw at every one of the 19 ticks**, with `trigger currently sits at 4500`.
  That is the `ticks == 0 → DRIVE_CCR_FULL` case confirmed, and it reproduces
  Sep 20's "flat 3–5" pedestal. `drv current` at duty 0 correctly fell through:
  `Isup 2 mA` plus the rewritten NOT SYNCHRONISED warning citing the 652-tick
  minimum. The gate refuses what it should refuse.

  **Not exercised:** the tick-0 `--` cosmetic fix. Every scan started at 3550;
  it needs one with `from = 0`.

  **Loose end worth 0.4%:** `offset 0` in both readings means `drv zero` had not
  been run this boot, so every reading carries the 4-count (~5 mA) pedestal the
  duty-0 scan just measured. Subtracting it moves 1268 → 1263 — cosmetically
  exact, materially irrelevant, but free.

  **NEW LEAD — the decay phase may be readable, which would dissolve the W5
  low-duty blind spot.** Tick 3550 sits in the decay (brake) window, 50 ticks
  before the drive edge:

  | run | decay t3550 | drive tail | ratio |
  |---|---|---|---|
  | 1 | 709 | 1056 | **0.671** |
  | 2 | 642 | 958 | **0.670** |

  Three digits of agreement, across two runs whose absolute levels differ by 9%.
  That is a *reproducible attenuation*, not noise. And it is not physics: with
  both low-side FETs on and a stalled rotor there is no back-EMF to drive decay,
  so over a 40 µs brake window at τ = L/R = 0.9 ms the current should fall
  **4.4%**, not 33%. Something in the DRV8874's mirror reports the brake-phase
  current at roughly two thirds — plausibly because it mirrors one sense element
  while the circulating current splits between two low-side FETs.

  **Why this matters:** the standing W5 constraint is that no synchronised
  reading is possible below **14.5% duty**, because the drive window gets shorter
  than the 500-tick settle — and the rover creeps below that (breakaway was
  12–14% duty). The decay window is *widest* exactly where the drive window is
  too narrow. If the 0.670 factor holds across duties, a current inner loop
  becomes possible at low duty with a single calibration constant, and the
  fallback options (free-running `Isup`, or a slower carrier in that band) become
  unnecessary. **Two points at one duty is a lead, not a result** — it needs a
  decay-phase scan across several duties before anything is designed on it.

  **Console fixed afterwards, not during.** The one remaining lie, `drv trip`'s
  `1 code = 1 ADC LSB`, is now `1 DAC code = 1/3 ADC LSB (VREF/3)` — a DAC step
  moves VREF by one step but the comparator sees VREF/3, so it moves the trip by
  one third of a current step (0.410 mA, against the 1.231 mA ADC LSB). This was
  known wrong before the bench session and **deliberately held** until the
  measurement was done, per the Sep 20 key learning about not changing firmware
  between flashing and verifying. Builds clean, flash 91312 → 91320 B (+8, the
  longer string).

  **Task 20 is closed. W4 has no owed items left.** W5 remains blocked on the
  12 V plant re-take (task 17), which is unchanged by any of this.



- **Sep 20 (later) — TASK 20 IMPLEMENTED. The current path now means what it
  prints: `drv trip N` is N milliamps, the sampler reads the settled tail of the
  drive window instead of its contaminated middle, and the two constants
  measured on Sep 12 are finally applied. W4 closes on this; W5 opens blocked on
  one measurement.**
  - **`k = 3` as a config key, not a `#define`.** `CFG_VREF_DIVIDER` /
    `cfg vref_div`, range 1..4, default 3, appended before `CFG_KEY_COUNT`.
    Applied in the **conversion pair only** — `isense_ma_to_vref_mv()` and
    `isense_vref_mv_to_ma()` — so every derived function (`trip_code`,
    `isense_trip_ma`, `isense_trip_max_ma`, `isense_trip_min_ma`,
    `isense_code_for_trip_ma`) follows without touching any of them.
    `isense_raw_to_ma()` and `isense_full_scale_ma()` are deliberately
    **untouched**: the ADC reads the resistor directly and never sees the
    divider. A `k == 0` guard returns 1 rather than dividing by zero — a config
    module that fails open is worse than none in a current-limit path.
    - **Chosen over a compile-time constant** so a second-source part with a
      different divider is a console command rather than a rebuild. The cost is
      that `k` is runtime-writable, and `cfg vref_div 1` would triple every real
      trip while the console reported no change; the 1..4 range and the help
      text carry that warning.
  - **A latent uint32 overflow, found while applying `k`.** Tripling the
    multiply made it wrap three times sooner. `drv trip 3000` — exactly what
    habit types, since it was the old boot default — computes
    `3000 × 3 × 450 × 1465 = 5.93e9`, over `UINT32_MAX`, and would have come
    back as **~811 mA reported as though it had been honoured**, instead of
    clamping to the 1580 mA ceiling. Both conversion functions moved to `uint64`
    intermediates with a 65535 mV clamp. **The pre-`k` code had the same fault
    above 6516 mA**; it was simply further from anything anyone typed.
  - **The timing budget, in `drive.h` rather than `isense.h`.** These are
    TIM4-tick quantities about the drive window, `drive.c` does not include
    `isense.h`, and the budget belongs next to the code that honours it:

    | quantity | ticks | source |
    |---|---|---|
    | `DRIVE_IPROPI_SETTLE_TICKS` | 500 | measured Sep 20 (flat from 4100, edge at 3600) |
    | `DRIVE_ADC_APERTURE_TICKS` | 112 | 28 cycles @ 22.5 MHz = 1.24 µs |
    | `DRIVE_TRIGGER_MARGIN_TICKS` | 40 | chosen guard |
    | `DRIVE_PHASE_MIN_TICKS` | **652** | sum → **14.5% duty** |

    `ISENSE_SYNC_MIN_TICKS` (192, 4.3% duty) is retired. It was derived from the
    aperture alone and ignored the settle, so it green-lit readings whose entire
    drive window was shorter than the time IPROPI needs to stop ringing.
  - **`place_trigger()` is end-relative, not a fraction.** The task originally
    proposed `start + (ticks × 4) / 5`. That was rejected during implementation:
    a fraction gives a *different* amount of settling time at every duty, so it
    is correct at one operating point and quietly wrong elsewhere. The rule is
    now `trigger = start + ticks − (aperture + guard)`, floored at
    `start + settle` and capped below `DRIVE_CCR_FULL`. The `ticks == 0 →
    DRIVE_CCR_FULL` case is unchanged.

    | duty | window | trigger | gate |
    |---|---|---|---|
    | 10% | 4050–4500 | 4499 | refused |
    | 14% | 3870–4500 | 4370 | refused |
    | 15% | 3825–4500 | **4348** | ready |
    | 20% | 3600–4500 | **4348** | ready |
    | 50% | 2250–4500 | **4348** | ready |

    At 20% that is 83% through the window and well past the measured 4100 settle
    point. The trigger stops moving above 15% duty, which is the rule working:
    the useful sample is a fixed distance from the *falling* edge.
  - **`drive_phase_start()` added, and it was load-bearing.**
    `console.c` derived the window start as `trig − ticks/2`, true only under the
    midpoint convention. Left alone, moving `place_trigger()` would have
    mis-placed `drv iscan`'s in-window `*` markers — **corrupting the exact
    instrument used to verify the fix.** Accessor and call site changed in the
    same step, before any measurement was taken.
  - **Multi-tick averaging (item 5).** `drv current` was one tick on a waveform
    that still carries commutation ripple — the Sep 20 settled tail wandered
    **739–764 raw** across the region. `sync_burst_spread()` now spreads the
    samples over `ISENSE_SYNC_POINTS` = **4** ticks evenly across
    `[start + settle, start + ticks − aperture − guard]`, inclusive of both ends
    so the two most informative points are always taken. **Same total periods**
    — 64 samples is still 64 PWM periods, still 3.2 ms — with the `n % points`
    remainder given to the first point and the mean weighted by share, so the
    count is exactly what the caller asked for. Degrades to a single tick when
    the region collapses at the 652-tick minimum, which is the budget being
    honest rather than an edge case.
    - **`isense_read_sync_at()` still calls `sync_burst()` directly.** `drv
      iscan` has to stay a single-tick probe: it is the instrument that measures
      where the settled region *is*, and averaging inside it would hide the
      ringing the whole mechanism exists to avoid.
    - `was_saturated` is now accumulated across the spread rather than left to
      whichever point went last. Saturation is a safety flag.
  - **`drv iscan` prints `--` at tick 0**, with a one-line footnote, and keeps it
    out of the peak search. CCR4 = 0 leaves TIM4_CH4 permanently high, raises no
    compare event, times out `adc_wait_eoc()` and returns 0 for "took nothing" —
    which printed as `raw 0` beside 35 real numbers and read as a current.
  - **The two measured constants applied at last**, the Sep 12 hold released now
    that the sweep is finished: `ISENSE_VDDA_MV_DEFAULT` 3300 → **3325**,
    `ISENSE_R_IPROPI_OHM_DEFAULT` 1474 → **1465**. Everything derived moves:

    | | nominal | measured |
    |---|---|---|
    | scale | 0.6632 V/A | **0.6593 V/A** |
    | ADC full scale | 4.975 A | **5.044 A** |
    | one ADC LSB | 1.215 mA | **1.231 mA** |
    | trip range, buffered | 100–1558 mA | **101–1580 mA** |
    | trip range, unbuffered | 0–1658 mA | **0–1681 mA** |

    Every current logged before today reads **~1.4% low**. Both are per-board
    figures — a second carrier gets metered and `cfg`-set, not handed these.
  - **`CONFIG_VERSION` 1 → 2, and the stored record is discarded.**
    `CFG_KEY_COUNT` goes 8 → 9 and `trip_ma` changed meaning while keeping its
    name, units and range — which is precisely what the version field is for.
    `scan()` rejects every existing record and the board boots on defaults
    reporting `CONFIG_LOAD_VERSION`. **This is the safe direction**: a stored
    `trip_ma 3000` would otherwise have become a real 3 A limit on a board whose
    ceiling is 1.58 A. `CFG_TRIP_BOOT_MA` range tightened 0..6000 → **0..1600**,
    and `ISENSE_TRIP_DEFAULT_MA` 3000 → **1000**, written as the figure it
    always physically was. Putting the newly measured constants into the
    *defaults* in the same change means the wiped board comes up **calibrated**,
    not nominal.
  - **Desk verification, before anything was flashed.**

    ```
    buffered    floor 200 mV -> trip min  101 mA   ceil 3125 mV -> trip max 1580 mA
    unbuffered  floor   0 mV -> trip min    0 mA   ceil 3325 mV -> trip max 1681 mA
    drv trip 1000 -> VREF 1977 mV, DAC code 2435, reads back  999 mA
    drv trip 3000 -> clamps to 1580 mA   (pre-fix: wrapped to ~811, reported as honoured)
    place_trigger, 20% duty (start 3600, ticks 900) -> tick 4348
    ```

    Build clean under `-Wall -Wextra`. Flash **89792 → 91312 B** (+1520, the
    uint64 divide helper and the spread burst), RAM 5384 → 5392 B. Baseline
    measured by building `HEAD` in a throwaway git worktree, not recalled.
  - **Documentation rewritten in `isense.h`**, which is this project's real
    documentation and had three sections that were now false: the plateau test
    posed as an open question, the Sep 12 "reading is SUPPLY current" conclusion
    with its IMODE-blanking cause and `trip² × R_motor / Vm` quadratic, and the
    192-tick gate rationale. Retractions are left visible rather than deleted,
    the convention the file already uses for its Sep 14/15 aliasing retraction.
  - **Still owed on the bench**, and the reason task 20 is not yet ✅: the
    re-take of the calibration point — 20% duty, stalled, `drv current` against
    both `D × Vm / R_motor` = 1263 mA and the settled `drv iscan 64 3550 4500 50`
    tail. Success is `drv current` landing within a few percent of the iscan tail
    instead of ~13% below it; that one comparison validates the trip scaling, the
    gate, the placement and the averaging at once. **First step is a `cfg show`,
    not a current reading** — the stored record is gone and must be confirmed
    and re-saved before any number is trusted.

- **Sep 20 — PLATEAU SWEEP DONE: the DRV8874 compares IPROPI against VREF/3.
  Every `drv trip` is 3× too high, the real trip range is 100–1558 mA, the
  current-sense chain is calibrated against physics for the first time, and
  three bugs were found in the Sep 16 synchronised sampler.**
  - **Method.** Motor stalled against a mechanical hold (`enc` confirming
    `rpm 0.00` before every run), 20% duty, slow decay, 12.0 V at VM. Each point
    is `drv iscan 64 3550 4500 50` and the figure taken is the **mean of the
    settled tail, ticks 4100–4450** (8 points), not the `drv current` single
    sample — see the trigger-placement bug below. Scale 1.215 mA/count.
  - **The unregulated reference validates the whole chain.** With the trip
    parked above demand, 20% duty at stall read **1062 raw = 1290 mA** against
    `0.20 × 12.0 V / 1.90 Ω` = **1263 mA predicted — 2%.** IPROPI, the modified
    1.474 kΩ R_IPROPI, `ISENSE_VDDA_MV`, the ADC and `isense_raw_to_ma()` all
    agree with an independent physical prediction. First end-to-end current
    calibration the project has had.
  - **The sweep** (all at 20% duty stalled; "demand" = unregulated current):

    | `drv trip` | VREF | tail raw | mA | reading |
    |---|---|---|---|---|
    | 4673 | 3099 mV | 1062 | 1290 | unregulated reference (early) |
    | 4673 | 3099 mV | 1143 | 1389 | unregulated reference (late) |
    | 3297 | 2187 mV | 987 | 1199 | ambiguous — sat on the onset |
    | 2997 | 1988 mV | 768 | 933 | flat plateau |
    | 2995 | 1987 mV | 750 | 911 | flat plateau, repeat |
    | 1997 | 1325 mV | 171 | 208 | ramp then collapse |
    | 999 | 663 mV | 59 | 72 | ramp then collapse |

  - **`k = 1` refuted.** A trip of 1997 mA sits above every demand measured
    (1290–1389 mA), so under `k = 1` nothing would clamp and the tail would have
    read ~1062. It read 171.
  - **`k = 2` refuted.** At trip 2997, `k = 2` puts the limit at 1499 mA, again
    above demand, so the tail should have been the unregulated value. It sat
    28–33% below the reference band. For that to be demand rather than
    regulation, `R_motor` would have to be 2.57 Ω — 35% above the measured
    1.90 Ω, a ~90 °C winding rise. Not credible.
  - **`k = 3` confirmed predictively, by demand-independence.** The two trip-2997
    plateaus (933 and 911 mA) were taken either side of a reference that moved
    **+7.6%** (1062 → 1143 raw). A current limit must ignore that; a measurement
    of current must track it. The plateau moved **−2.3%**, inside its own ±1.5%
    tail scatter. The prediction was 768 raw if regulating, 826 if tracking; it
    came in at **750**.
  - **The regulated average sits at ~92% of the peak limit.** Plateaus of 933 and
    911 mA against a nominal `trip/3` of 999 and 998 mA. The shortfall is
    chopping ripple: the peaks touch the limit, the troughs do not.
  - **Why the shapes differ, and the regime that is readable.** With the limit
    29–39% below demand the current hovers at the threshold and the tail is
    **flat**. With it ~94% below (trip 1997) the current slams into the limit
    early and the tail **collapses to near zero**. That collapse is *not* the
    current going away — falling from ~1500 mA to ~70 mA in 5 µs is two orders
    of magnitude faster than L/R decay allows (τ = L/R ≈ 0.9 ms), and body-diode
    coasting at 12 V would shed only ~40 mA over that span. **IPROPI goes blind
    while regulation is active.** W5 cannot measure current whenever the loop is
    hard-limiting.
  - **Regulation is audible.** A low but clear high-pitched tone appeared at
    exactly the points the electrical data says the driver was limiting.
    Chopping adds switching events unrelated to the 20 kHz carrier and drops the
    acoustic signature into hearing range. Useful bench indicator: chirp means
    limiting, no console required.
  - **CONSEQUENCE 1 — `drv trip` and `cfg trip_ma` are 3× too high.** Commanding
    2997 mA yields a 999 mA peak limit. The console's printed range of
    301–4673 mA is really **100–1558 mA**, and the **3000 mA boot default is
    really 1000 mA** — which is what has actually been protecting the bench all
    day.
  - **CONSEQUENCE 2 — the maximum trip is always exactly one third of the ADC
    ceiling, whatever R_IPROPI is.** Both the comparator and the ADC read the
    same resistor, so the ceiling is `VDDA / (R × 450 µA/A)` and the trip tops
    out at `VREF_max/3` over the same product. Reaching a 4 A trip means
    accepting a ~12 A ceiling (R_IPROPI ≈ 600 Ω) and surrendering two thirds of
    the ADC's resolution to get it. **That is a hardware trade for HW1, not a
    firmware fix.** The cold stall at 12 V is ~6.3 A and is presently
    unreachable as a trip by a factor of four.
  - **BUG 1 — IPROPI needs 5.6 µs to settle, not the datasheet's 1.6 µs
    `tDELAY`.** From the trip-4673 run, the drive edge is at tick 3600 and the
    reading rings through 3650 (1757), 3700 (212), 3800 (1659), 3950 (1275),
    4050 (994) before going flat from **tick 4100 — 500 ticks, 5.6 µs.** Almost
    certainly ringing in the sense network rather than the mirror itself.
  - **BUG 2 — `ISENSE_SYNC_MIN_TICKS` is 192 and needs to be ~600.** At 192 the
    gate admits a reading at 4.27% duty, where the entire drive window is
    shorter than the settling time. **Nothing measured below ~13–15% duty is
    valid**, and the console reports those readings with no warning at all.
  - **BUG 3 — `place_trigger()` samples the window midpoint, which is the
    contaminated half.** All the ringing is at the leading edge, so the midpoint
    is as close to it as the window permits. At 20% duty the midpoint (tick
    4050) read 994 against a settled 1143 — **13% low**. It belongs at 75–85%
    through the window. The Sep 15 conclusion that the trigger placement was
    correct is withdrawn.
  - **Casualties of bugs 2 and 3, measured the same day:** `drv current` gave
    **87 mA at 13% duty free-running** and **211 mA at 8% duty stalled** (against
    505 mA predicted). Both sampled inside the ringing. Both are garbage.
  - **Zero offset is a non-issue.** `drv zero` with nSLEEP low returned **0
    counts over 256 samples** — legitimate, since the sleeping mirror is high-Z,
    R_IPROPI pulls the ADC input to ground, and there is no negative rail to
    dither below. The *awake* pedestal, which `drv zero` structurally cannot
    reach (it refuses while the bridge is live), was measured instead by an
    `iscan` at `duty 0` with nSLEEP high: **flat 3–5 counts ≈ 4 mA** across the
    whole period. Negligible, and no firmware change needed.
  - **That zero-duty scan also served as the control that confirmed PWM mode a
    third time.** The 13% scan showed 107–339 raw through the decay phase where
    Sep 15 read exact zeros; the zero-duty scan proves that is real recirculating
    current and not a mirror pedestal. IPROPI sees the low-side FETs during
    brake, which only low-side decay produces.
  - **Tick 0 in an `iscan` is a non-measurement, always.** It reads exactly 0 in
    every scan including the zero-duty control, because CCR4 = 0 leaves TIM4_CH4
    permanently high and never produces a compare edge — `adc_wait_eoc()` times
    out and `sync_burst()` returns 0 on `taken == 0`. Documented behaviour
    (`drive.c`, `place_trigger()`), but it prints as though it were data.
  - **Stall measurements carry a ±8% rotor-position noise floor.** The
    unregulated reference moved 1062 → 1143 raw (+7.6%) across the session, and
    the encoder crept tens of counts *during* individual runs (1181203 → 1181272
    inside one scan). The hold is compliant, the motor twists against it, and at
    stall the armature resistance depends on which commutator segments are
    bridged. **Every plateau point needs its own reference taken back-to-back**;
    one reference compared against forty minutes of later points is worthless,
    and treating it as a constant cost one wasted confirmation run (trip 3297,
    which landed inside the scatter band and resolved nothing).
  - **RETRACTION — the Sep 12 "`isense_read_ma()` returns SUPPLY current"
    conclusion, and its stated cause, are both wrong.**
    - The cause on record was that "the carrier's 20 kΩ IMODE strap blanks the
      mirror during recirculation." It was never IMODE. The driver was in
      **independent half-bridge**, so decay was **high-side**, and IPROPI —
      which mirrors only the low-side FETs, drain→source — is physically blind
      to it.
    - The conclusion itself was a **sampling artifact that a later commit already
      fixed**. `place_trigger()` landed in `9187dc9` on **Sep 16**; on Sep 12
      there was no phase-synchronised sampling at all. The ADC free-ran and
      averaged across the whole period, and a free-running average of a signal
      that is zero for `1 − D` of it is `I_motor × D` by construction. The Sep 16
      trigger work, not the Sep 19 mode fix, is what turned the reading into
      motor current.
    - **Therefore the plateau-sweep formula recorded in the carrier section —
      `trip² × R_motor / Vm`, quadratic in the trip — describes only the
      unsynchronised fallback path.** On the synchronised path the plateau is
      `trip/k` directly. Applying the quadratic would have produced a badly wrong
      `k`.

- **Sep 19 (later) — PMODE CONFIRMED LATCHED IN PWM MODE, ground return rebuilt,
  motor rail raised to 12 V. The Sep 16 blocker is cleared and current
  regulation is live for the first time.**
  - **Ground return replaced: three thicker conductors** from breadboard PGND to
    the MCU carrier board, in place of the single DuPont that carried the Sep 16
    fault current. Sized for stall rather than the working point, per the rule
    that fault current, not working current, sets the conductor.
  - **The `drv pin`-free build was flashed**, which also restored PB7 to
    AF2/TIM4_CH2 (the reset re-runs `drive_init()`), so no separate restore was
    needed.
  - **The mode test, and why it is conclusive this time.** Both motor outputs
    scoped simultaneously at `drv duty 13`: **one output carries PWM switching to
    the rail, the other sits at GND for the whole period.** At `drv duty -13` the
    roles swap cleanly. The discriminating channel is **the quiet one, not the
    switching one**:

    | | Driven output | Other output |
    |---|---|---|
    | **PWM mode** (IN1 high, IN2 PWM'd) | PWM, 0 ↔ VM | **constant GND** |
    | Independent half-bridge | PWM, 0 ↔ VM | **constant VM** |

    In independent half-bridge each output follows its own input, so IN1 held
    high would park OUT1 at the **rail** all period. Ground is only reachable via
    `IN1=1, IN2=1 → OUT1 L, OUT2 L` — the low-side slow decay of Table 4. This is
    the state the two modes disagree on, so it is the only state worth probing,
    and it is now measured rather than inferred. **Scoping IN1/IN2 could never
    have settled this**: PMODE changes how the driver interprets its inputs, not
    what the MCU emits, so the input waveforms are identical in all three modes.
  - **This closes the Sep 14 error properly.** That session ruled out PH/EN from
    an rpm figure and treated PWM mode as proven, never enumerating independent
    half-bridge — which gives the same average voltage under slow decay and
    therefore the same rpm. The rpm evidence discriminated one alternative out of
    three; the OUT1/OUT2 decay state discriminates the remaining two.
  - **Motor rail raised to 12 V.** **12.0 V measured with a DMM at the DRV8874's
    VM pin — this is the authoritative figure.** The oscilloscope read ~12.4 V on
    the driven output; the ~0.4 V discrepancy is scope ADC accuracy, and the DMM
    value is the one to quote. Note this is VM **at the driver**, not at the motor
    terminals: task 17 asks for 12 V at the *terminals*, and there is ~0.10 V of
    harness drop at light load and more under current, so the terminal figure is
    still unmeasured and will read slightly lower.
  - **Current regulation is live for the first time, which changes what every
    trip number means.** Independent half-bridge disabled internal current
    regulation outright, so all `drv trip` / PA4 VREF work from Sep 11–16 acted
    on nothing. Those settings now reach the hardware, and the **boot default of
    3000 mA is a real trip point** — below the ~6.3 A cold stall the 12 V rail
    now implies. Task 17's precondition (`drv trip 5000` before raising the rail)
    was not applied ahead of the change; the consequence is that the motor is
    trip-limited rather than over-current, which fails in the safe direction but
    makes any stall figure taken right now a property of the trip, not the motor.
  - **Every recorded plant number now belongs to the wrong rail.** The duty→speed
    line (`rpm = 0.672 × duty% − 1.8`), the deadband, breakaway, dropout and the
    minimum sustainable speed were all measured at 9.35 V. R, L, Ke and
    counts/rev carry over; the mappings do not. Re-measurement is now blocking
    W5 rather than merely pending.

- **Sep 19 — The replacement MCU board passes both pad checks and PMODE is
  strapped. Task 19's first item is retired; the bench is not yet unblocked.**
  The board fitted on Sep 18 was flashed and checked **bare** — DRV8874 wiring to
  PB6/PB7 left disconnected, because a pull-up anywhere on the net makes a
  healthy pad and a dead one read identically.
  - **Bare `drv pin`:** both pads `mode 2 af 2 pupd 0 od 0  ODR 0 IDR 0`. TIM4
    `CR1 0x0081` (CEN + ARPE), `CCER 0x1011` (CC1E + CC2E + **CC4E**, no polarity
    bits), `CCMR1 0x6868` — byte-identical halves, OC1M = OC2M = PWM1, both
    preloaded — `CCR1 = CCR2 = 0`, `ARR 4499` (20.0 kHz). This is the *same*
    register picture the Sep 16 dead board produced in every single field. The
    only value that ever differed between a live and a dead pad here is PB7's
    IDR, which is exactly why the register dump alone was never sufficient.
  - **`drv pin pd` is the verdict, and it passes:** PB7 reconfigured as an input
    on the internal ~40 kΩ pull-down, with every external wire off, reads
    `mode 0 af 2 pupd 2 od 0  ODR 0  IDR 0`. **The Sep 16 board read `IDR 1` at
    precisely this step.** The distinction matters: the 0%-duty AF report shows a
    *push-pull* low, which a partly-damaged pad can still produce, since the
    output transistor may sink harder than a leaking clamp sources. Only the weak
    pull-down, fighting pad leakage alone with nothing else on the net, separates
    a live pad from a blown ESD clamp to VDD. `af 2` persisting on the PB7 line
    while MODER selects input is not a contradiction — AFR simply retains its
    value and is ignored outside AF mode.
  - **PMODE strapped: 10 kΩ from pin 16 to 3V3, fitted.** The value is the Sep 16
    calculation, not a guess — 100 kΩ against the internal 156 kΩ/44 kΩ divider
    reaches only ≈1.66 V, 160 mV over `V_TIH`. **Not yet confirmed latched.**
    PMODE is sampled at nSLEEP rising, so the strap proves nothing until
    `drv disable` → `drv enable` with a scope on IN1/IN2 — and that waits on the
    ground return, which must be in before any signal wiring goes back on.
  - **VREF (pin 5) confirmed bare — no resistor fitted, and none should be.**
    This matches the record: the carrier's stock 10 kΩ nSLEEP→VREF was removed on
    Sep 12, and VREF is driven by PA4/DAC1_OUT1 alone. The question arose from the
    three *internal* 100 kΩ pulldowns in the datasheet (nSLEEP, and PH/IN2 in all
    three PMODE modes) and the rejected 100 kΩ PMODE value — none of which are
    fitted parts on this net.
  - **Considered and not adopted Sep 19: a 100 kΩ pull-down on VREF.** Two costs,
    one of them severe:
    - **It would corrupt the plateau sweep, which is the next experiment.** That
      sweep exists to determine whether VREF is compared directly or through an
      internal divider (k = 1/2/3). An external pull-down divides VREF by a factor
      the firmware does not model, and is therefore **indistinguishable from the
      internal divider it is trying to measure** — the measured k would be partly
      the resistor's, and would then be baked into every current figure taken
      afterwards.
    - **It loads the unbuffered DAC.** Buffered, output impedance is a few ohms
      and 100 kΩ draws ~33 µA — irrelevant. But `drv trip buf off` is on the table
      to reclaim 4.67 A → 4.98 A, and the unbuffered DAC output is high-impedance
      (tens of kΩ — check the F446 datasheet for the exact `R_O`), against which
      100 kΩ is a real divider biasing the trip low.
    - **The argument in its favour, which is real:** removing the 10 kΩ removed
      the fail-safe, so if the DAC peripheral is not running PA4 is high-Z and
      VREF *floats* — the PMODE mistake repeated on an analog pin. `isense_init()`
      sets VREF at boot, so the hole is narrow. If a defined fail-safe is ever
      wanted, **1 MΩ** buys it at a tenth of the loading and keeps `buf off`
      usable. Either way, meter PA4 against the commanded value before trusting a
      trip figure.
  - **Still open at session end:** PB7 was left in input/pull-down mode by the
    `pd` test and needs `drv pin af` before PWM can reach the driver; the ground
    return is untouched; the PMODE latch is unconfirmed.

- **Sep 16 — PMODE was never strapped, and it cost the MCU. PB7 is destroyed,
  the board is being replaced, and the session STOPPED at the damage by bench
  rule.** The pin that selects the DRV8874's control mode has been physically
  unconnected since the driver was wired on Sep 11. It is a **tri-level** input
  with an internal 156 kΩ to an internal 5 V over 44 kΩ to GND, so an open pin
  self-biases to ≈1.1 V — dead centre of the Hi-Z band. **Floating is a
  selected mode, not an absent one.**
  - **The authoritative table (SLVSF66A, Table 2)** — this closes the open item
    carried in `Motor_Driver_Selection.md` since Aug 25: **PMODE logic LOW →
    PH/EN; logic HIGH → PWM (IN1/IN2); Hi-Z → independent half-bridge.**
    Thresholds `V_TIL` 0–0.65 V, `V_TIZ` 0.9–1.2 V, `V_TIH` 1.5–5.5 V, so 3.3 V
    is a valid high. **10 kΩ to 3V3** lands the pin at ≈2.8 V against the
    internal divider; **100 kΩ reaches only ≈1.66 V** — 160 mV of margin over
    `V_TIH`, which is not enough.
  - **The mode is LATCHED at nSLEEP rising** (§7.3.2), not sampled continuously.
    Changing the strap does nothing until `drv disable` → `drv enable` or a
    power cycle, which makes "I changed it and nothing happened" a false
    negative.
  - **Why five days of correct-looking results hid it.** In independent
    half-bridge mode each output simply follows its own input (Table 5). Slow
    decay holds IN1 high and PWMs IN2 inverted, so the motor sees OUT1−OUT2 =
    0 V for (1−D) and +VM for D — **the same average as PWM mode**. The Sep 12
    figure of 11.07 rpm at 20% duty is therefore consistent with *both*, and the
    Sep 14 conclusion that PMODE was confirmed in PWM mode is **wrong**: it
    correctly ruled out PH/EN and then treated the remainder as proven, never
    excluding Hi-Z. Corrected in task 6b below.
  - **This also explains the Sep 15 current-sense anomalies — the decay phase
    was on the wrong side of the bridge.** The two truth tables differ in
    exactly one state, and it is the one slow decay spends most of its time in:

    | IN1, IN2 | PWM mode (Table 4) | Independent half-bridge (Table 5) |
    |---|---|---|
    | 1, 0 | OUT1 H, OUT2 L — forward drive | OUT1 H, OUT2 L — forward drive |
    | 1, 1 | OUT1 L, OUT2 L — **low-side** slow decay | OUT1 H, OUT2 H — **high-side** slow decay |

    The drive phase is identical, which is why the rpm looked right. The
    recirculation path is not: it moved from the bottom of the bridge to the
    top. **IPROPI mirrors only the low-side FETs, and only drain→source**
    (§7.3.3.1), so in the intended low-side decay the recirculating current
    still passes through a sensed FET — TI sells this as *"continuous current
    monitoring"* across both drive and brake. In high-side decay nothing
    touches a low-side FET, and **IPROPI reads exactly zero for the whole decay
    phase**. The Sep 15 `drv iscan` result — *"all 32 points outside the drive
    phase read exactly 0"* — was therefore not only a clean confirmation that
    the TIM4_CH4 trigger was aimed correctly; it was **also the signature of the
    wrong control mode**. In PWM mode those points should have been non-zero and
    decaying.
  - **The leading dead zone inside the drive window has a candidate: `tDELAY`.**
    Current sense delay is **1.6 µs typical**, and §7.3.3.1 says it has no
    impact *provided* "the low-side MOSFET sensing the current is continuously
    on" — true in low-side decay, false in high-side decay, where the sensed FET
    switches off every period and the mirror restarts each cycle. At 13% duty
    the drive window is ticks 3915..4500 (585 ticks, 6.5 µs) and 1.6 µs is 144
    ticks, i.e. settling until ≈4059. The measurement has the reading still at
    raw 8 at tick 4080 and at 771 by 4110 — **a good match for the first dead
    stretch**. It does *not* explain the later two bumps or the dead valley at
    4200-4260 where the geometric-centre trigger lands; that remains open, and
    must be re-measured in PWM mode before any more effort goes into it.
  - **One correction to the "sum of both currents" concern:** IPROPI reports the
    sum only when both low-side FETs conduct *simultaneously*, which this slow-
    decay pattern never does (its decay phase has both HIGH sides on). Summing
    was not a factor in the Sep 15 readings.
  - **"Fast decay" was not fast decay either.** `drive.c` idles the undriven
    input at 0 and PWMs the other, so the off phase is IN1 = IN2 = 0 — Hi-Z
    **coast** in PWM mode, but **both low-sides on, a brake**, in independent
    half-bridge. Fast decay therefore never coasted, and because its brake phase
    *is* on the low side, IPROPI could see it. Fast and slow decay had their
    sensing behaviour inverted relative to what the firmware assumed. Every
    fast-decay result is suspect on top of the regulation ones.
  - **A floating tri-level input does not latch the same way every time.** This
    session opened with `drv enable` then `drv duty 13`, and the wheel went to
    full speed. That is the **PH/EN signature** — EN held permanently enabled,
    PH toggling at 20 kHz, giving |2D−1| ≈ 74% of the rail instead of 13%.
    Earlier power-ups behaved as Hi-Z. One unconnected pin latching differently
    across power-ups is the only thing that explains both.
  - **PB7 is dead — pad shorted to VDD, proven by register readback rather than
    inference.** A temporary **`drv pin`** console command dumps GPIOB
    MODER/AFR/PUPDR/IDR for PB6+PB7 alongside TIM4 `CR1`/`CCER`/`CCMR1`/`CCR1`/
    `CCR2`, and can force PB7 out of AF to a plain GPIO or to an input with the
    internal pull-down. The chain that settled it: both channels at `mode 2
    af 2`, `CCER 0x1011` (CC1E+CC2E+CC4E set, no polarity bits), `CCMR1 0x6868`
    — **byte-identical halves**, OC1M = OC2M = PWM1, both preloaded — and
    CCR1 = CCR2 = 0, yet PB6 read `IDR 0` and PB7 read `IDR 1`. Reconfigured as
    an input with the internal ~40 kΩ pull-down, **and with every wire removed
    from the pin**, PB7 still read 1. No register difference, nothing external,
    pad will not go low. The MCU also ran hot, which is the 3V3 rail feeding
    that clamp back out through the pin's own low-side transistor every time the
    timer drove it low.
  - **The likely killer is the ground return, not PMODE itself.** `PH/IN2` is an
    *input* with a 100 kΩ internal pulldown in **all three** PMODE modes
    (datasheet pin table: *"PH/IN2 pins have an internal pulldown resistor to
    ensure the outputs are Hi-Z if no inputs are present"*), so no mode
    selection can source current back into PB7. What the mis-latch did was turn
    a 13% command into ~74% of the rail — roughly a sevenfold step in return
    current through a **single DuPont wire** between breadboard PGND and MCU
    ground. When the power-ground path is worse than the signal path, return
    current comes home through the signal wires, and current *into* a GPIO pad
    forward-biases its clamp into VDD. The DRV8874's logic pins are rated to
    5.75 V while the STM32 clamps at VDD+0.3, so the MCU always dies first —
    consistent with the driver appearing to have survived.
  - **Found in the cheapest possible place.** One bench MCU, rather than six
    assembled boards each missing a pull-up resistor. The strap is now a
    per-board schematic item, not a bench workaround — see task 19.
  - **Not done, deliberately:** nothing was rewired after the fault was
    confirmed. The Sep 15 scope-on-PA2 plan is untouched and still queued behind
    the board swap.

- **Sep 15** — **PWM-synchronised current sampling: the trigger works, the
  signal does not cooperate. STOPPED HERE; next session starts with a scope on
  PA2.** TIM4_CH4 now raises a compare event at the middle of the drive phase
  and the ADC converts on it. `drive.c` owns the placement because it is the
  only module that knows which end of the period is driven — in slow decay the
  driven window is the TAIL, `[ccr, ARR]`, so at 13% duty it is ticks
  3915..4500 and the trigger sits at 4207. CH4 is configured in `drive_init()`
  rather than the `.ioc`, for the same reason the PWM is started there. CH4 is
  PB9 = CAN1_TX (AF9), so enabling CC4E routes nothing to the pin; the event is
  internal.
  - **The trigger is verified correct.** A new diagnostic, **`drv iscan
    [n] [from] [to] [step]`**, sweeps the sample point across the PWM period
    and prints the ADC reading at each tick. Across the full period at 13%
    duty, **all 32 points outside the drive phase read exactly 0 and every
    point inside it is non-zero** — placement, blanking and phase geometry all
    confirmed in one measurement. ⚠️ **Re-read Sep 16:** the trigger placement
    conclusion stands, but the zeros were *also* the mis-latched control mode —
    a high-side decay phase is invisible to IPROPI. In PWM mode those points
    should read non-zero. See the Sep 16 entry.
  - **But IPROPI is not flat inside the drive window.** A 20-point sweep across
    ticks 3900..4500, repeated three times, reproduces the same shape every
    time: three bumps separated by hard zeros, peaking at **raw ~730-770 (≈1.1 A)
    around ticks 4110-4140**, with a dead valley at 4200-4260 — which is
    precisely where the geometric-centre trigger lands. That is why `drv
    current` reported raw 1-22 while the peak nearby was 40× higher. Between
    ticks 4080 and 4110, 0.33 µs apart, the reading goes 8 → 771.
  - **A current that returns to zero three times in 6.5 µs is not the motor's
    current** — L/R is 0.90 ms, so the real ripple at 13% should be ~31 mA on a
    near-DC level. The structure is reproducible and PWM-phase-locked, so it is
    neither noise nor a firmware defect, but whether it is mirror blanking, a
    regulation retry, or ringing on the net is **not yet known**. ⚠️ **Partly
    answered Sep 16:** a regulation retry is ruled out — regulation was disabled
    the whole time — and the leading dead stretch matches the 1.6 µs `tDELAY`,
    which only applies because the decay phase was high-side. The remaining
    structure must be re-measured with PMODE strapped before it is worth
    chasing.
  - **The mean over the drive window is physically sensible**: averaging the
    in-window points gives raw 100/138/149 across the three sweeps, i.e.
    ~120-180 mA of motor current, consistent with August's 154 mA no-load
    figure. So the information is present and it is the single-point sampling
    strategy that is wrong. Swept-phase averaging is the obvious candidate, but
    it should not be built until the waveform is understood — averaging over an
    unexplained artifact would produce a plant model that fits the artifact.
  - **NEXT SESSION, FIRST STEP** — ⚠️ **BLOCKED Sep 16: PB7 is destroyed and
    cannot be used as a trigger on this board; and every reading below was taken
    with the driver in independent half-bridge mode, so the waveform itself may
    not reproduce once PMODE is strapped. Re-run this only after task 19.**
    Scope **PA2 (IPROPI)** with **PB7 (IN2)** as
    trigger (IN2 falling = start of drive phase), ~1 µs/div, wheel at 13% duty
    slow decay. Flat level → the structure is a sampling artifact after all;
    spikes with hard zeros → the mirror genuinely does this; decaying ringing →
    layout, and an RC on IPROPI is the fix.

- **Sep 15 — RETRACTION: the "free-running sampler aliases" diagnosis does not
  hold.** It was raised on the strength of three low-count readings (raw 16/1/12
  at 12/13/14% duty) and does not survive the Sep 12 stall test, where the same
  free-running sampler returned **189 and 190 mA on repeat** — 0.5%
  repeatability, which a badly-aliasing sampler cannot produce. The real pattern
  is **steady at high signal, scattered at low signal**, which is a different
  problem with a different cause. The `isense.h` note written under the aliasing
  hypothesis overstates the case and should be read with this entry beside it.
  Nothing measured before Sep 14 needs re-taking on aliasing grounds; the low-
  current readings are still suspect, but for the reason above.

- **Sep 15 — the free motor and the wheeled motor are different mechanical
  systems, and August's dynamics do not transfer.** Standing rule, after
  repeatedly comparing the two: what carries over is **motor-owned and
  electrical** — R = 1.90 Ω, L = 1.70 mH, Kt, Ke, 8403.2 counts/output-rev, and
  the under-2% match between units. What does **not** carry over is
  **system-owned and mechanical** — breakaway duty, dropout duty, the speed
  floor, friction and its Stribeck shape, inertia, and the 154 mA no-load
  current, all of which were measured on a free shaft and now have a kilogram of
  wheel on them. The subtlety that catches this out: in
  `ω = k·(V·D) − (R/(Kt·Ke))·τ` **both coefficients are motor constants**, so the
  wheel does not tilt the speed-torque line, it moves where you sit on it. That
  is the actual justification for characterising the bare wheel first.
  - **R_motor does not need re-measuring** — it was bench-measured Aug 25 on two
    units at 1.90 Ω and winding resistance is a property of the motor whatever
    is bolted to the shaft. Two stale "measure the winding resistance" open
    items that prompted asking again have been closed in
    `Motor_Driver_Selection.md`, and `CQR37D12V64EN-M_Drive_Motor.md` now
    carries the measured 1.90 Ω beside the datasheet-derived 2.18 Ω it was
    written against. A stall-derived estimate of ~4.1 Ω computed this session is
    **withdrawn** — it was built on the same suspect low-current readings.

- **Sep 15 — loaded-rig mechanical characterisation (encoder data, trustworthy;
  the current readings from the same sweep are not).** With the wheel mounted:
  **breakaway 12-14% duty**, position-dependent between runs minutes apart (the
  131:1 gearbox means the wheel's resting angle does not pin the rotor's);
  **dropout ~10.5%** on the descending sweep. Speeds fall linearly 20%→12% at
  ~0.68 rpm/%, then bend hard and cliff: 11% → 4.86 rpm, 10% → dead. **Minimum
  sustainable speed ≈ 4.9 rpm** — no duty produces 2, 3 or 4 rpm. That is the
  Stribeck signature, and it is a **W4-time discovery that constrains the
  demo's slowest manoeuvre**; the fix is mechanical or gearing, not control.
  The 1-3 point hysteresis band is narrow enough that a conventional PID
  suffices without dither or breakaway kicks. **Wheel diameter still needed** to
  convert 4.9 rpm to a linear speed.
- **Sep 14 (later)** — **Module identity implemented and verified; W7's
  firmware dependency is closed.** `dipsw.c` reads PB13/PB14/PB15 once at boot
  and latches. All eight codes swept on a board with a real switch block: every
  bit maps correctly with PB13 as LSB, every role boundary lands where the
  design says, and the CAN ID tracks `0x500 + id` across the range. The latch
  held at ID 0 while the pins read 1, reporting the divergence instead of
  acting on it — which is the invariant that stops two boards sharing an
  address mid-run. The heartbeat now uses the module ID, and identity gates the
  *transmit* as well: at code 7 it reports `NO ID` with `lec none`, proving
  nothing reached a mailbox. PB14/PB15 are configured by `dipsw_init()` rather
  than CubeMX, for the same reason `drive_init()` starts its own PWM. The
  documented halt-on-invalid is deferred to W7 — it would remove the console to
  prevent a collision the transmit gate already prevents. Also established that
  `NO MAILBOX` after exactly three frames is the lone-node signature of
  `AutoRetransmission = ENABLE`, not a fault.

- **Sep 14** — **`config` module verified on hardware; two bugs found and
  fixed.** Eight bench checks, all passing: boot scan on a blank sector, the
  save/reset/read-back round trip, range rejection, unchanged-detection with no
  slot burned, revert and default, the save refusal while the bridge is
  enabled, and the live-apply coupling both ways. The post-refactor ceiling and
  trip came back byte-identical to the pre-refactor values, proving the
  macro-to-function move was transparent. **A reflash does not erase sector 7**,
  so per-node calibration survives firmware updates — load-bearing for W7.
  Both bugs were the same class: a consistency check comparing two values in a
  space where they were not comparable. The `drv trip` hint compared milliamps
  when one side had been through DAC quantisation and the other had not, so it
  fired every time; `cfg revert` replaced stored values without re-applying
  them, so the display could claim a 40% duty cap while the bridge enforced
  100%. Known untested: the corrupt-record fallback and the sector-full wrap.

- **Sep 13** — **`config` module built: the tunables now live in FLASH.**
  Eight keys (`vdda_mv`, `r_ipropi`, `a_ipropi`, `trip_ma`, `duty_limit`,
  `rail_mv`, `isense_avg`, `sat_raw`) moved out of `#define`s and into sector 7
  as an append-only log of CRC'd records — 1024 saves per erase, newest valid
  `seq` wins, and a save interrupted by power loss fails its own checksum with
  the previous record still live. The compiled numbers stayed in `isense.h` as
  `_DEFAULT`s so the reasoning stays next to the hardware it describes.
  `isense_full_scale_ma()`, `isense_raw_to_ma()` and the two VREF conversions
  stopped being macros/inlines — an inline would have frozen the build-time
  default, which is precisely what this module exists to undo.
  Safety: out-of-range rejected rather than clamped (a clamp hides the typo),
  per-key range check on load as well as on set (a valid CRC says the bytes
  survived, not that the number still makes sense), fall back to defaults
  *loudly* on the boot line, and **`cfg save` refused while the bridge is
  enabled** — flash writes stall instruction fetch for up to 3 s and a turning
  motor keeps turning open-loop through the whole stall. `cfg trip_ma` and
  `cfg duty_limit` apply live too, and `drv trip` / `drv limit` now flag
  themselves as non-persistent only once they have actually diverged.
  Builds clean at 84 KB of 384 KB; **nothing bench-verified yet.**

- **Sep 12 (4)** — **The motor rail goes 9.5 V → 12 V, and a `config` module is
  required.** **(1) Rail.** 9.5 V was never a motor requirement — it was the
  DRV8833's 10.8 V ceiling with margin, and it survived the DRV8874 swap on
  Aug 25 for a reason that has since expired (current regulation being
  unproven; it is now built and metered). The motor is a 6 V/**12 V** unit, so
  9.5 V gives up ~21% of its speed and, more usefully, torque headroom at
  speed. **Nothing about the architecture changes:** the per-motor step-down on
  each node PCB stays, fed independently from the >13.5 V rail; only its output
  set-point moves to 12 V. **One number to keep straight** — the datasheet
  stall at 12 V is **5.5 A**, while this project's own bench measurement
  (Aug 25, two motors, three methods) gives **6.3 A cold**. Both are right:
  5.5 A is a warm winding, and copper falls ~15% in resistance between hot and
  cold. Setting `drv trip 5000` makes the distinction moot and caps it below
  the DRV8874's 6 A peak either way. Note the trip then becomes the binding
  limit on **peak** torque, so the gain from 12 V is speed and torque *at
  speed*, not a higher stall torque. **(2) A `config` module is required** —
  see task 18. Constants like `ISENSE_R_IPROPI_OHM` compiled into
  the image mean an edit-build-flash cycle to change a number, no way to differ
  between the seven nodes without seven builds, and no way to know what is
  actually in a device without reading source at the matching commit. Values
  move to FLASH, editable from the serial console, with the compiled-in values
  demoted to *defaults*.
- **Sep 12 (3)** — **Three things closed in one bench session: the CubeMX
  blocker, a silent regression it caused, and the IPROPI question.**
  **(1) The `.ioc` was invalid, not corrupted.** PA15 had been red and
  unclickable in CubeMX for two sessions; the cause was that the file used
  signal names that do not exist in the device DB — `S_TIM2_CH1` instead of
  **`S_TIM2_CH1_ETR`**, and bare `TIM4_CH1`/`TIM4_CH2` instead of
  **`S_TIM4_CH1`**/**`S_TIM4_CH2`** — with the required `SH.*` shared-signal
  blocks missing entirely. CubeMX silently dropped TIM2 and TIM4 on every load
  and left PA15 `Locked=true` pinned to nothing. Fixed, regenerated, ADC1 + DAC
  now generate, build clean.
  **(2) The regeneration then silently ate two USER CODE blocks** —
  `TIM4_Init 2` and `TIM2_Init 2` — which were the only places
  `HAL_TIM_PWM_Start()` and `HAL_TIM_Encoder_Start()` were called. The motor
  went dead with **`nFAULT clear`, no build error and no warning**: the timers
  simply were never running. Diagnosed from the console line
  `I 0 mA (raw 0)` on an awake, unfaulted driver. Both starts now live in
  `drive_init()` / `encoder_init()`, files CubeMX never touches, so they cannot
  be lost that way again.
  **(3) IPROPI is decoded — it reports SUPPLY current**, `I_motor × D`; the
  20 kΩ IMODE strap blanks the mirror during slow-decay recirculation. Settled
  by stalling the output shaft, which removes back-EMF and makes the motor
  current pure Ohm's law: at 20% duty the two hypotheses predicted **984 mA**
  (continuous) versus **197 mA** (supply), and the bench read **189 / 190 mA**.
  A 5× discriminator with no friction model in the way. **The consequence that
  bites:** the trip regulates *motor* current (correctly — `drv trip 3000` does
  limit the motor to 3 A) while `drv current` reports the duty-averaged
  *supply* figure, so a plateau sweep plateaus at `trip² × R_motor / Vm`, not
  at the trip. `drv current` now prints `Isup` and the implied `Imotor`.
  Also fixed: 40 em dashes in string literals that the serial terminal rendered
  as empty squares. **Still pending:** `ISENSE_VDDA_MV` 3300 → **3325** and
  `ISENSE_R_IPROPI_OHM` 1474 → **1465**, both measured, both deliberately held
  until after the plateau sweep so nothing changes mid-experiment.
- **Sep 12 (2)** — **The bench carrier was modified and VREF moved under software
  control.** Lifted the 10 kΩ between nSLEEP and VREF, and replaced the 2.48 kΩ
  R_IPROPI with **2.0 kΩ ∥ 5.6 kΩ = 1.474 kΩ**. Result: the ADC ceiling and the
  regulation trip are **no longer the same number** — R_IPROPI alone fixes the
  ceiling at **4.975 A**, and **PA4/DAC1_OUT1** sets the trip anywhere below it,
  at 1.215 mA per DAC code (exactly one ADC LSB). Motivation was the 5.0 A
  stall, which the stock 2.957 A carrier could neither measure nor permit.
  Seven spare carriers remain stock. Firmware: DAC added to the `.ioc`, VREF
  control folded into `isense` (same module, because trip and measurement are
  the same arithmetic and two copies of the scale constant would drift), new
  `drv trip [<mA> | buf on|off]` console command, boot line reports ceiling and
  trip separately. **Two new standing rules:** VREF must be set before nSLEEP
  rises (the 10 kΩ used to guarantee that; `isense_init()` now does), and the
  buffered DAC ceiling is **4.67 A**, below the 4.92 A stall — `drv trip buf off`
  reclaims it but needs a meter on PA4 to confirm the unbuffered output holds.
  **Still does not build until CubeMX is regenerated** (ADC1 + DAC).
- **Sep 12** — **The DRV8874 carrier's three straps were measured, and they settle
  more than the scaling.** R_IPROPI is **2.48 kΩ** (the selection doc's 2.2 kΩ was
  an assumption), IMODE has **20 kΩ to GND**, and **nSLEEP feeds VREF through
  10 kΩ** — so VREF sits at ~3.3 V whenever the driver is awake. Consequences:
  current sensing works out of the box at **1.116 V/A, 2.957 A full scale**;
  the chip now enforces a **hardware current limit at that same 2.957 A**, which
  the DRV8833 never had, so a stalled motor regulates instead of pulling its 5 A
  stall; full scale and the trip point are one knob, not two; and the planned
  **PA4/DAC1 software-settable VREF is foreclosed** on this carrier, one lifted
  resistor away from being possible again. Firmware: new **`isense` module**
  (`isense.h`/`isense.c`) on **PA2 / ADC1_IN2**, with `drv current [n]` and
  `drv zero` console commands; `drv current` prints duty and decay mode on every
  line so logs stay valid whichever way the IPROPI recirculation question
  resolves. **The firmware does not build until CubeMX is regenerated** — the
  `.ioc` has ADC1, the generated HAL does not. Still open: decode the 20 kΩ
  IMODE strap; confirm the PMODE strap selects PWM mode; confirm the nFAULT
  pull-up is fitted.
- **Sep 11** — The three-week gap in this log was bench and delegation work, not a
  stall. Four things changed, and two of them close open W4 items outright:
  **(1) All seven motors** (six wheels plus the spare) have had their encoders
  checked and their cable extensions **re-terminated with crimped joints to
  NASA-STD-8739.4A**. The broken-VCC conductor that killed two encoders on
  Aug 25 is now ruled out fleet-wide rather than assumed. **(2) The DRV8874
  arrived early** — before the ~Sep 15 estimate — so the interim DRV8833 and
  its ~1.7 A ceiling stop constraining the bench. **(3) A loaded wheel test rig
  was designed and built**: a static base that works like a treadmill belt under
  a single wheel, so the wheel can be driven **while carrying real weight**.
  This is a material upgrade to W5 — every plant figure on record was taken on
  a free shaft, and PID tuned against a free shaft does not transfer to a loaded
  one. **(4) Node PCB design (HW1/HW2) has been delegated to a student**,
  working since roughly Aug 28. Expected to be slow, but it runs in parallel
  and off the critical path. **Still not done: the ANT CNC parameters.**
- **Aug 26 (late)** — Motor terminal voltage measured: **9.45 V supply → 9.35 V
  at the motor**, so the rail was never near the DRV8833's 10.8 V ceiling and
  the motor is simply ~11% faster than its datasheet. Gives **Ke ≈ 0.138 V/rpm**.
  **Stop policy decided: coast by default, brake only below ~40 rpm** — braking
  from full speed draws 4.7 A, above the carrier's peak. Brake current
  circulates locally through the driver rather than through the branch return,
  so it is *less* exposed to the documented ground-loop mechanism than driving
  is, but it puts high current across the local star point.
- **Aug 26 (evening)** — **First powered motion**, free shaft, console-driven.
  Plant is strikingly linear: **rpm = 0.672 × duty% − 1.8**, per-step
  increments varying only ±1.5% across 20–100%, so W5 needs no gain
  scheduling. **Sign convention settled: positive duty → CW → positive
  counts**, no lead swap needed. **Drive scheme closed — slow decay
  (drive-brake) wins decisively**: its deadband is ~2.6% duty against fast
  decay's >20%, which would not even break static friction at 20%. Two smaller
  findings: a consistent ~3.5% CCW-faster direction asymmetry (brush timing,
  normal, absorbed by integral action), and every velocity reading landing on
  an exact multiple of the designed 0.357 rpm quantum.
- **Aug 26 (later)** — **PWM scope-verified with the motor disconnected**, the
  gate before any actuator is wired. All four quadrants correct: in fast decay
  the driven pin carries the PWM and the other sits low; in slow decay the
  other sits high and the PWM'd pin is inverted. Brake and coast as expected.
  Period measured with cursors at 50.0 µs = **20.000 kHz**, and duty at 25.2 %
  and 79.9 % for 25 % and 80 % commanded. Two scope readings that initially
  looked wrong — 19.92 kHz, and a duty compressed toward 50 % when derived from
  V_avg/V_rms — were both instrument error, not firmware; see drive.h.
- **Aug 26** — **W4 ACCEPTANCE CRITERION MET.** Encoder firmware written (TIM2
  delta accumulation, TIM6 1 kHz tick, `enc`/`drv` console commands) and
  verified on the bench: ten hand turns of the output shaft gave 83 949 counts
  = **8394.9 per revolution against the predicted 8403.2, 0.1% low**. Both the
  64 CPR and 131.3:1 figures are confirmed; no constant changes. `enc probe`
  independently confirmed x4 decoding (393 + 393 edges vs 778 net counts), and
  the accumulator matched the raw counter difference exactly across 84 000
  counts — the delta arithmetic loses nothing.
- **Aug 25–26** — **Encoder dead on two motors, root-caused to a broken VCC
  conductor** in cable extensions a student had soldered. Both lines sat at a
  constant 1.8 V (a ~10 kΩ resistive divider against the 10 kΩ pull-ups, ratio
  unchanged at 5 V) — the signature of an unpowered IC, not a switching output.
  Highly likely the other five motors share it. See KEY LEARNINGS.
- **Aug 25 (later)** — Motor bench-measured on two units: R ≈ 1.90 Ω (stall,
  supply-sag corrected), L ≈ 1.70 mH, no-load 154/155 mA at 9.5 V, and the two
  motors match to under 2% on every parameter that matters — so one PID gain set
  should fit all six wheels. **Motor rail set at 9.5 V**, which puts stall at
  5.0 A inside the DRV8874's 6 A peak; the second buck therefore stays in HW1.
  L/R = 0.90 ms confirms 20 kHz PWM gives ~99 mA ripple.
- **Aug 25 (earlier)** — Motor spec obtained and it moved two things. Encoder is
  **64 CPR at the motor shaft**, not the estimated 11 PPR → **8403.2 counts/rev**
  at the output, 45% higher than the 5777.2 on record. And the winding is
  2.18 Ω (cross-checked on both stall points), so stall is 4.1 A at 9 V or
  6.2 A at the 13.5 V branch rail — the motor comfortably out-muscles the
  DRV8833. **Driver changed to DRV8874** (4.5–37 V, 6 A peak, 200 mΩ,
  VREF-programmable current regulation, IPROPI current feedback). Bought 8
  carriers + 10 bare ICs, arriving ~Sep 15. DRV8876 kept as the pin-identical
  second source; DRV8871 and DRV8833 ruled out. Full four-way comparison in
  `docs/research/drive-motor/Motor_Driver_Selection.md`.
- **Aug 24–25** — W4 opened. Node pin map settled and built clean: heartbeat
  moved off TIM3 to TIM7 (basic timer, no pins, identical 0.5 s tick) to free
  TIM3's encoder interface — then the encoder was moved again to **TIM2**, the
  only 32-bit counter available, on PA15/PB3. Rollover horizon goes from 11.3
  to ~743,000 output revolutions. Discovered along the way that encoder mode is
  CH1+CH2 only, in silicon, so TIM2_CH3/CH4 (PA2/PA3) could not be used.
  DRV8833 carrier characterised from its vendor page and confirmed on the
  bench: J1 cut so nSLEEP is MCU-driven, external pull-up on nFAULT, all four
  DRV pins verified at their off levels. Current sense pins are grounded on
  this carrier, so **there is no current feedback and no hardware current
  limit** — W5's PID is velocity-only. Paralleled current budget corrected
  down from 3 A RMS to ~2 A.
- **Aug 13** — W3 acceptance criterion MET: STM32 commands the SERVO42C to a
  target angle, confirmed against its own encoder, round-trip repeatability
  1/10 of a microstep. Three device/HAL traps found: driver echoes every
  request, idle-line framing doesn't work on this link, clearing UART error
  flags steals a byte from the DMA.
- **Aug 11** — Bus-load ramp to saturation: no FIFO overrun at any rate,
  main-loop period bounded under 1.6 ms. Polled vs. interrupt-driven CAN RX
  left as an OPEN question, not settled by this result.
- **Aug 10** — W2 COMPLETE: STM32F446RE heartbeat crossing a real 250 kbps
  bus to Orion, confirmed independently on the CANable, zero error counters
  both ends. Termination measured 59.79R. Floating CAN_RX pin identified as
  the cause of three different pre-wiring fault signatures.
- **Aug 9** — W2 firmware written and building: DMA console on USART1, bxCAN
  driver at 250 kbps with accept-all filter, serial command interpreter for
  bench work. PLL divider record corrected to M=4/N=180; MCO2 pin frequency
  clarified.
- **Aug 9 (earlier same day)** — W2 STM32 toolchain established: STM32CubeIDE
  replaced by CubeMX + CMake + CubeCLT + VS Code on daedalus; full flash/
  debug chain verified against a WeAct STM32F446 Core Board; 8 MHz HSE →
  180 MHz PLL reconfirmed; module identity settled as a 3-bit DIP switch
  read at boot (one binary serves all six modules).
- **Aug 6** — CAN bus W1 COMPLETE: MKS CANable V2.0 Pro flashed to
  candleLight, SN65HVD230 wired to J17, two-node physical bus validated at
  250 kbps with 500-frame load test, pinmux + can0 config made persistent via
  systemd. Also recovered the missing MKS SERVO42C UART protocol + torque
  characterization from the June 8, 2026 bench session (previously
  undocumented here).


## NEXT TASKS — completed items, moved out of the context file Sep 18, 2026

These were finished. They live here so the context file's NEXT TASKS list holds
only work that is actually still open. Original numbering kept.

### 6. CAN Bus — STM32 firmware (COMPLETED August 10, 2026)

6. **CAN Bus — STM32 firmware:** ✅ **COMPLETED August 10, 2026** — this is
   roadmap W2. Full log in the Aug 10 verification section above.
   - ✅ bxCAN at 250 kbps (BRP 12, BS1 12, BS2 2, SJW 2, 86.7% sample point)
   - ✅ Accept-all mask filter on bank 0, `HAL_CAN_Start()`, heartbeat on 0x500
   - ✅ DMA console + command interpreter for bench work (see FIRMWARE MODULES)
   - ✅ `loopback on` + `send 123 DEADBEEF` self-test round-tripped with nothing
     attached — proves bit timing, filter and FIFO independently of any wiring
   - ✅ Termination measured 59.79R with all three nodes connected
   - ✅ Three-way verification: STM32 → Orion `candump`, confirmed independently
     on the CANable, 17 frames, zero error counters on both ends
   - Open, non-blocking: `cmd_errors` ESR snapshot (see Known issue above)
   - **Open decision:** polled vs interrupt-driven CAN RX — the Aug 11 load ramp
     bounds throughput only, not latency. Hybrid ISR-to-ring is the leading
     candidate. Settle before W6.

### 6b. W4 — drive motor + encoder closed loop, the completed part

The task itself is still in progress; this is the full ✅ history as it stood on
Sep 18, 2026, with the open items left behind in the context file.

6b. **W4 — drive motor + encoder closed loop (IN PROGRESS, opened Aug 24):**
    Acceptance criterion: encoder counts are read correctly and match physical
    rotation. Note this needs **no motor power at all** — turning the wheel by
    hand is enough, and it also settles the 11 PPR question W5 depends on.
    - ✅ Pin allocation settled, `.ioc` and generated code building clean
    - ✅ Heartbeat moved TIM3 → TIM7; TIM2 chosen for the 32-bit encoder
    - ✅ DRV8833 carrier characterised; J1 cut, nFAULT pull-up fitted, all
      four DRV pins verified at their off levels on the bench
    - ✅ TIM6 control-loop tick at 1 kHz (PSC 89 / ARR 999, own ISR)
    - ✅ Encoder read + `int32_t` delta accumulate, `enc` console commands
      including `enc probe`, which watches the raw A/B pins and TIM2 together
      and localises a fault to either side of the MCU pin
    - ✅ **ACCEPTANCE MET Aug 26** — 8394.9 counts/rev measured over ten hand
      turns, 0.1% from predicted. Sign convention recorded on the bench
    - ✅ PWM helpers, both decay modes reachable. **Drive-scheme choice CLOSED
      Aug 26: slow decay (drive-brake).** Measured deadband ~2.6% duty against
      fast decay's >20% — fast decay would not move the free shaft at all at
      20% duty, because the current collapses to zero each off phase and
      average torque never breaks static friction
    - ✅ **`drv` console commands**, mirroring the existing `mks <sub>` pattern
      in `console.c`'s command table: `drv duty <±pct>`, `drv enable|disable`,
      `drv coast|brake`, `drv status` (duty, nSLEEP level, nFAULT state).
      These exist so the driver can be exercised by hand from the bench
      without a debugger or a reflash, the same way `send` and `mks` do.
    - ✅ **PASSED Aug 26 — PWM scoped with the motor disconnected**, the gate
      between "the code compiles" and "power touches an actuator". 20.000 kHz
      by cursor, duty exact to within one cursor step, all four
      direction/decay quadrants correct, brake and coast confirmed. The
      original checklist follows, kept because it is the right list to re-run
      after any timer change:
      - 20.0 kHz on both channels, and the measured frequency actually matches
        (this is the check that catches a wrong APB1 timer-clock assumption —
        a ×2 error here would show as 10 or 40 kHz)
      - duty tracks the commanded value across 0 / 25 / 50 / 75 / 100 %
      - the two channels are edge-aligned (same timer, same ARR)
      - the commanded drive scheme produces the expected waveform *pair* —
        for drive-brake, one pin sits high while the other is PWM'd; for
        sign-magnitude, one pin is PWM'd while the other sits low
      - direction reversal swaps which pin carries what, with no shoot-through
        window where both go high unintentionally
      - `drv disable` returns both pins to 0 V and drops nSLEEP
      - nSLEEP rises only on a non-zero command and falls again at zero
      Only after all of that passes does a motor get connected.
    - ✅ **DONE Sep 11 — all seven motors checked and re-harnessed.** Every
      motor (six wheels plus the spare) had its encoder verified and its cable
      extension **re-terminated with crimped joints per NASA-STD-8739.4A**.
      The Aug 25 broken-VCC failure is closed fleet-wide, not sampled.
    - ✅ **`DIP_SW_1`/`DIP_SW_2` on PB14/PB15 and the latch-once ID read —
      done and verified Sep 14**, all eight codes. The heartbeat now goes to
      `0x500 + module_id`, and identity gates the transmit. W7 no longer
      depends on anything unwritten in firmware
    - ✅ **First powered motion Aug 26** — free shaft, both directions, full
      duty range, coast and brake all correct. See the plant model below.
    - ✅ **Motor-terminal voltage confirmed Aug 26: 9.45 V supply → 9.35 V at
      the motor**, ~1% drop. The rail is not near the DRV8833's 10.8 V ceiling;
      the motor is simply ~11% faster than its datasheet (7.01 rpm/V measured
      against the spec's 6.33). Ordinary spec conservatism
    - ⬜ Measure real drive current — the input HW4's PDB branch sizing has
      been waiting on. **Two ways in, and neither waits for the DRV8874:**
      (a) measure the motor's winding resistance with a multimeter, rotating
      the shaft between readings to average brush position — if it lands near
      2.18 Ω the whole current analysis is validated; (b) stall the motor from
      a current-limited bench supply with no driver in the loop and read the
      current directly (spec says 2.8 A at 6 V). **Now a third and better
      path exists: measure it on the loaded wheel rig at real weight**, which
      gives the actual duty cycle of operation rather than a bounding figure.
      The DRV8874's IPROPI output makes this a firmware reading, not a
      multimeter session.
    - ✅ **DRV8874 arrived Sep 11 and is wired and running** — IPROPI on PA2
      (`ADC1_IN2`), VREF on PA4 (`DAC1_OUT1`), carrier modified, R_IPROPI
      measured at 1465 Ω. `drive.c` needed no change for the swap, as designed.
    - ❌ **SUPERSEDED Sep 16 — "PMODE confirmed to select PWM mode" was wrong.**
      The Sep 14 reasoning still holds as far as it goes: at 20% duty `drive.c`
      emits IN1 constantly high and IN2 PWM'd at 80% (slow decay), under either
      PH/EN pin assignment one of those is EN and a 20% command would have given
      roughly 50–55 rpm, and the Sep 12 measurement was **11.07 rpm**. That
      rules out PH/EN. **It does not confirm PWM mode**, because the third
      option was never enumerated: in independent half-bridge each output
      follows its own input, so slow decay produces the *same* average motor
      voltage and the *same* 11.07 rpm. PMODE was in fact unconnected — Hi-Z,
      independent half-bridge — the whole time, and internal current regulation
      was therefore disabled, meaning **every `drv trip` / PA4 VREF result taken
      before Sep 16 was inert and must be re-taken**. See the Sep 16 log entry
    - ✅ **`.ioc` root cause found and fixed Sep 12** — the file used signal
      names absent from the CubeMX device DB (`S_TIM2_CH1` for what must be
      **`S_TIM2_CH1_ETR`**; bare `TIM4_CH1`/`CH2` for **`S_TIM4_CH1`/`S_TIM4_CH2`**)
      with the `SH.*` shared-signal blocks missing. CubeMX had been silently
      dropping TIM2 and TIM4 on load since commit `30944da`. ADC1 + DAC now
      generate and the project builds clean
    - ✅ **First motion on the DRV8874 Sep 12** — 20% duty, +11557 counts for
      positive duty (sign convention holds), 11.07 rpm, extrapolating to ~55 rpm
      at the full 9.35 V rail. ADC1/PA2 confirmed live in the same test
    - ✅ **IPROPI decoded Sep 12 — the reading is SUPPLY current**, `I_motor × D`.
      Stalled-shaft test: predicted 984 mA (continuous) vs 197 mA (supply),
      measured **189/190 mA**. See the IPROPI section above for the consequence
      that a plateau sweep plateaus at `trip² × R_motor / Vm`, not at the trip
    - ⬜ **Plateau sweep** — `drv trip 1000 / 2000 / 3000`, stall, step duty up,
      record where the reported current flattens, to settle the internal VREF
      divider (k = 1 / 2 / 3). Immune to both pending constant corrections,
      because measured current and commanded trip pass through the same
      R_IPROPI and the same VDDA and the ratio cancels them
    - ⬜ **Apply the two measured constants after the sweep** (held until then so
      nothing changes mid-experiment): `ISENSE_VDDA_MV` 3300 → **3325**,
      `ISENSE_R_IPROPI_OHM` 1474 → **1465**. Net effect is that readings
      currently sit ~1.4% low
    - ✅ **`config` module verified on hardware Sep 14** — eight checks, plus
      the finding that a reflash preserves sector 7. See NEXT TASKS item 18
    - ✅ **nFAULT pull-up confirmed fitted Sep 14** — `ASSERTED` when shorted to
      GND, clean `clear` when released. MCU side only; that the driver asserts
      on a real fault is still unproven, and UVLO during the rail work is the
      cheap way to close it
    - ⚠️ **Nothing polls nFAULT at runtime.** It is read at boot and by `drv`,
      nowhere else, so a fault that occurs and clears mid-run is invisible.
      `drive.h` defers the policy to W5 deliberately and that is right — but a
      *sticky latch* in the 1 kHz tick is not policy, it is observation, and
      without one the loaded-wheel current measurement could trip a transient
      OCP that leaves no trace. Do this before the rig work

**Superseded Sep 11 — the DRV8874 arrived, so none of this is live any more.** It is kept because the reasoning held: nothing in W4 or W5 was blocked by the wait. W4's
acceptance needs no motor power at all, and W5's PID tuning runs at bench loads
far below the DRV8833's ~1.7 A. The only real collision is HW3 (Sep 7–13),
which mills and populates one reference board — populate everything except the
driver and fit it on arrival. Boards are milled in-house, so this costs a
rework session, not a week.

### 18. `config` module — the full build and verification record

18. **`config` module — BUILT Sep 13, VERIFIED ON HARDWARE Sep 14, 2026**

    Eight checks on the bench, all passing; two bugs found and fixed in the
    process (below).
    `Core/Inc/config.h` + `Core/Src/config.c`, plus a `cfg` console command.
    What exists:

    - **Eight keys**: `vdda_mv`, `r_ipropi`, `a_ipropi`, `trip_ma`,
      `duty_limit`, `rail_mv`, `isense_avg`, `sat_raw`. Each carries name,
      units, min, max, default and one line of help in a single table in
      `config.c`; adding a key is one enum entry plus one row.
    - **Defaults stay in the owning header.** `isense.h` keeps the numbers and
      the reasoning, renamed with a `_DEFAULT` suffix so that reading one at
      runtime — where `config_get()` was meant — reads wrong at the call site.
    - **Sector 7** (`0x08060000`, 128 KB) reserved; `FLASH` in
      `STM32F446xx_FLASH.ld` shortened to 384 KB, with a named `CONFIG` region
      so the map file shows the reservation. Image now 84 KB of 384 KB, and
      objdump confirms nothing is linked above `0x08014818`.
    - **Append-only log**: 48-byte record at a fixed 128-byte stride → 1024
      saves per erase, so the 10k-cycle endurance becomes ~10M saves. CRC is
      the last field and is programmed last, so a save interrupted by a power
      loss fails its own checksum and the previous record stays live. Boot
      scans all 1024 slots (a corrupt magic mid-log therefore cannot hide the
      records after it) and takes the highest `seq` that validates.
    - **Hardware CRC unit driven at register level**, deliberately not through
      CubeMX — adding a peripheral to the `.ioc` means a regeneration, and this
      project has already lost USER CODE blocks to one.
    - **Fails to defaults, loudly.** Corrupt / version-mismatched / out-of-range
      each print on the boot line *and* raise a second WARNING line, so a board
      silently on defaults cannot be mistaken for one deliberately at defaults.
      Out-of-range is per-key: a valid CRC is not a reason to trust a number.
    - **`cfg save` refuses while the bridge is enabled**, mirroring `drv zero`,
      with the reason printed: flash writes stall instruction fetch for up to
      3 s and a turning motor keeps turning open-loop through all of it.
    - **Out-of-range values are rejected, not clamped** — a clamp accepts
      `trip 9000`, silently gives 6000, and hides the typo where it is most
      expensive.
    - **`cfg trip_ma` and `cfg duty_limit` apply live as well as at boot**, and
      `drv trip` / `drv limit` now say "not persistent" *only when* they have
      actually diverged from the stored value. Without this the two commands
      would disagree about the same number until the next reset.

    **Verified Sep 14** — boot scan on a blank sector; the save/reset/read-back
    round trip; range rejection; unchanged-detection (no slot burned);
    `revert` and `default`; the `cfg save` refusal while the bridge is enabled;
    and the live-apply coupling in both directions. The post-refactor ceiling
    and trip came back byte-identical to the pre-refactor values (4975 mA,
    2997 mA), which is what proved the macro-to-function move was transparent.

    **A firmware reflash does NOT erase sector 7** — confirmed Sep 14 by
    flashing and finding the stored record intact. The toolchain sector-erases
    only the regions it writes. This is load-bearing for W7: without it, every
    firmware update would silently wipe each node's calibration.

    **Two bugs found by the verification, both the same class** — a consistency
    check compared in a space where the two sides were not comparable:
    - The `drv trip` "not persistent" hint compared milliamps.
      `isense_trip_ma()` derives back from the DAC code and is therefore always
      quantised; a stored config value is not. It fired on every call, gate
      useless. Fixed by comparing DAC codes, the only space where "would a
      reset change this?" has a yes/no answer.
    - `cfg revert` and `cfg default` replaced the stored values without
      re-applying them, so `cfg` could report a 40% duty cap while the bridge
      still enforced 100% — the dangerous direction. Fixed with
      `cfg_apply_live()`.

    **Still to do:**
    - Two paths remain untested and are known to be so: the corrupt-record
      fallback (needs garbage deliberately written into sector 7), and the
      sector-full erase and wrap at save 1025, which contains the only
      `HAL_FLASHEx_Erase` call. Cheap way to reach the second: build with
      `CONFIG_SLOTS` forced to 4 and wrap it in seconds.
    - Apply the two measured constants through `cfg` rather than a rebuild:
      `cfg vdda_mv 3325`, `cfg r_ipropi 1465` (after the plateau sweep).
    - Set `cfg rail_mv` once task 17's 12 V is metered at the motor terminals.
    - **Add W5's PID gains as keys before tuning starts** — the biggest payoff
      of the module. Tuning a velocity loop without a reflash between trials is
      the difference between an afternoon and a week.


## Verification & characterisation logs

Moved out of `PROJECT_CONTEXT_WHEEL_FW.md` on Sep 18, 2026. These are session
records — the bench runs that established results now summarised in that file.
Nothing here is current state; read it to see *how* something was established.

### Verification log — Aug 9, 2026 (daedalus + WeAct F446 board)

Each step verified before proceeding to the next (W1 method).

```
1. PATH after logout/login
   /opt/st/stm32cubeclt_1.22.0/{STM32CubeProgrammer,STLink-gdb-server,CMake,
   Make,Ninja,st-arm-clang,GNU-tools-for-STM32}/bin   OK all present

2. Tool versions
   gcc 14.3.1 · gdb 15.2.90 · CubeProgrammer 2.23.0 · cmake 4.3.1 · ninja 1.13.2  OK

3. Cortex-M4F compile+link (trivial main, nosys.specs)
   linked against thumb/v7e-m+fp/hard/libc.a   OK hard-float multilib confirmed
   text 5260 · data 1372 · bss 840

4. SVD present · probe enumerated · SWD connect
   STM32F446.svd found                                                    OK
   ST-LINK SN 37FF71064E573436D7331B43, FW V2J46S7, no VCP -> V2 not V2-1  OK
   Device ID 0x421 · Rev A · STM32F446xx · 512 KB · Cortex-M4 · 3.28 V     OK

5. GDB chain (ST-LINK_gdbserver -p 61234 + arm-none-eabi-gdb)
   attach halts core                                                      OK
   sp = 0x20020000  (= 0x20000000 + 128 KB, top of SRAM)                  OK
   xpsr = 0x01000000 (Thumb bit set)                                      OK

6. monitor reset -> "Successfully completed reset operation (System reset)"
   pc = 0x08000b48 after reset
   x/2xw 0x08000000 -> 0x20020000  0x08000b49
   i.e. vector[0] = initial MSP (matches sp), vector[1] = reset handler with
   Thumb bit -> handler at 0x08000b48 = pc.   OK reset-and-halt confirmed
   (Handler sits ~2.8 KB into flash because the vector table reserves ~97
   interrupt entries first. Normal.)

7. Full VS Code round trip: build -> flash -> halt at main               OK

8. Application-level confirmation under the new toolchain:
   MCO1 = 8 MHz HSE at the pin; MCO2 sources the 180 MHz PLL but runs
   through a /5 prescaler, so **PC9 carries 36 MHz** — that reading is
   correct, not a fault. Both scope-verified previously under CubeIDE and
   reproduced identically here, plus TIM3 interrupt-driven
   blinky on PB2.                                                        OK
   -> TIM3 firing at the expected rate validates the APB1 timer clock, and
   **APB1 at 45 MHz is the clock that feeds bxCAN** — so the bit-timing
   divisor chain above is validated on hardware, not only on paper.
```

**Conclusion: the toolchain is no longer a suspect.** Any subsequent failure
belongs to firmware or wiring. Same position W1 left `can0` in, and it is what
makes W3–W5 debugging tractable.

### Verification log — Aug 10, 2026 (W2 acceptance criterion)

**Three independent views of the same 17 frames:**

| Source | Frames |
|---|---|
| STM32 console | `hb 354` … `hb 370` (17) |
| Orion `candump -tz can0` | seq `0x161` … `0x171` (17) |
| daedalus / CANable `candump -tz can0` | seq `0x161` … `0x171` (17) |

No gaps, no duplicates, sequence numbers matching across all three. (The console
prints `seq` after incrementing, so `hb 354` carries payload `0x161` = 353.)

- STM32: `tec 0 rec 0 lec none` throughout
- Orion: `can state ERROR-ACTIVE (berr-counter tx 0 rx 0)`, bitrate 250000,
  sample-point 0.875, `tq 20 prop-seg 87 phase-seg1 87 phase-seg2 25 sjw 16`
- Not one retransmission across the run. **A zero TEC is the proof the ACK came
  back** — a receiver must assert a dominant bit in the ACK slot of the
  transmitter's frame, so the link is bidirectional even though traffic only
  went one way.

**Two clocks, one bus.** Orion runs 200 tq of 20 ns from 50 MHz; the STM32 runs
15 tq of 266.67 ns from 45 MHz. Both land on exactly 250 000 bps, sample points
87.5% and 86.7%.

**Inter-frame timing confirmed the divisor chain a third time.** Predicted TIM3
period: 90 MHz / (1800+1) / (25000+1) = 1.99881 Hz = 500.298 ms. Orion
timestamped the frames 500.297 / 500.295 / 500.304 ms apart. After the
oscilloscope (MCO1/MCO2) and the blinky, APB1 = 45 MHz is now also confirmed by
a stopwatch on the far side of the bus.

### The floating CAN_RX lesson (Aug 10, 2026) — do not skip this

Before the transceiver was wired, PB8 was left floating. **Three consecutive
bench runs of identical firmware failed three different ways:**

| Run | TEC | REC | LEC | What it looked like |
|---|---|---|---|---|
| 1 | 128 | 0 | `ack` | transmitted fine, nothing acknowledged |
| 2 | 0 | 255 | `form` | never transmitted, receiver drowning in garbage |
| 3 | 0 | counting down | `bit-dominant` | cycling in and out of BUS-OFF |

All three are the same root cause: an undriven CMOS input settling differently
each power-up. Floating high looks like an unacknowledged bus; floating low or
noisy looks like a corrupted one.

**The non-determinism was the diagnosis.** A driven input cannot behave
differently run to run — that alone ruled out firmware before any register was
examined.

### Bus-load ramp — Aug 11, 2026

`cangen can0 -g <gap> -I i -L 8` from daedalus, heartbeat running on the STM32,
`monitor off`, counters cleared between steps. Run durations were derived from
the heartbeat count (1.99881 Hz), which makes the STM32 its own stopwatch.

| `-g` | duration | measured rate | bus load | frames RX | FIFO-full | overruns |
|---|---|---|---|---|---|---|
| 5 | 87.1 s | 206 f/s | ~11% | 17,959 | 0 | 0 |
| 2 | 68.5 s | 509 f/s | ~27% | 34,891 | 0 | 0 |
| 1 | 68.0 s | 970 f/s | ~52% | 65,993 | 0 | 0 |
| 0.5 | 68.5 s | **1,858 f/s** | **~100%** | 127,343 | 0 | 0 |

`TEC 0 / REC 0 / lec none / error-active` at every step, including saturation.

**The ramp topped out on the wire, not on the MCU.** At `-g 0.5` cangen asked
for 2,000 f/s and got 1,858 — that is 538 us/frame, about 134 bits, exactly an
8-byte standard frame plus typical stuffing at 250 kbps. Every derived figure
agrees with theory independently, which is what makes the measurement
trustworthy.

**What it bounds.** `FULL0` sets when FIFO0 holds 3 messages. It never
incremented across 127,343 frames at saturation, so the main loop always drained
before three frames could accumulate:

```
worst-case main-loop period < 3 x 538 us = 1.6 ms
```

Not an average — a bound, held for 68 s with no outlier.

**Arbitration held too:** `can tx 137 frames, 0 dropped` at ~100% load. With
`-I i` sweeping the whole ID range, roughly five-eighths of cangen's frames
outrank 0x500, yet the heartbeat never missed a mailbox. At 2 Hz it has 500 ms
to win one arbitration, which is ample even on a saturated bus.

**What it does NOT prove — read this before citing the table.** The ramp
measured *throughput*: whether frames are lost. It measured neither **latency**
(how long a frame waits in the FIFO before being handled) nor **coupling** (that
the result is a property of a nearly empty main loop). Do not cite it as
evidence that polling is the right architecture — see below.

### Verification log — Aug 13, 2026 (W3 first motion)

Encoder position is `carry x 65536 + value`; the SERVO42C encoder is 16-bit per
**motor** revolution.

| Step | `pulses` (`33`) | encoder raw | position | predicted |
|---|---|---|---|---|
| start | — | carry 0, 42 | +42 | — |
| 3 x `move 16` CW (48 pulses) | −48 | carry −1, 63614 | −1,922 | −1,924 |
| `deg 5` (+422 pulses) | −470 | carry −1, 46329 | −19,207 | −19,209 |
| `deg −5` (−422 pulses) | — | carry −1, 63610 | −1,926 | −1,922 |

**Mstep = 8 confirmed by measurement, not by menu.** Predictions use
`pulses / 1600 x 65536`. Both forward steps land within **2 counts**, and the
offset is constant rather than growing — an artifact, not a scale error. At
Mstep 16 the first row would have read −941, so the result is decisive. This is
the reliable way to check Mstep: it measures what the mechanism did.

**Round-trip repeatability: 4 counts.** Out 5 deg and back landed −1,926
against −1,922. One microstep at Mstep 8 is 65536/1600 = **41 counts**, so the
error is about **one tenth of a single microstep** — 0.022 deg at the motor,
**0.0012 deg at the output**. No measurable backlash contribution at this
amplitude. Useful baseline for W9 precision calibration.

**Angle conversion is exact to quantisation.** `deg 5` issued 422 pulses =
4.9974 deg at the output; the 0.0026 deg residual is one-pulse quantisation
(0.0118 deg), i.e. the mechanism's floor.

**`33` counts UART-commanded pulses**, not just hardware STEP input: −470 is
exactly 48 + 422. That makes it usable as the feedback path for absolute
positioning, which `FD` alone cannot provide since it is a relative move.

**Direction convention:** positive degrees / `ccw = false` **decrements** both
the encoder position and the `33` pulse counter. Pin this down before Ackermann
sign conventions are written.

### Torque characterization — June 8, 2026

**Rig:** AMF-300 digital force gauge (300 N max) rigidly frame-mounted at exactly
**10 cm** from the rotation axis. A 20×20 aluminium profile on the gearbox output
shaft presses against it. `Torque (N·m) = Force (N) × 0.10`. If the gauge reads
kgf: `Torque = kgf × 9.81 × 0.10`.

**Method:** enable, advance in 16-pulse steps (`E0 FD 02 00 00 00 10 EF`), and at
each step record force, angle error (`E0 39 19`), and supply current.

**Run 1 — default MaxT:**
| Force (N) | Torque (N·m) | Angle error (°) |
|---|---|---|
| 7.3 | 0.73 | −0.566 |
| 9.4 | 0.94 | −0.697 |
| 11.6 | 1.16 | −0.900 |
| 13.7 | 1.37 | −1.038 |
| 15.7 | 1.57 | −1.170 |
| 17.7 | 1.77 | −1.312 |
| 19.7 | 1.97 | −1.471 |
| 21.7 | 2.17 | −1.602 |
| 23.7 | 2.37 | −1.794 |
| 25.8 | 2.58 | −1.971 |
| 27.5 | 2.75 | −2.234 |
| 29.6 | 2.96 | −2.393 |
| 31.6 | 3.16 | −2.658 |
| 33.6 | 3.36 | −2.850 |
| 35.7 | 3.57 | −3.091 |

Run 1 peaked at **4.95 N·m** before the driver stopped.

**Run 2 — MaxT raised to maximum (`E0 A5 04 B0 39`):** pushed to a true stall
boundary at **5.57 N·m**, drawing **1550 mA** = **18.66 W** at 12.04 V.


## DRV8833 — superseded driver (moved out of the context file Sep 18, 2026)

The project ran on a DRV8833 breakout from Aug 16 until the driver was changed to
the DRV8874 on Aug 25, 2026. None of this is current state; it is kept because
seven DRV8833 carriers are still in the parts box and the reasoning is correct
*for a DRV8833*. The one live consequence — the second buck, now 12 V for the
motor's sake, fed independently of the logic buck — was left in
`PROJECT_CONTEXT_WHEEL_FW.md`.

### Driver
**Commercial DRV8833 breakout PCB**, already designed and in hand. The intent is
to **parallel both H-bridges per motor** for higher current.

TI documents parallel mode explicitly (DRV8833 datasheet, Figure 7): the two
bridges may be tied together, and the device's internal dead time prevents
cross-conduction between them, so no external protection is needed. Inputs are
tied in pairs (IN1=IN3, IN2=IN4) and outputs joined (OUT1+OUT3, OUT2+OUT4). On a
breakout the outputs are usually separate screw terminals, so joining them is
external wiring.

### ⚠️ Supply-voltage conflict with the PDB — resolve before HW1 layout

| | Voltage |
|---|---|
| DRV8833 operating VM range | **2.7 – 10.8 V** |
| Planned PDB branch rail | **13.0 – 13.5 V** |

**The DRV8833 cannot be fed from the branch rail.** The PDB was deliberately set
to 13–13.5 V to pre-compensate branch-wire drop against the SERVO42C's 12 V
floor — that decision is sound and should not change, because the steering
driver needs it. But it puts the rail roughly 2.2 – 2.7 V above the DRV8833's
operating maximum, and above its absolute-maximum rating too.

This is not a derating margin question. It is over the limit.

**Options, in order of preference given the parts are already bought:**

1. **Second buck on the node PCB: 13–13.5 V → ~9 V motor rail.** Keeps every
   part already purchased. The motor is a 6 V/12 V unit, so ~9 V costs some top
   speed but nothing else. The node PCB already carries one buck for 3.3 V logic;
   this adds a second, sized for the motor current. **Recommended.**
   Keep the two bucks fed independently from the 13 V rail rather than cascading
   3.3 V off the motor rail — motor noise should not sit upstream of the MCU.
2. **Swap the driver for a higher-voltage part** (e.g. one rated for the full
   12–24 V range). Removes a buck, but discards drivers already in hand and
   restarts the driver-selection work.
3. **Lower the PDB rail.** Rejected — it breaks the SERVO42C drop budget, which
   is the reason 13–13.5 V was chosen.

**RESOLVED Sep 11, 2026 by option 2 — the driver was swapped.** The DRV8874
runs to 37 V, so the conflict that created this section no longer exists: the
branch rail is inside the driver's range with enormous margin. Option 1's buck
stays anyway, because the **motor** is a 6 V/12 V unit and the rail is
13.5 V — the regulator now exists to protect the motor, not the driver, and its
output is **12 V** as of Sep 12. This section is kept because the DRV8833
reasoning is still the correct reasoning for a DRV8833, and seven stock
carriers remain in the parts box.

### The carrier in hand — characterised Aug 25, 2026

Vendor documentation: https://lastminuteengineers.com/drv8833-arduino-tutorial/

Terminals are `IN1 IN2 IN3 IN4` / `OUT1 OUT2 OUT3 OUT4`, `VCC`, `GND`, plus two
control pins whose **silkscreen labels are truncated and confusing**: `EEP` is
nSLEEP, `ULT` is nFAULT. Do not read them as anything else.

No onboard regulator — the DRV8833 runs its logic off VM, so the carrier takes a
single supply. Logic inputs are 3 V/5 V compatible, so 3.3 V PWM drives it
directly.

**Paralleling is external wiring on this board.** All four outputs are separate
terminals, so tie IN1+IN3 to one MCU pin, IN2+IN4 to the other, and join
OUT1+OUT3 / OUT2+OUT4. Costs no extra MCU pins — one STM32 pin drives two
carrier inputs.

**J1 ships CLOSED, which pulls nSLEEP up and leaves the driver ENABLED with no
MCU involved.** Cut it. With J1 open the chip's on-chip pull-down holds nSLEEP
low and "MCU not running ⇒ bridge disabled" becomes a property of the hardware
rather than a firmware promise. **Done on the bench board Aug 25; must be
repeated on all seven carriers.** Also: with J1 closed the EEP pin may sit at
VM, so measure it before connecting to a 5 V-tolerant STM32 pin.

**nFAULT floats by default — there is no pull-up on the carrier.** An external
10 kΩ is fitted on the bench build and belongs on the HW1 schematic. Keep the
STM32 internal pull-up enabled as well: on a production board with the resistor
unpopulated it stops the input floating and inventing faults.

**nFAULT high does NOT prove the driver is alive.** With VM absent the DRV8833
is unpowered, its open-drain output is off, and the pull-up reads 3.3 V anyway.
A clear nFAULT means "nothing is pulling this low", not "the motor rail is good".

### ⚠️ No current sensing and no current limit (DRV8833 only)

**AISEN/BISEN are tied directly to GND on this carrier**, which disables the
DRV8833's current-limiting feature entirely. Two consequences:

1. **Nothing limits stall current except the chip's own OCP.** A jammed wheel
   pulls whatever the motor pulls until OCP trips and asserts nFAULT. Firmware
   must monitor nFAULT, implement a stall timeout, and latch off rather than
   retrying into a stuck wheel. **This rule survives the driver change** — it is
   good practice on the DRV8874 too, which simply regulates instead of tripping.
2. **There is no current feedback available at all** on this carrier.
   ~~W5's PID is therefore velocity-only~~ — **superseded Aug 25, 2026.** The
   DRV8874's IPROPI output restores current feedback, so a torque/current inner
   loop is reachable again. Until the parts arrive (~Sep 15) the bench remains
   velocity-only, which is fine: W4 and W5 both run at loads far below the
   DRV8833's limits.

### Current: ~2 A RMS paralleled — DRV8833 interim figure only

An earlier revision of this section recorded **3 A RMS / 4 A peak**, taken from
TI's 1.5 A RMS per-bridge silicon figure. That is too optimistic for this
carrier. The vendor rates the board at **1.2 A continuous / 2 A peak per
channel** — a board-level thermal rating below the silicon's.

Paralleling halves R<sub>DS(on)</sub>, so for the same dissipation the current
scales by √2, not 2: about **1.7 A continuous**, perhaps up to ~2.4 A if the
carrier's copper is generous. **Design the PDB branch around ~2 A RMS / 4 A
peak**, and treat the real number as something W4 measures.

**Answered Aug 25:** stall is **4.1 A at 9 V**, well past the ~2 A figure — the
reason the driver changed. **For PDB branch sizing use the DRV8874's numbers,
not these**: ~3 A continuous with regulation set below that, 6 A peak.

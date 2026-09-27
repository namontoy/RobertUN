# RobertUN — Wheel Controller Firmware: Full Progress Log
**Last updated:** September 26, 2026 (**THE STAIRCASE WAS RUN IN REVERSE AND THE DIRECTION ASYMMETRY IS ENTIRELY THE INTEGRATOR** — `corr(Δ|out|, Δi) = 0.9986`; tracking is *better* reverse (0.006 rpm worst vs 0.015), 0% saturation at all 21 holds, and the asymmetry **changes sign at 14.3 rpm** so "reverse is x% harder" is false. A ripple panel was caught about to claim an agreement its estimator could not measure. `bench.py` gained the `stair` profile, with a reverse-direction sign bug in its ceiling guard caught before it ran)

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

- **Sep 26 (bench, reverse) — THE SAME STAIRCASE THE OTHER WAY. The direction
  asymmetry is the integrator and nothing else, "reverse is x% harder" is
  false, and a figure panel was caught about to assert an agreement its
  estimator cannot measure.**

  **Why reverse is not an optional symmetry check.** The rover reverses. And
  the friction feedforward in `velocity.c:346` *"takes the sign of where we are
  trying to go, not of the error"* — `ff_term = ff_slope * sp_rpm`, then
  `+ ff_offset` if `sp_rpm > 0` and `- ff_offset` if `< 0`. It is therefore
  symmetric **by construction**, which means it cannot absorb a plant that is
  not. Whatever the plant does differently in reverse has to show up somewhere
  else, and that somewhere is measurable.

  **Tooling first: the `stair` profile, and two sign bugs caught before it ran.**
  The forward staircase had been driven by a scratchpad script; it was landed in
  `bench.py` as a real profile (one arming, a long dwell per point, settled
  stats past `--hold-settle`, abort on `--abort-ma` or any watchdog fraction).
  Promoting it surfaced two bugs, both fatal, neither cosmetic:
    - A literal `--lo -10 --hi -20` makes the step count negative and produces
      an **empty setpoint list**.
    - ⚠️ **The 30% ceiling guard was written as `max(setpoints)`.** For a
      reverse run that reads **−10**, which is below every ceiling, so the guard
      **silently stops guarding in exactly the direction about to be tested.**
      The standing instruction is that 30% duty is the hard ceiling for
      characterisation work; this would have removed it at the moment it
      mattered most.
    - Fix: `--lo`/`--hi` are **magnitudes**, `--dir` carries the sign (the same
      split `profile_step` already uses), a `walk` term handles descending
      ranges, and the guard tests `max(abs(s))`. Verified offline across four
      argument combinations before anything spun. `import statistics` was also
      missing — the new profile used it.

  **A short −10 rpm direction check** ran first (`runs/2026-09-26T11-01-57_stair`,
  25 s), then the full run.

  **The run — `runs/2026-09-26T11-02-46_stair`, `outcome=ok`, NOT SUSPECT.**
  21 points, −10.0 → −20.0 rpm in 0.5 rpm steps, 60 s each, 1265.5 s total.
  **63,203 `V` rows, 0 sequence gaps, 0 velocity gaps, 0 unpublished steps,
  0 tx_dropped.**

  | | forward | reverse |
  |---|---|---|
  | worst settled error | 0.015 rpm | **0.006 rpm** |
  | per-point sem | 0.020 | 0.019 |
  | inverse fit | `12.559·rpm + 30.54` (rms 1.51) | `11.503·\|rpm\| + 43.65` (rms 1.54) |
  | peak \|out\| | 284 o/oo | 277 o/oo (of 300) |
  | saturation | 0% at all 21 | 0% at all 21 |
  | mean current | 270 mA | **292 mA (+8.1%)** |
  | ripple sd | 0.87 → 1.21 rpm | 0.88 → 1.00 rpm |
  | integrator range | −0.6 .. +3.7 o/oo | −6.7 .. +5.3 o/oo |

  ⚠️ **A prediction I made was wrong, and the data corrected it.** From the
  single −10 rpm check — where reverse costs ~3.8% more duty — I said the top of
  the reverse range would saturate against the 300 o/oo ceiling. It did not:
  **0% saturation at all 21 points, peak 277.** The extrapolation came from the
  one point where reverse costs *more*, and the asymmetry **changes sign near
  14.3 rpm**.

  **THE HEADLINE — the asymmetry is the integrator, and that is arithmetic, not
  luck.** `corr(Δ|out|, Δi) = 0.9986` across the 21 setpoints, the two
  difference curves **0.62 o/oo rms apart**. The feedforward is bit-identical in
  both runs (asserted at figure load: `fwd.ff() == rev.ff()`), so there is
  nowhere else for a direction difference to land. The consequence is the useful
  part: **the integrator is a direct readout of the model's direction error**,
  which is how a separate reverse `ff_b` would get *measured* rather than
  guessed.

  ⚠️ **"Reverse is x% harder" is false.** The asymmetry spans **−10.0 to
  +1.7 o/oo**, mean −2.74, and clears the comparison's own noise floor —
  **±2.16 o/oo**, the quadrature sum of the two fits' residuals — at only
  **9 of 21 setpoints**, with the sustained sign change at **14.3 rpm**. Reverse
  costs more below it and less above. Two fits with different slope *and*
  intercept cannot be summarised by one scalar.
    - The crossing detector needed fixing too: it first reported **10.5 rpm**,
      having taken the *first* zero crossing — one of four noise wobbles inside
      the ±2.16 floor. It now takes the last crossing the data does not return
      from.

  ⚠️ **THE PANEL THAT WAS ABOUT TO SHIP A FALSE CLAIM.** The raw
  events-per-revolution figures came out **identical to every digit** in both
  directions: 11.91 ± 0.19 vs 11.91 ± 0.19. That is not independent agreement.
  An autocorrelation period is quantised to an integer number of **20 ms control
  periods**, and at these speeds one bin is worth **±0.72 events/rev** — four
  times the "spread" being reported. The estimator cannot resolve a difference
  smaller than its own bin, so two perfectly coincident marker sets would have
  asserted a precision the method does not have, and the whole ±0.19 was
  quantisation rather than physics.
    - **Fix: parabolic sub-bin peak refinement** (fit a parabola through the
      peak and its two neighbours, take the vertex; reject it if the curvature
      is non-negative or the vertex leaves its own bin). Added as **opt-in
      (`interp=True`)** so the forward figure's published numbers do not move —
      `velocity_loop_stair_21min.png` was md5-baselined before the change and
      re-verified byte-identical after it (`8a9865010df31cf6f6216e7ac04498a8`).
    - **The refined result is a better statement than the one it replaced:**
      **11.998 ± 0.017** forward, **11.978 ± 0.017** reverse — an 11× tightening
      that puts the forward run on **exactly 12 per revolution to 0.02%**.
      The paired difference is **−0.0204 ± 0.0030 events/rev**, negative at
      **20 of 21** setpoints: statistically resolved, but 0.17% — far too small
      to be a different feature.
    - The ripple's **amplitude** is *not* the same and the figure says so
      explicitly, because "the ripple is identical" is only half true.

  ⚠️ **Reverse draws +8.1% current while commanding LESS output** (292 vs
  270 mA; 277 o/oo against 284 at the top of the range). More current for less
  duty is either a real direction-dependent load or a **sign-dependent offset in
  the current sense**. These two runs cannot separate them, and the low end sits
  inside the 145 o/oo synchronised-sense floor's reach anyway. It still wants an
  independent ammeter.

  ⚠️ **THE CONFOUND, stated on the figure itself.** The two runs are **67 min
  apart and NOT interleaved**. Temperature, belt tension, and where the carriage
  sits on a treadmill belt that travels the *other way* in reverse are all
  aliased into the word "direction". **An A/B/A staircase would separate them
  and has not been run.** Until it is, "direction" is the honest label for the
  difference but not a proven cause of it.

  **Figure: `figures/velocity_loop_stair_direction.png`** + 
  `plot_velocity_loop_stair_direction.py`, sharing `stairdata.py` with the
  forward figure and likewise transcribing nothing; both scripts end by printing
  the same numbers they drew. A separate figure rather than more panels on the
  forward one, deliberately: the forward figure's four panels each carry a
  single-run argument with long annotations and have no room, and this data's
  value is **entirely comparative** — every interesting number here is a
  difference, which is a different thesis. `stairdata.py` gained magnitude
  views (`m_sp`, `m_rpm`, `m_out`, `m_ff`) so both signs share one code path;
  ⚠️ **`m_i` is deliberately NOT `abs()`** — it is `i * sign(setpoint)`, what
  the integrator adds to the *magnitude* of the commanded output, because the
  integrator's sign is only meaningful against the output it is correcting.

  **Still unexercised:** bit 64 (MISSED) has still never fired; **`cfg save` has
  still never been run**, so every gain remains RAM-live; the ~12-per-revolution
  mechanical feature is still unidentified; and the A/B/A interleaved staircase
  is owed.

- **Sep 26 (bench) — THE VELOCITY LOOP RAN FOR 21 CONTINUOUS MINUTES AND THE
  GAINS ARE GOOD. Then the step metric that said otherwise turned out to be
  measuring the slew limiter, and was rewritten. Two figures added.**

  ### The 21-minute staircase — `runs/2026-09-26T09-55-08_stair`

  21 setpoints 0.5 rpm apart, 10.0 → 20.0 rpm, **60 s each, uninterrupted**,
  shipped gains, 12 V, loaded rig, `enc window 20`. Chosen over another short
  step because a step says whether the loop is *stable* and says nothing about
  whether it is *accurate*, whether it drifts, or whether the ¼-of-textbook
  derating costs anything — and those are the questions that decide whether
  these gains go to the rover.

  - **Integrity first, as always.** 63,202 `V` rows and 63,214 `T` rows,
    **0 sequence gaps on either channel, 0 unpublished control steps,
    0 tx_dropped**, `suspect False`. The overrun bit added for exactly this
    purpose never fired in 21 minutes at 50 Hz.
  - **Tracking: mean error +0.0008 rpm, worst 0.015 rpm, sd 0.005 rpm**, against
    a per-point standard error of ~0.020 rpm. **The loop is accurate to below
    the noise floor of the instrument measuring it**, at all 21 speeds.
  - **The 30% ceiling is not reached, and an earlier claim is corrected.**
    0% saturation at every hold; peak output **284 of 300 o/oo** at 20 rpm. The
    6% saturation a short step reported at 20 rpm was the **acceleration
    transient**, not the operating point.
  - **The derated gains are not costing accuracy, because the plant model behind
    the feedforward is right.** Closed-loop inverse **`out = 12.559 × rpm +
    30.54 o/oo`** (rms 1.51 o/oo, n=21) against Sep 25's open-loop sweep
    inverted, `12.511 × rpm + 30.28` — **0.39% apart, from different
    excitation, different data and a different estimator**. Shipped feedforward
    error at 15 rpm: **−1.3 o/oo (−0.6%)**. The integrator therefore has almost
    nothing to do: **mean +1.46 o/oo, range −0.6..+3.7, out of ~219 o/oo
    commanded**. ⚠️ **The "~4.5% optimistic" caveat carried on that plant model
    does not hold in this band.**
  - **The ripple is MECHANICAL, and the run proves it rather than asserting it.**
    **11.91 ± 0.19 events per output revolution**, range 11.5–12.2 across all
    21 holds — held constant over a 2:1 speed range while the *period* swept
    500 ms → 260 ms. **A control limit cycle holds a fixed period; only a
    rotating feature holds a fixed count per revolution.** Amplitude grows
    0.87 → 1.21 rpm (2.4 → 3.4 encoder counts). Which feature — gear, magnet
    ring, coupling — is a mechanical inspection, not a telemetry one.
  - **No thermal drift.** Within-hold drift over 60 s: **+0.0005 rpm,
    −0.04 o/oo**. And the current U-shape is **not** a warm-up transient: a
    90 s re-take 22 minutes later (`runs/2026-09-26T10-17-57_stair`) reproduced
    **294→293 mA at 10.0 rpm and 274→278 mA at 10.5 rpm**. ⚠️ The low-end
    current shape is the one number in that figure not to trust — it sits near
    the 145 o/oo synchronised-sense floor and wants an independent ammeter.

  ### The step metric was measuring `vel_slew`, not the loop

  ⚠️ **`bench.py run step` reported "rise 2.0 s, overshoot 14%" for a 0 → 10 rpm
  step. Both numbers were measurements of something other than the controller,
  and both pointed at a gain change that would have made the loop worse.**

  - **The setpoint is a ramp, not a step.** `vel_slew` ships at **4 rpm/s**, so
    0 → 10 rpm spends its first **2.5 s** with the loop tracking a moving target
    and 0 → 20 rpm spends **5 s**. Anchoring rise and settling at the command
    instant charges that ramp to the loop. **The tell was that the answer did
    not depend on Kp**: a 0 → 10 step returned 2.0 s whatever the gain, because
    2.0 s is 0.8 × 2.5 s — the limiter's own 10→90% time.
  - **The fix is an anchor, not a formula change.** `overshoot` and `settle_s`
    are now measured **from the instant `VFLAG_RAMPING` clears**; `rise_s` is
    still reported from the command (it is what an operator waits) but **next to
    `rise_slew_floor_s`**, with `ramp_limited` set when it is within 30% of that
    floor. New `track_lag_rpm` measures how far behind the moving setpoint the
    loop sits *during* the ramp — **that is the quantity a gain change moves,
    and the old metric had no slot for it**. `settle_from_command_s` is kept
    alongside `settle_s` because both are true and they answer different
    questions.
  - **Verified by replaying all three committed step runs.** `slew_rpm_s`
    recovers **3.97–3.98 rpm/s** against the configured 4.0. The 0 → 20 run
    settles in **2.94 s from the ramp's end** versus **7.94 s from the
    command** — and those are **the same instant**, 5.00 s apart only in what
    they subtract. That 5.00 s is exactly `vel_slew`.
  - ⚠️ **Then the corrected metric exposed a second defect: the "overshoot" is a
    ripple peak.** A single maximum drawn from a signal carrying ~1.2 rpm sd of
    mechanical ripple sits 2–3 sd high whatever the gains do. All three runs
    peak **4 encoder counts (1.42 rpm) above target — the same distance at 10
    and at 20 rpm**, which a controller's overshoot would not be. `step_metrics`
    now reports `tail_sd_rpm` and sets `overshoot_above_ripple`, which is
    **False for all three runs**: *no overshoot is resolvable on this rig.* The
    peak search is also bounded to `OVERSHOOT_WINDOW_S` = 3 s (the plant's slow
    pole is 2.75 s) so that a longer `--dwell` cannot manufacture a larger
    overshoot, with `overshoot_window_truncated` set when a run was too short to
    fill it — the 3 s-dwell run had only 0.48 s and is not comparable.
  - **The honest verdict for all three step runs is "no overshoot resolvable,
    and rise not measurable above the limiter".** That is less satisfying than
    "14% overshoot, lower Kp" and it is the correct answer. A true step response
    needs `--slew 0`, **which has not been run**.

  ### Two figures

  `docs/environment/figures/`, both following `rigdata.py`'s rule that the
  loader **transcribes nothing** and every script ends by printing the same
  numbers it drew:

  - **`velocity_loop_stair_21min.png`** / `plot_velocity_loop_stair.py` /
    `stairdata.py` — the 21 minutes, the closed-loop plant inverse against the
    open-loop sweep, the ripple's events-per-revolution against a limit-cycle
    null, and the current U-shape with its sense-floor caveat.
  - **`velocity_step_slew_anchor.png`** / `plot_velocity_step_anchor.py` /
    `stepdata.py` — what the metric fix corrected. ⚠️ **`stepdata.py` imports
    `step_metrics` from `bench.py` and calls it rather than recomputing it**, so
    the figure cannot drift from the tool it documents; if the tool's definition
    of rise or overshoot changes, the figure changes with it or fails loudly.
  - ⚠️ **Two bugs were found by drawing the data, not by reading the code.**
    `stairdata.ripple_table()` returned 20 of 21 holds, because the last hold's
    window ran to end-of-file and swallowed the `vel stop` ramp-down — a
    monotonic collapse to zero whose autocorrelation never goes negative, so no
    period was found at all. And the autocorrelation's **global** maximum picks
    the **second harmonic** whenever the fundamental's peak is the shorter one;
    it did so at 13.5, 18.5 and 20.0 rpm, inflating the spread from sd 0.19 to
    sd 2.09. The fundamental is the first strong local maximum *after* the
    correlation first goes negative.

  ### Still owed

  Bit 64 (MISSED) is still unexercised on hardware — 21 minutes at 50 Hz never
  triggered it. `cfg save` has still never been run, so every setting above is
  RAM-live. The ~12-per-revolution mechanical feature needs identifying. The
  low-end current wants an independent ammeter. And a `--slew 0` step is the
  only way to get a real loop rise time.

- **Sep 26 (later) — W5's VELOCITY PID IS WRITTEN, AND SO IS THE INSTRUMENT
  THAT WILL TUNE IT. Branch `w5-velocity-pid`. Two commits' worth of work:
  the loop itself, then the telemetry channel and host profile that make it
  tunable. NOTHING HERE HAS TOUCHED HARDWARE — every claim below is a code or
  build claim, and the bench pass is owed.**

  ### The module — `Core/Src/velocity.c`, `Core/Inc/velocity.h`

  - **It is a policy layer above `drive.c`, and the split is the same one the
    project has now settled on twice.** `drive.c` actuates and protects; it
    decides nothing. `velocity.c` decides — what speed, how fast to get there,
    what to do when the bridge is unavailable — and reaches the bridge only
    through `drive_set_duty()`. Nothing in `drive.c` changed.
  - **It steps at the MEASUREMENT rate, not the tick rate.** `velocity_on_tick()`
    is called from the TIM6 1 kHz callback, but the first thing it does is
    compare `encoder_velocity_seq()` against the last one it saw and return if
    it has not moved. The encoder's velocity is a boxcar over
    `ENCODER_VELOCITY_WINDOW_DEFAULT` = 20 ticks, so the loop actually runs at
    **50 Hz**, and at `enc window 5` it would run at 200 Hz. **A loop that runs
    faster than its sensor updates is differentiating a staircase and
    integrating the same error several times over** — the derivative term would
    be reading quantisation and the integral would be counting one real error as
    twenty. `encoder_velocity_seq()` was added to `encoder.c` for this: it bumps
    on window closure, which is the only honest "new measurement" event
    available. The tick-callback order is `encoder_on_tick()` →
    `drive_on_tick()` → `velocity_on_tick()`, velocity last and deliberately so:
    it acts on the measurement taken this millisecond, not last.
  - **Feedforward from the inverse plant.** `duty‰ = ff_a × rpm + ff_b`, with
    `ff_a` = 12510 milli-o/oo-per-rpm from the loaded fit's inverse
    `duty% = 1.251 × rpm + 3.028`, and `ff_b` = 30 o/oo applied **with the sign
    of the setpoint** — friction opposes motion, so its compensation has to flip
    with direction, which a slope term alone cannot do. The PID therefore only
    corrects the fit's error rather than building the whole output from scratch,
    which is what keeps the integrator small and its resting value diagnostic.
  - **The setpoint ramp lives here, exactly as this project predicted it would.**
    Task 21's note said "ramping the setpoint, once the velocity loop exists, is
    a *different* ramp and does still belong to the control layer — `drive.c`
    has no setpoint." That held: `vel_slew` (milli-rpm/s, default 4000 = 4 rpm/s)
    ramps `sp_rpm` toward the commanded target inside `velocity.c`. Limiting the
    setpoint rather than the output is what stops the loop winding up against
    its own ramp.
  - **Anti-windup freezes the integrator under four conditions, and reports
    which.** Output saturated in the direction the error is pushing; `drive.c`'s
    own limiter slewing; the bridge disabled or a fault latched; and the
    setpoint watchdog expired. Lumping them into one "frozen" bit would have
    been the easy thing and would have been useless — see the telemetry section.
  - **Kd defaults to 0, is taken on the measurement rather than the error, and
    is low-passed** (`D_FILTER_ALPHA` in `velocity.c`, deliberately *not* a
    config key — a filter constant that can be set live invites tuning the
    filter instead of the loop). Derivative on measurement means a setpoint step
    does not produce a derivative kick.
  - **Stopping is coasting.** `vel stop` walks the setpoint down and then coasts;
    it is explicitly **not** an emergency stop, and `drv coast` stays the
    immediate one. Same reasoning as `drive.c`'s watchdog action: braking from
    speed drives I = E/R through the low-side FETs.
  - **Gains ship at about a quarter of textbook, on purpose.** For K = 0.07993
    rpm per o/oo and τ_fast = 0.219 s, the textbook pair is Kp = 1/K = **12.51**
    and Ki = 1/(K·τ) = **57.1**. Shipped: **Kp 3.0, Ki 10.0**. The derate is not
    conservatism for its own sake — the plant fit those numbers come from is the
    one flagged 4.5% optimistic and due a re-take, and the rig is 2.87× light in
    inertia against the rover. Gains derived from a model that is known wrong
    should not be shipped at their full value.
  - **Nine `cfg` keys, all in milli-units** because the store is int32-only:
    `vel_kp` 3000, `vel_ki` 10000, `vel_kd` 0, `vel_ff_a` 12510, `vel_ff_b` 30,
    `vel_ilim` 150, `vel_max` 300, `vel_slew` 4000, `vel_tmo` 1000. `vel_max` is
    300 o/oo — **the 30% ceiling, in the loop's own units.** Each key is
    live-applied by `cfg <key> <val>` with no save, which is the
    set-for-this-session path the Sep 25 note asked for: the config log is
    append-only with 1024 slots and one full snapshot per save, so a 50-point
    gain scan that persisted every trial would burn 5% of the log. Adding the
    keys cost nothing because **`cfg save` has still never been run on this
    board** — the key-count gotcha would otherwise have discarded the record.

  ### ⚠️ The loop defeats `drive.c`'s command watchdog, by construction

  `drive_set_duty()` calls `drive_kick()` deliberately — a duty command is
  evidence of a live host, which is the reasoning task 21 settled on. The
  velocity loop calls `drive_set_duty()` fifty times a second, forever. So
  **arming the loop means `drive.c`'s command watchdog can never expire again.**

  This is not a bug to fix in `drive.c`; it is the necessary consequence of
  putting a controller above it, and the answer is the same one the layering
  already implies: the layer that can defeat a watchdog carries its own.
  `velocity.c` has a **setpoint watchdog** (`vel_tmo`, default 1000 ms, **armed
  by default — the opposite of `drv timeout`'s default-off**), and **only
  `vel target` kicks it.** A setpoint arriving is evidence that something
  upstream is still *choosing*, which is the thing actually worth watching once
  the loop below is being kept alive unconditionally. On expiry it **coasts
  immediately** rather than ramping down — matching `drive.c`'s precedent, on
  the grounds that a dead host is not the moment to ease off over six seconds —
  and the expired flag is **sticky**: a kick refreshes the countdown but never
  clears the latch, which again matches `drive.c`. Only `velocity_enable()`
  clears it.

  ### ⚠️ A real defect in what had just been committed: no `volatile`

  `velocity.c` was committed with **not one `volatile`** on any static written
  by the TIM6 ISR and read from thread context — the whole loop-state block and
  the whole watchdog block. `encoder.c` and `drive.c`, the two modules it sits
  between and the two it was written by reading, both mark theirs correctly.

  It was latent: nothing yet copied that state out in a way the compiler could
  reorder or cache badly. The very next change — the telemetry snapshot — is
  exactly what would have made it bite. Fixed as the first step of that change.
  The **cached gain floats are deliberately left non-volatile**, with a comment
  saying so: thread context writes them, the ISR only reads, and each is a
  single word.

  The generalisable part is in KEY LEARNINGS: a new module hanging off an
  existing ISR does not inherit that ISR's concurrency discipline just by
  sitting next to code that has it.

  ### The telemetry channel — `V,`

  **Why it was needed at all.** The `vel` console command prints loop state at
  console pace, one line at a time for a human. A step response is a 1–2 second
  event at 50 Hz. The existing `T,` line carries `duty`, `count`, `mrpm`, `ma`
  and `flags` — what the **bridge and plant** did — and says nothing about what
  the **loop decided**: no setpoint, no error, no term breakdown, no saturation
  or anti-windup state. Tuning against `T,` alone means inferring the
  controller's internals from its output.

  ```
  V,<seq>,<ms>,<sp_mrpm>,<meas_mrpm>,<out>,<ff>,<p>,<i>,<d>,<flags>
  ```

  - **A separate record, not more columns on `T,`.** `TELEM_RE` in `node.py` is
    unanchored, so an extended `T,` would still match its first seven groups —
    but `Telem.parse()` checks the field count and would reject it, and every
    committed run directory holds a 7-field `telemetry.csv` header. A widened
    `T,` would therefore mean **two incompatible things depending on which code
    path read it**. A new record type breaks nothing: old logs parse unchanged,
    and a reader that does not know about `V,` ignores it.
  - **One line per control step, not on the `telem` timer.** The loop advances
    at `1000 / enc window` Hz. Riding the telem scheduler would alias it — at
    100 Hz every step appears twice, at 30 Hz they beat — and neither is
    readable as a step response. The integrator and the derivative **only mean
    anything per step**. So the loop publishes a snapshot and the main loop
    drains it: exactly one row per control decision, self-limiting by
    construction.
  - **All four terms carried separately, and before the clamp.** `out` differing
    from `ff + p + i + d` is then *exactly* the saturation, and an oscillation
    says in the data which term is driving it. This was a deliberate choice over
    a narrower 7-field line — the line is wider, and the bandwidth note below is
    the price.
  - **One slot with overrun reporting, not a ring.** A step the main loop failed
    to drain before the next one overwrote it sets a sticky `pub_missed`, OR'd
    into the **next** published line as bit 64. A ring would have hidden the
    problem; silent decimation is worse than a gap, because **a decimated stream
    reads as a slow control loop** — the wrong conclusion for someone about to
    change a gain. Note this is a *different* failure from a `seq` gap: a gap is
    lines lost on the wire and shows up as a missing number, while an unpublished
    step leaves no hole to find, which is why it is flagged in-band and counted
    separately (`veloc_steps_missed`).
  - **The flag set splits the freeze reason three ways** (bit 2 frozen, bit 4
    because drv is slewing, bit 8 because the bridge is unavailable). **Bit 2
    alone is anti-windup doing its job under saturation; bit 2 with 4 or 8 is
    the loop being held off by something else.** They are identical in `out` and
    they want opposite corrections — one says the gains are fine and the output
    is limited, the other says the run measured an obstruction. That distinction
    is the entire reason the bits are separate.
  - **`telem on` stays the master switch**: `V` requires `telem_on &&
    telem_vel_on`, so `telem off` — which every host stop sequence already sends
    — remains a complete stop for both channels.
  - ⚠️ **Bandwidth is the binding constraint.** 115200 8N1 is 11.52 kB/s. A `T,`
    line is ~45–59 bytes and a `V,` line ~50–76. `T` at 100 Hz plus `V` at 50 Hz
    is **~9.7 kB/s, 84% of the link**, before the echo of anything typed —
    and the console echoes each character as its own write. **`telem rate 50` is
    the pairing that fits**, and `telem vel on` warns when `telem_ms < 20`.
    The TX ring is 1024 bytes with a `tx_dropped` counter, which is the check.

  ### Host side — `node.py`, `bench.py`

  - **Both records are parsed from ONE regex alternation, not two `finditer`s.**
    `_consume()`'s residual-rejoining — the logic that reassembles a command echo
    a telemetry line landed inside of — depends on matches arriving **ordered and
    non-overlapping**. One regex guarantees that; two merged iterators do not.
    Dispatch is on `m.group(0)[0]`. Verified offline against a synthetic stream
    with a `V,` line cutting `drv timeout 2000` in half: the echo reassembles
    intact and both records come out.
  - `Veloc` dataclass with `sp_rpm` / `meas_rpm` / `error_rpm` and a boolean
    property per flag, plus `freeze_reason` returning the *reason*, not the bit.
  - `velocity.csv` in every run directory; `veloc_samples`, `veloc_gaps` and
    `veloc_steps_missed` in `meta.json` and `status.json`, and **either of the
    last two now marks a run `suspect`** alongside the existing `seq` gaps and
    `tx_dropped`.
  - **`dwell()` gained `vel_kick`.** The velocity watchdog is armed at 1000 ms
    and **only `vel target` refreshes it** — `drv timeout` kicks the layer below
    and does nothing for it. Without this a 6 s dwell would coast the wheel one
    second in, in the middle of the measurement, and the data would look like a
    plant that cannot hold speed.

  ### ⚠️ THE FINDING THAT GENERALISES: `safe_stop()` did not stop the loop

  `node.safe_stop()` had sent `drv duty 0` → `drv coast` → `drv disable` →
  `telem off` since the tool was written. **With the velocity loop armed, the
  first two are overwritten by the loop about 20 ms after they land.** Only
  `drv disable`, cutting nSLEEP, actually stopped anything. The sequence still
  worked — by accident, and only because of its last step. `vel off` now goes
  first.

  The general form: **arming a control loop invalidates every stop sequence that
  addresses the layer below it.** This is task 21's watchdog-defeat problem seen
  from the other side — there the loop's continuous calls *kept alive* a
  watchdog meant to detect a dead host; here they *overrode* a stop. Both are
  one layer holding another's state open. It will recur at CAN and again at the
  rover supervisor, and it is worth checking for deliberately each time rather
  than finding it.

  ### `bench.py run step`

  Commands a setpoint step and reduces the `V,` rows to: rise time (10→90% of
  the commanded change), overshoot, settling to ±2%, steady-state error,
  saturation fraction, freeze fraction split by reason, and **the integrator's
  resting value — which is how much the feedforward missed by**. A large steady
  `i` with a small steady-state error says `ff_a`/`ff_b` want re-fitting, not
  that Ki wants raising; without the term breakdown those two look the same.

  - **Settling is the LAST moment outside the band, not the first moment inside
    it.** A response that dips back out is not settled, and the first-crossing
    definition would call it settled anyway.
  - **`enc window` defaults to 20 here, NOT the sweep's 100.** This is the one
    argument default that must not be copied across: the loop advances once per
    window, so window 100 is a **10 Hz control loop with 100 ms of measurement
    lag**, which would dominate the very response being measured — gains chosen
    against it are gains for a different plant. `--window` therefore has no
    single default any more; it is per-profile, and `step` warns above 40.
  - **`cfg ramp_pmps 0` is sent unconditionally.** `drv ramp` and `vel_slew` are
    two slew limiters in series and must not both be armed: with drv's running,
    `drive_slewing()` is true almost continuously, the integrator is frozen for
    essentially the whole run, and the step measures the limiter.
  - **`--max-rpm 22`** is the 30% duty ceiling pushed through the plant fit
    (0.7993 × 300/10 − 2.42 = 21.6 rpm), expressed in the units this profile
    commands. Same explicit-raise pattern as `--max-duty`.
  - **`--return`** steps back down, and is not symmetry-checking for its own
    sake: the feedforward applies its friction offset **with the sign of the
    setpoint**, so the down-step is the one place a wrong `ff_b` shows up as a
    different *response* rather than as a constant error.
  - ⚠️ **Every metric is computed from `meas_mrpm`**, the boxcar the controller
    acted on. That is the right frame for choosing gains — it describes the
    closed loop as the loop experienced it — and the **wrong** frame for a plant
    time constant. The standing warning about fitting `mrpm` applies with more
    force inside a loop, because the filter's lag is now in the feedback path.
    Plant-side timing still comes from the `T,` rows and `rpm_from_counts()`.

  ### State

  Firmware builds clean under `-Wall -Wextra` at **27.14% flash (106 728 B of
  the 384 kB application region), 4.35% RAM (5696 B)**. Both Python files parse; the parser and the metrics
  reducer were exercised offline against synthetic streams (interleaved echo,
  gapped `seq`, a missed-step flag, an overshooting up-step and a decaying
  down-step) before any of it is pointed at a board. **The bench pass is owed,
  desk first and one step at a time:** bridge disabled to prove the publish/drain
  path and that the loop will not wind up against a dead bridge; `enc window 5`
  to force bit 64 and confirm overrun is reported; the 1000 ms watchdog at the
  desk; the parser against `run step --rpm 0`; a `run sweep` non-regression;
  and only then the rig, `--rpm 10` before `--rpm 20`, watching `ma` against the
  1580 mA trip before trusting any gain.

- **Sep 26 — TASK 21: THE DUTY SLEW LIMITER IS IMPLEMENTED, INSIDE `drive.c`,
  which reverses the placement this project had written down. Code is complete
  and building clean; nothing is verified on hardware yet.**
  - **Why the placement moved.** Task 21 said the limiter "belongs in the
    control layer, not `drive.c`". Reading the code before writing any showed
    that cannot work: `drive_set_duty()` calls `drive_kick()` on every call,
    deliberately — the file's own comment is that "a duty command is proof the
    caller is alive". Any ramp module sitting above `drive.c` must reach the
    bridge through that function, so it would refresh the command watchdog
    **1000 times a second, forever**, and a dead host would never again be
    detected. The safety feature added on Sep 25 would have been silently
    disabled by the safety feature added on Sep 26. Secondary reason: a limiter
    that callers can route around by calling `drive_set_duty()` directly is
    advisory, not a limit.
  - **It is therefore the command watchdog's own split, reused.** Mechanism in
    `drive.c` (a rate, and a bridge that walks at it); policy above (whether to
    ramp, how fast, what to do about a fault). `drive.c` still does not act on
    nFAULT, and **setpoint** ramping — the kind that stops a velocity PID
    winding up against its own ramp — is still the control layer's job and is
    explicitly not in this change.
  - **The arithmetic, and why the accumulator is in milli-per-mille.** The rate
    that matters is sub-unit per tick: 5%/s = 50 per-mille/s = **0.05 per-mille
    per 1 ms tick**, which an integer per-mille accumulator cannot represent —
    it would either stall at zero or, rounded up, run 20× fast. So `applied_mpm`
    is `int32_t` in thousandths of a per-mille. The scaling then collapses to an
    exact identity: `rate [o/oo per s] × 1000 [milli per o/oo] ÷ 1000 [ticks/s]
    = rate`. **The per-tick step in milli-per-mille numerically equals the
    configured rate in per-mille per second.** No division in the ISR, no
    accumulator residue, and the ramp lands exactly on its target. Full scale is
    1 000 000 mpm; at the 10 000 pmps ceiling that is 100 ms end to end.
  - **Structure.** The body of the old `drive_set_duty()` — decay branch,
    `apply()`, `place_trigger()`, `duty = permille` — became a private `emit()`
    that deliberately does **not** kick the watchdog. `drive_set_duty()` now
    kicks, clamps, stores `target`, and with a rate armed **writes nothing to
    the timer**: the tick owns the bridge from there. `ramp_step()` runs from
    `drive_on_tick()` *after* the watchdog and *before* the `if
    (!drive_faulted()) return;` early return, for the reason already written
    above the watchdog block — a ramp that only advances while something is
    wrong is not a ramp. The CCR is rewritten only when the truncated per-mille
    value actually changes, so at 5%/s that is **50 writes/s, not 1000**.
  - **The floor, and why it is in the command path rather than the tick.**
    Breakaway is 9–11% duty loaded; a ramp from zero at 5%/s would spend **2 s
    energised below breakaway**, stalled, no back-EMF, drawing the 275–467 mA
    that the trip exists for. `bench.py` solved this with `--ramp-from 12`.
    `ramp_floor` does the same, evaluated once, conditioned on
    `applied_mpm == 0`. Conditioning on *being at rest* rather than on the tick
    is what stops a **direction reversal** re-triggering the jump at the zero
    crossing — a wheel that is still turning has back-EMF and needs no floor.
    `bench.py`'s 1 s hold at the floor was for *measurement*, not protection, so
    it was deliberately not ported.
  - **What is not ramped, and why.** `drive_coast()` and `drive_brake()` are
    immediate — coast is the safe stop, the watchdog's action and
    `drive_disable()`'s first step, and a dead host is not the moment to ease
    off over six seconds; brake is an explicit act. The standing "ramp duty down
    before braking" policy is therefore written **above**, as
    `drive_set_duty(0)` → wait `!drive_slewing()` → `drive_brake()`, which is
    what the new accessor exists for. A tightened `drive_set_limit()` is
    immediate too — it is protection, not a command — and clamps `target`
    before `applied_mpm`, an order that is load-bearing and commented as such.
    `drive_set_decay()` now re-emits the applied value instead of calling
    `drive_set_duty()`, which would have reset the ramp.
  - **Concurrency, stated accurately.** The ISR is the only writer of the CCR
    pair *while a ramp is in flight*. The command path still writes at discrete
    moments (floor jump, coast, brake, limit, decay, and the rate-0 path), and
    each of those sets `applied_mpm` **before** emitting, so the worst an
    interleaved tick can do is emit the same value twice. That is the same
    two-instruction window the file already reasons about and accepts for the
    watchdog's coast. No critical section was added.
  - **`drive_set_ramp(0)` holds the bridge where it is.** The first version
    jumped it to the target on disarm. That is wrong: a *configuration* command
    must not produce a current step, least of all at the moment the operator has
    just decided they no longer want ramping. It now adopts the applied value as
    the new target.
  - **Console.** `drv duty <n>p` commands per-mille (`strtol` with an end
    pointer; ×10 only when the suffix is absent), backward compatible, and it
    **unblocks the 1%-step stiction bracket across 8–13%** that was listed as
    blocked on exactly this. `drv ramp [<o/oo per s> | floor <o/oo>]` sets and
    reports; bare `drv` prints rate, floor, target, applied and whether it is
    slewing.
  - **Config.** `ramp_pmps` (0..10000, default 0) and `ramp_floor` (0..300,
    default 0). Both **0 = off**, on the watchdog's precedent that nothing
    changes behaviour until armed. `config_set()` is already the volatile
    set-for-this-session path, so a rate scan costs no flash wear.
  - **Two things learned from reading `cfg` before flashing.** ⚠️ **Adding a
    config key discards the board's stored record** — `config.c` rejects any
    record whose `count != CFG_KEY_COUNT`, so the first boot after this change
    falls back to compiled defaults and would lose `vdda_mv`, `r_ipropi`,
    `vref_div`, `rail_mv`, `trip_ma`, `duty_limit`. `CONFIG_VERSION` is **not**
    what guards this, which corrects the note in task 21 that paired the two.
    On *this* board it cost nothing — `cfg` reported `slot 0/1024 used` with no
    overrides, i.e. `cfg save` has never been run here — but that is luck, not a
    reason to skip the check.
  - **Verified offline only.** The ramp arithmetic was transcribed to Python and
    checked before the board was touched: the floor jump, no floor re-trigger at
    a reversal's zero crossing, a tightened cap being instant and sticky, exact
    arrival on target, and the 50-writes/s figure. 8/8 pass. Builds at
    **96 972 B flash (24.66%), 5 456 B RAM (4.16%)**, no warnings.
  - ✅ **THE BENCH PASS IS DONE, and the feature is closed.** Taken in two
    stages: everything that could be proven with the driver *disabled* was
    proven at the desk first, and only then was anything energised.
    - **Desk, driver disabled — six checks, all exact.** Rate: 289 telemetry
      samples for 289 duty increments, last-zero to first-290 = **5800 ms**
      against 5.80 s predicted, landing and stopping with no overshoot. Floor:
      the sample before the command reads 0 and the next one, 20 ms later, reads
      **120** with no intermediate values; the 120→290 climb took **3400 ms**
      against 3.4 s predicted. Down-ramp 290→0 took **5800 ms**, symmetric, and
      passed straight *through* 120 without the floor firing. Reversal walked
      `… 2, 1, 0, 0, −1, −2 …` — **no floor jump at the zero crossing**, which
      is what conditioning the floor on `applied_mpm == 0` was for — 8000 ms for
      400 o/oo, and the floor re-armed correctly after a `drv coast`.
    - ⚠️ **A gap of my own found and fixed before energising:** the
      `cfg <key> <value>` handler applied values live for only two keys, so
      `cfg ramp_pmps 50` would have stored the rate and left the limiter at 0
      until reboot — exactly the "cfg and drv disagreeing about the same number"
      failure the surrounding comment warns about, and worse because `drv ramp`
      actively *tells* the operator to type that command. Extended to all four;
      verified on hardware, all four now print `applied now:` and the "not
      persistent" hint correctly disappears.
    - **Rig, motor live.** Logged to file at 100 Hz by script rather than pasted
      from a terminal — **1462 lines, zero seq gaps, no fault latch, no ADC
      saturation.** Slew rate fitted over 340 samples: **50.00 o/oo/s** against
      50 commanded. Climb 120→290: **3400 ms against 3400 predicted.** Floor
      jump in a single sample with the **encoder moving 10 ms later** — the same
      breakaway latency the hard step achieves, so the ramp costs nothing at
      start. Sync set at duty **145**, as on every run since Sep 20. **Peak
      synchronised current 581 mA**, within 1.6% of the host prototype's 572 and
      **2.7× under the 1579 mA trip.**
    - ✅ **The A/B against `drv ramp 0` — the evidence the feature works.** Same
      0→29% command, un-ramped: **1582 mA on the very first telemetry sample**
      against a trip programmed at 1579, then 1300 / 1008 / 911 / 913 mA over
      the next 40 ms before back-EMF built and it fell away. Reading the number
      correctly matters: **1582 is the clamp's value, not the demand's** — the
      driver was in ITRIP regulation and at 10 ms sampling the true peak is
      unknown and higher. So the honest claim is that **the ramp cuts peak
      inrush at least 2.7×**, from at-the-limit to 37% of it.
    - ⚠️ **And the un-ramped case latched NO fault** — `flags` never set bit 4
      or bit 8 through the whole regulated event. Stepping duty from rest does
      not fail *visibly*; it silently leans on the hardware current limit on
      every start. That is a stronger argument for the limiter than a visible
      trip would have been, because nothing in the telemetry would ever have
      surfaced it. Written up in *Key learnings*.
    - ✅ **The watchdog non-regression test passed twice.** At the desk first,
      deliberately reordered ahead of the scope step and run with the driver
      disabled — just stop typing — so the decisive question was settled before
      anything was energised: command at ms 922420, duty 99 at 924400, **0 at
      924420 = exactly 2000 ms**, snapping to zero in a single sample. Then
      repeated **with the motor live**: the ramp had been writing the CCR for
      3.4 s and holding for 6.6 s, and the 10 s deadline still landed on the
      **exact millisecond**, duty **290→0 in one sample** — a coast, not a
      ramp-down. Coast to standstill took 540 ms. **`emit()` is not kicking the
      watchdog, so the placement argument holds.**
    - ⚠️ **An unexplained mechanical event, seen once and not reproduced.** On
      an earlier attempt the wheel *slowed while duty was still rising* —
      10.0 → 6.4 rpm over ~100 ms at duty 157→167, confirmed in the raw `count`
      deltas, not a filter artifact, with current tripling to 580 mA. On the
      clean run the largest such drop was 3.57 rpm, inside the boxcar ripple.
      Most likely belt take-up or a tight spot. Recorded because a repeat would
      mean something real.
    - ⚠️ **Both runs settled 4.5% below the Sep 25 fit** — 19.82 rpm (ramped)
      and 19.97 (un-ramped) by count slope at 29% duty, against 20.76 from
      `rpm = 0.7993 d − 2.420`. Two runs agreeing with each other and
      disagreeing with the fit points at the fit. The plateau also showed the
      **2.75 s belt pole** plainly, rising 19.28 → ~20.7 rpm over the first two
      seconds before settling, which is why the settled figure is taken over the
      last 5 s and not the whole plateau. Flagged in NEXT TASKS as a re-take.
    - **Two caveats kept rather than buried.** At 50 Hz the desk telemetry
      aliased 1:1 with the emission rate, so it proves the average rate but
      cannot resolve sub-20 ms jitter; and the scope check of PWM high time was
      **not** done — the 50.00 o/oo/s fit and the exact 3400 ms climb measure
      the same thing from the telemetry side, so it was judged redundant rather
      than skipped silently.
    - **A hazard noticed, deliberately not fixed.** After the reversal test the
      bridge was parked at duty −200 o/oo with nSLEEP low; `drv enable` from
      there would energise the motor at 20% reverse instantly with no ramp,
      because the floor only fires from `applied_mpm == 0`. Pre-existing, not
      ramp-caused, but the ramp makes it easier to land there unnoticed.
      `drive_enable()` arguably should force duty 0 first — out of scope, left
      open.
    - ⬜ **`cfg save` has NOT been run.** The board still boots with the ramp
      **off**; 50 / 120 are live-set only.

- **Sep 25 (later) — LOADED-RIG PASS TAKEN, AND τ MEASURED TWICE BY TWO
  INDEPENDENT ROUTES. The plant is two-pole, and the τ ≈ 0.65–0.70 s reported
  earlier this session was wrong.** Two 30 s-per-point sweeps in 2% steps with
  the wheel on the treadmill belt, then the transient analysis the sweeps were
  really for. The headline correction: **τ_fast = 0.219 ± 0.007 s**, not 0.7 s.

  **Conditions, recorded rather than typed** (both runs' `meta.json`):
  wheel on the **treadmill-belt rig**, wheel + aluminium carriage **1047 g**
  pressing on the belt, 12 V rail, `enc window 100`, telemetry 50 Hz,
  `drv trip 1580`, watchdog 2000 ms, **30 s dwell**, `settle 0.25` (the leading
  quarter of each dwell discarded), 1125 settled samples per point.
  - Ascending `21-29-48`: 5→39% in 2% steps, from rest at each point.
  - Descending `22-00-34`: **entry ramp 12→29% at 5%/s**, then 29→5% in 2%
    steps, `--max-duty 30`. The ramp was asked for explicitly — it is the
    stand-in for the firmware slew limiter that does not exist yet, and it is
    also what made the second τ route possible.
  - **All four integrity counters zero on both runs**: 0 seq gaps, 0
    `tx_dropped`, 0 echo mismatches, `suspect: false`.

  **The 1047 g is dead weight, not accelerated mass.** The rig presses one
  wheel against a belt, so the load it applies is a *normal force*: the friction
  is honest and the inertia is not. The rover is ~18 kg over six wheels, ~3 kg
  per wheel, so the rig is **2.87× light in inertia**. Every friction and
  steady-state number below transfers; the mechanical time constant does not,
  and τ on the rover will be longer.

  **Steady state, 11–29% (the band that matters — 30% is the stated ceiling):**

  | pass | fit | R² | max\|res\| | rms |
  |---|---|---|---|---|
  | loaded ascending | `rpm = 0.7924 d − 2.137` | 0.99751 | 0.333 | 0.227 |
  | loaded descending | `rpm = 0.8062 d − 2.704` | 0.99933 | 0.230 | 0.120 |
  | **loaded pooled** | **`rpm = 0.7993 d − 2.420`** | 0.99735 | 0.458 | 0.236 |
  | free wheel, same band | `rpm = 0.8356 d − 1.573` | 0.99987 | 0.100 | 0.058 |

  - **The load costs 4.3% of slope and 0.85 rpm of intercept.** That is the
    whole effect of putting 1047 g on the belt. It is a much smaller slope
    change than expected and it **re-confirms the Sep 25 mounting rule from the
    other side**: the fixture moves the intercept, the motor and the rail own
    the slope.
  - **Repeatability floor: ±0.174 rpm** between the two passes (mean
    descending − ascending = **−0.292 rpm**). Nothing smaller than that is a
    measurement on this rig, and several tempting sub-0.2 rpm effects were
    discarded against it.

  **Breakaway and dropout both fall in 9–11% duty**, from the logged counts:
  9% held `count` at exactly 0.00 rpm on both passes; 11% ran at 6.27 (asc) and
  6.00 (desc) rpm. With 2% steps the two thresholds cannot be separated — that
  needs a 1%-step run, which the console cannot do without a per-mille `drv`
  verb (already logged as an open item).

  > **BREAKAWAY IS A VOLTAGE THRESHOLD, NOT A DUTY THRESHOLD.** Sep 15 measured
  > breakaway at 12–14% duty on this same rig on the **9.35 V** rail; today it
  > is 9–11% on **12.03 V**. Bracket midpoints: 0.13 × 9.35 = **1.216 V** and
  > 0.10 × 12.03 = **1.203 V** — **1.0% apart**. The brackets themselves overlap
  > (1.12–1.31 V then 1.08–1.32 V), so this is *consistent with* a fixed
  > terminal-voltage threshold rather than a proof of one; a 1%-step run would
  > tighten it. It already carries the useful consequence: **breakaway duty is
  > not portable across rails, and the Sep 15 figure was never stale data — it
  > was the same physics at a different rail.**

  **No Stribeck cliff at 11% any more.** Sep 15 saw speeds bend hard below 12%
  and cliff at 11% → 4.86 rpm. Today 11% sits **on** the straight line
  (residuals −0.10 and −0.37 rpm against the pooled fit). The cliff moved below
  11% with the rail, which is the same voltage story.

  **Current separates Coulomb friction from viscous friction, cleanly:**
  - **Free wheel: 232.5 ± 18.5 mA, slope −0.85 mA/%** — flat, i.e. consistent
    with zero. Constant torque, no speed dependence. *Coulomb.*
  - **Loaded, 17→29%: 266.4 → 312.6 mA, slope +4.4 mA/%** — a real, positive
    trend. *Viscous*, and it is the belt contact that added it.
  - The trend is taken from **17%** up, not 15%, because the 15% point
    (312.4 mA against ~266 for its neighbours) sits **on the 14.5% sync gate**
    and is the gate's marginal edge, exactly as the free-wheel pass found. It is
    plotted, not hidden.
  - ⚠️ **Below the gate the numbers are not current and this run shows why
    loudly.** Stalled at 5/7/9% the readings *rise* 320→453 mA (asc) and
    276→467 mA (desc) — a stalled motor at 9% duty reading higher than a running
    one at 29%. Physics permits nothing of the sort. The console is right to
    refuse these as `NOT SYNCHRONISED`.

  **Above ~31% the wheel bounces, and the data says so numerically.** The
  ascending run went to 39% (the descending one was capped at 30% deliberately).
  Slope over **31–39% is 0.9219 rpm/%** against **0.7924** over 11–29% —
  **16.3% steeper**, with 39% overshooting the low-band extrapolation by
  +0.50 rpm. The belt surface is not homogeneous, the wheel starts to skip, and
  less contact means less friction and more speed. **That is a property of the
  rig, not of the plant**, and it is why the characterisation ceiling is 30%.

  **The curvature in the loaded band is real, and it is NOT thermal drift.**
  The pooled residuals bend. The discriminator: **the ascending and descending
  residuals correlate at +0.718**. Thermal drift follows *elapsed time*, so
  reversing the duty order flips its sign and would make the two residual sets
  **anti**-correlate. They agree instead, so the shape belongs to the duty axis
  — real plant curvature. (This is the same discriminator that identified the
  free-wheel asc/desc gap as warming, run the other way round.)
  - Quantified: a quadratic gives local gain **0.899 → 0.700 rpm/%** across
    11→29%, a **25% droop**.
  - **The model stays linear anyway.** The quadratic buys rms 0.236 → 0.174 rpm
    — 0.063 rpm — against a **0.174 rpm repeatability floor**. It is fitting the
    noise budget. The 25% gain droop is carried as a **known PID design
    constraint** (design the loop at the low-gain end of the band) instead of as
    a model term.
  - Also discarded on the same grounds: point-to-point local gain. The
    ±0.174 rpm floor over a 2% step is **±0.123 rpm/%** of apparent gain, which
    is most of the visible swing. The gain curve is drawn against that noise
    band rather than as a series of points.

  ---

  **τ — THE PART THAT WAS WRONG, AND HOW IT WAS FIXED.**

  **τ ≈ 0.65–0.70 s was reported twice earlier in this session. It is wrong.**
  Three errors compounded, in increasing order of importance:

  1. **The ramp rate was computed as 3.75 %/s** by including the 1 s breakaway
     hold in the ramp interval. The true rate, read off the `duty` column
     itself, is **4.814 %/s**.
  2. **The ramp-end speed was read off `mrpm`.** `mrpm` is a 100 ms boxcar,
     ~50 ms of lag, and on this rig it does not even lag cleanly — it swings
     ±1.4 rpm around the count-derived speed because the belt has a ~2.5 Hz
     ripple. **`count` is the measurement.** This is already a documented rule
     and it was broken anyway.
  3. **The real one: a single exponential was fitted to a two-pole system.**
     This is what produced the number, and neither of the other two would have
     mattered without it.

  **The diagnostic that exposed it: refit over shrinking windows.** A genuine
  first-order system returns the same τ at every fit horizon. This one does not:

  | fit horizon | one-pole τ |
  |---|---|
  | 1 s | 0.290 s |
  | 3 s | **0.496 s** |
  | 10 s | **0.724 s** |

  It climbs monotonically and never settles — and the 10 s value is essentially
  the number originally quoted. **That figure was never a time constant; it was
  an artifact of the fit window.** The two-route disagreement that started the
  investigation (0.225 s from one method, 0.496 s from the other, a clean 2×)
  was the same fact seen from the side.

  **Fixed with a two-pole fit, and then verified from two genuinely independent
  measurements:**

  | route | excitation | data | τ |
  |---|---|---|---|
  | **ensemble step response** | 42 × 2% duty steps, stacked | both runs | **0.219 ± 0.007 s** (weight 84%) |
  | **ramp-tracking lag** | the 12→29% entry ramp at 4.814 %/s | descending run, 185 samples | **0.207 ± 0.007 s** |

  > **THE TWO AGREE TO 5.7%.** Different excitation (a step versus a ramp),
  > different data (every dwell transition versus one continuous ramp),
  > different estimator (a curve fit versus a steady-state lag). That is what
  > makes it a verification rather than a repeat — the same failure cannot
  > produce both, and the one-pole artifact above demonstrably could not.

  - **Slow pole: τ_slow = 2.75 ± 0.05 s, weight 16%.** Belt and contact
    settling, not the motor. **This is why the 30 s dwell was necessary** and
    why a shorter one would have quietly biased every steady-state point; it
    also retroactively explains the earlier 3.121 s "relaxation" fit, which had
    caught this pole alone.
  - **Two-pole vs one-pole fit quality: 11.5 vs 58.4 mrev rms — 5.1× better.**

  **Why the individual steps had to be stacked.** One 2% duty step moves the
  wheel ~1.6 rpm against 0.5–0.7 rpm of belt ripple — under 3:1 SNR, which is
  why single-step fits scattered uselessly. Stacking all 42 beats the ripple
  down by √42. Two details that made the stack work:
  - **The ensemble is built in DISTANCE, not velocity.** Integration is a
    low-pass, so no differentiation noise enters the fit at all. Velocity is
    derived only for display, and there through a Savitzky–Golay filter
    (11 samples, cubic ≈ 0.22 s) drawn over the raw derivative.
  - **Each step is normalised by its own final velocity change** before
    stacking, so the 25% gain droop across the band does not smear the average.

  **Headroom, for the record:** peak current through the entry ramp was
  **572 mA against the 1580 mA trip — 2.8×**. The ramp never came near
  tripping, which is the answer to why the earlier un-ramped step entries did.

  **Figures** — `docs/environment/figures/`, both matching the free-wheel
  figure's conventions and palette (validated by `vpal.py`, a Python port of the
  dataviz validator written because this bench has no Node: normal-vision min
  ΔE 24.0, CVD min ΔE 9.2, all above the floors):
  - `plant_12v_loaded_rig.png` (`plot_plant_12v_loaded.py`) — the steady-state
    pass: the band with both directions and the free-wheel reference, current
    versus duty, the asc/desc residual correlation that killed the thermal
    hypothesis, and local gain against its noise band.
  - `plant_12v_loaded_tau.png` (`plot_plant_12v_tau.py`) — the transient pass:
    the stacked ensemble with one-pole and two-pole fits, the
    window-dependence trap as its own panel, the independent ramp-tracking
    route, and all five τ values on one log axis with the artifacts drawn
    hollow.
  - **Both scripts derive every number from the committed run directories**
    (`rigdata.py` is the shared loader) — nothing is transcribed, and the
    console output of each script is the source of the numbers in this entry.
  - The four run directories are **committed** (`git add -f`, since `runs/` is
    gitignored) so the analysis stays re-runnable.

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


## Moved from the context file — 2026-09-26 reorganisation

Verbatim material removed from the hot file when it was split into tiers.

### Header history — the `Last updated` / `Previously` chain

# RobertUN — Wheel Controller Firmware Context
**Last updated:** September 26, 2026 (**THE STAIRCASE WAS RUN IN BOTH DIRECTIONS, AND THE DIRECTION ASYMMETRY IS ENTIRELY THE INTEGRATOR.** A second 21-minute staircase at **−10.0 → −20.0 rpm** (63,203 `V` rows, 0 gaps, 0 unpublished steps) tracks **better** than forward — worst settled error **0.006 rpm** vs 0.015 — with **0% saturation at all 21 holds**, peak 277 of 300 o/oo. ⚠️ **That corrects a prediction that the top of the reverse range would saturate**, extrapolated from the single −10 rpm check; it does not, because the asymmetry **changes sign at 14.3 rpm**. **`corr(Δ|out|, Δi) = 0.9986`, 0.62 o/oo rms apart** — the feedforward is symmetric *by construction* (it takes the sign of the setpoint), so every bit of direction dependence lands in the integrator, which makes **the integrator a direct readout of the model's direction error** and is how a separate reverse `ff_b` gets measured rather than guessed. ⚠️ **"Reverse is x% harder" is false**: the asymmetry spans −10.0 to +1.7 o/oo and clears the comparison's own **±2.16 o/oo noise floor at only 9 of 21 setpoints**. Reverse draws **+8.1% current while commanding LESS output** — real load or sign-dependent current sense, undecidable from these two runs. ⚠️ **A panel was caught about to ship a false claim:** the ripple count came out *identical to every digit* in both directions, which was the 20 ms lag grid (**±0.72 events/rev**), not agreement — sub-bin refinement tightens it 11× and gives **11.998 ± 0.017 forward, 11.978 ± 0.017 reverse**, i.e. the same feature at **exactly 12 per revolution**. ⚠️ **The confound is stated on the figure:** the runs are **67 min apart and not interleaved**, so temperature, belt tension and belt position are aliased into "direction"; an **A/B/A staircase has not been run**. `bench.py` gained the **`stair` profile** — with a sign bug caught before it ran, where a ceiling guard written as `max(setpoints)` would have read −10 and waved through any speed in the very direction about to be tested)

**Previously:** September 26, 2026 (**THE VELOCITY LOOP RAN FOR 21 CONTINUOUS MINUTES ON THE RIG, AND THE SHIPPED GAINS ARE GOOD.** 21 setpoints 0.5 rpm apart, 10→20 rpm, 60 s each, uninterrupted: **mean tracking error +0.0008 rpm** against a 0.020 rpm instrument floor, **0% saturation at every hold** (peak 284 of 300 o/oo — the 6% an earlier short step reported was the acceleration transient, not the operating point), and **zero drift** over each 60 s. The closed-loop plant inverse `out = 12.559 × rpm + 30.54 o/oo` sits **0.39% from the Sep 25 open-loop sweep inverted**, from different excitation and a different estimator — **the feedforward is doing essentially all the work** (integrator mean +1.5 of ~219 o/oo) and the ¼-of-textbook derating is costing no accuracy. ⚠️ The **±1 rpm ripple is MECHANICAL and the run proves it**: 11.91 ± 0.19 events *per output revolution*, held across a 2:1 speed range while the period swept 500→260 ms — a limit cycle holds a fixed period, only a rotating feature holds a fixed count per rev. ⚠️ **Then `bench.py run step`'s metric was caught measuring `vel_slew` under the loop's name** — a 0→10 step returned "rise 2.0 s" whatever Kp was, because 2.0 s is 0.8 × the 2.5 s the 4 rpm/s setpoint ramp costs on its own. Rise/overshoot/settling now anchor at the **end of the ramp**, `track_lag_rpm` measures the loop *during* it, and the **"14% overshoot" turned out to be a ripple peak** — 4 encoder counts above target at both 10 and 20 rpm, `overshoot_above_ripple` **False on all three runs**. Two figures added; `stepdata.py` **imports `step_metrics` from `bench.py`** so the figure cannot drift from the tool)

**Previously:** September 26, 2026 (**W5's VELOCITY PID MODULE IS WRITTEN, AND SO IS THE INSTRUMENT TO TUNE IT WITH — both on `w5-velocity-pid`, neither yet run on hardware.** `velocity.c`/`.h` is a **policy layer above `drive.c`**, stepping at the *measurement* rate (once per `enc window`, 50 Hz default) rather than on the 1 kHz tick, with feedforward from the inverse plant, the **setpoint ramp that task 21 said belongs up here**, four named anti-windup freeze conditions, and nine `cfg` keys in milli-units. Then the telemetry: a second opt-in **`V,` record — one line per control step**, carrying setpoint, measurement, output and **all four terms separately**, plus a flag set that splits *why* the integrator froze. Host side gained the parser, `velocity.csv`, and **`bench.py run step`** — rise time, overshoot, settling, steady-state error, and the integrator's resting value, which is how much the feedforward missed by. ⚠️ **The finding that generalises: arming a control loop invalidates every host-side stop sequence that talks to the layer below it.** `safe_stop()`'s `drv duty 0` / `drv coast` were being overwritten by the loop 20 ms later; `vel off` now goes first. Same shape as task 21's watchdog-defeat problem, one layer up. ⚠️ Also fixed: **`velocity.c` shipped with no `volatile` on any ISR-written static** — caught while adding the snapshot, before it cost anything)

**Previously:** September 25, 2026 (**LOADED-RIG PASS TAKEN, AND THE PLANT'S TIME CONSTANT MEASURED TWICE BY TWO INDEPENDENT ROUTES.** Two clean 30 s/point sweeps in 2% steps on the treadmill belt at 1047 g: pooled **`rpm = 0.7993 d − 2.420`** over 11–29%, **the load costs only 4.3% of the free-wheel slope** and 0.85 rpm of intercept. **Breakaway 9–11% duty is the SAME TERMINAL VOLTAGE** as Sep 15's 12–14% on the 9.35 V rail (1.203 V vs 1.216 V) — breakaway is a voltage threshold, not a duty one. Current separates **Coulomb (free, flat) from viscous (loaded, +4.4 mA/%)**; above ~31% the wheel bounces on the belt and the slope lifts 16.3%, which is the rig and not the plant. **⚠️ THE τ ≈ 0.65–0.70 s REPORTED EARLIER THIS SESSION IS WRONG** — the plant is **two-pole**: **τ_fast 0.219 ± 0.007 s** (84%) + **τ_slow 2.75 s** (16%, the belt), verified against an independent ramp-tracking lag of **0.207 ± 0.007 s** — **5.7% apart**. A one-pole fit returns a window-dependent artifact that climbs to 0.724 s at a 10 s horizon. **The model stays linear**; the 25% gain droop is carried as a PID design constraint. Rig inertia is **2.87× light** vs the rover, so τ there will be longer)

**Previously:** September 25, 2026 (**BENCH TOOLING BUILT, AND THE 12 V FREE-WHEEL PLANT RE-TAKEN WITH IT.** Firmware gained `telem` (machine-readable stream, 1–100 Hz) and `drv timeout` (command watchdog, coasts on expiry); `tools/bench/` drives runs and logs them to files. Four clean sweeps, **wheel now CLAMPED to the table** rather than hand-held: **CW `rpm = 0.8327 d − 1.524`, CCW `0.8618 d − 1.164`, R² ≥ 0.9999**. **Task 17's CCW is CLOSED — +3.49% asymmetry against Aug 26's +3.5%**, confirmed by a different method, rail and mounting. **The mounting moves the intercept, not the slope** (clamped vs hand-held slope agree to 0.3%); the tool validated like-for-like against the hand-typed table to **−0.18%**. What reads as hysteresis is the motor **warming**. Loaded-rig pass and the deliberate watchdog test still owed)

**Previously:** September 23, 2026 (**12 V FREE-WHEEL PLANT MODEL MEASURED — `rpm ≈ 0.83 × duty% − 0.96`, one straight line from 3% to 100% duty, no hysteresis in the 5–30% operating band**; stiction is separate and large — breakaway 5–6% duty, dropout 2–3%, minimum sustainable speed ≈1.6 rpm; a duty slew-rate limiter was raised as a W5 requirement after every duty step fired the trip; **CCW and the loaded-rig pass still owed**)

**Sibling files:** `PROJECT_CONTEXT_REST.md` — machines, network, ROS 2/Jetson/Isaac, bus-wide CAN architecture, power distribution, and tooling. `PROJECT_CONTEXT_WHEEL_FW_LOG.md` — the full, unedited progress log behind the one-line summaries below. Paste this file alone for routine wheel-firmware session starts; pull in the log file only when you need the exact numbers/reasoning behind a specific entry.

*Split from the original PROJECT_CONTEXT.md on Sep 17, 2026. See PROJECT_CONTEXT_REST.md for the split rationale.*

> **THIS IS THE DEFAULT FILE FOR A WHEEL-FIRMWARE SESSION.** Read it whole; do
> **not** also read the LOG file unless a specific entry's exact numbers are
> actually needed. That restraint is the reason the split exists.
>
> **When writing at the end of a session:** the detailed entry goes in
> `PROJECT_CONTEXT_WHEEL_FW_LOG.md`, and only **one summary line** comes back
> here. Durable rules go to KEY LEARNINGS below, open work to NEXT TASKS below.
> Keep this file's own length roughly flat over time — if a section here is
> growing into a narrative, it belongs in the log.


### Brief progress log — as it stood in the hot file

## Progress log (most recent first) — brief

One line per entry. Full detail (exact numbers, register values, reasoning chains) is in `PROJECT_CONTEXT_WHEEL_FW_LOG.md`.

- **Sep 26 (bench, reverse)** — **THE SAME STAIRCASE IN REVERSE, AND THE ASYMMETRY IS THE INTEGRATOR AND NOTHING ELSE.** 21 points, −10.0 → −20.0 rpm, 60 s each, uninterrupted (`runs/2026-09-26T11-02-46_stair`, **63,203 `V` rows, 0 gaps, 0 unpublished steps, 0 tx_dropped**). **Tracking is better than forward** — worst settled error **0.006 rpm** vs 0.015, per-point sem 0.019 — and **0% saturation at all 21 holds**, peak **277 of 300 o/oo**. ⚠️ **This corrects a prediction I made** from the single −10 rpm direction check, that the top of the reverse range would saturate: it does not, because the asymmetry **changes sign**. **The headline: `corr(Δ|out|, Δi) = 0.9986`, 0.62 o/oo rms apart.** The feedforward takes the sign of the *setpoint* ([`velocity.c:346`](../../firmware/RobertUN_ModuleNode/Core/Src/velocity.c#L346)) and is therefore symmetric **by construction**, so it cannot absorb a plant that is not — every bit of direction dependence lands in the integrator, which makes **the integrator a direct readout of the model's direction error**. ⚠️ **No single "reverse is x% harder" is true:** the asymmetry spans **−10.0 to +1.7 o/oo** (mean −2.74) and **changes sign at 14.3 rpm**, clearing the comparison's own **±2.16 o/oo noise floor** (quadrature sum of the two fits' residuals) at **9 of 21 setpoints**. Reverse inverse **`out = 11.503 × |rpm| + 43.65`** (rms 1.54) vs forward `12.559 × rpm + 30.54` — different slope *and* intercept. **The ripple is the same mechanical feature, at 12.00 per revolution**: 11.998 ± 0.017 forward, 11.978 ± 0.017 reverse, paired difference −0.0204 ± 0.0030 (negative at 20 of 21) — resolved but 0.17%. ⚠️ **That panel was about to ship a false claim:** on the raw grid an autocorrelation period is an integer count of 20 ms control steps, worth **±0.72 events/rev**, so both directions returned *identical digits* at all 21 holds and reporting it as agreement would have claimed a precision the method does not have; sub-bin parabolic refinement (opt-in, so the forward figure's numbers do not move) tightens it 11×. The ripple's **amplitude** is not the same (0.87→1.21 rpm fwd, 0.88→1.00 rev). ⚠️ **Reverse draws +8.1% current (292 vs 270 mA) while commanding LESS output** — a real direction-dependent load or a sign-dependent offset in the current sense, undecidable from these two runs and partly inside the 145 o/oo sense floor. ⚠️ **THE CONFOUND, carried on the figure:** the runs are **67 min apart and NOT interleaved**, so temperature, belt tension and carriage position on a belt that travels the *other way* in reverse are all aliased into "direction"; **an A/B/A staircase would separate them and has not been run.** Tooling: `bench.py` gained the **`stair` profile** (one arming, long dwell per point, settled stats) — ⚠️ **two sign bugs were caught in it before it ran**, the fatal one a ceiling guard written as `max(setpoints)`, which reads **−10** for a reverse run and would have waved through any speed at all in the very direction about to be tested; `--lo`/`--hi` are now **magnitudes** with `--dir` carrying the sign, and the guard tests magnitude. Figure `velocity_loop_stair_direction.png` + script added.
- **Sep 26 (bench)** — **THE VELOCITY LOOP RAN 21 CONTINUOUS MINUTES AND THE SHIPPED GAINS ARE GOOD; THEN THE STEP METRIC THAT SAID OTHERWISE TURNED OUT TO BE MEASURING THE SLEW LIMITER.** A 21-point staircase, 10.0→20.0 rpm in 0.5 rpm steps, **60 s each, uninterrupted** — chosen over another short step because a step says whether the loop is *stable* and nothing about whether it is *accurate*. **63,202 `V` rows, 0 sequence gaps, 0 unpublished control steps, 0 tx_dropped.** **Tracking: mean error +0.0008 rpm, worst 0.015, sd 0.005**, against a per-point standard error of ~0.020 rpm — the loop is accurate below the noise floor of the instrument measuring it. **0% saturation at all 21 holds**, peak **284 of 300 o/oo**, which corrects the 6% an earlier short step reported at 20 rpm: that was the **acceleration transient**, not the operating point. **The derating costs nothing because the plant model behind the feedforward is right** — closed-loop inverse **`out = 12.559 × rpm + 30.54 o/oo`** (rms 1.51, n=21) against Sep 25's open-loop sweep inverted, **`12.511 × rpm + 30.28` — 0.39% apart** from different excitation and a different estimator; shipped `ff` error at 15 rpm **−1.3 o/oo (−0.6%)**, integrator **mean +1.46 o/oo of ~219 commanded**. ⚠️ **The "~4.5% optimistic" caveat on that plant model does not hold in this band.** ⚠️ **The ±1 rpm ripple is MECHANICAL and the run proves it rather than asserting it: 11.91 ± 0.19 events per output revolution**, range 11.5–12.2, held across a 2:1 speed range while the *period* swept 500→260 ms — **a control limit cycle holds a fixed period; only a rotating feature holds a fixed count per revolution.** Identifying which feature is a mechanical job. Within-hold drift **+0.0005 rpm / −0.04 o/oo over 60 s**, and the low-end current U-shape is **not** a warm-up transient (a 90 s re-take 22 min later reproduced 294→293 and 274→278 mA) — but it sits near the 145 o/oo sense floor and wants an independent ammeter. ⚠️ **THEN THE METRIC.** `run step` reported "rise 2.0 s, overshoot 14%" for 0→10 rpm; **both were measurements of something other than the controller, and both pointed at a gain change that would have made the loop worse.** `vel_slew` ships at **4 rpm/s**, so a 0→10 "step" is 2.5 s of ramp and 0→20 is 5 s — **the tell was that rise did not depend on Kp**, because 2.0 s is 0.8 × 2.5 s, the limiter's own 10→90% time. Overshoot and settling now anchor **at the instant `VFLAG_RAMPING` clears**; `rise_s` is still reported from the command but beside **`rise_slew_floor_s`** with **`ramp_limited`** set when it is within 30% of it; new **`track_lag_rpm`** measures how far behind the moving setpoint the loop sits *during* the ramp, **which is the quantity a gain change actually moves**. Replaying all three committed runs recovers **`slew_rpm_s` 3.97–3.98** against the configured 4.0, and the 0→20 run settles **2.94 s from the ramp's end vs 7.94 s from the command** — **the same instant**, 5.00 s apart only in what they subtract. ⚠️ **The corrected metric then exposed a second defect: the "overshoot" is a ripple peak.** A single maximum drawn from ~1.2 rpm sd of mechanical ripple sits 2–3 sd high whatever the gains do; all three runs peak **4 encoder counts (1.42 rpm) above target — the same distance at 10 and at 20 rpm**, which a controller's overshoot would not be. `step_metrics` now reports **`tail_sd_rpm`** and sets **`overshoot_above_ripple`, False on all three**, bounds the peak search to a 3 s window (the plant's slow pole is 2.75 s) so a longer `--dwell` cannot manufacture a bigger overshoot, and flags **`overshoot_window_truncated`** when a run was too short to fill it. **The honest verdict for all three is "no overshoot resolvable, and rise not measurable above the limiter"** — less satisfying than "lower Kp", and correct. A real step response needs **`--slew 0`, which has not been run.** Two figures added; ⚠️ **`stepdata.py` imports `step_metrics` from `bench.py` and calls it rather than recomputing it**, so the figure cannot drift from the tool it documents.
- **Sep 26 (later)** — **W5's VELOCITY PID IS WRITTEN, AND SO IS THE INSTRUMENT TO TUNE IT.** Branch `w5-velocity-pid`; **nothing here has been run on hardware yet.** `velocity.c`/`velocity.h` is a **policy layer above `drive.c`** — it decides, `drive.c` actuates — and it **steps at the measurement rate, not the tick rate**: `velocity_on_tick()` runs off TIM6 but advances only when `encoder_velocity_seq()` changes, which is once per `enc window` (50 Hz at window 20). A loop running faster than its sensor updates would differentiate a staircase and integrate the same error twice. Feedforward comes from the inverse plant (`duty% = 1.251 × rpm + 3.028`) with the friction offset applied **with the sign of the setpoint**, so the PID only has to correct the fit's error rather than build the whole output. **The setpoint ramp lives here**, which is exactly what task 21 said it would: `drive.c` has no setpoint, and ramping the setpoint is what stops the loop winding up against its own ramp. Anti-windup freezes the integrator under four named conditions, and **which one fired is reported**. Gains ship at **¼ of textbook** (Kp 3.0 / Ki 10.0 against 1/K = 12.51 and 1/(K·τ) = 57.1) on purpose — the plant fit they derive from is the one flagged 4.5% optimistic. Nine `cfg` keys in **milli-units**, because config is int32-only. ⚠️ **A real defect was found in what had just been committed: not one of `velocity.c`'s ISR-written statics was `volatile`** — latent while nothing copied them out, load-bearing the moment a snapshot did. Fixed before the snapshot was added. **Then the telemetry, which is the half that makes tuning possible at all.** The `T,` line says what the *bridge and plant* did; it says nothing about what the *loop decided*. So a second opt-in record, **`V,seq,ms,sp_mrpm,meas_mrpm,out,ff,p,i,d,flags`** — a separate record rather than more columns on `T,`, because `Telem.parse()` length-checks and every committed run directory holds a 7-field `telemetry.csv`, so a widened `T,` would mean two incompatible things depending on the reader. **Published once per control step, not on the `telem` timer**: riding that timer would alias the loop (duplicates at 100 Hz, beats at 30 Hz), and the integrator and derivative only mean anything per step. **One slot with overrun reporting, not a ring** — an unpublished step sets a sticky bit that is OR'd into the next line, because a silently decimated stream reads as a *slow control loop*, which is the wrong conclusion for someone about to change a gain. The flag set **splits the freeze reason three ways**: frozen-alone is anti-windup working, frozen-plus-slewing or frozen-plus-no-bridge is the loop being held off by something else — identical in `out`, opposite corrections. `telem on` stays the master switch, so `telem off` remains a complete stop. Host side: `node.py` parses both records from **one regex alternation** (the echo-reassembly logic needs ordered non-overlapping matches, which two iterators cannot promise), `bench.py` writes `velocity.csv` and gained **`run step`** — rise time, overshoot, settling to ±2%, steady-state error, saturation and freeze fractions, and **the integrator's resting value, which is how much the feedforward missed by**. Two profile defaults that are not copied from `sweep` and must not be: **`enc window 20`, not 100** (window 100 is a 10 Hz loop with 100 ms of lag that would dominate the response being measured) and **`cfg ramp_pmps 0`** (with drv's limiter armed, `drive_slewing()` is true almost continuously and the integrator is frozen for the whole run). ⚠️ **The finding worth carrying forward: arming a control loop invalidates every host-side stop sequence that addresses the layer below it.** `safe_stop()` sent `drv duty 0`, `drv coast`, `drv disable`, `telem off` — with the loop armed the first two are overwritten 20 ms later, and only the third actually stopped anything. `vel off` now goes first. This is task 21's watchdog-defeat problem one layer up, and it will recur again at CAN and at the rover supervisor. Firmware builds clean under `-Wall -Wextra` at **27.14% flash, 4.35% RAM**.
- **Sep 26** — **TASK 21: THE DUTY SLEW LIMITER IS IN THE FIRMWARE.** `drive.c` gained a per-mille slew limiter on the existing 1 kHz TIM6 tick — `drv ramp <o/oo per s>`, `drv ramp floor <o/oo>`, both backed by config keys (`ramp_pmps`, `ramp_floor`) and both **0 = off**, so nothing behaves differently until armed. Also `drv duty <n>p`, a per-mille command form that **unblocks the 1%-step stiction bracket**. ⚠️ **The placement reverses what this file said.** Task 21 had it in the control layer; reading the code showed that cannot work, because **`drive_set_duty()` calls `drive_kick()`** on purpose — a duty command is evidence of a live host — so any ramp module above `drive.c` would refresh the command watchdog a thousand times a second and a dead host would never be detected again. It is therefore the **command watchdog's own split**: mechanism in `drive.c`, policy above. Secondary reason: a limiter callers can route around is advisory; inside, the invariant is unconditional. **Coast and brake are deliberately NOT ramped** — coast is the safe stop and the watchdog's action, brake is an explicit act — so the standing "ramp duty down before braking" policy is written above as `drive_set_duty(0)` → `!drive_slewing()` → `drive_brake()`, which is what the new accessor is for; a tightened `drive_set_limit()` is immediate too, being protection rather than a command. `drive_duty()` now reports what the **bridge is running**, not the target, so a ramp shows up in telemetry as a ramp. The accumulator is in **milli-per-mille** because 5%/s is 0.05 per-mille per tick, and that scaling collapses to an identity — **the per-tick step in milli-per-mille IS the rate in per-mille per second** — so there is no division in the ISR and a ramp lands exactly on its target. Arithmetic verified offline against a transcription of the C before the board was touched (floor jump, no floor re-trigger at a reversal's zero crossing, cap-tightening instant and sticky, 50 CCR writes/s not 1000). Builds clean at 96 972 B flash. **Two things learned from reading `cfg` first:** the bench board has **never had `cfg save` run** (`slot 0/1024`, no overrides), so adding keys cost no calibration — but in general **adding a config key discards the stored record**, since `config.c` rejects a record whose key count differs, and `CONFIG_VERSION` is not what guards that. ✅ **Verified on the loaded rig the same day**, 1462 telemetry lines with no seq gaps: slew rate **50.00 o/oo/s** fitted over 340 samples, 120→290 in **3400 ms against 3400 predicted**, floor jump in one sample and the encoder turning 10 ms later, sync setting at duty 145 as always, **peak 581 mA against the host prototype's 572**. **The A/B against `drv ramp 0` is the evidence:** the un-ramped step read **1582 mA against a 1579 mA trip** — the clamp value, not the demand — and held the driver in regulation ~40 ms, so **the ramp cuts peak inrush 2.7×**; ⚠️ it latched **no fault and no ADC saturation**, meaning an un-ramped start relies on ITRIP silently and nothing in the telemetry would ever have shown it. **The watchdog test was repeated with the motor live and passed**: the 10 s deadline landed on the exact millisecond and duty went 290→0 in one sample (coast, not ramp). ⚠️ Both runs settled at **19.82 / 19.97 rpm against the fit's 20.76**, 4.5% low and mutually consistent — **the loaded plant line needs re-fitting, not explaining**.
- **Sep 25 (later)** — **LOADED-RIG PASS, AND τ MEASURED TWICE FROM TWO INDEPENDENT ROUTES.** Two integrity-clean sweeps on the treadmill belt (wheel + carriage **1047 g**, 12 V rail, **2% steps, 30 s dwell**, 1125 settled samples/point; the descending run entered on a **host-side 12→29% ramp at 5%/s**, standing in for the firmware slew limiter that still does not exist). **Steady state, 11–29%: pooled `rpm = 0.7993 d − 2.420`** against the free wheel's `0.8356 d − 1.573` — **the load costs 4.3% of slope and 0.85 rpm of intercept**, so the Sep 25 mounting rule holds from the other side. Repeatability floor **±0.174 rpm**. **Breakaway and dropout both land in 9–11% duty**, and that is the *same terminal voltage* as Sep 15's 12–14% on the 9.35 V rail (**1.203 V vs 1.216 V, 1.0% apart**) — **breakaway is a voltage threshold, and breakaway duty is not portable across rails**; the Stribeck cliff moved below 11% with it. Current finally separates the two friction terms: **free wheel flat at 232.5 ± 18.5 mA (slope −0.85 mA/%, Coulomb), loaded rising 266→313 mA at +4.4 mA/% (viscous)**. **Above ~31% the wheel bounces on the belt** — slope lifts from 0.7924 to **0.9219 rpm/% (+16.3%)**, which is the rig and not the plant, and is why 30% is the characterisation ceiling. The band's curvature is **real, not thermal**: asc/desc residuals correlate **+0.718** where drift would anti-correlate. **The model stays linear anyway** — a quadratic buys 0.063 rpm of rms against a 0.174 rpm floor — and the **25% gain droop** (0.899 → 0.700 rpm/%) is carried as a PID design constraint instead. **⚠️ THE BIG CORRECTION: the τ ≈ 0.65–0.70 s reported earlier today is wrong.** The plant is **two-pole** — **τ_fast 0.219 ± 0.007 s (84%)** plus **τ_slow 2.75 ± 0.05 s (16%, belt and contact settling, which is why the 30 s dwell was needed)**. A one-pole fit returns a **window-dependent artifact**: 0.290 s at 1 s, 0.496 s at 3 s, **0.724 s at 10 s** — never settling, and the last is essentially the number first quoted. **Verified from two independent measurements: an ensemble of 42 stacked 2% steps gives 0.219 s, and the ramp-tracking lag on the entry ramp gives 0.207 ± 0.007 s — 5.7% apart**, from different excitation, different data and a different estimator. Two supporting errors were also found and fixed (the ramp rate was 4.814 %/s, not 3.75, and the end-of-ramp speed had been read off `mrpm` instead of `count`). ⚠️ **The rig loads the wheel but does not carry the rover's inertia** — 1047 g is normal force, and the rover is ~3 kg/wheel, so the rig is **2.87× light**: friction transfers, τ does not, and τ on the rover will be longer. Two figures and their scripts added under `docs/environment/figures/`.
- **Sep 25** — **BENCH HOST TOOLING, end of the hand-transcription era.** Firmware gained **`telem`** (`T,seq,ms,duty,count,milli_rpm,mA,flags` at 1–100 Hz, integer fields only, emitted from the main loop) and **`drv timeout`** (a command watchdog that was simply absent — it **coasts** on expiry, and is checked *before* the fault path's early return). `tools/bench/` (`node.py` + `bench.py`) drives profiles, logs raw-before-parsed, and rewrites `status.json` once a second so a run can be left alone and checked on by reading one small file. Proven at the wire with the motor stopped: **31 lines in 3.0 s, zero seq gaps, board-stamped intervals 99–100 ms against a nominal 100.** One real bug found and fixed: the prompt carries no newline, so a telemetry line landing behind it merged in the buffer and the prompt was never seen again. **The tool then found a second bug in itself** — the pre-flight fault gate would have refused every run, because nFAULT reads low the whole time nSLEEP is low, so a reset board always reports a latched fault; it now wakes the driver and clears the latch before looking. **First motor run: no 12 V rail.** Board accepted 20% duty and was genuinely switching (`flags=3`), but zero counts and 11 mA across 150 samples — diagnosed from the recorded run in one look, no re-run. After the rail was repaired, **a third bug surfaced only on real hardware**: the console echoes each typed character as its own one-byte write while a telemetry line is one atomic write, so a `T,` record lands *inside* a command echo — the parser now extracts records anywhere in a line and reassembles the echo around them, and an echo mismatch is counted rather than fatal. **Then four clean sweeps re-took the plant** (0 seq gaps, 0 echo mismatches, 0 `tx_dropped` on all four), with the **wheel clamped to the table** instead of hand-held: CW asc `0.8327 d − 1.524`, CW desc `0.8187 d − 0.752`, CCW `0.8618 d − 1.164`, CW full range `0.8186 d − 0.833`, every R² ≥ 0.9999. **The validation gate passed at −0.18%** against the Sep 23 hand-typed table over the same 20–100% points, and **CCW closed task 17's last free-wheel item at +3.49%**, matching Aug 26's +3.5% from a different method. Two findings worth keeping: **mounting moves the intercept and leaves the slope alone**, and the asc/desc gap is **the motor warming, not hysteresis** — it tracks elapsed time, not direction. Loaded rig and the deliberate `kill -9` watchdog test still owed.
- **Sep 20 (later)** — **W4 CLOSED. Task 20 implemented and built clean.** `k = 3` now applied in the mA↔VREF conversion pair (as a config key, `cfg vref_div`, so a different part is a console command and not a rebuild); the sampler moved from the window MIDPOINT to its settled tail, `trigger = end − (aperture + guard)`, which cost the minimum synchronised duty 4.3% → **14.5%** and bought back the ~13% the midpoint read low; `drv current` now spreads its samples over 4 ticks for the same 64 periods. **The two measured constants were finally applied** — VDDA 3300 → **3325**, R_IPROPI 1474 → **1465** — held back since Sep 12 so nothing moved underneath the plateau sweep, closing the last open W4 item. Printed trip range is now **~101–1580 mA** (was 1558 on the nominal constants); full scale 5.044 A, one LSB 1.231 mA. Two latent bugs caught on the way: the mA→mV multiply wrapped uint32 at `drv trip 3000` and would have reported ~811 mA as honoured, and `iscan`'s window markers were derived from the old midpoint convention. **`CONFIG_VERSION` 1 → 2, so the stored calibration record is discarded on this boot.** Bench re-take of the calibration point still owed.
- **Sep 21** — **TASK 20 BENCH-VERIFIED, W4's last owed item cleared.** At 12.0 V, 20% duty, shaft stalled: **`drv current` = 1268 mA against `D × Vm / R_motor` = 1263 mA, +0.4%**, with `sync: 64 samples over 4 ticks in 4100..4348` confirming the spread, the settle floor and the end-relative placement in one line. The same two traces re-reproduced the bug that was fixed — the old midpoint tick 4050 read **14.3% and 16.1% below** the settled tail of its own run. The `cfg` check came first and passed: the forced wipe booted the board **already calibrated** (`config v2, 9 keys, slot 0/1024`, vdda_mv 3325, r_ipropi 1465, vref_div 3, no overrides), and `drv trip 1580` read back 1579 mA with `range 101..1580 mA` — the predicted ceiling to the digit. Duty-0 checks confirmed the `ticks == 0 → CCR 4500` case and the gate's refusal. **New lead:** the decay-phase tick reads a reproducible **0.670** of the drive tail across runs, where physics allows only 4.4% of droop — if that factor is real it makes current readable below the 14.5% synchronised floor, which is the standing W5 constraint. Console fixed afterwards, not during: `1 code = 1 ADC LSB` → `1 DAC code = 1/3 ADC LSB (VREF/3)`.
- **Sep 20** — **PLATEAU SWEEP DONE: `k = 3`.** The DRV8874 compares IPROPI against **VREF/3**, so **every `drv trip` is 3× too high** — the real range is ~100–1558 mA and the 3000 mA boot default is really 1000 mA. `k=1`/`k=2` refuted; confirmed predictively by a plateau that ignored a 7.6% shift in demand. The current-sense chain is **calibrated against physics for the first time** (1290 mA measured vs 1263 predicted, 2%). Three sampler bugs found: IPROPI settles in **5.6 µs not 1.6 µs**, `ISENSE_SYNC_MIN_TICKS` 192 is far too low (nothing below ~13% duty is valid), and `place_trigger()` samples the contaminated half of the window. The **Sep 12 "reading is SUPPLY current" conclusion is retracted** — it was a pre-Sep-16 sampling artifact, and the stated IMODE cause was wrong too.
- **Sep 19 (later)** — **Task 19 CLEARED.** Ground return rebuilt with three thick conductors; `drv pin`-free build flashed; **PMODE confirmed latched in PWM mode** by scoping both motor outputs — at 13% duty one carries PWM and **the other sits at GND**, which only low-side slow decay produces (independent half-bridge would park it at the rail). Roles swap cleanly at −13%. Motor rail raised to **12.0 V, DMM at the DRV8874 VM pin**. Current regulation is live for the first time, so the 3000 mA boot trip is now real — and every plant figure on record belongs to the old 9.35 V rail.
- **Sep 19** — Replacement MCU board verified **bare** on both pad checks (`drv pin` → both pads `mode 2 af 2`, `IDR 0`; `drv pin pd` → PB7 `IDR 0`, where the dead board read 1), so PB6/PB7 stay put and TIM3/PC6-PC7 is off the table. PMODE strapped with 10 kΩ to 3V3, **not yet confirmed latched**. VREF confirmed bare and a 100 kΩ pull-down rejected — it would be indistinguishable from the internal divider the plateau sweep is meant to measure. **Task 19 still blocks: ground return next.**
- **Sep 18** — Context file restructured: DRV8833 history and the completed NEXT TASKS moved to the log file (2338 → 1961 lines). MCU board replaced; PB7 not yet re-checked — **task 19 is still the blocker, and the bare-board `drv pin` check is step one.**
- **Sep 16** — PMODE was never strapped. Floating is Hi-Z, which latched the driver into independent half-bridge for five days (invisible on rpm, but it disabled current regulation and made IPROPI blind to the decay phase); on the last power-up it latched PH/EN instead, turning a 13% duty command into ~74% of the rail, and that return current destroyed PB7. MCU board is being replaced — **task 19 is a blocker on all bench work.**
- **Sep 15** — PWM-synchronised current sampling (`drv iscan`) confirmed the TIM4_CH4 trigger placement is correct, but the IPROPI waveform inside the drive window showed unexplained structure — later traced (Sep 16) to sampling during the wrong, high-side decay phase.
- **Sep 15 — retraction** — Withdrew the "free-running sampler aliases" diagnosis; it failed a repeatability check (189/190 mA on repeat), so the low-current scatter has a different, still-open cause.
- **Sep 15 — free vs wheeled motor** — Clarified which bench figures carry over to the loaded wheel (motor-electrical: R, L, Kt, Ke, counts/rev) versus which don't (system-mechanical: breakaway, friction, no-load current).
- **Sep 15 — loaded-rig characterisation** — Loaded wheel: breakaway 12–14% duty, dropout ~10.5%, minimum sustainable speed ≈4.9 rpm (Stribeck cliff) — constrains the demo's slowest manoeuvre. Current readings from the same sweep flagged untrustworthy.
- **Sep 14 (later)** — Module identity (`dipsw.c`, 3-bit DIP switch) implemented and verified on hardware across all eight codes; heartbeat and CAN ID now track module ID, closing W7's firmware dependency.
- **Sep 14** — `config` module verified on hardware (8/8 bench checks); two consistency-check bugs found and fixed; confirmed a reflash does not erase the calibration sector.
- **Sep 13** — `config` module built: eight tunables moved from compiled `#define`s into FLASH as a CRC'd, append-only log. Builds clean, not yet bench-verified.
- **Sep 12 (4)** — Decided to raise the motor rail from 9.5 V to 12 V (the motor is a 12 V unit; 9.5 V was only the old DRV8833's ceiling) and identified the need for the `config` module.
- **Sep 12 (3)** — Fixed an invalid `.ioc` (wrong CubeMX signal names) that had silently dropped TIM2/TIM4, and a regeneration that silently deleted the USER CODE blocks starting those timers. Decoded IPROPI as reporting **supply** current, not motor current.
- **Sep 12 (2)** — Modified the DRV8874 bench carrier to make VREF software-settable via a DAC, decoupling the current ceiling (4.975 A) from the regulation trip point; added `drv trip` command. Firmware pending a CubeMX regen.
- **Sep 12** — Measured the DRV8874 carrier's three straps (R_IPROPI, IMODE, nSLEEP→VREF); added the `isense` current-sensing module; found the carrier's fixed VREF forecloses software-settable current limiting.
- **Sep 11** — Three-week gap explained: all seven motor encoders re-terminated with crimped joints, the DRV8874 arrived early, a loaded wheel test rig was built, and node PCB design was delegated to a student.
- **Aug 26 (late)** — Measured motor terminal voltage (9.45 V→9.35 V) and Ke≈0.138 V/rpm; decided stop policy: coast by default, brake only below ~40 rpm.
- **Aug 26 (evening)** — First powered motion (free shaft): plant is linear (rpm = 0.672×duty% − 1.8); sign convention and drive scheme (slow decay) settled.
- **Aug 26 (later)** — PWM scope-verified with motor disconnected: all four drive quadrants correct at 20.000 kHz; two apparent anomalies were instrument error, not firmware.
- **Aug 26** — W4 acceptance criterion met: encoder firmware (TIM2 + TIM6) verified on the bench at 8394.9 counts/rev against the predicted 8403.2 (0.1% low).
- **Aug 25–26** — Encoder dead on two motors, root-caused to a broken VCC conductor in student-soldered cable extensions.
- **Aug 25 (later)** — Bench-measured two motors: R≈1.90 Ω, L≈1.70 mH, matched to <2% — one PID gain set should fit all six wheels; motor rail set at 9.5 V.
- **Aug 25 (earlier)** — Got the motor datasheet: encoder is 64 CPR (8403.2 counts/rev at output); driver changed from DRV8833 to DRV8874 for current sensing/limiting headroom.
- **Aug 24–25** — W4 opened: node pin map settled (encoder moved to TIM2); DRV8833 carrier characterised — no current feedback or hardware current limit on that carrier.
- **Aug 13** — W3 acceptance criterion met: STM32 commands the SERVO42C to a target angle with 1/10-microstep repeatability; three UART/HAL traps found.
- **Aug 11** — Bus-load ramp to saturation: no FIFO overrun at any rate; polled-vs-interrupt CAN RX left open.
- **Aug 10** — W2 complete: STM32F446RE heartbeat crossing a real 250 kbps CAN bus to Orion, zero error counters; floating CAN_RX pin identified as the root cause of earlier faults.
- **Aug 9** — W2 firmware written and building: DMA console, bxCAN driver, serial command interpreter.
- **Aug 9 (earlier)** — W2 toolchain established: CubeMX + CMake + CubeCLT + VS Code on daedalus; module identity settled as a 3-bit DIP switch.
- **Aug 6** — CAN bus W1 complete: CANable flashed to candleLight, two-node bus validated at 250 kbps, pinmux/can0 made persistent; recovered MKS SERVO42C UART protocol docs.


### Closed tasks moved out of NEXT TASKS

### Task 6b — W4 — closed Sep 20

6b. **W4 — drive motor + encoder closed loop: ✅ CLOSED Sep 20, 2026**
    (opened Aug 24). Acceptance criterion: encoder counts read correctly and
    match physical rotation — **MET Aug 26**, 8394.9 counts/rev over ten hand
    turns, 0.1% from predicted, with the sign convention recorded on the bench.

    **Why it stayed open for three weeks after its criterion was met, and why
    that was right:** the criterion was about the encoder, but the week's real
    deliverable was a drive chain you could trust the numbers from. Everything
    that kept it open was current-sense work — the DRV8874 swap, the carrier
    modification, PMODE, the plateau sweep — and closing on the letter of the
    criterion would have handed W5 a plant model measured through an
    uncalibrated sensor on a driver that was not in the commanded mode. The
    cost of the delay was three weeks; the cost of the alternative was tuning
    gains against fiction.

    ✅ **Done — full detail in the LOG file:** pin allocation and the `.ioc`
    root cause (Sep 12); encoder read, `int32_t` delta accumulate and the `enc`
    commands including `enc probe`; TIM6 1 kHz tick; PWM helpers and the
    **slow-decay (drive-brake) choice, CLOSED Aug 26** on a measured ~2.6% vs
    >20% deadband; the `drv` console commands; the **disconnected-motor PWM
    scope pass (Aug 26)** — its checklist is kept in the log, and is the right
    list to re-run after any timer change; first powered motion Aug 26 and first
    DRV8874 motion Sep 12; motor terminals measured at 9.35 V (Aug 26); all
    seven motors re-harnessed to NASA-STD-8739.4A (Sep 11); DIP-switch module ID
    on PB14/PB15 (Sep 14), so W7 no longer waits on firmware; IPROPI decoded as
    **supply** current (Sep 12); `config` verified (Sep 14); nFAULT pull-up
    confirmed (Sep 14) and **latched in the 1 kHz tick (Sep 16)**, so a
    transient fault now leaves a mark.

    **The four items that kept it open are all resolved:**
    - ✅ **Real drive current measured (Sep 20)** — the input HW4's PDB branch
      sizing was waiting on. Done as a firmware reading through IPROPI rather
      than the multimeter or bench-supply fallbacks that were held in reserve:
      **1290 mA at 20% duty, stalled, on the 12 V rail**, against 1263 mA
      predicted from `D × Vm / R_motor`. 2%, inside the ±8% rotor-position
      noise floor. The loaded-rig figure at real weight is the one HW4 should
      size against and is now a measurement, not an estimate.
    - ✅ **Plateau sweep run (Sep 20)** — five points, 300–1500 mA commanded,
      slope 1/3 to better than 1%. **`k = 3`.** The sweep was designed to be
      immune to both pending constant corrections, because measured current and
      commanded trip pass through the same R_IPROPI and the same VDDA and the
      ratio cancels; that immunity is what let the constants be held back until
      it was finished.
    - ✅ **The two measured constants applied (Sep 20)**, the hold released now
      that nothing is mid-experiment: `ISENSE_VDDA_MV_DEFAULT` 3300 → **3325**,
      `ISENSE_R_IPROPI_OHM_DEFAULT` 1474 → **1465**. Every current logged before
      today reads ~1.4% low. Derived figures move with them — full scale 4.975 →
      **5.044 A**, one LSB 1.215 → **1.231 mA**, printed trip range ~100–1558 →
      **~101–1580 mA**. Both are **per-board** figures: a second carrier gets
      metered and `cfg`-set, not handed these.
    - ✅ **Task 20 firmware landed (Sep 20)** — the four bugs the sweep exposed,
      plus multi-tick averaging. See task 20 below for what remains to *verify*;
      the implementation itself is done and builds clean.
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

    **Carried out of W4, not dropped** — these were never W4 acceptance items
    and are tracked where they belong: the 12 V plant re-measurement and the
    motor-terminal metering in task 17, the PID gain keys in task 18, nFAULT on
    a real fault in task 19, and task 20's bench verification.


### Task 19 — MCU board replacement, PMODE, ground return — cleared Sep 20

19. **✅ CLEARED Sep 20 — MCU board replacement, PMODE strap, ground return
    (opened Sep 16, 2026).** PB7 on the old board was destroyed and the
    conditions that destroyed it were still wired up. Board replaced Sep 18,
    both pad checks passed Sep 19, PMODE confirmed latched and the ground return
    rebuilt the same day, and the **current-regulation path proven by the Sep 20
    plateau sweep**. **One item is left open below: nFAULT assertion on a real
    fault.** The bench is unblocked.

    - ✅ **Check the new board bare — DONE Sep 19, PASSED.** Flashed with the
      DRV8874 wiring to PB6/PB7 disconnected: bare `drv pin` gave both pads
      `mode 2 af 2 pupd 0 od 0  ODR 0 IDR 0` with TIM4 correct
      (`CR1 0x0081`, `CCER 0x1011`, `CCMR1 0x6868`, `CCR1/2 0`, `ARR 4499`), and
      **`drv pin pd` gave PB7 `IDR 0`** where the dead board read 1. The
      pull-down test is the one that counts — a push-pull low can be faked by a
      damaged pad, a weak pull-down against pad leakage cannot.
      **PB6/PB7 stay where they are; TIM3/PC6-PC7 is off the table.**
    - ✅ **PMODE pull-up fitted Sep 19: 10 kΩ from PMODE (pin 16) to 3V3.**
      Not 100 kΩ — against the internal 156 kΩ/44 kΩ divider that reaches only
      ≈1.66 V, 160 mV over the 1.5 V `V_TIH` minimum. **This is a per-board
      schematic item for all six nodes and for HW1, not a bench workaround.**
      Fitted is not latched — see the confirm step below.
    - ✅ **`drv pin`-free build flashed (Sep 19)** — the reset restored PB7 to
      AF2/TIM4_CH2, so the PB7 restore came free with it.
    - ✅ **Ground return rebuilt (Sep 19): three thicker conductors** from
      breadboard PGND to the MCU carrier board, replacing the single DuPont that
      carried the Sep 16 fault current.
    - ✅ **PMODE CONFIRMED LATCHED IN PWM MODE (Sep 19).** Scoped **both motor
      outputs** at `drv duty 13`: one carries PWM to the rail, **the other sits
      at GND for the whole period**, and the roles swap cleanly at `-13`. That
      quiet channel is the whole proof — independent half-bridge parks it at the
      **rail**, since each output follows its own input; only PWM mode's
      `IN1=1, IN2=1 → OUT1 L, OUT2 L` low-side decay pulls it to ground.
      **Scoping IN1/IN2 cannot settle this** (the earlier wording here was
      wrong): PMODE changes how the driver interprets its inputs, not what the
      MCU emits, so the input waveforms are identical in all three modes.
    - ✅ **Current-regulation path PROVEN (Sep 20).** The plateau sweep drove
      the DRV8874 into regulation at four different trips and it limited
      cleanly every time, gently at 29–39% below demand and hard at ~94% below.
      The bridge, the comparator, the VREF DAC path and the IPROPI mirror all
      work. **What remains of "did the driver survive" is only nFAULT assertion
      on a real fault** — cheapest check is UVLO: drop VM below ~4.5 V with
      nSLEEP high and watch the pin. Everything else about this driver is now
      positively demonstrated, not merely un-disproven.
    - ✅ **Current-regulation results re-taken (Sep 20).** All `drv trip` / PA4
      VREF work from Sep 11–16 was inert under independent half-bridge and has
      been superseded by the plateau sweep, which also found that **every trip
      value ever commanded was 3× too high** — see the IPROPI section and
      task 20.
    - ✅ **`drv iscan` re-run in PWM mode — the shape DID change (Sep 20).**
      Decay-phase samples read **107–339 raw** where Sep 15 read exact zeros,
      and an `iscan` at `duty 0` with the driver awake read a flat **3–5** —
      proving the decay-phase current is real recirculation and not a mirror
      pedestal. IPROPI sees the low-side FETs during brake, which only low-side
      decay produces. **That is PWM mode confirmed a third time, independently
      of the scope.** The scope-on-PA2 plan is not needed.
    - ✅ **Temporary `drv pin` command removed from `console.c` (Sep 19)** —
      `pin_report()`, the `pin` branch and its help line, 3659 bytes. Builds
      clean at RAM 4.11% / flash 22.84%. `console.c` now holds no direct
      register or HAL-GPIO access at all; everything goes through the driver
      modules, which is how the rest of the file already worked. Recoverable
      from git history if a future board ever needs the same pad check.


## 2026-09-26 (evening) — A/B/A staircase: direction and drift separated (task 21)

Conditions: loaded rig 1047 g, shipped gains, `enc window 20`, 50 Hz telemetry,
30% duty ceiling. Fresh battery (16.32 V at swap) through the regulator; VM
metered at **12.02 V** (motor off) before leg 1. Each leg: `bench.py run stair
--lo 10 --hi 20 --stair-step 0.5 --hold 60 --hold-settle 10 --abort-ma 1200`,
21 points × 60 s, legs started back to back (gaps ~30 s and ~75 s).

- A first leg 1 at 17:29 died at +15.5 rpm when the old battery ran out (the
  wheel stopped, loop saturated at 100% into a dead rail). The runner's safe
  stop ran cleanly. The run directory was **deleted at the user's request**:
  a run on a failing supply has no meaning, not even its first half.
- Leg 1 A (fwd) 18:10, leg 2 B (rev) 18:31, leg 3 A (fwd) 18:52.
- Tracking: max |err| 0.008 / 0.016 / 0.011 rpm; 0% saturation in all three;
  peak output 282 / 272 / 282 o/oo.
- Leg 3 flagged suspect: one T and one V record lost at 152 s (+11.0 rpm
  hold); `T,7542` truncated mid-line in `console.log` with `V,7539` missing
  behind it. Board `tx_dropped 0`, `veloc_steps_missed 0` → ~50 bytes lost on
  the host side only; the loop ran every step. 1 of ~2500 records in that
  hold; leg used.

Mean of (|out| − |ff|) across the 21 steps, o/oo:

| Leg | mean |out|−|ff| | mean i | mean mA |
|---|---|---|---|
| A1 fwd | +1.62 | +1.20 | 286.0 |
| B rev | −5.81 | +5.28 (sign-flipped: pushes toward zero) | 291.8 |
| A2 fwd | −1.43 | −1.06 | 285.8 |

- **Drift is real:** the forward legs moved −3.05 o/oo in 42 min under the same
  conditions. Linear interpolation puts forward at ≈ +0.1 o/oo at B's midpoint.
- **Direction is real too:** reverse needs ≈ **5.9 o/oo (0.59% duty) less**
  than forward at the same speed, after removing drift. The drift is about half
  the size of the direction effect, so the morning's 67-min-apart comparison
  was partly confounded, but its sign was right.
- Assumes linear drift over 63 min. Cause of the drift not identified (not
  claimed: warm-up is a guess).
- Current: forward legs agree to 0.2 mA mean. Reverse draws +5.8 mA mean
  (~2%), concentrated above 14.5 rpm (+10–19 mA), against the morning's
  "+8.1%". The morning's battery state was not recorded.
- Integrator carries the whole difference in every leg; tracking meets the
  staircase criterion in both directions without any ff change.

## 2026-09-26 (evening) — true step response, `--slew 0` (task 21)

Same battery and VM (12.02 V) as the A/B/A, shipped gains, `enc window 20`,
`--dwell 10`. Three runs, all `outcome=ok`, 0 gaps, 0 missed steps, 0% saturation:

| Run | Step | Rise 10→90% | Peak current | Verdict |
|---|---|---|---|---|
| `19-30-35_step` | 0 → +10 rpm from rest | 0.22 s | 904 mA | peak inside ripple |
| `19-31-06_step` | +10 → +15 → +10 | 0.08 s up / 0.22 s down | 984 mA | up inside ripple; down flagged, is ripple |
| `19-32-19_step` | −10 → −15 → −10 | 0.12 s up / 0.26 s down | 885 mA | up inside ripple; down flagged, is ripple |

- Peak current ≤ 984 mA against the 1580 mA trip; no step needs the setpoint
  ramp to stay off ITRIP at these sizes.
- **Both down-steps were flagged `overshoot_above_ripple=True` (64% / 50% of the
  5 rpm step), and both are the 12-per-rev mechanical dip, not the loop.** Fwd:
  speed came 15.7 → 10 rpm in 0.23 s without crossing; the flagged minimum
  (6.78 rpm at 2.51 s) sits in a dip train at exactly 0.50 s spacing
  (= 12 events/rev at 10 rpm), and the settled 3–10 s hold dips to 7.14 rpm,
  one encoder count (0.357 rpm) away. Rev: dip train at 0.27, 0.79, 1.33,
  1.81, 2.31, 2.81 s; flagged minimum 7.50 vs settled 7.85 rpm, again one count.
- **Metric flaw:** `overshoot_above_ripple` compares the peak with 2 × tail sd.
  The ripple is impulsive (periodic dips), so its own extremes exceed 2 sd and
  the test fires on ripple. It should compare against the settled tail's own
  extreme excursion (plus one count). Not yet changed.
- `settle_s` is n/a on most segments: the ±2% band (0.2 rpm at 10 rpm) is
  narrower than the ±1 rpm mechanical ripple, so no settling instant exists.
  Settling to ±2% is not a usable criterion on this rig until the ripple is gone.
- Verdict against the acceptance criterion: no sustained oscillation, no
  overshoot resolvable above the mechanical ripple, in either direction, from
  rest or from a turning wheel. Steps run: up to 10 rpm in size, up to 15 rpm
  setpoint; larger steps (e.g. 0 → 20) were not run.

## 2026-09-26 (evening) — ripple test fixed in `bench.py`; CORRECTION to the step verdict above

`overshoot_above_ripple` now requires the peak to beat the settled tail's own
worst excursion in the overshoot direction (`tail_excursion_rpm`, new column)
plus one encoder count (0.357 rpm at window 20), instead of 2 × tail sd.
Extreme against extreme: the tail (last quarter of a 10 s dwell, ~5 dips at
10 rpm) and the 3 s overshoot window (~6 dips) sample a similar number of ripple
events. Re-reduced from the saved `velocity.csv` of the three step runs:

| Segment | overshoot | tail excursion | old flag | new flag |
|---|---|---|---|---|
| 0 → +10 | 1.424 | 1.424 | ripple | ripple |
| +10 → +15 | 1.422 | 1.422 | ripple | ripple |
| +15 → +10 | 3.217 | 2.860 | above | **ripple** (within 1 count) |
| −10 → −15 | 1.779 | 1.779 | ripple | ripple |
| −15 → −10 | 2.503 | 1.789 | above | **above** (by 2 counts, 0.71 rpm) |

**Correction:** the entry above says both down-steps were ripple. The reverse
one is not resolved as ripple: it dips 0.71 rpm (2 counts) past the settled
tail's worst dip, and the trace sits 0.4–0.7 rpm below setpoint for ~0.3 s
after the dip. That comparison was made against the 3–10 s hold, which holds a
deeper dip than the last quarter the metric uses. So: a small reverse
down-step undershoot (≤ ~0.7 rpm beyond ripple, gone in ~0.35 s) — bounded,
still inside the acceptance criterion, not tuned against yet. Forward
down-step is ripple.

Re-reduction segments on host time; overshoot figures reproduce exactly, one
rise time differs (0.22 vs 0.26 s, reverse down) from the profile's own cut.
`figures/plot_velocity_step_anchor.py` panel C still draws ±2 sd bands for the
morning runs; not regenerated.

## 2026-09-26 (evening) — reverse down-step repeated ×3: the undershoot does not recur

Same conditions and command as `19-32-19_step` (−10 → −15 → −10 rpm,
`--slew 0`, `--pre 10 --dwell 10`), run back to back with the fixed ripple test.
All `ok`, 0 gaps, 0 missed steps, 0% saturation, peak 706–968 mA.

| Run | down-step overshoot | tail excursion | above ripple |
|---|---|---|---|
| `20-32-46_step` | 2.146 | 2.146 | no |
| `20-33-20_step` | 2.503 | 2.146 | no (1 count) |
| `20-33-55_step` | 2.503 | 2.503 | no |

- **0 of 3 repeats flagged.** The deepest down-step dip (2.503 rpm) is the same
  value as the original run's, and in `20-33-55` the settled tail dips exactly
  that deep. What made `19-32-19` pass the threshold was a shallow tail (1.789),
  not a deep step.
- The settled tail's worst excursion varies 1.79 → 2.50 rpm (2 counts) run to
  run at the same setpoint, so a single-run verdict at a 1–2 count margin is
  marginal. Repeat before calling a small flag real.
- Rise on these 5 rpm steps spans 0.04–0.26 s across the four reverse runs:
  with ±1 rpm ripple on a 5 rpm step, the 10/90% crossings depend on ripple
  phase. Not a stable number at this step size.
- **Revised verdict:** no overshoot resolvable above the mechanical ripple in
  either direction, from rest or from a turning wheel. The correction entry
  above is itself superseded on this point.

## 2026-09-26 (evening) — decision: no direction-dependent `ff_b` for now (task 21)

The feedforward is `ff = ff_a × rpm + sign(rpm) × ff_b` (`ff_a` 12.51 o/oo/rpm,
`ff_b` 30 o/oo), symmetric by construction. The A/B/A run showed reverse needs
~5.9 o/oo (0.59% duty) less output than forward at the same speed; a
direction-dependent offset would be ~24 o/oo in reverse against 30 forward.

**Decision (agreed with the user): do not add it now.** The integrator carries
the offset (+5 to +7 o/oo in reverse) with tracking inside ±0.016 rpm and no
step transient above the ripple. Consistent with task 21's earlier note that
the brush-timing asymmetry is "absorbed by integral action".

Why not now:
- The estimate is soft: drift (−3.05 o/oo in 42 min) is half the size of the
  effect, from a single A/B/A on one motor, on the rig.
- The rover differs: ~2.87× the inertia, real load, five other motors with
  their own brush timing — a rig value may not carry over.
- **It implies another variable in the `cfg` menu** (e.g. `vel_ff_b_rev`), and
  adding a `cfg` key discards the stored flash record (read `cfg` before
  flashing such a build). It is also one more number to calibrate per wheel.

**Must be re-checked on the rover, on at least two wheels:** an A/B/A
forward/reverse comparison per wheel. Add the reverse offset only if the
asymmetry is consistent across wheels and large, or if direction reversals show
visible transients.

## 2026-09-26 — session summary (evening, task 21)

- First A/B/A leg 1 aborted at +15.5 rpm when the battery died; stopped cleanly,
  run deleted (user rule: runs on a failing supply are erased). Battery swapped
  (16.32 V through the regulator, VM 12.02 V metered).
- A/B/A staircase done: direction −5.9 o/oo and drift −3.05 o/oo in 42 min,
  both real. Max error ≤0.016 rpm, 0% saturation.
- True steps (`--slew 0`) 0→10 and ±10→±15→±10: peak ≤984 mA, no overshoot
  above the 12/rev ripple in either direction (reverse flag not reproduced ×3).
- `bench.py`: `overshoot_above_ripple` now tests against the tail's own worst
  excursion + 1 count; new column `tail_excursion_rpm`.
- Decision: no direction-dependent `ff_b` for now; re-check on the rover on ≥2
  wheels; adding it costs a new `cfg` key.
- Next: current sensing below 14.5% duty.
Details: the dated entries above.

## 2026-09-26 (night) — decay-phase current reading validated below the 14.5% floor (task 21)

**Result: IPROPI is readable in the slow-decay (brake) phase. Reading =
0.690 × motor current (±1.5%), valid from 6% duty upward to ±4%. The open
item "current sensing below 14.5% duty" is decided: decay-phase reading.
Free-running `Isup` and the slower carrier are not needed.**

Setup: 12.0 V rail, shaft stalled (same hold as Sep 21), slow decay, trip
1579 mA (VREF 3123 mV, DAC 3847), `drv iscan 64` full period, 125-tick step,
from a scratch script over `node.py` (transcripts saved first, in
`tools/bench/runs/iscan-20260926-*`, local-only). `nFAULT ASSERTED` in the
pre-run status is the known nSLEEP-low behaviour; no fault during any scan.

Correction on the way in: the `isense.h` note that IPROPI "reads exactly 0
outside the drive phase" is from Sep 15, before the PMODE strap (Sep 19). With
PMODE in PWM mode, decay is low-side and IPROPI (low-side mirror) sees it;
already shown Sep 20 (13% scan 107–339 raw in decay, 0%-duty control flat 3–5).
Comment in `isense.h` annotated.

Step 1 — 20% full-period scan, shape of the decay phase:
- Transient after the falling edge: 1774, 854, 665, 792 … settled by ~1000
  ticks (11 µs) — twice the 500-tick settle after the rising edge.
- Plateau ticks 1000→3500: 746 → 709, smooth, monotonic, −5.0% (L/R 0.9 ms
  predicts −3.0%). Drive tail (4000–4375) mean 1016.
- Ratio decay(t3500)/drive tail = 0.698 (0.689 with the 4125–4375 tail).

Step 2 — 15%: decay 592→554 (1000→3500), ratio 0.669 against a single,
possibly unsettled drive point (4375 = 828; last valid trigger at 15% is
~4348). Nominal fail of ±3%, but the reference is too weak to mean anything.
Change of method: calibrate the factor at 20% (solid tail), then test that the
decay reading stays ∝ duty (at stall, I = D·Vm/R exactly), each low duty
bracketed by 20% runs; triple void if the two 20% refs differ > 5%.

Step 3 — 20/10/20: 752 / 376 / 743 at t3500 → 10% expected 374, +0.6%. Ratios
0.697, 0.695.
- Tick 4000 at 10% (50 ticks before the 4050 drive edge) dipped to 346 vs a 375
  plateau: the 112-tick ADC aperture straddles the edge. Sep 21's 0.670 was
  taken at t3550, 50 ticks before the 20% edge — the same contamination. The
  correct factor is ~0.69, sampled ≥ ~150 ticks before the edge.

Step 4 — 20/5/20: first triple void (20% ref plateau jumped 590→720 mid-scan,
rotor shifted; refs 587/710). Repeat: 735 / 170 / 762 → 5% expected 187,
−9.1%. The void triple's 5% read 147.

Series — 20,5,20,6,20,7,20,8,20,9,20,10,20,11,20,12,20 (t3500, err vs
proportional, ref agreement):
  5% 120 −38.3% (1.8) · 6% 236 +2.3% (0.5) · 7% 255 −3.6% (2.9) ·
  8% 263 −15.0% (7.4, VOID) · 9% 354 +0.2% (4.3) · 10% 393 +0.8% (3.1) ·
  11% 424 +3.4% (12.5, VOID) · 12% 430 −4.3% (13.4, VOID)
Repeat 20,8,20,11,20,12,20:
  8% 315 +4.9% (6.0, VOID) · 11% 442 +3.6% (0.8) · 12% 461 −0.8% (1.2)

Conclusions:
- Valid points 6–12% (6, 7, 9, 10, 10, 11, 12): −3.6% … +3.6% → **±4%**.
  8% has two void readings (−15.0, +4.9); accepted as covered by 7 and 9.
- 5% is outside the valid range (−9, −17, −38%). Accepted: the rig breaks away
  at 10–12%, so 5% is not an operating point.
- Factor decay/drive at 20%: 17 reference runs, 0.678–0.699, mean **0.690**
  (±1.5%), while the absolute level wandered 699–802 (±7%, rotor position in
  the hold). The factor does not depend on where the rotor sits.
- Void triples came from the hold (20% refs jumping ~13%), not the sensor.
- Mechanism of the 0.69 not identified (physics predicts ~4% loss over the
  window, not 31%); it is reproducible, so it is treated as a calibration.

Design rule for a decay-phase sample: end-relative, trigger ≤ drive edge − ~150
ticks (aperture 112 + guard 40, as for the drive phase) and ≥ falling edge +
~1000 ticks; I_motor = raw / 0.690. Valid ≥ 6% duty. Not yet implemented in
`isense.c`/`drive.c`.

Next: re-measure τ on the rover (meter the motor terminals); same session A/B/A
on ≥ 2 wheels for the `ff_b` decision.

## 2026-09-26 (late night) — decay-phase sample implemented; valid at stall only

**Code (branch `w5-velocity-pid`).** Below 14.5% duty, in slow decay, current is
now sampled in the brake phase `[0, ccr)` instead of being refused.
- `drive.h/.c`: `DRIVE_DECAY_SETTLE_TICKS` 1000, `DRIVE_DECAY_SPAN_TICKS` 500,
  `drive_sense_t` {NONE, DRIVE, DECAY}, `drive_sense_kind/first/last()`.
  `place_trigger(start, ticks, decay_ok)`: drive phase if ≥ 652 ticks; else if
  decay allowed and `start ≥ 1152`: last = ccr − 152, first = max(1000, last − 500).
  Only the slow-decay call passes decay_ok. drive.c stays geometry-only.
- `config.h/.c`: `isense_dk` 690 o/oo (400–1000), `isense_dmin` 60 o/oo (30–145).
  New keys discard the stored cfg record (`cfg save` never run on the bench board).
- `isense.c/.h`: `isense_sync_ready()` accepts DECAY at |duty| ≥ isense_dmin;
  `isense_read_sync_avg()` scales DECAY readings ×1000/isense_dk after offset
  subtraction; burst spread over the sense window; `isense_sync_is_decay()`.
  Stale PMODE/high-side and free-running Isup = I×D text rewritten.
- `console.c`: `drv current` names the phase and window; NOT SYNCHRONISED gives
  the reason (duty 0/brake, fast decay, < isense_dmin). Telemetry flag 0x20.
- `tools/bench/node.py`: `FLAG_DECAY`, `Telem.decay`.
- Build clean: RAM 5720 B (4.36%), FLASH 107832 B (27.42%). Flashed; RAM-only
  cfg values trip_ma 1580, duty_limit 300, vel_slew 0, vel_tmo 2000 re-entered.

**Bench, 12.0 V.**
- First 20/10/20 run was with the shaft free (not clamped): void.
- Stalled 20/10/20: 20% drive phase 1329 / 1346 mA, within 0.9% / 1.3% of the
  iscan tail; 10% BRAKE phase window 3398..3898, 715 mA vs 669 expected = +6.9%.
- Stalled back-to-back at 10%: `drv current` raw 503/506 vs iscan 3398..3773
  mean 343 ×1000/690 = 497 → firmware matches the scan within 1.2%. Current at
  10% was 619 mA vs 715 ten minutes earlier: stall noise / winding temperature.
- Stalled 20/10/20 repeat: refs 1133 / 1290 mA (12.9% apart) → void. Stall
  A/B/A cannot resolve ±5% on this clamp; stopped.
- Free shaft, 20→10% in 1% steps, 64-sample `drv current` + single iscan per
  step: pure noise (102–343 mA, no trend) — commutation-scale ripple is slow
  against a 3.2 ms burst.
- Free shaft, telemetry 5 s/step after 10 s settle: drive 271–325 mA at 20–15%,
  brake 187–196 mA at 14–10%; flag 0x20 on exactly below 14.5%; 0 gaps.
- Free shaft, 0.5% steps 20.0→10.0, 60 s settle, 20 s of repeated iscans over
  the firmware's own windows (>250/step) + telemetry (~1000 lines/step):
  brake raw 102–110 flat over the whole sweep; drive raw 225 (20%) … 221 (17%),
  244 (16.5%), 251 (15%), 260 (14.5%); ratio 0.455–0.464 at 17–20%, 0.40–0.42
  at 14.5–16.5%; telemetry 274 → 313 mA, then 189 mA at 14.0% (−40% step);
  rpm 13.9 → 5.7. Wheel kept turning to 10%.

**Conclusion.** The code is right (placement, scaling, flag, telemetry all
verified). The 0.690 factor holds at stall only. While turning, back-EMF makes
the current ripple inside the period: the drive-phase sample (end of the drive
phase) reads near the peak and the brake-phase sample near the trough; neither
is the mean. Documented in `isense.h` and `_REF_DRIVE`; committed as is (user
decision). Open: supply-side DMM reference at 20% and 15% to find which phase
is biased; then maybe a speed-dependent factor (not before).

## 2026-09-26 (late night, bench) — supply-side DMM reference at 20% and 15%, free shaft

**Goal.** Open item from the decay-phase session: find which IPROPI phase is
biased while the wheel turns, using a DMM in series with the motor supply (VM).
In slow decay the supply only carries current in the drive phase, so
I_supply ≈ D × I_motor(mean) + quiescent.

**Setup.** Scratchpad script `hold.py` (not committed): telem 10 Hz, `drv timeout
2000` kicked every 0.5 s, `drv ramp 50` + `drv ramp floor 120` for the start
(restored to `drv ramp 0` after), 20 s at 0% then 90 s at the duty; prints only
10 s means of telemetry mA and rpm. Runs local in `tools/bench/runs/dmm-*`.

**False starts (no data, no damage).**
- First run: script left out `drv enable` — pins only, 0 rpm, ~10 mA. Fixed.
- DMM on the 10 A range: readings unusable (resolution). Firmware at 20%: 275 mA,
  13.8 rpm.
- After switching to the 500 mA range: motor power lead not reconnected — 0 rpm
  with duty 200, no fault flag, current ~17 mA (offset). User reconnected.

**Results (500 mA range, 12 V rail).** Baseline (0%, driver on) 6–7 mA, used 6.5.
| duty | DMM supply min–max | motor from DMM (sup−6.5)/D min/mid/max | firmware drive phase | fw ÷ DMM mid | rpm |
|---|---|---|---|---|---|
| 20% | 56–80 mA | 248 / 308 / 368 mA | 227 mA (223–231) | 0.74 | 12.9 |
| 15% | 48–67 mA | 277 / 340 / 403 mA | 283 mA (280–290) | 0.83 | 9.0 |
15% baseline not read (display was moving); 6.5 mA carried over from the 20% run.

**Notes.**
- The mA-range shunt (burden voltage) cut speed and current at 20%: 13.8 → 12.9 rpm,
  firmware 275 → 227 mA vs the 10 A-range run.
- DMM swung ±20%; likely the 12/rev mechanical ripple (~2.6 Hz at 13 rpm) beating
  with the DMM update rate. Only min/max were read, no average.

**Conclusion.** While turning, the drive-phase sample reads LOW (at or below the
DMM minimum at both duties), not high as guessed on Sep 26 night. The brake-phase
sample reads ~40% below the drive phase, so it sits near half the true mean.
Size of the error (0.74 vs 0.83) is inside the DMM's ±20% swing: no speed
dependence can be claimed, and no firmware factor is changed on these numbers.
Next: a steady supply-side reference (shunt + RC filter on the scope or a DMM on
mV, or a bench supply with a current readout), then decide on a turning factor.

## 2026-09-26 (late night) — first `cfg save` on the bench board; bench settings persisted

- Board read before: all 22 keys at compiled defaults (a reset had dropped the
  RAM-only bench values). W5 gains are the compiled defaults, so they need no
  save; freezing them waits for the rover τ session.
- Set and saved: `trip_ma` 1580 (default 1000 — the `--slew 0` steps peaked at
  984 mA, 16 mA under the default trip), `duty_limit` 300, `ramp_pmps` 50,
  `ramp_floor` 120. `cfg save` → slot 2 of 1024.
- Verified: board reset by hand, `cfg` read back — same four values, slot 2/1024.
  First exercise of save + boot restore on hardware. Corrupt-record fallback and
  the sector-full wrap (task 18) are still unexercised.

## 2026-09-26 (late night, desk) — W5 acceptance tolerance stated

Set from the 12 V rig data (forward/reverse staircases, A/B/A, true steps);
full text in `_REF_TASKS` task 21. All four must hold:
1. Tracking: 60 s hold mean − command ≤ ±0.05 rpm, both directions. Basis:
   per-hold standard error ~0.020 rpm (×2.5), worst seen 0.016 (×3); tighter
   would test the instrument, looser loses meaning against the 0.174 rpm
   open-loop repeatability floor. Instantaneous error is excluded: the ±1 rpm
   ripple is mechanical (12/rev).
2. Headroom: 0% saturated steps in a hold; peak output ≤ 95% of vel_max
   (seen 284/300 at 20 rpm).
3. No sustained oscillation: ripple 12.0 ± 0.5 per output rev, peak ≤ ±1.5 rpm
   (seen 11.91 ± 0.19, 1.42 rpm).
4. Bounded overshoot: `--slew 0` ±5 rpm step, overshoot_above_ripple False,
   rise ≤ 0.3 s (seen 0.08–0.26). Rise limit is a rig figure; re-set on the
   rover (2.87× inertia). 1–3 carry over.
Usable range on the 12 V rig: ~6–20 rpm each way. 10–20 rpm passes all four;
the 6–10 rpm staircase (both directions, A/B/A) is the remaining rig item.

## 2026-09-27 (bench, 00:00) — 6–10 rpm A/B/A staircase; W5 met on the rig; criterion 3 amended

**Setup.** DMM removed from the supply, VM 12.02 V at the driver. Three legs back
to back: `bench.py run stair --lo 6 --hi 10 --stair-step 0.5 --hold 60
--hold-settle 10 --abort-ma 1200 --trip 1580 --max-duty 30 --rate 50 --window 20`,
`--dir cw` / `ccw` / `cw`. Shipped gains. Runs (local):
2026-09-26T23-52-04_stair, 2026-09-27T00-01-08_stair, 2026-09-27T00-10-12_stair.
All clean: 0 gaps, 0 missed steps, ~2500 V rows per hold.

**Tracking (criterion 1).** Worst mean error: leg 1 +0.010, leg 2 −0.019 (at
−6.0), leg 3 −0.006 rpm. Pass (≤ ±0.05).
**Headroom (2).** 0% saturated everywhere; peak |out| 162 / 155 / 165 of 300.
**Ripple (3), subagent via `stairdata.py` `Stair.ripple()`.** Events/rev,
refined (interp=True): 12.01 ± 0.01, 12.00 ± 0.02, 12.00 ± 0.01; raw 20 ms grid
11.90–12.17 at every setpoint (grid-limited, identical in all legs). sd 1.00–1.19
(leg 1), 0.78–0.88 (leg 2), 1.01–1.12 (leg 3). Peak excursions: forward +1.4…+1.9
/ −3.2…−4.0 rpm; reverse 1.4–2.2 speeding / 1.4–2.9 slowing. 1 count per 20 ms
window = 0.357 rpm. Flat with speed.
**A/B/A.** Reverse needs ~4–6 o/oo less output at the same |rpm|; forward output
fell ~3–5 o/oo between legs 1 and 3 (integrator −3.3…+4.2 → −3.8…−0.8). Same
pattern as at 10–20 rpm.
**Current.** Telemetry ~190 mA at 6–9 rpm, then 302–321 mA at 9.5–10 forward:
the output crosses 145 o/oo there, i.e. the decay→drive sample switch, not a
real change.

**Criterion 3 amended.** The Sep 26 text had "peak ≤ ±1.5 rpm", taken from the
step's one-sided 4-count peak (1.42 rpm). The ripple is a lopsided dip; the Sep 26
10–20 rpm data (stair.csv min/max) already showed +1.2…+1.9 / −2.4…−4.4 rpm, so
that limit never matched the data it came from. New criterion 3: 12.0 ± 0.5
events/rev and within-hold sd ≤ 1.5 rpm; the peak dip is recorded, not pass/fail
(it measures the 12/rev mechanical feature). With it, 6–20 rpm passes all four:
W5 acceptance met on the rig. Remaining: rover τ session (re-set the rise limit,
freeze gains), reverse offset on ≥2 wheels.

**Tooling bug.** `./bench.py status` with no argument picks the "latest" run by
name, and timestamp-named runs (`2026-…_stair`) sort before letter-named ones
(`sweeptelem-…`), so it shows an old run. Workaround: pass the run folder.

## 2026-09-27 — W5 declared complete on the rig (user decision)

Nothing else in W5 can be done without the rover. Status: complete on the rig
(6–20 rpm, both directions, all four acceptance criteria). Deferred to the rover:
τ at real weight with the motor terminals metered, re-set the step rise limit,
freeze the gains (compiled defaults or `cfg save`), reverse-vs-forward offset
A/B/A on ≥2 wheels to settle `vel_ff_b_rev`. Task 21 stays open (🟡) for those.

## 2026-09-27 — 12-per-revolution ripple source identified

The ±1 rpm velocity ripple, pinned at exactly 12.00 events per output revolution
in both directions (0.02%), is the tyre: the wheel's rubber has 12 tread grooves
for grip on rough ground. Identified by the user on inspection. Resolved; no
firmware or cfg change. The W5 ripple criterion (12.0 ± 0.5/rev, sd ≤ 1.5 rpm)
therefore measures the tyre on this rig, not the loop.

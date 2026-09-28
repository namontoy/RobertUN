# RobertUN wheel firmware — reference: tasks — open and in progress

> Reference tier. Moved verbatim from `PROJECT_CONTEXT_WHEEL_FW.md` on 2026-09-26.
> Do not read whole: `grep -n '^#' <this file>` and read the section you need.
> Contents: full text of every open task; closed tasks move to the LOG file

## NEXT TASKS — wheel firmware track

(Original numbering preserved for cross-reference with PROJECT_CONTEXT_REST.md)

Tasks 17, 18 and 20 were closed on 2026-09-28; their full text is in the LOG
(heading "Tasks 17, 18 and 20 closed by the user"). References to them in the
other files point there.

### Task 6 — CAN bus — STM32 firmware (W2 done; RX interrupt-driven, closed Sep 28)

6. **CAN Bus — STM32 firmware:** ✅ **COMPLETED August 10, 2026** (roadmap W2).
   250 kbps bxCAN, accept-all filter on bank 0, DMA console, loopback self-test,
   termination measured 59.79R, three-way verification against Orion `candump`
   and the CANable with zero error counters. Full detail in the LOG file.
   - ✅ `cmd_errors` ESR snapshot fixed Sep 28 (`can_bus_errors()`, one read of
     `CAN1->ESR`).
   - ✅ **RX decision closed Sep 28, 2026: interrupt-driven, ISR-to-ring**
     (`CAN1_RX0` at preemption 1, 32-frame ring), merged from branch
     `ISR-to-ring`. Bench steps 1-7 in the plan `docs/plans/ISR-to-ring.md`
     passed (20000/20000 at saturation, 0 overruns; overflow counted, not
     silent). Not verified: the delivered-count invariant. Details in
     `_REF_MCU` "DECIDED — interrupt-driven CAN RX" and the LOG.

### Task 21 — W5 — velocity PID (COMPLETE ON THE RIG Sep 27; rover items pending)

21. 🟡 **W5 — velocity PID on the drive motor (OPENED Sep 20, 2026; COMPLETE ON
    THE RIG Sep 27, 2026).** Deferred until the rover exists: re-measure τ at
    real weight (meter the motor terminals), re-set the rise limit, freeze the
    gains, and the reverse-vs-forward offset A/B/A on ≥2 wheels (`ff_b` decision).
    Acceptance criterion, to be met on the loaded wheel rig at real weight:
    **commanded output speed is held within a stated tolerance across the usable
    speed range, with no sustained oscillation and bounded overshoot from a
    step.** The tolerance is deliberately left to be set from the 12 V plant
    re-measurement rather than picked now from 9.35 V figures.

    ✅ **TOLERANCE STATED Sep 26, 2026** (from the 12 V rig data; all four must hold):
    1. **Tracking:** mean speed over a 60 s hold minus command, both directions,
       **≤ ±0.05 rpm** (2.5 × the ~0.020 rpm per-hold standard error; 3 × the
       worst seen, 0.016). Measured by encoder over the hold, not instantaneous.
    2. **Headroom:** **0%** of control steps saturated during a hold, and peak
       output **≤ 95% of `vel_max`** (seen: 0%, 284/300 at 20 rpm).
    3. **No sustained oscillation:** within-hold ripple locked to rotation,
       **12.0 ± 0.5 events per output rev**, and within-hold **sd ≤ 1.5 rpm**
       (seen 11.91 ± 0.19 at 10–20 rpm, 12.00 ± 0.02 at 6–10; sd 0.78–1.19).
       A limit cycle holds a period; only a rotating feature holds a count per
       rev. **Amended Sep 27:** the first text had "peak ≤ ±1.5 rpm", taken
       from the step's one-sided 4-count (1.42 rpm) peak. The ripple is a
       lopsided dip: −3.2…−4.0 rpm forward, 1.4–2.9 rpm reverse, the same at
       10–20 rpm on Sep 26. Peak dip is recorded, not pass/fail — it measures
       the 12/rev mechanical feature, not the loop.
    4. **Bounded overshoot:** true step (`--slew 0`), ±5 rpm:
       `overshoot_above_ripple` False, **rise ≤ 0.3 s** (seen 0.08–0.26 s).
       The rise limit is a rig figure — re-set it in the rover τ session; 1–3
       carry over unchanged.
    **Usable range, 12 V rig:** ~6–20 rpm each direction (bottom from breakaway
    at 9–11% duty; top where `vel_max` 300 leaves ~5% headroom). 10–20 rpm
    passes all four; ✅ **6–10 rpm A/B/A passes all four (Sep 27)** — W5 is
    met on the rig; remaining: re-set the rise limit and freeze gains after
    the rover τ session.

    **What carries over from W4, already established:**
    - The plant is **linear to ±1.5%** across the duty range, so **no gain
      scheduling** — the single most useful thing the Aug 26 sweep established.
    - **Sign convention: positive duty → CW → positive encoder counts**,
      confirmed at every point. If the loop ever runs away inverted, the fix is
      to swap the MOTOR leads, not the encoder.
    - **Direction asymmetry ≈ 3.5%** (CCW faster above 20% duty) — normal brush
      timing, absorbed by integral action, not a thing to compensate explicitly.
    - Two motors matched to <2% on R and L, so **one gain set should fit all six
      wheels** — the assumption W7 rests on, worth re-checking on a third motor
      before it is relied on.
    - TIM2 is 32-bit and free-running, so **counter rollover is off the table**
      for the whole of tuning.

    ✅ **UNBLOCKED Sep 25, 2026 — the 12 V plant re-take (task 17) is done,
    free wheel and loaded rig.** W5 now has a model to design against:

    > **`rpm = 0.7993 × duty% − 2.420`** (11–29%, 1047 g on the belt, 12 V),
    > inverse **`duty% = 1.251 × rpm + 3.028`**; **two-pole step response,
    > τ_fast 0.219 ± 0.007 s (84%) + τ_slow 2.75 s (16%)**; breakaway and
    > dropout both in **9–11% duty**; **repeatability floor ±0.174 rpm**.

    ⚠️ **That line is 4.5% optimistic as of Sep 26 and is due a re-take.** Two
    independent runs that day at 29% duty on the same rig settled at **19.82 rpm**
    (ramped) and **19.97 rpm** (un-ramped) by count slope, against the fit's
    **20.76**. Two runs agreeing with each other and disagreeing with the fit
    points at the fit, not the runs; belt tension is not a controlled condition
    between sessions. **Do not design gains against 0.7993 without re-measuring
    first** — and re-record belt tension as a run condition when doing so.

    Four things W5 must design around rather than rediscover:
    - **Gain droops 25% across the band** (0.899 → 0.700 rpm/% from 11 to 29%).
      The model is deliberately kept linear because the curvature sits inside
      the repeatability floor — so **place the gains at the low-gain, high-duty
      end** and accept a slightly sluggish bottom rather than a marginal top.
    - **τ_fast is the pole to place against; τ_slow is the belt, not the
      motor** — do not integrate against a 2.75 s pole that will not exist on
      the vehicle.
    - ⚠️ **The rig's τ is a LOWER BOUND.** Its 1047 g is a normal force, not
      inertia; the rover carries ~3 kg per wheel, so the rig is **2.87× light**
      and τ on the vehicle will be longer. **Re-measure τ on the rover before
      the gains are frozen.**
    - **Breakaway is a voltage threshold**, so the 9–11% figure belongs to the
      12 V rail alone. If HW4's branch rail lands anywhere else, breakaway moves
      with it — convert, do not carry the duty across.

    The rail moved 9.35 V → 12.0 V on Sep 19, which retired
    `rpm = 0.672 × duty% − 1.8`, the ~2.6% deadband and the old 4.9 rpm minimum
    sustainable speed. R, L and Ke carried over; **duty→speed and duty→current
    did not, and gains tuned at one rail do not transfer.** Still owed at
    tuning time: meter the **motor terminals**, not just VM — 12.0 V at the
    driver is not 12 V at the motor.

    **Can start now, without the bench:**
    - ✅ **PID gains as `config` keys BEFORE tuning starts — DONE Sep 26, 2026.**
      Nine keys, all in **milli-units** because the store is int32-only:
      `vel_kp` 3000, `vel_ki` 10000, `vel_kd` 0, `vel_ff_a` 12510, `vel_ff_b` 30,
      `vel_ilim` 150, `vel_max` 300, `vel_slew` 4000, `vel_tmo` 1000. Each is
      live-applied by `cfg <key> <val>` without a save, which **is** the
      volatile set-for-this-session path the note below asked for: the log is
      append-only with 1024 slots, so a 50-point gain scan that persisted each
      trial would burn 5% of it. `cfg save` persists only a keeper.
      **Adding them cost nothing** — `cfg save` has still never been run on this
      board, so there was no stored record to discard. (Original note follows.)
    - ⬜ **PID gains as `config` keys BEFORE tuning starts** (task 18 flags this
      as the biggest payoff, and it is right — re-flashing to change a gain makes
      tuning miserable). `kp`, `ki`, `kd`, output clamp, integral limit. **Do it
      in the same version bump as task 20's** — `CONFIG_VERSION` is already at 2
      and the stored record is already discarded, so adding them now is free,
      where adding them later costs a second wipe. **Design a volatile
      set-for-this-session path in the same bump** (raised Sep 25, 2026): the
      log is append-only with 1024 slots and one full snapshot per save, so a
      50-point gain scan that persists each trial burns 5% of it. Persist only
      a keeper. Cheap to build in now, a retrofit later.
    - ⬜ **Tuning data now comes from `tools/bench/`, not by hand** (Sep 25,
      2026). `telem` streams at up to 100 Hz with a board timestamp and a
      sequence number, which is what makes a step response or a coast-down
      measurable at all — the console's 5 Hz hand-read pace never could.
    - ✅ **Control-loop skeleton — DONE Sep 26, 2026, `velocity.c`/`velocity.h`.**
      It hangs off the TIM6 1 kHz tick but **advances at the measurement rate,
      not the tick rate**: it runs only when `encoder_velocity_seq()` changes,
      which is once per `enc window` — 50 Hz at the default 20. Running a loop
      faster than its sensor updates differentiates a staircase and integrates
      the same error repeatedly; the encoder's boxcar sets the honest rate.
      Includes feedforward from the inverse plant with a sign-of-setpoint
      friction offset, the setpoint ramp (see the next bullet — it belongs here,
      as this file predicted), a four-condition anti-windup freeze that reports
      *which* condition fired, and its own setpoint watchdog. **Untested on
      hardware.**
      - ⚠️ **It defeats `drive.c`'s command watchdog by construction, and that
        is deliberate but must be remembered.** `drive_set_duty()` kicks that
        watchdog, and the loop calls it 50×/s forever, so **arming the loop
        means drive.c's watchdog can never expire**. That is why `velocity.c`
        carries its own setpoint watchdog — armed by default, the opposite of
        drive's — and why only `vel target` kicks it: a setpoint is evidence
        something upstream is still choosing, which is the thing actually worth
        watching once the loop is keeping the layer below alive.
      - ⚠️ **Shipped with no `volatile` on any ISR-written static.** Caught the
        same day while adding the telemetry snapshot. Harmless until something
        copied the state out; the lesson is that a new module written against
        an existing tick does not inherit the tick's discipline automatically —
        `encoder.c` and `drive.c` both had it right and it still got missed.
    - ✅ **DUTY SLEW-RATE LIMITER — ramp the speed command, never step it
      (raised Sep 23, 2026, at the bench).** `drv duty 60` from rest is a step
      change, and because back-EMF is zero at t=0 it is a stall-current event
      every time — the trip fires on each step, which is the protection working
      but is not how the rover should be driven. Needs a gentle
      acceleration/deceleration ramp between the commanded speed and what
      reaches the bridge, so current never spikes to the limit in normal
      operation.
      - ✅ **Implemented AND bench-verified on the loaded rig Sep 26, 2026.** `drv ramp <o/oo per s>` and
        `drv ramp floor <o/oo>`, per-mille on the existing 1 kHz TIM6 tick,
        **off by default**. Also added: `drv duty <n>p` for a per-mille command,
        which unblocks the 1%-step stiction bracket below.
      - ⚠️ **It went INSIDE `drive.c`, which reverses the position this file
        held until Sep 26** — the line below said "control layer, not
        `drive.c`", and that placement turned out to be unbuildable for one
        specific reason worth keeping: **`drive_set_duty()` calls
        `drive_kick()`**, deliberately, because a duty command is evidence of a
        live host. A ramp module *above* `drive.c` must reach the bridge through
        that function, so it would refresh the command watchdog **1000×/s
        forever** and a dead host would never again be detected. The watchdog is
        the one safety property that exists purely for the unattended case. A
        secondary reason: a limiter any caller can go around is advisory, and
        inside the module the invariant is unconditional.
      - The split is the **command watchdog's**, not a new one: mechanism in
        `drive.c` (a rate the caller arms, 0 = off = the old behaviour bit for
        bit), policy above. `drive.c` still decides nothing — not whether to
        ramp, not how fast, and still not what to do about nFAULT.
      - Ramping the **setpoint**, once the velocity loop exists, is a *different*
        ramp and **does** still belong to the control layer — limiting the
        setpoint is what keeps the PID from winding up against its own ramp, and
        `drive.c` has no setpoint. Not done.
      - **`coast` and `brake` are deliberately NOT ramped.** Coast is the safe
        stop and the watchdog's action; a dead host is not the moment to ease
        off over six seconds. Brake is an explicit act. The standing "ramp duty
        down before braking" policy (see *Stopping: coast or brake*) is
        therefore written **above** as `drive_set_duty(0)` → wait for
        `!drive_slewing()` → `drive_brake()`, which is what the new
        `drive_slewing()` accessor is for. Tightening `drive_set_limit()` is
        immediate too: a cap is protection, not a command.
      - Rate and floor are `config` keys (`ramp_pmps`, `ramp_floor`), tunable on
        the bench without a reflash. **`CONFIG_VERSION` was NOT bumped** — no key
        changed meaning — which corrects the earlier note here that paired this
        with the PID bump. But see the key-count gotcha in *Key learnings*:
        adding keys still discards any stored record.
      - `drive_duty()` now reports what the bridge is **actually running**, not
        the target, so a ramp appears in telemetry as a ramp — which is how it
        gets measured. `drive_duty_target()` is the commanded value.
      - This also makes the low-duty band usable: a ramp that walks up through
        stiction is gentler than a step that has to break it.
      - **The stiction numbers measured Sep 23 set the ramp's floor.** Breakaway
        is 5–6% duty but dropout is 2–3%, so a ramp from rest must actually
        reach ~6% to get the wheel turning — it cannot creep in at 3% and expect
        motion. Once moving, the command can fall back to 3% (≈1.6 rpm, the
        slowest sustainable speed). A velocity PID sees this for free: the
        integrator winds up through breakaway, then unwinds. But **the ramp rate
        must not be so slow that the wheel sits energised below breakaway for a
        long time** — that is stall current with no back-EMF, exactly the
        condition the trip exists for.
      - ✅ **A host-side prototype exists and is proven on the rig (Sep 25).**
        `bench.py --ramp <%/s> --ramp-from <%>` walked 12→29% at **5%/s** with a
        **peak of 572 mA against the 1580 mA trip — 2.8× headroom**, where
        un-ramped duty steps had been tripping. It also confirms the floor rule
        above in code: `--ramp-from` is **stepped to directly and held 1 s**,
        because a ramp through a stationary wheel means nothing until it has
        broken away. **Treat it as the reference behaviour to port**, with two
        differences the firmware must fix: it is a host-side staircase at
        **1% granularity** (`drv duty` parses percent), and it runs at the
        console's pace rather than on the **1 kHz tick**. The firmware version
        should be **per-mille on the tick**.
      - ⚠️ **The rig's τ_fast (0.219 s) is a lower bound on how fast the ramp
        may usefully be** — ramping much faster than the plant can follow just
        re-creates the step. On the rover, where inertia is ~2.87× higher, the
        usable rate is lower still.
      - ✅ **The bench pass is done (Sep 26).** Desk first with the driver
        disabled — rate, floor, down-ramp, reversal and live config apply all
        exact — then the rig, logged to file at 100 Hz rather than pasted.
        **1462 lines, no seq gaps, no fault, no ADC saturation.** Rate fitted
        over 340 samples: **50.00 o/oo/s** against 50 commanded, and the
        120→290 climb took **3400 ms against 3400 predicted**. Floor jumped
        0→120 in one sample with the encoder moving **10 ms** later — the same
        breakaway latency as a hard step, so the ramp costs nothing at start.
        **Peak synchronised current 581 mA** against the host prototype's 572,
        **2.7× under the 1579 mA trip**.
      - ✅ **The A/B against `drv ramp 0` is the evidence the feature works.**
        Same 0→29% command, un-ramped: **1582 mA on the very first sample
        against a trip programmed at 1579**, holding ~40 ms in regulation
        (1300 / 1008 / 911 / 913 mA) before back-EMF built. That 1582 is the
        **clamp's value, not the demand's** — at 10 ms sampling the true peak is
        unknown and higher. **So the ramp reduces peak inrush 2.7×**, from
        at-the-limit to 37% of it.
      - ⚠️ **The un-ramped start sets NO fault flag.** `flags` never set bit 4
        (nFAULT) or bit 8 (ADC saturated) through the whole regulated event. So
        stepping duty from rest does not fail visibly — it silently leans on the
        DRV8874's hardware current limit on every single start, which is a
        better argument for the limiter than a visible trip would have been,
        because nothing in the telemetry would ever have surfaced it.
      - ✅ **The watchdog non-regression test passed twice — at the desk and
        then with the motor live**, which is the test that decides the placement
        argument. Live: the ramp had been writing the CCR for 3.4 s and holding
        for 6.6 s, and the 10 s deadline still landed on the **exact
        millisecond**, with duty dropping **290→0 in a single sample** — a
        coast, not a ramp-down. `emit()` is not kicking the watchdog.
      - ⬜ **Not done, and probably not needed: the scope check of PWM high
        time.** The 50.00 o/oo/s fit and the exact 3400 ms climb measure the
        same thing from the telemetry side. Left open rather than claimed.
      - ⬜ **`cfg save` has NOT been run**, so the board still boots with the
        ramp **off**. The rate and floor (50 / 120) are live-set only. One
        command whenever the bench wants them persistent.
      - `bench.py --ramp` is **kept, not deprecated**: it is the reference the
        firmware version is checked against, and the un-ramped case is the
        evidence the feature works.
      - The arithmetic was checked offline before the board was touched: the
        per-tick step in **milli-per-mille equals the rate in per-mille per
        second**, exactly, so there is no division in the tick and a 50 o/oo/s
        ramp lands on its target rather than near it. The accumulator has to be
        milli-per-mille because 5%/s is **0.05 per-mille per tick** — an integer
        per-mille accumulator would stall at zero or run 20× fast.
    - ✅ **Telemetry path — DONE Sep 26, 2026, and RTT was not needed.** A
      second opt-in console record, **`V,seq,ms,sp_mrpm,meas_mrpm,out,ff,p,i,d,flags`**,
      behind `telem vel on`. Integer fields only, like `T,`. The four terms are
      carried **separately and before the clamp**, so `out` differing from their
      sum is exactly the saturation, and an oscillation says in the data which
      term is driving it.
      - **One line per control step, not on the `telem` timer.** The loop runs
        at `1000 / enc window` Hz; riding the telem schedule would alias it —
        duplicates at 100 Hz, beats at 30 Hz — and the integrator and derivative
        only mean anything per step.
      - **A separate record, not more columns on `T,`.** `Telem.parse()`
        length-checks its fields and every committed run directory holds a
        7-field `telemetry.csv`, so a widened `T,` would silently mean two
        incompatible things depending on which reader saw it.
      - **Overrun is reported, not hidden.** One slot, and a step the main loop
        failed to drain sets a sticky bit OR'd into the next line. A silently
        decimated stream reads as a *slow control loop* — precisely the wrong
        conclusion for someone about to change a gain.
      - **The flag set splits the freeze reason.** Frozen alone is anti-windup
        working; frozen with slewing or no-bridge is the loop held off by
        something else. They look identical in `out` and want opposite
        corrections.
      - ⚠️ **Bandwidth is the real limit.** 115200 8N1 is 11.52 kB/s; `T` at
        100 Hz plus `V` at 50 Hz is ~9.7 kB/s before the echo of anything typed.
        **`telem rate 50` is the pairing that fits**, and `telem vel on` warns
        below 20 ms.
    - ✅ **`bench.py run step` — the profile the loop gets tuned against.**
      Commands a setpoint step and reduces it to rise time (10→90%), overshoot,
      settling to ±2%, steady-state error, saturation and freeze fractions split
      by reason, and **the integrator's resting value — which is how much the
      feedforward missed by**, a large steady `i` with a small error meaning
      `ff_a`/`ff_b` want re-fitting rather than Ki raising. Two defaults that are
      deliberately *not* the sweep's: **`enc window 20`** (100 makes it a 10 Hz
      loop whose 100 ms lag would dominate the response being measured) and
      **`cfg ramp_pmps 0`** (drv's limiter and `vel_slew` in series means
      `drive_slewing()` is true almost continuously and the run measures the
      limiter). Its metrics come from `meas_mrpm`, the boxcar the controller
      acted on — the right frame for choosing gains, the wrong one for a plant
      time constant.

      ⚠️ **REWRITTEN Sep 26 after the bench pass — the first version measured
      `vel_slew` and reported it under the loop's name.** See the figure and the
      Key Learning below. What changed:

      | field | what it now means |
      |---|---|
      | `ramp_s`, `slew_rpm_s` | how long the setpoint ramp ran and how fast — recovers the configured 4 rpm/s to 0.8% |
      | `anchor_s` | the instant `VFLAG_RAMPING` cleared; **`overshoot` and `settle_s` are measured from here** |
      | `track_lag_rpm` | how far behind the moving setpoint the loop sat *during* the ramp — **the quantity a gain change moves** |
      | `rise_s` + `rise_slew_floor_s` + `ramp_limited` | rise is still from the command (it is what an operator waits), reported beside the floor the limiter imposes and flagged when it is inside 30% of it |
      | `settle_s` + `settle_from_command_s` | the loop's own, and end-to-end. **The same instant** — their difference is `vel_slew` |
      | `tail_sd_rpm` + `overshoot_above_ripple` | the settled ripple, and whether the peak clears 2 sd of it. **False means the "overshoot" is ripple** |
      | `overshoot_window_s` + `overshoot_window_truncated` | the peak is searched over 3 s (the plant's slow pole is 2.75 s) so a longer `--dwell` cannot manufacture a bigger overshoot; flagged when a run was too short to fill it |

      Two stderr warnings fire on a run that cannot answer the question asked of
      it: one when `ramp_limited`, pointing at `track_lag_rpm` or `--slew 0`,
      and one when the ±2% settle band is **narrower than one encoder count**,
      where `settle_s` cannot be computed honestly at all.

      ![velocity step response, anchored at the end of the setpoint ramp](figures/velocity_step_slew_anchor.png)

      *Regenerate with `python3 figures/plot_velocity_step_anchor.py`.
      ⚠️ **`figures/stepdata.py` imports `step_metrics` from `bench.py` and calls
      it** rather than carrying a second implementation — a figure that documents
      a metric must not be able to disagree with it. A: the 0→20 rpm run, with
      the 5 s of setpoint ramp shaded and the one settling instant shown against
      both references. B: tracking lag during the ramp, all three runs. C: every
      peak against ±2 sd of that run's settled ripple. D: the term breakdown,
      showing the 6% saturation is entirely inside the acceleration.*

    - ✅ **THE BENCH PASS — 21 continuous minutes, and the shipped gains stand.**
      21 setpoints 0.5 rpm apart, 10.0 → 20.0 rpm, **60 s each, uninterrupted**
      (`runs/2026-09-26T09-55-08_stair`). A step response says whether the loop
      is *stable*; it says nothing about whether it is *accurate*, whether it
      drifts, or whether the ¼-of-textbook derating costs anything — and those
      are the questions that decide whether these gains go to the rover.
      **63,202 `V` rows, 0 sequence gaps, 0 unpublished control steps,
      0 tx_dropped**; the overrun bit never fired in 21 minutes at 50 Hz.
      - **Tracking: mean error +0.0008 rpm, worst 0.015, sd 0.005**, against a
        per-point standard error of ~0.020 rpm. **Accurate below the noise floor
        of the instrument measuring it**, at all 21 speeds.
      - **0% saturation at every hold**, peak **284 of 300 o/oo**. This
        **corrects** the 6% a short step reported at 20 rpm — that was the
        acceleration transient, not the operating point.
      - **The derating costs no accuracy, because the model behind the
        feedforward is right.** Closed-loop inverse **`out = 12.559 × rpm +
        30.54 o/oo`** (rms 1.51, n=21) vs Sep 25's open-loop sweep inverted,
        `12.511 × rpm + 30.28` — **0.39% apart**. Shipped `ff` error at 15 rpm
        **−1.3 o/oo (−0.6%)**; integrator **mean +1.46 o/oo of ~219 commanded**.
        ⚠️ **The "~4.5% optimistic" caveat on that plant model does not hold in
        this band.**
      - ⚠️ **The ±1 rpm ripple is MECHANICAL, and the run proves it.**
        **11.91 ± 0.19 events per output revolution** across all 21 holds, while
        the *period* swept 500 → 260 ms. Identifying the feature is a mechanical
        inspection, not a telemetry one. *(Refined below to* **11.998 ± 0.017**
        *— same holds, same data, sub-bin peak interpolation. The ±0.19 here is
        the 20 ms lag grid, not the feature's spread.)*
      - **No drift** within a hold (+0.0005 rpm / −0.04 o/oo over 60 s), and the
        low-end current U-shape is **not** warm-up — a 90 s re-take 22 minutes
        later reproduced it. ⚠️ It sits near the 145 o/oo synchronised-sense
        floor and is the one number in the figure to distrust.

      ![the velocity loop over 21 continuous minutes](figures/velocity_loop_stair_21min.png)

      *Regenerate with `python3 figures/plot_velocity_loop_stair.py`;
      `figures/stairdata.py` **transcribes nothing** and the script ends by
      printing the same numbers it drew. A: all 21 minutes of raw control steps
      with the settled means and a tracking-error inset. B: output vs achieved
      rpm against the 300 o/oo ceiling, with the shipped feedforward dashed
      underneath the fit — it hides under it, which is the finding. C: ripple
      period vs setpoint against the fixed-period line a limit cycle would lie
      on. D: current vs output, with the sense floor and the warm re-take.*

      ⚠️ **Still unexercised after all this:** bit 64 (MISSED) never fired;
      **`cfg save` has still never been run**, so every gain above is RAM-live;
      and a true loop rise time needs **`--slew 0`**, which has not been run.

    - ✅ **THE SAME STAIRCASE IN REVERSE — 21 more minutes, and the asymmetry is
      the integrator.** Identical profile at −10.0 → −20.0 rpm
      (`runs/2026-09-26T11-02-46_stair`), same gains, same rig, **67 min after
      the forward run**. **63,203 `V` rows, 0 gaps, 0 unpublished steps,
      0 tx_dropped**; `outcome=ok`, not SUSPECT. Reverse is not a symmetry
      check you can skip — the rover reverses, and the feedforward's sign
      handling ([`velocity.c:346`](../../firmware/RobertUN_ModuleNode/Core/Src/velocity.c#L346))
      takes the sign of the *setpoint*, so it is symmetric **by construction**
      and cannot absorb a plant that is not.
      - **Tracking is as good or better: worst settled error 0.006 rpm**
        (forward 0.015), per-point sem 0.019. **0% saturation at all 21 holds**,
        peak **277 of 300 o/oo** — ⚠️ **this corrects a prediction I made** from
        the single −10 rpm direction check, that the top of the reverse range
        would saturate. It does not, because the asymmetry **changes sign**.
      - **The headline: `corr(Δ|out|, Δi) = 0.9986`, 0.62 o/oo rms apart.**
        The feedforward is bit-identical in both runs, so every bit of direction
        dependence has nowhere to land except the integrator — which makes
        **the integrator a direct readout of the model's direction error**.
        That is how a separate reverse `ff_b` gets *measured* rather than
        guessed.
      - ⚠️ **"Reverse is x% harder" is false.** The asymmetry spans
        **−10.0 to +1.7 o/oo** (mean −2.74) and **changes sign at 14.3 rpm** —
        reverse costs *more* below it and *less* above. It clears the
        comparison's own **±2.16 o/oo noise floor** (quadrature sum of the two
        fits' residuals) at only **9 of 21 setpoints**. Reverse inverse
        **`out = 11.503 × |rpm| + 43.65`** (rms 1.54) vs forward
        `12.559 × rpm + 30.54` — different slope *and* intercept, which is why
        no single scalar describes it.
      - **The ripple is the same mechanical feature, and it is 12.00 per
        revolution.** **11.998 ± 0.017** forward, **11.978 ± 0.017** reverse;
        paired difference **−0.0204 ± 0.0030**, negative at 20 of 21 setpoints
        — resolved, but 0.17%, far too small to be a different feature.
        ⚠️ **This needed an estimator fix to be honest.** On the raw grid an
        autocorrelation period is an integer count of 20 ms control steps, worth
        **±0.72 events/rev** at these speeds — so both directions returned
        *identical digits* at all 21 holds, and reporting that as agreement
        would have claimed a precision the method does not have. Sub-bin
        parabolic peak refinement (opt-in, so the forward figure's published
        numbers do not move) tightens it 11×. The ripple's **amplitude** is
        *not* the same: 0.87 → 1.21 rpm forward, 0.88 → 1.00 reverse.
      - ⚠️ **Reverse draws +8.1% current (292 vs 270 mA) while commanding LESS
        output** at the top of the range. More current for less duty is either a
        real direction-dependent load or a **sign-dependent offset in the
        current sense**; these two runs cannot tell which, and the low end sits
        inside the 145 o/oo sense floor's reach anyway. Still wants an
        independent ammeter.

      ![the same staircase both ways, forward and reverse](figures/velocity_loop_stair_direction.png)

      *Regenerate with `python3 figures/plot_velocity_loop_stair_direction.py`;
      shares `figures/stairdata.py` with the forward figure and likewise
      transcribes nothing. A: the difference curves — Δ|out| and Δi lying on top
      of each other, against the ±2.16 o/oo floor, with the unchanged tracking
      error inset. B: both inverse fits against the 300 o/oo ceiling and the
      shipped feedforward. C: events per revolution both ways against exactly
      12, with the amplitude inset that does *not* agree. D: current vs output
      with the sense floor.*

      ⚠️ **THE CONFOUND, carried on the figure itself:** the two runs are
      **67 min apart and NOT interleaved**, so temperature, belt tension and
      where the carriage sits on the belt — which travels the *other way* in
      reverse — are all aliased into "direction". **An A/B/A staircase would
      separate them, and has not been run.** Until it is, "direction" is the
      honest label for the difference but not a proven cause of it.

    **Open work the two staircases leave behind:**
    - ⬜ **An A/B/A staircase, to un-alias "direction" from everything that
      drifted between the two runs.** Forward, reverse, forward again, in one
      session: if the return leg reproduces the first, direction is the cause;
      if it lands between them, drift is. ~63 min, no rewiring, and it settles
      the one caveat on every number in the comparison figure. Run this before
      anyone acts on the reverse `ff_b` the integrator is pointing at.
    - ✅ **Identify the ~12-per-revolution mechanical feature.** Resolved 09-27:
      the tyre's 12 tread grooves (for grip on rough ground). Now pinned at
      **exactly 12.00 per output revolution** in both directions, to 0.02%. That
      is a count, not a frequency, so it is a mechanical inspection — gear teeth,
      a coupling, or the wheel-to-belt contact — and not a telemetry question.
    - ⬜ **An independent ammeter on the low-duty end.** The +8.1% reverse
      current and the low-end U-shape both sit near the 145 o/oo synchronised
      sense floor, and neither can be trusted from the board's own reading.

    **Two constraints inherited from Sep 20, to be designed around rather than
    discovered during tuning:**
    - ⚠️ **No synchronised current reading below 14.5% duty** at the 20 kHz
      carrier — the honest floor, replacing a 4.3% one that was admitting pure
      ringing. **W5 operates near the stiction floor, which is below it**
      — and the 12 V re-take made the gap *wider*, not narrower: **breakaway is
      now 9–11% duty** on the loaded rig (it was 12–14% at 9.35 V, which is the
      same terminal voltage at a lower rail). A **current inner
      loop** — re-enabled by the 13.5 V branch-rail decision — is therefore blind
      in exactly the region the rover creeps in. Options are the free-running
      `Isup` fallback, a slower PWM carrier in that band, or the **decay-phase
      lead below**. **Decide before tuning, not during.**
    - ✅ **DONE Sep 26 (night) — the decay phase IS readable.** Stalled A/B/A
      scans 5–12% duty: raw = **0.690 × I** (±1.5%, 17 refs at 20%), within
      **±4%** from 6% duty; 5% invalid. The 0.670 below was edge-contaminated
      (t3550 is 50 ticks before the drive edge). Decision: decay-phase reading;
      `Isup` and the slower carrier are not needed. → LOG 2026-09-26 (night)
    - ~~⬜ **One experiment worth running first: is the DECAY phase readable?** (superseded above)~~
      In both Sep 21 traces the decay-phase tick 3550 read a fixed **0.670** of
      the drive-phase tail (642/958 and 709/1056 — the same ratio to three
      digits across runs that differed 9% from each other). Physics says a 40 µs
      brake window at τ = 0.9 ms should lose only **4.4%**, not 33%, so
      something is attenuating the brake-phase mirror by a *reproducible*
      factor — and a reproducible factor is a calibration, not noise. If it
      holds it **dissolves this constraint entirely**, because the decay window
      is widest exactly where the drive window is too narrow to sample. Two
      points at one duty is a lead, not a result: needs a decay-phase scan
      across several duties before anything is built on it.
    - ⚠️ **The loop cannot measure current while it is hard-limiting.** Push the
      DRV8874 ~90% below demand and IPROPI collapses to near zero far faster
      than L/R decay allows; regulation is audible before it is visible. Any
      current-aware supervision has to treat "trip active" as a distinct state,
      not as a low reading.


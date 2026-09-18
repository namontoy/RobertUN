# RobertUN — Wheel Controller Firmware: Full Progress Log
**Last updated:** September 17, 2026 (split out of `PROJECT_CONTEXT.md`; this file now carries the detailed session log for the wheel-firmware track)

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

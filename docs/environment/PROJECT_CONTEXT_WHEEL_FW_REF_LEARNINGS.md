# RobertUN wheel firmware — reference: key learnings and gotchas

> Reference tier. Moved verbatim from `PROJECT_CONTEXT_WHEEL_FW.md` on 2026-09-26.
> Do not read whole: `grep -n '^#' <this file>` and read the section you need.
> Contents: every lesson with its evidence; the hot file keeps only one-line rules

## KEY LEARNINGS & GOTCHAS

⚠️ **AN ESTIMATOR THAT RETURNS THE SAME ANSWER TWICE HAS NOT AGREED WITH
ITSELF — CHECK ITS RESOLUTION BEFORE YOU BELIEVE IT.** Found Sep 26, 2026 while
drawing the forward/reverse comparison. Two independent 21-minute runs returned
ripple counts **identical to every digit** — 11.91 ± 0.19 both ways. That reads
as a beautiful confirmation and is actually the opposite: the autocorrelation
period is quantised to an integer number of 20 ms control periods, so the
smallest difference the method can express is **±0.72 events/rev**, four times
the "spread" being quoted. Both runs simply selected the same bin at all 21
holds. **Suspicion should rise, not fall, when two measurements agree better
than their own quoted uncertainty** — the agreement is then a property of the
instrument, not of the thing. The test is arithmetic and takes a minute: work
out what one quantum of the estimator is worth in the units being plotted, and
compare it to the difference being claimed. The fix here (parabolic sub-bin peak
refinement) tightened the estimate 11× and turned a non-claim into a real one —
**exactly 12.00 per revolution**, with a resolved 0.17% direction offset. Add
any such refinement **opt-in**, and md5-baseline every figure that shares the
loader, so already-published numbers cannot move underneath you.

⚠️ **A GUARD WRITTEN WITH `max()` OVER SIGNED VALUES STOPS GUARDING THE MOMENT
THE SIGN FLIPS — AND IT FAILS SILENTLY, IN THE NEW DIRECTION, ON THE FIRST RUN.**
Found Sep 26, 2026 in the `stair` profile, before it was ever run. The 30% duty
ceiling was enforced as `max(setpoints) > max_rpm`. For a reverse staircase that
expression evaluates to **−10**, which is below every conceivable ceiling, so
the check passes trivially and any speed at all goes through — in exactly the
direction being tested for the first time. **A limit on a magnitude must be
tested on a magnitude** (`max(abs(s))`). The structural fix is to keep sign out
of the quantities being bounded altogether: `--lo`/`--hi` are magnitudes and
`--dir` carries the sign, so there is only one place a sign error can live.
Generalises to every `min`/`max`/`>`/`<` guard that will one day see a negative
setpoint — current limits, position limits, and the rover's own speed clamps.

⚠️ **A FEEDFORWARD THAT IS SYMMETRIC BY CONSTRUCTION TURNS ITS INTEGRATOR INTO A
MEASUREMENT INSTRUMENT.** Sep 26, 2026. `velocity.c`'s friction feedforward
takes the sign of the *setpoint* and is therefore identical in both directions
by construction — it cannot absorb a plant that is not symmetric. That sounds
like a limitation and is actually the useful property: whatever the plant does
differently in reverse has **nowhere to land except the integrator**, so
`corr(Δ|out|, Δi) = 0.9986` is not a coincidence, it is arithmetic. The
integrator's resting value therefore reads out **the model's error, per
direction, per operating point** — which is how a separate reverse `ff_b` gets
measured instead of guessed. Whenever a term is fixed by construction, the term
downstream of it becomes the instrument for everything that term got wrong.

⚠️ **TWO RUNS AN HOUR APART ARE NOT AN A/B TEST — WHATEVER ELSE DRIFTED IS NOW
CALLED "DIRECTION".** Sep 26, 2026. The forward and reverse staircases are 67
minutes apart and not interleaved, so temperature, belt tension, and where the
carriage sits on a treadmill belt that travels the *other way* in reverse are
all aliased into the one variable that was deliberately changed. The difference
is real and the label is the honest one available, but it is **not a proven
cause**, and the figure says so in its own panel rather than in a footnote.
**An A/B/A ordering costs one extra run and separates them** — the return leg
either reproduces the first or it does not, and that is the whole answer.

⚠️ **A METRIC WITH A RATE LIMITER UPSTREAM OF IT MEASURES THE LIMITER, NOT THE
THING UNDER TEST — AND IT WILL NOT LOOK BROKEN.** Found Sep 26, 2026 in
`bench.py`'s `step_metrics`. `vel_slew` ships at 4 rpm/s, so a "0 → 10 rpm step"
is 2.5 s of ramp followed by whatever the loop does; anchoring rise and settling
at the command instant charged all of that to the controller. The reported
numbers were plausible, repeatable, and wrong. **The tell was invariance: rise
came back at 2.0 s no matter what Kp was**, because 2.0 s is 0.8 × 2.5 s — the
limiter's own 10→90% time and nothing to do with the loop. **A tuning number
that does not move when you move the gain is not a tuning number.** The fix is
an anchor, not a formula: measure the response from where the input stops
moving, keep the end-to-end figure alongside it because both are true and they
answer different questions, and report the limiter's floor next to the
measurement so the reader can see which one they are looking at. Generalises to
every layer with a rate limit below the thing being characterised — the drive
slew limiter, CAN transmit pacing, and the rover supervisor's own ramps will
each do this again.

⚠️ **AN OVERSHOOT IS A MAXIMUM, AND A MAXIMUM OF A NOISY SIGNAL IS A MEASUREMENT
OF THE NOISE.** Same session, exposed only after the anchor above was fixed.
The rig carries ~1.2 rpm sd of mechanical ripple, so a single peak sits 2–3 sd
above target *whatever the controller does* — the "14% overshoot" was **4 encoder
counts above target at 10 rpm and 4 encoder counts at 20 rpm**, which is the
signature of quantisation and ripple, not of a gain. Two rules follow.
**Compare a peak against the settled noise before attributing it** (`tail_sd_rpm`
and `overshoot_above_ripple`), and **bound the window it is searched over**, or
the number becomes a function of how long the run was rather than of the system.
The honest answer — *no overshoot is resolvable on this rig* — is less
actionable than "lower Kp" and is the one that does not damage the loop.

⚠️ **A FIGURE THAT DOCUMENTS A METRIC MUST CALL THE METRIC, NOT RE-IMPLEMENT
IT.** `figures/stepdata.py` imports `step_metrics` from `bench.py`. A second
implementation in the plotting script would have agreed on the day it was
written and drifted silently afterwards, and the drawing is exactly the artifact
nobody re-derives before trusting. Same discipline as `rigdata.py`'s
"transcribes nothing", one level up: the loader reads committed run
directories, and the analysis is imported from the tool that owns it.

⚠️ **PLOT THE DATA TO FIND THE BUGS IN THE ANALYSIS.** Two defects in the same
session were found by drawing, not by reading code. A hold window that ran to
end-of-file swallowed the `vel stop` ramp-down, and the resulting monotonic
collapse made the autocorrelation never go negative — so one of 21 holds
silently returned no period at all. And the autocorrelation's **global** maximum
picks the **second harmonic** whenever the fundamental's peak is the shorter one,
which inflated a spread from sd 0.19 to sd 2.09. **The fundamental is the first
strong local maximum after the correlation first goes negative**, never the
largest one.

⚠️ **ARMING A CONTROL LOOP INVALIDATES EVERY STOP SEQUENCE THAT ADDRESSES THE
LAYER BELOW IT.** Found Sep 26, 2026, in `node.safe_stop()`, which had sent
`drv duty 0` → `drv coast` → `drv disable` → `telem off` since the tool was
written. With the velocity loop armed, the board calls `drive_set_duty()` fifty
times a second from its own tick, so the first two commands are **overwritten
about 20 ms after they land** and only `drv disable` — the third step, cutting
nSLEEP — actually stopped anything. The sequence still worked, by accident, and
would have kept working right up until someone shortened it or reordered it.
The rule: **disarm the outermost active loop first, then walk down.** The
general form of the trap is worth stating separately, because it is not about
this one function: *whenever a new layer is added that drives an existing layer
autonomously, every command path into the lower layer silently becomes advisory.*
That is the same mechanism as task 21's watchdog problem seen from the other
side — there, the loop's continuous calls **kept alive** a watchdog that exists
to detect a dead host; here, they **overrode** a stop. Both are one layer
holding another's state open, and the countermeasure is the same in both: the
lower layer keeps its mechanism, and the layer that can defeat it carries its
own equivalent (`velocity.c` has its own setpoint watchdog, armed by default).
**Expect this again at CAN and again at the rover supervisor**, and check it
deliberately each time rather than waiting to find it.

⚠️ **A NEW MODULE DOES NOT INHERIT THE DISCIPLINE OF THE TICK IT HANGS OFF.**
`velocity.c` was committed with **not one `volatile`** on statics written from
the TIM6 ISR and read from thread context, while `encoder.c` and `drive.c` — the
two modules it sits between, and the two it was written by reading — both mark
theirs correctly. It was harmless only because nothing yet copied that state
out; the very next change, a telemetry snapshot, is what would have made it bite.
When adding a module to an existing ISR, the concurrency contract is a **checklist
item**, not something the surrounding code confers.

⚠️ **CURRENT REGULATION IS SILENT — a clamped start looks identical to a normal
one in telemetry.** Measured Sep 26, 2026: an un-ramped 0→29% duty step on the
loaded rig read **1582 mA against a trip programmed at 1579** and held the
DRV8874 in ITRIP regulation for ~40 ms, yet `flags` set **neither bit 4 (nFAULT)
nor bit 8 (ADC saturated)** at any point. ITRIP regulates, it does not fault. So
"no fault latched" is **not** evidence that a manoeuvre stayed inside its current
budget, and a reading sitting exactly on the programmed limit is the **clamp's**
value, not the demand's — the real peak is unknown and higher. Two rules follow:
treat *at-the-limit* as a distinct state to be detected by value, not by flag
(the same conclusion the Sep 21 hard-limiting note reached from the other side);
and sample fast enough that the first sample is not already clamped, since at
10 ms telemetry the entire rise was over inside one interval.

**A firmware reflash does NOT erase flash sector 7** — confirmed Sep 14, 2026 by
flashing and finding the stored `config` record intact. The toolchain
sector-erases only the regions it writes. This is load-bearing for W7: without
it, every firmware update would silently wipe each node's calibration.

⚠️ **…but ADDING A CONFIG KEY does discard it, and `CONFIG_VERSION` is not what
guards that.** `config.c` rejects any stored record whose `count` differs from
`CFG_KEY_COUNT` — correctly, since the record is read positionally and key 5
there is not key 5 here. The consequence is easy to miss because the header says
adding keys "is handled by the key count, not by this [version]", which is true
about the *version* and says nothing about what the board loses: the first boot
after a key is added reports `CONFIG_LOAD_VERSION` and **every calibrated value
reverts to its compiled default** — VDDA, R_IPROPI, the trip, the duty cap.
**Read `cfg` and keep the output before flashing a build that adds a key**, then
re-enter and `cfg save`. This will bite again at the PID gain bump, which adds
several keys at once. *(It cost nothing on Sep 26: `cfg` showed `slot 0/1024`
with no `*` markers, i.e. the bench board has never had `cfg save` run and every
live value was already the compiled default. That is luck, not a reason to skip
the check.)*

Short, generalised rules. Machine-specific detail belongs in that machine's
section; this is for things that will bite again somewhere else.

- **A driver's fault pin can be telling you about its own sleep state.** The
  DRV8874 holds nFAULT low the entire time nSLEEP is low, and `drive_faulted()`
  reads the pin directly — so a freshly reset board **always** reports a
  latched fault. A pre-flight gate written against that reading refuses every
  run on an artifact. **Wake the device, clear the latch, and only then decide
  whether a fault is real.** The general rule: a status line read while a
  peripheral is disabled is describing the disable, not the peripheral.
- **Write the raw transcript before you parse a byte of it.** Bench time is the
  expensive input; a parser is cheap and re-runnable. Logging raw-first means a
  parser bug costs an analysis and never a run — and it turns a failed run into
  a diagnosis, which is how the dead 12 V rail was identified from the record in
  one look on Sep 25, 2026 instead of by repeating the experiment.
- **A protocol's frame boundary needs testing against a simulator before the
  bench, not on it.** The console's prompt carries no newline, so a host that
  matched it as a buffer suffix broke as soon as an async line landed behind it.
  That class of bug is nearly invisible at a bench — it reads as flaky hardware
  — and took minutes to find against a pty that emulated the console.

- **A "hysteresis" that shrinks as the two passes get closer in time is
  thermal.** Sweeping duty up and then back down and reading the gap as
  hysteresis will instead measure the motor warming: on Sep 25, 2026 the
  descending-minus-ascending gap ran +0.94 rpm at the ends of the sweep (~11 min
  apart) down to +0.05 rpm in the middle (back-to-back). **The tell is that the
  gap tracks elapsed time rather than direction.** When hysteresis is the actual
  question, interleave or randomise the duty order so time and direction stop
  being the same axis.
- **Mounting changes the intercept; the slope belongs to the motor and the
  rail.** The same wheel hand-held and then clamped to a table gave slopes 0.830
  and 0.8327 rpm/duty% — 0.3% apart — while the intercept moved −0.96 → −1.52.
  So a bench figure is only comparable to another if the mounting matches, but
  **the slope is portable across mountings and the offset is not**. Record the
  mounting as a run condition, and when a plant number shifts, check the
  fixture before suspecting the motor.
- **Confirm an old result with a different method before trusting it, not by
  repeating it.** The CCW direction asymmetry measured +3.5% in Aug 2026 by
  hand, on a 9.35 V rail, on a bare shaft — and +3.49% in Sep 2026 by tool, on a
  12 V rail, on a clamped wheel. Agreement across three changed variables is
  evidence the effect is in the motor; a second run of the same procedure would
  only have confirmed the procedure.
- **If a fitted time constant depends on the fit window, the system is not
  first-order and the number is an artifact.** A one-pole fit to this loaded
  plant returned 0.290 s over 1 s, 0.496 s over 3 s and 0.724 s over 10 s,
  climbing monotonically and never settling — and 0.724 s was reported twice as
  "the plant's τ" before the window was varied. A genuine single pole gives the
  same answer at every horizon. **Refitting over shrinking windows costs one
  loop and is the cheapest test there is; run it before publishing any τ.** The
  fix here was a two-pole fit (0.219 s at 84%, 2.75 s at 16%), which improved
  the residual 5.1× and made an independent measurement agree.
- **A second route to the same quantity must differ in excitation, data AND
  estimator, or it is a repeat rather than a check.** τ was pinned by stacking
  42 step responses (0.219 ± 0.007 s) and, separately, by the steady-state lag
  of speed behind a 4.814 %/s ramp (0.207 ± 0.007 s) — **5.7% apart**. A step
  and a ramp fail differently, so agreement means something; two more step fits
  would only have confirmed the fitting code, and in fact the one-pole error
  reproduced perfectly across runs.
- **A threshold that moves when the rail moves is a voltage threshold —
  normalise before calling an old measurement stale.** Breakaway on the loaded
  rig read 12–14% duty at 9.35 V and 9–11% at 12.03 V. As duty those disagree by
  30%; as terminal voltage they are 1.216 V and 1.203 V, **1.0% apart**. **A
  breakaway, dropout or deadband duty is only valid for the rail it was measured
  on** — record the rail beside it, and convert before comparing.
- **Reversing the sweep direction separates a real shape from thermal drift, by
  the SIGN of the residual correlation.** Drift follows elapsed time, so
  reversing duty order flips it and the two passes' residuals
  **anti**-correlate; a genuine function of duty makes them **correlate**. Here
  +0.718 settled that the loaded band's curvature is real (and the same test run
  the other way identified the free-wheel asc/desc gap as warming). One extra
  pass answers a question that no amount of staring at a single sweep will.
- **Establish the repeatability floor first, then refuse to model anything
  smaller than it.** Two passes over the same band differed by ±0.174 rpm. A
  quadratic term improved the fit rms from 0.236 to 0.174 rpm — real-looking,
  and entirely inside the floor — so **the model stayed linear** and the 25%
  gain droop it described was carried as a design constraint instead. The same
  floor killed a point-to-point gain curve: ±0.174 rpm over a 2% step is
  ±0.123 rpm/% of apparent gain, i.e. most of the visible structure.
- **When one excitation has poor SNR, stack many of them — and stack the
  INTEGRATED quantity.** A single 2% duty step moved the wheel 1.6 rpm against
  0.5–0.7 rpm of rig ripple, under 3:1, and single-step fits scattered
  uselessly; 42 of them stacked beat the ripple down by √42. Build the ensemble
  in **distance, not velocity** — integration is a low-pass, so no
  differentiation noise ever enters the fit — and **normalise each event by its
  own final amplitude** so a varying gain across the range does not smear the
  average.
- **A rig that presses a load against a wheel gives honest friction and the
  wrong inertia.** The 1047 g on the treadmill belt is a *normal force*, not
  mass being accelerated, so every friction, current and steady-state number
  transfers to the rover and **the mechanical time constant does not** — the rig
  is 2.87× light against ~3 kg per wheel. **Ask of any load fixture which of the
  two it reproduces**, and carry the τ it gives as a lower bound.
- **Hold a calibration correction until the experiment that does not need it is
  finished.** VDDA 3325 and R_IPROPI 1465 were measured Sep 12 and deliberately
  not applied until Sep 20, because the plateau sweep's result was a *ratio*
  through the same two constants and therefore immune to them — while applying
  them mid-sweep would have moved every number underneath a measurement in
  progress, for no gain. **Check whether the pending correction cancels in the
  measurement you are about to take; if it does, the correction can wait and
  should.** The discipline costs nothing and removes a whole class of "did the
  numbers move because of the physics or because of me?"

- **Place a sampling trigger relative to the END of a window, not as a fraction
  of it.** A fraction (`start + 4/5 × ticks`) gives a different amount of
  settling time at every duty, so it is right at one operating point and quietly
  wrong everywhere else. `end − (aperture + guard)`, floored at
  `start + settle`, puts the aperture in settled signal at *any* window width
  and needs no re-tuning. It also makes the minimum usable window fall out as a
  sum rather than being guessed — and **a gate that rises when you measure the
  real settle time is a gate that was previously lying**, not a regression: the
  floor here went 4.3% → 14.5% duty, and everything in between had been
  reported without a warning.

- **Changing what a stored value MEANS is a version bump, not an edit.**
  `trip_ma` kept its name, range and units when `k = 3` landed, but a stored
  `3000` went from meaning "3 A" to meaning "3 A, enforced as 1 A". Bumping
  `CONFIG_VERSION` discards every stored record — including good calibration —
  and that is the correct trade: a silently re-interpreted current limit is
  worse than a re-entered one. **Put the newly measured constants into the
  DEFAULTS in the same change, so the board that loses its record comes back up
  calibrated rather than nominal.**

- **A datasheet settling time is a floor for the silicon, not for your board.**
  The DRV8874 quotes 1.6 µs `tDELAY` for IPROPI; the measured settle on this
  bench is **5.6 µs** — 3.5× — because the sense network rings and the datasheet
  number does not include it. **Measure the settle before trusting a
  synchronised sample, and set the gate from the measurement.** A guard band
  derived from the datasheet admitted readings that were pure ringing, and the
  console reported them with no warning.

- **When a sense resistor feeds both a comparator and an ADC, their ranges are
  locked together and you cannot optimise one without paying in the other.**
  Here the trip ceiling is always exactly **one third** of the ADC ceiling,
  whatever `R_IPROPI` is. Wanting a 4 A trip means accepting a 12 A full scale
  and surrendering two thirds of the resolution. Check this coupling at
  schematic time; by bench time it is a resistor swap with a fixed cost.

- **A reference measurement is only valid back-to-back with the point it
  calibrates.** A stalled motor's armature resistance depends on which
  commutator segments happen to be bridged, and a compliant hold lets the rotor
  creep — here a **±8% noise floor** on every stall reading, drifting across a
  session. One reference compared against forty minutes of later points cost a
  wasted run and a false hypothesis before it was caught.

- **Prove a limit by showing the reading ignores a change, not by matching a
  number.** Absolute agreement drowns in scatter near the regulation onset. The
  `k = 3` result was settled by taking two plateaus either side of a **7.6%**
  shift in demand and watching the plateau move **−2.3%** — a discriminator that
  works even when the noise floor is comparable to the effect.

- **A current mirror on the low-side FETs is blind to high-side decay, and blind
  again under deep regulation.** Both look like "the current vanished" and
  neither is. If a reading collapses orders of magnitude faster than `L/R`
  allows, suspect the sensing topology before the physics.

- **Chopping is audible.** A driver entering current regulation emits a
  high-pitched tone unrelated to the carrier frequency. Free, instant, and
  needs no console — worth listening for before reading any current number.

- **A conclusion drawn before a fix can survive the fix and go on being wrong.**
  "`isense_read_ma()` returns supply current" was a free-running-ADC artifact
  that phase-synchronised sampling fixed on Sep 16; it stayed on record as
  settled physics for another eight days, with a wrong cause attached. **When a
  commit changes how a measurement is taken, re-derive every conclusion that
  rested on the old one** — `git log -S` on the function that changed finds them.

- **A mode-select pin left floating is a selected mode, not an absent one — and
  a tri-level one may not select the same mode twice.** Multi-level config
  inputs self-bias through an internal divider (the DRV8874's PMODE: 156 kΩ to
  an internal 5 V over 44 kΩ to GND → ≈1.1 V, the middle band). Leaving it open
  therefore *picks* the middle option, and sits close enough to a threshold that
  a different power-up can latch a different mode — which turns one bug into an
  intermittent one. Strap every mode pin explicitly, including the one whose
  default you believe you want. This cost an MCU on Sep 16, 2026.
- **Probe the node where the modes DIFFER — which is usually the quiet one.**
  Confirming the DRV8874's control mode took eight days of wrong answers from
  rpm figures and input waveforms, and five minutes once both *outputs* were
  scoped: the switching output looks the same in either mode, while the idle one
  sits at ground in PWM mode and at the rail in independent half-bridge. Inputs
  cannot answer a question about how a part *interprets* its inputs. Write down
  the state the candidates disagree on, then go and measure that state.
- **Find out what a config pin is LATCHED on.** Many drivers sample mode pins
  once, at enable, rather than continuously — the DRV8874 latches PMODE on
  nSLEEP rising. A strap changed on a live board does nothing until the part is
  slept and woken, so "I changed it and nothing happened" is a false negative.
- **Ruling out one alternative does not confirm the remaining one unless the
  alternatives were enumerated first.** An rpm figure was used on Sep 14 to
  "confirm" PMODE was in PWM mode; it ruled out PH/EN and treated the rest as
  proven. Independent half-bridge gives the *same* average voltage under slow
  decay and was never on the list. When a measurement is used as proof, write
  down every state it has to discriminate, then check it against each.
- **Do not add an unmodelled passive to a net you are about to calibrate
  through.** A resistor the firmware does not know about is arithmetically
  indistinguishable from the unknown the calibration is trying to find — a
  pull-down on VREF divides it by exactly the kind of factor the plateau sweep
  exists to measure, so the result would silently absorb it and every later
  reading would inherit the error. Decide what is on a net *before* the
  experiment that characterises it, and write down what is fitted.
- **Know which part of the circuit a current sensor can physically see.**
  IPROPI mirrors only the low-side FETs, drain→source, so whether a reading
  exists at all depends on which side of the bridge the recirculation uses —
  low-side decay is sensed, high-side decay reads a clean, convincing zero. A
  plausible zero from a sensor that is blind to that path looks exactly like a
  real zero. Before trusting a current waveform, check the conduction path for
  every phase of the switching pattern, not just the driven one.
- **A GPIO that still reads high with its own internal pull-down enabled and
  every wire removed is a dead pad, not a wiring fault.** The order that proves
  it: confirm the peripheral registers are identical to a working sibling
  channel; drive the pin low and read IDR; reconfigure as an input with the
  internal pull-down and read IDR; then remove every external connection and
  repeat. Only the last step separates an external short from a blown ESD clamp
  to VDD. A hot MCU beside it is the 3V3 rail pouring through that clamp and out
  through the pin's own low-side transistor.
- **Size the ground return for the fault current, not the working current.** A
  single DuPont from breadboard PGND to MCU ground carried months of correct
  bench work, then failed the first time a control-mode fault turned a 13% duty
  command into ~74% of the rail. When the power-ground path is worse than the
  signal path, motor return current comes home through the *signal* wires and
  destroys GPIO pads. Motor-driver logic pins are typically rated to 5.75 V
  while a 3.3 V MCU clamps at VDD+0.3 — in that contest the MCU always loses.
- **When something is damaged, stop for the session.** Standing bench rule: the
  conditions that destroyed one part are still set up on the bench, and the next
  thing to go in is exposed to all of them. Diagnose and document, but do not
  rewire.

- **Crimp harness joints, never solder them.** Solder wicks up the strands and
  creates a hard-to-soft transition; all subsequent bending concentrates there
  until the copper work-hardens and cracks — inside insulation that still looks
  perfect. This is what killed the encoder VCC conductor on **two** motors
  (Aug 25, 2026) after cable extensions were soldered, and it is the same
  failure family as the four MCUs lost last semester. A rocker-bogie vibrates
  continuously, so this is not a marginal concern here.
  - Use a **ratcheting** crimper whose die matches the terminal *family* —
    insulated-barrel nests and open-barrel F-crimp dies are not
    interchangeable, and 22 AWG at the bottom of a "22–10 AWG" tool is where
    under-compression happens.
  - **Test destructively before committing to a batch:** crimp a scrap joint
    and pull it apart. The wire must break before the crimp releases.
  - **Stagger** splices along a bundle and strain-relieve both sides, so
    flexing happens in free wire rather than at a joint.
  - Avoid solder-ring heatshrink connectors: they combine an uninspectable
    joint with exactly the brittle transition above.
  - Adhesive-lined heatshrink does not bond well to **silicone** insulation.
    Self-amalgamating silicone tape does, and stays flexible.
  - Reference: NASA-STD-8739.4A, and the illustrated accept/reject criteria at
    https://workmanship.nasa.gov/lib/insp/2%20books/links/sections/407%20Splices.html
- **A pulled-up signal line sitting at a constant mid-rail voltage means an
  unpowered IC, not a stuck output.** A working open-drain output has only two
  states: pulled down hard (<0.4 V) or released (full rail). Anything in
  between is a resistive divider. Confirm by changing the supply voltage — if
  the *ratio* holds, it is passive silicon (ESD structures, bias resistors)
  and the chip has no power. This diagnosed the dead encoders in minutes after
  a scope had shown only "no pulses".
- **When a signal is missing, first prove which side of the MCU pin the fault
  is on.** Reading a GPIO's input register works even while the pin is in
  alternate-function mode, so the raw wire can be observed without disturbing
  the peripheral. `enc probe` does this for the encoder and splits "nothing
  reaches the MCU" from "the timer is not decoding what arrives" — two faults
  with identical symptoms and completely different fixes.
- **Verify wire colours against resistance, not against the datasheet.** On the
  drive motor the two leads reading ~2.5 Ω are unambiguously the winding;
  everything else follows from there. Colour codes vary by batch, and a
  swapped supply pair produces exactly the passive-divider signature above.
- **After any CubeMX regeneration, diff the USER CODE blocks — it can drop one
  silently.** Regenerating for ADC1 + DAC (Sep 12, 2026) emptied
  `USER CODE BEGIN TIM4_Init 2` and `TIM2_Init 2`, which between them held the
  only calls to `HAL_TIM_PWM_Start()` and `HAL_TIM_Encoder_Start()`. The cause
  is that the merge is keyed on marker position, and the newer CubeMX emits
  `HAL_TIM_MspPostInit()` on the *other* side of the markers than the old one
  did. There is **no warning and no build error** — the motor was simply dead
  with `nFAULT clear`, which reads like a hardware fault and is not one.
  - The fix that generalises: **put peripheral start calls in your own `.c`
    files**, not in USER CODE blocks. `drive_init()` and `encoder_init()` are
    ours; CubeMX cannot touch them.
  - The check that generalises: after regenerating, extract every
    `USER CODE BEGIN/END` block and diff the set against `HEAD`. A block that
    went from N bytes to 0 is the signature.
- **A `.ioc` that loads without error can still be silently invalid.** CubeMX
  accepts unknown *signal names* and just drops the peripheral — the symptom
  surfaces as a pin stuck red and unclickable in the GUI ("reset state"),
  because it stays `Locked=true` while pinned to nothing. Two rules that would
  have saved two sessions:
  - Signals with a `ShareableGroupName` in the IP-modes XML must be written as
    `P<pin>.Signal=<GroupName>` **plus** a paired `SH.<GroupName>.0=<real
    signal>,<mode>` and `SH.<GroupName>.ConfNb=1`. The pin's mode lives *inside*
    the `SH` entry, not in a separate `P<pin>.Mode=` line. Instance-specific
    exclusions are real: on the F446 it is `S_TIM2_CH1_ETR`, not `S_TIM2_CH1`,
    because CH1 and ETR share a pin.
  - **Never hand-edit an `.ioc` and trust it.** Let CubeMX save the file once
    and adopt its canonical form as the oracle. Validate headlessly with
    `STM32CubeMX -q <script>` (`config load …` / `project generate` / `exit`)
    and read `~/.stm32cubemx/STM32CubeMX.log` for
    `ImportTextPane … (OptionalMessage_ERROR)` lines — **the log is overwritten
    on every run**, so save it before the next invocation.
- **When a driver reports "awake, unfaulted, zero current", suspect the
  controller, not the driver.** A bridge that is enabled with no fault flag and
  passes *exactly* zero current is not failing — it is being correctly commanded
  to do nothing. `raw 0` on the current ADC is the same evidence twice. Reach
  for "is the timer actually running" before reaching for a scope.


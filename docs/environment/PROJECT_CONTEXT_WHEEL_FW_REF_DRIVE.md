# RobertUN wheel firmware — reference: drive motor, encoder, DRV8874 and plant

> Reference tier. Moved verbatim from `PROJECT_CONTEXT_WHEEL_FW.md` on 2026-09-26.
> Do not read whole: `grep -n '^#' <this file>` and read the section you need.
> Contents: motor, encoder, electricals, loaded rig, plant model, stop policy, DRV8874 wiring/straps/IPROPI calibration, bench tooling

## DRIVE MOTOR & DRIVER — hardware in hand (recorded Aug 16, 2026)

Steering is the MKS SERVO42C (section above). This is the *other* actuator: the
wheel drive, present on all six modules — corners run both, centers run drive
only.

### Motor
**CQRobot DC geared motor with encoder, 6 V / 12 V, 131.3:1 gearbox.**
Seven bought and already installed — six wheels plus one spare, matching the
seven WeAct F446 boards.

### Driver
**DRV8874** (HTSSOP-16 PWP carrier) — selected Aug 25, 2026, wired Sep 11. Specs,
connection map, straps and firmware implications are in the DRV8874 sections
below. It replaced a DRV8833 breakout; that history is in
`PROJECT_CONTEXT_WHEEL_FW_LOG.md`.

**Live consequence of the superseded DRV8833 work — do not lose it at HW1
layout.** The PDB branch rail is deliberately **13.0–13.5 V**, set to
pre-compensate branch-wire drop against the SERVO42C's 12 V floor; that decision
is sound and must not change. The DRV8874 spans 4.5–37 V, so the rail no longer
conflicts with the *driver* — but the **motor** is a 6 V/12 V unit, so the node
PCB still carries a second buck, now regulating the rail down to **12 V** to
protect the motor rather than the driver. **Feed the 3.3 V logic buck and the
motor buck independently from the 13 V rail — do not cascade 3.3 V off the motor
rail**, so motor noise never sits upstream of the MCU. Seven DRV8833 carriers
remain in the parts box, unused.

### Encoder — RESOLVED Aug 25, 2026: 8403.2 counts/rev

The vendor spec gives **64 CPR at the MOTOR shaft**, x4 quadrature (both edges
of both channels) — 16 pulses per channel per motor revolution. At the output
shaft, with the 131.3:1 gearbox:

```
64 × 131.3          = 8403.2  counts/rev   (x4 quadrature — what TIM2 counts)
16 × 131.3          = 2100.8  pulses/rev   (single channel)
2100.8 × 2          = 4201.6  edges/rev    (single channel, both edges)
```

**This supersedes the earlier 11 PPR estimate**, which gave 5777.2 counts/rev.
The real figure is 45% higher. Every W5 velocity and position figure uses
**8403.2**.

**This makes the W4 hand-rotation test diagnostic, not merely confirmatory.**
Turn the output shaft exactly one revolution and the count identifies the
decoding: ~8403 = x4 working; ~4202 = x2, one channel's edges only; ~2101 = x1.

**131.3:1 is itself a rounded number.** Real gear trains give awkward exact
ratios, so 8403.2 is not an integer and never will be. That matters at W9
(precision calibration), the same way the SERVO42C's own encoder resolution does.

Full derivation, speeds, count rates and the filter margin check are in
`docs/research/drive-motor/CQR37D12V64EN-M_Drive_Motor.md`.

### Motor electricals — MEASURED Aug 25, 2026 (two units)

Bench-measured, superseding the datasheet-derived 2.18 Ω previously recorded:

| | Motor #1 | Motor #2 |
|---|---|---|
| DC resistance (multimeter, **includes leads**) | 2.6 Ω | 2.8 Ω |
| Inductance @ 1 kHz (ZOYI ZT-MD1) | 1.702 mH | 1.690 mH |
| Rs @ 1 kHz | 3.947 Ω | 3.998 Ω |
| Q @ 1 kHz | 2.713 | 2.654 |
| No-load current @ 9.5 V | 154 mA | 155 mA |
| Stall: 4.1 A at 7.8 V (supply sagged from 9.5 V) | ⇒ **1.90 Ω** | same |

**Design figure: R ≈ 1.90 Ω — CONFIRMED Aug 25** by a clean low-current stall
(supply set to CC at 1 A, terminal voltage read, ~2 W of heating instead of
32 W): **2.042 V / 1.020 A = 2.00 Ω**, and **2.3 V / 1.00 A = 2.30 Ω** with the
leads reversed. The direction difference is not polarity — copper does not care
— it is rotor position: reversing torques the rotor the other way, so a
different set of armature coils lands on the brushes.

Combining the low- and high-current points separates winding from brush drop:

```
2.042 V = 1.020·R_w + V_brush        R_w     = 1.87 Ω
7.800 V = 4.100·R_w + V_brush   →    V_brush = 0.14 V

Stall at 9.5 V = (9.5 − 0.14) / 1.87 = 5.0 A
```

The small fixed brush drop is why the high-current test appeared to show lower
resistance — 0.14 V is a smaller fraction of 7.8 V than of 2.0 V. All methods
converge on **5.0 A at 9.5 V**, inside the DRV8874's 6 A peak.

**Do NOT use the LCR meter's Rs for DC calculations.** At 1 kHz, Rs includes
core loss — eddy currents and hysteresis in the rotor iron — plus skin and
proximity effects, so it reads ~58 % above R<sub>DC</sub>. Both meters are
internally consistent (Q = ωL/Rs checks out on both motors).

**Rail decision: 9.5 V**, via the second buck. Stall is then **5.0 A**, inside
the DRV8874's 6 A peak — simpler than feeding 13.5 V direct and relying on
current regulation to hold back a 7.1 A stall.

| Rail | Stall @ 1.90 Ω | No-load speed |
|---|---|---|
| **9.5 V (chosen)** | **5.0 A** | **60 rpm** |
| 12 V | 6.3 A | 76 rpm |
| 13.5 V | 7.1 A | 85.5 rpm |

**Starting from rest draws stall current momentarily**, because back-EMF is
zero at t = 0 — so a step from 0 to full duty is a 5 A event in normal
operation, not only during a fault.

### Loaded wheel test rig — built Sep 11, 2026

A static base was designed and built that holds one wheel against a
**treadmill-style belt**, so the wheel can be driven at speed **while carrying
real weight**. Nothing else on the bench can do this: every motor and plant
figure on record — the 0.672 rpm/duty slope, the 2.6% deadband, the 154 mA
no-load draw — was taken on a **free shaft**, which is the easiest load the
motor will ever see.

**Why this matters more than it sounds.** A velocity PID tuned on a free shaft
does not transfer to a loaded wheel. Load changes the three things the loop is
built around at once: the deadband widens (more torque is needed to break
static friction), the duty-to-rpm slope drops, and the mechanical time constant
lengthens, so gains that are crisp unloaded turn sluggish or oscillatory under
weight. The rig means W5 can tune against the load the rover will actually
carry, and can test the one case bench tuning normally misses — a **load step**,
which is what a wheel hitting a rock or dropping off a ledge looks like to the
controller.

**Re-measure on the rig, at the weight the rover will actually run:** the
duty-to-rpm line and its deadband, the current draw at each operating point
(this is the honest input to HW4's PDB branch sizing, better than either the
resistance calculation or the stall test), and the response to a step change in
both command and load. Keep the free-shaft numbers below for comparison rather
than overwriting them — the difference between the two is itself the useful
figure, and it is what tells you how much margin the gain set has.

✅ **DONE Sep 25, 2026 — see task 17's loaded-rig block for the full result.**
At **1047 g** (wheel + aluminium carriage) on the belt, 12 V rail:
`rpm = 0.7993 d − 2.420` over 11–29%, current 266→313 mA at +4.4 mA/%, and a
two-pole step response, **τ_fast 0.219 s + τ_slow 2.75 s**. The load turned out
to cost only **4.3% of the free-wheel slope**. The load-step case is still
untested.

⚠️ **THE RIG APPLIES A NORMAL FORCE, NOT AN INERTIA — and that limits which of
its numbers transfer.** The 1047 g presses the wheel onto the belt; it is not
mass being accelerated. The three effects predicted above are therefore not
equally reproduced: **the widened deadband and the reduced slope are honest**
(they come from friction, which is what a normal force produces), but **the
lengthened mechanical time constant is NOT** — the rover carries ~3 kg per
wheel against the rig's effective load, making the rig **2.87× light**. τ
measured here is a **lower bound**, and must be re-taken on the vehicle before
gains are finalised.

### Plant model — measured Aug 26, 2026 (free shaft, no load, slow decay)

⚠️ **Every figure in this section was measured at a 9.35 V motor terminal
voltage. The rail was raised to 12 V on Sep 19, 2026, so the duty→speed
line, the deadband and the current figures below are all stale** — they
describe a rail the bench no longer runs. R, L, Ke and counts/rev carry
over unchanged; the mappings do not. **Re-measured Sep 23 and re-taken by tool
Sep 25 — use the 12 V free-wheel model further down, not these numbers.** The
one thing here that survived the re-take is the **direction asymmetry**: +3.5%
measured here, +3.49% measured Sep 25 on a different rail by a different
method.

Console-driven, motor unloaded, 9.5 V rail nominal. Averaged over both
directions:

| duty | CW rpm | CCW rpm |
|---|---|---|
| 20 | 11.78 | −11.78 |
| 40 | 24.63 | −25.35 |
| 60 | 37.84 | −39.27 |
| 80 | 51.05 | −52.84 |
| 100 | 64.26 | −66.76 |

**Least-squares fit: `rpm = 0.672 × duty% − 1.8`, deadband ≈ 2.6% duty.**

Per-step increments are 13.21 / 13.57 / 13.39 / 13.56 rpm per 20% — varying by
only ±1.5% over the whole range. **The plant is linear enough that W5 needs no
gain scheduling**, which is the single most useful thing this test established.

**Sign convention: positive duty → CW → positive encoder counts.** Confirmed at
every point. W5's PID sign follows from this; if it were ever inverted the fix
is to swap the MOTOR leads, not the encoder.

**Direction asymmetry ≈ 3.5%**, CCW consistently faster above 20% duty. Normal
for a brushed motor — usually brush timing — and absorbed by integral action.
Recorded so it is not chased as a fault later.

**Velocity quantisation confirmed.** Every reading is an exact multiple of the
designed quantum: 1000 Hz / 20 ticks = 50 windows/s, and 50 × 60 / 8403.2 =
0.3570 rpm per count. 11.78 = 33 counts, 65.51 = 183. The encoder velocity path
behaves exactly as `encoder.h` predicts.

### Stopping: coast or brake — decided Aug 26, 2026

**Default stop is COAST** (`duty 0`, both inputs low). Brake (both inputs high)
is reachable only by calling `drive_brake()` deliberately.

The two are electrically very different:

- **Coast** — outputs go Hi-Z, the motor freewheels. The remaining inductive
  energy is 1/2·L·I² = **19 µJ** at 155 mA, which decays through the body
  diodes. A zero-current event at any speed.
- **Brake** — both low-side FETs on, winding shorted through them. Back-EMF
  then drives `I = E / R_winding`.

From the measured terminal voltage and no-load current,
`E = 9.35 − 0.155 × 1.9 = 9.06 V` at 65.5 rpm, so **Ke ≈ 0.138 V/rpm** at the
output shaft:

| Speed | Back-EMF | Brake current |
|---|---|---|
| 65 rpm | 8.98 V | **4.73 A** |
| 50 rpm | 6.91 V | **3.64 A** |
| 40 rpm | 5.53 V | 2.91 A |
| 20 rpm | 2.76 V | 1.45 A |
| 10 rpm | 1.38 V | 0.73 A |

**Braking from full speed draws essentially stall current**, above the DRV8833
carrier's 4 A paralleled peak, and dissipates it in the low-side FETs.
**On the interim carrier, brake only below ~40 rpm.**

**The ground-loop exposure differs, and not in the intuitive direction.** The
two cases use different current loops:

```
DRIVING:  PDB(+) -> driver -> motor -> driver -> PDB(-)
          the branch return wire is INSIDE this loop
BRAKING:  motor -> OUT1 -> LS FET -> PGND -> LS FET -> OUT2 -> motor
          entirely LOCAL to the driver board
```

Brake current never traverses the branch return, so interrupting that wire
mid-brake does not produce the destructive mechanism documented under *Ground
loops* — there is no path through the MCU for it to divert into. Driving
current is the exposed case.

The risk **relocates** rather than disappearing: the brake loop runs through
the driver's PGND and the local star point, so 4.7 A sits next to the MCU's
ground reference. This is a direct argument for being fussy about that one
joint on the node PCB.

**Emergency stop should be COAST, not brake.** Counter-intuitive, but braking
creates a 4.7 A event at exactly the wrong moment — and if the emergency is a
wiring fault, that is the worst current to have looking for a path. The
131.3:1 spur reduction is stiff enough that coasting holds the rover on any
slope it can climb.

**W5 policy, not driver code:** the control layer should ramp duty down before
braking, keeping the brake current inside the driver. `drive.c` deliberately
implements no policy.

**Revisit once the DRV8874 arrives** — its current regulation caps brake
current at the VREF setting, so braking from speed stops being a high-current
event and the 40 rpm threshold can be lifted.

### The inductance validates the 20 kHz PWM choice

With L = 1.70 mH and R = 1.90 Ω:

```
Electrical time constant  τ = L/R = 0.90 ms
PWM period at 20 kHz          T = 0.05 ms      → τ/T = 17.9
Ripple at 50 % duty   Δi = V·D·(1−D)·T/L ≈ 99 mA p-p
```

~100 mA of ripple against amp-level working currents is negligible. At 1 kHz it
would have been ~2 A p-p. It also means the DRV8874's fixed 25 µs
current-regulation off-time will hold a smooth setpoint rather than chattering.

**Loop-rate consequence:** τ<sub>elec</sub> = 0.90 ms is *faster* than a 1 kHz
control tick. A velocity loop at 1 kHz on TIM6 is fine — the mechanical time
constant through a 131.3:1 gearbox is far slower. But **a current inner loop at
1 kHz would be sampling slower than the dynamics it controls**; run it at
5–10 kHz or synchronised to the PWM. Cascade the two at different rates.

### Motor-to-motor matching — one gain set should fit all six

Inductance agrees to 0.7 %, Rs to 1.3 %, no-load current to 0.6 %, stall to
1.3 %. That is tight for inexpensive gearmotors and means **one set of PID gains
should transfer across all six wheels** — which is what the one-binary /
DIP-switch architecture already assumes. The DC-resistance spread (7.7 %) is
the outlier, and is the reading most sensitive to brush/commutator position and
lead resistance. Worth confirming against a third motor before relying on it.

### ⚠️ Connector hazard — replace the motor's DuPont crimps

The motor ships with 2.54 mm DuPont *female* crimps on its 22 AWG power leads.
Those contacts are rated 1–3 A against a 4.1 A stall, and they are friction-fit
and non-locking — precisely the "intermittent power-ground contact under
vibration" mechanism documented in **POWER DISTRIBUTION & GROUNDING → Ground
loops**, and the leading explanation for last semester's four destroyed MCUs.
Rocker-bogie articulation means continuous vibration at every corner.

**Replace the crimps on the red/black power pair** with something retained —
JST VH, screw terminal, or soldered and heat-shrunk. The four encoder leads
carry milliamps and can stay DuPont.

Note also the leads are only **20 cm**, so the driver must sit within 20 cm of
the motor.

### DRV8874 — the selected driver (decided Aug 25, 2026)

Full four-way comparison against DRV8876, DRV8871 and DRV8833 in
`docs/research/drive-motor/Motor_Driver_Selection.md`.

| | DRV8874 |
|---|---|
| VM | 4.5–37 V — **takes the 13–13.5 V branch rail directly** |
| Peak / realistic continuous | 6 A / ~3 A |
| R<sub>DS(on)</sub> HS+LS | 200 mΩ |
| Current regulation | **Yes, VREF-programmable** |
| Current feedback | **Yes — IPROPI, 450 µA/A** |
| nFAULT / nSLEEP | Both present; nSLEEP has a 100 kΩ internal pulldown |
| Package | HTSSOP-16 (PWP) |

**Bought Aug 25: 8 × carrier board + 10 × bare IC. Arrival ~Sep 15, 2026.**

**What it changes:**
- **The second buck disappears.** 4.5–37 V covers the branch rail. 13.5 V is
  above the motor's 12 V rating, but a duty cap of 12/13.5 = **89 %** gives
  exactly 12 V average at no cost.
- **W5 can have a current inner loop after all** — previously ruled out.
- **IPROPI measures real drive current**, closing the open W4 task that HW4's
  PDB branch sizing waits on.
- **Firmware carries over unchanged** if PMODE is strapped for PWM mode: IN1/IN2
  have the same truth table as the DRV8833, and t<sub>WAKE</sub> is 1 ms on both.
- **One new MCU pin: PA2 (`ADC1_IN2`) for IPROPI.** `R_IPROPI = 2.2 kΩ` gives
  0.99 V/A → 2.97 V at 3 A, near-perfect full scale for a 3.3 V ADC.
  Optionally **PA4 (`DAC1_OUT`) drives VREF** for a software-programmable
  current limit; PA5/DAC2 stays reserved for SPI1_SCK.

**DRV8876 is the second source** — pin-identical in PWP, and stocked ~8× deeper.
**But A<sub>IPROPI</sub> differs: 450 µA/A on the DRV8874, 1000 µA/A on the
DRV8876.** Electrically drop-in, but the sense resistor and the firmware
calibration constant both change by 2.22×. The swap is not transparent.

### DRV8874 — CONNECTION MAP (bench wiring, Sep 11, 2026)

**The single biggest change is that the two bridges stop being paralleled.**
The DRV8833 has two half-bridge pairs and the carrier had IN1+IN3 and IN2+IN4
tied together to share current. The DRV8874 is **one full bridge rated 6 A on
its own**, so PB6 goes to one input and PB7 to the other, full stop. Any
leftover jumper that parallels inputs is now wrong.

| STM32 pin | Signal | DRV8874 pin | Direction | Notes |
|---|---|---|---|---|
| PB6 | DRV_PWM_A | **1 — EN/IN1** | MCU → driver | TIM4_CH1, 20 kHz |
| PB7 | DRV_PWM_B | **2 — PH/IN2** | MCU → driver | TIM4_CH2, 20 kHz |
| PB5 | DRV_nSLEEP | **3 — nSLEEP** | MCU → driver | low = disabled; 100 kΩ internal pulldown |
| PB0 | DRV_nFAULT | **4 — nFAULT** | driver → MCU | open-drain, active low, **needs a 10 kΩ pull-up to 3V3**. On PB12 until Oct 4, 2026 (PB12 is now DIP_SW_0) |
| **PA2** | **DRV_IPROPI** | **6 — IPROPI** | driver → MCU | **new.** ADC1_IN2; **R_IPROPI now 1.474 kΩ** (2.0k∥5.6k) → 0.6632 V/A |
| **PA4** | **DRV_VREF** | **5 — VREF** | MCU → driver | **new.** DAC1_OUT1. Carrier's 10 kΩ to nSLEEP **removed Sep 12** |
| — | PMODE | 16 — PMODE | strap | **PWM mode = logic HIGH. Fit 10 kΩ to 3V3.** Open = Hi-Z = independent half-bridge, NOT "unset" (Sep 16) |
| — | IMODE | 7 — IMODE | strap | **carrier fits 20 kΩ to GND** — decode against the datasheet table |
| GND | common | 9 PGND / 15 GND | — | one ground reference, star point at the supply |

**Power and motor:** VM (11) to the motor rail, OUT1 (8) and OUT2 (10) to the
two motor leads — **the pair that reads ~1.9 Ω**, per the colour-code rule in
KEY LEARNINGS. The encoder's own supply and its A/B pair do not touch the
driver at all; they go to the MCU side as before.

**Three things to confirm on the physical carrier before wiring**, all of them
open items from the selection doc:

1. ~~Which PMODE strap selects PWM (IN1/IN2) mode.~~ **ANSWERED Sep 16, 2026
   from SLVSF66A Table 2: PWM mode is PMODE = logic HIGH.** Fit 10 kΩ to 3V3
   (not 100 kΩ — see the Sep 16 log). Logic low is PH/EN and **Hi-Z, which is
   what an open pin self-biases to, is independent half-bridge**. The pin was
   left open from Sep 11 to Sep 16 and the driver therefore ran in independent
   half-bridge with internal current regulation disabled. The mode is latched
   on nSLEEP rising, so the strap takes effect only after `drv disable` →
   `drv enable` or a power cycle.
2. ~~Whether the carrier already populates R_IPROPI, and at what value.~~
   **ANSWERED Sep 12, 2026: 2.48 kΩ, fitted.** The selection doc's 2.2 kΩ was
   an assumption and is superseded — see the scaling block below.
3. ~~Whether the carrier populates the nFAULT pull-up.~~ **ANSWERED Sep 14,
   2026: fitted and working.** Shorted to GND, the boot line reported
   `nFAULT=ASSERTED`; released, it reported `clear`. A clean high on release is
   the part that proves the pull-up exists — a missing one gives an
   inconsistent float, not a steady high.
   **Still unproven: that the driver itself pulls nFAULT low on a real fault.**
   This test exercised the MCU side only. Cheapest honest check is UVLO — with
   nSLEEP high, drop VM below ~4.5 V and the driver should assert. Fold it into
   the 12 V rail work, when the bench supply is already in hand.

**What does NOT change:** `drive.c` needs no edit. nSLEEP polarity, the 1 ms
wake, the IN1/IN2 truth table and the open-drain active-low nFAULT are the same
on both parts — that is exactly what the module was written driver-agnostic
for. What does change is policy, not plumbing: the DRV8833's `drive_set_limit()`
bench cap existed because a 4 A carrier faced a 5.0 A stall. A 6 A part at the
9.5 V rail does not need it.

### DRV8874 carrier — the three straps, and the two that were changed

Measured as-shipped on Sep 12, 2026, then **the bench carrier was modified the
same day**. Seven spare carriers remain stock.

| Strap | As shipped | On the bench carrier now |
|---|---|---|
| **PMODE** | **not populated — pin left OPEN** | **10 kΩ to 3V3 — FITTED and CONFIRMED IN PWM MODE Sep 19** (OUT1/OUT2 decay state) |
| nSLEEP → VREF | 10 kΩ | **REMOVED** — VREF driven by PA4/DAC1_OUT1. **Pad confirmed bare Sep 19; fit nothing in its place** (see below) |
| IMODE → GND | 20 kΩ | unchanged — **still not decoded** |
| R_IPROPI → GND | 2.48 kΩ | **1.474 kΩ** (2.0 kΩ ∥ 5.6 kΩ) |

**Why:** the motor stalls at 5.0 A on the 9.5 V rail, and the stock carrier
could neither measure that (2.957 A ceiling) nor permit it (2.957 A trip). The
modification is small, reversible, and off the spares.

**The scaling as modified:**

```
scale       = A_IPROPI × R_IPROPI = 450 µA/A × 1465 Ω = 0.6593 V/A
ADC ceiling = 3.325 V / 0.6593 V/A                    = 5.044 A
LSB         = 5044 / 4096                             = 1.231 mA
                                                        (811.7 counts/A)

integer form, no float:   I_mA = raw × 5044 / 4096
```

**These are the MEASURED constants, applied Sep 20, 2026** — R_IPROPI **1465 Ω**
across the fitted pair (against a 1474 Ω nominal) and VDDA **3325 mV** on this
board. They were measured Sep 12 and deliberately held back until the plateau
sweep was finished, because that sweep's result was a ratio through both
constants and immune to them. On the old nominals the same board computes
0.6632 V/A, 4.975 A and 1.215 mA, so **every current logged before Sep 20 reads
~1.4 % low**.

Both are **per-board** figures. A second carrier gets metered and `cfg`-set, not
handed these — a 1 % error in R_IPROPI is a 1 % error in every current ever
logged, and `cfg r_ipropi <ohms>` changes it live with no rebuild.

**Removing the 10 kΩ undid the coupling, which was the whole point.** On the
stock carrier VREF followed nSLEEP, so the trip point and the ADC full scale
were the same number and could not be moved apart. They are now independent:

- **R_IPROPI alone sets the CEILING** — 5.044 A, fixed in hardware.
- **VREF sets the TRIP**, anywhere from 0 up to **a third** of that ceiling, in
  software — the DRV8874 compares against `VREF/3` (`k = 3`, measured Sep 20).
- One DAC code moves the trip by **0.410 mA — one third of an ADC LSB**, because
  both converters are 12 bits across the same 3.325 V through the same resistor
  but the comparator sees VREF divided by three. The limit is finer-grained than
  the measurement can read back.

**The cost is that the fail-safe is gone.** The 10 kΩ guaranteed VREF could
never be wrong while nSLEEP was high; they moved together. Now **VREF must be
set before nSLEEP rises**, every time, including after any reset. `isense_init()`
does this at boot. If the DAC is somehow not running the trip is 0 A and the
motor will not turn — the safe direction to fail, and deliberate.

**The DAC output buffer costs the top of the range.** Buffered, the F446 swings
roughly 0.2 V to VDDA−0.2 V:

| buffer | VREF range | trip range |
|---|---|---|
| on (default) | 0.200–3.100 V | **0.30–4.67 A** |
| off | 0.000–3.300 V | 0.00–4.98 A |

4.67 A is *below* the 4.92 A the motor draws stalled at the measured 9.35 V
motor terminal, so with the buffer on a genuine stall regulates rather than
being measured. `drv trip buf off` reclaims it — but the unbuffered DAC is
high-impedance, so **put a meter on PA4 and confirm VREF reads what was
commanded** before trusting it.

**Default trip is 3000 mA**, matching what the stock carrier enforced, so
lifting the resistor did not quietly make anything more dangerous. Full
capacity is an explicit act: `drv trip 4600`.

**Considered and not adopted Sep 19 — a pull-down on VREF.** Fit nothing on this
net. A 100 kΩ to GND would divide VREF by a factor the firmware does not model,
making it **indistinguishable from the internal divider the plateau sweep below
exists to measure** — the k that came back would be partly the resistor's, and
would then be baked into every current figure afterwards. It also loads the
unbuffered DAC, which `drv trip buf off` needs. The one real argument for it is
that VREF now *floats* if the DAC peripheral is not running (the PMODE mistake on
an analog pin); `isense_init()` sets VREF at boot, so the hole is narrow, and
**1 MΩ** would buy the fail-safe at a tenth of the loading if it is ever wanted.
Full reasoning in the LOG file.

**SETTLED Sep 20, 2026 — VREF goes through an internal ÷3.** The sweep was run:
motor stalled, 20% duty, 12.0 V rail, reading the settled tail of a `drv iscan`
rather than `drv current`. Plateaus of 933 and 911 mA against a commanded
`drv trip 2997`, holding still while the unregulated demand moved 7.6% between
runs — a reading that ignores demand is a limit, not a measurement. `k = 1` and
`k = 2` are refuted outright: a trip of 1997 regulated hard where neither
predicts any regulation at all.

> **The DRV8874 compares the instantaneous IPROPI voltage against `VREF / 3`.**
> **Every `drv trip` and `cfg trip_ma` on record is therefore 3× too high.**

- The console's printed range **301–4673 mA was really ~100–1558 mA**, and the
  **3000 mA boot default was really 1000 mA** — which is what had actually been
  protecting the bench. **Fixed Sep 20** (task 20): the printed range is now
  **~101–1580 mA** on the measured constants, and the boot default is written as
  the 1000 mA it always physically was.
- The **regulated average sits at ~92% of the peak limit** (933/911 mA against a
  nominal 999), the shortfall being chopping ripple.
- **The maximum trip is always exactly one third of the ADC ceiling, whatever
  R_IPROPI is** — comparator and ADC read the same resistor. A 4 A trip needs a
  ~12 A ceiling (R_IPROPI ≈ 600 Ω) and surrenders two thirds of the ADC
  resolution to get it. A **hardware** trade for HW1, not a firmware fix. The
  cold stall at 12 V is ~6.3 A and is unreachable as a trip by a factor of four.
- **IPROPI goes blind while regulation is active.** Push the limit ~90% below
  demand and the tail collapses to near zero far faster than L/R decay (τ ≈
  0.9 ms) allows. **W5 cannot measure current while the loop is hard-limiting.**
- **Regulation is audible** — a high-pitched tone at exactly the points the
  electrical data says the driver is chopping. Bench indicator, no console
  needed.

The fix belonged in firmware (task 20), not on the bench, and **landed Sep 20**:
`drv trip` multiplies the requested mA by `k` before computing the DAC code, and
the printed range follows. `k` lives in config as `cfg vref_div` (1..4, default
3) so a second-source part with a different divider is a console command rather
than a rebuild.

### IPROPI — CALIBRATED Sep 20, 2026: the reading is MOTOR current

`I_IPROPI = I_OUT × 450 µA/A`, and R_IPROPI converts that to a voltage the ADC
reads. At the modified **1.465 kΩ** measured (2.0k ∥ 5.6k, 1.474 kΩ nominal):
**0.6593 V/A**, ADC ceiling **5.044 A**, **1.231 mA per count** (811.7 counts/A).

> **On the synchronised path, `drv current` returns MOTOR current.** The sample
> is taken inside the drive window, where IPROPI mirrors the conducting low-side
> FET directly. Divide by nothing.

**Calibrated against physics, Sep 20** — the first end-to-end check the project
has had. Stalled (no back-EMF, so current is pure Ohm's law), 20% duty, 12.0 V
at VM, trip parked above demand: predicted `0.20 × 12.0 / 1.90` = **1263 mA**,
measured **1290 mA — 2%**. IPROPI, R_IPROPI, `ISENSE_VDDA_MV`, the ADC and
`isense_raw_to_ma()` all agree with an independent physical prediction.

**Zero offset is a non-issue.** `drv zero` with nSLEEP low reads **0 counts over
256 samples** — legitimate: the sleeping mirror is high-Z, R_IPROPI pulls the
input to ground, and there is no negative rail to dither below. The *awake*
pedestal, which `drv zero` structurally cannot reach, measures **3–5 counts
(≈4 mA)** flat across the period via an `iscan` at `duty 0`. No change needed.

**⚠️ RETRACTED — the Sep 12 "the reading is SUPPLY current" conclusion and its
stated cause are both wrong.** Recorded because the wrong version was
load-bearing for a week.
- The cause on record was the carrier's 20 kΩ IMODE strap "blanking the mirror
  during recirculation." It was never IMODE. The driver was in **independent
  half-bridge**, so decay was **high-side**, and IPROPI — which mirrors only the
  low-side FETs, drain→source — is physically blind to it.
- The conclusion was a **sampling artifact that a later commit already fixed**.
  Phase-synchronised sampling did not exist until `9187dc9` (Sep 16); before it
  the ADC free-ran across the whole period, and a free-running average of a
  signal that is zero for `1 − D` of it is `I_motor × D` by construction. The
  **Sep 16 trigger work**, not the Sep 19 PMODE fix, turned the reading into
  motor current.
- **The quadratic plateau formula that followed from it — `trip² × R_motor / Vm`
  — describes only the unsynchronised fallback path.** On the synchronised path
  the plateau is `trip / k` directly. Applying the quadratic to the Sep 20 sweep
  would have produced a badly wrong `k`.

**Three bugs in the synchronised sampler, found Sep 20 — ALL FIXED the same
day (task 20) and ✅ BENCH-VERIFIED Sep 21.**
- **IPROPI needs 5.6 µs (500 ticks) to settle, not the datasheet's 1.6 µs
  `tDELAY`.** At 20% duty the drive edge is at tick 3600 and the reading only
  goes flat from tick 4100, ringing through 1757 / 212 / 1659 / 1275 / 994 on
  the way. Probably the sense network, not the mirror.
- **`ISENSE_SYNC_MIN_TICKS` was 192; it needed to be ~600.** At 192 the gate
  admitted a reading at 4.27% duty, where the whole drive window is shorter than
  the settling time. **Nothing below ~13–15% duty is valid**, and the console
  warned about none of it. → now `DRIVE_PHASE_MIN_TICKS` = **652** in `drive.h`
  (settle 500 + aperture 112 + guard 40) = **14.5% duty**.
- **`place_trigger()` sampled the window midpoint — the contaminated half.** All
  the ringing is at the leading edge. At 20% duty the midpoint read 994 against
  a settled 1143, **13% low**. The Sep 15 conclusion that trigger placement was
  correct is withdrawn. → now **end-relative**, `end − (aperture + guard)`
  floored at `start + settle`, rather than the "75–85% through the window"
  fraction first proposed: a fraction gives a different settle allowance at every
  duty, an end-relative rule gives the same one at all of them.

Casualties measured the same day, both garbage: `drv current` gave **87 mA at
13% duty** free-running and **211 mA at 8% duty** stalled (against 505 mA
predicted). Both were taken below the 14.5% floor that now refuses them.

**Verified on the bench Sep 21, at 12.0 V, 20% duty, shaft stalled:
`drv current` = 1268 mA against `D × Vm / R_motor` = 1263 mA, +0.4%.** The old
midpoint tick (4050) read 808 raw in the same trace where `drv current` read
1030 — a **27% correction** at one operating point, landing on physics.
`drv current` is now the instrument to quote; the iscan tail is the
cross-check, not the source.

**Stall readings carry a ±8% rotor-position noise floor.** The unregulated
reference moved 1062 → 1143 raw across one session, and the encoder crept tens
of counts *within* single runs. The hold is compliant, the motor twists against
it, and at stall the armature resistance depends on which commutator segments
are bridged. **Take a reference back-to-back with every point it calibrates** —
one reference compared against forty minutes of later points is worthless.

**ADC config is right and needs no change.** `ADC_SAMPLETIME_28CYCLES`: at
PCLK2/4 = 22.5 MHz a conversion is 28+12 = 40 cycles = **1.78 µs**, ~2.5× what a
1.474 kΩ source needs to settle to 12 bits, and short enough to sit inside the
drive window. 480 cycles (21.9 µs) would have straddled half the PWM period and
made synchronised sampling impossible without returning to CubeMX.

**Tick 0 in an `iscan` is always a non-measurement** — CCR4 = 0 leaves TIM4_CH4
permanently high, no compare edge is generated, `adc_wait_eoc()` times out and
`sync_burst()` returns 0. Documented in `drive.c`, but it prints as though it
were data. Cosmetic fix queued in task 20.

#### Decay-phase sample below 14.5% — implemented Sep 26, 2026

- **Source selection** is in `drive.c` `place_trigger()`, geometry only:
  drive phase if it is ≥ 652 ticks (unchanged); otherwise, in **slow decay
  only**, the brake phase `[0, ccr)` with `last = ccr − 152`,
  `first = max(1000, last − 500)` (`DRIVE_DECAY_SETTLE_TICKS` 1000,
  `DRIVE_DECAY_SPAN_TICKS` 500); otherwise none. Accessors
  `drive_sense_kind/first/last()`. Fast decay, coast, brake and duty 0 → none.
- `isense_read_sync_avg()` scales a decay reading ×1000/`isense_dk`;
  `isense_sync_ready()` refuses below `isense_dmin`. New cfg keys
  **`isense_dk` 690** (400–1000) and **`isense_dmin` 60 o/oo** (30–145);
  adding them discards the stored cfg record.
- `drv current` names the phase and window; NOT SYNCHRONISED gives the reason.
  Telemetry flag **0x20** = brake-phase sample (`Telem.decay` in `node.py`).
- **Bench, 12.0 V:** at 10% the brake window is 3398..3898; `drv current` agrees
  with an iscan over that window ×1000/690 within 1.2% (stalled); 20% still
  drive phase, within 2% of the iscan tail. Stalled A/B/A at 20/10/20 gave
  +6.9% once, then void (refs 13% apart) — stall noise, not code.
- **Valid at stall only.** Free shaft, 20 → 10% in 0.5% steps, 1 min settle,
  >250 reads/step: brake/drive = **0.40–0.46** (not 0.690); telemetry
  **313 → 189 mA across the 14.5% switch (−40%)**. Brake reading flat
  102–110 raw over the whole sweep; drive reading rises 221 → 260 raw as its
  window shortens. Interpretation (not proven): while turning the current
  ripples inside the period; the drive sample sits near the peak, the brake
  sample near the trough, neither is the mean. Use the decay reading for
  stall/protection, not as a torque measure while turning.
- **Open:** a supply-side DMM reference (avg Isup ≈ D × mean drive-phase
  current, less the board's draw with the bridge off) at 20% and 15%, to see
  which phase is biased and by how much; only then consider a speed-dependent
  factor.

### ⚠️ HW1: fit the CARRIER, not the bare IC, on the milled board

The DRV8874's 36 °C/W assumes its exposed thermal pad is soldered to copper
with a via array — a JEDEC multi-layer board. The node PCB is **single-sided
isolation milling on the ANT CNC**: no plane under the part, no vias. Realistic
R<sub>θJA</sub> is then 60–90 °C/W, which at 3 A means a ~144 °C rise and erodes
most of the advantage over the DRV8833. A 16-pin HTSSOP with an exposed pad is
also far harder to mill and hand-solder than the SN65HVD230's SOIC-8, currently
the board's worst case.

**So HW1 designs the DRV8874 in as a carrier on a header.** The carrier is a
properly fabricated multi-layer board that handles the thermal pad correctly.
The 10 bare ICs are for the generation after, when the node board is fabricated
externally (JLCPCB) rather than milled.

**On arrival, check whether the carrier already populates an IPROPI resistor
and at what value** — the 2.2 kΩ figure only holds if the value is ours to pick.

### Firmware implications (W4) — pins done Aug 25, control loop still open

Pin allocation is settled and building clean; see **STM32F446RE — PIN
ALLOCATION** above for the full map and the reasoning. Six pins, not the five
originally budgeted — nFAULT was added once the carrier turned out to expose it.

**Done:**
- TIM2 encoder on PA15/PB3, TIM4 PWM on PB6/PB7, nSLEEP on PB5, nFAULT on PB12.
- All four DRV pins initialised to the driver-off state and bench-verified:
  PB5/PB6/PB7 at 0 V, PB12 at 3.3 V (open-drain released = no fault).
- `drv8833_disable()` asserts *both* off conditions — nSLEEP low **and** both
  inputs low — rather than relying on either alone.
- PB6/PB7 are parked as GPIO outputs driven low inside `MX_GPIO_Init`, because
  TIM4 does not claim them until several init functions later and they would
  otherwise float on the DRV8833's inputs for that whole window. ODR is written
  before MODER; the AF handover happens low-to-low with CCR already 0, so no
  pulse reaches the motor.
- Boot line on the console reports the state instead of leaving it assumed:
  `DRV8833: disabled (nSLEEP low, PWM 0%), nFAULT=clear`.

**Still open:**
- **TIM6 as the control-loop tick** (1 kHz, no pins). Do not hang the control
  loop off TIM7 — that is the 2 Hz heartbeat.
- **Encoder read + accumulate.** `TIM2->CNT` must never be read as an absolute
  position: take an `int32_t` delta each tick into a wider software counter.
  The code is the same whether the counter is 16- or 32-bit; 32-bit only means
  a stalled loop cannot silently lose revolutions.
- **Drive scheme not yet chosen.** Two independent PWM channels keep all three
  reachable without rewiring: sign-magnitude (fast decay), drive-brake (slow
  decay), locked antiphase. Drive-brake generally gives better low-speed
  duty-to-speed linearity, which is what PID cares about. The vendor page lists
  both `LL` and `HH` as "motor off" — they are not the same: **`LL` is coast
  (outputs Hi-Z), `HH` is brake (both outputs low)**, and it is the `HH` state
  that makes drive-brake possible.
- **Raise `nSLEEP` only when a non-zero command is issued**, and drop it again
  on fault or stall.

**Bench checks worth doing once:** scope PB5/PB6/PB7 from power-on to confirm
they come up low and stay low across the AF handover; and short the nFAULT node
to GND to confirm the boot line reports `ASSERTED` — a pin that has only ever
read high proves nothing about whether it is connected.

**Bring-up order — the motor is the LAST thing connected.** The PWM must be
driven from the `drv` console commands and verified on a scope with the motor
disconnected before any actuator is wired: frequency, duty linearity, channel
alignment, the waveform pair for the chosen drive scheme, direction reversal,
and that disable returns both pins to 0 V. Full checklist in NEXT TASKS item
6b. The console exists precisely so this can be done by hand without a
debugger or a reflash — same role it already plays for `send` and `mks`.
Doing it this way means a firmware mistake shows up as a wrong trace on a
screen rather than as an unexpected motion, which on a rover with a 131.3:1
gearbox is the difference between a note and a broken bench setup.

### Bench host tooling — `tools/bench/` (built Sep 25, 2026)

`node.py` (serial transport + console protocol) and `bench.py` (CLI, run
directories, safety). Under the **firmware** tree deliberately: the parser is
coupled to the console's exact output format, so a console change and its parser
change land in the same commit.

```sh
./bench.py run sweep --duty 5,10,15,20,25,30    # CW; --dir ccw for the other
./bench.py status                                # latest run's status.json
```

**No daemon, no IPC — the file *is* the interface.** The runner owns the port
for the duration and rewrites `status.json` once a second, so checking in on a
long run is reading one small file rather than attaching to anything. That is
the whole "start it, come back in five minutes" story. A run directory holds
`meta.json` (profile, outcome, and full `info`/`cfg`/`drv`/`enc` dumps taken at
connect — the conditions, recorded rather than typed), `console.log`,
`telemetry.csv`, `events.csv`, `status.json`, and the profile's own summary.

- **Raw log written before anything is parsed.** A parser bug then costs an
  analysis but never a bench run. This paid for itself the same day: the dead
  12 V rail was diagnosed from the recorded transcript in one look, with no
  re-run.
- ⚠️ **Telemetry lands *inside* command echoes, and the parser must expect it.**
  The console echoes each typed character as its own one-byte write, while a
  telemetry line is one atomic write — so a `T,` record routinely appears in the
  middle of an echo: `drv t` + `T,226,575448,200,…\r\n` + `imeout 2000`. A
  parser that anchors the record at the start of a line loses the record *and*
  corrupts the command echo behind it. `node.py` matches `T,…` **anywhere** in a
  line, extracts every match, and rejoins the residue into the echo. An echo
  mismatch is **counted** (`echo_mismatches`, reported in `meta.json` and
  `status.json`) rather than fatal, because the echo is a convenience and the
  telemetry is the measurement. This only ever showed up against real hardware.
- **Defaults track the rover's real band, not the full range.** `--duty`
  defaults to `5,7,9,…,29` — **5–29% in 2% steps** — and `--max-duty` refuses
  anything above **30%** (revised Sep 25 from 39%/40%: the rover runs slow, and
  above ~31% the wheel bounces on the rig belt, which makes those points a
  measurement of the rig rather than of the plant). Full-range sweeps are
  calibration and must be asked for explicitly. `--trip` defaults to **1580 mA**
  and is *set* at run start rather than assumed, so the trip is a recorded run
  condition instead of whatever the board booted with.
- **`--ramp <%/s>` + `--ramp-from <%>` give host-side entry and exit ramps**,
  the stand-in for the firmware slew limiter that does not exist yet. `--ramp-from`
  is **stepped to directly and held 1 s** — a ramp through a stationary wheel
  means nothing until it has broken away. Ramped entry kept peak current to
  **572 mA against the 1580 mA trip** where un-ramped duty steps had been
  tripping it. The ramp is a **staircase**, because `drv duty` takes integer
  percent.
- **`--note` records the physical setup into `meta.json`.** The rig, the load,
  the mounting and the rail are things the board cannot know, so if they are not
  typed they are not recorded — and a plant number without its mounting is not
  comparable to anything.
- **`count` is the measurement; `milli_rpm` is a convenience column.**
  `enc window` is a boxcar over N × 1 ms ticks, so at window 100 the rpm figure
  lags ~50 ms and is smoothed over 100 ms. Fitting *that* to an exponential
  would measure the filter's time constant and report it as the plant's. The
  tool derives speed from the least-squares slope of `count` against the
  board's own `ms` stamp.
- **Safety, in the order it matters.** (1) `try`/`finally` plus a SIGTERM
  handler, so a normal exit, an exception, Ctrl-C and `kill` all end at
  `drv duty 0` → `coast` → `disable`. (2) **`drv timeout 2000` armed on the
  board and kicked while dwelling** — the part `finally` cannot cover, because
  `kill -9` and a yanked cable run no Python at all. (3) A **30% duty ceiling**
  by default, matching the rover's real band. (4) Pre-flight refusal on a real
  fault or a non-zero duty. (5) Post-flight: any `seq` gap or `tx_dropped`
  marks the run **suspect**, because a stream with holes must not be quietly
  fitted.
- ⚠️ **The watchdog test has to be done deliberately, not assumed** — start a
  long run, `kill -9` it, and watch the wheel coast. It only ever fires when
  everything else has already failed. **Still owed.**
- ⚠️ **Flash wear will bite gain scanning.** `cfg` save appends a full snapshot
  to the next free slot, 1024 slots, currently at 0. A 50-point Kp/Ki scan that
  persists each trial burns 5% of the log per scan. W5's config bump needs a
  **volatile set-for-this-session path**, with `cfg save` only for a keeper.

- **Taking a run needs exactly one third-party package: `pyserial`.** `bench.py`
  and `node.py` import nothing else outside the standard library — the
  least-squares slope in `rpm_from_counts()` is hand-rolled rather than handed
  to numpy, deliberately, so a bench box stays minimal and a missing analysis
  library can never cost a run. numpy / matplotlib / scipy are for looking at
  the data afterwards and are listed in `tools/bench/requirements.txt` as
  floors, not pins. **No Node.js anywhere** — nothing in this project is
  JavaScript.

**Profiles:** `sweep` (done). `coastdown`, `step`, `hold`, `stiction` planned —
`coastdown` and `step` are the two that actually unblock gain selection, since
together they give a first-order model and therefore starting gains by pole
placement instead of by guessing.


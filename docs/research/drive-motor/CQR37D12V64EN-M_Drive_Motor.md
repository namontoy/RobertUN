# CQRobot CQR37D12V64EN-M — 131.3:1 Metal Gearmotor with 64 CPR Encoder

**Source:** https://cqrobot.com/product/CQR37D12V64EN-M
**Captured:** August 25, 2026
**Role in RobertUN:** wheel drive motor on all six modules — corners run this
plus the MKS SERVO42C steering; centres run this only. Seven purchased (six
wheels plus one spare), matching the seven WeAct F446 boards.

---

## Product identification

| | |
|---|---|
| Title | 131.3:1 Metal DC Geared-Down Motor 37Dx65L mm 6V or 12V, with 64 CPR Encoder and Mounting Bracket |
| SKU | CQR37D12V64EN-M |
| Brand | CQRobot |
| Price at capture | $21.99 (bulk discounts 5–20% at qty 10+) |

> **Note on the size discrepancy.** The product *title* says 37Dx65L mm, which
> is the generic family name across all gear ratios. The specification list for
> this particular ratio gives **37D × 72.5L mm**. Use 72.5 mm for enclosure and
> bracket clearance on the corner modules.

## Specification (verbatim from the page)

- Gear Ratio: 131.3:1
- Max Power @ 6V: 3 W; Max Power @ 12V: 6 W
- No-load Speed @ 6V: 38 rpm; No-load Speed @ 12V: 76 rpm
- No-load Current @ 6V: 0.15 A; No-load Current @ 12V: 0.2 A
- Stall Current @ 6V: 2.8 A; Stall Current @ 12V: 5.5 A
- Stall Torque @ 6V: 23 kg.cm (319 oz.in); Stall Torque @ 12V: 45 kg.cm (625 oz.in)
- Encoder Operating Voltage: 3.3V to 24V
- Encoder Type: Hall (Incremental Type)
- Encoder Resolution: 64CPR
- Cable Specifications: 22AWG; Length: 20 cm
- Cable Material: Silicone
- Cable Output: 2.54 mm DuPont Female
- Operating Temperature: -20 °C to +60 °C
- Motor Size: 37D * 72.5L mm; Mounting Bracket Size: 38 mm * 30 mm * 30 mm
- Weight: 235 g

## Wire colours (verbatim)

| Colour | Function |
|---|---|
| Red | Motor Power (connects to one motor terminal) |
| Black | Motor Power (connects to the other motor terminal) |
| Gray | Encoder GND |
| Blue | Encoder VCC (3.3 V to 24 V) |
| Yellow | Encoder A Output |
| White | Encoder B Output |

## Mechanical warning (verbatim)

> "Do not screw too far into the mounting holes as the screws can hit the
> gears. We recommend screwing no more than 3mm (0.12″) into the screw hole."

---

## Derived figures for RobertUN

Everything below is calculated from the specification above, not quoted from
the vendor. Recompute if any input changes.

### Encoder resolution

**64 CPR is counts per revolution of the MOTOR shaft, with x4 quadrature
decoding** (both edges of both channels) — the standard convention for this
class of 37D gearmotor. That means 16 pulses per channel per motor revolution.

At the output shaft:

```
64 × 131.3          = 8403.2  counts/rev   (x4 quadrature — what TIM2 counts)
16 × 131.3          = 2100.8  pulses/rev   (single channel)
2100.8 × 2          = 4201.6  edges/rev    (single channel, both edges)
```

**This supersedes the earlier 11 PPR estimate**, which gave 5777.2 counts/rev.
The real figure is **8403.2**, about 45 % higher.

The W4 hand-rotation test is diagnostic rather than merely confirmatory: turn
the output shaft exactly one revolution and read the count.

| Reading | Meaning |
|---|---|
| ~8403 | x4 decoding, `TIM_ENCODERMODE_TI12` working as intended |
| ~4202 | x2 — only one channel's edges are being counted |
| ~2101 | x1 — single edge, single channel |
| ~64 or ~16 | the gearbox ratio is not being applied; encoder is on the motor shaft as expected but something else is wrong |

**131.3:1 is a rounded ratio.** Real gear trains give awkward exact values, so
8403.2 is not an integer and never will be. That matters at W9 (precision
calibration), the same way the SERVO42C's own encoder resolution does.

### Speeds and count rates

The planned motor rail is ~9 V (a second buck down from the 13–13.5 V PDB
branch, because the DRV8833 tops out at 10.8 V).

```
No-load speed @ 9 V   ≈ 76 × 9/12   = 57 rpm      (0.95 rev/s at the output)
Count rate @ 9 V      ≈ 8403.2 × 0.95 = 7983 counts/s
Count rate @ 12 V     ≈ 8403.2 × 76/60 = 10 644 counts/s
```

Dropping from 12 V to 9 V costs about 25 % of top speed and nothing else.

**Encoder input filter check.** TIM2's IC filters are set to 15 (≈2.8 µs
rejection). The fastest per-channel edge spacing is 4201.6 edges/rev at
0.95 rev/s → 3992 edges/s → **250 µs apart**. Roughly 90× margin, so filter 15
is comfortable. Even at 12 V it is ~188 µs, still ~67×.

**16-bit rollover, retrospectively.** 65536 / 8403.2 = **7.8 output
revolutions**, or 8.2 s at 57 rpm — worse than the 11.3 revolutions estimated
from 11 PPR. This strengthens the decision to put the encoder on TIM2's 32-bit
counter, which reaches ~511 000 output revolutions (about six days continuous).

### Electrical — the motor is stronger than the driver

Winding resistance from the two stall points, which cross-check cleanly:

```
R = 12 V / 5.5 A = 2.18 Ω
R =  6 V / 2.8 A = 2.14 Ω     ← consistent, so the linear model holds
```

At the planned 9 V rail:

```
Stall current  @ 9 V ≈ 9 / 2.18        = 4.1 A
Max-power-point current @ 9 V ≈ (4.1 + 0.15) / 2 ≈ 2.1 A
Stall torque   @ 9 V ≈ 45 × 9/12       = 33.8 kg·cm = 3.31 N·m
```

Against the DRV8833 carrier's **~1.7 A continuous / 4 A peak** when paralleled:

| Condition | Motor draws | Driver allows | |
|---|---|---|---|
| No-load running | ~0.15 A | 1.7 A cont. | comfortable |
| Max mechanical power | ~2.1 A | 1.7 A cont. | **over continuous** |
| Stall, or start from rest at full duty | ~4.1 A | 4 A peak | **at the limit** |

**Starting from rest draws stall current momentarily**, because back-EMF is
zero at t = 0. A step from 0 to full duty is therefore a 4 A event every time,
not only during a fault.

See `PROJECT_CONTEXT.md` → *DRIVE MOTOR & DRIVER* for what this implies for
firmware (duty slew limiting, stall timeout, nFAULT handling) and for HW1.

### Torque headroom — worked example

Stall torque at 9 V is 3.31 N·m per wheel. For a rough sanity check, assuming
a 15 kg rover on six wheels (2.5 kg per wheel) and a 60 mm wheel radius:

```
Flat ground, rolling resistance μ ≈ 0.15:
  F = 2.5 × 9.81 × 0.15 = 3.7 N   →  T = 0.22 N·m   ≈ 7 % of stall
30° slope:
  F = 2.5 × 9.81 × sin30 + rolling = 12.3 + 3.7 = 16 N
                                  →  T = 0.96 N·m   ≈ 29 % of stall
                                  →  I ≈ 0.29 × 4.1 = 1.2 A
```

**Rover mass and wheel radius are assumptions, not measurements** — redo this
once the frame exists. But the shape of the answer is clear: the gearmotor has
large torque margin, and the binding constraint is the driver's current
rating, not the motor's ability to turn the wheel.

### Mass budget

235 g × 7 units = **1.65 kg in drive motors alone**, before the four SERVO42C
steering assemblies, the frame, batteries and compute. This feeds directly
back into the torque figures above.

---

## Integration notes and open questions

**Power the encoder from 3.3 V.** The 3.3–24 V input range means the outputs
swing to whatever VCC is given, so 3.3 V puts A/B directly at STM32 logic
levels and removes any dependence on PA15/PB3 being five-volt tolerant.

**Open: are the A/B outputs push-pull or open-drain?** Not stated on the page.
If the scope shows weak or slow rising edges during the W4 hand-turn test, fit
10 kΩ pull-ups to 3.3 V on the encoder lines. Worth resolving before HW1 fixes
the node PCB schematic.

**⚠️ The DuPont motor-power connector is a known hazard on this vehicle.**
The motor ships with 2.54 mm DuPont *female* crimps on its 22 AWG power leads.
Those contacts are typically rated 1–3 A; stall on this motor is 4.1 A at 9 V.
Worse, they are friction-fit and non-locking — which is precisely the
"intermittent power-ground contact under vibration" mechanism documented in
`PROJECT_CONTEXT.md` → *Ground loops*, and the leading explanation for last
semester's four destroyed MCUs. Rocker-bogie articulation means continuous
vibration at every corner.

**Replace the DuPont connectors on the two motor-power leads** with something
retained — JST VH, a screw terminal, or soldered and heat-shrunk — before any
extended running. The four encoder leads carry milliamps and can stay DuPont.

**The leads are only 20 cm.** The DRV8833 (or its replacement) has to sit
within 20 cm of the motor, or the run needs extending — and any extension on
the power pair should be at least 22 AWG and should not reintroduce a
friction-fit joint.

**Encoder GND (gray) and motor GND meet at the node PCB's star point**, per the
PCB-level grounding rule already recorded in `PROJECT_CONTEXT.md`. Do not tie
them at the motor end.

**Mounting bracket is 38 × 30 × 30 mm** — needed for corner-module enclosure
geometry, which is also what gates the 3D-printed XT60 retention cover.

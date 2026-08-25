# Drive motor driver selection — DRV8874 vs DRV8876 vs DRV8871 vs DRV8833

**Decided:** August 25, 2026
**Outcome:** **DRV8874** selected. Purchased: **8 × carrier board + 10 × bare IC**,
expected arrival ~September 15, 2026.
**Supersedes:** the Aug 16, 2026 decision to keep the DRV8833 carrier.

**Sources**
- DRV8874 datasheet, TI SLVSF66A (Aug 2019, rev. Dec 2019)
- DRV8876 datasheet, TI SLVSDS7B (Aug 2019, rev. Nov 2019)
- DRV8871 product page, https://www.ti.com/product/DRV8871
- DRV8833 carrier, https://lastminuteengineers.com/drv8833-arduino-tutorial/
  (captured in `DRV8833_Carrier_Module.md`, this folder)
- Pricing/stock: DigiKey, checked Aug 25, 2026

---

## What the requirement actually is

From `CQR37D12V64EN-M_Drive_Motor.md`: winding resistance **2.18 Ω**,
cross-checked against both stall points on the datasheet.

| Rail | Stall current | No-load speed |
|---|---|---|
| 9 V (needs a second buck) | 4.1 A | 57 rpm |
| **13.5 V (branch rail direct)** | **6.2 A** | **85.5 rpm** |

A driver that takes the branch rail directly deletes a buck converter *and*
gains 50 % top speed. 13.5 V is above the motor's 12 V rating, but the drive is
PWM anyway — a duty cap of 12/13.5 = **89 % gives exactly 12 V average** at no
cost.

**The more important criterion is current *regulation*, not peak current
rating.** The DRV8833 carrier's real failing is not only that 4 A exceeds it;
it is that with the sense pins grounded there is nothing between "fine" and
"OCP trips, wheel stops." A driver that chops at a programmed threshold turns
stall current into a *setting*. The motor then becomes torque-limited and keeps
pushing, instead of the driver faulting.

## The four options

| | **DRV8874** ✅ | DRV8876 | DRV8871 | DRV8833 (incumbent) |
|---|---|---|---|---|
| **VM range** | **4.5 – 37 V** | 4.5 – 37 V | 6.5 – 45 V | **2.7 – 10.8 V** ❌ |
| VM abs max | 40 V | 40 V | 50 V | — |
| **Peak I<sub>OUT</sub>** | **6 A** | 3.5 A | 3.6 A | 2 A/ch → 4 A paralleled |
| Realistic continuous | ~3 A | ~1.5 A | — | ~1.7 A paralleled |
| **R<sub>DS(on)</sub> HS+LS** | **200 mΩ** (100+100) | 700 mΩ (350+350) | 565 mΩ | not captured |
| R<sub>θJA</sub> HTSSOP-16 | **36.0 °C/W** | 44.3 °C/W | — | n/a (carrier) |
| OCP trip point | 6 A min / 10 A typ | 3.5 A min / 5.5 A typ | — | — |
| **Current regulation** | **Yes, VREF-programmable** | Yes, VREF-programmable | Yes, resistor-set | **Disabled on carrier** ❌ |
| **Current feedback out** | **Yes — IPROPI** | Yes — IPROPI | **No** | **None** ❌ |
| A<sub>IPROPI</sub> scaling | **450 µA/A** | **1000 µA/A** | — | — |
| Sense accuracy | ±6 % (1–2 A) | ±4 % (0.5–2 A, PWP) | — | — |
| nFAULT pin | Yes, open-drain | Yes, open-drain | not listed <sup>1</sup> | Yes (`ULT`), floating |
| nSLEEP pin | Yes, 100 kΩ pulldown | Yes, 100 kΩ pulldown | not listed <sup>1</sup> | Yes (`EEP`), J1 pull-up |
| Logic V<sub>IH</sub> | 1.5 – 5.5 V | 1.5 – 5.5 V | — | 3 V / 5 V |
| Max f<sub>PWM</sub> | 100 kHz | 100 kHz | — | 250 kHz |
| t<sub>WAKE</sub> | 1 ms | 1 ms | — | 1 ms |
| Package | HTSSOP-16 (PWP) | HTSSOP-16 **+ VQFN-16** | HSOIC-8 | carrier module |
| Price @1 / @10 | $3.40 / $2.55 | **$1.95 / $1.44** | — | — |
| DigiKey stock | 776 | **6 583** | — | — |

<sup>1</sup> The TI product page lists "Low-Power Sleep Mode" and integrated
protections but does not enumerate nFAULT or nSLEEP pins for the 8-pin package.
Not verified against the datasheet, because the part was ruled out on current.

### Why each of the three was rejected

**DRV8833 — the incumbent.** Two independent disqualifications: the 10.8 V
ceiling sits below the 13–13.5 V branch rail, and the carrier grounds
AISEN/BISEN so there is no current limiting and no current feedback at all.
~1.7 A paralleled against a 4.1 A stall at 9 V.

**DRV8871.** 3.6 A peak is below the 6.2 A stall. R<sub>DS(on)</sub> is nearly
3× the DRV8874's, and the 8-pin package appears to offer neither nFAULT nor
nSLEEP — it would cost both status pins already allocated on the node.

**DRV8876.** Ruled out on **thermals**, not on features. It is functionally
identical to the DRV8874 and shares its pinout exactly, but its output stage is
**3.5× more resistive**, and that goes in as I²R:

| Current | DRV8874 rise | DRV8876 rise |
|---|---|---|
| 1.5 A | 16 °C | 70 °C |
| 2 A | 29 °C | **124 °C** |
| 3 A | 65 °C | — |

Against the motor's stated **+60 °C ambient** limit:

- DRV8874 at 3 A → T<sub>J</sub> = 125 °C, inside the 150 °C maximum
- DRV8876 at 2 A → T<sub>J</sub> = 184 °C, **past thermal shutdown** (175 °C typ)
- DRV8876 at 1.5 A → T<sub>J</sub> = 130 °C, workable and about its ceiling

At ~1.5 A continuous the DRV8876 is **no better than the DRV8833 paralleled**.
It fixes the voltage problem and adds current sensing, but not the current
problem — which is the one that actually bites.

### Keep the DRV8876 as the second source

The PWP pinout is **identical, pin for pin**:

```
1 EN/IN1   2 PH/IN2   3 nSLEEP   4 nFAULT   5 VREF    6 IPROPI  7 IMODE  8 OUT1
9 PGND    10 OUT2    11 VM      12 VCP     13 CPH    14 CPL   15 GND   16 PMODE
```

One footprint, one symbol, either part populated — worth recording as `AltMPN`,
especially since DRV8874 stock (776) is an order of magnitude thinner than the
DRV8876's (6 583).

> **⚠️ One caveat on the swap: A<sub>IPROPI</sub> differs — 450 µA/A on the
> DRV8874, 1000 µA/A on the DRV8876.** The parts are electrically drop-in but
> the current-sense scaling changes by 2.22×. Swapping requires a different
> IPROPI resistor *and* a different firmware calibration constant. Do not treat
> the second source as transparent.

---

## What the DRV8874 buys

**Pins 1–4 are the four we already have.** `EN/IN1`, `PH/IN2`, `nSLEEP`,
`nFAULT`. With PMODE strapped for PWM mode, IN1/IN2 carry the same truth table
as the DRV8833, so the drive logic written in W4 carries over unchanged.
t<sub>WAKE</sub> is 1 ms on both, so sleep handling is unchanged too.

**nSLEEP has a 100 kΩ internal pulldown** — the "disabled unless the MCU says
otherwise" property that required cutting J1 on the DRV8833 carrier is the
default here.

**IPROPI closes two open items.** It restores the option of a current inner
loop in W5 — which the DRV8833 made unreachable — and it measures real drive
current, which is the open W4 task that HW4's PDB branch sizing waits on.

### IPROPI design values

`I_IPROPI = I_OUT × 450 µA/A` for the DRV8874.

```
R_IPROPI = 2.2 kΩ  →  0.99 V/A  →  2.97 V at 3 A
```

That is near-perfect full scale against a 3.3 V ADC reference, with no op-amp
and no scaling stage. IPROPI is rated 0–3 mA, reached at 6.67 A output — above
the 6 A peak, so the resistor never clips before the driver does.

The same relation sets the regulation threshold against VREF, giving roughly
**1 A per volt** on a 2.2 kΩ sense resistor. **Confirm the exact I<sub>TRIP</sub>
formula in the datasheet's Current Regulation section before fixing the
schematic** — whether VREF is compared directly or through an internal divider
has not been verified here.

> **Correction, Aug 25, 2026.** An earlier working figure of "R_IPROPI = 1 kΩ,
> 1 mV/mA, 3.0 V at 3 A" was wrong for this part. It came from the DRV8876's
> A<sub>IPROPI</sub> of 1000 µA/A; the DRV8874 is **450 µA/A**. The correct
> value is 2.2 kΩ.

### Pin impact on the node

| Signal | Pin | Note |
|---|---|---|
| IN1, IN2 | PB6, PB7 | unchanged from DRV8833 |
| nSLEEP | PB5 | unchanged |
| nFAULT | PB12 | unchanged, still wants a 10 kΩ external pull-up |
| **IPROPI** | **PA2** (`ADC1_IN2`) | **new** — the ADC input |
| VREF | fixed divider, or **PA4** (`DAC1_OUT`) | optional: a software-programmable current limit |
| PMODE, IMODE | resistor straps | no MCU pins |

PA4/DAC1 is free (PA5/DAC2 stays reserved for SPI1_SCK). Using it maps the
DAC's 0–3.3 V output linearly onto a 0–3.3 A programmable limit — firmware
could allow a higher burst limit for climbing and a lower one for cruising.

---

## ⚠️ HW1 impact: use the carrier on the milled board, not the bare IC

**The DRV8874's 36 °C/W assumes its exposed thermal pad is soldered to copper
with a via array** — a JEDEC multi-layer board. The node PCB is
**single-sided isolation milling on the ANT CNC**: no plane under the part, no
vias. The pad would have nowhere to dump heat, and R<sub>θJA</sub> realistically
lands around 60–90 °C/W:

```
Bare HTSSOP on single-sided milled FR4, ~80 °C/W:
  2 A → 0.8 W → 64 °C rise    workable
  3 A → 1.8 W → 144 °C rise   not workable
```

That erodes most of the advantage over the DRV8833. It also makes the part the
hardest footprint on the board by a wide margin — the roadmap notes the current
worst case is the SN65HVD230's SOIC-8, and a 16-pin HTSSOP with an exposed pad
is well past that.

**Therefore: HW1 should design the DRV8874 in as a carrier on a header.** The
carrier is a properly fabricated multi-layer board that handles the thermal pad
correctly, and it needs no fine-pitch milling.

The 10 bare ICs are not wasted — they are the right part the day the node board
is fabricated externally (JLCPCB) rather than milled in-house. Keep them for
that generation.

**On carrier arrival, check whether it already populates an IPROPI resistor and
what value** — the 2.2 kΩ figure above only holds if the value is ours to choose.

## Open items

- Which **PMODE** tri-level selects PWM (IN1/IN2) mode — determines whether the
  W4 firmware truly drops in unchanged
- The exact **I<sub>TRIP</sub> vs VREF** formula (Current Regulation section)
- Whether the **carrier populates IPROPI**, and its value
- **Measure the motor's winding resistance directly** to validate the 2.18 Ω
  that this entire analysis rests on — a multimeter across the terminals,
  rotating the shaft between readings to average brush position. Needs no
  driver and can be done before the parts arrive.

# DRV8833 Dual H-Bridge Carrier — module reference

**Source:** https://lastminuteengineers.com/drv8833-arduino-tutorial/
**Captured:** August 25, 2026 (first consulted August 24)
**Role in RobertUN:** drive-motor driver on all six modules, paralleled
(IN1=IN3, IN2=IN4; OUT1+OUT3, OUT2+OUT4) to raise current capability. Pairs
with the CQRobot CQR37D12V64EN-M — see the companion document in this folder.

---

## What the chip is

A dual NMOS H-bridge driver from Texas Instruments. The MOSFET output stage is
the reason it is preferred over an L293D or L298N: BJT-based drivers drop
around 1–2 V per side and dissipate it as heat, whereas the DRV8833 delivers
nearly the full supply voltage to the motor.

Drives two brushed DC motors, one bipolar stepper, solenoids, or other
inductive loads.

## Specification

| Parameter | Value |
|---|---|
| Motor voltage (VM) | **2.7 V – 10.8 V** |
| Logic voltage | 3 V and 5 V compatible |
| Continuous output current | **1.2 A per channel** |
| Peak output current | **2 A per channel** |
| Motor channels | 2 |
| Protection | Under-voltage lockout, over-current, over-temperature |

**Single supply.** Unlike most drivers there is no separate logic rail — the
DRV8833 runs its internal logic and charge pump from VM. The carrier therefore
takes one supply and has no onboard regulator.

## Terminals

**The silkscreen truncates two labels and it is genuinely confusing:**

| Silkscreen | Actual signal |
|---|---|
| `EEP` | **nSLEEP** |
| `ULT` | **nFAULT** |

Full set: `VCC`, `GND`, `IN1` `IN2` `IN3` `IN4`, `OUT1` `OUT2` `OUT3` `OUT4`,
`EEP`, `ULT` — twelve pins.

- **VCC / GND** — motor supply, 2.7–10.8 V
- **OUT1, OUT2** — motor A; **OUT3, OUT4** — motor B
- **IN1, IN2** — motor A control; **IN3, IN4** — motor B control

All four control inputs accept PWM.

## Truth table

The vendor page lists both `LL` and `HH` as "Motor OFF". **They are not the
same state**, and the difference is what makes slow-decay drive possible:

| IN1/IN3 | IN2/IN4 | Vendor wording | Actual behaviour |
|---|---|---|---|
| 0 | 0 | Motor OFF | **Coast** — outputs Hi-Z, fast decay |
| 1 | 0 | Forward | Forward |
| 0 | 1 | Reverse | Reverse |
| 1 | 1 | Motor OFF | **Brake** — both outputs low, slow decay |

Speed control is applied as PWM on the pin that would otherwise be held high.

**The control inputs are pulled low internally**, which disables the outputs
by default. This is what makes the STM32's boot window survivable: between
reset and `MX_GPIO_Init()` the MCU pins float, and the chip's internal
pull-downs put the bridge in coast rather than driving.

## nSLEEP (`EEP`) and the J1 jumper

- Low → low-power sleep: H-bridges disabled, charge pump stopped, all logic
  reset, clocks stopped, inputs ignored
- High → driver active
- **Wake-up takes up to 1 ms.** Firmware must wait after raising nSLEEP before
  expecting the outputs to respond.

**J1 is a solder jumper on the back of the module and ships CLOSED**, which
connects an onboard pull-up and therefore **leaves the driver ENABLED by
default, with no microcontroller involved.** Opening J1 disconnects that
pull-up and lets the on-chip pull-down hold nSLEEP low, so the driver is
disabled by default.

> **RobertUN decision (Aug 25, 2026): J1 is cut.** With it open, "MCU not
> running ⇒ bridge disabled" becomes a property of the hardware rather than a
> firmware promise. Done on the bench board; **must be repeated on all seven
> carriers.**
>
> Additionally, with J1 closed the EEP pin may sit at VM. At a 9 V motor rail
> that would exceed what a 5 V-tolerant STM32 pin can accept — **measure EEP
> before connecting it to a GPIO** if J1 has not been cut.

## nFAULT (`ULT`)

- Open-drain output, active low
- Pulled low on over-current, over-temperature, or under-voltage
- **Floats by default — there is no pull-up on the carrier.** An external
  pull-up or the microcontroller's internal one is required

> **RobertUN:** a 10 kΩ external pull-up is fitted on the bench build and
> belongs on the HW1 schematic. The STM32's internal pull-up on PB12 is kept
> enabled alongside it, so that a production board with the resistor
> unpopulated still cannot float and invent faults.
>
> **nFAULT high does not prove the driver is alive.** With VM absent the chip
> is unpowered, its open-drain transistor is off, and the pull-up reads 3.3 V
> regardless. A clear nFAULT means "nothing is pulling this low."

## ⚠️ Current limiting is disabled on this carrier

The DRV8833 supports current limiting via sense resistors from AISEN (motor A)
and BISEN (motor B) to ground. Quoting the page directly:

> "this specific DRV8833 breakout board connects these current sense pins
> directly to ground, effectively disabling the current limiting feature."

Two consequences for RobertUN:

1. **Nothing limits stall current except the chip's own OCP.** A jammed wheel
   pulls whatever the motor pulls until over-current protection trips and
   asserts nFAULT. Firmware must monitor nFAULT, implement a stall timeout,
   and latch off rather than retrying into a stuck wheel.
2. **There is no current feedback at all**, so **W5's PID is velocity-only.**
   A torque or current inner loop is unreachable with this carrier. Obtaining
   one means putting a DRV8833 (or another driver) directly on the node PCB
   with real sense resistors — an HW1 decision, not a firmware one.

---

## RobertUN-specific notes

### Paralleling

TI documents parallel operation explicitly (DRV8833 datasheet, Figure 7): the
two bridges may be tied together and the device's internal dead time prevents
cross-conduction, so no external protection is needed.

On this carrier every output is a separate terminal, so paralleling is
external wiring:

```
PB6 (TIM4_CH1) ──┬── IN1        OUT1 ──┬── motor +
                 └── IN3        OUT3 ──┘
PB7 (TIM4_CH2) ──┬── IN2        OUT2 ──┬── motor −
                 └── IN4        OUT4 ──┘
PB5            ──── EEP  (nSLEEP)
PB12           ──── ULT  (nFAULT, 10k pull-up to 3.3 V)
```

Costs no extra MCU pins — one STM32 pin drives two carrier inputs.

### Current budget — ~2 A, not 3 A

An earlier revision of `PROJECT_CONTEXT.md` recorded 3 A RMS paralleled, taken
from TI's 1.5 A RMS per-bridge silicon figure. That is too optimistic for this
carrier, whose 1.2 A per-channel rating is a **board-level thermal** limit
below the silicon's.

Paralleling halves R<sub>DS(on)</sub>, so for the same dissipation the current
scales by √2, not 2:

```
1.2 A × √2 ≈ 1.7 A continuous     (perhaps ~2.4 A if the carrier's copper is generous)
2 A × 2    =  4 A peak
```

**Design the PDB branch around ~2 A RMS / 4 A peak.**

### The motor out-muscles the driver

From the companion motor document: stall is **4.1 A at the planned 9 V rail**,
and the max-power-point current is ~2.1 A. Starting from rest at full duty
draws stall current momentarily, because back-EMF is zero at t = 0.

So the driver, not the motor, is the binding constraint. Mitigations, in the
order they cost least:

1. **Slew-limit the commanded duty** (soft start) so a step command does not
   become a 4 A event.
2. **Cap maximum duty.** At stall, `I = duty × V / R`; with R = 2.18 Ω and a
   9 V rail, holding I ≤ 2 A needs duty ≤ 48 %. Crude, and it costs top speed.
3. **Lower the motor rail.** At 6 V stall is 2.8 A — within peak, still above
   continuous, and no-load speed drops to 38 rpm.
4. **Change the driver in HW1.** A part rated for the full 13–13.5 V branch
   rail at 3–5 A would remove the 10.8 V ceiling *and* the current ceiling
   *and* delete the second buck converter. HW1's schematic is being drawn now,
   which is the cheapest moment this decision will ever be available.

### Voltage conflict with the PDB

The PDB branch rail is 13–13.5 V, chosen to pre-compensate wire drop against
the SERVO42C's 12 V floor. That is **above the DRV8833's 10.8 V maximum**, so
the carrier cannot be fed from the branch rail directly. Current plan is a
second buck per node PCB stepping 13–13.5 V down to ~9 V, fed independently
from the 13 V rail rather than cascaded off the motor rail, so motor noise
does not sit upstream of the MCU supply.

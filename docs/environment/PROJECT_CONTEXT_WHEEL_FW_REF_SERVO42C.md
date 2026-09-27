# RobertUN wheel firmware — reference: MKS SERVO42C steering driver

> Reference tier. Moved verbatim from `PROJECT_CONTEXT_WHEEL_FW.md` on 2026-09-26.
> Do not read whole: `grep -n '^#' <this file>` and read the section you need.
> Contents: UART protocol, verified command set, echo/framing traps, bench results

## MKS SERVO42C V1.1 — UART PROTOCOL & BENCH VALIDATION

**Recovered and added Aug 6, 2026. Source session: June 8, 2026 ("Rover wheel
control board testing"). This was omitted from this file at the time — the whole
session went undocumented, which later caused W3 to be misjudged as higher-risk
than it is. The protocol below is validated against real hardware.**

### Role in the architecture
Steering actuator for the four corner modules. **UART-only — not CAN-native.**
This is why each corner STM32F446RE is a bridge: CAN in from Orion, UART out to
the SERVO42C. Motor is a NEMA 17 driving a custom 3D-printed 19:1 cycloidal
gearbox.

### Driver configuration (set via the onboard menu, per unit)
```
Menu → Mode     → CR_UART     (default is CR_vFOC — motion commands are IGNORED until changed)
Menu → UartBaud → 38400       (factory default)
Menu → UartAddr → 0xE0        (ALL units — see addressing note below)
```
- **Bench unit as tested:** CR_UART, addr `0xE0`, 38400 baud, **Mstep = 8**
- ⚠️ Read commands (e.g. `30`) respond in ANY mode; motion commands (`FD`, `F6`)
  require CR_UART. A driver that answers an encoder read but ignores a move
  command is almost certainly still in CR_vFOC.

### Addressing: all four steering drivers stay at 0xE0
The UART link is **point-to-point** — each corner STM32F446RE has its own
dedicated UART to its own SERVO42C. There is no shared bus, so there is nothing
for the address byte to disambiguate. `0xE0` is a protocol constant, not an
identifier.

**Module identity lives in the CAN ID, not the UART address.** Each STM32 has a
unique CAN node ID; the SERVO42C behind it does not need one.

Consequences, all favourable:
- **Identical firmware on all four corners** — no per-unit address constant, no
  build variants. Directly simplifies W7 replication.
- **Drivers are interchangeable spares** — a failed unit is swapped from the
  shelf with no menu reconfiguration.
- One less commissioning step per module, and one less thing to get silently wrong.

Addresses `0xE0`–`0xE3` would only be needed if several drivers shared one UART
(multi-drop), which this architecture does not do. Keep it in mind only for a
bench scenario where two drivers are deliberately hung off one USB-serial adapter.

### Resolution with Mstep = 8 and the 19:1 gearbox
```
Pulses per motor revolution:   8 × 200        = 1,600
Pulses per output revolution:  1,600 × 19     = 30,400
Angular resolution at output:  360° / 30,400  = 0.01184°  (~43 arcseconds)
```

### Packet format
```
[addr] [function code] [data bytes...] [checksum]
```
Checksum = sum of ALL preceding bytes (addr and function code included), `& 0xFF`.
It is a plain 8-bit additive checksum, not a real CRC, despite being called CRC.

> **MANDATORY WORKING RULE — Claude has made repeated checksum errors on this
> protocol.** Always write the full decimal breakdown before stating a checksum
> byte, so the arithmetic can be checked at a glance. Example:
> ```
> E0 + 30 = 224 + 48 = 272
> 272 & 0xFF = 272 − 256 = 16 = 0x10
> ```

### Verified command set (every checksum below re-verified Aug 6, 2026)

**Read-only diagnostics — safe in any mode, no motion:**
| Purpose | Full packet | Checksum arithmetic | Returns |
|---|---|---|---|
| Read encoder | `E0 30 10` | 224+48=272 → 16 | int32 carry + uint16 value |
| Pulses received | `E0 33 13` | 224+51=275 → 19 | int32 pulse count |
| Shaft angle error | `E0 39 19` | 224+57=281 → 25 | int16 (65536 = 360°) |
| EN pin status | `E0 3A 1A` | 224+58=282 → 26 | `01`=enabled, `02`=disabled |
| Protection state | `E0 3E 1E` | 224+62=286 → 30 | `01`=protected, `02`=clean |

`E0 30 10` is the **safe first command** for any bring-up — no motion, no config change.

**Motion (CR_UART only):**
| Purpose | Full packet | Checksum arithmetic |
|---|---|---|
| Enable motor | `E0 F3 01 D4` | 224+243+1=468 → 212 |
| Stop / hold | `E0 F7 D7` | 224+247=471 → 215 |
| Move 1,600 pulses CW (1 motor rev @ Mstep 8) | `E0 FD 02 00 00 06 40 25` | 224+253+2+0+0+6+64=549 → 37 |
| Move 1,600 pulses CCW | `E0 FD 82 00 00 06 40 A5` | 224+253+130+0+0+6+64=677 → 165 |
| Move 160 pulses CW | `E0 FD 02 00 00 00 A0 7F` | 224+253+2+0+0+0+160=639 → 127 |
| Move 160 pulses CCW | `E0 FD 82 00 00 00 A0 FF` | 224+253+130+0+0+0+160=767 → 255 |
| Move 16 pulses CW (fine step) | `E0 FD 02 00 00 00 10 EF` | 224+253+2+0+0+0+16=495 → 239 |

`FD` layout: `FD [VAL] [uint32 pulses, big-endian]`, where VAL bit7 = direction
(0 = CW, 1 = CCW) and bits6–0 = speed. **`0x02` = CW speed 2; `0x82` = CCW speed 2.**

`F6` runs at constant speed with the same VAL byte encoding. Speed formula for a
1.8° motor: `RPM = (Speed × 30000) / (Mstep × 200)`.
For steering, use low speed values (1–4) with `FD`, not `F6`.

**Configuration (persistent, written to flash):**
| Parameter | Code | Data | Default |
|---|---|---|---|
| Motor type | `81` | `00`=0.9°, `01`=1.8° | `01` |
| Work mode | `82` | `00`=OPEN, `01`=vFOC, `02`=UART | `01` |
| Microstepping | `84` | `00`–`FF` | `10` (16) |
| EN pin polarity | `85` | `00`=L, `01`=H, `02`=Hold | `00` |
| Direction | `86` | `00`=CW, `01`=CCW | `00` |
| Locked-rotor protection | `88` | `00`=off, `01`=on | `00` |
| Baud rate | `8A` | `01`=9600 … `06`=115200 | `04` (38400) |
| UART address | `8B` | `00`–`09` → addr = `0xE0 + n` | `00` |
| Restore defaults | `3F` | — | — |
| **Kp** (position) | `A1` | uint16 | `0x0650` = 1616 |
| **Ki** (position) | `A2` | uint16 | `0x0001` = 1 |
| **Kd** (position) | `A3` | uint16 | `0x0650` = 1616 |
| **ACC** (accel ramp) | `A4` | uint16 | `0x011E` = 286 — ⚠️ too large can damage the board |
| **MaxT** (max torque) | `A5` | uint16, range 0–`0x04B0` | `0x04B0` = 1200 |

Set MaxT to maximum: `E0 A5 04 B0 39` → 224+165+4+176 = 569 → 569−512 = 57 = 0x39
Set Kp to default:   `E0 A1 06 50 D7` → 224+161+6+80 = 471 → 471−256 = 215 = 0xD7

### ⚠️ The driver ECHOES every request before replying

**Discovered Aug 13, 2026, on the STM32.** The SERVO42C retransmits the bytes it
just received, then sends its answer. What actually arrives is:

```
E0 30 10 | E0 00 00 00 00 00 2A 0A
└─ echo ┘ └──── the actual reply ────┘
```

**This was never noticed in the June 8 bench session** because that used a hex
terminal, where a human eye reads past the repeated bytes without registering
them. It only surfaced once software had to validate a frame: the checksum gets
computed across echo *and* reply together and a perfectly good response is
rejected as corrupt.

**Confirmed device behaviour, not a wiring loop.** Verified across two different
SERVO42C boards and two motors, with connectors swapped and MCU-side wiring
re-checked — TX and RX are not bridged anywhere.

**Handling it (`strip_echo()` in `mks_servo.c`):** discard a leading run that
exactly matches the bytes just transmitted, then validate what remains. Two
properties make this safe rather than a heuristic:

- The echo **cannot arrive split**. Its bytes stream back continuously while we
  transmit, so an idle gap can only open after the last of them — the echo is
  either wholly present or not started, and an exact prefix match is sound.
- Echo and reply may arrive as **one burst or two** separated by an idle gap.
  Both occur; an echo-only burst must be treated as "keep waiting", not as a
  malformed reply.

The strip is a no-op on a link that does not echo, so it costs nothing if a
future firmware revision drops the behaviour. Each one is counted in
`mks_stats.echoes` and reported by the console as "echoes stripped (normal)" —
**a non-zero count is expected, not a fault.**

> Anyone writing a new command for this protocol, or debugging one that returns
> "bad checksum", should read this first. The reply is very likely fine.

### Response format
| Response | Meaning | Checksum |
|---|---|---|
| `E0 01 E1` | command accepted / run starting | 224+1=225 |
| `E0 02 E2` | run complete | 224+2=226 |

**Encoder read** `E0 30 10` → e.g. `E0 FF FF FF F8 2B B1 B1`
```
E0            addr
FF FF FF F8   carry, int32 = −8 (full encoder overflows)
2B B1         value, uint16 = 11,185
B1            checksum
```

**Angle error** `E0 39 19` → e.g. `E0 FF 99 78`
```
0xFF99 as int16 = −103
−103 / 65536 × 360° = −0.566°
```
Negative = shaft pushed back from its target by the external load.

> ⚠️ **OPEN QUESTION for W9 precision calibration.** The SERVO42C encoder sits on
> the **motor** shaft, so these angle-error degrees are motor-side. The stiffness
> figures below pair motor-side degrees with output-side torque, so they are a
> mixed-unit convenience number, not a true output stiffness. Divide by 19 for
> output-referred deflection (−0.566° motor ≈ −0.0298° output). Resolve this
> convention explicitly before quoting stiffness anywhere it matters.

### Reply lengths by function code (measured Aug 13, 2026)

Every command answers with a fixed number of bytes. This is what makes framing
deterministic — see the next section for why timing cannot be used instead.

| Function | Command | Reply | Layout |
|---|---|---|---|
| `30` | read encoder | **8** | `E0` + int32 carry + uint16 value + ck |
| `33` | pulses received | **6** | `E0` + int32 + ck |
| `39` | shaft angle error | **4** | `E0` + int16 + ck |
| `3A` | EN pin status | **3** | `E0` + status + ck |
| `3E` | protection state | **3** | `E0` + status + ck |
| `3F` | restore defaults | **3** | ack |
| `F3` | enable / disable | **3** | ack |
| `F6` | constant speed | **3** | ack, then a second 3-byte completion |
| `F7` | stop | **3** | ack |
| `FD` | relative move | **3** | ack, then a second 3-byte completion |
| `81`–`8B` | config writes | **3** | ack |
| `A1`–`A5` | Kp/Ki/Kd/ACC/MaxT | **3** | ack |

**Beware the 3-byte collision.** `E0 01 E1` and `E0 02 E2` are simultaneously
the generic accepted/complete acknowledgements *and* valid status values for
`3A` (01 = enabled, 02 = disabled) and `3E` (01 = protected, 02 = clean). The
bytes alone cannot distinguish them — **decode by the function code you sent**,
never by reply length or content. Verified Aug 13 by toggling `F3` and watching
`3A` follow it, which rules out the possibility that `3A` was merely being
acknowledged rather than answered.

### ⚠️ Idle-line framing does NOT work on this link

The obvious way to delimit a variable-length reply is the UART's IDLE flag, and
it is what `debug_uart` uses successfully for the console. **It is wrong here.**

The SERVO42C echoes in *software* — one byte at a time as it processes them —
and pauses for longer than one character time *within* a single message. IDLE
fires after one character time (~260 us at 38400), so it triggers mid-message
and is not a boundary at all.

**How this presented:** a move command returned

```
00 00 10 EF E0 01 E1
```

which is the **tail of our own request** (`E0 FD 02 00 00 00 10 EF`, bytes 4-7)
followed by a valid accepted-ack. An IDLE fired during transmission, the
handler flushed the receive buffer, and the first four echo bytes were
destroyed — leaving a fragment that failed address validation. Cost a bring-up
session to diagnose, and the symptom pointed at everything except the real
cause.

**The rule:** accumulate received bytes unconditionally, never resetting on a
timing boundary, and decide a message is complete by its **expected length**
(table above) or — for an undocumented function code — by a quiet period long
enough to clear this device's inter-byte gaps (8 ms is used).

### ⚠️ Clearing UART error flags steals a byte from the DMA

Generic STM32 trap, not MKS-specific, and worth knowing anywhere HAL UART DMA
reception is used.

`__HAL_UART_CLEAR_OREFLAG()` and its siblings all expand to the same thing on
F4: a read of `SR` followed by a read of `DR`. **Reading DR while DMA reception
is running consumes a byte the DMA was entitled to**, and it is gone. Calling
these from `HAL_UART_ErrorCallback()` — the natural place — does exactly that
whenever HAL treated the error as non-blocking and left the transfer running.

It surfaces as occasional inexplicable checksum failures under load, and gets
blamed on wiring.

**Related: re-arming unconditionally causes an error cascade.** Calling
`HAL_UARTEx_ReceiveToIdle_DMA()` on every error means one seed event can flag
another error during the re-arm, which re-arms again. **180 error callbacks
across 7 transactions** were observed this way on Aug 13, then zero on the next
boot — dormant, not fixed.

**The rule for both:** ask the hardware whether reception actually stopped —

```c
still_running = (huart->Instance->CR3 & USART_CR3_DMAR) &&
                (hdma_rx.Instance->CR & DMA_SxCR_EN);
```

— and only clear flags or re-arm when it has. HAL clears both bits when it
treats an error as blocking (overrun, DMA fault) and leaves them alone
otherwise, so this observes the real state rather than assuming a policy.

### Bench results that W3 rests on

**Established Aug 13, 2026** (first commanded motion; step-by-step record in
`PROJECT_CONTEXT_WHEEL_FW_LOG.md`):
- **Mstep = 8 confirmed by measurement, not by the menu** — predicted vs actual
  landed within **2 counts**, with a constant (not growing) offset. At Mstep 16
  the first row would have been off by a factor of two, so the result is
  decisive. Measuring what the mechanism did is the reliable way to check Mstep.
- **Round-trip repeatability: 4 counts** — about one tenth of a single microstep
  (41 counts), i.e. 0.0012° at the output. No measurable backlash at that
  amplitude. Baseline for W9 precision calibration.
- **`33` counts UART-commanded pulses, not just hardware STEP input**, which is
  what makes it usable as the feedback path for absolute positioning — `FD`
  alone cannot, being a relative move.
- **Direction convention: positive degrees / `ccw = false` DECREMENTS** both the
  encoder position and the `33` pulse counter. Pin this down before Ackermann
  sign conventions are written.

**Torque, measured June 8, 2026** (AMF-300 gauge at 10 cm; rig, method and the
15-step force/angle table are in the log file):
- **Safe continuous operating torque: ~3.5 N·m**
- **Peak / stall torque: 5.57 N·m**
- Average stiffness ~1.36 N·m/° early, falling to ~0.90 N·m/° near 3.5 N·m
  (see the mixed-unit caveat above)
- Gearbox mechanical efficiency estimated **50–65%** — consistent with printed
  cycloidal expectations
- **Scrub torque required, 45 kg rover with 13 cm wide wheels: ~1.325 N·m**,
  computed with the **contact-patch model, NOT the wheel-radius model**
  → **4.2× safety margin** against the 5.57 N·m stall figure

**Notable measured behaviour — current draw is remarkably low under load.** At
3.36–3.57 N·m the supply drew only **135–166 mA at 12 V (~1.6–2.0 W)**, with no
audible noise and no heat. Current climbs slowly as the PID works harder, then
spikes sharply at the stall boundary (1550 mA). That transition is the reliable
indicator of the true torque limit — watch current, not force.

### What this means for W3 (STM32 UART firmware)
**Already proven, do not re-derive:** packet format, checksum, command bytes,
response decoding, motor behaviour under load.
**Discovered in W3, was not in the June 8 notes:** the driver echoes every
request before replying (see the section above). Cost a bring-up session to
diagnose because it presents as a checksum failure on a reply that is
actually correct.

**Resolved Aug 13, 2026 — the acceptance criterion is met.** HAL UART config,
asynchronous response timing including the two-stage `FD` reply, and response
parsing in C all work against real hardware; see the Aug 13 verification log
above. Still outstanding from the list below: confirming the other three
drivers are set to CR_UART / 38400 / Mstep 8.

**Originally listed as unproven:** STM32 HAL UART configuration;
asynchronous response timing (the driver replies in two stages for `FD` — start
then complete — and `39` reads inside a control loop have latency implications);
parsing responses in C rather than reading hex by eye; and confirming the other
three drivers are set to CR_UART / 38400 / Mstep 8 (address needs no change —
all stay at `0xE0`), since only one unit has ever been on the bench.

### Tooling
Custom serial terminal running on daedalus (built earlier with Claude Design +
Claude Code), used in hex mode — sends raw byte sequences and shows raw responses.


# RobertUN wheel firmware — reference: MCU, pins, CAN and firmware modules

> Reference tier. Moved verbatim from `PROJECT_CONTEXT_WHEEL_FW.md` on 2026-09-26.
> Do not read whole: `grep -n '^#' <this file>` and read the section you need.
> Contents: CAN peripheral and bit timing, pin allocation, timers, firmware modules (debug_uart, can_bus, console), CAN error counters, bus load

## CAN BUS — STM32 PERIPHERAL & BIT TIMING

### STM32 CAN peripheral
- **STM32F4xx uses bxCAN peripheral** (Basic Extended CAN)
  - Full layer 2 implementation in hardware: frame construction, arbitration,
    CRC, ACK, fault confinement, acceptance filters — all in silicon
  - bxCAN outputs logic-level CAN_TX / CAN_RX to the SN65HVD230 transceiver
  - No built-in transceiver — external IC always required (same as Jetson)
- **CAN1 pins on the WeAct board: PB9 (TX) / PB8 (RX).** The alternate CAN1
  mapping PA11/PA12 is wired to the board's USB-C connector and must not be
  used. Use CAN1, not CAN2 — on the F446 CAN2 is a slave peripheral that cannot
  run without CAN1's clock enabled anyway.
- **bxCAN SJW maxes out at 4 tq** (the register field is 2 bits wide), and must
  also be <= BS2. Orion's `sjw 16` therefore does *not* transfer literally to
  the STM32. The rule that carries across is **"set SJW explicitly on every
  node, at the highest value that node's hardware allows"** — not the number 16.

#### Bit timing @ 250 kbps (derived Aug 9, 2026, from 8 MHz HSE)

| Parameter | Value | Notes |
|---|---|---|
| HSE | 8 MHz | crystal on WeAct board, scope-verified |
| SYSCLK | 180 MHz | PLL M=4, N=180, P=2 — see divider note below |
| APB1 (bxCAN clock) | 45 MHz | /4 |
| Prescaler (BRP) | 12 | tq = 266.67 ns |
| Bit Segment 1 (BS1) | 12 tq | HAL: `CAN_BS1_12TQ` |
| Bit Segment 2 (BS2) | 2 tq | HAL: `CAN_BS2_2TQ` |
| SJW | 2 tq | HAL: `CAN_SJW_2TQ` — hardware max here (<= BS2) |
| Total | 15 tq | 15 x 266.67 ns = 4.0 us = 250 kbps |
| Sample point | 86.7% | (1+12)/15 — closest achievable to Orion's 87.5% |

**Why 180 MHz and not the conventional 168 MHz:** 45 MHz on APB1 divides into
250 kbps with a better sample point than 42 MHz does. Consequence: USB's 48 MHz
must come from PLLSAI rather than PLLQ. Irrelevant unless USB is ever needed.

**PLL dividers — record corrected Aug 9, 2026.** An earlier revision of this
table recorded M=8, N=360. Both pairs reach 180 MHz, but they are not
equivalent: M=8 gives a 1 MHz PLL input, M=4 gives 2 MHz. RM0390 requires the
PLL input to be 0.95-2.1 MHz and explicitly recommends **2 MHz to limit PLL
jitter**. The firmware uses **M=4, N=180, P=2**, which is the better of the two
and the pair to replicate to the other five boards in W7. Verify against
`SystemClock_Config()` rather than this table if they ever disagree again.

**Oscillator tolerance check.** df <= SJW/(20 x NBT): STM32 = 2/(20x15) = 0.67%;
Orion = 16/(20x200) = 0.40%. Orion is the binding node. Two crystals at +/-30 ppm
(0.003%) sit two orders of magnitude inside that margin. This is the calculation
that rules out HSI (+/-1%) — see the W1 debrief in the roadmap.

- **bxCAN receives nothing until an acceptance filter is configured AND
  activated.** Default state is all filters disabled. Presents as "TX works
  perfectly, RX is dead" and gets misdiagnosed as wiring or termination. For
  bring-up use filter bank 0, mask mode, ID 0x000 / mask 0x000 (accept
  everything); narrow it once the CAN ID table is real.


## STM32F446RE — PIN ALLOCATION (settled Aug 25, revised Sep 11, 2026)

Complete map for the module node. Everything below is in the `.ioc`. The
encoder and the four DRV lines are wired and exercised under power; **PA2
(IPROPI) was added Sep 11** for the DRV8874 and is configured but not yet
wired.

| Pin | Signal | Peripheral | AF | Task |
|---|---|---|---|---|
| PA0 | UART4_TX | UART4 @ 38400 8N1 | AF8 | MKS SERVO42C steering link |
| PA1 | UART4_RX | UART4 | AF8 | SERVO42C replies |
| PA2 | DRV_IPROPI | **ADC1_IN2**, 12-bit | — | drive current feedback, 28-cycle sample |
| PA4 | DRV_VREF | **DAC1_OUT1**, 12-bit | — | drive current LIMIT; must be set before nSLEEP rises |
| PA8 | MCO1 | RCC, HSE ÷1 | AF0 | 8 MHz clock-out, scope check |
| PA9 | USART1_TX | USART1 @ 115200 | AF7 | `debug_uart` console |
| PA10 | USART1_RX | USART1 | AF7 | console command interpreter |
| PA15 | ENC_A | **TIM2_CH1**, encoder | AF1 | drive-motor quadrature A |
| PB2 | LED_BLINKY | GPIO out | — | heartbeat LED (also BOOT1) |
| PB3 | ENC_B | **TIM2_CH2**, encoder | AF1 | drive-motor quadrature B |
| PB5 | DRV_nSLEEP | GPIO out | — | DRV8874 nSLEEP; **low = disabled** |
| PB6 | DRV_PWM_A | TIM4_CH1, PWM 20 kHz | AF2 | DRV8874 **EN/IN1** (single bridge) |
| PB7 | DRV_PWM_B | TIM4_CH2, PWM 20 kHz | AF2 | DRV8874 **PH/IN2** (single bridge) |
| PB8 | CAN1_RX | bxCAN1 @ 250 kbps | AF9 | rover CAN bus |
| PB9 | CAN1_TX | bxCAN1 | AF9 | rover CAN bus |
| PB12 | DRV_nFAULT | GPIO in, pull-up | — | DRV8874 nFAULT, open-drain, active low |
| PB13 | DIP_SW_0 | GPIO in, pull-up | — | module ID bit 0 (LSB) |
| PB14 | DIP_SW_1 | GPIO in, pull-up | — | module ID bit 1 — **configured by `dipsw_init()`, not the `.ioc`** |
| PB15 | DIP_SW_2 | GPIO in, pull-up | — | module ID bit 2 (MSB) — **configured by `dipsw_init()`, not the `.ioc`** |
| PH0/PH1 | HSE | 8 MHz crystal | — | → 180 MHz PLL (M=4, N=180, P=2) |
| PC14/PC15 | LSE | 32.768 kHz | — | in the `.ioc`, **not enabled** in code |

⚠️ **PB7 was destroyed on the *old* bench board on Sep 16, 2026** (pad shorted
to VDD — see the log entry). The allocation is unchanged and correct, and the
**replacement board passed both pad checks on Sep 19**, so PB6/PB7 stay put.
Kept here because the cause (see task 19) applies to every board. If a future
failure ever forces the PWM off PB6/PB7, the
replacement is **TIM3 on PC6/PC7 (AF2)**: PORTC is completely unused, TIM3 is
free, it is on APB1 at 90 MHz like TIM4 so **PSC 0 / ARR 4499 and every tick
constant in `drive.h` survive unchanged**, and TIM3's TRGO can be driven from
OC4REF to replace the TIM4_CH4 ADC trigger (`T3_TRGO` is a valid ADC1 source).
Do **not** take PA6/PA7 — they are reserved for SPI1 below.

**Off-limits, and why:** PA13/PA14 are SWDIO/SWCLK and the board has no other
debug access. PA11/PA12 are the USB-C connector (this is why CAN1 lives on
PB8/PB9). PB4 is NJTRST — avoided deliberately; PA15/PB3 are the other two
JTAG remnants and *are* used, which is fine under 2-wire SWD but means full
JTAG is gone for good on this design.

**PA4 is still free and is the VREF option** — `DAC1_OUT` driving the
DRV8874's VREF gives a software-programmable current limit (roughly 0–3.3 A on
a 2.2 kΩ IPROPI resistor). Not fitted; a fixed divider is the simpler start.
PA5/DAC2 stays reserved for SPI1_SCK.

**Deliberately kept free:** PA5/PA6/PA7 for SPI1 (software NSS) and PB10/PB11
for I2C2 — the two obvious buses if a per-module IMU ever appears. This is the
reason the encoder did not take PA6/PA7, and the reason TIM2_CH1 is on PA15
rather than PA5: both PA5 and PB3 are SPI1_SCK candidates, and taking both
would have killed SPI1 outright.

### Timers

| Timer | Role | Pins | Notes |
|---|---|---|---|
| TIM2 | Encoder interface, `TIM_ENCODERMODE_TI12` (x4) | PA15, PB3 | **32-bit.** Period `0xFFFFFFFF`, IC filters 15 |
| TIM4 | PWM CH1+CH2, PSC 0 / ARR 4499 → 20.0 kHz | PB6, PB7 | Both channels start at 0%; `__HAL_DBGMCU_FREEZE_TIM4()` set. PB7 verified healthy on the replacement board Sep 19 |
| TIM4_CH4 | ADC trigger only — PWM mode 2, compare at the middle of the drive phase | none | PB9 is CAN1_TX (AF9), so CC4E routes nothing to a pin; configured in `drive_init()`, not the `.ioc` |
| TIM7 | 0.5 s heartbeat tick (LED + CAN frame) | none | Basic timer; PSC 1800 / ARR 25000 at 90 MHz APB1 |
| TIM6 | 1 kHz control tick — runs `drive_on_tick()` (nFAULT latch) | none | Configured and running as of Sep 2026 |
| TIM3 | **free** — the PWM fallback if TIM4 must be abandoned | PC6, PC7 (AF2) | PORTC otherwise unused; APB1 at 90 MHz so PSC 0 / ARR 4499 and every tick constant carries over. `T3_TRGO` from OC4REF replaces the TIM4_CH4 ADC trigger |
| TIM1 | **unusable** | — | All four channels land on PA8/PA9/PA10/PA11, every one taken |

⚠️ **PB7 was destroyed on the old board on Sep 16, 2026** (pad shorted to VDD —
see task 19); the replacement passed both pad checks on Sep 19. Relocating the
PWM to TIM3/PC6-PC7 remains the fallback **only** if a board fails this way
again. Do **not** take PA6/PA7 for it — those are reserved for SPI1.

**Encoder mode is CH1+CH2 only — this is silicon, not a HAL limitation.** The
interface decodes TI1FP1/TI2FP2; `HAL_TIM_Encoder_Init()` writes only `CCMR1`
and the CC1/CC2 bits of `CCER`, never `CCMR2`. Channels 3 and 4 cannot do
quadrature on any STM32 timer. This is what ruled out PA2/PA3 (TIM2_CH3/CH4)
when 32-bit counting was the goal.

**Only TIM2 and TIM5 have 32-bit counters**, and TIM5's CH1/CH2 are PA0/PA1 —
already the SERVO42C UART. TIM2 was therefore the only route to a 32-bit
encoder, which is what justified spending two JTAG-remnant pins on it.

**The 16-bit counter would have wrapped every 11.3 output revolutions**
(65536 / 5777 counts per rev), or ~6.8 s at 100 rpm. The delta-accumulate code
is identical either way and is required regardless, but the 32-bit counter
moves the horizon to ~743,000 revolutions — roughly five days of continuous
running — which takes rollover off the table entirely during W5 PID tuning.

**Encoder input filter set to 15 on both channels:** fDTS/32 with N=8 rejects
glitches shorter than ~2.8 µs. Fastest real edge spacing per channel at 100 rpm
output is ~200 µs, so there is ~70× margin. Drop it toward 0 if counts are ever
missed at high speed.

**`__HAL_DBGMCU_FREEZE_TIM4()` is set, TIM2 is deliberately left running.**
Halting at a breakpoint with PWM still active leaves the wheel turning while
the control loop is frozen, and the delta accumulated on resume is meaningless.
Freezing TIM4 stops the motor with the core. TIM2 stays live on purpose, so
counts still accrue if the wheel is back-driven by hand while halted.

## STM32F446RE — FIRMWARE MODULES (W2)

Three application modules live alongside the CubeMX output. All are added to the
**root** `CMakeLists.txt` user-sources block, not `cmake/stm32cubemx/`, so a
CubeMX regeneration cannot drop them. All integration into `main.c` sits inside
`USER CODE` blocks for the same reason.

Build state Aug 9, 2026: RAM 3.4%, flash 8.5% of the F446RE. Clean under
`-Wall -Wextra -Wconversion -Wshadow`.

### `debug_uart` — non-blocking DMA console on USART1 (PA9 TX / PA10 RX)

115200 8N1. TX is a 1 KB ring drained by DMA2_Stream7; callers never block, so
output is safe from control loops and from interrupt context. RX is a 512 B
**circular** DMA on DMA2_Stream2 whose write pointer is read from the DMA
counter (`__HAL_DMA_GET_COUNTER`) rather than from a callback — bytes are
captured whether or not any ISR got to run.

```
debug_uart_write/puts/printf/write_hex   debug_uart_available/read/peek
debug_uart_flush/tx_pending              debug_uart_take_idle_event
debug_uart_stats/clear_stats             debug_uart_rx_flush
```

**Three CubeMX settings this depends on — all three are silent failures:**
- **USART1 global interrupt MUST be enabled in NVIC.** `HAL_UART_TxCpltCallback`
  is raised from the USART TC interrupt, *not* from the DMA stream interrupt.
  Without it the TX ring stalls after the first transfer: you see the first line
  of output and then permanent silence, which reads exactly like a wiring or
  baud fault. IDLE detection and UART error interrupts are also lost.
- **RX DMA must be Circular.** In Normal mode the stream halts at the first idle
  event and must be re-armed from the callback, losing whatever arrives in the
  gap. `debug_uart_init()` checks this and returns `DEBUG_UART_RX_UNAVAILABLE`
  rather than pretending to work.
- **TX DMA stays Normal.** Circular TX would retransmit the buffer forever.

**Idle-line framing is the point, not a side effect.** The UART idle line is a
hardware frame delimiter, which is how a variable-length reply is known to be
complete without knowing its length in advance — exactly the W3 problem, where
`E0 30 10` returns 8 bytes and `E0 F3 01 D4` returns 3.
`HAL_UARTEx_RxEventCallback` fires on half-transfer and full-transfer as well as
idle, so the handler filters on `HAL_UARTEx_GetRxEventType() ==
HAL_UART_RXEVENT_IDLE`; without that filter a reply straddling a buffer boundary
is reported as two frames, which would show up in W3 as occasional truncated
responses.

**`%f` needs `-Wl,-u,_printf_float`** — newlib-nano omits float printf by
default and prints garbage silently. Already added to `CMakeLists.txt`; needed
for W5 PID telemetry.

### `can_bus` — bxCAN on CAN1 (PB9 TX / PB8 RX) @ 250 kbps

Owns everything CubeMX does not generate: the acceptance filter, starting the
peripheral, and read access to the error state.

```
can_bus_init/send/receive                can_bus_tec/rec/esr/last_error[_str]
can_bus_set_loopback/is_loopback         can_bus_is_error_warning/passive/bus_off
can_bus_get_timing                       can_bus_stats/clear_stats
```

- Filter: bank 0, mask mode, 32-bit, ID 0x000 / mask 0x000, FIFO0,
  `SlaveStartFilterBank = 14`.
- **CubeMX settings that matter:** `AutoRetransmission = ENABLE` (the default
  DISABLE is one-shot mode — an unacknowledged frame is dropped after a single
  attempt, which makes "did it transmit?" much harder to answer during
  bring-up) and `AutoBusOff = ENABLE`, mirroring `restart-ms 100` on Orion.
- `can_bus_get_timing()` reads bit timing back out of `CAN1->BTR` — the
  silicon's own view, not what the source asked for. This catches a CubeMX
  regeneration silently resetting a field, which would otherwise surface as
  intermittent bus errors.
- RX is **interrupt-driven, ISR-to-ring (decided Sep 28, 2026 — see the
  DECIDED section below).** `CAN1_RX0_IRQn` (preemption 1) drains FIFO0 into a
  32-frame software ring; `can_bus_receive()` pops the ring from the main loop.
- `stats` reports the receive-pressure counters: `rx_fifo_full` (FIFO0 reached
  its 3-message depth — the ISR was held off ~3 frame-times, nothing lost),
  `rx_overruns` (a frame was lost in hardware — with the ISR draining, a design
  failure), `rx_ring_dropped` (ring full, frames discarded, counted) and
  `rx_ring_hwm` (ring high-water mark). `rx_overruns` counts **events, not
  frames**: `FOVR0` is sticky rc_w1, so the hardware cannot say how many were
  lost, only that some were.
- Heartbeat on **ID 0x500** (the 0x500-0x5FF telemetry/heartbeat group) at the
  TIM7 rate, so the LED blink and the CAN frame share a cadence. Payload is
  self-describing in `candump`: bytes 0-3 big-endian sequence, then TEC, REC,
  LEC, and a status bitfield (bit0 warning, bit1 passive, bit2 bus-off).
  **Now `0x500 + module_id` (Sep 14, 2026)** — the DIP switch supplies it, and
  identity gates the *transmit*, not just the address: a board reading `0b111`
  sends nothing at all, and the per-frame line says `NO ID`. Without that gate
  an unconfigured board would heartbeat at the base address and collide with
  module 0, which on a six-node bus does not present as "wrong ID" — it
  presents as arbitration chaos.

**Reading `lec` during bring-up:** `lec ack` means the frame went out correctly
but nothing acknowledged it. A transmitter cannot ACK itself, so this says "no
other node is listening", not "this node is broken". A steady `tec 0 rec 0
lec none` is the proof the ACK came back — the on-chip equivalent of Orion's
`berr-counter tx 0 rx 0`.

### `console` — line-based command interpreter

Turns the board into a bench instrument: inject frames, read error registers,
and switch CAN modes with no debugger session and no reflash.

```
help  info  stats  errors  clear  send <id> [hex]
heartbeat [on|off]   monitor [on|off]   loopback [on|off]   reset
enc [sub]   drv [sub]   cfg [key] [val]   mks <sub>   id
telem [on|off|rate <1..100>]
```

- `send` accepts the payload however it is easiest to type — `send 123 DEADBEEF`,
  `send 123 DE AD BE EF` and `send 123 DEAD BEEF` are identical. Odd digit
  counts and >8 bytes are rejected rather than silently truncated.
- **`loopback on` is the solo self-test W1 concluded does not exist on the
  SocketCAN side.** bxCAN loopback stays off the wire and self-ACKs, so
  `send 123 DEADBEEF` returns through the filter and prints. That proves bit
  timing, filter bank 0, the FIFO path and both HAL call paths with no
  transceiver, no cable and no second node. If loopback works and normal mode
  does not, the fault is downstream of the MCU — which splits the search space
  before touching wiring.
- `heartbeat off` / `monitor off` silence async output while typing.
- `info` prints live clocks and the bit timing read back from `CAN1->BTR`.
- **`telem` is the machine-readable half of the console (added Sep 25, 2026).**
  One line per sample, `T,<seq>,<ms>,<duty>,<count>,<milli_rpm>,<mA>,<flags>`,
  flags `1 sync · 2 enabled · 4 fault · 8 saturated · 16 watchdog`. **Integer
  fields only** — `%f` pulls in newlib's float formatter, far too slow at
  100 Hz, so speed goes out as milli-rpm. Emitted from the **main loop**, not
  the TIM6 ISR, because the synchronised ADC read waits on conversions
  triggered once per 50 µs PWM period. **Capped at 100 Hz**: a ~55 byte line at
  100 Hz is ~5.5 kB/s of the 11.52 kB/s available, where 200 Hz would be ~95%
  and lines would start vanishing into `tx_dropped`. `seq` restarts at 0 on
  every `telem on`, so a host detects dropped lines rather than inferring them.
  Turn `monitor off` first — CAN frame lines interleave into the stream.

**The prompt is the frame boundary, and it has no newline.** `execute_line()`
emits `\r\n`, dispatches, then prints `"> "` unterminated, and every printable
character is echoed as typed. So one exchange on the wire is
`<echo>\r\n<output lines>\r\n> `. A host that tests for the prompt as a
*buffer suffix* breaks the moment a `telem` line lands behind it — the two merge
into one run of text and the prompt is never seen again. Consume it in arrival
order instead. Cost an hour Sep 25, 2026, and would have been near-impossible to
diagnose at the bench rather than against a simulated console.

**Terminal line endings — cost real debugging time Aug 9, 2026.** The
interpreter executes on CR or LF. CoolTerm with *Enter Key Emulation* set to
`None` sends no terminator, so commands echoed back correctly while nothing ever
ran — a symptom that looks like a parser bug and is not. `console_poll()` now
also executes on an **idle line** when the burst held more than one byte, which
covers Send-String-style terminals; the >1 byte guard is what keeps interactive
typing from executing a character at a time. Set Enter Key Emulation to `CR`
anyway rather than depending on the fallback.

### Fixed (Sep 28, 2026) — `cmd_errors` read CAN_ESR non-atomically

**Fixed on branch `ISR-to-ring`:** `can_bus_errors()` reads `CAN1->ESR` once and
decodes raw, TEC, REC, LEC and the warning/passive/bus-off bits from that one
snapshot; `errors` and the heartbeat payload both use it. Bench-checked on a
healthy bus only (raw 0, TEC/REC 0, heartbeat bytes 4-6 match `errors`); not
re-observed during bus-off recovery, where the original disagreement appeared.
The original report follows.

`console.c`'s `errors` command called `can_bus_esr()`, then `can_bus_tec()`, then
`can_bus_rec()` — each performing its own read of `CAN1->ESR`. The register can
change between them, so the printed raw value and the decoded fields may
disagree. Observed Aug 10, 2026: raw `0x66000055` (REC 102) printed alongside
`REC : 103`, because REC was moving during bus-off recovery.

Harmless for steady-state inspection, misleading when counters are in motion —
which is exactly when the command matters. **Fix:** snapshot `CAN1->ESR` once
and decode every field from that snapshot. Not urgent; recorded so it is not
rediscovered as a mystery.

### Considered and not adopted — internal pull-up on PB8 (CAN1_RX)

PB8 is currently `GPIO_NOPULL`, as CubeMX generates it. Enabling the internal
pull-up would hold CAN_RX at recessive whenever nothing is driving it.

**The argument for it:** an undriven CAN_RX is the fault described below, and on
the rover it is reachable in normal service — an unpowered transceiver, or a
Bulgin connector working loose at a Rocker-Bogie flex point. With the pull-up,
that fault presents as a quiet node; without it, as a node cycling in and out of
BUS-OFF and generating error frames that disturb the whole bus. The transceiver's
push-pull output overrides a ~40k internal pull-up, so it costs nothing while
things are connected.

**Decision Aug 10, 2026: not adopted** — the `.ioc` stays as ST generates it.
Recorded here with the rationale so the option is not re-derived from scratch,
and so the trade-off is on the table if a loose-connector failure ever shows up
in W7/W8 with six nodes wired.

### CAN error counters — how to read them

**First, the operational rule: do not debug CAN error counters on a node whose
transceiver is not connected and powered.** On Aug 10, 2026 three consecutive
bench runs of *identical* firmware failed three different ways (`ack`, `form`,
`bit-dominant`) because PB8 was left floating. The non-determinism was itself the
diagnosis — a driven input cannot behave differently run to run, which ruled out
firmware before a single register was examined. Full account in
`PROJECT_CONTEXT_WHEEL_FW_LOG.md`.

Reading notes that generalise:

- **`lec bit-dominant`** = transmitted recessive, monitored dominant. Points at
  CAN_RX held low, TX shorted, or a transceiver holding the bus dominant.
- **`lec ack`** = the frame went out correctly and nothing acknowledged it. A
  transmitter cannot ACK itself, so this means no other node is listening.
- **LEC is sticky** — it holds the last error until a new one overwrites it or
  software clears it. Read TEC's *trend* for "erroring right now", not LEC.
- **TEC parks at 128 and never reaches BUS-OFF when a node is alone on the bus.**
  The CAN spec exempts an error-passive node from further TEC increment on ACK
  errors, precisely so a lone node cannot take itself bus-off. `passive` without
  `BUS-OFF` is the correct signature of "nobody else is out there".
- **Going BUS-OFF resets TEC to 0**, and bxCAN then reuses REC to count the 128
  sequences of 11 recessive bits required to rejoin. A falling REC with TEC at 0
  and `BOFF` set is `AutoBusOff` recovery in progress, not a receive problem.
- **TEC/REC survive `HAL_CAN_Stop()` + `HAL_CAN_Init()`.** Only a peripheral or
  system reset clears them, so counters seen after a mode change may predate it.
- **`NO MAILBOX` after exactly three frames is the lone-node signature, not a
  fault** (observed Sep 14, 2026). `AutoRetransmission = ENABLE`, so a frame
  that is never acknowledged is retried *forever* and its mailbox is never
  released. Three mailboxes, three frames, then every subsequent send fails to
  find one. Read it together with `lec ack` and `tec 128`: all three are the
  same single fact, which is that nobody else is on the bus. It disappears the
  moment a second node acknowledges.
  - **Open question for W6:** whether the heartbeat should be one-shot (`NART`)
    instead. A heartbeat retried for seconds is stale by the time it lands, and
    the retry jams the mailboxes that real traffic needs. The counter-argument
    is that auto-retransmit is right for commands. Likely answer: per-frame
    choice, which bxCAN does not offer — so it becomes "which matters more on
    this bus". Do not change it mid-bring-up; the current behaviour is now a
    known-good reference signature.

**Operational rule: do not debug CAN error counters on a node whose transceiver
is not connected and powered.** The numbers are not merely unhelpful, they are
actively misleading, and each of the three signatures above is individually
plausible enough to send you after the wrong fault.

### Bus-load headroom — established Aug 11, 2026

Ramped to bus saturation with `cangen` while the heartbeat ran. **At ~100% bus
load (1,858 f/s, 127,343 frames over 68 s): zero FIFO-full events, zero
overruns, `TEC 0 / REC 0 / lec none` throughout, and 137 heartbeats transmitted
with none dropped.** The ramp topped out on the wire, not on the MCU.

**What it bounds:** `FULL0` sets at 3 messages in FIFO0 and never incremented, so
`worst-case main-loop period < 3 × 538 µs = 1.6 ms` — a bound, not an average,
held for 68 s with no outlier.

⚠️ **What it does NOT prove — read before citing it.** It measured *throughput*
(are frames lost), not **latency** (how long a frame waits before handling) and
not **coupling** (the result is a property of a nearly empty main loop). It is
not evidence that polling is the right architecture — see the open item below.
Method and the full step table are in `PROJECT_CONTEXT_WHEEL_FW_LOG.md`.

### DECIDED (Sep 28, 2026) — interrupt-driven CAN RX, ISR-to-ring

**Decision: the hybrid, merged from branch `ISR-to-ring`.** `CAN1_RX0_IRQn` at
preemption 1 (every other application IRQ stays at 0, so TIM6's 1 kHz control
tick preempts it and is unaffected by bus load) drains FIFO0 into a 32-frame
ring; the main loop pops it. Plan: `docs/plans/ISR-to-ring.md`. Bench
(Sep 27-28): 20000/20000 frames at saturation (~1860 f/s), with and without the
motor at 18% duty; 0 overruns, 0 FIFO-full, ring hwm 1, TEC/REC 0; forced
overflow (`canhold 100`) gave hwm 32 and ~150 dropped, 0 overruns, console and
heartbeat unaffected, RX resumed. Heartbeat jitter max |dev| 0.46-0.74 ms across
the five load conditions, no worse than polled beyond run-to-run spread.
Not verified: the `rx_frames - dropped = delivered` invariant (no delivered
counter in `stats`; `monitor` lines drop bytes on a 32-frame burst). Note the
STM32 bxCAN `loopback` mode still drives the TX pin (a frame sent in loopback
appeared on the wire), despite the console's "off the wire" label.

The Aug 11 analysis below is kept as the record of why.

**Status when open: undecided as of Aug 11, 2026.** The ramp above does not settle it.

**Leading candidate (adopted): the hybrid.** An interrupt-driven FIFO drain that pushes
frames into a software ring, consumed by the main loop. The ISR does one bounded
thing — pull from FIFO0, push to ring, return — and all application logic stays
in main-loop context where it can be single-stepped.

This is **the same pattern `debug_uart` already uses**: hardware and ISR fill a
ring, the main loop drains it. Making CAN symmetric with UART means one mental
model for both paths, which matters when six of these are in a rover and one
misbehaves in the field.

**For interrupts:**
- **Latency is unmeasured and lands in the control path.** With polling, a
  frame's worst-case wait is one loop period. That is jitter, not constant lag,
  so it does not calibrate out. Against a 50 Hz Ackermann command cycle (20 ms),
  several milliseconds of variable delay is a meaningful fraction of a cycle.
- **Polling correctness is contingent on the whole program staying fast.** The
  margin measured today is a property of an almost-empty loop, and every feature
  added between now and December erodes it silently. The failure mode is a
  synchronous MKS retry added in W6 causing intermittent frame loss under
  load — which presents as a wiring or bus problem, exactly the class of
  disguised fault the W1 debrief warns about.
- **CANopen may force it anyway.** `CanOpenSTM32`'s driver layer is built around
  CAN RX callbacks feeding the stack's receive buffers, and CiA 402 cyclic
  synchronous velocity mode is SYNC-timed, where jitter becomes control jitter.
  **Verify against the version actually used** — if it holds, the IRQ is required
  in Phase 2 regardless of what W2 measured.

**For polling:**
- No shared state between ISR and main loop, no critical sections on the frame
  path, no interrupt-priority reasoning (at the time TIM6, TIM7, USART1, UART4
  and the DMA streams all sat at priority 0; CAN1 RX0 now sits at 1).
- Every frame is handled in one context that can be single-stepped.
- Measured to work with margin at bus saturation.

**Standing preference to weigh in:** interrupt-driven is the preferred style on
this project where a correct interrupt solution exists — polling is accepted only
where it is clearly the better engineering answer, not as a default.

### Planned — main-loop period in `stats`

Track min/mean/max main-loop time with the DWT cycle counter and report it in
`stats`. This turns "polling latency" from an argument into a number, and gives
an early-warning signal: the margin can be re-checked after every W3-W6
milestone and watched shrinking **before** it starts dropping frames rather than
after. Worth adding whichever way the RX question is decided.


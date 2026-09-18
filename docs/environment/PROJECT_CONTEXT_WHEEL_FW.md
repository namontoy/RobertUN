# RobertUN — Wheel Controller Firmware Context
**Last updated:** September 18, 2026 (DRV8833 history and the completed NEXT TASKS moved to the log file, 2338 → 1961 lines; MCU board replaced, PB7 re-check pending)

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

## Progress log (most recent first) — brief

One line per entry. Full detail (exact numbers, register values, reasoning chains) is in `PROJECT_CONTEXT_WHEEL_FW_LOG.md`.

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


## STM32F446RE — DEVELOPMENT ENVIRONMENT (daedalus)

### Toolchain decision: CubeMX + CMake + CubeCLT + VS Code

**STM32CubeIDE is not used for this project**, despite ~5 years of prior
familiarity with it. Reasons, in order of weight:

1. **W7 replicates firmware across 6 near-identical nodes.** CubeIDE's
   `.cproject`/`.project` are Eclipse-managed XML that regenerate on every
   settings change and produce diffs no human can review. `CMakeLists.txt` is a
   file you can read.
2. **Headless build over SSH/tmux** matches how the rest of this project is
   worked. CMake + Ninja build from the command line; the IDE wants a GUI.
3. **ST is migrating to VS Code.** As of VS Code extension v2.0.0, CMake project
   generation moved *into* STM32CubeMX (6.11.0+) and the extension no longer
   depends on CubeIDE at all.

The `.ioc` file remains the source of truth either way — the CubeMX graphical
clock tree and pinout are unchanged. Only the generator output target changes.
**In CubeMX Project Manager, Toolchain/IDE MUST be set to `CMake`.** Any other
value and the VS Code extension will not work with the project.

Migration was done in W2 deliberately, while firmware was a blinky — the same
move at W6, with UART/MKS and encoder PID entangled, would cost a week out of
the Oct 4 gate.

### Installed stack (daedalus, verified Aug 9, 2026)

| Component | Version | Path / source |
|---|---|---|
| STM32CubeCLT | 1.22.0 | `/opt/st/stm32cubeclt_1.22.0` |
| arm-none-eabi-gcc | 14.3.1 (GNU Tools for STM32 14.3.rel1) | bundled |
| arm-none-eabi-gdb | 15.2.90 | bundled |
| STM32CubeProgrammer | 2.23.0 | bundled |
| ST-LINK_gdbserver | 7.14.0 | bundled |
| CMake | 4.3.1 | bundled (resolves ahead of Ubuntu's) |
| Ninja | 1.13.2 | bundled |
| STM32CubeMX | standalone | separate download, ST account required |
| VS Code extensions | — | STM32 VS Code Extension, **Cortex-Debug**, CMake Tools, stm32-cube-clangd |

Key paths:
- SVD: `/opt/st/stm32cubeclt_1.22.0/STMicroelectronics_CMSIS_SVD/STM32F446.svd`
- gdbserver: `/opt/st/stm32cubeclt_1.22.0/STLink-gdb-server/bin/ST-LINK_gdbserver`
- toolchain bin: `/opt/st/stm32cubeclt_1.22.0/GNU-tools-for-STM32/bin`

**CubeCLT installs a profile script in `/etc/profile.d/` — you must log out and
back in before any tool resolves.** This is the #1 "I installed it and nothing
works" cause.

**Do NOT also `apt install gcc-arm-none-eabi`.** Two `arm-none-eabi-gcc` on PATH
produce linker errors that make no sense.

Verified compiler flags for this target:
```bash
-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
```
A successful link reports the multilib as `lib/thumb/v7e-m+fp/hard/libc.a`.
**Check this string** — with subtly wrong flags GCC silently falls back to a
soft-float library, which surfaces later as inexplicably slow PID math.

### Board: WeAct STM32F446 Core Board V1.1

- **8 MHz HSE crystal + 32.768 kHz LSE, both populated on board.** This is a real
  crystal, not a Nucleo's ST-LINK MCO — no debugger dependency, nothing to cut
  away when the board goes into the rover. This is what makes the HSE decision
  (W1 debrief) cheap.
- CAN1: PB9 (TX) / PB8 (RX)
- User LED: **PB2 — which is also BOOT1.** Harmless as an output; know it before
  wiring anything external there.
- USB-C is on PA11/PA12 (MCU native USB) — blocks the alternate CAN1 mapping
- BOOT0 key present → dfu-util flashing possible with no probe at all
- **No onboard debugger.** SWD header exposes only 3V3, GND, SWCLK, SWDIO.
- **No SWO pin exposed** → ITM/SWO `printf` unavailable unless PB3 is wired out
  manually. Use a UART for console. For W5 PID telemetry, OpenOCD's RTT support
  is the better option than burning a second UART.

### Module identity: 3-bit DIP switch (decided Aug 9, 2026)

**One firmware binary for all six modules.** Module identity is a property of
the hardware, not of the build: a 3-position DIP switch on each board is read
once at boot and yields a module ID 0–5, from which the firmware derives both
the CAN node ID and the module role.

| ID | Role | Behavior |
|---|---|---|
| 0–3 | Corner | Steering (UART → MKS SERVO42C) + drive (encoder PID) |
| 4–5 | Center | Drive only (encoder PID); steering block inactive |
| 6 | *reserved* | Future module / bench-test mode |
| 7 (`0b111`) | **INVALID** | Halt, blink error pattern, **do not join the bus** |

**Why this matters for W7.** The roadmap originally described W7 as "replicate
firmware to 3 more corners + write a center variant" — six near-identical builds
to keep in sync. With identity in a DIP switch and role derived from it, there
is one binary. The *"which build is on this board?"* failure class disappears
entirely; it would otherwise have surfaced in W8 disguised as a CAN problem,
which is the most expensive place to meet it.

**Electrical convention (don't "simplify" these away):**
- **Internal pull-ups enabled; switches pull to GND.** Closed = 0, open = 1. No
  external resistors needed.
- **`0b111` is deliberately the invalid code**, because it is *also* what you
  read from a board with no DIP switch fitted, a broken connection, or a
  floating input. An unconfigured board therefore fails loudly instead of
  silently impersonating module 7.
- **DIP switches, not solder jumpers or hardwired straps.** Reconfigurable on the
  bench when swapping a board to isolate a fault, and readable by eye without a
  meter — at W8 with six nodes live, "which module does this board think it is?"
  should be answerable by looking, not by attaching a debugger.

**Firmware convention:**
- Read the pins **once in `main()`**, before any peripheral init that depends on
  role, and latch into a variable. Identity must never change mid-run.
- Firmware is identical across all corners in another respect too: the UART link
  to the SERVO42C is point-to-point, so all four steering drivers stay at
  address `0xE0` (see MKS SERVO42C section). Module identity lives **only** in
  the CAN ID and this DIP switch.

**Pins: `DIP_SW_0` = PB13, `DIP_SW_1` = PB14, `DIP_SW_2` = PB15. Implemented
and verified on hardware Sep 14, 2026** (`Core/Src/dipsw.c`). PB13 as the LSB.
PB3 is now the encoder's TIM2_CH2 and is no longer available, which is why the
block sits at PB13–PB15.

**PB13 is in the `.ioc`; PB14/PB15 are configured by `dipsw_init()` instead.**
Same reasoning as `drive_init()` starting its own PWM: a CubeMX regeneration has
already silently emptied a USER CODE block on this project once, and the `.ioc`
is the one file we cannot defend. The pin macros are `#ifndef`-guarded, so
adding those pins in CubeMX later changes nothing.

**Verified Sep 14 on a board with a real switch block** — all eight codes swept,
every bit mapping correctly and every role boundary landing where this table
says; the latch holding at ID 0 while the pins read 1, with the divergence
*reported* rather than acted on; and the transmit gate proven in both
directions (`NO ID` with `lec none` at code 7, `queued` at `0x500` with
`lec ack` at code 0). `lec none` is the strong evidence in the first case: with
no second node, a frame that had actually been attempted would have come back
`lec ack`, so the gate held before the frame ever reached a mailbox.

**The invalid code does not halt — deferred, not dropped.** The table above says
`0b111` halts and blinks. Taken literally that would have bricked every board on
the bench, since the switch block is an HW1 part and until Sep 14 every board
read `0b111`; halting removes the console, which is the only way to bring a
board up. What carries the safety is the transmit gate, and that is implemented:
`dipsw_valid()` is false, the boot banner says so loudly, and nothing goes on
the bus. Revisit the halt in W7, when a board with no identity is a real
assembly error rather than the normal state.

### Debug probes

| Probe | SN | Firmware |
|---|---|---|
| #1 | `37FF71064E573436D7331B43` | V2J46S7 |

- These are **ST-Link/V2, not V2-1** — `Board Name` is empty and no VCP appears
  in `STM32_Programmer_CLI -l`. **Consequence: no virtual COM port comes with the
  probe — console output requires a separate USB-TTL adapter** (on hand; needed
  by W3 to watch MKS SERVO42C protocol responses).
- Label each probe physically with its SN. By W7 there will be six boards, and
  reconstructing the SN→board mapping later is pure wasted time.
- The `serialNumber` field in `launch.json` pins a debug config to one probe.
  Without it the debugger attaches to whichever probe enumerated first — and you
  will single-step the wrong node while convinced the right one is broken.

### `.vscode/launch.json` (working reference)

The ST extension does **not** generate this when you simply open a folder.
Without it, the VS Code play button executes the ELF on the host — Ubuntu's
binfmt_misc hands it to `qemu-arm`, which segfaults on the first Cortex-M
peripheral access. That is not a flashing failure; it means no launch config
exists.

```json
{
  "version": "0.2.0",
  "configurations": [
    {
      "name": "Debug (ST-Link)",
      "type": "cortex-debug",
      "request": "launch",
      "cwd": "${workspaceFolder}",
      "executable": "${workspaceFolder}/build/Debug/<PROJECT>.elf",
      "servertype": "stlink",
      "device": "STM32F446RETx",
      "interface": "swd",
      "v1": false,
      "serialNumber": "37FF71064E573436D7331B43",
      "runToEntryPoint": "main",
      "serverpath": "/opt/st/stm32cubeclt_1.22.0/STLink-gdb-server/bin/ST-LINK_gdbserver",
      "stm32cubeprogrammer": "/opt/st/stm32cubeclt_1.22.0/STM32CubeProgrammer/bin",
      "armToolchainPath": "/opt/st/stm32cubeclt_1.22.0/GNU-tools-for-STM32/bin",
      "gdbPath": "/opt/st/stm32cubeclt_1.22.0/GNU-tools-for-STM32/bin/arm-none-eabi-gdb",
      "svdFile": "/opt/st/stm32cubeclt_1.22.0/STMicroelectronics_CMSIS_SVD/STM32F446.svd"
    }
  ]
}
```

- **Cortex-Debug (marus25) must be installed separately** — the ST extension pack
  does not pull it in as a hard dependency.
- `svdFile` is what populates the Cortex Peripherals view while halted. This is
  how `CAN_ESR` (error counters, last error code), `CAN_TSR` and the filter banks
  get read directly on target — the on-chip equivalent of `berr-counter` on Orion.
- Launch from the **Run and Debug** panel with this config selected, *not* the
  CMake Tools play button in the status bar.
- No `preLaunchTask` yet — build first, then launch.

### clangd vs cpptools

The extension pack ships `stm32-cube-clangd`, which conflicts with Microsoft's
cpptools IntelliSense and produces a repeating warning popup. **Keep clangd,
disable cpptools' engine at workspace level** (`.vscode/settings.json`):

```json
{ "C_Cpp.intelliSenseEngine": "disabled" }
```

Disable the engine, don't uninstall cpptools. clangd is correct here because it
reads `compile_commands.json` and therefore sees the real cross-compilation
flags; cpptools guesses and mis-parses CMSIS headers.

### Gotchas (don't rediscover these)

- **`monitor reset halt` is OpenOCD syntax and fails on ST's gdbserver**
  (`Unknown reset option` / `Protocol error with Rcmd: 05`). ST's server wants
  plain **`monitor reset`**, which resets *and halts at the reset handler*.
- **The ST-Link is exclusive.** `STM32_Programmer_CLI`, `ST-LINK_gdbserver` and a
  VS Code debug session cannot hold it simultaneously. A stray gdbserver left
  running in a background terminal produces connection errors that read exactly
  like a hardware fault.
- **`--specs=nosys.specs` link warnings are expected and harmless:** `_close`,
  `_lseek`, `_read`, `_write` "not implemented and will always fail". Correct on
  bare metal with no filesystem. They disappear individually as you retarget
  (e.g. overriding `_write` for UART `printf`).
- **ModemManager** can grab USB serial devices on connect, causing intermittent
  failures that look like flaky hardware. Fix with a udev rule setting
  `ENV{ID_MM_DEVICE_IGNORE}="1"`.
- If clangd reports missing HAL headers, it hasn't found `compile_commands.json`
  (CMake writes it into the *build* directory). Fix with a `.clangd` file in the
  project root: `CompileFlags: { CompilationDatabase: build/Debug }`. Not needed
  as of extension v2.x — it configured this automatically.
- **MCO2 on PC9 reads 36 MHz, not 180 MHz, and that is correct** — MCO2 sources
  the PLL but runs it through a /5 prescaler. Scope-verified twice. Do not chase
  it as a clock fault.

**The toolchain is not a suspect.** It was verified end to end on Aug 9, 2026 —
PATH, tool versions, hard-float multilib, SWD connect, the GDB chain, reset-and-
halt, a full VS Code build→flash→halt round trip, and APB1 = 45 MHz confirmed on
hardware via MCO and a TIM3 blinky. Any subsequent failure belongs to firmware or
wiring. Step-by-step record in `PROJECT_CONTEXT_WHEEL_FW_LOG.md`.

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

⚠️ **PB7 was destroyed on the bench board on Sep 16, 2026** (pad shorted to
VDD — see the log entry). The *allocation* is unchanged and correct; the board
is being replaced. If a future failure ever forces the PWM off PB6/PB7, the
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
| TIM4 | PWM CH1+CH2, PSC 0 / ARR 4499 → 20.0 kHz | PB6, **PB7 (DEAD — see below)** | Both channels start at 0%; `__HAL_DBGMCU_FREEZE_TIM4()` set |
| TIM4_CH4 | ADC trigger only — PWM mode 2, compare at the middle of the drive phase | none | PB9 is CAN1_TX (AF9), so CC4E routes nothing to a pin; configured in `drive_init()`, not the `.ioc` |
| TIM7 | 0.5 s heartbeat tick (LED + CAN frame) | none | Basic timer; PSC 1800 / ARR 25000 at 90 MHz APB1 |
| TIM6 | 1 kHz control tick — runs `drive_on_tick()` (nFAULT latch) | none | Configured and running as of Sep 2026 |
| TIM3 | **free** — the PWM fallback if TIM4 must be abandoned | PC6, PC7 (AF2) | PORTC otherwise unused; APB1 at 90 MHz so PSC 0 / ARR 4499 and every tick constant carries over. `T3_TRGO` from OC4REF replaces the TIM4_CH4 ADC trigger |
| TIM1 | **unusable** | — | All four channels land on PA8/PA9/PA10/PA11, every one taken |

⚠️ **PB7 was destroyed on Sep 16, 2026** (pad shorted to VDD — see task 19).
Relocating the PWM to TIM3/PC6-PC7 is the fallback if a replacement MCU shows the
same fault. Do **not** take PA6/PA7 for it — those are reserved for SPI1.

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
- RX is polled from the main loop **for now — see the OPEN question below.**
  The API is deliberately context-agnostic: moving to interrupt-driven means
  enabling `CAN1_RX0_IRQn` in CubeMX and calling the same `can_bus_receive()`
  from `HAL_CAN_RxFifo0MsgPendingCallback` — the function body does not change.
- `stats` reports two receive-pressure counters: `rx_fifo_full` (FIFO0 reached
  its 3-message depth — margin gone, nothing lost) and `rx_overruns` (a frame
  was lost). The latter counts **events, not frames**: `FOVR0` is sticky rc_w1,
  so the hardware cannot say how many were lost, only that some were.
- Heartbeat on **ID 0x500** (the 0x500-0x5FF telemetry/heartbeat group) at the
  TIM3 rate, so the LED blink and the CAN frame share a cadence. Payload is
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

**Terminal line endings — cost real debugging time Aug 9, 2026.** The
interpreter executes on CR or LF. CoolTerm with *Enter Key Emulation* set to
`None` sends no terminator, so commands echoed back correctly while nothing ever
ran — a symptom that looks like a parser bug and is not. `console_poll()` now
also executes on an **idle line** when the burst held more than one byte, which
covers Send-String-style terminals; the >1 byte guard is what keeps interactive
typing from executing a character at a time. Set Enter Key Emulation to `CR`
anyway rather than depending on the fallback.

### Known issue — `cmd_errors` reads CAN_ESR non-atomically

`console.c`'s `errors` command calls `can_bus_esr()`, then `can_bus_tec()`, then
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

### OPEN — polled vs interrupt-driven CAN RX

**Status: undecided as of Aug 11, 2026.** The ramp above does not settle it.

**Leading candidate: the hybrid.** An interrupt-driven FIFO drain that pushes
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
  path, no interrupt-priority reasoning (TIM3, USART1 and both DMA streams
  currently all sit at priority 0).
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

### Plant model — measured Aug 26, 2026 (free shaft, no load, slow decay)

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
| PB12 | DRV_nFAULT | **4 — nFAULT** | driver → MCU | open-drain, active low, **needs a 10 kΩ pull-up to 3V3** |
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
| **PMODE** | **not populated — pin left OPEN** | **10 kΩ to 3V3 required** (fitted Sep 16, unverified) |
| nSLEEP → VREF | 10 kΩ | **REMOVED** — VREF driven by PA4/DAC1_OUT1 |
| IMODE → GND | 20 kΩ | unchanged — **still not decoded** |
| R_IPROPI → GND | 2.48 kΩ | **1.474 kΩ** (2.0 kΩ ∥ 5.6 kΩ) |

**Why:** the motor stalls at 5.0 A on the 9.5 V rail, and the stock carrier
could neither measure that (2.957 A ceiling) nor permit it (2.957 A trip). The
modification is small, reversible, and off the spares.

**The scaling as modified:**

```
scale       = A_IPROPI × R_IPROPI = 450 µA/A × 1474 Ω = 0.6632 V/A
ADC ceiling = 3.300 V / 0.6632 V/A                    = 4.975 A
LSB         = 4975 / 4096                             = 1.215 mA
                                                        (823.1 counts/A)

integer form, no float:   I_mA = raw × 4975 / 4096
```

R_IPROPI is the **nominal** parallel value. If the pair reads differently on a
meter, `ISENSE_R_IPROPI_OHM` is the one constant to change — a 1 % error there
is a 1 % error in every current ever logged.

**Removing the 10 kΩ undid the coupling, which was the whole point.** On the
stock carrier VREF followed nSLEEP, so the trip point and the ADC full scale
were the same number and could not be moved apart. They are now independent:

- **R_IPROPI alone sets the CEILING** — 4.975 A, fixed in hardware.
- **VREF sets the TRIP**, anywhere from 0 up to that ceiling, in software.
- One DAC code moves the trip by **1.215 mA — exactly one ADC LSB**, because
  both converters are 12 bits across the same 3.3 V through the same resistor.

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

**Still open: is VREF compared directly, or through an internal divider?** With
VREF under software control this stops being a guess and becomes a measurement:
**set a known trip, stall the motor, and see where the reading plateaus.**

```
drv trip 2000 → plateau at ~2000 mA  →  k = 1, as assumed
drv trip 2000 → plateau at ~1000 mA  →  k = 2, halve every trip
drv trip 2000 → plateau at  ~667 mA  →  k = 3
```

Sweep it — 1000, 2000, 3000 — and the relation should be a straight line
through the origin. If it is, the scaling is confirmed end to end: the ADC and
the DAC agree and every current this firmware reports is trustworthy. **This is
the highest-value hour available on the bench right now.** `drv current` names
the plateau when a reading sits within 5 % of the commanded trip, so it is hard
to mistake for a broken sensor.

### IPROPI — SETTLED Sep 12, 2026: the reading is SUPPLY current

`I_IPROPI = I_OUT × 450 µA/A`, and R_IPROPI converts that to a voltage the ADC
reads. At the carrier's measured 2.48 kΩ: **1.116 V/A**, so 3.3 V at 2.957 A —
full scale against 3.3 V with no op-amp.

**Which current does it report, and when?** IPROPI mirrors the current in the
driver's FETs, and in **slow decay (drive-brake), which is the chosen scheme**,
the bridge spends only D of each 50 µs period pulling current from VM — the rest
of the time current recirculates locally through the low-side FETs. With the
carrier's **20 kΩ IMODE strap the mirror is blanked during that recirculation**,
so a free-running ADC returns a duty-weighted average:

> **`isense_read_ma()` returns SUPPLY current — `I_motor × D`.**

That is precisely the figure HW4's PDB branch sizing needs, and the one the old
resistance calculation and stall test could only bound. It is *not* the motor
current a W5 current inner loop would want; divide by D for that.

**How it was settled (Sep 12, 2026) — stall the shaft.** At stall there is no
back-EMF, so the motor current is pure Ohm's law and neither hypothesis needs a
friction model. At 20% duty, Vm 9.35 V, R_motor 1.90 Ω:

| Hypothesis | Predicted `drv current` | |
|---|---|---|
| Continuous (motor current) | `0.20 × 9.35 / 1.90` = **984 mA** | ✗ |
| Drive-phase only (supply) | `984 × 0.20` = **197 mA** | ✓ |
| **Measured**, 256-sample average, twice | **189, 190 mA** | |

A 5× discriminator landing within 4% of the supply prediction. The residual is
accounted for: the bridge's own R<sub>DS(on)</sub> (~0.16 Ω across the two
conducting FETs) drops ~0.15 V at 950 mA so the motor never sees the full rail,
plus the ~1.4% the module currently reads low from `ISENSE_VDDA_MV` and
`ISENSE_R_IPROPI_OHM` both being uncorrected. Together those close it to ~2.5%.

A free-run duty sweep pointed the same way first and is worth recording as the
cheap pre-test: 20% → 23 mA, 40% → 64 mA, a ratio of **2.9×**. Free-run friction
is Coulomb plus a viscous term rising with speed, so under continuous reporting
the ratio is *capped at 2* however the friction splits; the extra factor of D is
what pushes it past 2. Suggestive, but not conclusive on its own — a
grease-packed 131:1 gearbox could plausibly have superlinear losses. The stall
test is the one that closes it.

**⚠️ The consequence that bites: the trip and the reading are in different
units.** The DRV8874 regulates by comparing the *instantaneous* IPROPI voltage
to VREF, cycle by cycle. Since IPROPI mirrors the drive phase, the regulated
quantity is true **motor** current during drive — so the trip does the right
thing, and `drv trip 3000` really does limit the motor to 3 A. But `drv current`
reports the duty-averaged **supply** figure. So during a plateau sweep the
reported current does **not** plateau at the trip value; it plateaus at

> `trip × D_regulation`, where `D_regulation = trip × R_motor / Vm`
> — i.e. at **`trip² × R_motor / Vm`**, quadratic in the trip.

Read a plateau as though it were the trip itself and the VREF divider will look
wrong when it is not. `drv current` prints `Isup` and the implied `Imotor`
(= `Isup / D`) on separate lines for exactly this reason.

**The ADC is configured so either answer works without reconfiguring.**
Sampling time is **28 cycles**, not the maximum 480: at PCLK2/4 = 22.5 MHz a
full conversion is 28+12 = 40 cycles = **1.78 µs**, which fits comfortably
inside the on-phase even at 25 % duty (12.5 µs). 28 cycles is still ~2× what
a 2.48 kΩ source needs to settle to 12-bit accuracy, so nothing is given up.
Had the sample been set to 480 cycles (21.9 µs) it would have straddled nearly
half the PWM period and PWM-synchronised sampling would have been impossible
without going back to CubeMX.

**The upgrade path, when W5 wants it:** trigger the ADC from **TIM4_CH4's
compare event**. TIM4 is already the PWM timer, and a compare channel generates
its event with no pin configured — TIM4_CH3/CH4 land on PB8/PB9, which are the
CAN pins, but the *internal* event does not need them. That places the sample
at a chosen point in the on-phase and removes the question entirely.

Until that upgrade lands, the bench method is to **average many
software-triggered conversions** — `drv current [n]`, default 32, and the
scatter is binomial sampling noise, not instability — and to record the duty
alongside every reading so the D factor can be divided out. `drv current` prints
duty and decay mode on every line for that reason.

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

## NEXT TASKS — wheel firmware track

(Original numbering preserved for cross-reference with PROJECT_CONTEXT_REST.md)

6. **CAN Bus — STM32 firmware:** ✅ **COMPLETED August 10, 2026** (roadmap W2).
   250 kbps bxCAN, accept-all filter on bank 0, DMA console, loopback self-test,
   termination measured 59.79R, three-way verification against Orion `candump`
   and the CANable with zero error counters. Full detail in the LOG file.
   - Open, non-blocking: `cmd_errors` ESR snapshot (see Known issue above)
   - **Open decision:** polled vs interrupt-driven CAN RX — the Aug 11 load ramp
     bounds throughput only, not latency. Hybrid ISR-to-ring is the leading
     candidate. Settle before W6.

6b. **W4 — drive motor + encoder closed loop (IN PROGRESS, opened Aug 24):**
    Acceptance criterion: encoder counts read correctly and match physical
    rotation — **MET Aug 26**, 8394.9 counts/rev over ten hand turns, 0.1% from
    predicted, with the sign convention recorded on the bench.

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

    Still open:
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
    - ⬜ **Plateau sweep** — `drv trip 1000 / 2000 / 3000`, stall, step duty up,
      record where the reported current flattens, to settle the internal VREF
      divider (k = 1 / 2 / 3). Immune to both pending constant corrections,
      because measured current and commanded trip pass through the same
      R_IPROPI and the same VDDA and the ratio cancels them
    - ⬜ **Apply the two measured constants after the sweep** (held until then so
      nothing changes mid-experiment): `ISENSE_VDDA_MV` 3300 → **3325**,
      `ISENSE_R_IPROPI_OHM` 1474 → **1465**. Net effect is that readings
      currently sit ~1.4% low
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


17. **Motor rail 9.5 V → 12 V (decided Sep 12, 2026)**
    The motor is a **6 V / 12 V** unit run at **9.35 V at the terminals** since
    Aug 26. That figure exists only because the DRV8833 could not exceed 10.8 V.
    The architecture does not change: the **per-motor step-down on each node
    PCB stays**, fed independently from the >13.5 V rail — only its output
    set-point moves from 9.5 V to 12 V.

    **What changes** (R_w 1.87 Ω, V_brush 0.14 V, L 1.70 mH — electrical
    properties, unchanged by the supply):

    | | 9.35 V (now) | 12 V |
    |---|---|---|
    | No-load output speed | ~60 rpm | **~76 rpm** |
    | Stall current, datasheet | — | 5.5 A |
    | Stall current, measured cold | 5.0 A | **6.3 A** |
    | Deadband (slow decay) | ~2.6% duty | ~2.0% duty |
    | Supply draw at a regulated 5 A stall | — | ~3.9 A |

    **On the two stall figures.** The datasheet says 5.5 A at 12 V (implying
    2.18 Ω); the Aug 25 bench work on two motors, cross-checked three ways,
    gives 1.87 Ω + 0.14 V of brush drop, i.e. 6.3 A. These are not in conflict —
    copper rises ~0.39%/°C, so a winding hot from stalling reads ~15% higher
    resistance than the cold one that was measured. **5.5 A is the settled
    figure; 6.3 A is the first instant.** Since every start from rest draws
    stall current momentarily, the cold number is the one the driver sees.

    - ⬜ **Set `drv trip 5000` before raising the rail.** This makes the
      5.5-vs-6.3 question moot, sits below the DRV8874's 6 A peak, and stays
      under the 4.975 A ADC ceiling only in the *supply* reading — the motor
      figure will clip, which is expected and harmless.
    - ⬜ Raise the supply so the **motor terminals** read 12 V, not the supply
      output — there is ~0.10 V of harness drop at light load and more under
      current. The 9.45 V → 9.35 V measurement is the precedent.
    - ⬜ The 5.5 A bench-supply limit **needs no change**: in slow decay the
      supply sees `I_motor × D`, so even a regulated 5 A stall draws ~3.9 A.
    - ⬜ **Peak torque will be trip-limited, not voltage-limited.** Torque ∝
      current, so capping current at 5 A caps stall torque at roughly what
      9.5 V already gave. The real gain from 12 V is **speed, and torque at
      speed** — more voltage headroom to drive current against back-EMF, which
      shifts the whole torque-speed curve out. Worth being explicit about so
      nobody expects a bigger stall number.
    - ⬜ Re-measure the plant at 12 V before W5 tuning. R, L and Ke carry over;
      the duty→speed and duty→current mappings do not. **W5 has not started, so
      this is the right moment** — gains tuned at one rail do not transfer.
    - ⬜ Restate the recorded plant figures with their rail attached, so a
      future reader cannot mistake a 9.35 V number for a 12 V one.

18. **`config` module — BUILT Sep 13, VERIFIED ON HARDWARE Sep 14, 2026**

    `Core/Inc/config.h` + `Core/Src/config.c`, plus a `cfg` console command.
    Eight keys (`vdda_mv`, `r_ipropi`, `a_ipropi`, `trip_ma`, `duty_limit`,
    `rail_mv`, `isense_avg`, `sat_raw`) in an append-only log in flash sector 7
    — 48-byte records at a 128-byte stride, ~10M saves of endurance, hardware
    CRC written last so an interrupted save loses itself and not the previous
    record. Fails to defaults **loudly**, rejects out-of-range rather than
    clamping, and `cfg trip_ma` / `cfg duty_limit` apply live as well as at
    boot. Eight bench checks passed; two bugs found and fixed. The design
    rationale, the check list and both bugs are in the LOG file.

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

19. **⛔ BLOCKER — MCU board replacement, PMODE strap, ground return
    (opened Sep 16, 2026).** Nothing else on the bench runs until this is done.
    PB7 on the old board was destroyed and the conditions that destroyed it are
    still wired up. **The MCU board was replaced Sep 18, 2026; the new one has
    not been checked yet.**

    - ⬜ **Step one, before anything else is wired: check the new board bare.**
      Flash, leave the DRV8874 wiring to PB6/PB7 **disconnected**, and run
      `drv pin`. Bare matters — a pull-up anywhere on the net makes a healthy
      pad read 1 and look identical to the dead one. Pass is:
      ```
      PB6 mode 2 af 2 pupd 0 od 0  ODR 0 IDR 0
      PB7 mode 2 af 2 pupd 0 od 0  ODR 0 IDR 0
      ```
      PB6/PB7 stay where they are — TIM3/PC6-PC7 is a fallback only if the new
      board also fails, and it is not on the table otherwise.
    - ⬜ **Then fit the PMODE pull-up, before re-wiring anything else: 10 kΩ
      from PMODE (pin 16) to 3V3.**
      Not 100 kΩ — against the internal 156 kΩ/44 kΩ divider that reaches only
      ≈1.66 V, 160 mV over the 1.5 V `V_TIH` minimum. **This is a per-board
      schematic item for all six nodes and for HW1, not a bench workaround.**
    - ⬜ **Replace the single DuPont between breadboard PGND and MCU ground**
      with a short, thick, dedicated conductor, separate from the logic ground
      link, sized for stall rather than for the working point.
    - ⬜ **Confirm the mode actually latched**: `drv disable` → `drv enable`
      (PMODE latches on nSLEEP rising), then `drv duty 10` and scope IN1/IN2.
      Slow-decay forward is IN1 constantly high, IN2 PWM'd at 90%. Cross-check
      with `drv duty 50`: a near-stationary wheel there means PMODE is still
      reading low (PH/EN, speed ∝ |2D−1|) and the strap has not taken.
    - ⬜ **Verify the DRV8874 survived.** The MCU clamps first so the driver
      is probably intact, but it took the same event.
    - ⬜ **Re-take every current-regulation result.** Independent half-bridge
      disables internal current regulation, so all `drv trip` / PA4 VREF work
      from Sep 11–16 was inert. The plateau sweep in task 17 is the first thing
      that becomes meaningful again.
    - ⬜ **Re-run `drv iscan` in PWM mode and expect a DIFFERENT shape.** With
      low-side slow decay restored, the samples outside the drive window should
      read **non-zero and decaying**, not the exact zeros seen on Sep 15 — those
      zeros were a high-side decay phase that IPROPI cannot see. The 1.6 µs
      `tDELAY` should also stop biting, since it is waived while the sensed
      low-side FET stays continuously on. Only if structure *survives* this is
      the scope-on-PA2 plan worth running.
    - ⬜ **Remove the temporary `drv pin` command from `console.c`** once the
      new board is verified. It is marked TEMPORARY and nothing depends on it.

## KEY LEARNINGS & GOTCHAS

**A firmware reflash does NOT erase flash sector 7** — confirmed Sep 14, 2026 by
flashing and finding the stored `config` record intact. The toolchain
sector-erases only the regions it writes. This is load-bearing for W7: without
it, every firmware update would silently wipe each node's calibration.

Short, generalised rules. Machine-specific detail belongs in that machine's
section; this is for things that will bite again somewhere else.

- **A mode-select pin left floating is a selected mode, not an absent one — and
  a tri-level one may not select the same mode twice.** Multi-level config
  inputs self-bias through an internal divider (the DRV8874's PMODE: 156 kΩ to
  an internal 5 V over 44 kΩ to GND → ≈1.1 V, the middle band). Leaving it open
  therefore *picks* the middle option, and sits close enough to a threshold that
  a different power-up can latch a different mode — which turns one bug into an
  intermittent one. Strap every mode pin explicitly, including the one whose
  default you believe you want. This cost an MCU on Sep 16, 2026.
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


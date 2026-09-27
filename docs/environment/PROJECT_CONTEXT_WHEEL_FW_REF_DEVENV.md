# RobertUN wheel firmware — reference: development environment

> Reference tier. Moved verbatim from `PROJECT_CONTEXT_WHEEL_FW.md` on 2026-09-26.
> Do not read whole: `grep -n '^#' <this file>` and read the section you need.
> Contents: toolchain, installed stack, board, DIP-switch identity, debug probes, launch.json, gotchas

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


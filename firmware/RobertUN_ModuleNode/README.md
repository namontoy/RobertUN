# RobertUN_ModuleNode

Firmware for one RobertUN rover module node.

**One binary, six boards.** The same image is flashed to all six modules — the
four steering corners and the two centre drive modules. A board's identity comes
from its DIP switches at runtime, not from a separate build (see roadmap W7).

Target: **STM32F446RET6** on a **WeAct STM32F446 Core Board** — not a Nucleo.
It has a real 8 MHz HSE crystal and a 32.768 kHz LSE, no onboard debugger and no
SWO. Flashing and debugging go through an external ST-Link/V2.

## Peripheral map

| Peripheral | Pins | Rate | Purpose |
|---|---|---|---|
| USART1 | PA9 TX / PA10 RX | 115200 8N1 | Debug console (DMA TX + circular DMA RX) |
| CAN1 | PB9 TX / PB8 RX | 250 kbps | Rover bus, via SN65HVD230 |
| UART4 | PA0 TX / PA1 RX (AF8) | 38400 8N1 | MKS SERVO42C steering driver |

Clock tree: HSE 8 MHz → PLL M=4, N=180, P=2 → **180 MHz** SYSCLK.
APB1 = 45 MHz (feeds bxCAN), APB2 = 90 MHz.

UART4 was chosen over the PC10/PC11 mapping to keep the WeAct board's SD-card
pins (PC8–PC12, PD2) free; PC9 was released from MCO2 for the same reason.

## Modules

| File | Responsibility |
|---|---|
| `can_bus.c` | CAN1 init, TX/RX, loopback, error counters, FIFO pressure stats |
| `debug_uart.c` | Non-blocking DMA console — TX ring, circular RX, idle-line framing |
| `console.c` | Line interpreter: `help info stats errors clear send heartbeat monitor loopback mks reset` |
| `mks_servo.c` | MKS SERVO42C UART transport and state machine |
| `uart_events.c` | Sole owner of the weak HAL UART callbacks; dispatches by `huart->Instance` |

Each module carries its own header-level documentation, including the data-flow
diagram and the CubeMX settings it depends on. Read those first.

## Build

Requires **STM32CubeCLT 1.22.0** (supplies CMake, Ninja and arm-none-eabi-gcc).

```sh
export PATH=/opt/st/stm32cubeclt_1.22.0/CMake/bin:\
/opt/st/stm32cubeclt_1.22.0/Ninja/bin:\
/opt/st/stm32cubeclt_1.22.0/GNU-tools-for-STM32/bin:$PATH

cmake --preset Debug
cmake --build --preset Debug
```

Output: `build/Debug/RobertUN_ModuleNode.elf`.
Current footprint — RAM 4904 B (3.74%), flash 59948 B (11.43%).

Debugging is via the `Debug (ST-Link)` configuration in `.vscode/launch.json`,
which uses cortex-debug. The ST-Link serial number is hardcoded there; change it
if you swap probes.

## Regenerating from CubeMX

Open `RobertUN_ModuleNode.ioc`. Hand-written code lives in `USER CODE` blocks and
in the user-sources block of the **top-level** `CMakeLists.txt`, both of which
survive regeneration.

`cmake/stm32cubemx/CMakeLists.txt` does **not** survive — never add sources there.

## Gotchas worth knowing before you touch this

Fully written up in `docs/environment/PROJECT_CONTEXT.md`. The three that cost
real bench time:

1. The SERVO42C **echoes every request** before replying. Strip it, or checksum
   validation silently fails.
2. **Idle-line framing does not work on the MKS link** — the driver echoes in
   software with gaps longer than a character time, so IDLE fires mid-message.
   Framing there is by expected reply length, keyed on function code.
3. **Clearing HAL UART error flags reads DR and steals a byte from the DMA**, and
   re-arming unconditionally causes an error cascade. Both UART drivers gate
   re-arming on an observed-from-hardware `rx_is_running()` check.

There is also a standing rule for this protocol, repeated here because it has
been violated more than once: **always write the full decimal breakdown before
stating a checksum byte**, so the arithmetic can be checked at a glance.

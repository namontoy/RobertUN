# Wheel Firmware — Module Map

Survey of the application modules in `firmware/RobertUN_ModuleNode/Core/Inc`
(HAL vendor headers excluded), cross-checked against the firmware-modules
section of `PROJECT_CONTEXT_WHEEL_FW_REF_MCU.md`.

```mermaid
mindmap
  root((Wheel Firmware))
    Communication
      can_bus
        bxCAN 250 kbps, accept-all filter
        SW+HW error stats, bus-off checks
        RX polled from main loop, loopback self-test
      debug_uart
        USART1 115200, DMA TX/RX rings
        RX position via DMA counter, not callback
        Idle-line framing for variable-length replies
      mks_servo
        UART4, SERVO42C at 38400, address 0xE0
        Async state machine, two-stage replies
        Handles echo quirk and additive checksum
    Motor control
      drive
        TIM4 PWM H-bridge, DRV8833/DRV8874 agnostic
        Command watchdog, duty slew limiter
        Places ADC current-sense trigger
      velocity
        PI-D closed loop, stepped at 50 Hz
        Feedforward from measured plant fit
        Anti-windup freezes, own setpoint watchdog
    Sensing
      encoder
        TIM2 quadrature counter, 8403.2 counts per rev
        Windowed velocity, 50 Hz default
        Sequence number flags fresh vs stale
      isense
        ADC current plus DAC trip on DRV8874 carrier
        Free-running vs phase-synced reads
        VREF must be set before nSLEEP rises
    Persistence
      config
        Flash sector 7 append-only log
        Per-key range check, version check
        Save refused while bridge enabled
    Identity and safety
      dipsw
        3-bit DIP read once at boot, latched
        Derives CAN ID and role
        All-high code is invalid sentinel
    Bench tooling
      console
        Line-based command interpreter on USART1
        Idle-line fallback for bare lines
        Emits machine-readable telem stream
```

## Module list

| Module | Category | Key characteristics |
|---|---|---|
| `can_bus` | Communication | bxCAN on CAN1 @ 250 kbps, accept-all filter; SW stats (tx/rx, dropped, overruns) plus HW TEC/REC/LEC; RX polled from main loop, loopback self-test |
| `debug_uart` | Communication | Non-blocking DMA on USART1, 115200 8N1; RX write position read from DMA counter, not a callback; idle-line framing delimits variable-length replies |
| `mks_servo` | Communication | UART4 driver for SERVO42C, 38400 baud, fixed address 0xE0; async ST_TX/ST_WAIT_REPLY/ST_WAIT_MOTION state machine; handles echo-of-request quirk and additive checksum |
| `drive` | Motor control (mechanism) | TIM4 PWM H-bridge (DRV8833/DRV8874 agnostic), signed per-mille duty, slow/fast decay; owns command watchdog and duty slew limiter; places ADC current-sense trigger for `isense` |
| `velocity` | Motor control (policy) | PI(D) speed control stepped at 50 Hz from `encoder`; feedforward from measured duty-vs-rpm fit; anti-windup freezes on saturation/slewing/fault, own setpoint watchdog |
| `encoder` | Sensing | TIM2 hardware quadrature, 8403.2 counts/output-rev; windowed velocity (default 50 Hz); sequence number distinguishes fresh vs stale reads |
| `isense` | Sensing / safety | ADC current sense + DAC current-limit trip on modified DRV8874 carrier; free-running vs phase-synchronised read paths; VREF must be set before nSLEEP rises (fail-safe) |
| `config` | Persistence | Tunables persisted in flash, editable from console; append-only log in sector 7 (~1024 saves before erase), CRC-checked; per-key range/version check, save refused while bridge enabled |
| `dipsw` | Identity / safety | 3-bit DIP read once at boot and latched; derives CAN ID and role; all-high code is a deliberate invalid sentinel, gates bus join |
| `console` | Bench tooling / HMI | Line-based command interpreter on USART1 (`send`, `drv`, `cfg`, `mks`, `telem`, …); idle-line fallback for bare lines; emits machine-readable `telem` stream |

CubeMX-generated glue (`main`, `stm32f4xx_it`, `stm32f4xx_hal_conf`) is board
entry point / HAL config / interrupt table — not an application module, no
distinct characteristics.

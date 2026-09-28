# Task 6 — CAN RX: interrupt-driven, ISR-to-ring (implementation plan)

Branch `ISR-to-ring`. It merges only if the checks below pass. Approved.

## Context

CAN RX is polled today. `can_bus_receive()` (`Core/Src/can_bus.c:220`) pops one
frame from FIFO0, and the main loop calls it in a drain loop.

The Aug 11 load ramp only bounded throughput: zero FIFO-full events at 1858
f/s, and a worst-case loop period under 1.6 ms. It said nothing about latency.
The Sep 27 heartbeat-jitter bench (`_REF_TASK6_CAN_LATENCY.md`) found no
breakover up to ~1902 f/s: max |dev| went from 0.43 ms to 0.72 ms. That method
cannot separate MCU latency from capture noise.

`_REF_MCU` "OPEN — polled vs interrupt-driven CAN RX" names this hybrid as the
leading candidate, and the project prefers interrupt-driven RX where a correct
solution exists. This change settles task 6 before W6. The polled margin is a
property of a nearly empty loop, and W6 adds MKS work to that loop.

Sources for this plan: task 6 in `_REF_TASKS`, the CAN sections of `_REF_MCU`,
and `can_bus.c` / `can_bus.h`. Two facts come from the earlier survey and are
checked again in step 1: every application IRQ is at priority 0 with
`NVIC_PRIORITYGROUP_4`, and the `.ioc` has no CAN1 NVIC entry.

## What "hybrid" means here

- **Capture is interrupt-driven, and handling stays in the main loop.** The
  CAN1 RX0 ISR does one bounded job: it empties FIFO0 into a software ring and
  returns. Dispatch, printing and any application logic stay in main-loop
  context, where they can be single-stepped. This mirrors `debug_uart`'s ring.
- **There is no polled fallback.** The main loop never reads FIFO0 or `RF0R`
  again.
  - A second reader would race the ISR on `RFOM0` (the FIFO release) and on
    the write-1-to-clear of `FULL0`/`FOVR0`.
  - A fallback that runs only when the IRQ is broken would hide exactly the
    fault the counters are there to show.
  - The fallback is the branch itself: if the tests fail, `main` keeps
    polling.

## Design

### Interrupt, callback and priority

- **Interrupt:** `CAN1_RX0_IRQn` only. It is enabled in CubeMX (step 1), and
  CubeMX generates `CAN1_RX0_IRQHandler` → `HAL_CAN_IRQHandler(&hcan1)`.
- **Notification:** `CAN_IT_RX_FIFO0_MSG_PENDING` (FMPIE0) only.
  - FULL/OVERRUN interrupts are not needed, because the drain samples those
    flags on every pass.
  - RX1 is not needed, because the filter routes everything to FIFO0.
  - SCE (error/status) is out of scope. Errors are read on demand.
- **Callback:** `HAL_CAN_RxFifo0MsgPendingCallback()`, defined in `can_bus.c`
  as a weak-symbol override. It is not in a CubeMX file, so there are no USER
  CODE constraints.
- **Priority: preemption 1.** Today every application IRQ sits at 0.
  - TIM6 (the 1 kHz control tick) stays at 0, so it preempts the CAN ISR and
    its jitter is not affected by bus load.
  - The encoder (TIM2) is a hardware counter with no IRQ in the path.
  - Why 1 is safe: the ISR can wait behind the 0-level ISRs (TIM6, TIM7,
    USART1, UART4, the DMA streams), but FIFO0 holds 3 frames. That is ~1.6
    ms at saturation (3 × 538 µs), far longer than any of those ISRs.
  - SysTick stays at 15. The ISR calls nothing that waits on `HAL_GetTick`.

### Ring

- **Shape:** 32 slots of `can_frame_t` (~16 B each, ~512 B RAM). 32 is a
  power of two, so the index uses a mask.
  - At bus saturation, 32 frames ≈ 17 ms of main-loop stall before the first
    drop. That is most of one 50 Hz command cycle, and ~10× the Aug 11 loop
    bound.
- **Indices:** SPSC with `static volatile uint32_t head, tail`, free-running,
  and `count = head - tail`.
  - `head` is written only by the ISR (producer).
  - `tail` is written only by the main loop (consumer).
- **Publishing:**
  - The producer writes the slot, then `__DMB()`, then `head++`.
  - The consumer copies the slot, then `__DMB()`, then `tail++`.
  - The barrier matters because the slots themselves are not volatile, and
    `volatile` alone does not order non-volatile stores around it. Use the same
    barrier that `debug_uart`'s ring uses, if it differs.
- **Overflow: drop and count, never block.**
  - When `count == 32`, the ISR still pops the frame from FIFO0, so the FIFO
    never backs up and the interrupt doesn't re-fire. It discards the frame and
    increments `rx_ring_dropped`.
  - The oldest frames are kept and the newest are dropped.
- **High-water mark:** the ISR updates `rx_ring_hwm = max(count)` after each
  push.
- **ISR body:** `while (fifo_pop(&f)) { push or drop }`. It drains to empty,
  because FIFO0 is only 3 deep, and doesn't rely on the interrupt re-firing.
- **Code layout:**
  - `fifo_pop()` is today's `can_bus_receive()` body, unchanged and made
    static.
  - The public `can_bus_receive()` keeps its signature but now pops from the
    ring.
  - The main-loop call site (`main.c`, USER CODE 3) doesn't change.
- **Arming:**
  - `can_bus_init()`: call `HAL_CAN_ActivateNotification(&hcan1,
    CAN_IT_RX_FIFO0_MSG_PENDING)` after `HAL_CAN_Start`.
  - `can_bus_set_loopback()`: call it again after its `HAL_CAN_Start`. It is
    idempotent, and this protects against Stop/Init clearing IER.
  - In both, reset `head`/`tail` with the IRQ masked before starting, so no
    stale frames cross a mode change.

### Counters and volatile rules

- **One writer per field.**
  - The ISR writes `rx_frames`, `rx_fifo_full`, `rx_overruns`,
    `rx_ring_dropped` and `rx_ring_hwm`.
  - The main loop writes `tx_frames` and `tx_dropped`.
- **Declaration:** `stats` becomes `static volatile can_bus_stats_t`. The two
  new fields are added to `can_bus_stats_t`.
- **Reading and clearing:**
  - `can_bus_stats()` returning a borrowed pointer is replaced by
    `void can_bus_stats_snapshot(can_bus_stats_t *out)`. It copies with
    `CAN1_RX0_IRQn` masked, so the `stats` line is consistent.
  - `can_bus_clear_stats()` masks the same IRQ around the memset.
  - The masking uses `HAL_NVIC_DisableIRQ` / `EnableIRQ` on `CAN1_RX0_IRQn`
    only, never a global `__disable_irq`, so TIM6 is untouched.
- **Invariant**, checked on the bench:
  `rx_frames == delivered + rx_ring_dropped + in-ring`. With the ring empty and
  no drops, this reads `rx_frames == frames seen by the main loop`.

### Hardware FIFO overrun and ESR

- **FOVR0/FULL0:**
  - Semantics are unchanged: sampled and cleared inside `fifo_pop()`, now in
    ISR context only.
  - `rx_overruns` still counts events, not frames.
  - With the ISR draining, `rx_fifo_full` > 0 now means the ISR was held off
    for about 3 frame-times. `rx_overruns` > 0 is a design failure.
- **ESR, the known issue in task 6:**
  - Add `can_bus_errors(can_bus_err_t *out)`. It reads `CAN1->ESR` once and
    decodes raw, TEC, REC, LEC and the warning/passive/bus-off bits from that
    single read.
  - `cmd_errors` (console.c) and the heartbeat payload both switch to it.
  - The single-field accessors stay, for callers that need only one field, and
    their header comment is updated.

## Files

- `Core/Src/can_bus.c`: ring, ISR callback, notification arming, snapshot and
  clear, ESR snapshot.
- `Core/Inc/can_bus.h`:
  - new stats fields, `can_bus_err_t`, and the new prototypes
  - the "RX IS POLLED" block rewritten
  - stale `PROJECT_CONTEXT.md` pointers changed to `_REF_MCU`
- `Core/Src/console.c`:
  - `stats` prints the new fields from a snapshot
  - `errors` uses `can_bus_errors`
  - a test-only `canhold <ms>` command: the main loop skips the ring drain for
    that long (step 6)
- `Core/Src/main.c`, **inside USER CODE blocks only**:
  - the heartbeat payload uses `can_bus_errors`
  - the drain honours `canhold`
- Generated by CubeMX (the user regenerates, and I don't touch them): the
  `.ioc`, `stm32f4xx_it.c/.h`, and the CAN MSP NVIC lines.

## Steps

Bench steps are given to the user one at a time, and each step is checked
before the next.

**Stop rule:** if any check fails or shows something unexpected, stop and
report the observed output. Don't improvise a fix, don't re-run until it
passes, and don't move on. If hardware is damaged, the session ends.

Builds use the project filter (`grep -E 'error|warning|FLASH|RAM'`). Console
command names (`stats`, `errors`, `monitor`, `send`, loopback) are confirmed
against `console.c` at step 2, before first use.

0. **Pre-flight.** On the `ISR-to-ring` branch, the only change in the working
   tree is the replaced plan file. Commit it.
   - Check: `git status` is clean.
   - Baseline build: note the FLASH and RAM figures.
1. **CubeMX (user).** Do the change below, then click "Generate Code".
   - Where: Connectivity → CAN1 → NVIC Settings.
   - What: tick "CAN1 RX0 interrupts" and set preemption priority 1.
   - Check `git diff --stat`: changes appear only in the `.ioc`,
     `stm32f4xx_it.c/.h` and the CAN MSP/NVIC init.
   - Check that the USER CODE blocks are intact.
   - Check that `CAN1_RX0_IRQHandler` exists and the other IRQ priorities are
     unchanged.
   - Build: 0 errors, no new warnings.
   - Flash, then send `cansend can0 123#11` from orion. `stats` shows
     `rx_frames` +1. RX still works, because the NVIC is enabled but IER is
     not yet armed.
2. **ESR snapshot.** Code `can_bus_errors`, switch `cmd_errors` and the
   heartbeat to it, then build and flash.
   - Check `errors` on a healthy bus: `tec 0`, `rec 0`, `lec none`.
   - Check that the raw ESR value's REC/TEC bytes match the decoded fields.
   - Check that the heartbeat's bytes 4–6 in `candump` on orion match
     `errors`.
3. **Stats hardening, still polled.** Make `stats` volatile, add the snapshot
   and the masked clear, and add the `rx_ring_dropped` / `rx_ring_hwm` fields
   (reading 0). Build and flash.
   - Clear `stats`, then send 10 × `cansend` from orion.
   - Expected: `rx_frames 10`, `rx_fifo_full 0`, `rx_overruns 0`,
     `rx_ring_dropped 0`, `rx_ring_hwm 0`.
4. **Ring and ISR.**
   - Code: `fifo_pop`, the ring, the callback, arming in init and loopback, and
     the public `can_bus_receive` popping the ring.
   - Build: RAM grows by about 0.5 KB, FLASH by a few hundred bytes. Flash.
   - Check: after `monitor on`, one `cansend can0 123#DEADBEEF` from orion
     prints exactly one `can rx` line. `stats` shows `rx_frames 1`,
     `rx_ring_hwm 1`, `rx_ring_dropped 0`.
   - Check: loopback on, `send` one frame, and it is received. Loopback off,
     `cansend` from orion, and it is still received. This confirms the
     re-arm.
5. **Bus load from orion, at saturation.**
   - `stats` clear, `monitor off`.
   - On orion: note the `ip -s link show can0` TX packet count, then run
     `cangen can0 -g 0.45 -I 100 -L 8 -n 20000`.
   - Expected: `rx_frames` = 20000, and it equals the delta in orion's TX
     count. `rx_ring_dropped 0`, `rx_overruns 0`, `rx_fifo_full 0`,
     `rx_ring_hwm` in single digits, `errors` still `tec 0` / `rec 0` /
     `lec none`, and no heartbeat missing in orion's `candump`.
   - Repeat once with the motor held at 18% duty (the `_REF_TASK6` motor
     procedure). Same expected values.
6. **Overflow behaviour.**
   - During the same `cangen` stream, send `canhold 100`.
   - Expected: `rx_ring_hwm 32` and `rx_ring_dropped` > 0 (about 150 at
     ~1860 f/s).
   - Expected: `rx_overruns 0`, because the ISR kept FIFO0 empty.
   - Expected: no missed heartbeat, and the console stays responsive, which
     shows nothing blocked.
   - Expected: after the hold, the ring drains and new frames arrive normally.
   - Invariant: `rx_frames` − `rx_ring_dropped` = frames the main loop saw.
7. **Comparison with the polled data.**
   - Repeat the five `_REF_TASK6_CAN_LATENCY` conditions, with the same method
     and the same capture host as before: none, `-g 5` motor off, `-g 2` / `1` /
     `0.45` at 18% duty.
   - Report n, the mean interval and max |dev| next to the polled rows.
   - Expected: no row worse than its polled counterpart beyond run-to-run
     spread.
   - That method can't resolve RX latency, so the pass criterion is steps 1–6.
     Step 7 shows the change did no harm. **The user decides the merge.**
8. **On merge only.** Update the records:
   - `_REF_MCU`: mark the OPEN section decided, fix the TIM3→TIM7 and stale
     priority-list drift, and mark the ESR known issue fixed.
   - `_REF_TASKS`: close task 6.
   - `_REF_TASK6_CAN_LATENCY`: add the ISR rows.
   - LOG entry and hot-file line, then commit.

## Out of scope

- DWT main-loop period in `stats` (a separate planned item).
- Narrowing the acceptance filter.
- An SCE/error interrupt.
- The one-shot (NART) heartbeat question.
- CANopen.

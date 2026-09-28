Plan: CAN RX, ISR-to-ring (task 6)                                                           │
│                                                                                              │
│ Deliverable: this plan copied verbatim to docs/plans/ISR-to-ring.md (new                     │
│ directory). No code changes in this task.                                                    │
│                                                                                              │
│ ---                                                                                          │
│                                                                                              │
│ ISR-to-ring CAN RX — implementation plan (task 6)                                            │
│                                                                                              │
│ Context                                                                                      │
│                                                                                              │
│ CAN RX on the wheel node (STM32F446 bxCAN, CAN1, 250 kbps) is fully polled                   │
│ today. The main loop runs while (can_bus_receive(&rx))                                       │
│ (Core/Src/main.c ~304–317) after the console, MKS, the reports and the                       │
│ heartbeat. FIFO0 holds only 3 frames, so any main-loop stall longer than about               │
│ 3 frame times (about 1.6 ms at the Aug 11 saturation rate of about 1858 fps)                 │
│ loses frames. The only sign of that loss is the sticky FULL0/FOVR0 flags                     │
│ sampled in sample_fifo_flags().                                                              │
│                                                                                              │
│ The 2026-09-27 latency ramp found no breakover up to about 1902 fps with the                 │
│ motor at 18% duty. It also found that the candump heartbeat-jitter method sits               │
│ on its noise floor (_REF_TASK6_CAN_LATENCY.md). So the polled path has not                   │
│ been shown to fail, but nothity depends on how                  │
│ long the main loop's worst iteration takes, and that grows with every feature                │
│ added. This plan settles taskven RX. A short ISR                │
│ empties the hardware FIFO into a software ring, and the main loop drains the                 │
│ ring at its own pace. Loss beng depth and is                    │
│ counted above it.                                                                            │
│                                                                 │
│ What "hybrid" means here                                                                     │
│                                                                 │
│ "Hybrid" means an interrupt producer with a polled consumer, the same                        │
│ split debug_uart already usesat touches FIFO0.                  │
│ The main loop never reads the hardware and only pops from the ring through the               │
│ unchanged can_bus_receive() A                                   │
│                                                                                              │
│ There is no polled fallback o                                   │
│                                                                                              │
│ - Two readers of FIFO0 (ISR aFOM0 (release                      │
│   mailbox) and could release a frame the other side had not copied yet.                      │
│ - A runtime switch doubles th                                   │
│                                                                                              │
│ The A/B baseline is the pre-ca feature flag.                    │
│                                                                                              │
│ Design                                                          │
│                                                                                              │
│ Interrupt, callback, priority                                   │
│                                                                                              │
│ - IRQ: CAN1_RX0_IRQn only, enhe generated                       │
│   CAN1_RX0_IRQHandler in stm32f4xx_it.c calls HAL_CAN_IRQHandler(&hcan1).                    │
│ - Notification: HAL_CAN_ActivT_RX_FIFO0_MSG_PENDING) only. Do   │not enable                                                                                 │
│   CAN_IT_RX_FIFO0_FULL or CANose enabled,                       │
│   HAL_CAN_IRQHandler clears FULL0/FOVR0 itself before our code can count                     │
│   them.                                                         │
│ - Callback: override the weak HAL_CAN_RxFifo0MsgPendingCallback                              │
│   (USE_HAL_CAN_REGISTER_CALLBbody:                              │
│   a. Call sample_fifo_flags() (moved from the consumer; it is ISR-safe                       │
│      because it only reads RF                                   │
│   b. Loop while (HAL_CAN_GetRxFifoFillLevel(..FIFO0) > 0): call                              │
│      HAL_CAN_GetRxMessage and. This drains all                  │
│      3 slots in one entry.                                                                   │
│ - Priority: NVIC group 4 (preQ is at 0:                         │
│   - TIM6_DAC (the 1 kHz control tick, drive_on_tick/velocity_on_tick);                       │
│   - TIM7 (heartbeat tick);                                      │
│   - UART4, USART1;                                                                           │
│   - DMA1_Stream2/4, DMA2_Stre                                   │
│                                                                                              │
│   CAN1_RX0 goes at 1, so it nk or the                           │
│   UART/DMA handlers and waits at most one of their bodies. TIM2 (encoder) is                 │
│   in hardware counter mode wid. SysTick stays at                │
│   15.                                                                                        │
│ - Worst-case budget: one ISR  (about 16 B                       │
│   each), which is a few µs. At 1858 fps that is under 1% CPU. The control                    │
│   tick's jitter grows by at my when the CAN ISR                 │
│   is already running when TIM6 fires. Priority 0 preempts, so that never                     │
│   actually happens; the cost                                    │
│                                                                                              │
│ Ring                                                            │
│                                                                                              │
│ - static can_frame_t rx_ring[                                   │
│   CAN_RX_RING_SIZE 64 (about 1 KB of RAM) and                                                │
│   _Static_assert((CAN_RX_RING == 0, ...), as                    │
│   in debug_uart.c.                                                                           │
│   - 64 frames ≈ 34 ms of back which is about 20×                │
│     the FIFO's 1.6 ms.                                                                       │
│ - Indices: static volatile ui by the ISR) and                   │
│   static volatile uint32_t rx_tail; (written only by the main loop). Both                    │
│   are free-running, and a slo                                   │
│   - Fill level = head - tail (unsigned wrap is safe).                                        │
│   - Single producer, single cpath. 32-bit                       │
│     aligned loads and stores are atomic on Cortex-M4.                                        │
│ - Ordering: the producer writen                                 │
│   rx_head = h + 1. The consumer reads rx_head, then __DMB(), copies the                      │
│   slot, then __DMB(), then rx                                   │
│ - Overflow: drop and count, never block, never overwrite. When                               │
│   head - tail == SIZE, the ISssage, discards                    │
│   the frame and increments rx_ring_dropped.                                                  │
│   - The frame must still be pstays non-zero                     │
│     and the IRQ refires forever.                                                             │
│   - The newest frame is the od are never                        │
│     overwritten.                                                                             │
│ - rx_ring_hwm: the ISR updatel)), so the                        │
│   real margin shows under load.                                                              │
│                                                                 │
│ Stats and volatile rules                                                                     │
│                                                                 │
│ - can_bus_stats_t gains rx_ring_dropped and rx_ring_hwm.                                     │
│ - Ownership:                                                    │
│   - ISR-owned: rx_frames (counted on successful push), rx_ring_dropped,                      │
│     rx_ring_hwm, rx_fifo_full                                   │
│   - Main-loop-owned: tx_frames, tx_dropped.                                                  │
│                                                                 │
│   The static stats instance becomes volatile (project rule: ISR-written                      │
│   statics are volatile).                                        │
│ - Invariant for tests: frames sent on the bus = rx_frames +                                  │
│   rx_ring_dropped (+ frames s is 0 at rest).                    │
│   FULL0/FOVR0 should now stay 0 under any load the ISR can keep up with.                     │
│ - can_bus_stats() changes fro                                   │
│   void can_bus_stats_snapshot(can_bus_stats_t *out). It copies the struct                    │
│   under __disable_irq()/__ena2 idiom), so                       │
│   the stats line in cmd_stats is internally consistent.                                      │
│ - can_bus_clear_stats() runs k. It also resets                  │
│   rx_ring_hwm to the current fill, not to 0.                                                 │
│                                                                 │
│ FOVR, ESR and error counters                                                                 │
│                                                                 │
│ - FULL0/FOVR0 stay counted by sample_fifo_flags(), now called at the top of                  │
│   the ISR. With the ISR at pruns means the CAN                  │
│   ISR was held off for more than 3 frame times by priority-0 handlers. That                  │
│   is a real finding; stop andw).                                │
│ - Non-atomic ESR (task 6 known issue): each accessor (can_bus_tec,                           │
│   can_bus_rec, can_bus_last_e re-reads                          │
│   CAN1->ESR, so one printed line can mix two register states. The fix:                       │
│   - Add can_bus_esr_t can_bus CAN1->ESR                         │
│     once and decodes tec, rec, lec, ewgf, epvf and boff from                                 │
│     that one value.                                             │
│   - cmd_errors (console.c ~337–353) and the heartbeat payload                                │
│     (main.c ~267–297) each taits fields.                        │
│   - The old accessors stay as thin wrappers, for any other caller.                           │
│ - The SCE (status change/errostate is                           │
│   already visible from ESR on demand, and an error IRQ can storm during                      │
│   bus-off. That stays out of                                    │
│                                                                                              │
│ Loopback switch                                                 │
│                                                                                              │
│ can_bus_set_loopback() runs Hfilter →                           │
│ HAL_CAN_Start. HAL_CAN_Init does not preserve IER, so the new sequence is:                   │
│                                                                 │
│ 1. HAL_CAN_DeactivateNotification(...MSG_PENDING)                                            │
│ 2. Stop / Init / filter / Sta                                   │
│ 3. HAL_CAN_ActivateNotification(...MSG_PENDING)                                              │
│                                                                 │
│ The ring is not flushed; frames already queued are still valid.                              │
│                                                                 │
│ Consumer                                                                                     │
│                                                                 │
│ bool can_bus_receive(can_frame_t *out) keeps its signature. It returns false                 │
│ if the ring is empty; otherwiances the tail. The                │
│ main-loop drain loop in main.c does not change.                                              │
│                                                                 │
│ Files to change                                                                              │
│                                                                 │
│ - RobertUN_ModuleNode.ioc, plus the regenerated Core/Src/stm32f4xx_it.c                      │
│   and Core/Src/stm32f4xx_hal_ 156: NVIC                         │
│   priority 1 and enable). After regeneration, diff every USER CODE block.                    │
│ - Core/Src/can_bus.c: the rinflags moved into                   │
│   the ISR, can_bus_receive popping from the ring, snapshot, clear, ESR                       │
│   snapshot, and the loopback                                    │
│ - Core/Inc/can_bus.h: the new stats fields, can_bus_esr_t, the snapshot                      │
│   APIs, and the header commen STILL AN OPEN                     │
│   DECISION" rewritten to describe ISR-to-ring.                                               │
│ - Core/Src/main.c: after MX_C                                   │
│   HAL_CAN_ActivateNotification (USER CODE block), and switch the heartbeat                   │
│   payload to the ESR snapshot                                   │
│ - Core/Src/console.c:                                                                        │
│   - cmd_stats uses the snapshd ring hwm;                        │
│   - cmd_errors uses the ESR snapshot;                                                        │
│   - add the diagnostic stall                                    │
│                                                                                              │
│ Steps                                                           │
│                                                                                              │
│ Build: cmake --build --presetwarning|FLASH|RAM' | tail -40      │
│ Flash: STM32_Programmer_CLI -c port=SWD -w build/Debug/RobertUN_ModuleNode.elf -v -rst       │
│ Console: /dev/ttyUSB0, 115200                                   │
│ Bus load: from orion, or from daedalus via the CANable. can0 is up at                        │
│ 250000 in both cases (see _RE                                   │
│                                                                                              │
│ Load command, used in steps 0                                   │
│ cangen can0 -g 0.45 -I 100 -L 8 -n 10000 -i                                                  │
│ This sends exactly 10000 framach load run,                      │
│ ip -s link show can0 must show TX errors 0 on the host side.                                 │
│                                                                 │
│ Motor state: off unless a step says otherwise. Before any bus-load step, run                 │
│ clear on the console.                                           │
│                                                                                              │
│ Take the steps one at a time.check result and                   │
│ wait for the go-ahead before starting the next.                                              │
│                                                                 │
│ 0. A baseline (current polled firmware). Record git rev-parse HEAD.                          │
│    Flash that build. Run cleats and                             │
│    errors.                                                                                   │
│    Check: this step records nNote rx_frames                     │
│    (expected 10000), FIFO-full and overrun, and tec 0 rec 0 lec none.                        │
│ 1. CubeMX: enable CAN1 RX0 IRhe code.                           │
│    Check:                                                                                    │
│    - git diff shows only the                                    │
│      CAN1_RX0_IRQHandler in stm32f4xx_it.c, and                                              │
│      HAL_NVIC_SetPriority(CANIC_EnableIRQ in                    │
│      HAL_CAN_MspInit. Every USER CODE block is unchanged.                                    │
│    - The build has 0 errors a                                   │
│    - stats after the load command matches step 0. No notification is                         │
│      active yet, so behaviour                                   │
│ 2. ESR snapshot. Add can_bus_esr_snapshot() and switch cmd_errors and                        │
│    the heartbeat to it.                                         │
│    Check:                                                                                    │
│    - It builds and flashes.                                     │
│    - The errors output format is unchanged and reads tec 0 rec 0 lec none.                   │
│    - heartbeat on, then canduy ~500 ms with                     │
│      the same payload layout as before.                                                      │
│ 3. Ring, ISR producer, ring clear, loopback                     │
│    notification toggle.                                                                      │
│    Check:                                                       │
│    - It builds; the RAM figure rises by about 1 KB. It flashes.                              │
│    - At rest, stats shows 0 fg hwm 0.                           │
│    - loopback on, monitor on, send 123 DEADBEEF: the monitor prints the                      │
│      frame with ID 123 and da = 1.                              │
│    - loopback off, then send 123 DEADBEEF again: candump on the host shows                   │
│      the frame, which proves  re-init.                          │
│    - Load command, with clear beforehand: rx_frames = 10000, and                             │
│      ring dropped 0, FIFO-fulmall (single                       │
│      digits expected). errors reads tec 0 rec 0 lec none.                                    │
│ 4. Diagnostic stall <ms> commn loop on                          │
│    HAL_GetTick() with interrupts enabled, capped at 500 ms, and is                           │
│    refused unless drv is coas drop-and-count.                   │
│    Check: it builds and flashes. stall 10 on an idle bus returns after                       │
│    about 10 ms and all counte                                   │
│ 5. Drop-and-count proof. Run clear. Start the load command, and about                        │
│    1 s in, run stall 100 on t, run stats.                       │
│    Check:                                                                                    │
│    - rx_frames + ring dropped                                   │
│    - ring dropped ≈ 190 − 64 ≈ 125 (accept 100–150).                                         │
│    - ring hwm = 64.                                             │
│    - FIFO-full 0, overrun 0: the ISR kept draining the FIFO during the                       │
│      stall.                                                     │
│    - tec 0 rec 0 lec none.                                                                   │
│ 6. B run with the motor loade                                   │
│    drv clearfault, drv duty 18, then hold ≥10 s. Run clear, the load                         │
│    command, then stats. After                                   │
│    Check:                                                                                    │
│    - rx_frames = 10000, ring un 0, 0 bus                        │
│      errors.                                                                                 │
│    - drv reports no fault.                                      │
│ 7. A re-run (ABA). Reflash the step 0 commit and repeat step 6's motor                       │
│    and load sequence. Then re                                   │
│    Check: the counters match step 0's pattern. This gives the A/B/A result                   │
│    for the log.                                                 │
│ 8. Close task 6. Update the can_bus.h header comment. Add the                                │
│    stats/stall commands to _Record the                          │
│    steps 0/3/5/6/7 numbers there. Record the decision and this plan's outcome                │
│    in _REF_TASKS (task 6), _Rnd the log, and                    │
│    add one line to the hot file. Then commit.                                                │
│                                                                 │
│ Stop rule                                                                                    │
│                                                                 │
│ If any check fails, or reads something the step did not predict, stop and                    │
│ report. Report the step numbeput lines                          │
│ involved and the expected values. Do not change code, retry with different                   │
│ parameters or work around thees. Examples:                      │
│ - a USER CODE block altered by regeneration;                                                 │
│ - a new warning;                                                │
│ - rx_frames + ring dropped ≠ 10000;                                                          │
│ - any non-zero overrun in ste                                   │
│ - a non-zero tec/rec;                                                                        │
│ - a motor fault.                                                │
│                                                                                              │
│ If the hardware is damaged, t

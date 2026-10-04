# W6 — implement the plain-CAN command set (`docs/can_cmds.md`) on branch `w6-can-cmds`

## Context
Task 1 settled plain CAN. `docs/can_cmds.md` now uses `ID = (type<<4)|addr`, with 0 = broadcast.
The next task is one full corner node: a CAN command comes in, and both drive (velocity PID) and
steering (MKS) respond. Today's firmware already has a working bxCAN RX path that captures frames
in an ISR into a ring (32 frames), plus `can_bus_send()` and the heartbeat. But nothing decodes
commands: the main-loop drain (`main.c:308-321`, in USER CODE) only prints frames for `monitor`.
This plan adds the command layer in phases. Each phase is bench-verified before the next one starts.

## Decisions (user)
- Steering position = **sum of commanded pulses**. Zero is the position at boot / STEER_ENABLE
  (the wheel is aligned by hand). STEER moves by the difference to the target, and STATUS_STEER
  reports the tracked value. This settles spec Q3 for this branch.
- **§7.3 ownership** is implemented: stops always work from either side, and motion commands from
  the non-owner are refused (`UART_OWNS` on CAN, a message on the console). Settles Q13.
- Console **`estop clear`** clears the ESTOP latch, only when the loop is off (same rule as ARM 3).
  Settles Q18.
- Kept as in the spec: coast on stop (Q14), heartbeat unchanged (Q2), auto-retransmit stays on (Q8),
  STATUS_DRIVE at the control-step rate, 50 Hz (Q12), SPEED ±100 rpm (Q4), RESERVED acts like
  CORNER (Q5).

## Architecture
New module **`Core/Src/can_cmd.c` / `Core/Inc/can_cmd.h`** (not CubeMX-owned). It is added to
`target_sources` in `CMakeLists.txt:51-64`. Everything runs in main-loop context.
- `can_cmd_init()` after `can_bus_init` (in a `main.c` USER CODE block).
- `can_cmd_handle(const can_frame_t *f)` is called from the existing drain loop at
  `main.c:310-321`. The `monitor` print is kept.
- `can_cmd_poll()` runs once per main-loop pass. It handles STATUS_DRIVE/STEER TX, FAULT edge
  detection, ESR polling (§7.2), and steering-move completion.
- Inside `can_cmd_handle`: an O→N type check, then addr filter (own or 0) → DLC check → CRC-8 →
  counter (§5.1) → role/latch/ownership checks → action → CMD_RESULT (§6.2).
- **CRC-8/SAE-J1850**: poly 0x1D, init 0xFF, xorout 0xFF. Input is the ID (u16 LE), then
  PROTO_VER, then the payload minus the CRC byte (for CFG_REQ that means bytes 0–2 and 4–7).
  Check value "123456789" → 0x4B. A console `can crc` self-test prints it; the value also goes
  into spec §5.2.
- **Motion guard** (new `motion.c`/`.h`, small): ESTOP latch and owner (NONE/UART/CAN).
  `motion_estop()` does `velocity_disable` → `drive_coast` → `mks_stop` (CORNER), then latches.
  Console handlers for `vel on`, `vel target`, `drv enable`, `drv duty`, `mks move`, `mks deg`
  call `motion_claim(UART)`. They are refused while latched or while CAN owns motion. Stops
  release ownership.
- **Shared cfg live-apply**: move the per-key apply block out of `cmd_cfg`
  (`console.c:1871-1975`, plus `cfg_apply_live` at 1707) into `config_apply_live(key)` in
  `config.c`. The console and CFG_REQ then use the same path. The console's behaviour must
  stay byte-identical.
- **MKS completion contention**: `console_report_mks()` (`console.c:2456`) takes every completion.
  Add a requester tag so completions of moves that `can_cmd` started go to `can_cmd`, not the
  console.
- Reused APIs: `velocity_enable/disable/set_setpoint/saturated/measured/output/timeout_expired`;
  `drive_enable/coast/brake/set_limit/set_ramp/set_ramp_floor/clear_fault/fault_latched`;
  `isense_set_trip_ma/trip_min_ma/trip_max_ma`; `config_get/set/save/revert/reset_key/reset_all/
  min/max/default`; `mks_enable/stop/move_pulses/busy`; `dipsw_id/role/valid`;
  `can_bus_send/errors/stats_snapshot`; `encoder_velocity_seq()` to detect each control step.

## Host tool
`tools/bench/cancmd.py` (python-can, socketcan `can0` on daedalus via the CANable) builds each frame
type with its counter and CRC, prints the replies decoded, and has a `--watch` mode for STATUS/FAULT.
It reuses the CRC table from a small `canproto.py` shared with the tests. Only numbers are printed,
never raw dumps. Bring-up of `can0` follows `_REF_TASK6_CAN_LATENCY`.

## Phases (each ends with build + flash + one bench check + commit)
1. **Skeleton + safety.** can_cmd and motion modules, CRC, CMD_RESULT TX, address/DLC filter,
   ESTOP, STOP (modes 0–3, bit 7), FAULT ESTOP, `estop clear`.
   Bench: with the motor running from the console, ESTOP (broadcast 0x000) → coast, latch, OK +
   FAULT. `vel on` is refused. `estop clear` recovers.
2. **ARM + SPEED + STATUS_DRIVE + §5.1 counter + §7.1 timeout.**
   Bench: ARM 1 then SPEED at 10 rpm, 50 Hz from `cancmd.py`; STATUS_DRIVE shows the speed and
   echoes the counter. Then: a REPEAT is rejected and does not kick the watchdog; stopping the
   host trips `vel_tmo` (FAULT VEL_WD, coast); a bad CRC gives `CRC`; ARM 1 recovers.
3. **Ownership (§7.3).** Bench: a CAN-armed loop refuses console `drv duty`/`vel target`; console
   `vel stop` releases ownership; then a console-armed loop refuses CAN SPEED (`UART_OWNS`).
4. **LIMITS, RAMP, CFG_REQ/RESP** (with the `config_apply_live` refactor first, checked through the
   console before any CAN work). Bench: GET/SET/INFO over CAN matches `cfg`; out of range →
   `RANGE`, not clamped; SAVE while armed → `BUSY`.
5. **STEER + STATUS_STEER** (CORNER), position from commanded pulses, steps of 0.01°. Bench:
   STEER_ENABLE, then +30°, −30°, 0°. STATUS_STEER tracks; position read with `0x33` agrees.
   A new STEER replaces a move in progress.
6. **Bus errors and remaining FAULTs** (§7.2): poll ESR each pass. Bus-off is treated like a
   `vel_tmo` expiry and sends BUS_OFF_RECOVERED after rejoining. Also ERROR_PASSIVE,
   RX_RING_DROPPED, DRV_FAULT, SKIPPED_CTR, with at most one frame per code per 100 ms.
   Bench: unplug the bus with the loop running → the motor stops; reconnect → FAULT; ARM is
   needed again.
7. **Corner-node integration**: SPEED and STEER together from `cancmd.py` at 50 Hz / 10 Hz.
   Check bus load and zero ring drops in `stats`. This closes next-task 1.

## Docs at the end
- `can_cmds.md`: Q3, Q13 and Q18 resolved, the CRC check value, and the CFG_REQ CRC byte order.
- LOG entry per phase, from the shell.
- Hot file: active work.
- `_REF_MCU`: a "can_cmd" module section.

## Verification (every phase)
- `wheel-fw` skill build: 0 warnings, flash/RAM delta reported (base 110 568 B / 6 256 B).
- Flash with the skill. Run the phase's bench check one step at a time with the user; save
  transcripts to files and grep them.
- After phase 4: the console `cfg` set/get output is unchanged against a transcript taken before
  the refactor.
- The `main.c` edits stay inside USER CODE blocks. No CubeMX change is expected; if one turns out
  to be needed (e.g. an SCE interrupt), stop and hand over a CubeMX block.

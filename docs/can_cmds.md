# Wheel node — W6 CAN command set (specification)

Status: **draft, 2026-09-28; plain CAN selected 2026-10-04** (Q1: simpler, and
time is short before the Dec 10 demo). Addresses moved to the 4-bit DIP ID on
2026-10-04. No code implements this yet. Open questions are in section 8.

Sources:
- `PROJECT_CONTEXT_REST.md`: "Message ID design principles", "OSI layer
  mapping", and the CANopen key decision.
- `_REF_MCU`: `can_bus`, "CAN error counters", "Bus-load headroom" and
  "DECIDED — interrupt-driven CAN RX".
- Task 6, now in the LOG.
- `docs/plans/ISR-to-ring.md`.
- In the firmware: `console.c` (command table at line 2286 and its handlers),
  `config.h`/`config.c` (keys), `velocity.c`, `drive.c` and `dipsw.h`.

## 1. Conventions

| Item | Rule |
|---|---|
| Frame format | Classic CAN 2.0A, 11-bit ID, DLC ≤ 8, 250 kbps. No extended IDs, no RTR |
| Byte order | Little-endian for every multi-byte field. The existing heartbeat is the one exception (Q2) |
| Directions | O→N means orion to a node; N→O means a node to orion |
| Node address | `a` = 4-bit DIP module ID 1–14 (`dipsw_id()`). **0 = broadcast** (`DIPSW_ADDR_BROADCAST`). 15 = no switch fitted (`DIPSW_CODE_UNFITTED`). No node ever owns address 0 or 15 |
| Roles (`dipsw.h`) | 1–4 CORNER (steering + drive), 5–6 CENTER (drive only), 7–14 RESERVED (future / bench), 0 and 15 INVALID (transmits nothing) |
| Protocol version | `PROTO_VER = 1`. It is not transmitted. It is folded into the CRC (§5.2), so a layout mismatch fails the CRC |
| Reserved bytes | Senders send 0x00; receivers ignore them |
| Wrong DLC | A frame shorter than its table's DLC is rejected with `BAD_DLC` and not acted on. Exception: ESTOP and STOP act on the ID alone |
| RX path | The ISR only captures frames into the 32-frame ring. Parsing and actions run in main-loop dispatch (ISR-to-ring plan) |
| Acceptance | The accept-all hardware filter stays; narrowing it is out of scope in the plan. The node drops, in software, any O→N frame whose address is neither its own nor 0 |

## 2. Console command classification

**R** = runtime control (has its own CAN frame). **C** = configuration (generic
get/set by key index). **D** = diagnostics (stays on UART only).

| Console command | Cat. | CAN equivalent / note |
|---|---|---|
| `help` | D | — |
| `info` | D | — |
| `stats` | D | — |
| `errors` | D | Error state is also carried in the heartbeat and FAULT frames |
| `clear` | D | — |
| `send <id> [hex]` | D | — |
| `heartbeat on\|off` | D | — |
| `monitor on\|off` | D | — |
| `canhold <ms>` | D | Test only |
| `loopback on\|off` | D | — |
| `mks encoder\|pulses\|angle\|en\|protect` | D | The steering state they read is reported in STATUS_STEER |
| `mks enable on\|off` | R | ARM action 4/5 |
| `mks stop` | R | ESTOP, or STOP with the steer bit set |
| `mks move <±pulses> [speed]` | D | Raw relative move, bench only |
| `mks deg <±deg> [speed]` | R | STEER, **absolute** angle rather than relative (Q3) |
| `mks raw <hex…>` | D | — |
| `mks stats\|clear\|abort` | D | — |
| `enc` (state) | D | Speed is carried in STATUS_DRIVE |
| `enc zero` | D | — |
| `enc watch on\|off` | D | — |
| `enc window <ticks>` | D | Changes the control step rate; bench only (Q12) |
| `enc probe [ms]` | D | — |
| `drv` (state) | D | Carried in STATUS_DRIVE flags |
| `drv enable` | R | Part of ARM action 1 |
| `drv disable` | R | Part of ARM action 0 |
| `drv duty <±pct[p]>` | D | Open loop; not exposed on CAN (Q6) |
| `drv brake` | R | STOP mode 2 |
| `drv coast` | R | STOP mode 1 |
| `drv decay slow\|fast` | D | — |
| `drv limit <pct>` | R | LIMITS field `duty_limit` (live, not stored) |
| `drv current [n]` | D | Current is carried in STATUS_DRIVE |
| `drv zero` | D | Refused by the firmware while the bridge is enabled |
| `drv iscan …` | D | — |
| `drv trip <mA>` | R | LIMITS field `trip_ma` (live, not stored) |
| `drv trip buf on\|off` | D | — |
| `drv clearfault` | R | ARM action 2 |
| `drv timeout [ms]` | D | Not a cfg key. When the velocity loop is armed it keeps this watchdog alive (§7.1) |
| `drv ramp <o/oo/s>` | R | RAMP field `ramp_pmps` (live, not stored) |
| `drv ramp floor <o/oo>` | R | RAMP field `ramp_floor` (live, not stored) |
| `vel` (state) | D | Carried in STATUS_DRIVE |
| `vel on` | R | Part of ARM action 1 |
| `vel off` | R | Part of ARM action 0 |
| `vel target <rpm\|Nm>` | R | SPEED (milli-rpm) |
| `vel stop` | R | STOP mode 0 |
| `vel reset` | D | Clears the integrator and derivative; bench only |
| `vel gains` | C | GET keys 11–18 |
| `vel timeout <ms>` | C | Key 19 `vel_tmo` (the console form is live only) |
| `telem …` | D | Bench host stream |
| `cfg` (list) | C | CFG op INFO, then GET for each key |
| `cfg <key>` | C | CFG op GET |
| `cfg <key> <val>` | C | CFG op SET |
| `cfg save` | C | CFG op SAVE |
| `cfg revert` | C | CFG op REVERT |
| `cfg default [<key>]` | C | CFG ops DEFAULT_KEY / DEFAULT_ALL |
| `cfg help` | D | — |
| `id` | D | Identity is visible in every N→O frame ID |
| `reset` | D | Not exposed on CAN (Q7) |

## 3. ID scheme

`ID = (type << 4) | addr`. `type` is 7 bits (0x00–0x7F) and `addr` is 4 bits
(1–14 = node, 0 = broadcast, 15 = never owned). A lower ID wins arbitration.

The ID scheme follows REST "Message ID design principles":
- Priority is set by message type, not by node.
- The upper bits give the group and the lower bits the address, so a mask
  filter per type is possible.
- The type values are chosen so that each frame lands in REST's groups:
  - 0x000–0x00F safety
  - 0x010–0x07F control
  - 0x080–0x0FF sensor setpoints (REST); here it holds only CMD_RESULT
  - 0x100–0x4FF periodic data
  - 0x500–0x5FF telemetry, diagnostics and heartbeat
- ESTOP takes type 0x00, so broadcast ESTOP is ID 0x000, the highest priority
  on the bus. (The 09-28 draft kept 0x000 free for CANopen NMT; plain CAN was
  selected on 10-04, so that reason is gone.)
- With 16 IDs per type the control group holds only 7 types (0x01–0x07).
  CMD_RESULT is the eighth control frame, so it moves to type 0x08, just below
  them. The priority order of the 09-28 draft is kept.
- Address 15 is never owned, so IDs ending in 0xF stay unused.

| Type | ID range | Frame | Dir | Broadcast | REST group |
|---|---|---|---|---|---|
| 0x00 | 0x000–0x00E | ESTOP | O→N | yes (normally 0x000) | safety |
| 0x01 | 0x010–0x01E | STOP | O→N | yes | control |
| 0x02 | 0x021–0x02E | FAULT | N→O | — | control |
| 0x03 | 0x030–0x03E | ARM | O→N | yes | control |
| 0x04 | 0x041–0x04E | SPEED | O→N | no | control |
| 0x05 | 0x051–0x054 | STEER | O→N | no (CORNER only, IDs 1–4) | control |
| 0x06 | 0x060–0x06E | LIMITS | O→N | yes | control |
| 0x07 | 0x070–0x07E | RAMP | O→N | yes | control |
| 0x08 | 0x081–0x08E | CMD_RESULT | N→O | — | setpoints range (see above) |
| 0x10 | 0x101–0x10E | STATUS_DRIVE | N→O | — | periodic |
| 0x11 | 0x111–0x114 | STATUS_STEER | N→O | — (CORNER only, IDs 1–4) | periodic |
| 0x50 | 0x501–0x50E | HEARTBEAT (**existing** `0x500 + id`, unchanged) | N→O | — | heartbeat |
| 0x52 | 0x520–0x52E | CFG_REQ | O→N | yes | diagnostics |
| 0x53 | 0x531–0x53E | CFG_RESP | N→O | — | diagnostics |

Resulting priority, from highest to lowest:
1. ESTOP
2. STOP
3. FAULT
4. ARM
5. SPEED
6. STEER
7. LIMITS / RAMP
8. CMD_RESULT
9. STATUS
10. HEARTBEAT
11. CFG

Rules:
- Every ID is sent by exactly one transmitter. Orion never uses N→O types, and
  each node sends only its own address, so no two senders collide in
  arbitration.
- Within one type, broadcast (addr 0) is the lowest ID, so it wins
  arbitration against any single-node frame of the same type. Broadcast ESTOP
  (0x000) beats every frame on the bus (Q16).
- A node with role INVALID (DIP 0 or 15) transmits nothing, which is existing
  behaviour. It acts only on broadcast ESTOP and STOP.

## 4. Frames

Common fields:
- **`ctr`** is the rolling counter, uint8 (§5.1).
- **`crc`** is the CRC-8 (§5.2).
- The **"Out of range"** column says what the node does. A **rejected** frame:
  - applies nothing (there are no partial applies);
  - does not kick any watchdog;
  - does not advance the counter;
  - is answered with CMD_RESULT (or CFG_RESP) and a result code (§6.1).

### 4.1 ESTOP — 0x000 + addr, O→N, DLC 0–8

| Byte | Field | Type | Unit | Range | Out of range |
|---|---|---|---|---|---|
| 0–7 | ignored | — | — | — | — |

- The node acts on the ID alone. The payload, DLC, counter and CRC are never
  checked, and a repeated ESTOP acts again.
- Action:
  - `vel off`, then `drv coast`;
  - on CORNER nodes, also `mks stop` (F7);
  - then set the **ESTOP latch**.
- While the latch is set:
  - ARM action 1, SPEED and STEER are rejected with `ESTOP_LATCHED`;
  - the console refuses `vel on`, `drv enable`, `drv duty`, `mks move` and
    `mks deg`.
- The latch clears only with ARM action 3 or a reset.
- Reply: CMD_RESULT `OK`, plus FAULT code `ESTOP`.

### 4.2 STOP — 0x010 + addr, O→N, DLC 2

| Byte | Field | Type | Unit | Range | Out of range |
|---|---|---|---|---|---|
| 0 | `ctr` | u8 | — | any | Not checked; stops always act |
| 1 | `mode` | u8 bitfield | — | bits 0–1: 0 = ramp (`vel stop`), 1 = coast now (`drv coast`), 2 = brake now (`drv brake`); bit 7 = also `mks stop` | Mode 3 → coast now (fail-safe); undefined bits are ignored |

- The node acts on the ID. A short DLC is treated as mode 1, coast now.
- STOP does not latch. Mode 0 leaves the loop armed with setpoint 0.
- Modes 1 and 2 also disarm the loop (`vel off`).
- Reply: CMD_RESULT.

### 4.3 ARM — 0x030 + addr, O→N, DLC 8

| Byte | Field | Type | Unit | Range | Out of range |
|---|---|---|---|---|---|
| 0 | `ctr` | u8 | — | §5.1 | Rejected (`REPEAT` / `STALE`) |
| 1 | `action` | u8 | — | 0–5, see below | Rejected (`BAD_ACTION`) |
| 2–6 | reserved | — | — | 0 | Ignored |
| 7 | `crc` | u8 | — | — | Rejected (`CRC`) |

| `action` | Console equivalent | Precondition, or result code if unmet |
|---|---|---|
| 0 DISARM | `vel off` then `drv disable` | none |
| 1 ARM | `drv enable` then `vel on` | Not ESTOP-latched (`ESTOP_LATCHED`); no latched drive fault (`FAULT_LATCHED`); the UART does not own motion (`UART_OWNS`, §7.3) |
| 2 CLEAR_FAULT | `drv clearfault` | none |
| 3 CLEAR_ESTOP | — (new) | The loop is off |
| 4 STEER_ENABLE | `mks enable on` | CORNER role (`NOT_SUPPORTED`) |
| 5 STEER_DISABLE | `mks enable off` | CORNER role (`NOT_SUPPORTED`) |

- ARM action 1 also clears the latched velocity-watchdog flag, as
  `velocity_enable()` does.
- An accepted ARM resets the counter window for SPEED and STEER (§5.1).
- Reply: CMD_RESULT.

### 4.4 SPEED — 0x040 + node, O→N, DLC 8 (no broadcast)

| Byte | Field | Type | Unit | Scaling | Range | Out of range |
|---|---|---|---|---|---|---|
| 0 | `ctr` | u8 | — | — | §5.1 | Rejected (`REPEAT` / `STALE`) |
| 1 | reserved | u8 | — | — | 0 | Ignored |
| 2–5 | `setpoint` | i32 | milli-rpm, wheel output shaft | 1 = 0.001 rpm (the loop's own unit) | −100 000 … +100 000 (Q4) | Rejected (`RANGE`); the previous setpoint stays until `vel_tmo` |
| 6 | reserved | u8 | — | — | 0 | Ignored |
| 7 | `crc` | u8 | — | — | — | Rejected (`CRC`) |

- An accepted frame calls `velocity_set_setpoint()`, the same call as
  `vel target`. That call kicks the setpoint watchdog.
- The sign is the direction, with the same convention as the console.
- If the loop is not armed, the frame is rejected with `NOT_ARMED`. The console
  refuses in the same case.
- The output is still capped by `vel_max`, `vel_ilim` and `duty_limit`; SPEED
  does not bypass them.
- Reply: CMD_RESULT **only on rejection**. Success shows up as `ctr` echoed in
  STATUS_DRIVE.

### 4.5 STEER — 0x050 + node, O→N, DLC 8 (CORNER nodes only)

| Byte | Field | Type | Unit | Scaling | Range | Out of range |
|---|---|---|---|---|---|---|
| 0 | `ctr` | u8 | — | — | §5.1 | Rejected (`REPEAT` / `STALE`) |
| 1 | `speed` | u8 | MKS speed code | — | 1–127; 0 = node default 2 | Rejected (`RANGE`) |
| 2–3 | `angle` | i16 | degrees at the gearbox output, **absolute** from the steering zero | 1 = 0.01° (one pulse = 0.0118°) | −9000 … +9000 (±90°, Q3) | Rejected (`RANGE`); no motion |
| 4–6 | reserved | — | — | — | 0 | Ignored |
| 7 | `crc` | u8 | — | — | — | Rejected (`CRC`) |

- CENTER, RESERVED and INVALID nodes reply `NOT_SUPPORTED`.
- The node converts the absolute target into a relative FD move (target minus
  tracked position). The tracked position is the sum of completed moves'
  commanded pulses, zeroed at STEER_ENABLE (Q3).
- A STEER that arrives while a move is still running replaces the target. The
  running move is not cut short: when it completes, one FD goes out for the
  difference to the latest target.
- A move that ends any way other than "complete" (stop, ESTOP, timeout, link
  error) clears `position valid` in STATUS_STEER. STEER then replies
  `NOT_ARMED` (detail 1) until a new STEER_ENABLE re-zeroes.
- If steering is not enabled, the frame is rejected with `NOT_ARMED`.
- Send rate: on change, at most 10 Hz. It is not cyclic.
- Reply: CMD_RESULT, always.

### 4.6 LIMITS — 0x060 + addr, O→N, DLC 8

Live only; nothing is stored. It behaves like `drv limit` and `drv trip`. To
store a limit, use CFG SET and SAVE on keys 3–4.

| Byte | Field | Type | Unit | Range | Out of range |
|---|---|---|---|---|---|
| 0 | `ctr` | u8 | — | §5.1 | Rejected |
| 1 | `mask` | u8 | — | bit 0 `duty_limit`, bit 1 `trip_ma`; others 0 | Mask 0 → `BAD_ACTION` |
| 2–3 | `duty_limit` | u16 | o/oo | 0–1000 | Whole frame rejected (`RANGE`, detail = field bit). **Differs from the console**, which clamps `drv limit` |
| 4–5 | `trip_ma` | u16 | mA | `isense_trip_min_ma()`…`isense_trip_max_ma()` (hardware ~101–1580) | Whole frame rejected (`RANGE`) |
| 6 | reserved | — | — | 0 | Ignored |
| 7 | `crc` | u8 | — | — | Rejected |

- A tighter `duty_limit` takes effect immediately and does not kick the
  watchdog (`drive_set_limit()` semantics).
- Reply: CMD_RESULT.

### 4.7 RAMP — 0x070 + addr, O→N, DLC 8

Live only, like `drv ramp` and `drv ramp floor`. Stored keys: 9 and 10.

| Byte | Field | Type | Unit | Range | Out of range |
|---|---|---|---|---|---|
| 0 | `ctr` | u8 | — | §5.1 | Rejected |
| 1 | `mask` | u8 | — | bit 0 `ramp_pmps`, bit 1 `ramp_floor` | Mask 0 → `BAD_ACTION` |
| 2–3 | `ramp_pmps` | u16 | o/oo per s; 0 = off | 0–10 000 | Whole frame rejected (`RANGE`) |
| 4–5 | `ramp_floor` | u16 | o/oo | 0–300 | Whole frame rejected (`RANGE`) |
| 6 | reserved | — | — | 0 | Ignored |
| 7 | `crc` | u8 | — | — | Rejected |

Reply: CMD_RESULT.

### 4.8 CFG_REQ — 0x520 + addr, O→N, DLC 8

| Byte | Field | Type | Unit | Range | Out of range |
|---|---|---|---|---|---|
| 0 | `op` | u8 | — | 0 GET, 1 SET, 2 SAVE, 3 REVERT, 4 DEFAULT_KEY, 5 DEFAULT_ALL, 6 INFO, 7 GET_MIN, 8 GET_MAX, 9 GET_DEFAULT | Status `BAD_OP` |
| 1 | `key` | u8 | index into `config_key_t` | 0–21 (table below); ignored by ops 2, 3, 5, 6 | Status `UNKNOWN_KEY` |
| 2 | `tag` | u8 | — | any; echoed back | — |
| 3 | `crc` | u8 | — | — | Status `CRC`; nothing applied |
| 4–7 | `value` | i32 | the key's unit (table) | the key's min…max; used by SET only | Status `RANGE`; **rejected, not clamped** (`config_set()` semantics) |

Rules:
- SET applies live, exactly as `cfg <key> <val>` does: trip, limit, ramp and
  floor are re-applied, and the `vel_*` keys are read live.
- A SET lives in RAM until SAVE.
- SAVE is refused with `BUSY` while the bridge is enabled or the loop is armed
  (a flash write stalls the core; Q10).
- REVERT and DEFAULT re-apply every key to the running modules
  (`config_apply_all()`), as the console does.
- A SET of `vel_tmo`, and any REVERT or DEFAULT, re-arms the setpoint
  countdown, as the console does: a deadline just changed has not been missed
  yet. So a host repeating those keeps an armed loop alive without SPEED. Orion must
  not use it as a keep-alive; liveness is SPEED (§7.1).
- A frame with DLC < 8 is answered with status `CRC` (6): a truncated frame
  cannot carry a checkable CRC. There is no separate BAD_DLC status.
- There is no rolling counter: config is not motion. `tag` pairs each request
  with its response.
- Broadcast is allowed. Every node answers with its own CFG_RESP.
- Key indices are part of the protocol. New keys are only ever appended. A
  change of meaning bumps `CONFIG_VERSION` and `PROTO_VER`.

| Idx | Key | Unit | Min | Max |
|---|---|---|---|---|
| 0 | `vdda_mv` | mV | 2000 | 3600 |
| 1 | `r_ipropi` | Ω | 100 | 10000 |
| 2 | `a_ipropi` | µA/A | 100 | 2000 |
| 3 | `trip_ma` | mA | 0 | 1600 |
| 4 | `duty_limit` | o/oo | 0 | 1000 |
| 5 | `rail_mv` | mV | 0 | 40000 |
| 6 | `isense_avg` | — | 1 | 1024 |
| 7 | `sat_raw` | — | 1000 | 4095 |
| 8 | `vref_div` | — | 1 | 4 |
| 9 | `ramp_pmps` | o/oo/s | 0 | 10000 |
| 10 | `ramp_floor` | o/oo | 0 | 300 |
| 11 | `vel_kp` | m o/oo/rpm | 0 | 100000 |
| 12 | `vel_ki` | m/rpm·s | 0 | 200000 |
| 13 | `vel_kd` | m o/oo·s/rpm | 0 | 100000 |
| 14 | `vel_ff_a` | m o/oo/rpm | 0 | 100000 |
| 15 | `vel_ff_b` | o/oo | 0 | 300 |
| 16 | `vel_ilim` | o/oo | 0 | 1000 |
| 17 | `vel_max` | o/oo | 0 | 1000 |
| 18 | `vel_slew` | m rpm/s | 0 | 1000000 |
| 19 | `vel_tmo` | ms | 0 | 60000 |
| 20 | `isense_dk` | o/oo | 400 | 1000 |
| 21 | `isense_dmin` | o/oo | 30 | 145 |

### 4.9 CFG_RESP — 0x530 + node, N→O, DLC 8

| Byte | Field | Type | Meaning |
|---|---|---|---|
| 0 | `op` | u8 | Echoed |
| 1 | `key` | u8 | Echoed |
| 2 | `tag` | u8 | Echoed |
| 3 | `status` | u8 | 0 OK, 1 UNKNOWN_KEY, 2 RANGE, 3 BUSY, 4 FLASH_ERROR, 5 BAD_OP, 6 CRC |
| 4–7 | `value` | i32 | GET, SET, DEFAULT_KEY: the key's RAM value after the op. GET_MIN/MAX/DEFAULT: that bound. INFO: byte 4 = `CONFIG_VERSION`, byte 5 = key count, byte 6 = slot used, byte 7 bit 0 = dirty. SAVE/REVERT: the `config_load_t` or save result code |

### 4.10 CMD_RESULT — 0x080 + node, N→O, DLC 4

| Byte | Field | Type | Meaning |
|---|---|---|---|
| 0 | `type` | u8 | The type of the command answered (0x01–0x08) |
| 1 | `ctr` | u8 | The command's counter, echoed |
| 2 | `result` | u8 | Result code (§6.1) |
| 3 | `detail` | u8 | For `RANGE`: the offending field's mask bit. For `STALE`/`REPEAT`: the node's last accepted counter. Otherwise 0 |

### 4.11 FAULT — 0x020 + node, N→O, DLC 8, event-driven

| Byte | Field | Type | Unit | Meaning |
|---|---|---|---|---|
| 0 | `code` | u8 | — | 1 DRV_FAULT (nFAULT), 2 VEL_WD_EXPIRED, 3 ESTOP, 4 BUS_OFF_RECOVERED, 5 ERROR_PASSIVE, 6 RX_RING_DROPPED, 7 MKS_ERROR, 8 SKIPPED_CTR (above the §5.1 threshold) |
| 1 | `flags` | u8 | — | STATUS_DRIVE `flags` at the event |
| 2–3 | `duty` | i16 | o/oo | Applied duty at the event |
| 4–7 | `t_ms` | u32 | ms | Uptime at the event |

- Sent on the rising edge of each condition.
- Rate limit: at most one per code per 100 ms.
- A FAULT caused by bus-off is sent after recovery.

### 4.12 STATUS_DRIVE — 0x100 + node, N→O, DLC 8, periodic

| Byte | Field | Type | Unit | Scaling |
|---|---|---|---|---|
| 0 | `ctr` | u8 | — | Last accepted SPEED counter |
| 1 | `flags` | u8 | — | bit 0 loop armed, 1 bridge enabled, 2 drive fault latched, 3 saturated, 4 vel watchdog expired (latched), 5 ESTOP latched, 6 ramping, 7 motion owned by UART |
| 2–3 | `speed` | i16 | rpm | 1 = 0.01 rpm (±327 rpm), measured |
| 4–5 | `current` | u16 | mA | 1 = 1 mA, magnitude as `drv current` reports it |
| 6–7 | `output` | i16 | o/oo | Loop output / applied duty |

- Sent at the control-step rate, 50 Hz at `enc window 20`, and phase-locked to
  the step.
- Sent whether or not the loop is armed.

### 4.13 STATUS_STEER — 0x110 + node, N→O, DLC 8, periodic, CORNER only

| Byte | Field | Type | Unit | Scaling |
|---|---|---|---|---|
| 0 | `ctr` | u8 | — | Last accepted STEER counter |
| 1 | `flags` | u8 | — | bit 0 enabled, 1 moving, 2 protect/stall, 3 position valid, 4 UART transaction error |
| 2–3 | `target` | i16 | ° | 0.01° |
| 4–5 | `position` | i16 | ° | 0.01°, tracked absolute position (Q3) |
| 6–7 | reserved | — | — | 0 |

Rate: 10 Hz.

### 4.14 HEARTBEAT — 0x500 + node, N→O, DLC 8, existing

This frame is unchanged. It runs at TIM7's 2 Hz. Payload:
- bytes 0–3: sequence, **big-endian** (Q2);
- byte 4 TEC, byte 5 REC, byte 6 LEC;
- byte 7: bit 0 warning, bit 1 passive, bit 2 bus-off.

## 5. Integrity

### 5.1 Rolling counter

Counters are kept per (frame type, destination address). Orion increments
`ctr` by 1 (mod 256) on each send of that type to that address. Broadcast
(addr 0) has its own sequence. The node keeps `last[type]` for its own address
and a separate one for broadcast.

| Frames | Counter checked? |
|---|---|
| ESTOP, STOP | **No.** A stop always acts, even as a repeat |
| ARM, SPEED, STEER, LIMITS, RAMP | Yes |
| CFG_REQ | No; it uses `tag` |

Take `d = (ctr − last) mod 256`:

| `d` | Meaning | Node action |
|---|---|---|
| 1 | Next frame in sequence | Accept |
| 2–127 | `d − 1` frames skipped | **Accept** (the latest value is valid). Add `d − 1` to `can_ctr_skipped`. If `d − 1 ≥ 5`, send FAULT `SKIPPED_CTR` |
| 0 | Repeat | **Reject** (`REPEAT`). Not applied, **does not kick `vel_tmo`**. Counted |
| 128–255 | Older than `last`, or a stale burst | Reject (`STALE`). Counted |

Re-synchronisation: the first counted frame after boot, after an accepted ARM,
and after ESTOP is accepted with any `ctr`, and it sets `last`.

Purpose: a sender stuck re-sending its last frame does not keep the watchdog
alive. The counter catches a frozen host; the CRC does not.

### 5.2 CRC-8 — decision: **yes, on ARM, SPEED, STEER, LIMITS, RAMP and CFG_REQ**

| Question | Answer |
|---|---|
| What the controller already covers | CRC-15, bit stuffing, form, ACK and bit monitoring protect the frame **on the wire**. The residual undetected-error rate is negligible for this bus |
| What CRC-15 cannot see | (1) Corruption before the TX mailbox or after the RX FIFO: host software, SocketCAN buffers, the node's ring and parser. (2) A **layout mismatch**: orion and the node built against different frame definitions, or a frame sent with a valid ID and the wrong layout. (3) A frame built for one type or address sent under another ID. None of these is a wire error, so CRC-15 passes all three |
| What a payload CRC adds | End-to-end protection from the orion encoder to the node decoder. With the ID and `PROTO_VER` folded in, (2) and (3) fail deterministically. This is the AUTOSAR E2E idea, without its full profile |
| Cost | 1 byte of 8, one 256-byte table, about 1 µs per frame on the F446 |
| Decision | Add it to every frame that can **cause motion or change a limit or config**. Leave it off ESTOP and STOP, which must act even when corrupt (fail-safe). Leave it off N→O frames: orion can check their plausibility, and CAN load is lower |

Definition: CRC-8/SAE-J1850 (poly 0x1D, init 0xFF, xorout 0xFF, no
reflection). Input, in order:
1. the 11-bit ID as u16 little-endian (2 bytes);
2. `PROTO_VER` (1 byte);
3. the payload bytes, excluding the CRC byte.

On mismatch: reject with `CRC`; nothing applied, no watchdog kick, counter not
advanced.

## 6. Responses

### 6.1 Result codes (CMD_RESULT byte 2)

| Code | Name | When |
|---|---|---|
| 0 | OK | Accepted and applied |
| 1 | REPEAT | Counter repeated |
| 2 | STALE | Counter older than the last accepted |
| 3 | RANGE | A field is out of range; `detail` = its mask bit |
| 4 | NOT_ARMED | SPEED while the loop is off; STEER while steering is disabled |
| 5 | NOT_SUPPORTED | Wrong role, e.g. STEER on a CENTER node |
| 6 | ESTOP_LATCHED | Motion refused until CLEAR_ESTOP |
| 7 | FAULT_LATCHED | ARM refused while the drive fault is latched |
| 8 | CRC | CRC mismatch |
| 9 | BAD_DLC | Frame too short |
| 10 | UART_OWNS | Motion is owned by the console (§7.3) |
| 11 | BAD_ACTION | Unknown action, or an empty mask |
| 12 | BUSY | Command refused in the current state |

### 6.2 Which commands get a reply

| Command | Reply |
|---|---|
| ESTOP | CMD_RESULT OK, then FAULT ESTOP |
| STOP, ARM, STEER, LIMITS, RAMP | CMD_RESULT, always |
| SPEED | CMD_RESULT **only on rejection**; success is `ctr` in STATUS_DRIVE |
| CFG_REQ | CFG_RESP, always |
| Broadcast frames | Every addressed node replies with its own ID |

### 6.3 Periodic bus load, 6 wheel nodes

Frame time is the worst case for an 8-byte standard frame: 135 bits, 540 µs at
250 kbps. The Aug 11 saturation measured 1858 frames/s, which is the same
figure.

| Stream | Nodes | Rate | Frames/s |
|---|---|---|---|
| SPEED (O→N) | 6 | 50 Hz | 300 |
| STEER (O→N), worst case | 4 | 10 Hz | 40 |
| STATUS_DRIVE | 6 | 50 Hz | 300 |
| STATUS_STEER | 4 | 10 Hz | 40 |
| HEARTBEAT | 6 | 2 Hz | 12 |
| **Total** | | | **692 f/s → 37.4 %** |
| With STATUS_DRIVE at 25 Hz | | | 542 f/s → 29.3 % |

CMD_RESULT, FAULT and CFG frames are sporadic and not included, and neither is
traffic from non-wheel nodes (Q9).

## 7. Safety behaviour

### 7.1 Command timeout

| Rule | |
|---|---|
| Does `vel_tmo` apply to CAN? | **Yes, unchanged.** Accepted SPEED frames call `velocity_set_setpoint()`, which kicks the same setpoint watchdog as `vel target`. Default 1000 ms = 50 missed frames at 50 Hz (Q11) |
| What does not kick | Rejected frames (REPEAT, STALE, RANGE, CRC, NOT_ARMED), STATUS traffic, CFG, LIMITS and RAMP. Exception: CFG SET `vel_tmo`, REVERT and DEFAULT re-arm the countdown (§4.8) |
| On expiry | The existing behaviour: setpoint 0, coast, flag latched. Also FAULT `VEL_WD_EXPIRED` and STATUS_DRIVE bit 4. The latch clears with ARM action 1 (`velocity_enable()`) |
| `drv timeout` | Stays UART-only and off by default. While the loop is armed, each loop step's duty command kicks it, so orion's liveness is judged by `vel_tmo` |
| Steering | A move in progress completes; there is no steering timeout. FD moves are finite |

### 7.2 Bus errors

| State | Detection | Node action |
|---|---|---|
| Error-warning | ESR, polled every main-loop pass | Flag only (heartbeat byte 7) |
| Error-passive | ESR, polled | Keep running (RX still works). FAULT `ERROR_PASSIVE` once per entry. Flag in the heartbeat |
| Bus-off | ESR `BOFF`, polled | **Immediately** act as a `vel_tmo` expiry: setpoint 0, coast, watchdog latch set (`velocity_expire_now()`, also with `vel_tmo` 0). Don't wait up to 1 s for `vel_tmo`. Only a loop armed over CAN; a console-owned loop is not driven over this bus and keeps running. `AutoBusOff = ENABLE` rejoins after 128 × 11 recessive bits. After rejoin: FAULT `BUS_OFF_RECOVERED`. Motion resumes only after ARM action 1 |
| RX ring overflow | `rx_ring_dropped` increments | FAULT `RX_RING_DROPPED` (rate-limited). No motion change; the counter rules handle the lost commands |

An SCE (error) interrupt is out of scope (ISR-to-ring plan), so detection is by
polling.

A FAULT that cannot be queued (all three mailboxes full, e.g. nobody ACKing)
is held, not dropped, and goes out when a mailbox frees.

### 7.3 UART and CAN arbitration

| Command class | Rule |
|---|---|
| Stops: ESTOP, STOP; console `vel off`, `vel stop`, `drv coast`, `drv brake`, `drv disable`, `mks stop` | **Always accepted from either source.** A stop never needs ownership |
| Motion: ARM action 1, SPEED, STEER, LIMITS, RAMP; console `drv enable`, `vel on`, `vel target`, `drv duty`, `mks move`/`deg`, `drv limit`/`trip`/`ramp` | **The source that armed owns motion** until a disarm or stop from either source. Motion commands from the other source are refused: CAN gets `UART_OWNS`; the console prints "CAN owns motion — 'vel off' first" |
| Config: CFG_REQ, `cfg …` | Both sources allowed; the last write wins. SAVE follows the `BUSY` rule in §4.8 |
| Diagnostics | UART only; no conflict |

- After boot nobody owns motion; the first arming command takes ownership.
- STATUS_DRIVE bit 7 shows when the UART owns motion.

## 8. Open questions and assumptions

1. **Plain CAN vs CANopen — resolved 2026-10-04: plain CAN.** Reason:
   simplicity and lack of time before the Dec 10 demo. `docs/canopen_cmds.md`
   is kept for reference only. This overrides the CANopen selection in
   `PROJECT_CONTEXT_REST.md` for the wheel nodes.
2. **Heartbeat byte order.** The existing 0x500+ID heartbeat sends its sequence
   big-endian, which conflicts with the little-endian rule. The spec keeps the
   heartbeat unchanged. Should it switch?
3. **Absolute steering position — resolved 2026-10-04 (W6).** The tracked
   position is the sum of commanded pulses, advanced only when an FD reports
   "complete". Zero is the wheel's position at STEER_ENABLE (aligned by hand).
   A STEER during a move is deferred to the end of that move. `33` is the
   cross-check: position = −(`33` now − `33` at enable). Bench: +15, −15, 0
   and a deferred +15 → −10 all matched `33` exactly.
   - Assumed range ±90° at the output; the real mechanical limit is unknown.
   - A limit stored on the node would need a new cfg key, and a new key
     discards the stored config record.
4. **Speed range and unit.** Assumed ±100 rpm at the wheel output shaft in
   milli-rpm. The plant reaches ~77 rpm at 100 % duty at 12 V (loaded rig), and
   `vel_max` 300 caps it near 21 rpm. Orion converts m/s to rpm (wheel radius
   is not in this spec). Should the range be tighter?
5. **RESERVED role (IDs 7–14, future / bench).** Assumed to accept all frames
   and steer like a CORNER. INVALID (DIP 0 or 15) obeys only broadcast ESTOP
   and STOP.
6. **Open-loop duty over CAN.** `drv duty` is kept UART-only. Is a CAN duty
   frame wanted, for example for rover τ tests driven from orion?
7. **`reset` over CAN.** Kept UART-only.
8. **Auto-retransmit vs one-shot (NART).** This is the existing `_REF_MCU` open
   question. It is bxCAN-wide, so status frames are retried like commands.
   Unchanged here.
9. **Other nodes' traffic.** The bus plans 8 nodes, and the load table covers
   the 6 wheel nodes only.
10. **SAVE refused while armed.** This assumes a sector-7 write stalls the core
    long enough to disturb the 1 kHz tick. Not measured.
11. **`vel_tmo` value.** 1000 ms is the default. It is not changed here, but it
    may be long for a moving rover at 50 Hz commands.
12. **`enc window` and status rate.** STATUS_DRIVE is locked to the control
    step. Changing `enc window` over UART changes the CAN rate. 50 Hz or 25 Hz?
13. **Ownership rule (§7.3).** This is a new behaviour for the console, which
    today accepts everything. Confirm before implementation.
14. **Stop policy.** ESTOP and STOP mode 1 coast, following the W4 stop policy.
    Braking may be needed on slopes. Undecided.
15. **CENTER nodes (IDs 5–6) have no steering**, following `dipsw.h`. Confirm
    this against the chassis.
16. **Broadcast ESTOP arbitration — resolved 2026-10-04.** Broadcast is
    addr 0, the lowest ID in its type, so broadcast ESTOP (0x000) wins against
    every frame on the bus.
17. **CRC polynomial.** SAE-J1850 was chosen. Any 8-bit polynomial with a
    documented check value would do.
18. **ESTOP latch and the console.** The console refuses motion while the latch
    is set, but there is no console command to clear it. Add one, or keep it
    CAN and reset only?

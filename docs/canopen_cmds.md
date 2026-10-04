# Wheel node — W6 CANopen interface (specification)

Status: **not selected (2026-10-04).** W6 uses plain CAN, `docs/can_cmds.md`.
Kept for reference. Draft of 2026-09-28; no code implements this.

Sources:
- `PROJECT_CONTEXT_REST.md`: "Physical bus", "Message ID design principles",
  "OSI layer mapping", non-wheel task 7 (CANopen stack), and the CANopen key
  decision.
- `_REF_MCU`: `can_bus`, "CAN error counters", "DECIDED — interrupt-driven CAN
  RX, ISR-to-ring".
- Task 6 (now in the LOG) and `docs/plans/ISR-to-ring.md`.
- Firmware: the `console.c` command table and handlers (the survey done for
  `docs/can_cmds.md` §2), `config.h`/`config.c` (keys, ranges, defaults),
  `velocity.c`, `drive.c`, `dipsw.h`.

## 0. Conventions and REST decisions

### 0.1 Followed from REST

| REST decision | Used here as |
|---|---|
| CANopen with CANopenNode as the application layer | CiA 301 slave; stack choice revisited in §8 (Q6) |
| CiA 402 motor profile, cyclic synchronous velocity (CSV) | Drive axis = CiA 402, mode 9 CSV (§1) |
| Heartbeat, EMCY and SDO configuration | §3, §6 |
| ros2_canopen (Fraunhofer IPA / ROS-Industrial) on the Jetson | Orion = NMT master + SDO client via ros2_canopen (lely) |
| 250 kbps, 8 nodes on the bus | Unchanged; load budget in §4.4 |
| "Plain CAN message ID hierarchy … not used in final rover" | Predefined connection set (node-centric COB-IDs), not REST's message-centric groups |

### 0.2 Conventions

| Item | Rule |
|---|---|
| Frames | Classic CAN 2.0A, 11-bit ID, DLC ≤ 8, 250 kbps. No RTR (CiA 301 deprecates remote PDOs) |
| Byte order | Little-endian (CiA 301). The existing 0x500+ID heartbeat is **removed**, not kept big-endian (§3.2) |
| O→N / N→O | Orion to node / node to orion |
| Node-ID | `node_id = dipsw_id() + 1` (table below). Node-ID 0 is invalid |
| Orion (NMT master) node-ID | **0x7F** (assumed, Q8). Master heartbeat on 0x77F |
| Broadcast | NMT node-ID 0 (all nodes) and SYNC. There is no broadcast SDO or PDO |
| RX path | Frames reach the stack through the existing ISR-to-ring path (§8, Q6) |
| Types | CiA 301 names: U8/U16/U32 = UNSIGNED8/16/32, I8/I16/I32 = INTEGER8/16/32 |

| DIP code | `dipsw.h` role | Node-ID | Axes |
|---|---|---|---|
| 0–3 | CORNER | 1–4 | drive + steering |
| 4–5 | CENTER | 5–6 | drive only |
| 6 | RESERVED (bench) | 7 | drive + steering (as CORNER, Q9) |
| 7 | INVALID | none | Transmits nothing, not even boot-up (existing behaviour). Obeys NMT to node 0 only |

### 0.3 Differences from the plain-CAN spec (`docs/can_cmds.md`)

| Topic | Plain CAN | CANopen (this document) |
|---|---|---|
| IDs | Message-centric, `(type << 3) \| addr` | Node-centric predefined connection set |
| Emergency stop | ESTOP 0x008–0x00F | NMT Stop to node 0, ID **0x000** |
| Arm / disarm | ARM frame, 6 actions | CiA 402 controlword state machine |
| Config | CFG_REQ/RESP with tag | SDO (expedited), 0x1010 save, 0x1011 defaults |
| Faults | FAULT frame | EMCY (CiA 301/402 codes) |
| Acks | CMD_RESULT | SDO responses; PDOs unacknowledged, results in TPDOs |
| Liveness | `vel_tmo` only | Heartbeat consumer + RPDO deadline + `vel_tmo` |
| Integrity | Counter + CRC-8 | Counter only (§7) |
| Bus load, 6 nodes | 37.4 % | 42.6 % at 50 Hz SYNC, 24.2 % at 25 Hz (§4.4) |
| Orion side | Custom SocketCAN code | ros2_canopen + a custom driver for steering/counter (§1) |
| Firmware stack | ~1 module | CiA 301 slave + CiA 402 state machine (§8) |

## 1. Profile decision

### 1.1 Options

| Option | Drive axis | Steering axis | 0x1000 |
|---|---|---|---|
| A. CiA 402, two axes | 402 at 0x6000 | 402 axis 2 at 0x6800 (offset 0x800) | 0xFFFF0192 (multi-axis) |
| B. **CiA 402 drive + manufacturer steering** | 402 at 0x6000, CSV | Manufacturer objects 0x2100–0x210F | 0x00020192 |
| C. Two node-IDs per board | 402 on node-ID *n* | 402 (profile position) on node-ID *n*+8 | 0x00020192 each |
| D. Manufacturer-specific only | 0x2xxx objects | 0x2xxx objects | 0x00000000 |

### 1.2 Comparison

| Criterion | A | B | C | D |
|---|---|---|---|---|
| Firmware state machine | 2 × 402 (8 states, ~10 transitions each) | 1 × 402 + an enable bit | 2 × 402 + 2 stack instances | Enable bits only |
| Two axes | Standard, but rarely supported by masters | Drive standard, steering custom | Each axis looks like a separate drive | All custom |
| DIP → node-ID | 1 ID | 1 ID | 2 IDs; 14 of 127 used | 1 ID |
| ros2_canopen stock `cia402_driver` | Drive only, if it handles one axis per node (Q4) | Drive axis: yes | Both axes: yes | No; proxy driver + own code |
| Standard tools (python-canopen, CANopen Magic, lely `cocomm`) | Full | Full for drive, SDO for steering | Full | SDO/PDO only |
| Firmware cost | Highest | Medium | Highest; CANopenNode runs one node per CAN module | Lowest |
| Fits REST decision (402 CSV) | Yes | Yes | Yes | No |

### 1.3 Decision: **B**

- The drive axis is a CiA 402 servo drive in CSV (mode 9), as REST decided. Stock
  402 masters and tools can drive it.
- Steering is a set of manufacturer objects on the same node-ID. It is a finite
  absolute move to the SERVO42C, not a velocity or torque loop; a second 402
  machine adds states that the MKS cannot report (it has no fault reset, no
  "switched on" state).
- Orion needs a custom driver in any case: for steering objects (B, D) and for
  the motion counter (§7). B keeps that driver small: it extends the stock
  402 driver rather than replacing it.
- Steering motion is gated by the drive axis's 402 state (§5.3), so there is
  one state machine per node.

### 1.4 CiA 402 subset (drive axis)

| Item | Supported |
|---|---|
| Modes (0x6060) | **9 CSV** only. PV (3) is Q5 |
| States | All 8 of the 402 power state machine |
| Controlword bits | 0 switch on, 1 enable voltage, 2 quick stop (0 = active), 3 enable operation, 7 fault reset (rising edge), 8 halt. Bits 4–6, 9 ignored; 11–15 reserved (0) |
| Statusword bits | 0 ready to switch on, 1 switched on, 2 operation enabled, 3 fault, 4 voltage enabled, 5 quick stop (0 = active), 6 switch on disabled, 7 warning, 8 **mfr: output saturated**, 9 remote (0 = UART owns motion, §6.6), 10 reserved (0), 11 internal limit active, 12 drive follows the command value, 13 reserved (0), 14 **mfr: last RPDO1 rejected**, 15 **mfr: `vel_tmo` latched** |
| Units | Velocity: 0.001 rpm at the wheel output shaft (the loop's own unit). Position: encoder counts |

| 402 state | Firmware state | Console equivalent |
|---|---|---|
| Not ready to switch on | Boot, config load | — |
| Switch on disabled | Bridge disabled, loop off | `drv disable` |
| Ready to switch on | Bridge disabled, loop off | — |
| Switched on | Bridge enabled, duty 0 (coast), loop off | `drv enable` |
| Operation enabled | Bridge enabled, loop on, follows 0x60FF | `vel on` |
| Quick stop active | Per 0x605A, then → switch on disabled | `drv coast` / `drv brake` |
| Fault reaction active | Coast (0x605E = 0) | — |
| Fault | Bridge disabled, loop off, latched | `drv` shows fault |

## 2. Console command classification

**R** = RPDO (runtime control). **S** = SDO object (configuration).
**U** = UART-only (bench diagnostics). **N** = NMT service.

| Console command | Cat. | CANopen equivalent / note |
|---|---|---|
| `help` | U | — |
| `info` | U | Identity readable via SDO 0x1018, 0x2F02 |
| `stats` | U | — |
| `errors` | U | CAN error state in TPDO4 and EMCY |
| `clear` | U | — |
| `send <id> [hex]` | U | Must not use COB-IDs owned by the stack |
| `heartbeat on\|off` | S | 0x1017 (0 = off). The old 0x500+ID heartbeat is removed |
| `monitor on\|off` | U | — |
| `canhold <ms>` | U | Test only |
| `loopback on\|off` | U | — |
| `mks encoder\|pulses\|angle\|en\|protect` | U | Position and state in TPDO3 |
| `mks enable on\|off` | R | RPDO2 0x2100 bit 0 |
| `mks stop` | R | RPDO2 0x2100 bit 1; also any quick stop / NMT stop |
| `mks move <±pulses> [speed]` | U | Relative raw move, bench only |
| `mks deg <±deg> [speed]` | R | RPDO2 0x2101 (**absolute**, Q3) + 0x2102 |
| `mks raw <hex…>` | U | — |
| `mks stats\|clear\|abort` | U | — |
| `enc` (state) | U | Velocity in TPDO1, position in TPDO2 |
| `enc zero` | U | — |
| `enc watch on\|off` | U | — |
| `enc window <ticks>` | U | Sets the control step; must stay consistent with SYNC (Q7) |
| `enc probe [ms]` | U | — |
| `drv` (state) | U | Statusword in TPDO1 |
| `drv enable` | R | Controlword 0x06 then 0x07 (→ switched on) |
| `drv disable` | R | Controlword 0x00 (disable voltage → switch on disabled) |
| `drv duty <±pct[p]>` | U | Open loop; no CAN path (Q10) |
| `drv brake` | R | Controlword quick stop with 0x605A = −1 (mfr: brake) |
| `drv coast` | R | Controlword quick stop with 0x605A = 0 (coast), or disable voltage |
| `drv decay slow\|fast` | U | — |
| `drv limit <pct>` | S | 0x2004 `duty_limit`. Also changes the cfg RAM value (unlike the console) |
| `drv current [n]` | U | Current in TPDO2 |
| `drv zero` | U | Refused while the bridge is enabled |
| `drv iscan …` | U | — |
| `drv trip <mA>` | S | 0x2003 `trip_ma`. Also changes the cfg RAM value |
| `drv trip buf on\|off` | U | — |
| `drv clearfault` | R | Controlword bit 7 rising edge (fault reset) |
| `drv timeout [ms]` | U | Kept alive by the loop while armed (§6.3) |
| `drv ramp <o/oo/s>` | S | 0x2009 `ramp_pmps` |
| `drv ramp floor <o/oo>` | S | 0x200A `ramp_floor` |
| `vel` (state) | U | TPDO1 |
| `vel on` | R | Controlword 0x0F (enable operation) |
| `vel off` | R | Controlword 0x07 (disable operation, 0x605C = 0 → coast) |
| `vel target <rpm\|Nm>` | R | RPDO1 0x60FF (0.001 rpm) |
| `vel stop` | R | Controlword halt bit 8 (0x605D = 1, ramp by `vel_slew`) |
| `vel reset` | U | Bench only |
| `vel gains` | S | Read 0x200B–0x2012 |
| `vel timeout <ms>` | S | 0x2013 `vel_tmo` |
| `telem …` | U | Bench host stream |
| `cfg` (list) | S | Read 0x2000–0x2015 and 0x2F01 |
| `cfg <key>` | S | Read 0x2000 + key index |
| `cfg <key> <val>` | S | Write 0x2000 + key index |
| `cfg save` | S | 0x1010:01 ← `"save"` |
| `cfg revert` | S | 0x2F00 ← 1 (mfr; no CiA 301 object) |
| `cfg default` | S | 0x1011:01 ← `"load"` (Q11) |
| `cfg default <key>` | S | Write the EDS `DefaultValue` to 0x2000 + key index |
| `cfg help` | U | Names, units and ranges are in the EDS (§9) |
| `id` | U | Node-ID is in every COB-ID; DIP and role in 0x2F02 |
| `reset` | N | NMT Reset Node (0x81) |

## 3. Object dictionary

### 3.1 Mandatory objects

| Index:Sub | Name | Type | Access | Value / default |
|---|---|---|---|---|
| 0x1000 | Device type | U32 | ro | 0x00020192 (CiA 402, servo drive) |
| 0x1001 | Error register | U8 | ro | bit 0 generic, 1 current, 2 voltage, 3 temperature, 4 communication, 5 device profile, 7 manufacturer |
| 0x1017 | Producer heartbeat time | U16 | rw | **100 ms** (was 500 ms / 2 Hz). 0 = off |
| 0x1018:01 | Vendor-ID | U32 | ro | 0x00000000 (unregistered, Q12) |
| 0x1018:02 | Product code | U32 | ro | 0x00000001 = wheel node |
| 0x1018:03 | Revision number | U32 | ro | (OD major << 16) \| OD minor, starts 0x00010000. Major bumps when a PDO layout or object meaning changes |
| 0x1018:04 | Serial number | U32 | ro | XOR of the three STM32 UID words |

### 3.2 Communication objects

| Index:Sub | Name | Type | Access | Value / default | Note |
|---|---|---|---|---|---|
| 0x1003:00–08 | Pre-defined error field | U32 | ro/rw | 8 entries | EMCY history; write 0 to sub 0 clears |
| 0x1005 | COB-ID SYNC | U32 | rw | 0x00000080 | Consumer only |
| 0x1006 | Communication cycle period | U32 | rw | 20000 µs | Set by master; 0 = not monitored |
| 0x1010:01 | Store parameters, all | U32 | rw | read 0x00000001 | Write 0x65766173 (`"save"`) → `config_save()` (§3.4) |
| 0x1011:01 | Restore default parameters, all | U32 | rw | read 0x00000001 | Write 0x64616F6C (`"load"`) → `config_reset_all()` |
| 0x1014 | COB-ID EMCY | U32 | ro | 0x80 + node-ID | |
| 0x1015 | Inhibit time EMCY | U16 | rw | 1000 (× 100 µs = 100 ms) | |
| 0x1016:01 | Consumer heartbeat time | U32 | rw | 0x007F00FA | Master node-ID 0x7F, 250 ms |
| 0x1019 | Synchronous counter overflow | U8 | rw | 0 | No SYNC counter (§7) |
| 0x1029:01 | Error behaviour, communication | U8 | rw | 0 | Comm error → pre-operational |
| 0x1200:01/02 | SDO server COB-IDs | U32 | ro | 0x600 + id / 0x580 + id | Expedited SDO is enough for every object |
| 0x1400 | RPDO1 communication | record | rw | §4.1 | |
| 0x1401 | RPDO2 communication | record | rw | §4.1 | Disabled (bit 31) on CENTER nodes |
| 0x1600/0x1601 | RPDO1/2 mapping | record | ro | §4.1 | Fixed mapping (Q13) |
| 0x1800–0x1803 | TPDO1–4 communication | record | rw | §4.2 | TPDO3 disabled on CENTER nodes |
| 0x1A00–0x1A03 | TPDO1–4 mapping | record | ro | §4.2 | Fixed mapping (Q13) |

COB-IDs, predefined connection set (node-ID *n* = 1–7). No deviation: nothing
in this design needs one.

| Object | COB-ID | Dir | Used |
|---|---|---|---|
| NMT | 0x000 | O→N | yes, incl. node 0 = all |
| SYNC | 0x080 | O→N | yes |
| EMCY | 0x080 + n | N→O | yes |
| TIME | 0x100 | — | no |
| TPDO1 / RPDO1 | 0x180 + n / 0x200 + n | N→O / O→N | yes |
| TPDO2 / RPDO2 | 0x280 + n / 0x300 + n | N→O / O→N | yes (RPDO2 CORNER only) |
| TPDO3 / RPDO3 | 0x380 + n / 0x400 + n | N→O / — | TPDO3 CORNER only; RPDO3 unused |
| TPDO4 / RPDO4 | 0x480 + n / 0x500 + n | N→O / — | TPDO4 yes; RPDO4 unused |
| SDO | 0x580 + n / 0x600 + n | N→O / O→N | yes |
| Heartbeat / boot-up | 0x700 + n | N→O | yes; orion's own on 0x77F |

Priority (lowest ID wins): NMT > SYNC > EMCY > TPDO1 > RPDO1 > TPDO2 > RPDO2 >
TPDO3 > TPDO4 > SDO > heartbeat.
- A status TPDO beats the command RPDO of the same number. Cost: at most one
  frame time (≈ 0.54 ms) per contending frame. Accepted.
- The removed 0x500+ID heartbeat sat in the RPDO4 range (0x500 + node-ID); its
  payload moves to TPDO4.

### 3.3 Application objects — drive axis (CiA 402)

| Index:Sub | Name | Type | Access | PDO | Unit | Scaling | Range | Default | Replaces |
|---|---|---|---|---|---|---|---|---|---|
| 0x603F | Error code | U16 | ro | — | — | EMCY code of the active fault | — | 0 | `drv` fault line |
| 0x6040 | Controlword | U16 | rw | RPDO1 | — | bits §1.4 | — | 0 | `drv enable/disable`, `vel on/off/stop`, `drv clearfault`, `drv coast/brake` |
| 0x6041 | Statusword | U16 | ro | TPDO1 | — | bits §1.4 | — | — | `drv`, `vel` state |
| 0x605A | Quick stop option code | I16 | rw | — | — | 0 coast, 1 ramp (`vel_slew`) then disable, −1 brake then disable | −1, 0, 1 | 0 (W4 stop policy) | `drv coast` / `drv brake` |
| 0x605B | Shutdown option code | I16 | rw | — | — | 0 coast | 0 | 0 | — |
| 0x605C | Disable operation option code | I16 | rw | — | — | 0 coast, 1 ramp then disable | 0, 1 | 0 | `vel off` |
| 0x605D | Halt option code | I16 | rw | — | — | 1 ramp to 0 by `vel_slew`, stay enabled | 1 | 1 | `vel stop` |
| 0x605E | Fault reaction option code | I16 | rw | — | — | 0 coast | 0 | 0 | — |
| 0x6007 | Abort connection option code | I16 | rw | — | — | 0 none, 1 fault, 2 disable voltage, 3 quick stop | 0–3 | **1** | §6.1 |
| 0x6060 | Modes of operation | I8 | rw | RPDO1 | — | — | 9 | 9 | — |
| 0x6061 | Modes of operation display | I8 | ro | TPDO1 | — | — | 9 | 9 | — |
| 0x6064 | Position actual value | I32 | ro | TPDO2 | encoder counts | as `enc` | — | 0 at boot | `enc` count |
| 0x606C | Velocity actual value | I32 | ro | TPDO1 | 0.001 rpm | wheel output shaft | — | — | `enc` / `vel` speed |
| 0x60FF | Target velocity | I32 | rw | RPDO1 | 0.001 rpm | wheel output shaft; sign = direction, as console | −100 000 … +100 000 (Q2) | 0 | `vel target` |
| 0x6502 | Supported drive modes | U32 | ro | — | — | bit 8 = CSV | — | 0x00000100 | — |

Out-of-range handling (drive):
- SDO write outside range → abort 0x06090030/31/32; nothing changed.
- RPDO1 0x60FF outside range → the whole RPDO1 is rejected (previous target kept,
  no watchdog kick), statusword bit 14 set, 0x2120:05 = 3 (RANGE).
- RPDO1 0x6060 ≠ 9 → RPDO1 rejected, 0x2120:05 = 4 (MODE).
- Rejected RPDO1s still apply controlword stop requests (§7.2).

### 3.4 Application objects — manufacturer-specific

Drive status (0x2110–0x211F):

| Index:Sub | Name | Type | Access | PDO | Unit | Scaling | Range | Default | Replaces |
|---|---|---|---|---|---|---|---|---|---|
| 0x2111 | Motor current | U16 | ro | TPDO2 | mA | 1 mA; magnitude as `drv current` | — | — | `drv current` |
| 0x2112 | Loop output | I16 | ro | TPDO2 | o/oo | applied duty | ±1000 | — | `drv`, `vel` |
| 0x2113 | Drive flags | U8 | ro | — | — | bit 0 saturated, 1 ramping, 2 `vel_tmo` latched, 3 UART owns motion, 4 at current trip | — | — | `drv`, `vel` |

Steering axis (0x2100–0x210F; CORNER and RESERVED only; CENTER aborts 0x06020000):

| Index:Sub | Name | Type | Access | PDO | Unit | Scaling | Range | Default | Out of range | Replaces |
|---|---|---|---|---|---|---|---|---|---|---|
| 0x2100 | Steer control | U8 | rw | RPDO2 | — | bit 0 enable, bit 1 stop (acts on 0→1); bits 2–7 = 0 | 0–3 | 0 | bits 2–7 ignored | `mks enable`, `mks stop` |
| 0x2101 | Steer target | I16 | rw | RPDO2 | ° at gearbox output | 0.01° (1 pulse = 0.0118°); **absolute** from steering zero | −9000 … +9000 (Q3) | 0 | RPDO2 rejected, no motion, 0x2105 = RANGE | `mks deg` |
| 0x2102 | Steer speed | U8 | rw | RPDO2 | MKS speed code | — | 1–127; 0 = node default 2 | 0 | RPDO2 rejected | `mks deg [speed]` |
| 0x2103 | Steer position | I16 | ro | TPDO3 | ° | 0.01°, tracked absolute (Q3) | — | — | — | `mks angle` |
| 0x2104 | Steer status | U8 | ro | TPDO3 | — | bit 0 enabled, 1 moving, 2 protect/stall, 3 position valid, 4 UART transaction error | — | — | — | `mks en/protect` |
| 0x2105 | Steer result | U8 | ro | TPDO3 | — | result of the last RPDO2, codes §3.5 | — | 0 | — | — |

Motion counters (0x2120):

| Index:Sub | Name | Type | Access | PDO | Range | Default | Note |
|---|---|---|---|---|---|---|---|
| 0x2120:01 | RPDO1 counter | U8 | rw | RPDO1 | any | — | §7.1 |
| 0x2120:02 | RPDO2 counter | U8 | rw | RPDO2 | any | — | §7.1 |
| 0x2120:03 | Last accepted RPDO1 counter | U8 | ro | TPDO1 | — | — | Echo = acknowledgement |
| 0x2120:04 | Last accepted RPDO2 counter | U8 | ro | TPDO3 | — | — | Echo |
| 0x2120:05 | Last RPDO1 reject reason | U8 | ro | — | codes §3.5 | 0 | SDO-readable |
| 0x2120:06 | Skipped-counter total | U16 | ro | — | — | 0 | |
| 0x2120:07 | Counter check enable | U8 | rw | — | 0, 1 | **1** | 0 only for stock-driver bench tests |

CAN diagnostics (0x2130, TPDO4):

| Index:Sub | Name | Type | Access | Replaces |
|---|---|---|---|---|
| 0x2130:01 | TEC | U8 | ro | old heartbeat byte 4, `errors` |
| 0x2130:02 | REC | U8 | ro | old heartbeat byte 5 |
| 0x2130:03 | LEC | U8 | ro | old heartbeat byte 6 |
| 0x2130:04 | Bus flags | U8 | ro | bit 0 warning, 1 passive, 2 bus-off seen since boot |
| 0x2130:05 | RX ring dropped | U16 | ro | `stats` `rx_ring_dropped` |
| 0x2130:06 | Bus-off count | U16 | ro | — |

Configuration keys (0x2000 + key index). All I32, rw, not PDO-mappable. The
index is `0x2000 + config_key_t`, so key order is part of the protocol: keys
are only ever appended. A write applies live, exactly as `cfg <key> <val>`
does (trip, limit, ramp and floor re-applied; `vel_*` read live), and stays in
RAM until 0x1010. Out of range → SDO abort 0x06090031 (high) / 0x06090032
(low); **rejected, not clamped** (`config_set()` semantics).

| Index | Key | Unit | Min | Max | Default | Replaces |
|---|---|---|---|---|---|---|
| 0x2000 | `vdda_mv` | mV | 2000 | 3600 | `ISENSE_VDDA_MV_DEFAULT` | `cfg vdda_mv` |
| 0x2001 | `r_ipropi` | Ω | 100 | 10000 | `ISENSE_R_IPROPI_OHM_DEFAULT` | `cfg r_ipropi` |
| 0x2002 | `a_ipropi` | µA/A | 100 | 2000 | `ISENSE_A_IPROPI_UA_PER_A_DEFAULT` | `cfg a_ipropi` |
| 0x2003 | `trip_ma` | mA | 0 | 1600 | `ISENSE_TRIP_DEFAULT_MA` | `cfg trip_ma`, `drv trip` |
| 0x2004 | `duty_limit` | o/oo | 0 | 1000 | `DRIVE_DUTY_MAX` | `cfg duty_limit`, `drv limit` |
| 0x2005 | `rail_mv` | mV | 0 | 40000 | 12000 | `cfg rail_mv` |
| 0x2006 | `isense_avg` | — | 1 | 1024 | `ISENSE_AVG_DEFAULT` | `cfg isense_avg` |
| 0x2007 | `sat_raw` | — | 1000 | 4095 | `ISENSE_SATURATED_RAW_DEFAULT` | `cfg sat_raw` |
| 0x2008 | `vref_div` | — | 1 | 4 | 3 | `cfg vref_div` |
| 0x2009 | `ramp_pmps` | o/oo/s | 0 | 10000 | 0 | `cfg ramp_pmps`, `drv ramp` |
| 0x200A | `ramp_floor` | o/oo | 0 | 300 | 0 | `cfg ramp_floor`, `drv ramp floor` |
| 0x200B | `vel_kp` | m o/oo/rpm | 0 | 100000 | 3000 | `cfg vel_kp`, `vel gains` |
| 0x200C | `vel_ki` | m/rpm·s | 0 | 200000 | 10000 | `cfg vel_ki` |
| 0x200D | `vel_kd` | m o/oo·s/rpm | 0 | 100000 | 0 | `cfg vel_kd` |
| 0x200E | `vel_ff_a` | m o/oo/rpm | 0 | 100000 | 12510 | `cfg vel_ff_a` |
| 0x200F | `vel_ff_b` | o/oo | 0 | 300 | 30 | `cfg vel_ff_b` |
| 0x2010 | `vel_ilim` | o/oo | 0 | 1000 | 150 | `cfg vel_ilim` |
| 0x2011 | `vel_max` | o/oo | 0 | 1000 | 300 | `cfg vel_max` |
| 0x2012 | `vel_slew` | m rpm/s | 0 | 1000000 | 4000 | `cfg vel_slew` |
| 0x2013 | `vel_tmo` | ms | 0 | 60000 | 1000 | `cfg vel_tmo`, `vel timeout` |
| 0x2014 | `isense_dk` | o/oo | 400 | 1000 | 690 | `cfg isense_dk` |
| 0x2015 | `isense_dmin` | o/oo | 30 | 145 | 60 | `cfg isense_dmin` |

Defaults named by macro are compiled in the owning module (`isense.h`,
`drive.h`); the EDS takes their numeric value (§9).

Config service objects (0x2F00):

| Index:Sub | Name | Type | Access | Value | Replaces |
|---|---|---|---|---|---|
| 0x2F00 | Config command | U8 | wo | 1 = revert (reload from flash, re-apply live) | `cfg revert` |
| 0x2F01:01 | `CONFIG_VERSION` | U8 | ro | 2 | `cfg` |
| 0x2F01:02 | Slots used | U16 | ro | — | `cfg` |
| 0x2F01:03 | Slots total | U16 | ro | 1024 | `cfg` |
| 0x2F01:04 | Dirty | U8 | ro | 0/1 (`config_dirty()`) | `cfg` |
| 0x2F01:05 | Last load result | U8 | ro | `config_load_t` | boot line |
| 0x2F02:01 | DIP code | U8 | ro | 0–6 | `id` |
| 0x2F02:02 | Role | U8 | ro | 0 CORNER, 1 CENTER, 2 RESERVED | `id` |

0x1010 / 0x1011 rules:

| Write | Node action | SDO result |
|---|---|---|
| 0x1010:01 ← `"save"` | `config_save()` | OK; UNCHANGED is also OK |
| same, bridge enabled | Refused (flash stalls the core, `config.h`) | Abort 0x08000022 (device state) |
| same, flash error | — | Abort 0x06060000 (hardware error) |
| 0x1010:01 ← anything else | Nothing | Abort 0x08000020 |
| 0x1011:01 ← `"load"` | `config_reset_all()`: RAM, applied live; persistent only after 0x1010 | OK (Q11) |
| 0x1011:01 ← anything else | Nothing | Abort 0x08000020 |

### 3.5 Result and reject codes (0x2105, 0x2120:05)

| Code | Name | When |
|---|---|---|
| 0 | OK | Accepted |
| 1 | REPEAT | Counter repeated |
| 2 | STALE | Counter older than the last accepted |
| 3 | RANGE | A mapped value out of range |
| 4 | MODE | 0x6060 ≠ 9 |
| 5 | NOT_ENABLED | Steering not enabled, or the drive axis not in operation enabled |
| 6 | UART_OWNS | Motion is owned by the console (§6.6) |
| 7 | NOT_SUPPORTED | Steering object on a CENTER node |

## 4. PDO mapping

### 4.1 RPDOs (O→N)

| RPDO | COB-ID | Transmission type | Rate | Deadline (0x140x:05) | Nodes |
|---|---|---|---|---|---|
| RPDO1 drive | 0x200 + n | 1 (synchronous: applied at the next SYNC) | every SYNC, 50 Hz | **100 ms** | all |
| RPDO2 steering | 0x300 + n | 254 (event: applied on reception) | on change, ≤ 10 Hz | 0 (none) | CORNER, RESERVED |

RPDO1, DLC 8:

| Byte | Object | Field | Type |
|---|---|---|---|
| 0–1 | 0x6040 | Controlword | U16 |
| 2 | 0x6060 | Modes of operation | I8 |
| 3–6 | 0x60FF | Target velocity | I32 |
| 7 | 0x2120:01 | Counter | U8 |

RPDO2, DLC 5:

| Byte | Object | Field | Type |
|---|---|---|---|
| 0 | 0x2100 | Steer control | U8 |
| 1 | 0x2102 | Steer speed | U8 |
| 2–3 | 0x2101 | Steer target | I16 |
| 4 | 0x2120:02 | Counter | U8 |

- A PDO with fewer bytes than mapped is discarded; EMCY 0x8210.
- A STEER target that arrives while a move is running replaces the target.

### 4.2 TPDOs (N→O)

| TPDO | COB-ID | Transmission type | Event timer | Rate | Nodes |
|---|---|---|---|---|---|
| TPDO1 drive status | 0x180 + n | 1 (every SYNC) | — | 50 Hz | all |
| TPDO2 drive feedback | 0x280 + n | 5 (every 5th SYNC) | — | 10 Hz | all |
| TPDO3 steering status | 0x380 + n | 254 | 100 ms | 10 Hz | CORNER, RESERVED |
| TPDO4 CAN diagnostics | 0x480 + n | 254 | 1000 ms | 1 Hz | all |

TPDO1, DLC 8: bytes 0–1 0x6041 statusword (U16); 2 0x6061 mode display (I8);
3–6 0x606C velocity actual (I32, 0.001 rpm); 7 0x2120:03 last accepted RPDO1
counter (U8).

TPDO2, DLC 8: bytes 0–3 0x6064 position actual (I32, counts); 4–5 0x2111
current (U16, mA); 6–7 0x2112 loop output (I16, o/oo).

TPDO3, DLC 7: bytes 0–1 0x2103 steer position (I16, 0.01°); 2–3 0x2101 steer
target (I16, 0.01°); 4 0x2104 steer status (U8); 5 0x2105 steer result (U8);
6 0x2120:04 last accepted RPDO2 counter (U8).

TPDO4, DLC 8: bytes 0 TEC, 1 REC, 2 LEC, 3 bus flags (U8 each, 0x2130:01–04);
4–5 RX ring dropped (U16); 6–7 bus-off count (U16).

Rules:
- TPDOs are sent in NMT operational only (CiA 301).
- TPDO1/2 are sampled at the SYNC; the control step is phase-locked to the
  encoder window, not to SYNC (Q7).

### 4.3 SYNC

- Produced by orion at 20 ms (50 Hz), no counter (0x1019 = 0).
- The node applies RPDO1 data at the SYNC after reception, and sends TPDO1
  (every SYNC) and TPDO2 (every 5th).

### 4.4 Bus load, 6 wheel nodes

Worst-case bits per standard frame, with stuffing: 47 + 8·n + ⌊(33 + 8·n)/4⌋
(n = data bytes). n = 0: 55; 1: 65; 5: 105; 7: 125; 8: 135.

| Stream | Frames/s at 50 Hz SYNC | Bits | Frames/s at 25 Hz SYNC | Bits |
|---|---|---|---|---|
| SYNC (n 0) | 50 | 2 750 | 25 | 1 375 |
| RPDO1 (n 8), 6 nodes | 300 | 40 500 | 150 | 20 250 |
| TPDO1 (n 8), 6 nodes | 300 | 40 500 | 150 | 20 250 |
| TPDO2 (n 8), 6 nodes, every 5th SYNC | 60 | 8 100 | 30 | 4 050 |
| RPDO2 (n 5), 4 nodes, worst 10 Hz | 40 | 4 200 | 40 | 4 200 |
| TPDO3 (n 7), 4 nodes, 10 Hz | 40 | 5 000 | 40 | 5 000 |
| TPDO4 (n 8), 6 nodes, 1 Hz | 6 | 810 | 6 | 810 |
| Node heartbeat (n 1), 6 × 10 Hz | 60 | 3 900 | 60 | 3 900 |
| Orion heartbeat (n 1), 10 Hz | 10 | 650 | 10 | 650 |
| **Total** | **866 f/s** | **106 410 b/s → 42.6 %** | **511 f/s** | **60 485 b/s → 24.2 %** |

EMCY and SDO are sporadic and excluded, as is traffic from non-wheel nodes
(Q14).

## 5. NMT behaviour

### 5.1 Per state

| NMT state | SDO | PDO | EMCY | Heartbeat | SYNC | Drive axis | Steering | UART motion |
|---|---|---|---|---|---|---|---|---|
| Initialisation | no | no | no | boot-up (0x700+n, 0x00) at the end | no | Config load; bridge disabled; 402 → switch on disabled | Disabled | Refused |
| Pre-operational | yes | no | yes | yes | ignored | 402 transitions allowed up to **switched on**; operation enabled refused | Refused | Allowed only in bench mode (§5.4) |
| Operational | yes | yes | yes | yes | yes | Full 402 | Allowed in operation enabled | Refused while CAN owns (§6.6) |
| Stopped | no | no | no | yes | ignored | Coast, bridge disabled, 402 → switch on disabled | `mks stop`, disabled | **Refused** (e-stop state) |

### 5.2 Transitions

| Event | Node action |
|---|---|
| Leave operational (to pre-op, stopped, reset) | 0x6007 action (fault → coast) if in operation enabled; `mks stop`; RPDO data discarded |
| NMT Stop to node 0 (0x000: 02 00) | **Emergency stop.** Highest-priority ID on the bus; no counter, no payload checks |
| NMT Start after Stop | Node → operational, drive stays in switch on disabled / fault: orion must fault-reset and run shutdown → switch on → enable operation |
| NMT Reset Communication | Comm objects reset from the OD; then as leaving operational |
| NMT Reset Node | `NVIC_SystemReset()`; config reloads from flash (unsaved SDO writes lost) |
| Enter pre-operational | As leaving operational |

### 5.3 Motion gating (the rule)

1. The 402 transition to **operation enabled** (transition 4) is refused unless
   NMT is operational. Statusword stays at switched on.
2. Leaving operational forces the drive out of operation enabled (5.2).
3. Steering moves only when NMT is operational, the drive axis is in operation
   enabled, and 0x2100 bit 0 is set. Any quick stop, fault or NMT change stops
   the steering (`mks stop`).
4. SDO writes never cause motion.

### 5.4 Bench mode (no master)

- The node never self-starts (0x1F80 absent): without a master it stays
  pre-operational.
- Bench mode = pre-operational **and** no master heartbeat seen since boot. In
  bench mode the UART console works as today.
- The first master heartbeat ends bench mode until reset (Q15).

## 6. Safety

### 6.1 Heartbeat

| Item | Rule |
|---|---|
| Producer | 0x1017 = 100 ms; boot-up message after initialisation |
| Consumer | 0x1016:01 = master 0x7F, 250 ms. Monitoring starts at the first master heartbeat (CiA 301) |
| Master lost | EMCY 0x8130; 0x6007 = 1 → fault (coast), `mks stop`; 0x1029:01 = 0 → pre-operational |
| Recovery | Master heartbeat resumes → NMT Start → fault reset → enable sequence |
| Orion side | ros2_canopen monitors node heartbeats (≥ 3 × 100 ms suggested) |

### 6.2 RPDO timeout vs `vel_tmo`

| Monitor | Watches | Kicked by | Time | On expiry |
|---|---|---|---|---|
| RPDO1 deadline (0x1400:05) | RPDO1 frames stop arriving | Any received RPDO1 | 100 ms (5 SYNC periods) | Only in operation enabled: EMCY 0x8250, fault → coast |
| `vel_tmo` (0x2013) | No **new** setpoint | Accepted RPDO1 only (via `velocity_set_setpoint()`); repeats and rejects do not kick | 1000 ms default (Q16) | Existing: setpoint 0, coast, latch; statusword bit 15; EMCY 0xFF01 |
| Heartbeat consumer | Master node dead | Master heartbeat | 250 ms | §6.1 |
| SYNC | — | — | Not monitored (0x1006 may be set) | — |

- `vel_tmo` stays and applies to CAN, unchanged. It is the only monitor that
  catches a live master stack sending a frozen application's last value
  (§7).
- The `vel_tmo` latch clears on 402 transition to operation enabled
  (`velocity_enable()`).
- Steering has no timeout; FD moves are finite.

### 6.3 `drv timeout`

UART-only and off by default. While the loop is armed, each loop step kicks
it, so liveness from orion is judged by the monitors in §6.2.

### 6.4 EMCY

Frame (0x080 + n, DLC 8): bytes 0–1 error code (U16), 2 error register
(0x1001), 3 axis (0 node, 1 drive, 2 steering), 4–7 info (U32, per code).
Sent on the rising edge; code 0x0000 when the last error clears.

| Code | Condition | 0x1001 bit | Info (bytes 4–7) | Reaction |
|---|---|---|---|---|
| 0x2310 | Current at `trip_ma` (ITRIP regulating) for > 100 ms (Q17) | 1 | current mA | Warning only (statusword bit 7) |
| 0x5410 | DRV8874 nFAULT (OCP, TSD, UVLO; not distinguishable) | 0 | 0 | Fault → coast; clear with fault reset (`drv clearfault`) |
| 0x7121 | Steering protect/stall (MKS locked-rotor) | 7 | MKS status | `mks stop`; steer status bit 2 |
| 0x7121 | Drive stall (Q17) | 7 | current mA | Fault → coast |
| 0x7510 | MKS UART transaction error | 7 | error count | Steer status bit 4 |
| 0x8110 | CAN overrun: RX ring dropped or FIFO overrun | 4 | dropped count | None; counter rules cover lost commands |
| 0x8120 | CAN error-passive | 4 | TEC << 8 \| REC | None (RX still works) |
| 0x8130 | Heartbeat consumer timeout | 4 | master node-ID | §6.1 |
| 0x8140 | Recovered from bus-off | 4 | bus-off count | Sent after rejoin |
| 0x8210 | PDO length error | 4 | COB-ID | PDO discarded |
| 0x8250 | RPDO1 deadline | 4 | COB-ID | Fault → coast |
| 0xFF01 | `vel_tmo` expired | 7 | `vel_tmo` ms | Existing watchdog action |
| 0xFF02 | ≥ 5 counter values skipped at once | 7 | skipped count | None (frame accepted) |
| 0xFF03 | RPDO rejected (REPEAT / STALE / RANGE / MODE) | 7 | reason \| (counter << 8) | Frame not applied; rate-limited to 1 per 100 ms |

### 6.5 Bus errors

| State | Detection | Node action |
|---|---|---|
| Error-warning | ESR, polled every main-loop pass | TPDO4 flag only |
| Error-passive | ESR, polled | Keep running. EMCY 0x8120 once per entry |
| Bus-off | ESR `BOFF`, polled | **Immediately**: fault → coast, `mks stop`. `AutoBusOff = ENABLE` rejoins after 128 × 11 recessive bits. Then EMCY 0x8140 and 0x1029:01 → pre-operational. Motion resumes only after NMT Start + fault reset + enable |
| RX ring overflow | `rx_ring_dropped` increments | EMCY 0x8110 (inhibit-limited) |

No SCE interrupt (ISR-to-ring plan scope); detection is by polling.

### 6.6 UART and CAN arbitration

| Command class | Rule |
|---|---|
| Stops: NMT Stop/pre-op, controlword quick stop / disable voltage / disable operation / halt, RPDO2 stop bit; console `vel off`, `vel stop`, `drv coast`, `drv brake`, `drv disable`, `mks stop` | **Always accepted from either source**, including inside a rejected RPDO |
| Motion: 402 enable path, 0x60FF, RPDO2 enable/target; console `drv enable`, `vel on`, `vel target`, `drv duty`, `mks move`/`deg`, `drv limit`/`trip`/`ramp` | **The source that armed owns motion** until a disarm or stop from either source. CAN: transition 4 refused, statusword bit 9 (remote) = 0, reject code UART_OWNS. Console prints "CAN owns motion — 'vel off' first" |
| Config: SDO 0x2000–0x2015, 0x1010, 0x1011; `cfg …` | Both allowed; last write wins. Save refused while the bridge is enabled |
| Diagnostics | UART only |

- After boot nobody owns motion.
- NMT stopped refuses UART motion too (it is the e-stop state).

## 7. Integrity

### 7.1 What each mechanism catches

| Failure | CAN CRC-15 | Heartbeat | RPDO deadline | SYNC counter | App counter | App CRC |
|---|---|---|---|---|---|---|
| Wire corruption | yes | — | — | — | — | yes (redundant) |
| Master dead / cable cut | — | yes | yes | — | — | — |
| Master stack alive, application frozen, same RPDO re-sent each SYNC | — | **no** (stack thread) | **no** (frames arrive) | **no** (stack thread) | **yes** | no |
| Lost or reordered commands | — | — | partly | SYNCs only | yes | no |
| Layout mismatch (wrong OD/firmware pairing) | — | — | — | — | — | yes |
| Corruption in host/node software buffers | — | — | — | — | — | yes |

- The lely master in ros2_canopen sends synchronous PDOs from its own cyclic
  task using the last written values; heartbeat and SYNC come from the same
  task. None of them proves the control application is alive.
- Layout mismatch is covered without a CRC: the master checks 0x1000 and 0x1018
  (revision) at boot against its DCF, and refuses a node that differs.

### 7.2 Decision

| Item | Decision |
|---|---|
| Application counter in motion RPDOs | **Yes**: RPDO1 byte 7, RPDO2 byte 4 |
| Application CRC | **No.** Remaining gain is in-memory corruption only; costs a byte and makes every PDO non-standard |
| SYNC counter (0x1019) | **No** (0). Detects lost SYNCs only |

Counter rules (per RPDO; `d = (ctr − last) mod 256`):

| `d` | Meaning | Node action |
|---|---|---|
| 1 | Next | Accept |
| 2–127 | `d − 1` skipped | Accept; add to 0x2120:06; EMCY 0xFF02 if `d − 1 ≥ 5` |
| 0 | Repeat | **Reject**: target not applied, `vel_tmo` not kicked; stop bits in the controlword still act |
| 128–255 | Stale | Reject, as above |

- Re-synchronisation: the first RPDO after boot, after NMT Start and after
  transition 4 (operation enabled) is accepted with any counter.
- Orion increments the counter once per new command, not once per frame.
- Cost: the stock ros2_canopen `cia402_driver` does not write 0x2120:01. With
  0x2120:07 = 1 (default) it cannot command motion; the custom driver of §1.3
  must (Q4).

## 8. Stack

### 8.1 Options

| | Hand-written minimal slave | CANopenNode v4 core + own driver port | CANopenNode v4 + CanOpenSTM32 driver (REST) |
|---|---|---|---|
| Scope | NMT slave, heartbeat producer + 1 consumer, expedited SDO server, fixed PDOs, EMCY, SYNC consumer, 0x1010/0x1011 | Full CiA 301 (segmented/block SDO, configurable PDOs, LSS, error history, storage hooks) | Same |
| CiA 402 state machine | Hand-written | Hand-written (CANopenNode has no 402) | Hand-written |
| Flash (estimate, Q6) | +4–8 KB | +15–25 KB | +15–25 KB |
| RAM (estimate) | < 1 KB | +2–4 KB | +2–4 KB |
| Result vs 108 KB / 6 KB now | ~115 KB / ~7 KB | ~130 KB / ~10 KB | ~130 KB / ~10 KB |
| Headroom (384 KB flash region, sector 7 reserved; 128 KB RAM) | Ample | Ample | Ample |
| Effort | ~2 weeks incl. 402 and testing against lely | ~1 week integration + 402 | ~3–5 days integration + 402 |
| Conformance risk with ros2_canopen | Highest: partial SDO, fixed mapping; found only in integration | Low | Low |
| RX path | Existing ISR-to-ring, unchanged; main loop dispatches by COB-ID | **Existing ring kept**: the port's receive function is called from the main loop for each popped frame | **Replaced** (below) |
| Tooling | Own EDS generator | CANopenEditor (OD.c/OD.h + EDS) | Same |
| Licence | Own | Apache-2.0 | Apache-2.0 |

### 8.2 RX path — impact on finished work

- ISR-to-ring is **merged and closed** (task 6, bench-verified Sep 27–28):
  `CAN1_RX0_IRQn` at preemption 1 drains FIFO0 into a 32-frame ring; the main
  loop pops it.
- **CanOpenSTM32 brings its own RX path.** Its driver handles the HAL RX-FIFO
  callback itself, matches the ID against the stack's receive table and runs
  the stack's receive callbacks **in interrupt context**. It also owns TX
  (mailbox-empty interrupt and its own TX buffer) and the acceptance filter.
  Adopting it as-is replaces the ring, the RX0 handler and `can_bus` TX, and
  changes the verified priority arrangement.
- CANopenNode's driver layer (`CO_driver.h`) is a porting interface. A port on
  top of the ring keeps task 6's work: `CO_CANsend` → `can_bus` TX; each popped
  frame → the port's receive dispatch in the main loop. Cost: RPDO/SYNC
  handling gets main-loop latency (heartbeat jitter measured ≤ 0.74 ms, a
  proxy). Unverified; proving it is the first integration step.
- Console CAN commands (`send`, `monitor`, `loopback`, `canhold`) must share TX
  with the stack in every option.

### 8.3 Recommendation

CANopenNode v4 core with **its own driver port over the existing ISR-to-ring
path** (column 2). It gives conformance with the lely master at moderate cost,
and keeps task 6's merged RX path. CanOpenSTM32's driver, which REST names, is
not used (Q6).

## 9. EDS

| Item | Decision |
|---|---|
| Single source of truth | **Yes**: one OD description in the repo. EDS, `OD.c`/`OD.h` and orion's DCF are generated from it |
| With CANopenNode | CANopenEditor project (XDD/XPD) is the source; it exports the EDS and `OD.c`/`OD.h` |
| With a hand-written stack | A small script generating the EDS and a C table from one YAML file |
| Orion | ros2_canopen `bus.yml` references the EDS; lely `dcfgen` builds the DCF |
| cfg keys | `config.c` stays the source of min/max/default for 0x2000–0x2015 (it is also the UART's documentation). A check script, run in the build, compares the EDS `LowLimit`/`HighLimit`/`DefaultValue` with `config.c` and fails on a mismatch (Q18) |
| Versioning | Any OD change bumps 0x1018:03; a PDO layout or meaning change bumps its major part |
| Bench | python-canopen with the same EDS on the CANable |

## 10. Open questions and assumptions

1. **Plain CAN or CANopen.** This document and `docs/can_cmds.md` are the two
   options. REST selects CANopen; the W6 plain-CAN request did not. Decide
   before any implementation.
2. **Speed range and unit.** Assumed ±100 rpm at the wheel output in
   0.001 rpm. `vel_max` 300 caps it near 21 rpm. ros2_canopen scale:
   1 rad/s = 9549.3 units.
3. **Absolute steering position (open W6 item).** How the tracked position is
   kept (counting commanded pulses or reading the encoder) and where zero is.
   ±90° assumed; the real limit is unknown.
4. **ros2_canopen capabilities — not verified.** Whether its `cia402_driver`
   supports more than one axis per node, supports CSV (mode 9), and how a
   custom driver adds the steering PDO and the counter. The profile decision
   (§1.3) assumes one axis per node and a custom driver.
5. **CSV vs PV.** REST chose CSV (SYNC-timed). Profile velocity (mode 3) would
   need no SYNC and could use event-driven RPDOs; the loop already slews
   (`vel_slew`).
6. **Stack and driver.** REST names CanOpenSTM32; §8.3 recommends a CANopenNode
   port over the ISR-to-ring path instead. Flash/RAM figures are estimates;
   measure with a build.
7. **SYNC vs control step.** The loop step is phase-locked to the encoder
   window (`enc window 20` → 50 Hz), not to SYNC. Lock the step to SYNC, or
   accept up to one step of phase error?
8. **Orion node-ID 0x7F** is assumed, as are the node-IDs of the other two
   planned bus nodes.
9. **Role 6 (RESERVED/bench)** is assumed to have steering like a CORNER.
10. **Open-loop duty over CAN.** `drv duty` stays UART-only. Needed for rover τ
    tests driven from orion?
11. **0x1011 semantics.** CiA 301 restores defaults after the next reset; this
    spec applies them live in RAM and needs 0x1010 to persist, as `cfg default`
    does.
12. **Vendor-ID.** 0 is unregistered; a CiA vendor-ID costs a registration.
13. **Fixed PDO mapping.** Mapping objects are read-only. If ros2_canopen's DCF
    writes mappings at boot, they must be writable (CANopenNode supports it).
14. **Other bus traffic.** The load table covers the 6 wheel nodes only.
15. **Bench mode (§5.4).** UART motion in pre-operational without a master is a
    deviation from "no motion outside operational". Confirm.
16. **`vel_tmo` value.** 1000 ms default is long for a moving rover at 50 Hz; it
    is the only frozen-application detector.
17. **ITRIP and drive stall detection.** Neither exists in firmware today;
    method and thresholds undefined.
18. **Two sources for cfg metadata** (`config.c` and the EDS), reconciled by a
    check script. Generate one from the other instead?
19. **Flash save and comms.** A save normally appends 128 B (short), but when
    sector 7 is full the erase stalls the core ~1–3 s: no heartbeat, RX FIFO
    overflows, the master will see a heartbeat loss. Only possible with the
    bridge disabled.
20. **Steering gated by the drive axis state** (§5.3): steering needs the drive
    axis in operation enabled. Confirm this is wanted for steer-in-place.
21. **Stop policy.** Quick stop coasts (0x605A = 0, W4 stop policy); braking
    (−1) may be needed on slopes.

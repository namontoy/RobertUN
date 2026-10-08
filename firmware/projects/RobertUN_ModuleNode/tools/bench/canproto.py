"""W6 plain-CAN wheel-node protocol (docs/can_cmds.md): IDs, CRC-8, frame codecs.

Shared by cancmd.py and any test that needs to build or decode a frame.
Everything here is pure: no sockets, no state.
"""

import struct

PROTO_VER = 1

# Frame types, ID = (type << 4) | addr (§3). addr 0 = broadcast.
T_ESTOP = 0x00
T_STOP = 0x01
T_FAULT = 0x02
T_ARM = 0x03
T_SPEED = 0x04
T_STEER = 0x05
T_LIMITS = 0x06
T_RAMP = 0x07
T_CMD_RESULT = 0x08
T_STATUS_DRIVE = 0x10
T_STATUS_STEER = 0x11
T_HEARTBEAT = 0x50
T_CFG_REQ = 0x52
T_CFG_RESP = 0x53

TYPE_NAMES = {
    T_ESTOP: "ESTOP", T_STOP: "STOP", T_FAULT: "FAULT", T_ARM: "ARM",
    T_SPEED: "SPEED", T_STEER: "STEER", T_LIMITS: "LIMITS", T_RAMP: "RAMP",
    T_CMD_RESULT: "CMD_RESULT", T_STATUS_DRIVE: "STATUS_DRIVE",
    T_STATUS_STEER: "STATUS_STEER", T_HEARTBEAT: "HEARTBEAT",
    T_CFG_REQ: "CFG_REQ", T_CFG_RESP: "CFG_RESP",
}

RESULT_NAMES = [
    "OK", "REPEAT", "STALE", "RANGE", "NOT_ARMED", "NOT_SUPPORTED",
    "ESTOP_LATCHED", "FAULT_LATCHED", "CRC", "BAD_DLC", "UART_OWNS",
    "BAD_ACTION", "BUSY",
]

FAULT_NAMES = {
    1: "DRV_FAULT", 2: "VEL_WD_EXPIRED", 3: "ESTOP", 4: "BUS_OFF_RECOVERED",
    5: "ERROR_PASSIVE", 6: "RX_RING_DROPPED", 7: "MKS_ERROR", 8: "SKIPPED_CTR",
}

# STATUS_DRIVE byte 1 (§4.12), low bit first.
FLAG_NAMES = ["armed", "bridge", "drvfault", "sat", "wd", "estop", "ramp", "uart"]

# STATUS_STEER byte 1 (§4.13), low bit first.
STEER_FLAG_NAMES = ["enabled", "moving", "stall", "valid", "uarterr"]

ARM_ACTIONS = {
    "disarm": 0, "arm": 1, "clearfault": 2, "clearestop": 3,
    "steeron": 4, "steeroff": 5,
}

STOP_MODES = {"ramp": 0, "coast": 1, "brake": 2}
STOP_MKS = 0x80


# CFG_REQ ops (§4.8), CFG_RESP status (§4.9), key indices (§4.8 table).
CFG_OP_NAMES = ["get", "set", "save", "revert", "default", "default_all",
                "info", "min", "max", "def"]
CFG_STATUS = ["OK", "UNKNOWN_KEY", "RANGE", "BUSY", "FLASH_ERROR", "BAD_OP",
              "CRC"]
CFG_KEYS = ["vdda_mv", "r_ipropi", "a_ipropi", "trip_ma", "duty_limit",
            "rail_mv", "isense_avg", "sat_raw", "vref_div", "ramp_pmps",
            "ramp_floor", "vel_kp", "vel_ki", "vel_kd", "vel_ff_a", "vel_ff_b",
            "vel_ilim", "vel_max", "vel_slew", "vel_tmo", "isense_dk",
            "isense_dmin"]


def can_id(ftype, addr):
    return (ftype << 4) | (addr & 0x0F)


def split_id(cid):
    return cid >> 4, cid & 0x0F


def crc8(data, crc=0xFF):
    """CRC-8/SAE-J1850 without the final xor: poly 0x1D, no reflection."""
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def crc8_sae_j1850(data):
    return crc8(data) ^ 0xFF


def frame_crc(cid, payload):
    """§5.2: ID as u16 LE, PROTO_VER, then the payload without the CRC byte."""
    return crc8_sae_j1850(struct.pack("<HB", cid, PROTO_VER) + bytes(payload))


def seal(cid, body7):
    """Append the CRC to a 7-byte body, giving the 8-byte payload."""
    body7 = bytes(body7)
    assert len(body7) == 7
    return body7 + bytes([frame_crc(cid, body7)])


def arm_frame(addr, ctr, action):
    cid = can_id(T_ARM, addr)
    return cid, seal(cid, bytes([ctr & 0xFF, action, 0, 0, 0, 0, 0]))


def speed_frame(addr, ctr, milli_rpm):
    cid = can_id(T_SPEED, addr)
    return cid, seal(cid, struct.pack("<BBiB", ctr & 0xFF, 0, milli_rpm, 0))


def steer_frame(addr, ctr, cdeg, speed=0):
    """STEER: absolute angle in 0.01 deg, MKS speed code (0 = node default)."""
    cid = can_id(T_STEER, addr)
    return cid, seal(cid, struct.pack("<BBhBBB", ctr & 0xFF, speed & 0xFF,
                                      cdeg, 0, 0, 0))


def stop_frame(addr, ctr, mode):
    return can_id(T_STOP, addr), bytes([ctr & 0xFF, mode])


def estop_frame(addr=0):
    return can_id(T_ESTOP, addr), b""


def limits_frame(addr, ctr, mask, duty_limit=0, trip_ma=0):
    cid = can_id(T_LIMITS, addr)
    return cid, seal(cid, struct.pack("<BBHHB", ctr & 0xFF, mask, duty_limit,
                                      trip_ma, 0))


def ramp_frame(addr, ctr, mask, ramp_pmps=0, ramp_floor=0):
    cid = can_id(T_RAMP, addr)
    return cid, seal(cid, struct.pack("<BBHHB", ctr & 0xFF, mask, ramp_pmps,
                                      ramp_floor, 0))


def cfg_frame(addr, op, key=0, tag=0, value=0):
    """CFG_REQ: the CRC sits in byte 3 and covers bytes 0-2 and 4-7 (§4.8)."""
    cid = can_id(T_CFG_REQ, addr)
    val = struct.pack("<i", value)
    crc = frame_crc(cid, bytes([op, key, tag]) + val)
    return cid, bytes([op, key, tag, crc]) + val


def decode_cfg_resp(data):
    op, key, tag, st, value = struct.unpack("<BBBBi", bytes(data[:8]))
    return {"op": CFG_OP_NAMES[op] if op < len(CFG_OP_NAMES) else str(op),
            "key": CFG_KEYS[key] if key < len(CFG_KEYS) else str(key),
            "tag": tag,
            "status": CFG_STATUS[st] if st < len(CFG_STATUS) else str(st),
            "value": value, "raw": bytes(data[4:8])}


def decode_cmd_result(data):
    t, ctr, res, detail = data[:4]
    return {"type": TYPE_NAMES.get(t, hex(t)), "ctr": ctr,
            "result": RESULT_NAMES[res] if res < len(RESULT_NAMES) else str(res),
            "detail": detail}


def decode_fault(data):
    code, flags, duty, t_ms = struct.unpack("<BBhI", bytes(data[:8]))
    return {"code": FAULT_NAMES.get(code, str(code)), "flags": flags,
            "duty": duty, "t_ms": t_ms}


def decode_status_drive(data):
    ctr, flags, speed, cur, out = struct.unpack("<BBhHh", bytes(data[:8]))
    return {"ctr": ctr, "flags": flags, "rpm": speed / 100.0, "ma": cur,
            "out": out}


def decode_status_steer(data):
    ctr, flags, target, pos = struct.unpack("<BBhh", bytes(data[:6]))
    return {"ctr": ctr, "flags": flags, "target": target / 100.0,
            "pos": pos / 100.0}


def flags_str(flags, names=None):
    names = FLAG_NAMES if names is None else names
    return ",".join(n for i, n in enumerate(names) if flags & (1 << i)) or "-"


def steer_flags_str(flags):
    return flags_str(flags, STEER_FLAG_NAMES)


if __name__ == "__main__":
    # Self-test: the check value of §5.2 and a CRC that the node agrees with.
    assert crc8_sae_j1850(b"123456789") == 0x4B
    print("crc8 sae-j1850 '123456789' = 0x%02X PASS" % crc8_sae_j1850(b"123456789"))

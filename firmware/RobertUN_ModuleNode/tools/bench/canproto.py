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

ARM_ACTIONS = {
    "disarm": 0, "arm": 1, "clearfault": 2, "clearestop": 3,
    "steeron": 4, "steeroff": 5,
}

STOP_MODES = {"ramp": 0, "coast": 1, "brake": 2}
STOP_MKS = 0x80


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


def stop_frame(addr, ctr, mode):
    return can_id(T_STOP, addr), bytes([ctr & 0xFF, mode])


def estop_frame(addr=0):
    return can_id(T_ESTOP, addr), b""


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


def flags_str(flags):
    return ",".join(n for i, n in enumerate(FLAG_NAMES) if flags & (1 << i)) or "-"


if __name__ == "__main__":
    # Self-test: the check value of §5.2 and a CRC that the node agrees with.
    assert crc8_sae_j1850(b"123456789") == 0x4B
    print("crc8 sae-j1850 '123456789' = 0x%02X PASS" % crc8_sae_j1850(b"123456789"))

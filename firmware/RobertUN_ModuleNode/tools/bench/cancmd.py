#!/usr/bin/env python3
"""Send W6 CAN commands to a wheel node and print the replies, decoded.

    cancmd.py estop                       broadcast ESTOP (0x000)
    cancmd.py stop 2 coast [--mks]        STOP mode ramp|coast|brake
    cancmd.py arm 2 arm                   ARM disarm|arm|clearfault|clearestop|...
    cancmd.py arm 2 arm --bad-crc         same frame, CRC byte flipped
    cancmd.py speed 2 10 --duration 5     SPEED 10 rpm at 50 Hz, then listen
    cancmd.py speed 2 10 --repeat         ctr frozen after the first frame
    cancmd.py watch 2 --duration 3        STATUS_DRIVE / FAULT summary only

Counters (§5.1) are kept per (type, addr) in runs/.cancmd_ctr.json, so
successive invocations continue the sequence the node expects. Uses the
kernel's SocketCAN directly (no python-can); bring can0 up first, see
_REF_TASK6_CAN_LATENCY. Prints numbers only, never a frame dump.
"""

import argparse
import collections
import json
import os
import select
import socket
import statistics
import struct
import sys
import time

import canproto as cp

HERE = os.path.dirname(os.path.abspath(__file__))
CTR_FILE = os.path.join(HERE, "runs", ".cancmd_ctr.json")
CAN_FMT = "=IB3x8s"


# --- counters ---------------------------------------------------------------

def _load_ctrs():
    try:
        with open(CTR_FILE) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def next_ctr(ftype, addr, forced=None):
    ctrs = _load_ctrs()
    key = "%s:%d" % (cp.TYPE_NAMES[ftype], addr)
    ctr = forced if forced is not None else (ctrs.get(key, -1) + 1) & 0xFF
    ctrs[key] = ctr
    os.makedirs(os.path.dirname(CTR_FILE), exist_ok=True)
    with open(CTR_FILE, "w") as f:
        json.dump(ctrs, f)
    return ctr


def save_ctr(ftype, addr, ctr):
    next_ctr(ftype, addr, forced=ctr)


# --- socket -----------------------------------------------------------------

def open_bus(iface):
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((iface,))
    s.setblocking(False)
    return s


def tx(s, cid, data):
    data = bytes(data)
    s.send(struct.pack(CAN_FMT, cid, len(data), data.ljust(8, b"\0")))


def rx_until(s, deadline):
    """Yield (t, id, data) for every frame received before `deadline`."""
    while True:
        left = deadline - time.monotonic()
        if left <= 0:
            return
        r, _, _ = select.select([s], [], [], left)
        if not r:
            return
        while True:
            try:
                raw = s.recv(16)
            except BlockingIOError:
                break
            cid, dlc, data = struct.unpack(CAN_FMT, raw)
            yield time.monotonic(), cid & 0x7FF, data[:dlc]


# --- collecting replies -----------------------------------------------------

class Listener:
    """Sorts a node's N->O frames; STATUS_DRIVE is summarised, not printed."""

    def __init__(self, node):
        self.node = node
        self.status = []          # (t, decoded)
        self.results = collections.Counter()
        self.result_lines = []
        self.faults = []          # (t, decoded)

    def feed(self, t, cid, data):
        ftype, addr = cp.split_id(cid)
        if addr != self.node:
            return
        if ftype == cp.T_STATUS_DRIVE and len(data) >= 8:
            self.status.append((t, cp.decode_status_drive(data)))
        elif ftype == cp.T_CMD_RESULT and len(data) >= 4:
            r = cp.decode_cmd_result(data)
            self.results[(r["type"], r["result"])] += 1
            if len(self.result_lines) < 6:
                self.result_lines.append(r)
        elif ftype == cp.T_FAULT and len(data) >= 8:
            self.faults.append((t, cp.decode_fault(data)))

    def drain(self, s, deadline):
        for t, cid, data in rx_until(s, deadline):
            self.feed(t, cid, data)

    def print_results(self):
        for r in self.result_lines:
            extra = "  detail %d" % r["detail"] if r["detail"] else ""
            print("CMD_RESULT %-6s ctr %3d -> %s%s"
                  % (r["type"], r["ctr"], r["result"], extra))
        shown = len(self.result_lines)
        total = sum(self.results.values())
        if total > shown:
            print("CMD_RESULT totals: " + ", ".join(
                "%s %s x%d" % (k[0], k[1], n) for k, n in sorted(self.results.items())))

    def print_faults(self, t_ref=None, ref_name=""):
        for t, f in self.faults:
            rel = "  (+%.0f ms after %s)" % ((t - t_ref) * 1000, ref_name) if t_ref else ""
            print("FAULT %-17s flags %-24s duty %+5d  t_ms %d%s"
                  % (f["code"], cp.flags_str(f["flags"]), f["duty"], f["t_ms"], rel))

    def print_status(self, window=None):
        if not self.status:
            print("STATUS_DRIVE: none received")
            return
        ts = [t for t, _ in self.status]
        span = ts[-1] - ts[0]
        rate = (len(ts) - 1) / span if span > 0 else 0.0
        gaps = [b - a for a, b in zip(ts, ts[1:])]
        print("STATUS_DRIVE: %d frames, %.1f Hz, max gap %.1f ms"
              % (len(ts), rate, max(gaps) * 1000 if gaps else 0.0))
        sel = [d for t, d in self.status if window is None or window[0] <= t <= window[1]]
        if sel:
            rpm = [d["rpm"] for d in sel]
            print("  window %d frames: rpm mean %.2f min %.2f max %.2f | out mean %.0f o/oo"
                  " | current mean %.0f mA"
                  % (len(sel), statistics.mean(rpm), min(rpm), max(rpm),
                     statistics.mean(d["out"] for d in sel),
                     statistics.mean(d["ma"] for d in sel)))
        last = self.status[-1][1]
        print("  last: ctr %d flags %s rpm %.2f out %d"
              % (last["ctr"], cp.flags_str(last["flags"]), last["rpm"], last["out"]))


# --- commands ---------------------------------------------------------------

def cmd_estop(s, a):
    cid, data = cp.estop_frame(a.addr)
    lis = Listener(a.node)
    t0 = time.monotonic()
    tx(s, cid, data)
    lis.drain(s, t0 + a.listen)
    lis.print_results()
    lis.print_faults(t0, "ESTOP")


def cmd_stop(s, a):
    mode = cp.STOP_MODES[a.mode] | (cp.STOP_MKS if a.mks else 0)
    ctr = next_ctr(cp.T_STOP, a.addr)
    cid, data = cp.stop_frame(a.addr, ctr, mode)
    lis = Listener(a.node)
    tx(s, cid, data)
    lis.drain(s, time.monotonic() + a.listen)
    lis.print_results()
    lis.print_faults()


def cmd_arm(s, a):
    action = cp.ARM_ACTIONS[a.action]
    ctr = next_ctr(cp.T_ARM, a.addr, a.ctr)
    cid, data = cp.arm_frame(a.addr, ctr, action)
    if a.bad_crc:
        data = data[:7] + bytes([data[7] ^ 0xFF])
    lis = Listener(a.node)
    tx(s, cid, data)
    lis.drain(s, time.monotonic() + a.listen)
    lis.print_results()
    lis.print_faults()
    # An accepted ARM restarts the node's SPEED/STEER windows; restart ours too
    # so the next SPEED begins at 0 rather than somewhere arbitrary.
    if any(k == ("ARM", "OK") for k in lis.results):
        save_ctr(cp.T_SPEED, a.addr, 255)
        save_ctr(cp.T_STEER, a.addr, 255)


def cmd_speed(s, a):
    mrpm = int(round(a.rpm * 1000))
    period = 1.0 / a.rate
    n = max(1, int(round(a.duration * a.rate)))
    lis = Listener(a.node)

    first_ctr = next_ctr(cp.T_SPEED, a.addr, a.ctr)
    ctr = first_ctr
    t_start = time.monotonic()
    t_next = t_start
    t_last = t_start
    sent = 0
    for i in range(n):
        if i > 0 and not a.repeat:
            ctr = (ctr + 1) & 0xFF
        cid, data = cp.speed_frame(a.addr, ctr, mrpm)
        if a.bad_crc:
            data = data[:7] + bytes([data[7] ^ 0xFF])
        tx(s, cid, data)
        t_last = time.monotonic()
        sent += 1
        t_next += period
        lis.drain(s, t_next)
    t_end = time.monotonic()
    save_ctr(cp.T_SPEED, a.addr, ctr)

    lis.drain(s, t_end + a.tail)

    print("SPEED %.3f rpm: sent %d frames in %.2f s (%.1f Hz), ctr %d..%d%s%s"
          % (a.rpm, sent, t_last - t_start, (sent - 1) / (t_last - t_start) if sent > 1 else 0,
             first_ctr, ctr, "  [ctr frozen]" if a.repeat else "",
             "  [bad crc]" if a.bad_crc else ""))
    # The last ~2 s of sending: the settled part of a run started from rest.
    lis.print_status(window=(max(t_start, t_end - 2.0), t_end))

    # Counter echo: STATUS_DRIVE byte 0 against the last ctr we sent.
    if not a.repeat and lis.status:
        during = [d["ctr"] for t, d in lis.status if t_start + 0.1 < t <= t_end]
        if during:
            lags = []
            for t, d in lis.status:
                if t_start + 0.1 < t <= t_end:
                    k = int((t - t_start) / period)
                    sent_ctr = (first_ctr + min(k, n - 1)) & 0xFF
                    lags.append((sent_ctr - d["ctr"]) & 0xFF)
            print("  ctr echo: lag (frames) max %d, mean %.2f over %d STATUS frames"
                  % (max(lags), statistics.mean(lags), len(lags)))

    lis.print_results()
    ref_t, ref_name = (t_start, "the only accepted SPEED") if a.repeat else (t_last, "last SPEED")
    lis.print_faults(ref_t, ref_name)


def cmd_watch(s, a):
    lis = Listener(a.node)
    lis.drain(s, time.monotonic() + a.duration)
    lis.print_status()
    lis.print_results()
    lis.print_faults()


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("-i", "--iface", default="can0")
    p.add_argument("--listen", type=float, default=0.3,
                   help="seconds to collect replies after a one-shot command")
    sub = p.add_subparsers(dest="cmd", required=True)

    e = sub.add_parser("estop")
    e.add_argument("addr", type=int, nargs="?", default=0)
    e.add_argument("--node", type=int, default=2, help="whose replies to show")

    st = sub.add_parser("stop")
    st.add_argument("addr", type=int)
    st.add_argument("mode", choices=sorted(cp.STOP_MODES))
    st.add_argument("--mks", action="store_true", help="also stop the servo")

    ar = sub.add_parser("arm")
    ar.add_argument("addr", type=int)
    ar.add_argument("action", choices=sorted(cp.ARM_ACTIONS, key=cp.ARM_ACTIONS.get))
    ar.add_argument("--ctr", type=int, help="force this counter value")
    ar.add_argument("--bad-crc", action="store_true")

    sp = sub.add_parser("speed")
    sp.add_argument("addr", type=int)
    sp.add_argument("rpm", type=float)
    sp.add_argument("--rate", type=float, default=50.0)
    sp.add_argument("--duration", type=float, default=5.0)
    sp.add_argument("--tail", type=float, default=1.5,
                    help="seconds to keep listening after the last frame")
    sp.add_argument("--repeat", action="store_true",
                    help="freeze ctr after the first frame (REPEAT test)")
    sp.add_argument("--ctr", type=int, help="first counter value")
    sp.add_argument("--bad-crc", action="store_true")

    w = sub.add_parser("watch")
    w.add_argument("addr", type=int)
    w.add_argument("--duration", type=float, default=3.0)

    a = p.parse_args()
    if not hasattr(a, "node"):
        a.node = a.addr
    s = open_bus(a.iface)
    {"estop": cmd_estop, "stop": cmd_stop, "arm": cmd_arm,
     "speed": cmd_speed, "watch": cmd_watch}[a.cmd](s, a)
    return 0


if __name__ == "__main__":
    sys.exit(main())

"""Loader for the `bench.py run step` runs.

It differs from rigdata.py and stairdata.py in one deliberate way: it does not
recompute the step metrics, it IMPORTS `step_metrics` from bench.py and calls
it. The figure that consumes this module is therefore drawing the tool's own
arithmetic, and if the tool's definition of rise, overshoot or settling ever
changes, the figure changes with it or fails loudly. A figure that documents a
metric must not carry a second implementation of that metric — that is how a
plot and its tool quietly come to disagree.

THE SEGMENT BOUND — the one thing a replay has to get right. `profile_step`
slices `run.velocs[mark:]` while the step segment is still running, BEFORE it
sends `vel stop`. The velocity.csv on disk has no such bound: it runs on
through the stop ramp-down to the end of the file. Feed those extra rows to
`step_metrics` and every ramp figure describes the stop instead of the step —
`ramp_s` comes back as the whole dwell and `slew_rpm_s` as a tenth of its true
value. The segment therefore ends at `t_step + meta.args.dwell`, which is the
bound the runner itself used.

Runs are read from the committed run directories; nothing here is transcribed.
"""

import csv
import json
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_BENCH = os.path.normpath(os.path.join(
    _HERE, "..", "..", "..", "..", "firmware", "projects", "RobertUN_ModuleNode", "tools", "bench"))
if _BENCH not in sys.path:
    sys.path.insert(0, _BENCH)

from bench import OVERSHOOT_WINDOW_S, step_metrics   # noqa: E402
from node import Veloc                               # noqa: E402

RUNS = os.path.join(_BENCH, "runs")

# The three steps taken on the rig on 2026-09-26, cold, loaded, 12 V, all with
# the shipped gains (Kp 3.0, Ki 10.0, Kd 0, ff 12510/30) and vel_slew 4000.
STEPS = [
    ("2026-09-26T09-41-07_step", "0 -> 10 rpm, 3 s dwell"),
    ("2026-09-26T09-42-04_step", "0 -> 10 rpm, 8 s dwell"),
    ("2026-09-26T09-43-20_step", "0 -> 20 rpm, 8 s dwell"),
]

COUNTS_PER_REV = 8403.2
WINDOW_TICKS = 20
QUANT_RPM = 60_000.0 / (COUNTS_PER_REV * WINDOW_TICKS)   # 0.357 rpm/count
SHIPPED_SLEW_RPM_S = 4.0                                 # cfg vel_slew 4000 m rpm/s


class Step:
    """One step run, segmented exactly the way profile_step segmented it."""

    def __init__(self, name, label=""):
        self.name, self.label = name, label
        self.dir = os.path.join(RUNS, name)

        with open(os.path.join(self.dir, "meta.json")) as f:
            self.meta = json.load(f)
        self.args = self.meta.get("args", {})
        self.to_rpm = float(self.args["rpm"])
        self.from_rpm = float(self.args.get("step_from") or 0.0)
        self.dwell = float(self.args["dwell"])

        self.events = []
        with open(os.path.join(self.dir, "events.csv")) as f:
            for r in csv.DictReader(f):
                self.events.append((float(r["host_s"]), r["kind"], r["detail"]))
        self.t_step = next(t for t, k, _ in self.events if k == "step")

        self.rows, self.host = [], []
        with open(os.path.join(self.dir, "velocity.csv")) as f:
            for r in csv.DictReader(f):
                self.host.append(float(r["host_s"]))
                self.rows.append(Veloc(
                    seq=int(r["seq"]), ms=int(r["board_ms"]),
                    sp_mrpm=int(r["sp_mrpm"]), meas_mrpm=int(r["meas_mrpm"]),
                    out=int(r["out"]), ff=int(r["ff"]), p=int(r["p"]),
                    i=int(r["i"]), d=int(r["d"]), flags=int(r["flags"]),
                    host_t=float(r["host_s"])))
        self.host = np.asarray(self.host)

        lo, hi = self.t_step, self.t_step + self.dwell
        self.seg = [v for v, h in zip(self.rows, self.host) if lo <= h <= hi]
        self.m = step_metrics(self.seg, self.from_rpm, self.to_rpm)

    # -- the segment, on a clock that starts at the step command ------------
    def t(self):
        return np.array([(v.ms - self.seg[0].ms) / 1000.0 for v in self.seg])

    def meas(self):
        return np.array([v.meas_rpm for v in self.seg])

    def sp(self):
        return np.array([v.sp_rpm for v in self.seg])

    def out(self):
        return np.array([float(v.out) for v in self.seg])

    def term(self, which):
        return np.array([float(getattr(v, which)) for v in self.seg])

    def ramping(self):
        return np.array([bool(v.flags & 0x20) for v in self.seg])

    def anchor(self):
        """Where the setpoint stopped moving — the instant a step response
        actually begins. Everything the old metric got wrong, it got wrong by
        using 0.0 here."""
        return self.m["anchor_s"]

    def ideal_ramp(self):
        """What vel_slew alone would have commanded: the floor any measured
        rise time sits on, drawn so the reader can see how much of the response
        is the limiter and how much is the loop."""
        a = self.anchor()
        t = self.t()
        y = self.from_rpm + np.sign(self.to_rpm - self.from_rpm) * \
            SHIPPED_SLEW_RPM_S * np.clip(t, 0, a)
        return t, np.clip(y, min(self.from_rpm, self.to_rpm),
                          max(self.from_rpm, self.to_rpm))


def load():
    return [Step(n, lab) for n, lab in STEPS]


def report(steps):
    """The same numbers the figure draws, as text. If these and the panels ever
    disagree, the figure is the thing that is wrong."""
    print(f"step metrics  (overshoot window {OVERSHOOT_WINDOW_S:.0f} s, "
          f"encoder quantum {QUANT_RPM:.3f} rpm)\n")
    hdr = ("run", "step", "ramp", "slew", "lag", "rise", "floor", "lim",
           "over", "sd", ">rip", "settle", "cmd", "sserr", "sat")
    print("  {:<10}{:>9}{:>7}{:>7}{:>7}{:>7}{:>7}{:>5}{:>7}{:>6}{:>6}"
          "{:>8}{:>7}{:>7}{:>6}".format(*hdr))
    for s in steps:
        m = s.m

        def f(k, w=7, p=2):
            v = m.get(k)
            return f"{'-':>{w}}" if v is None else f"{v:>{w}.{p}f}"
        print("  {:<10}{:>9}{}{}{}{}{}{:>5}{}{}{:>6}{}{}{}{:>6}".format(
            s.name[11:19],
            f"{s.from_rpm:.0f}->{s.to_rpm:.0f}",
            f("ramp_s"), f("slew_rpm_s"), f("track_lag_rpm"), f("rise_s"),
            f("rise_slew_floor_s"),
            "yes" if m.get("ramp_limited") else "no",
            f("overshoot_rpm"), f("tail_sd_rpm", 6),
            {True: "yes", False: "NO", None: "-"}[m.get("overshoot_above_ripple")],
            f("settle_s", 8), f("settle_from_command_s"), f("ss_error_rpm"),
            f"{m.get('sat_fraction', 0) * 100:.0f}%"))
    print("\n  ramp/slew/lag  the setpoint limiter: seconds it ran, rpm/s it ran at,"
          "\n                 and how far behind it the loop sat while it ran."
          f"\n  floor/lim      0.8 x ramp_s — the rise {SHIPPED_SLEW_RPM_S:.0f} rpm/s costs before"
          "\n                 the loop does anything. 'lim yes' = rise is that floor, not Kp."
          "\n  over/sd/>rip   peak above target, settled ripple sd, and whether the"
          "\n                 peak clears the settled tail's own worst excursion plus"
          "\n                 one count. 'NO' means it is ripple, not overshoot."
          "\n  settle/cmd     from the ramp's end (the loop) and from the command"
          "\n                 (the operator's wait). Their difference is vel_slew.")

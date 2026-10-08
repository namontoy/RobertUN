#!/usr/bin/env python3
"""
Loader and analysis for the loaded-rig bench runs.

Both loaded-rig figures derive every number they draw from the committed run
directories under tools/bench/runs/. Nothing is transcribed. The free-wheel
figure inlines its arrays because it predates the runs being committed; this
one does not have that excuse, and a transcription error in a plant model is
expensive to find later.

`count` is the measurement everywhere in here. The `mrpm` column is never used:
it is a 100 ms boxcar, so it lags ~50 ms and smooths over 100 ms, and fitting a
time constant to it measures the filter. That trap cost a wrong tau once
already - see the header of plot_plant_12v_tau.py.
"""
import csv
import json
import os

import numpy as np

COUNTS_PER_REV = 8403.2
SYNC_GATE = 14.5          # below this duty the current column is Isup, not Imot

RUNS = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                    "..", "..", "..", "..", "firmware", "projects", "RobertUN_ModuleNode",
                    "tools", "bench", "runs")

# The four runs the 12 V plant model rests on.
FREE_ASC  = "2026-09-25T18-32-39_sweep"    # free wheel, clamped, ascending
FREE_DSC  = "2026-09-25T18-33-58_sweep"    # free wheel, clamped, descending
LOAD_ASC  = "2026-09-25T21-29-48_sweep"    # rig 1047 g, ascending, 30 s dwell
LOAD_DSC  = "2026-09-25T22-00-34_sweep"    # rig 1047 g, descending, entry ramp


class Run:
    """One bench run, as arrays. Speeds are derived, never read off `mrpm`."""

    def __init__(self, name):
        self.name = name
        d = os.path.join(RUNS, name)
        with open(os.path.join(d, "telemetry.csv")) as f:
            r = list(csv.DictReader(f))
        self.ms    = np.array([int(x["board_ms"]) for x in r], float) / 1000.0
        self.count = np.array([int(x["count"]) for x in r], float)
        self.duty  = np.array([int(x["duty_permille"]) for x in r]) / 10.0
        self.ma    = np.array([int(x["ma"]) for x in r], float)
        self.flags = np.array([int(x["flags"]) for x in r])
        # flags bit 0 = the current sample was synchronised to the PWM on-phase,
        # i.e. the reading is motor current. Below ~14.5% duty it never is.
        self.sync  = (self.flags & 1).astype(bool)
        self.host  = np.array([float(x["host_s"]) for x in r])
        with open(os.path.join(d, "meta.json")) as f:
            self.meta = json.load(f)
        with open(os.path.join(d, "sweep.csv")) as f:
            self.sweep = list(csv.DictReader(f))

    @property
    def clean(self):
        """No dropped lines and no sequence gaps - the run is fittable."""
        m = self.meta
        return m["seq_gaps"] == 0 and m["tx_dropped"] in (0, None)

    def settled(self):
        """Per-duty settled speed and current, from the runner's own sweep.csv.

        The runner already discards the leading `settle` fraction of each dwell
        and fits the count slope over the remainder, which is the same estimator
        used here. Re-deriving it would only risk disagreeing with the file the
        run itself published.
        """
        d   = np.array([float(x["duty_pct"]) for x in self.sweep])
        rpm = np.array([float(x["rpm"]) for x in self.sweep])
        ma  = np.array([float(x["mA_mean"]) for x in self.sweep])
        syn = np.array([x["current_is_motor"].strip().lower() in ("1", "true")
                        for x in self.sweep])
        o = np.argsort(np.abs(d))
        return np.abs(d[o]), np.abs(rpm[o]), ma[o], syn[o]

    def velocity(self, at, half=0.30):
        """Speed in rpm at board time `at`, as the count slope over +/-`half` s.

        A 0.6 s window is wide enough to average over the rig's ~2.5 Hz ripple.
        Narrower windows alias it and swing the answer by more than a rpm.
        """
        k = (self.ms >= at - half) & (self.ms <= at + half)
        if k.sum() < 8:
            return np.nan
        return np.polyfit(self.ms[k], self.count[k], 1)[0] * 60.0 / COUNTS_PER_REV


def fit(d, y, lo=None, hi=None):
    """Least-squares line over an optional duty sub-range. -> slope, intercept."""
    m = np.ones_like(d, bool)
    if lo is not None:
        m &= d >= lo
    if hi is not None:
        m &= d <= hi
    return np.polyfit(d[m], y[m], 1)


def r2(d, y, c):
    res = y - np.polyval(c, d)
    return 1 - res @ res / ((y - y.mean()) @ (y - y.mean()))


# ------------------------------------------------------ step-response tau ---
def ensemble(runs, horizon=10.0, grid=0.02, min_dv=0.8):
    """Stack every duty step in `runs` into one normalised step response.

    One 2% step moves the wheel ~1.6 rpm against 0.5-0.7 rpm of periodic rig
    ripple - under 3:1, which is why fitting the steps individually failed
    (many pinned at the solver's bound). The ripple is not phase-locked to the
    steps, so stacking N of them beats it down by sqrt(N) and recovers the SNR.

    The stack is built in DISTANCE rather than velocity. Integrating the count
    is itself a low-pass, so no differentiation noise enters the fit at all:

        excess(t) = [count(t) - count(t0)] - v_pre * t      (in revolutions)

    normalised by the step's own final velocity change, so steps of unequal
    size can be averaged together. Points below breakaway contribute no
    velocity change and are dropped by `min_dv`.
    """
    t = np.arange(0.0, horizon + grid / 2, grid)
    out = []
    for run in runs:
        for e in np.where(np.diff(run.duty) != 0)[0] + 1:
            t0 = run.ms[e]
            pre  = (run.ms > t0 - 2.0) & (run.ms < t0 - 0.1)
            post = (run.ms >= t0) & (run.ms <= t0 + horizon)
            nxt  = (run.ms > t0 + horizon + 12) & (run.ms < t0 + horizon + 19)
            if pre.sum() < 40 or post.sum() < 40 * horizon or nxt.sum() < 200:
                continue
            v_pre = np.polyfit(run.ms[pre], run.count[pre], 1)[0] * 60 / COUNTS_PER_REV
            v_end = np.polyfit(run.ms[nxt], run.count[nxt], 1)[0] * 60 / COUNTS_PER_REV
            dv = v_end - v_pre
            if abs(dv) < min_dv:
                continue
            tt = run.ms[post] - t0
            exc = (run.count[post] - run.count[e]) * 60 / COUNTS_PER_REV - v_pre * tt
            out.append(np.interp(t, tt, exc / dv))
    return t, np.array(out)


def step_one_pole(t, tau, c):
    """Excess distance of a first-order velocity step, normalised to dv = 1."""
    return t - tau * (1 - np.exp(-t / tau)) + c


def step_two_pole(t, tau_f, tau_s, w, c):
    """Same, for two parallel poles carrying weights w and 1-w."""
    return (w * (t - tau_f * (1 - np.exp(-t / tau_f)))
            + (1 - w) * (t - tau_s * (1 - np.exp(-t / tau_s))) + c)


def ramp_lag_tau(run, K, C, t_lo, t_hi):
    """Tau from the constant lag with which the wheel tracks a duty ramp.

    A first-order plant driven by a ramp settles to a fixed offset behind its
    quasi-static line: lag = tau * (dDuty/dt) * K. Averaging that offset over
    every sample of the ramp - rather than reading it at one instant - is what
    makes this route survive the rig ripple.

    Independent of the step ensemble: different excitation, and the steps in
    this window are excluded from the stack by their own dwell requirement.
    """
    m = (run.host > t_lo) & (run.host < t_hi)
    rate = np.polyfit(run.host[m], run.duty[m], 1)[0]
    v = np.array([run.velocity(x) for x in run.ms[m]])
    lag = (K * run.duty[m] + C) - v
    tau = lag.mean() / (rate * K)
    se = lag.std(ddof=1) / np.sqrt(len(lag)) / (rate * K)
    return tau, se, rate, lag.mean(), int(m.sum())

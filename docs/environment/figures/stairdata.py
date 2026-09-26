#!/usr/bin/env python3
"""
Loader and analysis for the velocity-loop STAIRCASE runs.

Same discipline as rigdata.py: every number the staircase figure draws comes
out of the committed run directory under tools/bench/runs/. Nothing is
transcribed.

WHAT IS DIFFERENT FROM rigdata.py, AND WHY IT IS ALLOWED TO BE.

rigdata refuses to touch the `mrpm` telemetry column, because the sweep runs
it loads use `enc window 100` and fitting a time constant to a 100 ms boxcar
measures the filter. Two things change here:

  1. The stair profile runs at `enc window 20`, so the boxcar is 20 ms, not
     100 ms. The slowest thing in this figure is the ~260-500 ms speed ripple;
     a 20 ms window attenuates that by under 1%.
  2. Nothing in this figure is a time constant. These are STEADY-STATE points
     from the settled tail of a 60 s hold, plus a ripple period two orders of
     magnitude longer than the filter.

And there is a positive reason to use `meas_mrpm` rather than re-deriving from
counts: it is the number the CONTROLLER acted on. A figure about how well a
loop tracks must use the loop's own measurement, or it is scoring the loop
against an input it never saw. The plant-side column is still there in
telemetry.csv if a future question needs it.

Quantisation at window 20 is 0.36 rpm per count. The within-hold sd is
0.87-1.21 rpm, so the ripple this figure characterises is 2.4-3.4 counts
peak - real signal, comfortably above the floor.
"""
import csv
import json
import os

import numpy as np

COUNTS_PER_REV = 8403.2
WINDOW_TICKS   = 20        # enc window for the stair profile -> 50 Hz, 0.36 rpm/count
QUANT_RPM      = 60_000.0 / (COUNTS_PER_REV * WINDOW_TICKS)
SYNC_GATE_PM   = 145       # below this duty the current column is Isup, not Imot

RUNS = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                    "..", "..", "..", "firmware", "RobertUN_ModuleNode",
                    "tools", "bench", "runs")

# The long run the velocity-loop model rests on, and the warm re-test that
# exists only to separate a warm-up transient from a real speed dependence.
STAIR_LONG = "2026-09-26T09-55-08_stair"   # 21 points x 60 s, 10.0 -> 20.0 rpm
STAIR_WARM = "2026-09-26T10-17-57_stair"   # 2 points x 90 s, re-taken warm

# Shipped feedforward, as flashed: cfg vel_ff_a / vel_ff_b (milli-o/oo per rpm,
# and o/oo). Read back out of the run's own preflight cfg dump by Stair.ff().
SHIPPED_FF_A = 12.510      # o/oo per rpm
SHIPPED_FF_B = 30.0        # o/oo

# The open-loop plant model this loop's feedforward was derived from, measured
# Sep 25 on the same rig with the bridge driven directly - see
# plot_plant_12v_loaded.py. Carried here so the closed-loop re-take can be
# checked against it rather than against a remembered number.
OPEN_LOOP_K, OPEN_LOOP_C = 0.7993, -2.420      # rpm = K * duty% + C


class Stair:
    """One staircase run, as arrays.

    `v_*` are the 50 Hz per-control-step rows from velocity.csv - one per
    decision the loop made. `h_*` are the per-hold settled aggregates the
    runner itself published in stair.csv; re-deriving them here would only
    risk disagreeing with the file the run shipped.
    """

    def __init__(self, name):
        self.name = name
        d = os.path.join(RUNS, name)

        with open(os.path.join(d, "velocity.csv")) as f:
            r = list(csv.DictReader(f))
        col = lambda k, t=float: np.array([t(x[k]) for x in r])
        self.v_host = col("host_s")
        self.v_ms   = col("board_ms") / 1000.0
        self.v_sp   = col("sp_mrpm") / 1000.0
        self.v_rpm  = col("meas_mrpm") / 1000.0
        self.v_out  = col("out")
        self.v_ff   = col("ff")
        self.v_i    = col("i")
        self.v_flag = col("flags", int)

        with open(os.path.join(d, "telemetry.csv")) as f:
            t = list(csv.DictReader(f))
        self.t_host = np.array([float(x["host_s"]) for x in t])
        self.t_ma   = np.array([float(x["ma"]) for x in t])

        with open(os.path.join(d, "stair.csv")) as f:
            s = list(csv.DictReader(f))
        hcol = lambda k: np.array([float(x[k]) for x in s])
        self.h_sp    = hcol("sp_rpm")
        self.h_rpm   = hcol("mean_rpm")
        self.h_sd    = hcol("sd_rpm")
        self.h_err   = hcol("err_rpm")
        self.h_sem   = hcol("sem_rpm")
        self.h_out   = hcol("out_pm")
        self.h_ff    = hcol("ff_pm")
        self.h_i     = hcol("i_pm")
        self.h_sat   = hcol("sat_frac")
        self.h_ma    = hcol("ma_mean")
        self.h_n     = hcol("n").astype(int)

        with open(os.path.join(d, "meta.json")) as f:
            self.meta = json.load(f)
        with open(os.path.join(d, "events.csv")) as f:
            self.events = list(csv.DictReader(f))

        # Hold boundaries come from the runner's own setpoint events, in the
        # same host clock as the samples. stair.csv's t_start_s is relative to
        # the FIRST setpoint, not to host zero; anchoring on the events avoids
        # having to remember that offset, and it is wrong by exactly the amount
        # nobody notices - 1.4 s of a 60 s hold.
        self.sp_at = [(float(e["host_s"]), float(e["detail"]))
                      for e in self.events if e["kind"] == "setpoint"]
        self.settle_s = 10.0      # stair.py's SETTLE_S, excluded from every point

    # ------------------------------------------------------------- integrity --
    @property
    def clean(self):
        """No lost lines, no unpublished control steps, nothing dropped board-side."""
        m = self.meta
        return (m["veloc_gaps"] == 0 and m["veloc_steps_missed"] == 0
                and m["seq_gaps"] == 0 and m["tx_dropped"] == 0)

    def integrity(self):
        m = self.meta
        return dict(v_rows=m["veloc_samples"], t_rows=m["samples"],
                    v_gaps=m["veloc_gaps"], missed=m["veloc_steps_missed"],
                    t_gaps=m["seq_gaps"], dropped=m["tx_dropped"],
                    suspect=m["suspect"], minutes=m["duration_s"] / 60.0)

    # ------------------------------------------------------------- segments ---
    def hold(self, i):
        """(t0, t1) host-clock bounds of hold i, settle fraction already removed.

        The LAST hold does not end at a setpoint event - it ends when the
        runner stops the loop, and the ramp-down that follows is still in the
        velocity stream. Left in, it is a monotonic collapse to zero that
        swamps the ripple autocorrelation completely (the correlation never
        goes negative, so no period is found at all, which is how this was
        noticed). Every hold is therefore bounded by the nominal duration, not
        by the end of the file.
        """
        t0 = self.sp_at[i][0]
        nominal = float(np.median(np.diff([t for t, _ in self.sp_at])))
        t1 = self.sp_at[i + 1][0] if i + 1 < len(self.sp_at) else t0 + nominal
        return t0 + self.settle_s, min(t1, self.v_host[-1])

    def settled(self, i):
        """Boolean mask over the velocity rows for the settled tail of hold i."""
        t0, t1 = self.hold(i)
        return (self.v_host >= t0) & (self.v_host < t1)

    # ------------------------------------------------------------- analysis ---
    def inverse_fit(self):
        """Least squares out[o/oo] = a * rpm + b over the unsaturated holds.

        This is the plant inverse re-measured THROUGH the closed loop: the
        output the loop had to hold to sit at each speed. It is directly
        comparable to the open-loop sweep's forward fit, and comparing them is
        the only independent check either model gets.
        """
        k = self.h_sat < 0.01
        a, b = np.polyfit(self.h_rpm[k], self.h_out[k], 1)
        rms = float(np.sqrt(((self.h_out[k] - (a * self.h_rpm[k] + b)) ** 2).mean()))
        return float(a), float(b), rms, int(k.sum())

    def ripple(self, i, max_lag_ms=1600):
        """Dominant ripple period of hold i, in ms, by autocorrelation.

        Returns (period_ms, events_per_rev, ac_lags_ms, ac) or None.

        THE TRAP THIS AVOIDS. Taking the global maximum of the autocorrelation
        picks a SECOND HARMONIC whenever the fundamental's peak is the shorter
        one - it did exactly that at 13.5, 18.5 and 20.0 rpm, returning doubled
        periods and a tidy-looking 6 events/rev that polluted the spread from
        sd 0.19 to sd 2.09. The fundamental is the FIRST strong local maximum
        after the correlation first goes negative, which is what is taken here.
        """
        m = self.settled(i)
        x = self.v_rpm[m]
        if x.size < 200:
            return None
        dt_ms = float(np.median(np.diff(self.v_ms[m]))) * 1000.0
        d = x - x.mean()
        var = float((d * d).mean())
        if var <= 0:
            return None
        n_lag = int(max_lag_ms / dt_ms)
        ac = np.array([float((d[:-k] * d[k:]).mean()) / var
                       for k in range(1, n_lag + 1)])
        neg = np.argmax(ac < 0) if (ac < 0).any() else None
        if neg is None or neg == 0 and ac[0] >= 0:
            return None
        tail = ac[neg:]
        thresh = 0.6 * tail.max()
        pk = None
        for k in range(neg, len(ac) - 1):
            if ac[k] >= thresh and ac[k] >= ac[k + 1] and ac[k] >= ac[k - 1]:
                pk = k
                break
        if pk is None:
            return None
        period_ms = (pk + 1) * dt_ms
        rev_ms = 60_000.0 / self.h_sp[i]
        return (period_ms, rev_ms / period_ms,
                (np.arange(1, n_lag + 1)) * dt_ms, ac)

    def ripple_table(self):
        """(sp, sd, period_ms, events_per_rev) for every hold that yields one."""
        out = []
        for i in range(len(self.h_sp)):
            r = self.ripple(i)
            if r is not None:
                out.append((self.h_sp[i], self.h_sd[i], r[0], r[1]))
        return np.array(out)

    def drift(self, i):
        """(d_rpm, d_out) second settled half minus first - slow wander within a hold."""
        m = self.settled(i)
        rpm, out = self.v_rpm[m], self.v_out[m]
        h = rpm.size // 2
        return float(rpm[h:].mean() - rpm[:h].mean()), float(out[h:].mean() - out[:h].mean())

    def ff(self):
        """Shipped feedforward (a, b) as actually read back in this run's preflight."""
        cfg = self.meta["conditions_at_connect"]["cfg"]
        a = b = None
        for line in cfg.splitlines():
            p = line.replace("*", "").split()
            if len(p) >= 2 and p[0] == "vel_ff_a":
                a = int(p[1]) / 1000.0
            if len(p) >= 2 and p[0] == "vel_ff_b":
                b = float(p[1])
        return (a if a is not None else SHIPPED_FF_A,
                b if b is not None else SHIPPED_FF_B)


def open_loop_inverse():
    """The Sep 25 open-loop model, inverted into o/oo per rpm for comparison."""
    a = 10.0 / OPEN_LOOP_K                     # o/oo per rpm
    b = -10.0 * OPEN_LOOP_C / OPEN_LOOP_K      # o/oo
    return a, b

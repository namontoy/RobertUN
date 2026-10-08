#!/usr/bin/env python3
"""
The velocity loop over 21 continuous minutes - a 10 -> 20 rpm staircase.

WHY THIS FIGURE EXISTS. The PID gains were shipped at roughly a quarter of
textbook, deliberately derated because the plant fit behind them was flagged
for re-take, and until this run they had never been tested on a moving wheel
for longer than a few seconds. A short step response would have said whether
the loop is stable. It would not have said whether it is ACCURATE, whether it
drifts, or whether the derating costs anything - and those are the questions
that decide whether these gains ship to the rover.

So: 21 setpoints, 0.5 rpm apart, 60 s each, no interruption. What it settles:

  1. TRACKING. Mean error +0.001 rpm, worst 0.015 rpm, against a per-point
     standard error of ~0.02 rpm. The loop is accurate to below the noise
     floor of the instrument measuring it, at every speed.

  2. THE 30% CEILING IS NOT REACHED. 20 rpm holds at 284 of the 300 o/oo
     limit. An earlier short run reported 6% saturation at 20 rpm; that was
     the ACCELERATION TRANSIENT, not the operating point, and this run
     corrects it. Panel B.

  3. THE DERATED GAINS ARE NOT COSTING ACCURACY. The feedforward is doing
     essentially all the work - the integrator contributes a mean of +1.5 o/oo
     out of ~220 - because the plant model behind it turns out to be good. The
     closed-loop re-take agrees with the Sep 25 open-loop sweep to 0.4%, from
     different excitation and a different estimator. The "~4.5% optimistic"
     caveat carried on that model does not hold in this band.

  4. THE RIPPLE IS MECHANICAL, AND THE FIGURE PROVES IT RATHER THAN ASSERTING
     IT. 11.91 +/- 0.19 events per output revolution, held across a 2:1 speed
     range. A control limit cycle holds a fixed PERIOD; this holds a fixed
     COUNT PER REVOLUTION, which only a rotating feature can do. Panel C.
     Identifying which feature is a mechanical job, not a telemetry one.

  5. THE CURRENT U-SHAPE IS REAL BUT IS PROBABLY NOT PHYSICS. Panel D, and
     the caveat in it, is the one number in this figure a reader should not
     trust at the low end.

    python3 plot_velocity_loop_stair.py
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import stairdata as D

OUT = "velocity_loop_stair_21min.png"

S1, S2, S3 = "#2a78d6", "#eb6834", "#1baf7a"
SURFACE, INK, INK_2 = "#fcfcfb", "#0b0b0b", "#52514e"
MUTED, GRID, BASELINE = "#898781", "#e1e0d9", "#c3c2b7"
WARNING, CRITICAL = "#fab219", "#d03b3b"

# ------------------------------------------------------------------- data ---
run  = D.Stair(D.STAIR_LONG)
warm = D.Stair(D.STAIR_WARM)
ig   = run.integrity()
assert run.clean, "the long run must be integrity-clean before anything is fitted"

t0     = run.sp_at[0][0]
t_min  = (run.v_host - t0) / 60.0
hold_t = np.array([(h - t0) / 60.0 for h, _ in run.sp_at])

FF_A, FF_B = run.ff()
A, B, RMS, NPT = run.inverse_fit()          # out[o/oo] = A * rpm + B
OA, OB = D.open_loop_inverse()              # the Sep 25 sweep, inverted
AGREE = 100 * abs(A - OA) / OA
FF_ERR_15 = (FF_A * 15 + FF_B) - (A * 15 + B)

LIMIT_PM = 300.0                            # cfg duty_limit, the 30% ceiling
OUT_MAX  = run.h_out.max()

rip = run.ripple_table()                    # sp, sd, period_ms, events/rev
EVR, EVR_SD = rip[:, 3].mean(), rip[:, 3].std()
PER_MEAN = rip[:, 2].mean()

drifts = np.array([run.drift(i) for i in range(len(run.h_sp))])

# ---------------------------------------------------------------- styling ---
plt.rcParams.update({
    "font.family": ["DejaVu Sans"], "font.size": 9,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
    "axes.edgecolor": BASELINE, "axes.linewidth": 0.9,
    "axes.labelcolor": INK_2, "axes.titlecolor": INK,
    "xtick.color": MUTED, "ytick.color": MUTED,
    "xtick.labelcolor": INK_2, "ytick.labelcolor": INK_2,
    "axes.spines.top": False, "axes.spines.right": False,
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE,
    "legend.frameon": False,
})
RING = dict(mec=SURFACE, mew=1.6)
BOX = dict(boxstyle="round,pad=0.45", fc="#f4f4f1", ec=GRID, lw=0.9)

fig = plt.figure(figsize=(13.2, 8.6))
gs = fig.add_gridspec(2, 3, height_ratios=[1.15, 1], hspace=0.40, wspace=0.27,
                      left=0.055, right=0.985, top=0.90, bottom=0.115)
axA = fig.add_subplot(gs[0, :])
axB, axC, axD = (fig.add_subplot(gs[1, i]) for i in range(3))

# ===== Panel A: the whole 21 minutes ========================================
axA.plot(t_min, run.v_rpm, "-", color=S1, lw=0.30, alpha=0.45, zorder=2,
         label=f"measured, every control step ({ig['v_rows']:,} rows at 50 Hz)")
axA.step(np.append(hold_t, hold_t[-1] + 1.0),
         np.append(run.h_sp, run.h_sp[-1]), where="post",
         color=S2, lw=1.7, zorder=5, label="commanded setpoint")
axA.plot(hold_t + 0.58, run.h_rpm, "o", color=INK, ms=4.0, zorder=6, **RING,
         label="settled mean of each hold")
axA.set_xlabel("time from the first setpoint  [min]")
axA.set_ylabel("wheel speed  [rpm]")
axA.set_title("The ripple is wider than the steps, and the loop resolves them anyway",
              fontsize=11.5, fontweight="bold", pad=9)
axA.set_xlim(-0.4, 21.6)
axA.set_ylim(5.4, 27.0)
axA.set_yticks([7.5, 10.0, 12.5, 15.0, 17.5, 20.0, 22.5])
axA.legend(loc="upper left", fontsize=8.3, labelcolor=INK_2,
           handletextpad=0.7, labelspacing=0.55)

axA.text(7.15, 26.5,
         f"Each step is 0.5 rpm. The band around it is +/-{run.h_sd.mean():.1f} rpm of MECHANICAL\n"
         f"ripple (panel C) - two to three times the step itself. The staircase is legible\n"
         f"only because a 60 s hold averages that away, and it averages away cleanly:\n"
         f"worst settled error {np.abs(run.h_err).max():.3f} rpm, and drift within a hold of "
         f"{drifts[:,0].mean():+.4f} rpm\nand {drifts[:,1].mean():+.2f} o/oo. "
         f"Nothing warms up, sags, or walks over the 21 minutes.",
         fontsize=8.3, color=INK_2, va="top", ha="left", bbox=BOX)

# inset: the tracking error itself, against the instrument's own noise floor
axE = axA.inset_axes([0.735, 0.115, 0.245, 0.30], zorder=8)
axE.axhspan(-D.QUANT_RPM / 2, D.QUANT_RPM / 2, color=MUTED, alpha=0.16, zorder=1)
axE.errorbar(run.h_sp, run.h_err, yerr=run.h_sem, fmt="o", color=S1, ms=3.4,
             elinewidth=1.3, capsize=2, zorder=4)
axE.axhline(0, color=BASELINE, lw=1, zorder=2)
axE.set_title("tracking error  [rpm]   (band = +/-1/2 encoder count)",
              fontsize=7.0, pad=3, color=INK_2)
axE.set_xlabel("setpoint [rpm]", fontsize=7, labelpad=1)
axE.tick_params(labelsize=6.5)
axE.set_ylim(-0.22, 0.22)
axE.set_facecolor(SURFACE); axE.patch.set_alpha(1.0)

# ===== Panel B: the plant inverse, re-measured through the closed loop ======
xs = np.linspace(9.4, 20.6, 50)
axB.axhspan(LIMIT_PM, 340, color=CRITICAL, alpha=0.10, zorder=0)
axB.axhline(LIMIT_PM, color=CRITICAL, lw=1.4, zorder=3)
axB.text(9.7, LIMIT_PM + 4.0, "duty_limit 300 o/oo - the 30% ceiling",
         fontsize=7.4, color=CRITICAL, va="bottom", ha="left")
axB.plot(xs, FF_A * xs + FF_B, "--", color=S3, lw=2.4, zorder=4)
axB.plot(xs, A * xs + B, "-", color=S2, lw=1.5, zorder=5)
axB.plot(run.h_rpm, run.h_out, "o", color=S1, ms=5.5, zorder=6, **RING)
axB.annotate("dashed = the shipped feedforward.\nIt hides under the fit, which is\n"
             "the whole finding: the model is right.",
             xy=(12.6, FF_A * 12.6 + FF_B), xytext=(9.8, 291),
             fontsize=7.3, color=S3, ha="left", va="top",
             arrowprops=dict(arrowstyle="-", color=S3, lw=1.0, shrinkA=3, shrinkB=4))
axB.annotate(f"{OUT_MAX:.0f} o/oo at 20 rpm -\n{LIMIT_PM - OUT_MAX:.0f} o/oo still in hand",
             xy=(20.0, OUT_MAX), xytext=(20.3, 208), fontsize=7.3, color=INK_2,
             ha="right", va="top",
             arrowprops=dict(arrowstyle="-", color=MUTED, lw=1.0, shrinkA=2, shrinkB=5))
axB.text(9.6, 110,
         f"closed loop   {A:.3f} x rpm + {B:.1f}   (rms {RMS:.2f} o/oo, n={NPT})\n"
         f"Sep 25 open-loop sweep, inverted:  {OA:.3f} x rpm + {OB:.1f}\n"
         f"-> {AGREE:.1f}% apart; the integrator carries only "
         f"{run.h_i.mean():+.1f} of the {run.h_out.mean():.0f}",
         fontsize=7.3, color=INK_2, ha="left", va="bottom", bbox=BOX)
axB.set_xlabel("achieved speed  [rpm]")
axB.set_ylabel("commanded output  [o/oo]")
axB.set_title("No saturation anywhere", fontsize=10, fontweight="bold", pad=8)
axB.set_xlim(9.4, 20.6); axB.set_ylim(103, 340)

# ===== Panel C: the ripple is mechanical ====================================
sp = rip[:, 0]
axC.plot(sp, 60_000.0 / (12.0 * sp), "-", color=S2, lw=1.8, zorder=3,
         label="12 events per output revolution")
axC.plot(sp, rip[:, 2], "o", color=S1, ms=6.0, zorder=6, **RING,
         label="measured ripple period")
axC.axhline(PER_MEAN, ls="--", color=S3, lw=1.8, zorder=3)
axC.text(20.4, PER_MEAN + 26, "a control limit cycle would lie here:\n"
         "one fixed period, indifferent to speed",
         fontsize=7.3, color=S3, ha="right", va="bottom")
axC.text(9.7, 300,
         f"{EVR:.2f} +/- {EVR_SD:.2f} events per revolution,\n"
         f"held over a 2:1 speed range. Only\n"
         f"something ROTATING does that.\n"
         f"Which feature - final-stage mesh? an\n"
         f"output-shaft defect? - needs the\n"
         f"gearbox opened, not more telemetry.",
         fontsize=7.4, color=INK_2, ha="left", va="top", bbox=BOX)
axC.set_xlabel("setpoint speed  [rpm]")
axC.set_ylabel("dominant ripple period  [ms]")
axC.set_title("The ripple is mechanical, not a limit cycle",
              fontsize=10, fontweight="bold", pad=8)
axC.set_xlim(9.4, 20.6); axC.set_ylim(185, 580)
axC.legend(loc="upper right", fontsize=7.5, labelcolor=INK_2, handletextpad=0.6)

# ===== Panel D: the current U-shape, and why not to trust its left half =====
axD.axvspan(130, 180, color=WARNING, alpha=0.14, zorder=0)
axD.axvline(D.SYNC_GATE_PM, color=WARNING, lw=1.5, zorder=2)
axD.text(139.5, 212, f"{D.SYNC_GATE_PM} o/oo - synchronised current-sense floor",
         fontsize=7.2, color="#9a6a04", ha="left", va="bottom", rotation=90)
axD.plot(run.h_out, run.h_ma, "o-", color=S1, lw=1.3, ms=5.5, zorder=5, **RING,
         label="long run, 21 holds")
axD.plot(warm.h_out, warm.h_ma, "s", color=S2, ms=9.0, zorder=6,
         markerfacecolor=SURFACE, markeredgecolor=S2, markeredgewidth=2.0,
         label="the same two, re-taken 20 min later, warm")
axD.text(172, 418,
         f"Warm {warm.h_ma[0]:.0f} / {warm.h_ma[1]:.0f} mA against cold "
         f"{run.h_ma[0]:.0f} / {run.h_ma[1]:.0f} mA, so the\n"
         f"low-speed hump is NOT a warm-up transient.\n\n"
         f"But every elevated point sits within ~25% of\n"
         f"the sense floor and decays smoothly as duty\n"
         f"leaves it - a MEASUREMENT artifact's signature.\n"
         f"The rise at {run.h_out[-1]:.0f} o/oo is far from the floor and\n"
         f"is probably real viscous loss.\n\n"
         f"Below ~180 o/oo do not fit this column. Settling\n"
         f"it needs an ammeter, not more telemetry.",
         fontsize=7.0, color=INK_2, ha="left", va="top", bbox=BOX)
axD.set_xlabel("commanded output  [o/oo]")
axD.set_ylabel("mean supply current  [mA]")
axD.set_title("Current: real, but not trustworthy at the low end",
              fontsize=10, fontweight="bold", pad=8)
axD.set_xlim(132, 314); axD.set_ylim(206, 436)
axD.legend(loc="lower right", fontsize=7.3, labelcolor=INK_2, handletextpad=0.6)

fig.text(0.5, 0.955,
         "The velocity loop over 21 continuous minutes - 10 to 20 rpm in 0.5 rpm steps",
         ha="center", va="center", fontsize=13.5, fontweight="bold", color=INK)
fig.text(0.5, 0.034,
         f"VM ~12 V  |  treadmill-belt rig, 1047 g normal load  |  "
         f"Kp 3.0  Ki 10.0  Kd 0  ff {FF_A:.3f} x rpm + {FF_B:.0f} o/oo  "
         f"ilim 150  slew 4 rpm/s  |  enc window {D.WINDOW_TICKS} -> 50 Hz, "
         f"{D.QUANT_RPM:.2f} rpm per count  |  duty_limit 300 o/oo enforced throughout\n"
         f"run {D.STAIR_LONG} ({ig['minutes']:.1f} min) and {D.STAIR_WARM} "
         f"(warm re-take)  |  {ig['v_rows']:,} control steps, {ig['t_rows']:,} "
         f"telemetry rows, {ig['v_gaps']} lost lines, {ig['missed']} unpublished "
         f"steps, {ig['dropped']} bytes dropped board-side",
         ha="center", va="center", fontsize=7.6, color=MUTED, linespacing=1.6)

fig.savefig(OUT, dpi=150, facecolor=SURFACE)
print(f"wrote {OUT}\n")

# ------------------------------------------------------------ console report --
print(f"{'integrity, long run':34s} V {ig['v_rows']:,} rows / T {ig['t_rows']:,} rows"
      f"   gaps {ig['v_gaps']}/{ig['t_gaps']}   unpublished steps {ig['missed']}"
      f"   tx_dropped {ig['dropped']}")
print(f"{'tracking error':34s} mean {run.h_err.mean():+.4f} rpm, "
      f"worst {np.abs(run.h_err).max():.3f} rpm, sd {run.h_err.std():.3f} rpm"
      f"   (per-point sem ~{run.h_sem.mean():.3f})")
print(f"{'saturation':34s} {run.h_sat.max()*100:.0f}% at every one of "
      f"{len(run.h_sp)} holds; peak output {OUT_MAX:.0f} of {LIMIT_PM:.0f} o/oo")
print()
print(f"{'plant inverse, closed loop':34s} {A:.3f} x rpm + {B:.2f} o/oo"
      f"   (rms {RMS:.2f} o/oo over {NPT} points)")
print(f"{'   as forward':34s} rpm = {10/A:.4f} x duty% {-B/A:+.3f}")
print(f"{'Sep 25 open loop, inverted':34s} {OA:.3f} x rpm + {OB:.2f} o/oo"
      f"   -> {AGREE:.2f}% apart")
print(f"{'shipped ff error at 15 rpm':34s} {FF_ERR_15:+.1f} o/oo "
      f"({100*FF_ERR_15/(A*15+B):+.1f}%)")
print(f"{'integrator at rest':34s} mean {run.h_i.mean():+.2f} o/oo, "
      f"range {run.h_i.min():+.1f} .. {run.h_i.max():+.1f}"
      f"   (of ~{run.h_out.mean():.0f} o/oo commanded)")
print()
print(f"{'ripple, events per revolution':34s} {EVR:.2f} +/- {EVR_SD:.2f}"
      f"   range {rip[:,3].min():.1f} .. {rip[:,3].max():.1f} over {len(rip)} holds")
print(f"{'ripple period':34s} {rip[0,2]:.0f} ms at {rip[0,0]:.1f} rpm "
      f"-> {rip[-1,2]:.0f} ms at {rip[-1,0]:.1f} rpm  (speed-locked)")
print(f"{'ripple amplitude':34s} sd {run.h_sd.min():.2f} -> {run.h_sd.max():.2f} rpm"
      f"   ({run.h_sd.min()/D.QUANT_RPM:.1f} - {run.h_sd.max()/D.QUANT_RPM:.1f} encoder counts)")
print(f"{'drift within a 60 s hold':34s} {drifts[:,0].mean():+.4f} rpm, "
      f"{drifts[:,1].mean():+.2f} o/oo   (mean over {len(drifts)} holds)")
print()
print(f"{'current, cold vs warm at 10.0':34s} {run.h_ma[0]:.0f} -> {warm.h_ma[0]:.0f} mA"
      f"   ({warm.h_ma[0]-run.h_ma[0]:+.0f} mA)")
print(f"{'current, cold vs warm at 10.5':34s} {run.h_ma[1]:.0f} -> {warm.h_ma[1]:.0f} mA"
      f"   ({warm.h_ma[1]-run.h_ma[1]:+.0f} mA)")
print("   -> not a warm-up transient. Every elevated point is within 25% of the")
print("      145 o/oo sense floor; below ~180 o/oo this column is not fittable.")

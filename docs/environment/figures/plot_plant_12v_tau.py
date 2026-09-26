#!/usr/bin/env python3
"""
Mechanical time constant of the LOADED wheel - two independent measurements.

WHY THIS FIGURE EXISTS. Tau is the number PID gains are derived from, and it
was got wrong twice before it was got right. The three mistakes are worth
recording, because each is a trap the next measurement can fall into:

  1. The ramp rate was computed over the wrong interval - the 1 s breakaway
     hold at 12% was included in the denominator, giving 3.75 %/s instead of
     the true 4.81 %/s read off the duty column.

  2. The ramp-end speed was read off the `mrpm` telemetry column. That column
     is a 100 ms boxcar; against this rig's ~2.5 Hz ripple it does not even
     lag cleanly, it swings +/-1.4 rpm around the count-derived speed. Every
     speed in this figure comes from the COUNT slope.

  3. The real one: a SINGLE exponential was fitted to a system that has TWO
     poles. A one-pole fit does not fail loudly - it returns a plausible
     number that depends entirely on the fit window (panel B). Over 10 s it
     returns 0.72 s, which is roughly the figure originally reported. It is an
     artifact of the window, not a time constant.

WHAT IT ACTUALLY IS. Two poles, cleanly separated:

  tau_fast ~ 0.22 s, 84% of the response - the electromechanical time constant,
             J/b of the wheel and rotor against the belt. This is the one PID
             design uses.
  tau_slow ~ 2.8 s,  16% - the belt and contact settling into a new speed. Too
             slow and too small to matter for a velocity loop, but it is why a
             30 s dwell was needed to get repeatable steady-state points.

    python3 plot_plant_12v_tau.py
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from scipy.signal import savgol_filter

import rigdata as R

OUT = "plant_12v_loaded_tau.png"

S1, S2, S3 = "#2a78d6", "#eb6834", "#1baf7a"
SURFACE, INK, INK_2 = "#fcfcfb", "#0b0b0b", "#52514e"
MUTED, GRID, BASELINE = "#898781", "#e1e0d9", "#c3c2b7"
WARNING, CRITICAL = "#fab219", "#d03b3b"

# ------------------------------------------------------------------- data ---
load_a, load_d = R.Run(R.LOAD_ASC), R.Run(R.LOAD_DSC)
K, C = 0.8062, -2.7041          # the descending run's OWN steady-state fit

# ---- route 1: ensemble-averaged step response ------------------------------
# 42 duty steps from both runs, stacked. One 2% step moves the wheel ~1.6 rpm
# against 0.5-0.7 rpm of rig ripple; fitting them individually failed (most
# pinned at the solver bound). Stacking beats the ripple down by sqrt(42).
tg, E = R.ensemble([load_a, load_d], horizon=10.0)
mean = E.mean(0)
p1, cv1 = curve_fit(R.step_one_pole, tg, mean, p0=[0.5, 0], bounds=([.02, -1], [9, 1]))
p2, cv2 = curve_fit(R.step_two_pole, tg, mean, p0=[.25, 4., .8, 0],
                    bounds=([.02, 1., .3, -1], [1., 20., 1., 1]))
se2 = np.sqrt(np.diag(cv2))
rms1 = np.sqrt(((mean - R.step_one_pole(tg, *p1)) ** 2).mean())
rms2 = np.sqrt(((mean - R.step_two_pole(tg, *p2)) ** 2).mean())
TAU_F, TAU_S, W = p2[0], p2[1], p2[2]

# velocity form, for reading: d(excess)/dt IS the normalised step response 0->1
v_meas = np.gradient(mean, tg)
# 0.22 s window, cubic. Wide enough to read through the ripple, narrow enough
# not to pull the initial rise earlier than it happens. The raw derivative is
# plotted underneath it regardless - the smooth is for reading, not evidence.
v_smooth = savgol_filter(v_meas, 11, 3)
v_one  = 1 - np.exp(-tg / p1[0])
v_two  = W * (1 - np.exp(-tg / TAU_F)) + (1 - W) * (1 - np.exp(-tg / TAU_S))

# ---- route 2: ramp-tracking lag --------------------------------------------
# Independent: different excitation (a 12->29% ramp, not 2% steps) and a window
# the step ensemble excludes. A first-order plant tracking a ramp settles to a
# CONSTANT lag = tau * rate * K; averaging it over every ramp sample is what
# makes it survive the ripple that defeated a single-instant reading.
TAU_R, SE_R, RATE, LAG, NR = R.ramp_lag_tau(load_d, K, C, 2.0, 5.70)

# ---- the window trap, for panel B ------------------------------------------
hors = np.arange(0.6, 10.01, 0.2)
tau_win = []
for h in hors:
    k = tg <= h
    q, _ = curve_fit(R.step_one_pole, tg[k], mean[k], p0=[.4, 0],
                     bounds=([.02, -1], [9, 1]))
    tau_win.append(q[0])
tau_win = np.array(tau_win)

# ---- the ramp itself, for panel C ------------------------------------------
rm = (load_d.host > 0.8) & (load_d.host < 12.0)
rt = load_d.host[rm]
rv = np.array([load_d.velocity(x) for x in load_d.ms[rm]])
rq = K * load_d.duty[rm] + C          # where a wheel with no inertia would be
rq[load_d.duty[rm] < 11] = np.nan     # below breakaway the line is meaningless
ramp_ma = load_d.ma[(load_d.host > 0.8) & (load_d.host < 6.0)]

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
LW = 1.8
BOX = dict(boxstyle="round,pad=0.45", fc="#f4f4f1", ec=GRID, lw=0.9)

fig = plt.figure(figsize=(13.2, 8.6))
gs = fig.add_gridspec(2, 3, height_ratios=[1.15, 1], hspace=0.40, wspace=0.27,
                      left=0.055, right=0.985, top=0.90, bottom=0.115)
axA = fig.add_subplot(gs[0, :])
axB, axC, axD = (fig.add_subplot(gs[1, i]) for i in range(3))

# ===== Panel A: the ensemble step response, one pole vs two ==================
axA.plot(tg, v_meas, "-", color=S1, lw=0.7, zorder=3, alpha=0.40)
axA.plot(tg, v_smooth, "-", color=S1, lw=2.6, zorder=4,
         label=f"measured: {len(E)} duty steps, stacked\n"
               f"(thin = raw, bold = 0.22 s smooth)")
axA.plot(tg, v_two, "-", color=S2, lw=1.9, zorder=5,
         label=f"two poles: {TAU_F:.3f} s ({W*100:.0f}%) + {TAU_S:.2f} s "
               f"({(1-W)*100:.0f}%)   rms {rms2*1000:.1f} mrev")
axA.plot(tg, v_one, "--", color=S3, lw=1.9, zorder=3,
         label=f"one pole:  {p1[0]:.3f} s                       "
               f"rms {rms1*1000:.1f} mrev")
axA.axhline(1.0, color=BASELINE, lw=1, zorder=1)
for tau, col, lbl in ((TAU_F, S2, "tau_fast"),):
    axA.axvline(tau, color=col, ls=":", lw=1.2, zorder=2)
    axA.annotate(f"{lbl} = {tau:.3f} s", xy=(tau, 0.10), xytext=(tau + 0.33, 0.075),
                 fontsize=8.4, color=col, va="center", ha="left",
                 arrowprops=dict(arrowstyle="-", color=col, lw=1.0, shrinkA=0, shrinkB=2))

axA.text(1.18, 0.655,
         f"The one-pole fit is not merely worse, it is WRONG\n"
         f"IN KIND: it undershoots for the first {TAU_F*3:.1f} s and\n"
         f"overshoots after, because it is averaging two\n"
         f"processes that differ by {TAU_S/TAU_F:.0f}x. Two poles fit "
         f"{rms1/rms2:.1f}x better\nand each has a physical owner:\n\n"
         f"  {TAU_F:.3f} s ({W*100:.0f}%)  electromechanical, J/b\n"
         f"              -> THIS is the one PID uses\n"
         f"  {TAU_S:.2f}  s ({(1-W)*100:.0f}%)  belt and contact settling",
         fontsize=8.4, color=INK_2, va="top", ha="left", bbox=BOX)

axA.set_xlabel("time since the duty step  [s]")
axA.set_ylabel("normalised speed response   (0 = before, 1 = settled)")
axA.set_title("The loaded wheel has TWO time constants, not one "
              "(42 duty steps, ensemble-averaged)",
              fontsize=11.5, fontweight="bold", pad=9)
axA.set_xlim(-0.1, 6); axA.set_ylim(-0.03, 1.62)
axA.legend(loc="upper right", bbox_to_anchor=(0.998, 0.995), fontsize=8.3,
           labelcolor=INK_2, handletextpad=0.7, labelspacing=0.7)

axR = axA.inset_axes([0.645, 0.085, 0.195, 0.275], zorder=8)
axR.plot(tg, (mean - R.step_one_pole(tg, *p1)) * 1000, "--", color=S3, lw=1.5)
axR.plot(tg, (mean - R.step_two_pole(tg, *p2)) * 1000, "-", color=S2, lw=1.5)
axR.axhline(0, color=BASELINE, lw=1)
axR.set_title("fit residual  [milli-rev]", fontsize=7.2, pad=3, color=INK_2)
axR.tick_params(labelsize=6.5)
axR.set_facecolor(SURFACE); axR.patch.set_alpha(1.0)

# ===== Panel B: why a one-pole fit cannot be trusted =========================
axB.plot(hors, tau_win, "-", color=S3, lw=2.2, zorder=4)
axB.axhline(TAU_F, color=S2, ls="-", lw=1.6, zorder=3)
axB.text(9.7, TAU_F - 0.045, f"true tau_fast {TAU_F:.2f} s", fontsize=7.8,
         color=S2, ha="right", va="top")
axB.plot([10.0], [p1[0]], "o", color=CRITICAL, ms=10, zorder=6, **RING)
axB.annotate(f"0.72 s - close to the\nvalue first reported,\nand an artifact",
             xy=(10.0, p1[0]), xytext=(7.4, 0.47), fontsize=7.8, color=CRITICAL,
             ha="center", va="center",
             arrowprops=dict(arrowstyle="-", color=CRITICAL, lw=1.0,
                             shrinkA=0, shrinkB=6))
axB.text(0.35, 1.00,
         "A one-pole tau is whatever the fit\nwindow says it is: it climbs "
         "monotonically\nwith the horizon and never settles.\n"
         "A genuinely first-order system would\ngive a FLAT line here. That test "
         "is what\nexposed the error.",
         fontsize=7.6, color=INK_2, ha="left", va="top", bbox=BOX)
axB.set_xlabel("fit horizon  [s]")
axB.set_ylabel("tau from a ONE-pole fit  [s]")
axB.set_title("The trap: window-dependent tau", fontsize=10, fontweight="bold", pad=8)
axB.set_xlim(0, 10.6); axB.set_ylim(0.1, 1.04)

# ===== Panel C: the independent route - ramp tracking ========================
axC.plot(rt, rq, "-", color=S2, lw=LW, zorder=3,
         label="quasi-static target  K*duty + C")
axC.plot(rt, rv, "-", color=S1, lw=2.2, zorder=4, label="measured (count slope)")
axC.fill_between(rt, rv, rq, where=~np.isnan(rq), color=S1, alpha=0.13, zorder=2)
axC.axvline(5.73, color=BASELINE, ls="--", lw=1.1, zorder=1)
axC.text(5.60, 1.2, "ramp ends", fontsize=7.6, color=INK_2, ha="right")
axC.text(5.95, 14.6,
         f"The wheel tracks the {RATE:.2f} %/s ramp a\n"
         f"CONSTANT {LAG:.2f} rpm behind (shaded).\n"
         f"lag = tau * rate * K, so\n"
         f"tau = {TAU_R:.3f} +/- {SE_R:.3f} s over {NR} samples.",
         fontsize=7.6, color=INK_2, ha="left", va="top", bbox=BOX)
axC.set_xlabel("time from run start  [s]")
axC.set_ylabel("wheel speed  [rpm]")
axC.set_title("Independent route: the entry ramp", fontsize=10,
              fontweight="bold", pad=8)
axC.set_xlim(0.8, 12); axC.set_ylim(0, 30)
axC.legend(loc="upper left", fontsize=7.6, labelcolor=INK_2, handletextpad=0.6)

# ===== Panel D: the two routes, and the artifacts, side by side ==============
rows = [
    ("ensemble"        ,   TAU_F,  se2[0], S1, True),
    ("ramp lag",           TAU_R,  SE_R,   S2, True),
    ("slow pole",          TAU_S,  se2[1], S3, True),
    ("1-pole 3 s",         0.496,  0.0,    MUTED, False),
    ("1-pole 10 s",        p1[0],  0.0,    MUTED, False),
]
y = np.arange(len(rows))[::-1]
for yy, (lbl, val, err, col, good) in zip(y, rows):
    axD.errorbar(val, yy, xerr=err if err else None, fmt="o", color=col, ms=9,
                 elinewidth=2.2, capsize=4, zorder=5,
                 markerfacecolor=col if good else SURFACE,
                 markeredgecolor=col, markeredgewidth=1.8)
    axD.text(val * 1.18, yy + 0.26, f"{val:.3f} s", fontsize=8, color=col,
             va="center", ha="left")
axD.axvspan(TAU_F - 3 * se2[0], TAU_F + 3 * se2[0], color=S1, alpha=0.13, zorder=0)
axD.set_yticks(y); axD.set_yticklabels([r[0] for r in rows], fontsize=8.2)
axD.set_xscale("log")
axD.set_xlim(0.14, 9.0)
axD.set_ylim(-1.75, len(rows) - 0.35)
axD.set_xlabel("time constant  [s]   (log scale)")
axD.set_title("Two routes agree; the rest are artifacts", fontsize=10,
              fontweight="bold", pad=8)
axD.grid(axis="y", visible=False)
axD.text(0.155, -1.52,
         f"Filled = a measurement.  Hollow = a one-pole fit, shown\n"
         f"only to expose what the fit window does to it.\n"
         f"The two real routes differ by "
         f"{100*abs(TAU_R-TAU_F)/TAU_F:.0f}% on independent data.",
         fontsize=7.5, color=INK_2, ha="left", va="bottom", bbox=BOX)

fig.text(0.5, 0.955,
         "Mechanical time constant of the loaded wheel - two independent measurements",
         ha="center", va="center", fontsize=13.5, fontweight="bold", color=INK)
fig.text(0.5, 0.034,
         "VM ~12 V  |  treadmill-belt rig, 1047 g normal load  |  "
         "every speed derived from the COUNT slope - the mrpm column is a 100 ms "
         "boxcar and is never used here\n"
         "runs 2026-09-25T21-29-48 (ascending) and T22-00-34 (descending, with "
         "the 12->29% entry ramp)  |  0 sequence gaps, 0 tx_dropped",
         ha="center", va="center", fontsize=7.8, color=MUTED, linespacing=1.6)

fig.savefig(OUT, dpi=150, facecolor=SURFACE)
print(f"wrote {OUT}\n")

# ------------------------------------------------------------ console report --
print(f"{'ensemble, two-pole   tau_fast':32s} {TAU_F:.3f} +/- {se2[0]:.3f} s"
      f"   weight {W*100:.0f}%   (n = {len(E)} steps)")
print(f"{'ensemble, two-pole   tau_slow':32s} {TAU_S:.2f}  +/- {se2[1]:.2f}  s"
      f"   weight {(1-W)*100:.0f}%")
print(f"{'ramp-tracking lag    tau':32s} {TAU_R:.3f} +/- {SE_R:.3f} s"
      f"   ({NR} samples, rate {RATE:.3f} %/s, lag {LAG:.3f} rpm)")
print(f"{'agreement between the two':32s} {100*abs(TAU_R-TAU_F)/TAU_F:.1f}%")
print()
print(f"{'one-pole fit, 3 s window':32s} 0.496 s   <- artifact")
print(f"{'one-pole fit, 10 s window':32s} {p1[0]:.3f} s   <- artifact, "
      f"and close to the value first reported")
print(f"{'two-pole vs one-pole fit rms':32s} {rms2*1000:.1f} vs {rms1*1000:.1f} mrev"
      f"   ({rms1/rms2:.1f}x better)")
print()
print(f"{'peak current through the ramp':32s} {ramp_ma.max():.0f} mA against a "
      f"1580 mA trip ({1580/ramp_ma.max():.1f}x headroom)")

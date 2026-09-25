#!/usr/bin/env python3
"""
12 V free-wheel plant characterisation - RobertUN wheel node.

TWO bench sessions are plotted, and they were taken under DIFFERENT MOUNTING:

  Sep 23, 2026 - wheel HAND-HELD, typed by hand at the console.
  Sep 25, 2026 - wheel CLAMPED TO THE TABLE, taken by tools/bench/bench.py.

Everything else is common: VM ~12 V at the driver input, DRV8874 slow decay,
free-spinning wheel on the motor shaft, NO RIG. The mounting change moves the
intercept and leaves the slope alone - that is the point of panel C.

    python3 plot_plant_12v.py
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = "plant_12v_free_wheel.png"

# ---------------------------------------------------------------- palette ---
# dataviz reference palette, light surface. Categorical slots 1-3 ONLY: the
# fourth slot puts yellow beside orange, which fails the all-pairs floors
# (normal-vision dE 13.7 < 15). These series are compared arbitrarily - CW vs
# CCW, ascending vs descending - so all-pairs is the gate that applies, and it
# caps a panel at three hues. Hence the faceting below.
# Validated all-pairs on #fcfcfb: normal-vision min dE 24.0, CVD min dE 10.0.
S1, S2, S3 = "#2a78d6", "#eb6834", "#1baf7a"   # blue, orange, aqua
SURFACE  = "#fcfcfb"
INK      = "#0b0b0b"
INK_2    = "#52514e"
MUTED    = "#898781"
GRID     = "#e1e0d9"
BASELINE = "#c3c2b7"
WARNING  = "#fab219"   # status, reserved - always with a label
CRITICAL = "#d03b3b"   # status, reserved - always with a label

# Slot 3 (aqua) measures 2.74:1 on this surface, under the 3:1 bar. The relief
# rule therefore applies and is honoured: every series is direct-labeled as
# well as legended, so identity never rests on hue alone.

# ------------------------------------------------- Sep 25, 2026: CLAMPED ----
# Free wheel clamped to the table. Taken with tools/bench/bench.py, sweep
# profile, enc window 100, trip 1580 mA, watchdog 2000 ms.
# Integrity on all four runs: 0 sequence gaps, 0 echo mismatches, 0 tx_dropped.
# Current tuples are (mA, synchronised) - False means the reading is supply
# current, not motor current, because duty sits under the 14.5% phase gate.

cw_asc_d   = np.array([5, 8, 11, 14, 17, 20, 23, 26, 29, 32, 35, 39], float)
cw_asc_rpm = np.array([2.073, 5.046, 7.519, 10.170, 12.660, 15.214,
                       17.680, 20.122, 22.607, 25.132, 27.520, 30.990])
cw_asc_ma  = [(246.7, 0), (192.0, 0), (154.4, 0), (152.0, 0), (225.0, 1),
              (263.8, 1), (213.1, 1), (232.6, 1), (227.9, 1), (226.4, 1),
              (229.7, 1), (231.6, 1)]

cw_dsc_d   = np.array([39, 35, 32, 29, 26, 23, 20, 17, 14, 11, 8, 5, 4, 3, 2],
                      float)
cw_dsc_rpm = np.array([31.042, 27.958, 25.475, 23.041, 20.553, 18.106, 15.608,
                       13.199, 10.734, 8.153, 5.740, 3.011, 2.137, 1.352,
                       0.000])
cw_dsc_ma  = [(227.8, 1), (221.5, 1), (215.0, 1), (218.1, 1), (223.5, 1),
              (201.1, 1), (207.6, 1), (213.8, 1), (172.0, 0), (128.2, 0),
              (147.3, 0), (156.0, 0), (227.6, 0), (244.3, 0), (194.5, 0)]

cw_full_d   = np.array([10, 20, 30, 40, 50, 60, 70, 80, 90, 100], float)
cw_full_rpm = np.array([7.256, 15.522, 23.713, 31.922, 40.111, 48.363, 56.558,
                        64.781, 72.925, 80.740])
cw_full_ma  = [(107.3, 0), (199.6, 1), (213.0, 1), (222.5, 1), (230.4, 1),
               (233.7, 1), (243.8, 1), (241.4, 1), (253.3, 1), (239.8, 1)]

# CCW. Recorded as negative duty / negative rpm; plotted as magnitude.
# 5% did not break away in this direction - the wheel never moved.
ccw_d   = np.array([5, 8, 11, 14, 17, 20, 23, 26, 29, 32, 35, 39], float)
ccw_rpm = np.array([0.000, 5.547, 8.298, 10.900, 13.492, 16.053, 18.664,
                    21.302, 23.840, 26.383, 28.991, 32.439])

# ---------------------------------------------- Sep 23, 2026: HAND-HELD ----
hh_full_d   = np.array([20, 40, 60, 80, 100], float)
hh_full_rpm = np.array([15.00, 31.95, 48.38, 64.44, 80.68])

SYNC_GATE = 14.5      # DRIVE_PHASE_MIN_TICKS = 652 of 4500
BAND      = (5, 39)   # the rover's important range

# ------------------------------------------------------------- fitting ------
def fit(d, rpm, lo=None, hi=None):
    """Least-squares line, optionally over a duty sub-range. -> slope, icept."""
    m = np.ones_like(d, bool)
    if lo is not None:
        m &= d >= lo
    if hi is not None:
        m &= d <= hi
    return np.polyfit(d[m], rpm[m], 1)

def r2(d, rpm, c):
    res = rpm - np.polyval(c, d)
    return 1 - res @ res / ((rpm - rpm.mean()) @ (rpm - rpm.mean()))

# The band fits start at 11%: below that the wheel is in the stiction region
# and the line is not the plant any more.
c_asc  = fit(cw_asc_d, cw_asc_rpm, lo=11)
c_dsc  = fit(cw_dsc_d, cw_dsc_rpm, lo=11)
c_ccw  = fit(ccw_d, ccw_rpm, lo=11)
c_full = fit(cw_full_d, cw_full_rpm)
# Like-for-like against Sep 23: the same 20-100% points, clamped vs hand-held.
c_f25  = fit(cw_full_d, cw_full_rpm, lo=20)
c_f23  = fit(hh_full_d, hh_full_rpm)
slope_agreement = 100 * (c_f25[0] - c_f23[0]) / c_f23[0]

# Thermal drift: descending minus ascending, at the duties both passes share.
shared = [d for d in cw_asc_d if d in set(cw_dsc_d)]
drift  = np.array([float(cw_dsc_rpm[cw_dsc_d == d][0] -
                         cw_asc_rpm[cw_asc_d == d][0]) for d in shared])
shared = np.array(shared, float)
# Elapsed time runs backwards along the descending pass, so the x-axis duty is
# also a proxy for "how long ago the ascending twin was taken".

# ------------------------------------------------------------- styling ------
plt.rcParams.update({
    "font.family": ["DejaVu Sans"],
    "font.size": 9,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
    "axes.edgecolor": BASELINE, "axes.linewidth": 0.9,
    "axes.labelcolor": INK_2, "axes.titlecolor": INK,
    "xtick.color": MUTED, "ytick.color": MUTED,
    "xtick.labelcolor": INK_2, "ytick.labelcolor": INK_2,
    "axes.spines.top": False, "axes.spines.right": False,
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE,
    "legend.frameon": False,
})
RING = dict(mec=SURFACE, mew=1.6)      # 2px surface ring on overlapping marks
LW   = 1.8

fig = plt.figure(figsize=(13.2, 8.6))
gs  = fig.add_gridspec(2, 3, height_ratios=[1.15, 1],
                       hspace=0.38, wspace=0.26,
                       left=0.055, right=0.985, top=0.90, bottom=0.115)
axA = fig.add_subplot(gs[0, :])
axB = fig.add_subplot(gs[1, 0])
axC = fig.add_subplot(gs[1, 1])
axD = fig.add_subplot(gs[1, 2])

# ============ Panel A: the 5-39% operating band, three series ================
axA.axvspan(*BAND, color=S1, alpha=0.045, zorder=0)
axA.axvline(SYNC_GATE, color=WARNING, ls="--", lw=1.3, zorder=1)

xb = np.linspace(2, 40, 200)
for c, col in ((c_asc, S1), (c_dsc, S2), (c_ccw, S3)):
    axA.plot(xb, np.polyval(c, xb), "-", color=col, lw=1.0, alpha=0.45,
             zorder=2)

axA.plot(cw_asc_d, cw_asc_rpm, "o", color=S1, ms=7, zorder=5, **RING,
         label=f"CW ascending    rpm = {c_asc[0]:.4f} d - {abs(c_asc[1]):.3f}")
axA.plot(cw_dsc_d[:-1], cw_dsc_rpm[:-1], "s", color=S2, ms=7, zorder=5, **RING,
         label=f"CW descending  rpm = {c_dsc[0]:.4f} d - {abs(c_dsc[1]):.3f}")
axA.plot(ccw_d[1:], ccw_rpm[1:], "^", color=S3, ms=8, zorder=5, **RING,
         label=f"CCW ascending  rpm = {c_ccw[0]:.4f} d - {abs(c_ccw[1]):.3f}")

# the two points where the wheel did not turn - status colour, always labelled
axA.plot([2], [0], "X", color=CRITICAL, ms=11, zorder=6, **RING)
axA.plot([5], [0], "X", color=CRITICAL, ms=11, zorder=6, **RING)
axA.annotate("wheel stationary\n(CW 2%, CCW 5%)", xy=(5.0, -0.15),
             xytext=(10.0, 1.1), fontsize=8, color=CRITICAL,
             ha="left", va="center",
             arrowprops=dict(arrowstyle="-", color=CRITICAL, lw=1.0,
                             shrinkA=0, shrinkB=3))

# direct labels - the relief rule for slot 3, and identity without the legend
for d, r, col, txt, dy in ((39, 30.990, S1, "CW asc", -1.35),
                           (39, 31.042, S2, "CW desc", 1.35),
                           (39, 32.439, S3, "CCW", 1.30)):
    axA.annotate(txt, xy=(d, r), xytext=(d + 0.7, r + dy), fontsize=8.5,
                 color=INK_2, va="center", ha="left")

axA.text(SYNC_GATE + 0.4, -1.35, "14.5% current-sense gate",
         fontsize=7.8, color=INK_2, ha="left", va="bottom")
axA.text(24.5, 0.6,
         f"CCW runs {100 * (c_ccw[0] - c_asc[0]) / c_asc[0]:+.1f}% faster than "
         f"CW for the same duty\n(Aug 26 measured +3.5% by a different method "
         "- independent confirmation).",
         fontsize=8.2, color=INK_2, va="bottom", ha="left",
         bbox=dict(boxstyle="round,pad=0.45", fc="#f4f4f1", ec=GRID, lw=0.9))

axA.set_xlabel("duty command  [%]")
axA.set_ylabel("wheel speed  [rpm]")
axA.set_title("The 5-39% operating band, wheel clamped to the table "
              "(Sep 25, 2026)", fontsize=11.5, fontweight="bold", pad=9)
axA.set_xlim(0, 47); axA.set_ylim(-1.5, 36.5)
axA.legend(loc="upper left", bbox_to_anchor=(0.305, 0.995),
           fontsize=8.3, labelcolor=INK_2, handletextpad=0.6)

# residuals, as an inset - one hue per series, same assignment
axR = axA.inset_axes([0.035, 0.615, 0.225, 0.32], zorder=8)
for d, r, c, col, mk in ((cw_asc_d, cw_asc_rpm, c_asc, S1, "o"),
                         (cw_dsc_d[:-1], cw_dsc_rpm[:-1], c_dsc, S2, "s"),
                         (ccw_d[1:], ccw_rpm[1:], c_ccw, S3, "^")):
    m = d >= 11
    axR.plot(d[m], r[m] - np.polyval(c, d[m]), mk, color=col, ms=4.5,
             mec=SURFACE, mew=1.0)
axR.axhline(0, color=BASELINE, lw=1)
axR.set_title("residual vs own fit, 11-39%  [rpm]", fontsize=7.2, pad=3,
              color=INK_2)
axR.tick_params(labelsize=6.5)
axR.set_ylim(-0.22, 0.22)
axR.set_facecolor(SURFACE)
axR.patch.set_alpha(1.0)

# ============ Panel B: thermal drift, one series =============================
axB.plot(shared, drift, "-o", color=S1, lw=LW, ms=7, **RING, zorder=4)
axB.axhline(0, color=BASELINE, lw=1, zorder=1)
axB.annotate("taken ~11 min apart", xy=(5, drift[0]), xytext=(11.5, 1.00),
             fontsize=8, color=INK_2, ha="left", va="center",
             arrowprops=dict(arrowstyle="-", color=MUTED, lw=1.0,
                             shrinkA=0, shrinkB=4))
axB.annotate("taken back-to-back", xy=(39, drift[-1]), xytext=(31, 0.55),
             fontsize=8, color=INK_2, ha="center", va="center",
             arrowprops=dict(arrowstyle="-", color=MUTED, lw=1.0,
                             shrinkA=0, shrinkB=4))
axB.text(5.0, 0.055,
         "The gap tracks ELAPSED TIME,\nnot direction: the motor warms,\n"
         "friction falls. Not hysteresis.",
         fontsize=7.8, color=INK_2, ha="left", va="bottom",
         bbox=dict(boxstyle="round,pad=0.4", fc="#f4f4f1", ec=GRID, lw=0.9))
axB.set_xlabel("duty command  [%]")
axB.set_ylabel("descending - ascending  [rpm]")
axB.set_title("Warming, not hysteresis", fontsize=10, fontweight="bold", pad=8)
axB.set_xlim(2, 42); axB.set_ylim(-0.02, 1.14)

# ============ Panel C: full-range calibration, mounting comparison ===========
xf = np.linspace(0, 102, 200)
axC.plot(xf, np.polyval(c_f25, xf), "-", color=S1, lw=1.0, alpha=0.45, zorder=2)
axC.plot(cw_full_d, cw_full_rpm, "o", color=S1, ms=7, zorder=5, **RING,
         label=f"Sep 25 clamped, by tool   {c_f25[0]:.4f} d - {abs(c_f25[1]):.3f}")
axC.plot(hh_full_d, hh_full_rpm, "D", color=S2, ms=7, zorder=5, **RING,
         label=f"Sep 23 hand-held, by hand  {c_f23[0]:.4f} d - {abs(c_f23[1]):.3f}")
axC.annotate("Sep 25 clamped", xy=(90, 72.925), xytext=(74, 84.0),
             fontsize=8.5, color=INK_2, ha="center", va="center",
             arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.9,
                             shrinkA=0, shrinkB=4))
axC.annotate("Sep 23 hand-held", xy=(40, 31.95), xytext=(60, 22.0),
             fontsize=8.5, color=INK_2, ha="center", va="center",
             arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.9,
                             shrinkA=0, shrinkB=4))
axC.text(3, 68,
         f"slopes agree to {slope_agreement:+.2f}%\n"
         "over the same 20-100% points.\n\n"
         "Mounting moves the INTERCEPT.\n"
         "The slope belongs to the motor\n"
         "and the rail.",
         fontsize=8, color=INK_2, ha="left", va="top",
         bbox=dict(boxstyle="round,pad=0.4", fc="#f4f4f1", ec=GRID, lw=0.9))
axC.set_xlabel("duty command  [%]")
axC.set_ylabel("wheel speed  [rpm]")
axC.set_title("Tool vs hand, clamped vs held", fontsize=10,
              fontweight="bold", pad=8)
axC.set_xlim(0, 104); axC.set_ylim(0, 90)
axC.legend(loc="lower right", fontsize=7.2, labelcolor=INK_2,
           handletextpad=0.6, borderaxespad=0.4)

# ============ Panel D: current ===============================================
axD.axvspan(0, SYNC_GATE, color=WARNING, alpha=0.10, zorder=0)
allpts = ([(d, m, s) for d, (m, s) in zip(cw_asc_d, cw_asc_ma)] +
          [(d, m, s) for d, (m, s) in zip(cw_dsc_d, cw_dsc_ma)] +
          [(d, m, s) for d, (m, s) in zip(cw_full_d, cw_full_ma)])
sync   = np.array([(d, m) for d, m, s in allpts if s])
nosync = np.array([(d, m) for d, m, s in allpts if not s])

axD.plot(nosync[:, 0], nosync[:, 1], "o", color=S2, ms=6.5, zorder=4, **RING,
         label="supply current (below the gate)")
axD.plot(sync[:, 0], sync[:, 1], "o", color=S1, ms=6.5, zorder=5, **RING,
         label="motor current (synchronised)")

mean_sync = sync[:, 1].mean()
axD.axhline(mean_sync, color=BASELINE, ls=":", lw=1.4, zorder=2)
axD.text(102, mean_sync - 26, f"mean {mean_sync:.0f} mA", ha="right",
         fontsize=8.5, color=INK_2)
axD.annotate("below 14.5% duty this is\nSUPPLY current, not motor",
             xy=(7.2, 95), xytext=(20, 48), fontsize=7.6, color=INK_2,
             ha="left", va="center",
             arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.9,
                             shrinkA=0, shrinkB=4))
axD.set_xlabel("duty command  [%]")
axD.set_ylabel("current  [mA]")
axD.set_title("Flat with duty - friction, not load", fontsize=10,
              fontweight="bold", pad=8)
axD.set_xlim(0, 104); axD.set_ylim(0, 375)
axD.legend(loc="upper center", fontsize=7.2, labelcolor=INK_2,
           handletextpad=0.6, borderaxespad=0.3)

fig.text(0.5, 0.955,
         "12 V free-wheel plant characterisation - RobertUN wheel node",
         ha="center", va="center", fontsize=13.5, fontweight="bold",
         color=INK)
fig.text(0.5, 0.040,
         "VM ~12 V at the driver input  |  DRV8874, slow decay, trip 1580 mA  |"
         "  free-spinning wheel, NO RIG  |  enc window 100  |  "
         "Sep 25 runs taken by tools/bench/bench.py, 0 sequence gaps and "
         "0 tx_dropped on all four",
         ha="center", va="center", fontsize=8, color=MUTED)

fig.savefig(OUT, dpi=150, facecolor=SURFACE)
print(f"wrote {OUT}")

# ---------------------------------------------------------- console report --
for name, c, d, rpm, lo in (("CW ascending  11-39%", c_asc, cw_asc_d, cw_asc_rpm, 11),
                            ("CW descending 11-39%", c_dsc, cw_dsc_d, cw_dsc_rpm, 11),
                            ("CCW ascending 11-39%", c_ccw, ccw_d, ccw_rpm, 11),
                            ("CW full      10-100%", c_full, cw_full_d, cw_full_rpm, 10),
                            ("CW full  20-100% (=Sep23 pts)", c_f25, cw_full_d, cw_full_rpm, 20),
                            ("Sep 23 hand-held 20-100%", c_f23, hh_full_d, hh_full_rpm, 20)):
    m = d >= lo
    res = rpm[m] - np.polyval(c, d[m])
    print(f"{name:32s} rpm = {c[0]:.4f} d {c[1]:+.4f}   "
          f"R2 {r2(d[m], rpm[m], c):.5f}   max|res| {np.abs(res).max():.3f}")
print(f"{'slope agreement clamped vs held':32s} {slope_agreement:+.2f}%")
print(f"{'CCW vs CW slope':32s} {100*(c_ccw[0]-c_asc[0])/c_asc[0]:+.2f}%")
print(f"{'mean synchronised current':32s} {mean_sync:.1f} mA")

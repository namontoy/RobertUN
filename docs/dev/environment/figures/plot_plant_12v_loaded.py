#!/usr/bin/env python3
"""
12 V LOADED-RIG plant characterisation - RobertUN wheel node, Sep 25 2026.

The companion to plot_plant_12v.py. Same board, same driver, same 12 V rail;
the wheel is now pressed against the treadmill-belt rig instead of spinning
free. The rig carries the wheel plus small aluminium parts, 1047 g, which is
dead weight providing normal force - the rig base is static, so that mass is
NOT accelerated and the rig reproduces the rover's friction but not its inertia.

Everything drawn is derived from the committed run directories; see rigdata.py.

    python3 plot_plant_12v_loaded.py
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rigdata as R

OUT = "plant_12v_loaded_rig.png"

# ---------------------------------------------------------------- palette ---
# dataviz reference palette, light surface, categorical slots 1-3 only. The
# fourth slot puts yellow beside orange and fails the all-pairs normal-vision
# floor, and these series are compared arbitrarily, so three hues is the cap.
# Validated all-pairs on #fcfcfb: normal-vision min dE 24.0, CVD min dE 10.0.
S1, S2, S3 = "#2a78d6", "#eb6834", "#1baf7a"   # blue, orange, aqua
SURFACE, INK, INK_2 = "#fcfcfb", "#0b0b0b", "#52514e"
MUTED, GRID, BASELINE = "#898781", "#e1e0d9", "#c3c2b7"
WARNING, CRITICAL = "#fab219", "#d03b3b"       # status, reserved - always labelled

# ------------------------------------------------------------------- data ---
load_a, load_d = R.Run(R.LOAD_ASC), R.Run(R.LOAD_DSC)
free_a = R.Run(R.FREE_ASC)
assert all(r.clean for r in (load_a, load_d, free_a)), "a run has gaps - do not fit it"

la_d, la_rpm, la_ma, la_s = load_a.settled()
ld_d, ld_rpm, ld_ma, ld_s = load_d.settled()
fa_d, fa_rpm, fa_ma, fa_s = free_a.settled()

BAND = (11, 29)     # the fitted band: above breakaway, at or below the ceiling
CEIL = 30           # the rover runs slow; characterisation stops here

c_la = R.fit(la_d, la_rpm, *BAND)
c_ld = R.fit(ld_d, ld_rpm, *BAND)
c_fa = R.fit(fa_d, fa_rpm, *BAND)

# Pooled model - the one that goes into the docs. Both directions together,
# because neither is more right than the other and the spread between them is
# the rig's own repeatability.
pool_d   = np.concatenate([la_d[(la_d >= BAND[0]) & (la_d <= BAND[1])],
                           ld_d[(ld_d >= BAND[0]) & (ld_d <= BAND[1])]])
pool_rpm = np.concatenate([la_rpm[(la_d >= BAND[0]) & (la_d <= BAND[1])],
                           ld_rpm[(ld_d >= BAND[0]) & (ld_d <= BAND[1])]])
c_pool = np.polyfit(pool_d, pool_rpm, 1)

# Ascending vs descending at the duties they share, inside the band.
shared = np.array(sorted(set(la_d[(la_d >= BAND[0]) & (la_d <= BAND[1])])
                         & set(ld_d)))
res_a = np.array([la_rpm[la_d == x][0] - np.polyval(c_pool, x) for x in shared])
res_d = np.array([ld_rpm[ld_d == x][0] - np.polyval(c_pool, x) for x in shared])
corr  = np.corrcoef(res_a, res_d)[0, 1]
gap   = np.array([ld_rpm[ld_d == x][0] - la_rpm[la_d == x][0] for x in shared])

# Local gain. Point-to-point differencing on 2% steps is mostly NOISE: the rig
# repeats to about +/-0.29 rpm, and differencing two such points over a 2% step
# turns that into +/-0.29*sqrt(2)/2 = +/-0.21 rpm/% of apparent gain - most of
# the visible swing. So the point-to-point series is plotted as context only,
# and the real curvature is read off a quadratic fitted to the pooled band.
gl_d   = (la_d[:-1] + la_d[1:]) / 2
gl_g   = np.diff(la_rpm) / np.diff(la_d)
keep   = gl_d >= 13
gl_d, gl_g = gl_d[keep], gl_g[keep]

q_pool   = np.polyfit(pool_d, pool_rpm, 2)
gain_x   = np.linspace(BAND[0], BAND[1], 100)
gain_q   = 2 * q_pool[0] * gain_x + q_pool[1]        # d(rpm)/d(duty) of the quadratic
rms_lin  = np.sqrt(((pool_rpm - np.polyval(c_pool, pool_d)) ** 2).mean())
rms_quad = np.sqrt(((pool_rpm - np.polyval(q_pool, pool_d)) ** 2).mean())
floor      = gap.std(ddof=1)          # spread between the two passes
gain_noise = floor * np.sqrt(2) / 2   # ...as apparent gain, over a 2% step

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
gs = fig.add_gridspec(2, 3, height_ratios=[1.15, 1], hspace=0.38, wspace=0.26,
                      left=0.055, right=0.985, top=0.90, bottom=0.115)
axA = fig.add_subplot(gs[0, :])
axB, axC, axD = (fig.add_subplot(gs[1, i]) for i in range(3))

# ===== Panel A: the loaded operating band, against the free wheel ============
axA.axvspan(*BAND, color=S1, alpha=0.045, zorder=0)
axA.axvspan(CEIL, 44, color=CRITICAL, alpha=0.05, zorder=0)
axA.axvline(R.SYNC_GATE, color=WARNING, ls="--", lw=1.3, zorder=1)

xb = np.linspace(8, 40, 200)
for c, col in ((c_la, S1), (c_ld, S2), (c_fa, S3)):
    axA.plot(xb, np.polyval(c, xb), "-", color=col, lw=1.0, alpha=0.45, zorder=2)

axA.plot(la_d, la_rpm, "o", color=S1, ms=7, zorder=5, **RING,
         label=f"loaded ascending    rpm = {c_la[0]:.4f} d - {abs(c_la[1]):.3f}")
axA.plot(ld_d, ld_rpm, "s", color=S2, ms=7, zorder=5, **RING,
         label=f"loaded descending  rpm = {c_ld[0]:.4f} d - {abs(c_ld[1]):.3f}")
axA.plot(fa_d, fa_rpm, "^", color=S3, ms=8, zorder=4, **RING,
         label=f"free wheel (ref)      rpm = {c_fa[0]:.4f} d - {abs(c_fa[1]):.3f}")

# Breakaway: the loaded wheel does not move at all below 11%.
stuck = la_d[la_rpm == 0]
axA.plot(stuck, np.zeros_like(stuck), "X", color=CRITICAL, ms=11, zorder=6, **RING)
axA.annotate("stationary 5-9%: breakaway is\n9-11% loaded vs 5-6% free",
             xy=(9, 0), xytext=(4.3, 7.4), fontsize=7.8, color=CRITICAL,
             ha="left", va="center",
             arrowprops=dict(arrowstyle="-", color=CRITICAL, lw=1.0,
                             shrinkA=0, shrinkB=4))

# Spread along the curve rather than stacked at one duty - three labels at the
# same x would overlap, and the two loaded series differ by well under a rpm.
for d, ys, txt, off in ((37, la_rpm, "loaded ascending", (1.1, -3.4)),
                        (23, ld_rpm, "loaded descending", (-5.6, 3.0)),
                        (33, fa_rpm, "free wheel", (-1.2, 3.2))):
    src = la_d if ys is la_rpm else (ld_d if ys is ld_rpm else fa_d)
    y = float(np.interp(d, src, ys))
    axA.annotate(txt, xy=(d, y), xytext=(d + off[0], y + off[1]), fontsize=8.3,
                 color=INK_2, va="center", ha="center",
                 arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.9,
                                 shrinkA=2, shrinkB=3))

axA.text(30.5, 1.0,
         "above 30%: the belt surface\nis not homogeneous and the\n"
         "wheel begins to BOUNCE -\nfriction drops, speed rises.\n"
         "A property of the rig, not\nthe plant. Characterisation\nstops at 30%.",
         fontsize=7.5, color=CRITICAL, ha="left", va="bottom", bbox=BOX)
# LEFT of the gate line, not right of it: the pooled-model box's left edge is
# only 4.5 duty% to the right and the label ran straight into it.
axA.text(R.SYNC_GATE - 0.4, -2.0, "14.5% current-sense gate",
         fontsize=7.8, color=INK_2, ha="right", va="bottom")
axA.text(19.3, 8.6,
         f"Pooled model, 11-29%:\n"
         f"   rpm = {c_pool[0]:.4f} x duty% - {abs(c_pool[1]):.3f}\n"
         f"The load costs {100*(1-c_pool[0]/c_fa[0]):.1f}% of the free-wheel slope\n"
         f"and pushes breakaway from 5-6% to 9-11%.\n"
         "Breakaway is a VOLTAGE threshold: 9-11%\n"
         "of 12 V = 1.08-1.32 V - the same bracket the\n"
         "9.35 V rail gave at 12-14% on Sep 15.",
         fontsize=8.0, color=INK_2, va="top", ha="left", bbox=BOX)

axA.set_xlabel("duty command  [%]")
axA.set_ylabel("wheel speed  [rpm]")
axA.set_title("The loaded operating band - wheel on the 1047 g treadmill-belt rig "
              "(Sep 25, 2026)", fontsize=11.5, fontweight="bold", pad=9)
axA.set_xlim(3, 44); axA.set_ylim(-2.2, 34)
axA.legend(loc="upper left", bbox_to_anchor=(0.285, 0.995), fontsize=8.3,
           labelcolor=INK_2, handletextpad=0.6)

axR = axA.inset_axes([0.035, 0.615, 0.225, 0.32], zorder=8)
for d, r, c, col, mk in ((la_d, la_rpm, c_la, S1, "o"),
                         (ld_d, ld_rpm, c_ld, S2, "s"),
                         (fa_d, fa_rpm, c_fa, S3, "^")):
    m = (d >= BAND[0]) & (d <= BAND[1])
    axR.plot(d[m], r[m] - np.polyval(c, d[m]), mk, color=col, ms=4.5,
             mec=SURFACE, mew=1.0)
axR.axhline(0, color=BASELINE, lw=1)
axR.set_title("residual vs own fit, 11-29%  [rpm]", fontsize=7.2, pad=3, color=INK_2)
axR.tick_params(labelsize=6.5); axR.set_ylim(-0.42, 0.42)
axR.set_facecolor(SURFACE); axR.patch.set_alpha(1.0)

# ===== Panel B: Coulomb vs viscous ===========================================
# 15% is the first synchronised point and sits right ON the 14.5% gate, where
# the on-phase only just clears the IPROPI settle window; it reads ~45 mA high
# against both its neighbours. The viscous trend is taken from 17% up, clear
# of the gate. The 15% point is still plotted - hiding it would be worse.
mb = la_s & (la_d <= CEIL)
mt = mb & (la_d >= 17)
mf = fa_s & (fa_d <= CEIL)
axB.plot(la_d[mb], la_ma[mb], "-o", color=S1, lw=LW, ms=7, **RING, zorder=4,
         label="loaded rig")
axB.plot(fa_d[mf], fa_ma[mf], "-^", color=S3, lw=LW, ms=8, **RING, zorder=3,
         label="free wheel")
cb = np.polyfit(la_d[mt], la_ma[mt], 1)          # loaded: viscous slope
cf = np.polyfit(fa_d[mf], fa_ma[mf], 1)          # free: should be ~0
sd_f = (fa_ma[mf] - np.polyval(cf, fa_d[mf])).std(ddof=1)
axB.text(13.4, 396,
         f"Free wheel has NO speed term: slope\n"
         f"{cf[0]:+.1f} mA per duty%, i.e. flat at "
         f"{fa_ma[mf].mean():.0f} +/- {sd_f:.0f} mA.\n"
         "That is pure Coulomb friction.\n"
         f"Loaded slopes {cb[0]:+.1f} mA per duty%, "
         f"{100*(la_ma[mt][-1]/la_ma[mt][0]-1):.0f}% across\n"
         "17-29% - a VISCOUS term the load adds.\n"
         "The 15% point sits ON the 14.5% sync gate\nand reads high; the trend starts at 17%.",
         fontsize=7.6, color=INK_2, ha="left", va="top", bbox=BOX)
axB.set_xlabel("duty command  [%]")
axB.set_ylabel("motor current  [mA]")
axB.set_title("Load adds a viscous term", fontsize=10, fontweight="bold", pad=8)
# Floor dropped to 180 so the legend clears the free-wheel trace, which dips to
# 212 mA at 23% and was running through the "loaded rig" label.
axB.set_xlim(13, 31); axB.set_ylim(180, 400)
axB.legend(loc="lower left", fontsize=8, labelcolor=INK_2, handletextpad=0.6)

# ===== Panel C: the curvature is real ========================================
axC.plot(shared, res_a, "-o", color=S1, lw=LW, ms=7, **RING, zorder=4,
         label="ascending")
axC.plot(shared, res_d, "-s", color=S2, lw=LW, ms=7, **RING, zorder=4,
         label="descending")
axC.axhline(0, color=BASELINE, lw=1, zorder=1)
axC.text(10.4, 1.06,
         f"Residuals track each other, r = {corr:+.3f}.\n"
         "Thermal drift would ANTI-correlate -\n"
         "it follows elapsed time, so reversing\n"
         "the duty order flips its sign. This does\n"
         "not flip, so the curvature belongs to\n"
         "the plant and not to the motor warming.",
         fontsize=7.3, color=INK_2, ha="left", va="top", bbox=BOX)
axC.set_xlabel("duty command  [%]")
axC.set_ylabel("residual vs pooled fit  [rpm]")
axC.set_title("Curvature, not drift", fontsize=10, fontweight="bold", pad=8)
axC.set_xlim(10, 30.5); axC.set_ylim(-0.62, 1.10)
axC.legend(loc="lower right", fontsize=8, labelcolor=INK_2, handletextpad=0.6)

# ===== Panel D: how much of the gain variation is real =====================
axD.plot(gl_d, gl_g, "o", color=S1, ms=6, **RING, zorder=3,
         label="point-to-point, 2% steps")
axD.plot(gain_x, gain_q, "-", color=S2, lw=LW + 0.4, zorder=5,
         label="quadratic fit, pooled band")
axD.axhline(c_pool[0], color=BASELINE, lw=1.2, ls="--", zorder=1)
axD.fill_between(gain_x, c_pool[0] - gain_noise, c_pool[0] + gain_noise,
                 color=BASELINE, alpha=0.22, zorder=0)
axD.text(29.6, c_pool[0], f"linear\n{c_pool[0]:.3f}", fontsize=7.5,
         color=INK_2, va="center", ha="left")
axD.text(12.6, 1.125,
         f"Point-to-point scatter is mostly MEASUREMENT:\n"
         f"the rig repeats to +/-{floor:.2f} rpm, which over a 2%\n"
         f"step is +/-{gain_noise:.2f} rpm/% of apparent gain (shaded).\n"
         f"The real curvature is the quadratic's, "
         f"{gain_q[0]:.2f} -> {gain_q[-1]:.2f}.\n"
         f"It buys only {rms_lin-rms_quad:.3f} rpm of rms against a\n"
         f"+/-{floor:.2f} rpm floor, so the MODEL STAYS LINEAR\n"
         f"and the {100*(gain_q[0]-gain_q[-1])/c_pool[0]:.0f}% droop is carried as a PID constraint.",
         fontsize=7.3, color=INK_2, ha="left", va="top", bbox=BOX)
axD.set_xlabel("duty command  [%]")
axD.set_ylabel("local gain  d(rpm)/d(duty%)")
axD.set_title("Gain droops ~25%, but stay linear", fontsize=10,
              fontweight="bold", pad=8)
axD.set_xlim(12, 30.5); axD.set_ylim(0.55, 1.30)
axD.legend(loc="lower left", fontsize=7.6, labelcolor=INK_2, handletextpad=0.6)

fig.text(0.5, 0.955,
         "12 V loaded-rig plant characterisation - RobertUN wheel node",
         ha="center", va="center", fontsize=13.5, fontweight="bold", color=INK)
fig.text(0.5, 0.034,
         "VM ~12 V  |  DRV8874 slow decay, trip 1580 mA  |  treadmill-belt rig, "
         "1047 g normal load  |  enc window 100, 30 s dwell, 2% steps\n"
         "speed from the COUNT slope, never from mrpm  |  0 sequence gaps and "
         "0 tx_dropped on every run  |  taken by tools/bench/bench.py",
         ha="center", va="center", fontsize=7.8, color=MUTED, linespacing=1.6)

fig.savefig(OUT, dpi=150, facecolor=SURFACE)
print(f"wrote {OUT}\n")

# ------------------------------------------------------------ console report --
for name, c, d, rpm in (("loaded ascending  11-29%", c_la, la_d, la_rpm),
                        ("loaded descending 11-29%", c_ld, ld_d, ld_rpm),
                        ("loaded POOLED     11-29%", c_pool, pool_d, pool_rpm),
                        ("free wheel ref    11-29%", c_fa, fa_d, fa_rpm)):
    m = (d >= BAND[0]) & (d <= BAND[1])
    res = rpm[m] - np.polyval(c, d[m])
    print(f"{name:26s} rpm = {c[0]:.4f} d {c[1]:+.4f}   R2 {R.r2(d[m], rpm[m], c):.5f}"
          f"   max|res| {np.abs(res).max():.3f}   rms {np.sqrt((res**2).mean()):.3f}")
print(f"\n{'load costs':26s} {100*(1-c_pool[0]/c_fa[0]):.1f}% of the free-wheel slope")
print(f"{'desc - asc, mean':26s} {gap.mean():+.3f} rpm   (the repeatability floor)")
print(f"{'residual correlation':26s} {corr:+.3f}  (thermal drift would be negative)")
print(f"{'gain, quadratic 11->29%':26s} {gain_q[0]:.3f} -> {gain_q[-1]:.3f} rpm/%"
      f"   ({100*(gain_q[0]-gain_q[-1])/c_pool[0]:.0f}% droop)")
print(f"{'gain noise from 2% steps':26s} +/-{gain_noise:.3f} rpm/%  (repeatability, not plant)")
print(f"{'quadratic vs linear rms':26s} {rms_lin:.3f} -> {rms_quad:.3f} rpm"
      f"   (gains {rms_lin-rms_quad:.3f} rpm against a {floor:.2f} rpm floor - keep it linear)")
print(f"{'repeatability floor':26s} +/-{floor:.3f} rpm between the two passes")
print(f"{'free-wheel current':26s} {fa_ma[mf].mean():.1f} +/- {sd_f:.1f} mA, slope {cf[0]:+.2f} mA/% (flat)")
print(f"{'loaded current 17->29%':26s} {la_ma[mt][0]:.1f} -> {la_ma[mt][-1]:.1f} mA (viscous, {cb[0]:.1f} mA/%)")

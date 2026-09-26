#!/usr/bin/env python3
"""
The same 21-minute staircase, run in both directions.

WHY THIS FIGURE EXISTS, AND WHY IT IS NOT A SECOND COPY OF THE FORWARD ONE.
plot_velocity_loop_stair.py argues four things about a single run: the loop
tracks, it never reaches the 30% ceiling, the derated gains cost nothing, and
the ripple is mechanical. Each of those is a claim about one direction. The
rover turns both ways, and the feedforward is SYMMETRIC BY CONSTRUCTION -
velocity.c:346 applies the friction offset with the sign of the setpoint and
nothing else about the model changes - so the obvious question is what that
symmetry costs.

Everything here is a DIFFERENCE, which is why it is a separate figure rather
than more series on the old one: a difference needs both runs on one axis and
an explicit noise floor, and neither fits in a panel built to make a
single-run point.

What it settles:

  1. THE LOOP IS DIRECTION-BLIND. Worst settled error 0.015 rpm forward,
     0.006 rpm reverse, against a per-point standard error of ~0.02 rpm. In
     neither direction is the tracking error resolvable at all. Panel A inset.

  2. THE PLANT IS NOT, AND THE ASYMMETRY IS ENTIRELY THE INTEGRATOR. The
     difference in commanded output tracks the difference in integrator
     contribution with r = 0.999 and 0.6 o/oo of residual. That is not a
     coincidence to be admired, it is arithmetic: the feedforward is identical
     in both directions, so whatever the plant does differently has nowhere to
     land except the integrator. Panel A - and it means the integrator is a
     direct readout of the model's direction error, which is how a future
     ff_b_reverse would be measured.

  3. NEITHER DIRECTION REACHES THE CEILING. Peak 284 o/oo forward, 277
     reverse, against 300. Reverse costs MORE at the bottom of the range and
     LESS at the top - the asymmetry changes sign near 14.5 rpm, so no single
     "reverse is x% harder" number is true. Panel B.

  4. THE RIPPLE IS THE SAME MECHANICAL FEATURE, AT 12.00 PER REVOLUTION.
     11.998 +/- 0.017 events per output revolution forward, 11.978 +/- 0.017
     reverse. Its AMPLITUDE is NOT the same: the ripple grows with speed
     forward and barely grows in reverse. Panel C.

     A NUMBER THAT MOVED, AND WHY IT IS NOT A DISAGREEMENT. The forward figure
     publishes 11.91 +/- 0.19 for the same holds of the same run. Both are
     correct; this one uses sub-bin peak refinement and that one does not.
     Without it an autocorrelation period can only be an integer number of
     20 ms control periods, a grid whose step is +/-0.47 events per revolution
     at these speeds - so the whole 0.19 "spread" was quantisation, and the two
     DIRECTIONS came out identical to every digit purely because they selected
     the same bin at all 21 setpoints. Reporting that as agreement would have
     claimed a precision the method does not have. Refined, the estimate
     tightens by a factor of 11 and the forward run lands on 12.00 to 0.02%.

  5. REVERSE DRAWS MORE CURRENT FOR LESS DUTY. +8.1% mean current while
     commanding less output at the top of the range. This figure records that
     and does not explain it. Panel D.

THE CONFOUND, STATED ONCE AND CARRIED IN PANEL A. The two runs are 67 minutes
apart, not interleaved. Anything slowly varying - motor and gearbox
temperature, belt tension, and in particular where the carriage sits on the
treadmill belt, which travels the opposite way in reverse - is aliased into
"direction". The comparison's own noise floor is drawn, so a reader can see
which differences survive it and which do not.

    python3 plot_velocity_loop_stair_direction.py
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import stairdata as D

OUT = "velocity_loop_stair_direction.png"

S1, S2, S3 = "#2a78d6", "#eb6834", "#1baf7a"
SURFACE, INK, INK_2 = "#fcfcfb", "#0b0b0b", "#52514e"
MUTED, GRID, BASELINE = "#898781", "#e1e0d9", "#c3c2b7"
WARNING, CRITICAL = "#fab219", "#d03b3b"

# ------------------------------------------------------------------- data ---
fwd = D.Stair(D.STAIR_LONG)
rev = D.Stair(D.STAIR_REV)
for s, lab in ((fwd, "forward"), (rev, "reverse")):
    assert s.clean, f"the {lab} run must be integrity-clean before anything is fitted"
assert np.allclose(fwd.m_sp, rev.m_sp), \
    "the two runs must command the same setpoint magnitudes to be differenced"
assert fwd.sign > 0 > rev.sign, "STAIR_LONG must be the forward run and STAIR_REV the reverse"
assert fwd.ff() == rev.ff(), "a feedforward difference would confound the whole figure"

gf, gr = fwd.integrity(), rev.integrity()
SP = fwd.m_sp

AF, BF, RMS_F, NF = fwd.inverse_fit()       # |out| = A * |rpm| + B
AR, BR, RMS_R, NR = rev.inverse_fit()
FF_A, FF_B = fwd.ff()
LIMIT_PM = 300.0

# The differences this figure is about. m_i is the integrator's contribution to
# the MAGNITUDE of the output, so both series are in the same units and the
# same sense, and subtracting them is meaningful.
D_OUT = rev.m_out - fwd.m_out
D_I   = rev.m_i   - fwd.m_i
CORR  = float(np.corrcoef(D_OUT, D_I)[0, 1])
RESID = float(np.sqrt(((D_OUT - D_I) ** 2).mean()))

# The noise floor OF THE DIFFERENCE. Each run's own straight-line fit leaves
# ~1.5 o/oo rms of systematic residual - real curvature in the plant, not
# sampling noise, since each point averages 2500 control steps. Two independent
# runs of that kind differenced carry the quadrature sum, and a difference
# smaller than that is not distinguishable from the two runs' own non-linearity.
FLOOR = float(np.hypot(RMS_F, RMS_R))

# interp=True: see the docstring. Without it both directions return the same
# number at every setpoint, because the estimator's bin is four times wider
# than the difference being looked for.
rip_f, rip_r = fwd.ripple_table(interp=True), rev.ripple_table(interp=True)
EVR_F, SDF = rip_f[:, 3].mean(), rip_f[:, 3].std()
EVR_R, SDR = rip_r[:, 3].mean(), rip_r[:, 3].std()
D_EVR = rip_r[:, 3] - rip_f[:, 3]
# The difference is a paired one - same 21 setpoints, same holds - so its
# uncertainty is the sem of the pairwise differences, not of either mean.
D_EVR_SEM = float(D_EVR.std(ddof=1) / np.sqrt(len(D_EVR)))
RAW_F = fwd.ripple_table()[:, 3]        # the coarse grid the forward figure publishes
# What one un-refined lag bin is WORTH, in the units panel C plots. A period is
# an integer count of 20 ms control steps, so the coarsest thing the estimator
# can say is +/- one step: evr * dt / period. This is the number the claimed
# difference has to beat, and on the raw grid it does not.
DT_MS = 1000.0 / 50.0
RAWBIN = float(np.mean(rip_f[:, 3] * DT_MS / rip_f[:, 2]))

MA_F, MA_R = fwd.h_ma.mean(), rev.h_ma.mean()
MA_PCT = 100.0 * (MA_R / MA_F - 1.0)

# Where the asymmetry changes sign for good, by linear interpolation on D_OUT.
# It must be the LAST crossing, not the first: D_OUT wobbles across zero four
# times inside the noise floor down at the bottom of the range, and taking the
# first of those reports the transition at 10.5 rpm when the sustained one is
# at 14.3. The test is therefore "the last k that is positive with everything
# after it negative" - a crossing the data does not come back from.
xing = None
for k in range(len(SP) - 1):
    if D_OUT[k] > 0 and (D_OUT[k + 1:] < 0).all():
        xing = SP[k] + (SP[k + 1] - SP[k]) * D_OUT[k] / (D_OUT[k] - D_OUT[k + 1])
XING = xing if xing is not None else float(SP[np.argmin(np.abs(D_OUT))])

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
HOLLOW = dict(markerfacecolor=SURFACE, markeredgewidth=2.0)

fig = plt.figure(figsize=(13.2, 8.6))
gs = fig.add_gridspec(2, 3, height_ratios=[1.15, 1], hspace=0.40, wspace=0.27,
                      left=0.055, right=0.985, top=0.90, bottom=0.115)
axA = fig.add_subplot(gs[0, :])
axB, axC, axD = (fig.add_subplot(gs[1, i]) for i in range(3))

# ===== Panel A: the asymmetry is the integrator =============================
axA.axhspan(-FLOOR, FLOOR, color=MUTED, alpha=0.15, zorder=1)
axA.axhline(0, color=BASELINE, lw=1.1, zorder=2)
axA.plot(SP, D_OUT, "o-", color=S1, lw=1.5, ms=6.0, zorder=6, **RING,
         label="difference in commanded output,  |out| reverse - forward")
axA.plot(SP, D_I, "s--", color=S2, lw=1.5, ms=6.5, zorder=5,
         markeredgecolor=S2, **HOLLOW,
         label="difference in what the INTEGRATOR contributed to it")
axA.set_xlabel("setpoint speed, magnitude  [rpm]")
axA.set_ylabel("reverse minus forward  [o/oo]")
axA.set_title("The direction asymmetry is the integrator, and nothing else",
              fontsize=11.5, fontweight="bold", pad=9)
axA.set_xlim(9.4, 20.6)
axA.set_ylim(-20.0, 13.5)
axA.set_xticks(np.arange(10.0, 20.5, 1.0))
axA.set_yticks(np.arange(-10.0, 6.0, 5.0))
axA.legend(loc="lower left", bbox_to_anchor=(0.004, 0.135), fontsize=8.3,
           labelcolor=INK_2, handletextpad=0.7, labelspacing=0.55,
           facecolor=SURFACE, framealpha=1.0, edgecolor=GRID)

axA.text(9.65, 13.0,
         f"The two curves are the same curve: r = {CORR:.3f}, {RESID:.2f} o/oo rms apart.\n"
         f"That is arithmetic, not luck. The feedforward is {FF_A:.2f} x rpm + {FF_B:.0f} o/oo "
         f"with the setpoint's\nsign and NOTHING else - identical in both runs - so whatever "
         f"the plant does\ndifferently has nowhere to land except the integrator. It follows "
         f"that the integrator\nis a direct readout of the model's direction error, which is "
         f"how a separate\nreverse ff_b would be measured rather than guessed.",
         fontsize=7.9, color=INK_2, va="top", ha="left", bbox=BOX)

axA.annotate(f"shaded: +/-{FLOOR:.1f} o/oo, this comparison's\n"
             f"own noise floor - the quadrature sum of the\n"
             f"two runs' fit residuals. Differences inside it\n"
             f"are the plant's curvature, not direction.",
             xy=(16.3, -FLOOR), xytext=(15.6, 13.0),
             fontsize=7.5, color=INK_2, ha="left", va="top",
             arrowprops=dict(arrowstyle="-", color=MUTED, lw=1.0, shrinkA=3, shrinkB=3))

axA.text(9.65, -19.5,
         f"CONFOUND, stated once: these two runs are 67 min apart and NOT interleaved, so temperature, belt tension and "
         f"where the carriage sits\non the belt - which travels the other way in reverse - are all aliased into \"direction\". "
         f"An A/B/A staircase would separate them.",
         fontsize=7.6, color="#9a6a04", va="bottom", ha="left",
         bbox=dict(boxstyle="round,pad=0.4", fc="#fdf6e3", ec=WARNING, lw=1.0))

# inset: tracking error, the thing that did NOT change
axE = axA.inset_axes([0.790, 0.615, 0.200, 0.355], zorder=9)
axE.axhspan(-D.QUANT_RPM / 2, D.QUANT_RPM / 2, color=MUTED, alpha=0.16, zorder=1)
axE.axhline(0, color=BASELINE, lw=1, zorder=2)
axE.errorbar(SP, fwd.h_err, yerr=fwd.h_sem, fmt="o", color=S1, ms=3.0,
             elinewidth=1.2, capsize=2, zorder=4)
axE.errorbar(SP, rev.h_err * rev.sign, yerr=rev.h_sem, fmt="s", color=S2, ms=3.0,
             elinewidth=1.2, capsize=2, zorder=5, markerfacecolor=SURFACE)
axE.set_title("tracking error, signed toward the setpoint  [rpm]\n"
              "band = +/- half an encoder count",
              fontsize=6.6, pad=3, color=INK_2)
axE.set_xlabel("setpoint magnitude [rpm]", fontsize=6.6, labelpad=1)
axE.tick_params(labelsize=6.3)
axE.set_ylim(-0.30, 0.30)
axE.set_facecolor(SURFACE); axE.patch.set_alpha(1.0)

# ===== Panel B: neither direction reaches the ceiling =======================
xs = np.linspace(9.4, 20.6, 50)
axB.axhspan(LIMIT_PM, 348, color=CRITICAL, alpha=0.10, zorder=0)
axB.axhline(LIMIT_PM, color=CRITICAL, lw=1.4, zorder=3)
axB.text(20.4, LIMIT_PM + 3.5, "duty_limit 300 o/oo - the 30% ceiling",
         fontsize=7.4, color=CRITICAL, va="bottom", ha="right")
axB.plot(xs, FF_A * xs + FF_B, "--", color=S3, lw=2.2, zorder=4)
axB.plot(xs, AF * xs + BF, "-", color=S1, lw=1.2, zorder=5)
axB.plot(xs, AR * xs + BR, "-", color=S2, lw=1.2, zorder=5)
axB.plot(fwd.m_rpm, fwd.m_out, "o", color=S1, ms=5.0, zorder=7, **RING,
         label="forward,  +10 -> +20 rpm")
axB.plot(rev.m_rpm, rev.m_out, "s", color=S2, ms=5.5, zorder=6,
         markeredgecolor=S2, **HOLLOW, label="reverse,  -10 -> -20 rpm")
axB.legend(loc="upper left", bbox_to_anchor=(0.005, 0.755), fontsize=7.4,
           labelcolor=INK_2, handletextpad=0.6,
           facecolor=SURFACE, framealpha=1.0, edgecolor=GRID)
axB.annotate("the shipped feedforward,\nsymmetric in both runs",
             xy=(12.6, FF_A * 12.6 + FF_B), xytext=(9.75, 262),
             fontsize=7.2, color=S3, ha="left", va="top",
             arrowprops=dict(arrowstyle="-", color=S3, lw=1.0, shrinkA=3, shrinkB=4))
# The crossing gets no floating callout of its own - there is no clear space
# left for one - so the fit box carries it and a leader points at the point.
axB.annotate(f"fwd  {AF:.2f} x rpm + {BF:.1f}   (rms {RMS_F:.2f})\n"
             f"rev  {AR:.2f} x rpm + {BR:.1f}   (rms {RMS_R:.2f})\n"
             f"they cross at {XING:.1f} rpm, so no single\n"
             f"\"reverse is x% harder\" is true\n"
             f"peak {fwd.m_out.max():.0f} / {rev.m_out.max():.0f} of {LIMIT_PM:.0f} o/oo; saturation\n"
             f"0% at all {len(SP)} holds, both runs",
             xy=(XING, AF * XING + BF), xytext=(20.4, 129),
             fontsize=7.2, color=INK_2, ha="right", va="bottom", bbox=BOX,
             arrowprops=dict(arrowstyle="-", color=MUTED, lw=1.0, shrinkA=4, shrinkB=5))
axB.set_xlabel("achieved speed, magnitude  [rpm]")
axB.set_ylabel("commanded output, magnitude  [o/oo]")
axB.set_title("No saturation in either direction", fontsize=10, fontweight="bold", pad=8)
axB.set_xlim(9.4, 20.6); axB.set_ylim(125, 348)

# ===== Panel C: the same mechanical feature, turning the other way ==========
axC.axhline(12.0, ls="--", color=S3, lw=1.8, zorder=3)
axC.annotate("exactly 12 per revolution", xy=(11.6, 12.0), xytext=(9.55, 12.052),
             fontsize=7.3, color=S3, ha="left", va="bottom",
             arrowprops=dict(arrowstyle="-", color=S3, lw=1.0, shrinkA=3, shrinkB=3))
axC.plot(rip_f[:, 0], rip_f[:, 3], "o", color=S1, ms=5.5, zorder=6, **RING,
         label=f"forward   {EVR_F:.3f} +/- {SDF:.3f}")
axC.plot(rip_r[:, 0], rip_r[:, 3], "s", color=S2, ms=6.0, zorder=5,
         markeredgecolor=S2, **HOLLOW, label=f"reverse   {EVR_R:.3f} +/- {SDR:.3f}")
axC.text(9.7, 11.750,
         f"The same feature, to 0.2%. Reverse sits\n"
         f"{abs(D_EVR.mean()):.3f} +/- {D_EVR_SEM:.3f} events/rev below forward,\n"
         f"negative at {int((D_EVR < 0).sum())} of {len(D_EVR)} setpoints - resolved,\n"
         f"but far too small to be a different feature.\n\n"
         f"Peak lags are refined to sub-bin here. On the\n"
         f"raw 20 ms grid one bin is +/-{RAWBIN:.2f} events/rev,\n"
         f"which is why the forward figure reads\n"
         f"{RAW_F.mean():.2f} +/- {RAW_F.std():.2f} - the same holds, coarser.",
         fontsize=6.9, color=INK_2, ha="left", va="bottom", bbox=BOX)
axC.set_xlabel("setpoint speed, magnitude  [rpm]")
axC.set_ylabel("ripple events per output revolution")
axC.set_title("The ripple is the same feature both ways",
              fontsize=10, fontweight="bold", pad=8)
axC.set_xlim(9.4, 20.6); axC.set_ylim(11.74, 12.15)
axC.set_yticks([11.8, 11.9, 12.0, 12.1])
axC.legend(loc="upper left", fontsize=7.0, labelcolor=INK_2, handletextpad=0.6,
           facecolor=SURFACE, framealpha=1.0, edgecolor=GRID)

# The amplitude, which is NOT the same - shown next to the count it is so
# easily confused with, because "the ripple is identical" is only half true.
axF = axC.inset_axes([0.635, 0.760, 0.350, 0.222], zorder=9)
axF.plot(SP, fwd.h_sd, "-", color=S1, lw=1.6)
axF.plot(SP, rev.h_sd, "--", color=S2, lw=1.6)
axF.set_title("but not the same size:  sd  [rpm]", fontsize=6.6, pad=3, color=INK_2)
axF.tick_params(labelsize=6.3)
axF.set_ylim(0.80, 1.28)
axF.set_yticks([0.9, 1.1])
axF.set_facecolor(SURFACE); axF.patch.set_alpha(1.0)

# ===== Panel D: more current for less duty ==================================
axD.axvspan(132, 180, color=WARNING, alpha=0.14, zorder=0)
axD.axvline(D.SYNC_GATE_PM, color=WARNING, lw=1.5, zorder=2)
axD.text(139.5, 211, f"{D.SYNC_GATE_PM} o/oo - synchronised current-sense floor",
         fontsize=7.2, color="#9a6a04", ha="left", va="bottom", rotation=90)
axD.plot(fwd.m_out, fwd.h_ma, "o-", color=S1, lw=1.3, ms=5.0, zorder=6, **RING,
         label=f"forward,  mean {MA_F:.0f} mA")
axD.plot(rev.m_out, rev.h_ma, "s--", color=S2, lw=1.3, ms=5.5, zorder=5,
         markeredgecolor=S2, **HOLLOW, label=f"reverse,  mean {MA_R:.0f} mA")
axD.text(186, 438,
         f"Reverse draws {MA_PCT:+.1f}% while commanding LESS\n"
         f"output at the top of the range - {rev.m_out.max():.0f} o/oo\n"
         f"against {fwd.m_out.max():.0f}, for the same speed.\n\n"
         f"More current for less duty is either real (a\n"
         f"direction-dependent load) or a sign-dependent\n"
         f"offset in the current sense. These two runs\n"
         f"cannot tell which, and the low end is inside\n"
         f"the sense floor's reach anyway. Wants an ammeter.",
         fontsize=7.0, color=INK_2, ha="left", va="top", bbox=BOX)
axD.set_xlabel("commanded output, magnitude  [o/oo]")
axD.set_ylabel("mean supply current  [mA]")
axD.set_title("Reverse: more current, less duty", fontsize=10, fontweight="bold", pad=8)
axD.set_xlim(132, 300); axD.set_ylim(205, 444)
axD.legend(loc="lower right", fontsize=7.4, labelcolor=INK_2, handletextpad=0.6,
           facecolor=SURFACE, framealpha=1.0, edgecolor=GRID)

fig.text(0.5, 0.955,
         "The same staircase both ways - 21 minutes forward, 21 minutes reverse, "
         "one argument changed",
         ha="center", va="center", fontsize=13.5, fontweight="bold", color=INK)
fig.text(0.5, 0.034,
         f"VM ~12 V  |  treadmill-belt rig, 1047 g normal load  |  Kp 3.0  Ki 10.0  Kd 0  "
         f"ff {FF_A:.3f} x rpm + {FF_B:.0f} o/oo  ilim 150  slew 4 rpm/s  |  "
         f"enc window {D.WINDOW_TICKS} -> 50 Hz, {D.QUANT_RPM:.2f} rpm per count  |  "
         f"duty_limit 300 o/oo enforced throughout\n"
         f"run {D.STAIR_LONG} ({gf['minutes']:.1f} min, --dir cw) and "
         f"{D.STAIR_REV} ({gr['minutes']:.1f} min, --dir ccw) - same profile, same gains, "
         f"same rig, 67 min apart  |  "
         f"{gf['v_rows'] + gr['v_rows']:,} control steps total, "
         f"{gf['v_gaps'] + gr['v_gaps']} lost lines, "
         f"{gf['missed'] + gr['missed']} unpublished steps, "
         f"{gf['dropped'] + gr['dropped']} bytes dropped board-side",
         ha="center", va="center", fontsize=7.6, color=MUTED, linespacing=1.6)

fig.savefig(OUT, dpi=150, facecolor=SURFACE)
print(f"wrote {OUT}\n")

# ------------------------------------------------------------ console report --
print(f"{'integrity':32s} fwd V {gf['v_rows']:,} gaps {gf['v_gaps']} missed {gf['missed']}"
      f"   |   rev V {gr['v_rows']:,} gaps {gr['v_gaps']} missed {gr['missed']}")
print(f"{'tracking, worst settled error':32s} fwd {np.abs(fwd.h_err).max():.3f} rpm, "
      f"rev {np.abs(rev.h_err).max():.3f} rpm   (per-point sem ~"
      f"{fwd.h_sem.mean():.3f} / {rev.h_sem.mean():.3f})")
print(f"{'saturation':32s} fwd {fwd.h_sat.max()*100:.0f}%, rev {rev.h_sat.max()*100:.0f}% "
      f"at all {len(SP)} holds; peak {fwd.m_out.max():.0f} / {rev.m_out.max():.0f} "
      f"of {LIMIT_PM:.0f} o/oo")
print()
print(f"{'plant inverse, forward':32s} {AF:.3f} x rpm + {BF:.2f} o/oo   (rms {RMS_F:.2f}, n={NF})")
print(f"{'plant inverse, reverse':32s} {AR:.3f} x rpm + {BR:.2f} o/oo   (rms {RMS_R:.2f}, n={NR})")
print(f"{'comparison noise floor':32s} +/-{FLOOR:.2f} o/oo   (quadrature sum of the two residuals)")
print(f"{'asymmetry, |out| rev - fwd':32s} {D_OUT.min():+.1f} .. {D_OUT.max():+.1f} o/oo, "
      f"mean {D_OUT.mean():+.2f}; sign change near {XING:.1f} rpm")
print(f"{'   above the floor at':32s} {int((np.abs(D_OUT) > FLOOR).sum())} of {len(SP)} setpoints")
print(f"{'asymmetry vs integrator':32s} r = {CORR:.4f}, {RESID:.2f} o/oo rms apart"
      f"   -> the difference IS the integrator")
print(f"{'integrator range':32s} fwd {fwd.m_i.min():+.1f} .. {fwd.m_i.max():+.1f}, "
      f"rev {rev.m_i.min():+.1f} .. {rev.m_i.max():+.1f} o/oo")
print()
print(f"{'ripple, events per rev':32s} fwd {EVR_F:.3f} +/- {SDF:.3f}, "
      f"rev {EVR_R:.3f} +/- {SDR:.3f}   (n={len(rip_f)}/{len(rip_r)} holds, sub-bin refined)")
print(f"{'   rev - fwd, paired':32s} {D_EVR.mean():+.4f} +/- {D_EVR_SEM:.4f} events/rev, "
      f"negative at {int((D_EVR < 0).sum())} of {len(D_EVR)}")
print(f"{'   one raw 20 ms lag bin is':32s} +/-{RAWBIN:.2f} events/rev -> the forward figure's "
      f"{RAW_F.mean():.2f} +/- {RAW_F.std():.2f} is the same holds on that grid")
print(f"{'ripple amplitude, sd':32s} fwd {fwd.h_sd.min():.2f} -> {fwd.h_sd.max():.2f} rpm, "
      f"rev {rev.h_sd.min():.2f} -> {rev.h_sd.max():.2f} rpm   (SE of each sd ~"
      f"{fwd.h_sd.mean()/np.sqrt(2*fwd.h_n.mean()):.3f})")
print(f"{'current':32s} fwd {MA_F:.0f} mA, rev {MA_R:.0f} mA   ({MA_PCT:+.1f}%)")

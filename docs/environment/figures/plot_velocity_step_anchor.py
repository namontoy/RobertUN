#!/usr/bin/env python3
"""
What a "step response" on this rig is actually a response to.

WHY THIS FIGURE EXISTS. `bench.py run step` reported, for a 0 -> 10 rpm step
with the shipped gains, a rise time of 2.0 s and 14% overshoot. Both numbers
were wrong, in the specific and dangerous sense that both were measurements of
something other than the controller, and both pointed at a gain change that
would have made the loop worse.

  1. `vel_slew` SHIPS AT 4 rpm/s, SO THE SETPOINT IS A RAMP, NOT A STEP.
     A 0 -> 10 rpm command spends its first 2.5 s with the loop tracking a
     moving target; 0 -> 20 spends 5 s. Anchoring rise and settling at the
     command instant charges that ramp to the loop. The tell was that a
     0 -> 10 step returned "rise 2.0 s" no matter what Kp was — it was
     reporting 0.8 x 2.5 s, the limiter's own 10-90% time. Panel A, and the
     2.94 s / 7.94 s pair under it: the loop settles in 2.9 s, the operator
     waits 7.9 s, and the 5 s between them belongs to vel_slew.

  2. THE 14% OVERSHOOT IS A RIPPLE PEAK. The settled signal carries ~1.2 rpm
     sd of mechanical ripple (12 events per output revolution — see
     velocity_loop_stair_21min.png). A single maximum drawn from that will
     sit 2-3 sd high whatever the gains do. Every peak in this figure is
     4 encoder counts above target — 1.42 rpm at both 10 and 20 rpm, which is
     the signature of quantisation and ripple, not of a controller. Panel C.

  3. WHAT SURVIVES IS `track_lag_rpm`. While the setpoint ramps, the loop's
     error IS its bandwidth: a fast loop sits close behind a moving target.
     That is the number a gain change moves, and it is the one the old metric
     had no slot for. Panel B.

  4. THE HONEST VERDICT FOR ALL THREE RUNS IS "NO OVERSHOOT RESOLVABLE, AND
     RISE NOT MEASURABLE ABOVE THE LIMITER." That is a less satisfying answer
     than "14% overshoot, lower Kp", and it is the correct one. A real step
     response needs `--slew 0`, which has not been run.

Panels B-D read the three steps taken on the rig on 2026-09-26 (cold, loaded,
12 V, shipped gains). The metrics are not recomputed here: stepdata.py imports
`step_metrics` from bench.py and calls it, so this figure cannot drift from the
tool it documents.

    python3 plot_velocity_step_anchor.py
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import stepdata as D

OUT = "velocity_step_slew_anchor.png"

S1, S2, S3 = "#2a78d6", "#eb6834", "#1baf7a"
SURFACE, INK, INK_2 = "#fcfcfb", "#0b0b0b", "#52514e"
MUTED, GRID, BASELINE = "#898781", "#e1e0d9", "#c3c2b7"
WARNING, CRITICAL = "#fab219", "#d03b3b"

# ------------------------------------------------------------------- data ---
runs = D.load()
r10a, r10b, r20 = runs
BIG = r20                        # the 0 -> 20 step, the one with room to settle

M = BIG.m
ANCHOR = M["anchor_s"]
SETTLE_LOOP = M["settle_s"]
SETTLE_CMD = M["settle_from_command_s"]
SLEW_COST = SETTLE_CMD - SETTLE_LOOP

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
gs = fig.add_gridspec(2, 3, height_ratios=[1.15, 1], hspace=0.42, wspace=0.27,
                      left=0.055, right=0.985, top=0.90, bottom=0.115)

# --------------------------------------------------- A: the anchor, in full --
axA = fig.add_subplot(gs[0, :])
t, y, sp = BIG.t(), BIG.meas(), BIG.sp()

axA.axvspan(0, ANCHOR, color=WARNING, alpha=0.10, lw=0)
axA.axhline(BIG.to_rpm, color=BASELINE, lw=1.0, ls=(0, (6, 4)), zorder=1)
axA.plot(t, sp, color=S1, lw=2.0, zorder=3,
         label="ramped setpoint — what the loop was given")
axA.plot(t, y, color=INK, lw=1.1, zorder=4,
         label="measured (encoder boxcar, what the loop acted on)")

axA.axvline(ANCHOR, color=CRITICAL, lw=1.3, zorder=5)
axA.text(ANCHOR + 0.10, 13.0, "the step response starts HERE", rotation=90,
         va="center", ha="left", fontsize=7.8, color=CRITICAL)
axA.text(0.25, 12.4, f"{ANCHOR:.1f} s of setpoint ramp at "
         f"{M['slew_rpm_s']:.2f} rpm/s\n(cfg vel_slew 4000). The loop is chasing\n"
         f"a moving target, not answering a step.",
         ha="left", va="center", fontsize=7.8, color=INK_2)

# settle_s and settle_from_command_s are the SAME INSTANT measured from two
# references, not two events. Drawing them as two dots would say the loop
# settles twice; drawing one dot with two spans says what the pair means.
T_SETTLE = SETTLE_CMD
axA.plot([T_SETTLE], [BIG.to_rpm], "o", ms=8, color=S3, zorder=6, **RING)
axA.plot([T_SETTLE, T_SETTLE], [2.6, BIG.to_rpm], color=S3, lw=0.9,
         ls=(0, (2, 3)), zorder=2)
axA.annotate("", xy=(T_SETTLE, 2.6), xytext=(0, 2.6),
             arrowprops=dict(arrowstyle="<->", color=MUTED, lw=1.0))
axA.text(3.30, 3.05, f"{SETTLE_CMD:.2f} s — what the operator waits,\n"
         f"measured from the command", ha="center", va="bottom", fontsize=7.8,
         color=INK_2)
axA.annotate("", xy=(T_SETTLE, 6.6), xytext=(ANCHOR, 6.6),
             arrowprops=dict(arrowstyle="<->", color=S3, lw=1.4))
axA.text((ANCHOR + T_SETTLE) / 2, 7.05, f"{SETTLE_LOOP:.2f} s — the loop's own,\n"
         f"measured from the ramp's end", ha="center", va="bottom",
         fontsize=7.8, color="#137f58")

axA.text(0.25, 23.6, f"One instant, two references. The {SLEW_COST:.0f} s "
         f"between them is vel_slew, and the old metric charged all of it to "
         f"the controller.", fontsize=8.4, color=INK, va="top")

axA.set_xlim(0, t[-1])
axA.set_ylim(0, 25.0)
axA.set_xlabel("seconds since the 0 → 20 rpm command")
axA.set_ylabel("rpm at the output shaft")
axA.set_title("A   A 20 rpm \"step\" is 5 s of ramp followed by 3 s of step",
              loc="left", fontsize=10.5, pad=8)
axA.legend(loc="upper left", fontsize=8.0, bbox_to_anchor=(0.003, 0.885))

# ------------------------------------------- B: what the ramp phase reveals --
axB = fig.add_subplot(gs[1, 0])


def roll(v, n=25):
    """Half a second of boxcar. The raw error swings +/-3 rpm on the ripple;
    the LAG is the thing the ripple is riding on, and it is invisible until
    the ripple is averaged out of the way."""
    if len(v) < n:
        return v
    k = np.ones(n) / n
    return np.convolve(v, k, mode="same")


for r, col, lab in ((r10a, S1, "0→10 rpm"), (r10b, S2, "0→10 rpm, re-run"),
                    (r20, INK, "0→20 rpm")):
    tt, ee = r.t(), r.sp() - r.meas()
    w = tt <= r.anchor()
    axB.plot(tt[w], ee[w], color=col, lw=0.8, alpha=0.22)
    axB.plot(tt[w], roll(ee[w]), color=col, lw=1.9,
             label=f"{lab}   lag {r.m['track_lag_rpm']:.2f} rpm")
axB.axhline(0, color=BASELINE, lw=1.0)
axB.set_xlabel("seconds into the ramp")
axB.set_ylabel("setpoint − measured, rpm")
axB.set_xlim(0, 5.05)
axB.set_ylim(-3.4, 6.6)
axB.set_title("B   Lag behind the moving setpoint\n      is the number gains move",
              loc="left", fontsize=10.5, pad=8)
axB.legend(loc="upper right", fontsize=7.6)
axB.text(0.1, -3.2, "faint = raw error, bold = 0.5 s mean. The 20 rpm ramp runs\n"
         "5 s, long enough for the loop to catch up: lag 0.13 rpm. The\n"
         "10 rpm ramps end at 2.5 s, still ~1 rpm behind. Same gains.",
         fontsize=7.2, color=INK_2, va="bottom")

# ---------------------------------------- C: the overshoot that is not one ---
axC = fig.add_subplot(gs[1, 1])
xs = np.arange(len(runs))
for k, r in enumerate(runs):
    m = r.m
    sd = m["tail_sd_rpm"]
    axC.add_patch(plt.Rectangle((k - 0.30, -2 * sd), 0.60, 4 * sd,
                                fc=WARNING, alpha=0.20, lw=0))
    axC.plot([k - 0.30, k + 0.30], [2 * sd] * 2, color=WARNING, lw=1.4)
    axC.plot([k - 0.30, k + 0.30], [-2 * sd] * 2, color=WARNING, lw=1.4)
    axC.plot([k], [m["overshoot_rpm"]], "o", ms=9, color=CRITICAL, zorder=5, **RING)
    axC.text(k, m["overshoot_rpm"] + 0.16, f"{m['overshoot_pct']:.1f}%",
             ha="center", fontsize=7.6, color=CRITICAL)
axC.axhline(0, color=BASELINE, lw=1.0)
axC.set_xticks(xs)
axC.set_xticklabels(
    [f"{r.from_rpm:.0f}→{r.to_rpm:.0f}\n{r.dwell:.0f} s dwell"
     + ("\n(0.5 s window *)" if r.m["overshoot_window_truncated"] else "")
     for r in runs], fontsize=7.6)
axC.set_xlim(-0.6, len(runs) - 0.4)
axC.set_ylim(-5.4, 4.6)
axC.set_ylabel("rpm relative to the target")
axC.set_title("C   Every \"overshoot\" is inside the ripple",
              loc="left", fontsize=10.5, pad=8)
axC.text(-0.5, 3.9, "band = ±2 sd of the settled ripple", fontsize=7.4,
         color="#9a7410", va="top")
axC.text(-0.5, -3.3, "all three peaks are 4 encoder counts (1.42 rpm) above\n"
         "target — the same distance at 10 and at 20 rpm, which a\n"
         "controller's overshoot would not be.\n"
         "* this run's dwell left only 0.5 s to search; its peak is\n"
         "   not comparable with the other two.",
         fontsize=7.2, color=INK_2, va="top")

# ------------------------------------------------- D: where the output came --
axD = fig.add_subplot(gs[1, 2])
t, ff, pi = BIG.t(), BIG.term("ff"), BIG.term("p")
ii, oo = BIG.term("i"), BIG.out()
axD.axvspan(0, ANCHOR, color=WARNING, alpha=0.10, lw=0)
axD.axhline(300, color=CRITICAL, lw=1.2, ls=(0, (5, 3)))
axD.text(0.15, 306, "300 o/oo — the 30% ceiling", ha="left", va="bottom",
         fontsize=7.4, color=CRITICAL)
axD.plot(t, oo, color=INK, lw=1.4, zorder=4)
axD.plot(t, ff, color=S1, lw=1.5, zorder=3)
axD.plot(t, pi, color=S2, lw=1.1, zorder=3)
axD.plot(t, ii, color=S3, lw=1.5, zorder=3)

# Direct labels, placed off the trace's OWN final value with a leader, because
# out and ff end 13 o/oo apart and P and I both hug zero — anything centred on
# the trace overlaps its neighbour.
xe = t[-1]
for val, lab, col, dy in ((oo[-1], "out", INK, 34),
                          (ff[-1], "feedforward", S1, -40),
                          (ii[-1], "I", S3, 52),
                          (pi[-1], "P", S2, -46)):
    axD.plot([xe - 0.06, xe - 0.06], sorted((val, val + dy)), color=col,
             lw=0.8, alpha=0.6, zorder=2)
    axD.text(xe - 0.20, val + dy, lab, ha="right", va="center", fontsize=8.0,
             color=col, fontweight="bold" if lab == "out" else "normal")

axD.set_xlim(0, t[-1])
axD.set_ylim(-115, 390)
axD.set_xlabel("seconds since the command")
axD.set_ylabel("o/oo of full duty")
axD.set_title("D   The saturation is in the ramp,\n      not at the operating point",
              loc="left", fontsize=10.5, pad=8)
axD.text(0.15, -108, f"{M['sat_fraction'] * 100:.0f}% of steps saturated, all of them\n"
         f"while accelerating; settles at "
         f"{M['out_at_rest_permille']:.0f} o/oo.",
         fontsize=7.2, color=INK_2, va="bottom")

fig.suptitle("Velocity step response — where the metric is anchored",
             x=0.055, ha="left", fontsize=13, color=INK, y=0.965)
fig.text(0.985, 0.963, "12 V loaded rig · 2026-09-26 · shipped gains "
         "(Kp 3.0  Ki 10.0  Kd 0  ff 12510/30) · vel_slew 4 rpm/s",
         ha="right", va="center", fontsize=8, color=MUTED)

fig.savefig(OUT, dpi=150, facecolor=SURFACE)
print(f"wrote {OUT}\n")

# -------------------------------------------------------------- the report --
D.report(runs)
print(f"\npanel A   0 -> 20 rpm: ramp {M['ramp_s']:.1f} s at "
      f"{M['slew_rpm_s']:.2f} rpm/s (configured {D.SHIPPED_SLEW_RPM_S:.1f}), "
      f"anchor {ANCHOR:.2f} s"
      f"\n          settle {SETTLE_LOOP:.2f} s from the anchor vs "
      f"{SETTLE_CMD:.2f} s from the command — {SLEW_COST:.2f} s is vel_slew")
print(f"panel B   track_lag_rpm  " + "  ".join(
    f"{r.from_rpm:.0f}->{r.to_rpm:.0f}: {r.m['track_lag_rpm']:.2f}" for r in runs))
print("panel C   " + "  ".join(
    f"{r.from_rpm:.0f}->{r.to_rpm:.0f}: peak +{r.m['overshoot_rpm']:.2f} rpm "
    f"= {r.m['overshoot_rpm'] / D.QUANT_RPM:.1f} counts, 2sd "
    f"{2 * r.m['tail_sd_rpm']:.2f}, above ripple "
    f"{r.m['overshoot_above_ripple']}" for r in runs))
print(f"panel D   peak out {oo.max():.0f} o/oo, saturated "
      f"{M['sat_fraction'] * 100:.0f}% of steps, at rest "
      f"{M['out_at_rest_permille']:.0f} o/oo "
      f"(ff {M['ff_at_rest_permille']:.0f}, i {M['i_at_rest_permille']:.1f})")

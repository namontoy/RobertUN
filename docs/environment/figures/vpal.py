#!/usr/bin/env python3
"""
Categorical-palette validator - OKLab separation, including colour-blind vision.

A Python port of the dataviz skill's validate_palette.js, which is JavaScript
and cannot run here: this bench box has no Node. The point of having it in the
repo at all is that the figure scripts assert a palette is safe, and an
assertion nobody can re-run is just a comment.

    python3 vpal.py "#2a78d6,#eb6834,#1baf7a" --surface "#fcfcfb"

Checks, in the order they matter:
  1. normal-vision dE over ALL pairs   >= 15   (hard floor - full-colour
     readers must be able to tell any two series apart)
  2. CVD dE over all pairs             >= 8    (6-8 is legal only with a
     secondary encoding; these figures direct-label, but the floor still holds)
  3. contrast against the surface      >= 3:1  (a WARN obligates visible
     labels; it is not dismissable)

Deltas are OKLab Euclidean distance x100. CVD is simulated with the Brettel /
Vienot severity-1.0 LMS transforms for protanopia, deuteranopia and tritanopia.
"""
import sys
import itertools
import math


def srgb_to_linear(c):
    c = c / 255.0
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def hex_to_rgb(h):
    h = h.strip().lstrip("#")
    return tuple(int(h[i:i + 2], 16) for i in (0, 2, 4))


def to_oklab(rgb):
    r, g, b = (srgb_to_linear(c) for c in rgb)
    l = 0.4122214708 * r + 0.5363325363 * g + 0.0514459929 * b
    m = 0.2119034982 * r + 0.6806995451 * g + 0.1073969566 * b
    s = 0.0883024619 * r + 0.2817188376 * g + 0.6299787005 * b
    l_, m_, s_ = (v ** (1 / 3) if v > 0 else -((-v) ** (1 / 3)) for v in (l, m, s))
    return (0.2104542553 * l_ + 0.7936177850 * m_ - 0.0040720468 * s_,
            1.9779984951 * l_ - 2.4285922050 * m_ + 0.4505937099 * s_,
            0.0259040371 * l_ + 0.7827717662 * m_ - 0.8086757660 * s_)


def de(a, b):
    return 100 * math.dist(to_oklab(a), to_oklab(b))


# Severity-1.0 sRGB-linear CVD matrices (Brettel/Vienot, as used by the skill).
CVD = {
    "protanopia":   ((0.152286, 1.052583, -0.204868),
                     (0.114503, 0.786281,  0.099216),
                     (-0.003882, -0.048116, 1.051998)),
    "deuteranopia": ((0.367322, 0.860646, -0.227968),
                     (0.280085, 0.672501,  0.047413),
                     (-0.011820, 0.042940, 0.968881)),
    "tritanopia":   ((1.255528, -0.076749, -0.178779),
                     (-0.078411, 0.930809,  0.147602),
                     (0.004733,  0.691367,  0.303900)),
}


def simulate(rgb, kind):
    lin = [srgb_to_linear(c) for c in rgb]
    m = CVD[kind]
    out = []
    for row in m:
        v = sum(row[i] * lin[i] for i in range(3))
        v = max(0.0, min(1.0, v))
        v = 12.92 * v if v <= 0.0031308 else 1.055 * v ** (1 / 2.4) - 0.055
        out.append(round(max(0.0, min(1.0, v)) * 255))
    return tuple(out)


def relative_luminance(rgb):
    r, g, b = (srgb_to_linear(c) for c in rgb)
    return 0.2126 * r + 0.7152 * g + 0.0722 * b


def contrast(a, b):
    la, lb = relative_luminance(a), relative_luminance(b)
    hi, lo = max(la, lb), min(la, lb)
    return (hi + 0.05) / (lo + 0.05)


def main(argv):
    hexes = [h for h in argv[1].split(",") if h.strip()]
    surface = "#fcfcfb"
    if "--surface" in argv:
        surface = argv[argv.index("--surface") + 1]
    cols = [hex_to_rgb(h) for h in hexes]
    surf = hex_to_rgb(surface)
    ok = True

    print(f"palette: {', '.join(hexes)}   surface {surface}\n")

    print("normal vision, all pairs      (floor 15)")
    worst = 999
    for (i, a), (j, b) in itertools.combinations(list(enumerate(cols)), 2):
        d = de(a, b)
        worst = min(worst, d)
        flag = "ok  " if d >= 15 else "FAIL"
        print(f"  {flag}  {hexes[i]} vs {hexes[j]}   dE {d:5.1f}")
        ok &= d >= 15
    print(f"  -> min {worst:.1f}\n")

    print("colour-blind vision, all pairs (floor 8)")
    cvd_worst = 999
    for kind in CVD:
        sims = [simulate(c, kind) for c in cols]
        m = min(de(a, b) for a, b in itertools.combinations(sims, 2))
        cvd_worst = min(cvd_worst, m)
        flag = "ok  " if m >= 8 else "FAIL"
        print(f"  {flag}  {kind:13s} min dE {m:5.1f}")
        ok &= m >= 8
    print(f"  -> min {cvd_worst:.1f}\n")

    print("contrast against the surface   (bar 3.0:1)")
    for h, c in zip(hexes, cols):
        r = contrast(c, surf)
        flag = "ok  " if r >= 3.0 else "WARN"
        print(f"  {flag}  {h}   {r:.2f}:1"
              + ("   -> needs a direct label" if r < 3.0 else ""))

    print("\n" + ("PASS" if ok else "FAIL") + " on the dE floors")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))

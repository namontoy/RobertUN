# Rocker-bogie geometric design: a parametric engineering guide

**The rocker-bogie can be sized entirely from two equilibrium equations and one obstacle constraint, yielding explicit formulas for every pivot location, arm ratio, and body height.** The load-equalization conditions derived by Kshirsagar & Guha (2016) and independently confirmed by Cosenza et al. (2023) dictate that the bogie pivot sits at the midpoint of the two bogie wheels, while the center of gravity must lie at exactly (2x₁ + x₂)/3 from the front wheel. These two constraints, combined with an obstacle-height geometric inequality, fully determine the skeleton of any rocker-bogie from scratch. What follows assembles every quantitative design rule, formula, and known pitfall found across the peer-reviewed literature, JPL engineering reports, optimization studies, and build-tested competition documentation — organized as a step-by-step design methodology for a 30–80 cm wheelbase, Spirit/Opportunity-inspired rover.

---

## 1. Sizing from scratch: the complete parametric procedure

The foundational framework draws on Kshirsagar & Guha (2016, *Int. J. Vehicle Structures & Systems*), Cosenza et al. (2023, *Robotica* 41(10)), Ullrich/Goktogan & Sukkarieh (2010, 10th Australian Space Science Conference), and Nayar et al. (2019, *Int. J. Advanced Robotic Systems*). Ten parameters define one side of the suspension: rocker pivot coordinates (xₐ, yₐ), bogie pivot coordinates (x_b, y_b), CoG position (x_cg, y_cg), wheel radius r, wheel width b, and inter-wheel distances x₁ (front-to-middle) and x₂ (middle-to-rear).

**Step 1 — Choose wheel diameter D from obstacle requirement.** The operational climbing limit is **≈1× wheel diameter**; the theoretical kinematic maximum approaches **2× wheel diameter** under ideal friction (μ ≥ 1.0). JPL's Rocky IV was optimized for axle-high obstacles (= wheel radius). MER was designed for 20 cm obstacles with 25 cm diameter wheels (0.8D). Curiosity handles obstacles up to ~1D (50 cm). For a medium-scale rover targeting 10 cm obstacles, D ≥ 100 mm is a reasonable starting point; for 15 cm obstacles, D ≥ 150 mm.

**Step 2 — Set the wheelbase L = x₁ + x₂.** The ratio of wheel radius to wheelbase spans **r/L ≈ 0.11–0.33** across validated designs. Nayar et al. used r/L = 0.11 (11 cm radius, 100 cm wheelbase); Kshirsagar's optimization yielded r/L ≈ 0.17 (37.5 cm radius, 225 cm half-wheelbase). For a 50 cm wheelbase with 5 cm radius wheels, r/L = 0.10 — at the lower bound, which favors stability over compactness.

**Step 3 — Set equal wheel spacing: x₁ = x₂ = L/2.** Kshirsagar's constrained Nelder-Mead optimization for a 900 kg rover converged to **x₁ = x₂ = 1.125 m** (equal spacing). This is not accidental — it is a direct consequence of the load equalization mathematics. Unequal spacing is permissible but shifts the optimal CoG location and complicates the design without clear performance benefit.

**Step 4 — Position the bogie pivot.** The **load equalization condition** requires:

> **x_b = (x₁ + x₂) / 2**

This places the bogie pivot at the horizontal midpoint of the two bogie wheels (front and middle). With equal spacing, x_b = L/2 from the front wheel. This is derived from moment balance about the bogie pin joint: setting N₁ = N₂ forces the pin joint reaction to act at the midpoint.

**Step 5 — Position the rocker pivot / center of gravity.** The second equilibrium condition:

> **x_cg = (2x₁ + x₂) / 3**

With equal spacing (x₁ = x₂ = L/2), this simplifies to **x_cg = L/2** — the CoG lies directly above the middle wheel. The rocker pivot must coincide with (or be directly above/below) this longitudinal position because the entire rover weight is transmitted through the rocker pivot to the suspension. The "near CoG" rule is not approximate — it is an exact static equilibrium requirement.

**Step 6 — Set the rocker pivot height yₐ.** This height is constrained by three competing requirements:
- **Minimum ground clearance**: yₐ must be high enough that the rover body clears obstacles. MER required ≥20 cm body clearance; Curiosity achieves **66 cm**.
- **Stability**: Higher pivot = higher CoG = worse tilt stability. MER required ≥45° static stability in pitch and roll. The static stability factor SSF = T_w / (2H), where T_w is track width and H is CoG height.
- **Obstacle geometry**: The pivot must be above the bogie pivot (yₐ > y_b > 0). Kshirsagar's optimization yielded yₐ < 1.2 m and y_b > 0.9 m for the Curiosity-class rover.

For a 50 cm wheelbase rover with 10 cm wheels, a practical pivot height is **yₐ ≈ 1.5–2.0 × wheel radius** above wheel-center height, giving a body clearance of roughly 1–1.5 wheel diameters.

**Step 7 — Set the bogie pivot height y_b.** Constrained by y_b < yₐ (bogie pivot below rocker pivot) and y_b > 0 (above ground contact plane). The bogie pivot height determines the lever arm for obstacle climbing forces. Higher y_b increases the moment arm for pushing the front wheel over obstacles but also raises the effective CoG of the bogie assembly.

**Step 8 — Verify obstacle negotiability.** Kshirsagar's Equations 12–14 provide the geometric constraints. For a step obstacle of height h:

> y_b ≥ f(r, h, x₂) — a geometric inequality ensuring the bogie can articulate over the step

For a block obstacle of height h₂ and width w:

> **x₂ ≥ w + 2r** — the wheel spacing must exceed the obstacle width plus one wheel diameter

---

## 2. Arm lengths, angles, and the rocker-bogie connection geometry

The angle between the rocker arm and the bogie arm on flat ground (γ_rb) is one of the most consequential design parameters. Senjaliya et al. (2022, arXiv:2209.06927) compared seven optimization algorithms on the Goktogan fitness function and found the **simulated annealing optimum at γ_rb ≈ 171.5°** — meaning the rocker and bogie form a nearly straight line with only ~8.5° of "bend" at the bogie pivot. This near-straight configuration maximizes ground clearance and the effective rocker-bogie link length (optimized at **≈500 mm**, the upper bound of the search space). An independent study on the double-lambda variant found an optimal connection angle of **τ ≈ 160°**, and yet another reported at-rest angles of α₁ = 82° and α₂ = 100° between individual links and the horizontal, which yields a comparable included angle.

**Rocker-to-bogie arm length ratio** is frequently cited as "1:1" in overview literature, but this refers to the *inter-wheel spacing* ratio (x₁:x₂), not the physical arm lengths. The actual physical rocker arm (from body pivot to bogie pivot) is always longer than each bogie arm (from bogie pivot to wheel center). Across validated builds: the JPL Open Source Rover uses a **288 mm rocker arm versus 96 mm bogie arms (3:1 ratio)**, while a Scribd design calculation uses 260 mm rocker / 140 mm bogie (**1.86:1**). NASA MER's rocker is approximately **2–2.5× the bogie length**. The general rule: **rocker arm is 1.5–3× each bogie arm segment**.

**Maximum articulation angles** are critical for functionality:
- **Bogie articulation**: Limited to approximately **±30°** from nominal. Beyond this, the mechanism geometry degrades — ground clearance collapses, and load distribution becomes severely unequal. NASA documentation and multiple sources consistently cite this ±30° range.
- **Rocker articulation**: MER's rocker-bridge joint deploys at **39° rotation** from stowed. The operational range is defined by bump stops (rubber pads on MER) that prevent over-rotation. The differential mechanism constrains the two rockers to equal-and-opposite rotation, so the chassis pitch = (γ_left + γ_right)/2.
- **Total system capability**: The rover body tilts to only **half** the angle that any individual wheel experiences, thanks to the differential averaging. A rover capable of ±30° rocker articulation on each side can handle terrain features causing up to 60° of wheel-level displacement while the body only tilts 30°.

**How these angles relate to obstacle height**: The maximum climbable obstacle height h_max depends on wheel radius r, the friction coefficient μ, and the suspension's ability to maintain positive normal forces on all wheels throughout the climbing sequence. Bickler himself established that rocker-bogie rovers **cannot climb vertical walls with μ < 0.68**. For μ = 0.5 (typical dry soil), the front wheel cannot begin climbing because the required friction force exceeds available traction before the middle wheel lifts off (the snagging condition, detailed in Section 4). For μ ≥ 1.0 (aggressive grousers on rocky terrain), obstacles up to 2D become achievable.

---

## 3. What goes wrong: validated failure modes and design pitfalls

The following failure modes are drawn from MER flight engineering (Harrington & Voorhees, 2004), university rover competition teams (Cal Poly 2023, R-BEAR 2019), and the JPL Open Source Rover community. Each represents a build-tested lesson.

**Rocker pivot too far forward or backward.** Moving the pivot shifts weight distribution between the front wheel and the bogie pair. The load equalization formula x_cg = (2x₁ + x₂)/3 is exact — any deviation produces unequal wheel loads on flat ground. The R-BEAR team placed battery packs at the bogie end, shifting the CoG rearward. Result: the bogie tipped upward and lost ground contact on slopes exceeding ~30°. When MER's CoG rose during development (instruments added), JPL had to **extend the bogie by 17 cm** to restore the 45° stability margin. The fix was not to move the pivot but to lengthen the bogie arm to widen the support polygon.

**Rocker arms too long versus too short.** Excessive rocker length increases bending and torsional loads at the pivot — MER's rocker-bridge joint endured **714 N·m bending + 506 N·m torsion** in the worst case (center wheel falling into a 20 cm hole), partly because the 12 cm lateral offset between the rocker-bridge joint and the wheel plane creates a large torsion moment arm. Longer arms also increase the turning radius and risk wheel-body interference during articulation. Arms too short reduce the stability polygon, decrease obstacle climbing ability, and prevent maintaining all-wheel ground contact on uneven terrain.

**Asymmetric bogie arms.** Equal-length bogie segments (L₁ = L₂) produce equal load on the two bogie wheels when the bogie pivot is centered. Asymmetry shifts the effective pivot, overloading the wheel closer to the pivot. Cosenza et al.'s numerical model with L₁ = L₂ = 54.73 mm shows exact equal loading on flat ground; any departure from this creates a permanent load bias. Intentional asymmetry can be used if one bogie wheel needs more traction (e.g., the front wheel for obstacle approach), but this is uncommon in standard designs.

**CoG too high.** Tilt stability is **directly and primarily limited by CoG height**. The static stability angle θ_tip = arctan(T_w / 2H_cg), where T_w is the track width and H_cg is the CoG height. For MER (T_w ≈ 1.5 m, H_cg ≈ 0.5 m estimated), this gives roughly 56° — comfortably above the 45° requirement. For a small rover with T_w = 40 cm and H_cg = 20 cm, θ_tip ≈ 45° — right at the limit. Every centimeter of CoG rise erodes stability.

**Bogie pivot failures.** This is the single most common mechanical failure point across competition rovers. Cal Poly's 2023 URC rover experienced bogie pivot failure because the clearance between an 8 mm dowel pin and a waterjet-cut carbon fiber hole was too large — the bogie bowed outward, nearly fracturing when driving in reverse. The R-BEAR team had 3D-printed ABS bogie pivot joints that fractured after just three test runs due to insufficient wall thickness around small fastener holes. **The fix: precision bearings, tight tolerances (<0.05 mm clearance), and post-processing all holes to final dimension regardless of manufacturing method.**

**Differential mechanism errors.** Omitting the differential entirely is the most common hobbyist mistake — without it, the body has no pitch constraint and simply flops forward or backward until it bottoms out. The differential bar length determines mechanical advantage; MER used a geared bevel-gear differential with design loads of 714 N·m bending and 506 N·m torsion. The differential must pivot at the body's longitudinal center. Short connecting rods from bar ends to rocker arms require free pivots — any binding kills the averaging function. The optimal differential gear ratio is **j ≈ 1.0–1.13** (Senjaliya optimization), meaning nearly equal-and-opposite rocker rotation. The R-BEAR team verified that their differential reduced body tilt to ~40% of rocker-bogie input angle, meeting the ≤50% design target.

**Critical fabrication mistakes that kill suspension function:**
- Overtightened pivot joints that prevent articulation (use self-locking nuts, tighten then back off slightly)
- Non-mirrored left/right assemblies (must be mirror images, not copies)
- Motor bodies facing ground, reducing clearance on rocky terrain
- Wire routing through articulating joints without slack, causing pinching
- Sharp aluminum extrusion edges that cut wires during motion

---

## 4. Obstacle climbing: the force geometry sequence and the snagging condition

Cosenza et al. (2023) provide the most complete published force analysis of obstacle climbing. The sequence, analyzed for the bogie-forward configuration (front bogie wheel contacts obstacle first), proceeds through six critical states.

**Phase 1 — Approach.** The front bogie wheel (A) contacts the vertical face of a step obstacle of height h ≥ r. The obstacle exerts a horizontal reaction R_AX and a vertical reaction R_AZ on the wheel. For the front wheel to begin climbing, the friction condition must hold:

> **R_AZ ≤ f · R_AX = f · (F_B + F_C)**

where f is the ground-wheel friction coefficient, and F_B, F_C are the driving forces from the middle and rear wheels. Simultaneously, the driving wheels must not slip:

> **F_B ≤ f · R_BZ** and **F_C ≤ f · R_CZ**

**Phase 2 — Force redistribution during approach.** As rear wheel C pushes forward with force F_C, the vertical reactions shift. From rocker equilibrium (Cosenza Eq. 12):

> **R'_CZ = W/3 + F_C · (h + r) / L**

The rear wheel load *increases* during pushing. From bogie equilibrium (Cosenza Eq. 13):

> **R'_BZ = [R_EZ · (L/3 + r) + F_B · r − F_C · h] / (2L/3 + r)**

As the pushing force F_C increases, **R_BZ decreases** — the middle wheel progressively unloads.

**Phase 3 — The snagging condition.** At a critical pushing force F_C*, the middle wheel normal force R_BZ reaches zero and the wheel lifts off the ground. Beyond this point, the bogie rotates freely around the lifted middle wheel, and the front wheel cannot generate enough vertical friction to climb. **For μ = 0.5 (typical loose soil), the snagging limit is reached before the climbing condition is satisfied** — the rover gets stuck. For μ = 1.0 (aggressive grousers on rock), a feasible range of F_C < F_C* exists where climbing succeeds. This is the fundamental reason why **grouser design and ground friction dominate obstacle climbing performance**, not just wheel diameter.

**Phase 4 — Front wheel crests obstacle.** Once the front wheel reaches the top of the step, it transitions from wall contact to top-surface contact. The bogie angle changes rapidly as the front wheel's contact point shifts. The rocker arm angle decreases (rocker tilts forward) as the bogie rides up.

**Phase 5 — Middle wheel approaches.** The middle wheel (worst climber per Wang et al. 2009, IEEE) must now surmount the obstacle. It gets squeezed between the traction forces of the front and rear wheels with unfavorable geometry — the front wheel is now above and pulling, while the rear wheel pushes. If the obstacle height exceeds the middle wheel's climbing capability, the rover stalls here.

**Phase 6 — Rear wheel climbs.** The rear wheel has the **best obstacle-climbing capability** of all three, because both front and middle wheels (now on the upper surface) provide traction to push/pull it over the obstacle.

**Climbing capability ranking** (confirmed by Chinese quasi-static simulation, Wang et al. 2009): **Rear wheel > Front wheel > Middle wheel**. The middle wheel is always the weakest link in the climbing sequence.

---

## 5. Ground clearance and body height: the geometric relationships

The body height above ground is determined by a chain of geometric relationships. On flat ground with all wheels at the same level:

> **z_body = r + y_a**

where r is wheel radius and y_a is the vertical distance from wheel center to the rocker pivot. More precisely, using the Cosenza notation with link lengths L₃ (rocker arm from chassis pivot D to rear wheel C) and L₄ (rocker arm from chassis pivot D to bogie pivot E):

> **z_D = z_C + L₃ · sin(θ₃)**

where z_C = r (rear wheel center height on flat ground) and θ₃ is the rocker link angle from horizontal. The bogie pivot height:

> **z_E = z_A + L₁ · sin(θ₁)**

where z_A = r and θ₁ is the bogie link angle. The closure equations connecting all link angles through the kinematic chain are:

> **(h₁ + r·cos α₁) + L₁·sin θ₁ = (h₂ + r·cos α₂) + L₂·sin θ₂**

> **(h₂ + r·cos α₂) + L₂·sin θ₂ + L₄·sin θ₄ = (h₃ + r·cos α₃) + L₃·sin θ₃**

where h_i are terrain heights and α_i are ground surface angles at each wheel contact. These reduce to the standard form **A·sin θ + B·cos θ + C = 0** and can be solved analytically.

**Minimum ground clearance** is the lowest point of the rover body (typically the underside of the chassis) above the highest ground point between the wheels. This is constrained by:

> **c_min = z_body − max(h_terrain)** for all terrain points under the body

MER's minimum ground clearance requirement was **20 cm**. The constraint yₐ > y_b (Kshirsagar Eq. 15) ensures the body is always above the bogie assembly.

**Geometric trafficability formula** (from Goktogan & Sukkarieh 2010): the maximum vertical distance z_t between the wheel center and an obstacle edge must satisfy **z_t ≥ c** (ground clearance), where z_t is a function of wheel radius r, obstacle height h, and geometric parameters. When z_t < c, the chassis bellies out on the obstacle — a geometric trafficability failure distinct from a traction failure.

---

## 6. Weight distribution: the complete force balance equations

### Flat ground, nominal loading

With the suspension designed per the equalization conditions (x_b = (x₁+x₂)/2, x_cg = (2x₁+x₂)/3), the result is trivial:

> **N₁ = N₂ = N₃ = W/3**

where W is the weight carried by one side of the rover (half total weight for a symmetric rover).

### Flat ground, eccentric payload

When the CoG shifts by distance e from nominal (Cosenza et al. 2023, Eq. 1):

> **N_front = N_middle = (W/3)(1 + e/p)**
>
> **N_rear = (W/3)(1 − 2e/p)**

where p is the inter-wheel spacing. Note the **2× sensitivity** on the rear wheel — a 10% shift in CoG (e = 0.1p) reduces the rear wheel load by 20% but increases each bogie wheel load by only 10%. This asymmetry explains why rear-biased CoG is more dangerous than forward-biased: the rear wheel can unload and lose traction entirely.

### Uneven terrain (one wheel elevated)

Cosenza et al. provide numerical examples for a small-scale rover (L₁ = L₂ = 54.73 mm, L₃ = 51.49 mm, L₄ = 105.26 mm, r = 31.7 mm, p = 76.87 mm, half-rover mass = 0.50 kg):

| Condition | N_front (N) | N_middle (N) | N_rear (N) |
|-----------|-------------|--------------|------------|
| Flat, level | 0.66 | 0.66 | 0.66 |
| Rear wheel raised 30 mm | 0.41 | 0.97 | 0.52 |
| Middle + rear raised 30 mm | 0.72 | 0.39 | 0.80 |

The **middle wheel absorbs the largest load variation** — it can spike to nearly 1.5× nominal when the rear wheel hits a bump. This is a direct consequence of the bogie lever arm geometry and explains why middle wheel bearing and motor sizing should not use the nominal 1/3 load.

### Dynamic two-rigid-body model

Kshirsagar's complete dynamic model (Eqs. 35–48) decomposes the system into rocker+chassis and bogie, connected by a pin joint at B. For each body (force balance + moment about CG):

**Rocker + Chassis:**
- m_rc · a_x = F_x − F_x3
- m_rc · a_y = F_y + F_y3 − m_rc · g
- I_rc · α_rc = M₃ + F_y3·(x₃ − x_cg_rc) − F_x3·(y₃ − y_cg_rc) − F_y·(x_b − x_cg_rc) + F_x·(y_b − y_cg_rc)

**Bogie:**
- m_bg · a_x = −F_x + F_x1 + F_x2
- m_bg · a_y = −F_y + F_y1 + F_y2 − m_bg · g
- I_bg · α_bg = M₁ + F_y1·(x₁ − x_cg_bg) + M₂ + F_y2·(x₂ − x_cg_bg) + F_y·(x_b − x_cg_bg) − F_x·(y_b − x_cg_bg)

The pin-joint kinematic constraint at B adds 2 equations (equal acceleration at the shared point), giving **8 equations in 8 unknowns** (F_x, F_y, α_rc, α_bg, and the four acceleration components). For quasi-static analysis (all accelerations → 0), this reduces to 6 algebraic equations that can be solved in closed form for the three wheel normal forces and three driving torques.

### Slope conditions

On a uniform slope of angle α, the normal reactions shift systematically. Uphill slopes increase the front wheel load and decrease the rear; downhill slopes do the opposite. The critical condition is when any wheel's normal force reaches zero — that wheel lifts off, and the rover begins to tip. Cosenza et al. show that including the mass of wheel-motor units (lumped at the contact points, not at pivot D) **extends the stable slope range** because this low-mounted mass has a stabilizing effect. The practical implication: heavy wheel-hub motors improve stability despite increasing total mass.

---

## Conclusion: design rules for a 30–80 cm wheelbase rover

The literature converges on a set of quantitative rules that are sufficient to size a rocker-bogie from first principles. **Equal wheel spacing (x₁ = x₂) is the validated default** — it simultaneously satisfies load equalization, simplifies the CoG constraint to x_cg = L/2, and was confirmed as optimal by every published optimization study. The rocker-bogie angle should be nearly straight at rest (**~170°**), providing maximum effective link length and ground clearance. Physical arm length ratios of **1.5:1 to 3:1** (rocker:bogie segment) appear across all successful builds from JPL OSR to MER.

Three insights emerge that are not obvious from first principles. First, **the middle wheel is the system's weakest link** — it bears the highest load spikes on rough terrain and has the worst obstacle-climbing capability, so it requires the most generous motor and bearing margins. Second, **friction coefficient dominates obstacle climbing more than geometry does** — at μ < 0.68, no rocker-bogie geometry can climb a vertical wall, while at μ ≥ 1.0, obstacles up to 2× wheel diameter become achievable. Third, **the bogie pivot is the most failure-prone joint**, not the rocker pivot — every competition team report documents failures here, driven by manufacturing tolerances, insufficient bearing constraints, or inadequate wall thickness in 3D-printed or composite components. The design imperative is clear: over-engineer the bogie pivot, keep the CoG low and centered, and treat μ ≥ 0.7 as the minimum friction target for any terrain you intend to traverse.
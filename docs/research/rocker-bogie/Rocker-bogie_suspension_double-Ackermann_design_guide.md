# Rocker-bogie suspension with double Ackermann steering: a design guide

**A six-wheeled rocker-bogie rover with four-wheel (double Ackermann) steering is one of the most capable rough-terrain platforms at any scale — but integrating active steering with passive suspension compliance introduces geometric, kinematic, and mechanical challenges that few open references address directly.** NASA's Curiosity and Perseverance are the definitive implementations: both steer four corner wheels independently via software-computed angles rather than mechanical linkages, while the rocker-bogie keeps all six wheels in ground contact passively. At medium scale (30–80 cm wheelbase), the design philosophy shifts significantly — terrain obstacles become relatively larger, structural loads are forgiving enough for aluminum or 3D-printed parts, and the differential mechanism can be simplified to a turnbuckle linkage bar. This report synthesizes NASA flight heritage, academic kinematic analyses, open-source project documentation, and stability research into actionable design guidance for a medium-scale double-Ackermann rocker-bogie build.

---

## Architecture constraints that define rocker-bogie geometry

The rocker-bogie suspension is fully parameterized by **five geometric variables**: wheel radius *r*, overall rover length *L*, rocker arm length *c* (rocker pivot to front wheel), and bogie arm lengths *d* and *e* (bogie pivot to middle and rear wheels respectively). These parameters, along with center-of-gravity placement, determine obstacle climbing ability, weight distribution, and stability. The original patent by Donald Bickler (US4840394, 1988) specifies that the rocker pivot should be placed "near the center of gravity of the body along its length."

The critical design condition is **equal wheel loading**. Kshirsagar and Guha (IIT Bombay, 2016) derived the analytical conditions: for a single side with wheel spacings *x₁* (front-to-middle) and *x₂* (middle-to-rear), equal normal forces N₁ = N₂ = N₃ require the bogie pivot at position **x_b = x₁ + x₂/2** and the center of gravity at **x_cg = (2x₁ + x₂)/3**. When *x₁ = x₂* (equal spacing), this simplifies to the bogie pivot at 1.5× the inter-wheel spacing from the front wheel and the CoG at the spacing distance from the front. With equal spacing, the rocker-to-bogie arm length ratio optimizes to approximately **1:1**. The JPL ocean-worlds rover optimization (Nayar et al., 2019) confirmed these relationships through high-fidelity simulation, producing a validated prototype at 720 × 623 × 349 mm — squarely in the medium-scale range.

Obstacle climbing capability is the headline metric. The widely cited claim is that rocker-bogie can negotiate obstacles up to **2× wheel diameter**, but this is a geometric maximum under ideal friction. Actual published figures are more conservative: MER Spirit/Opportunity was designed for 20 cm obstacles with 26 cm wheels (**0.77× diameter**), Curiosity climbs 65 cm obstacles with 50 cm wheels (**1.3×**), and Perseverance's official rating is obstacles equal to the wheel diameter (**1.0×**). The JPL Open Source Rover claims "over 2× wheel height" but this likely refers to optimistic bench conditions. For a medium-scale build, designing for **1.0–1.5× wheel diameter** is realistic with good friction and properly distributed weight.

### Dimensional ratios from NASA rovers

| Parameter | Sojourner | MER (Spirit) | Curiosity | Perseverance | JPL Open Source Rover |
|-----------|-----------|-------------|-----------|-------------|----------------------|
| Wheelbase (L) | ~28 cm | ~100 cm | ~200 cm | ~210 cm | ~55–60 cm |
| Track width (W) | 48 cm | 230 cm | 280 cm | ~280 cm | ~56 cm |
| Wheel diameter | 13 cm | 26 cm | 50 cm | 52.6 cm | ~15 cm |
| Mass | 10.5 kg | 185 kg | 899 kg | 1,025 kg | ~11 kg |
| L/W ratio | ~0.58 | ~0.43 | ~0.71 | ~0.75 | ~1.0 |
| Obstacle/wheel dia | ~1.0× | 0.77× (design) | 1.3× | 1.0× | ~2× (claimed) |

The wheelbase-to-track-width ratio varies from **0.4 to 1.0** across NASA designs. A wider track improves lateral stability; Curiosity's approximately square footprint (~1.07:1) represents a practical balance. For a medium-scale rover where lateral tipping is a concern, targeting **L/W ≈ 0.7–1.0** is sensible.

---

## Three ways to implement the differential mechanism

The differential is the mechanism that couples the left and right rocker-bogie assemblies, forcing the chassis to maintain the **average pitch angle** of both sides. Without it, the body would simply flop to one side — a common mistake among first-time builders. There are three main implementations:

**The differential bar** is the simplest and most common approach, used on Sojourner, Curiosity, and Perseverance. A rigid bar pivots at the chassis center; its ends connect to the left and right rocker arms. When one rocker rises, the bar forces the opposite rocker down. At medium scale, this is implemented with turnbuckle linkages and ball joints — both the JPL Open Source Rover and Sawppy use this approach, connecting 45° brackets on each rocker through a transverse beam. The differential bar is the recommended approach for a medium-scale build due to its mechanical simplicity, ease of fabrication, and zero backlash (when ball joints are properly preloaded).

**The geared (epicyclic) differential** was used on MER Spirit/Opportunity. Two sets of planetary gearing inside the rover body create the same averaging effect as the bar but in a more compact, enclosed package. This is unnecessarily complex for medium scale and offers no advantage unless the rover must be extremely compact vertically.

**The active (servo-based) differential** replaces the mechanical linkage with IMU feedback and two servos that actively level the body. The MADspace M.A.R.S. rover project demonstrated this approach. It eliminates the mechanical linkage entirely but introduces control complexity and potential failure modes from electronics or software faults. It is a valid option if the differential bar creates routing or packaging conflicts.

---

## Double Ackermann steering on a rocker-bogie frame

This is the most technically nuanced aspect of the design. **NASA's implementation is not traditional Ackermann** — there is no mechanical trapezoid linkage connecting left and right wheels. Instead, each of the four corner wheels has an independent steering actuator, and the correct angles are computed in software from the rover's inverse kinematics. The "Ackermann condition" (all wheel velocity vectors intersecting at a single instantaneous center of rotation, or ICR) is enforced computationally at every control cycle.

For a six-wheeled rover with four steered corners and two fixed middle wheels, the **middle wheels constrain the ICR** to lie along the line extending from the middle axle perpendicular to their rolling direction. All steering angles must be consistent with this single ICR. The inverse kinematics from Helmick et al. (JPL, 2006) compute each wheel's steering angle as:

**ψᵢ = arctan2(y_wheel_i − y_ICR, x_wheel_i − x_ICR) − π/2**

and each wheel's speed as proportional to its distance from the ICR. For a double-Ackermann configuration with counter-phase steering (front and rear steer in opposite directions), the effective turning radius drops dramatically: **R = L / (tan δ_f + tan δ_r)**, where δ_f and δ_r are the front and rear virtual steering angles. This enables point turns when the ICR falls at the rover's center. In-phase steering (all wheels steer the same direction) moves the ICR to infinity, producing pure crab/lateral translation — essential for slip compensation on slopes.

The seminal kinematic paper is Tarokh, McDermott, and Hayati (IEEE ICRA 1999), which derived the full 6-DOF forward and inverse kinematics for JPL's Rocky 7 rover using wheel Jacobian matrices. Helmick et al. (2006) extended this to slip-compensated path following on Rocky 8 (all six wheels steerable), showing that **six steerable wheels create a holonomic platform** — all three planar DOFs (x, y, heading) become independently controllable. With only four steerable wheels (the Curiosity/Perseverance configuration), the system is near-holonomic, with the middle-wheel constraint limiting pure lateral motion to a crab-steering approximation.

### The critical interaction: steering geometry changes as the rocker pivots

When the rocker articulates over terrain, the steered wheels change position in 3D space. The projected wheelbase and track width on the ground plane vary with rocker and bogie angles. Helmick et al. (2006) showed that **planar kinematic models introduce errors up to 30% of distance traveled** on rough terrain. The correct approach uses full 3D coordinate transforms incorporating measured rocker and bogie joint angles at each control cycle, propagating position and orientation through the Denavit-Hartenberg chain: body → differential → rocker → bogie → steering → wheel contact point.

For a medium-scale build with modest suspension travel (±15–20°), the planar approximation is acceptable on relatively flat indoor terrain but will degrade significantly outdoors. Implementing at minimum a 2.5D model that accounts for rocker angle in the steering computation is strongly recommended. The rocker and bogie joint angles can be measured with rotary potentiometers or magnetic encoders at each pivot.

### Steering actuator placement and practical challenges

Steering motors mount directly above each corner wheel on the rocker or bogie arm — never on the chassis body (which would require a flexible coupling across the rocker pivot). The JPL Open Source Rover uses bearing blocks to isolate the steering servo from lateral loads: "The bearing block design helps protect the motor and motor shaft from forces that could otherwise damage the motor and its gearbox." Cable routing is a persistent challenge; the JPL OSR routes wires through slits in the aluminum extrusion channel of the rocker arm, and MER required electron-beam-welded titanium box beams partly for cable routing.

For medium-scale builds, a critical pitfall is **steering interference under full suspension articulation**. MER's deployment sequence required a mid-deploy steering motion to avoid wheel-to-solar-array contact. Check the full envelope of rocker/bogie travel against steering sweep at every design stage. The holding torque requirement on steering servos is also non-trivial: on slopes, gravity creates persistent torques that must be resisted even when the rover is stationary.

---

## Stability analysis and weight distribution for medium scale

### Three stability measures worth computing

The **Force-Angle Stability Measure** (FASM, Papadopoulos & Rey, 2000) is the most rigorous approach for rocker-bogie rovers. It computes β = min(θᵢ · |dᵢ| · |fᵢ|) over all tip-over axis segments of the support polygon, where θᵢ is the angle between the net force vector and the tip-over axis normal, dᵢ is the distance from the axis, and fᵢ is the resultant force magnitude. Yang et al. (2016) applied FASM to a dynamic rocker-bogie, finding that the **middle wheel has the minimum stability metric** — it defines the weakest stability axis.

The **Static Stability Factor** SSF = T/(2·h_cg) provides a quick lateral rollover check. Both MER and Curiosity were designed to withstand **≥45° tilt** without overturning, with an operational software limit at **30°**. For a medium-scale rover with track width 40 cm and CoG height 15 cm, SSF = 40/(2×15) = 1.33, corresponding to a tip-over angle of arctan(1.33) ≈ 53° — comfortably above the 30° operational target.

The **Normalized Energy Stability Margin** (Hirose et al., 2001) measures the minimum energy needed to tumble the vehicle, normalized by weight. It is weight-independent and useful for comparing stability across different rover scales.

### Weight distribution subtleties

On flat ground, a properly designed rocker-bogie distributes weight equally (**W/6 per wheel**). However, Cosenza et al. (Robotica, 2023) showed that on uneven terrain, "load distribution can undergo considerable variations." The system is **statically determinate for normal forces but indeterminate for traction forces** — meaning wheel slip distribution requires active traction control for optimal performance. An interesting modification is adding a **torsional spring** between rocker and bogie (Cosenza et al., IFToMM 2022): a moderate preload reduces the front wheel's normal force, making it easier for the front wheel to lift onto obstacles. Excessive preload, however, creates problematic load imbalance.

---

## Scale effects that change the design calculus at 30–80 cm

The square-cube law works in favor of medium-scale rovers structurally: scaling from 2 m to 0.5 m wheelbase reduces volume/mass by **64×** while cross-sections drop only **16×**, making structures relatively stronger. This enables **aluminum extrusion, 3D-printed plastics, and off-the-shelf bearings** where NASA used titanium box beams, custom bushings, and electron-beam welding. The JPL Open Source Rover demonstrates this well: GoBilda aluminum channel and 3D-printed control arms handle the loads of an ~11 kg rover without exotic materials.

The critical disadvantage is **terrain scaling**. A 5 cm rock is 10% of Curiosity's wheel diameter but **50% of a 10 cm wheel's diameter** on a medium-scale rover. Since terrestrial obstacle sizes are absolute (not proportional to rover size), medium-scale rovers face relatively much harder terrain. The mitigation is to **maximize wheel diameter relative to body size** — wheels should be as large as the chassis geometry allows. The JPL OSR's ~15 cm wheels are borderline for anything rougher than groomed paths. For serious outdoor capability, **18–25 cm wheels** are preferable on a 50–80 cm wheelbase rover.

Motor selection shifts from custom actuators to commodity hobby-grade units. The JPL OSR uses GoBilda 5203 series Yellow Jacket planetary gearmotors (26.9:1 ratio, 223 RPM); Sawppy uses LewanSoul LX-16A serial bus servos for both drive and steering (10 total). A 30–80 cm rover at 10–25 kg needs roughly **1–5 N·m** per drive wheel on flat ground and **0.3–1 N·m** per steering actuator, well within hobby gearmotor capabilities.

### Material and manufacturing approaches by project

| Approach | Example Projects | Pros | Cons |
|----------|-----------------|------|------|
| Aluminum extrusion (GoBilda/Misumi) + brackets | JPL OSR, Sawppy | Strong, modular, precise | Higher cost, requires specific ecosystem |
| 3D-printed PLA/PETG structure | ExoMy, Papaya Pathfinder | Cheapest, fastest iteration | Weaker, UV-degradable, creep under load |
| Aluminum tube + 3D-printed joints | Rover-One | Good strength-to-weight | Requires cutting/drilling, less modular |
| Carbon fiber composite + 3D-printed | URC competition teams | Best strength-to-weight | Expensive, complex layup process |

Standard **608 bearings** (skateboard type, 8mm bore) are the community standard for pivot joints at this scale — inexpensive, widely available, and adequate for low-speed loads. Sawppy uses them throughout; the known weak point is 3D-printed bearing housings, where heat-set M3 brass inserts can be ripped out under high dynamic loads.

---

## Common failure modes and what to avoid

**Missing or poorly implemented differential** is the single most common failure. Without the transverse linkage connecting both rocker assemblies, the chassis has no pitch averaging, and the rover's obstacle climbing ability is gutted. Every published rocker-bogie implementation includes some form of differential.

**Curiosity's wheel damage** provides the highest-profile failure lesson. The 0.75 mm aluminum wheel skins suffered metal fatigue from repeated bending over hard bedrock, with cracks initiating near the chevron grouser features. **Middle wheels were damaged worst** because they experience the highest cyclic loads during obstacle traversal. Perseverance addressed this with thicker skins and smoother, curved grousers. At medium scale with rubber or foam-tire wheels, this specific failure mode is irrelevant, but the lesson that middle wheels see disproportionate stress is universal.

**Bogie pivot joint slop** is well-documented by the Cal Poly URC team (2022–2023): their 8 mm dowel pin in a carbon fiber plate hole had excessive clearance, causing instability. Pivot joints must be fitted tightly with appropriate bearings — radial ball bearings plus thrust washers at minimum. The MER design used 52 mm diameter Torlon thrust/radial bushings at the rocker-bridge joint.

Other pitfalls specific to medium-scale double-Ackermann builds include: **insufficient ground clearance** (even 2–3 cm of interference is mission-ending on rough terrain), **steering cable binding** at full suspension articulation, **CoG placed too high** from heavy electronics or batteries mounted above the chassis deck, and **underestimating steering torque requirements** on slopes where gravity creates persistent side-loads on the steering actuators.

---

## Open-source reference designs closest to the target

**JPL Open Source Rover** (~55–60 cm wheelbase, ~11 kg, $1,400–$1,900) is the gold-standard reference with 4-wheel Ackermann steering, differential pivot bar, full documentation on ReadTheDocs, OnShape CAD, and GoBilda COTS construction. It has the largest active community (Slack group) and is the closest analog to a double-Ackermann rocker-bogie at medium scale.

**Sawppy the Rover** (~45–55 cm wheelbase, ~$500) by Roger Cheng is the budget alternative, using Misumi 15 mm aluminum extrusion with 3D-printed connectors. It implements 4-wheel Ackermann with spot-turn and crabbing modes. ROS integration is available via the third-party "Curio" package (srmainwaring/curio on GitHub), which includes URDF models and Gazebo simulation — though rocker-bogie differential creates closed kinematic chains that URDF cannot fully represent (SDF in Gazebo captures the full kinematics).

**Rover-One** by Chetan Borse implements **all six wheels steerable** with MG995 servos on an aluminum tube chassis, making it the closest open-source reference for maximum steering flexibility. It demonstrates strafing and variable-radius turns.

**ExoMy** (ESA) uses a **triple-bogie** rather than rocker-bogie suspension (no differential bar), but its all-six-steerable-wheels design and excellent documentation make it a useful comparison for steering implementations.

---

## Key references for double Ackermann on rocker-bogie specifically

No single paper addresses the complete double-Ackermann-on-rocker-bogie problem for a medium-scale rover. The closest composite comes from these sources:

- **Helmick et al. (2006)**, "Slip-Compensated Path Following for Planetary Exploration Rovers," *Advanced Robotics* 20(11) — derives full 3D forward and inverse kinematics for a rocker-bogie rover with six steerable wheels, parameterizable to four. The definitive reference for steering kinematics on this platform.
- **Tarokh, McDermott & Hayati (1999)**, "Kinematic Modeling of a High Mobility Mars Rover," *IEEE ICRA* — the seminal kinematic model using wheel Jacobian matrices.
- **Kang, Pang & Zeng (2025)**, "Optimal Dimensional Synthesis of Ackermann Steering Mechanisms for Three-Axle, Six-Wheeled Vehicles," *Applied Sciences* 15(2):800 — optimizes Ackermann linkages for six-wheeled vehicles, including reverse-phase (counter-steer) mode.
- **Daniel et al. (2025)**, "Towards Safe Maneuvering of Double-Ackermann-Steering Robots," *arXiv:2510.10332* — defines Double-Ackermann-Steering Mobile Robots (DASMRs) and applies deep RL for navigation.
- **Spentzas, Alkhazali & Demic (2001)**, "Kinematics of four-wheel-steering vehicles," *Forschung im Ingenieurwesen* 66(6):211–216 — general 4WS kinematic formulae including non-negligible sideslip.
- **Nayar et al. (2019)**, "Design optimization of a lightweight rocker-bogie rover for ocean worlds applications," *Int. J. Adv. Robot. Syst.* 16(6) — JPL optimization of a medium-scale (~720 mm) rocker-bogie prototype with validated motion-capture experiments.

---

## Active research groups and institutions

**NASA JPL** remains the primary authority (Bickler, Harrington, Voorhees, Nayar, Helmick, Matthies). **MIT Field and Space Robotics Lab** (Iagnemma, Dubowsky) produced foundational traction control work. **ETH Zurich Autonomous Systems Lab** (Siegwart, Thueer, Lamon) conducted the most rigorous comparative studies of rover suspensions including rocker-bogie, CRAB, and Shrimp. **Tohoku University Space Robotics Lab** (Yoshida, Nagatani) leads terramechanics-based analysis. **University of Naples Federico II** (Niola, Pagano, Savino, Cosenza) has produced the most recent analytical work on rocker-bogie force distribution and spring-modified variants. **Harbin Institute of Technology** (Gao, Deng, Ding) and **IIT Bombay** (Kshirsagar, Guha) have published optimization frameworks applicable to any scale. **Tel Aviv University** (Shiller, Mann) developed the dynamic stability analysis most relevant to operational speed limits.

---

## Conclusion: design decisions for a medium-scale double-Ackermann rocker-bogie

The strongest design approach for a 30–80 cm wheelbase double-Ackermann rocker-bogie combines a **differential bar mechanism** (turnbuckle + ball joints), **independent steering actuators at each corner wheel** (not mechanical Ackermann linkages), and **software-computed steering angles** using the ICR constraint from the fixed middle wheels. Maximize wheel diameter — target at least **20% of wheelbase** for meaningful outdoor obstacle capability. Use the equal-loading geometric constraints (bogie pivot and CoG placement from Kshirsagar & Guha) as the starting point, and verify stability with the Static Stability Factor targeting SSF > 1.2.

The most underappreciated challenge is that **rocker articulation changes the effective steering geometry in 3D**. Implementing even a simplified correction that incorporates measured rocker angles into the steering angle computation will significantly improve path tracking on uneven terrain — the planar model's 30% error figure from Helmick et al. is not a theoretical curiosity but a measured operational reality. For software, neither ArduPilot nor PX4 natively supports individual wheel angle computation for a 4-corner independent steering rover; custom ROS nodes or Lua scripting will be necessary, with the Curio package for Sawppy providing the closest starting framework.

## Otras paginas interesantes:

https://newscrewdriver.com/2020/12/12/quick-look-frederic-jelmonis-reproduction-du-rover-mars-2020/

http://spiff.rit.edu/richmond/asras/perseverance/perseverance.html

https://science.nasa.gov/resource/mars-perseverance-rover-3d-model/

https://howtomechatronics.com/projects/diy-mars-perseverance-rover-replica-with-arduino/

https://jplopensourcerover.com/#!/explore

https://github.com/Roger-random/Sawppy_Rover/tree/main


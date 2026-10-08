/**
  ******************************************************************************
  * @file           : velocity.h
  * @brief          : Closed-loop wheel speed control (W5) — PI(D) over drive.c
  ******************************************************************************
  *
  * WHAT THIS IS FOR
  * ----------------
  * Everything below this module commands DUTY. Duty is not speed: the plant's
  * gain droops 25% across the operating band, the load changes it, and the rail
  * changes it again. W5's acceptance criterion is stated in speed — "commanded
  * output speed is held within a stated tolerance across the usable range, with
  * no sustained oscillation and bounded overshoot from a step" — so something
  * has to close the loop between encoder rpm and bridge duty. This is it.
  *
  *
  * THIS IS THE POLICY LAYER. drive.c IS THE MECHANISM.
  * ----------------------------------------------------
  * The split is the one the command watchdog and the duty slew limiter already
  * use, and this module is on the other side of it from both. `drive.c` owns
  * the bridge, the duty cap, the fault latch and the deadline; it decides
  * nothing. This module decides: what speed, how fast to get there, what to do
  * when the setpoint goes stale.
  *
  * Concretely, this module NEVER touches TIM4, the CCR pair, nSLEEP, or the
  * fault latch. Its entire actuator interface is drive_set_duty() and
  * drive_coast(). If something here needs a new bridge behaviour, that
  * behaviour belongs in drive.c and this module asks for it.
  *
  *
  * ⚠️ ENABLING THIS LOOP DEFEATS drive.c's COMMAND WATCHDOG
  * ---------------------------------------------------------
  * Read this before running the loop unattended.
  *
  * drive_set_duty() calls drive_kick(), deliberately — a duty command is
  * evidence of a live caller. That is exactly why the duty slew limiter had to
  * go INSIDE drive.c rather than above it. This module is above it, and it
  * calls drive_set_duty() fifty times a second, forever. So while the loop is
  * enabled, drive.c's command watchdog can never expire, no matter what
  * happens to the host.
  *
  * That is not a bug to be worked around; it is what closing a loop MEANS. An
  * autonomous controller genuinely is a live caller. But it moves the safety
  * property: the thing that must now be watched is not "is anyone writing duty"
  * but "is anyone still choosing a SETPOINT".
  *
  * So this module carries its own watchdog, at its own layer:
  *
  *     velocity_set_setpoint()  kicks it
  *     expiry                   -> setpoint forced to 0, loop coasts, latched
  *
  * It defaults to CFG_VEL_TIMEOUT (1000 ms) rather than to off, which is the
  * OPPOSITE of the drive watchdog's default and deliberately so. The drive
  * watchdog defaults off because arming it changes the behaviour of a board
  * that worked without it. This loop has no prior behaviour to preserve, and it
  * disables the only watchdog that was there — so it has to bring a replacement
  * with it, armed. A bench operator who wants an indefinite hold sets
  * `vel timeout 0` on purpose, the same way they would disarm any other guard.
  *
  *
  * IT STEPS AT THE MEASUREMENT RATE, NOT THE TICK RATE
  * ----------------------------------------------------
  * velocity_on_tick() is called from the 1 kHz TIM6 ISR, but the PID only
  * advances when the encoder's velocity window CLOSES — 50 Hz at the default
  * 20-tick window. This is not an optimisation, it is correctness:
  *
  *   - encoder_rpm() returns the SAME value between window closures. Running
  *     the integrator at 1 kHz against a 50 Hz measurement integrates each real
  *     sample twenty times, so the effective Ki is 20x what was configured and
  *     changes if anyone touches `enc window`.
  *   - The derivative would be worse: nineteen ticks of exactly zero slope,
  *     then one tick carrying the whole step.
  *
  * So the loop reads encoder_velocity_seq() and steps on a change. dt comes
  * from the window length, so `enc window` retunes the loop's rate honestly
  * instead of silently rescaling its gains.
  *
  * 50 Hz against tau_fast = 0.219 s is roughly eleven samples per time
  * constant, which is ample. Do not reach for a faster loop before a faster
  * one is shown to be needed — shortening the window costs resolution
  * (0.36 rpm per count at 20 ticks, 0.71 at 10) and buys nothing against a
  * plant this slow.
  *
  *
  * FEEDFORWARD DOES THE WORK; THE PID ONLY CORRECTS
  * -------------------------------------------------
  * The plant was measured, so most of the answer is known before the loop runs:
  *
  *     duty% = 1.251 x rpm + 3.028          (loaded rig, 12 V, Sep 25 2026)
  *
  * That inverse is applied directly as feedforward, which is why the default
  * gains can be small. Without it the integrator has to build the entire
  * operating point from scratch on every setpoint change — slow, and it makes
  * every overshoot an integrator-unwind problem. With it, the PID sees only
  * model error, load disturbance and friction.
  *
  * The offset term (3.028% -> CFG_VEL_FF_OFFSET) is friction, so it is applied
  * with the SIGN OF THE SETPOINT and only when the setpoint is non-zero.
  *
  * ⚠️ That line is known to be ~4.5% optimistic as of Sep 26 2026 and is due a
  * re-take. Feedforward error is absorbed by the integrator, so this costs
  * accuracy of the feedforward, not of the loop — but it is why the
  * coefficients are config keys and not #defines. Re-fit, then re-enter them.
  *
  *
  * THE SETPOINT RAMP IS HERE, NOT IN drive.c
  * -------------------------------------------
  * drive.c's slew limiter limits DUTY, which is the right thing for an
  * open-loop command and the wrong thing underneath a closed loop: the PID
  * would command a duty, watch the bridge fail to arrive at it, read that as
  * error, and wind up against its own actuator. This module therefore ramps the
  * SETPOINT instead, in rpm/s, so the PID is always chasing a target the plant
  * can actually follow.
  *
  * The default of 4000 milli-rpm/s is not a guess: 5%/s duty is the rate proven
  * on the loaded rig at a 581 mA peak against a 1579 mA trip, and
  * 5 / 1.251 = 4.0 rpm/s is the same rate expressed in setpoint units.
  *
  * ⚠️ Running BOTH limiters at once is supported but is not free. If
  * drive_ramp() is armed while this loop runs, the two fight, and the symptom
  * is integrator windup. The loop defends itself — the integrator is frozen
  * whenever drive_slewing() is true — but the honest configuration is to leave
  * `drv ramp 0` and let the setpoint ramp do the limiting.
  *
  *
  * ANTI-WINDUP, AND THE FOUR TIMES THE INTEGRATOR MUST NOT MOVE
  * --------------------------------------------------------------
  * The integrator is clamped to +/-CFG_VEL_I_LIMIT, and additionally frozen
  * when integrating would make things worse rather than better:
  *
  *   1. Output saturated AND the error pushes further into the saturation.
  *      Plain conditional integration. Integrating the other way is allowed,
  *      so recovery from saturation is immediate rather than delayed by an
  *      unwind.
  *   2. drive_slewing() — the bridge has not arrived at the last command, so
  *      the error is the actuator's lag and not the plant's.
  *   3. The bridge is disabled or faulted. Error here is meaningless; without
  *      this the integrator rails while the driver sleeps and slams the wheel
  *      on the next `drv enable`.
  *   4. Setpoint and measurement are both zero. Otherwise stiction holds the
  *      wheel at rest, the integrator climbs against it, and the wheel breaks
  *      away into a lurch. Stopping is coasting, not holding zero with torque.
  *
  *
  * STOPPING IS COASTING
  * --------------------
  * A commanded setpoint of 0 walks the ramped setpoint down to 0 and then
  * COASTS — it does not hold zero speed against a push. That matches the
  * standing stop policy (coast by default, brake only at low speed) and it is
  * what makes rule 4 above safe. Holding a wheel stationary under closed-loop
  * torque is a different feature, and it needs a position loop, not this one.
  *
  * The loop never brakes. Braking is an explicit act, and it stays with the
  * caller.
  *
  *
  * DERIVATIVE IS OFF BY DEFAULT AND IS ON THE MEASUREMENT
  * -------------------------------------------------------
  * Kd defaults to 0 because it is very unlikely to be needed: tau_fast is
  * 0.219 s against a 20 ms loop, so there is nothing fast enough to anticipate,
  * and the measurement is quantised at 0.36 rpm and boxcar-filtered, which is
  * the worst possible input to a differentiator.
  *
  * If it is turned on anyway, it differentiates the MEASUREMENT and not the
  * error, so a setpoint step does not produce a derivative spike, and the
  * result is low-passed by a fixed first-order filter (D_FILTER_ALPHA in
  * velocity.c, ~4 samples at 50 Hz). Both are standard and both are
  * load-bearing here. The filter is deliberately NOT a config key: a second
  * knob on a term whose default is zero is a trap, not a feature.
  *
  *
  * GAINS, AND WHERE THE DEFAULTS CAME FROM
  * -----------------------------------------
  * Treat the shipped gains as a SAFE STARTING POINT, not as a tuning. They were
  * derived, not measured:
  *
  *     plant gain  K    = 0.07993 rpm per o/oo of duty
  *     time const  tau  = 0.219 s (fast pole; the 2.75 s pole is the BELT)
  *
  * For a PI loop on a first-order plant placed at a closed-loop time constant
  * equal to the open-loop one (i.e. no attempt to speed the plant up):
  *
  *     Kp = tau / (K * tau_cl) = 1 / K   = 12.51 o/oo per rpm
  *     Ki = 1   / (K * tau_cl)           = 57.1  o/oo per rpm-second
  *
  * The shipped defaults are roughly a quarter of that (Kp 3.0, Ki 10.0),
  * because feedforward is already supplying the operating point and because the
  * first flash of a loop should be sluggish rather than marginal. The numbers
  * above are where to walk TOWARD during tuning, not past.
  *
  * Two constraints from the plant work that tuning must respect rather than
  * rediscover:
  *   - Gain droops 25% across the band (0.899 -> 0.700 rpm/% from 11 to 29%).
  *     Place the gains at the LOW-GAIN, HIGH-DUTY end and accept a sluggish
  *     bottom, rather than a marginal top.
  *   - tau_slow (2.75 s) is the belt, not the motor. Do not integrate against a
  *     pole that will not exist on the vehicle.
  *   - The rig is 2.87x light in inertia, so its tau is a LOWER BOUND.
  *     Re-measure on the rover before the gains are frozen.
  *
  *
  * OUTPUT CEILING
  * --------------
  * CFG_VEL_MAX defaults to 300 o/oo. That is not a hardware limit, it is the
  * 30% characterisation ceiling the whole campaign runs under — the rover moves
  * slowly and above ~31% the wheel bounces on the rig belt, which makes those
  * points a property of the rig. drive_set_limit() still applies on top; the
  * loop clamps to whichever is tighter so its anti-windup logic knows the real
  * boundary rather than discovering it downstream.
  *
  *
  * CONCURRENCY
  * -----------
  * velocity_on_tick() runs in the TIM6 ISR and is the only writer of the loop
  * state. Setpoint and configuration come from the console or CAN in thread
  * context, and are single int32/uint32 words — naturally atomic on Cortex-M4,
  * so no critical section is taken and none is needed. The setpoint is stored
  * as milli-rpm for exactly this reason: a float write is also atomic here, but
  * an integer keeps the "one word, one writer" argument free of the FPU.
  *
  * Floating point in an ISR is fine on this part and is already established —
  * encoder_on_tick() computes rpm in float in the same interrupt. The FPU is
  * hard-float (-mfpu=fpv4-sp-d16 -mfloat-abi=hard) with lazy stacking, so the
  * context cost is paid only by interrupts that actually use it.
  *
  *
  * WHAT THIS MODULE DELIBERATELY DOES NOT DO
  * -------------------------------------------
  *   - It does not brake. See STOPPING IS COASTING.
  *   - It does not enable or disable the bridge. If the driver is asleep the
  *     loop still runs its arithmetic but freezes the integrator and writes
  *     nothing; bringing the bridge up stays an explicit act.
  *   - It does not act on nFAULT beyond declining to integrate. Fault POLICY
  *     is still nobody's, which is a gap, and it is still not this module's.
  *   - It does not do current control. The inner current loop is a separate
  *     thing and is blind below 14.5% duty anyway, which is exactly where the
  *     rover creeps.
  *   - It does not schedule gains. The plant is linear to +/-1.5%; the 25%
  *     droop is handled by where the gains are placed, not by switching them.
  *   - It has no telemetry channel yet. The `T,` line's schema is mirrored in
  *     node.py and baked into every committed run, so a setpoint column is not
  *     added casually. Tuning will need setpoint-vs-measured logged together;
  *     that is the next piece of work, as a separate opt-in line.
  *
  ******************************************************************************
  */
#ifndef VELOCITY_H
#define VELOCITY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/** @brief Why the loop is not driving, if it is not. */
typedef enum
{
  VELOCITY_OFF = 0,     /*!< not enabled — nothing is written to the bridge  */
  VELOCITY_RUNNING,     /*!< enabled and tracking a setpoint                 */
  VELOCITY_HOLDING,     /*!< enabled, setpoint 0, coasting by design         */
  VELOCITY_TIMEOUT      /*!< setpoint went stale — forced to 0 and latched   */
} velocity_state_t;

/**
  * @brief  Read the gains and limits from `config` and clear the loop state.
  * @note   Call AFTER config_init() and after drive_init(); the loop reads
  *         drive_limit() to work out its true output ceiling.
  */
void velocity_init(void);

/**
  * @brief  Advance the loop. Call from the 1 kHz control tick.
  *
  * Returns immediately unless the encoder's velocity window has closed since
  * the last call, so the PID steps at the measurement rate. Safe to call when
  * the loop is disabled — it keeps its state clean and writes nothing.
  */
void velocity_on_tick(void);

/**
  * @brief  Arm the loop. The setpoint is left at 0, so nothing moves until one
  *         is commanded — the same precedent as the watchdog and the ramp.
  * @note   This DEFEATS drive.c's command watchdog. See the header.
  */
void velocity_enable(void);

/** @brief Disarm the loop and coast. The bridge is left enabled. */
void velocity_disable(void);

/** @brief True if the loop is armed. */
bool velocity_enabled(void);

/** @brief What the loop is doing, and why if it is doing nothing. */
velocity_state_t velocity_state(void);

/**
  * @brief  Command a wheel speed, in milli-rpm at the OUTPUT shaft.
  *
  * Signed; positive is CW, matching the encoder and duty sign convention. The
  * value is a TARGET — the ramped setpoint walks to it at CFG_VEL_SLEW. Kicks
  * the setpoint watchdog, and clears a latched timeout.
  */
void velocity_set_setpoint(int32_t milli_rpm);

/** @brief The commanded target, milli-rpm. */
int32_t velocity_setpoint(void);

/** @brief The RAMPED setpoint the PID is actually chasing right now,
  *        milli-rpm. Equal to velocity_setpoint() once the ramp arrives. */
int32_t velocity_ramped_setpoint(void);

/** @brief Most recent measurement the loop acted on, milli-rpm. */
int32_t velocity_measured(void);

/** @brief Loop error, milli-rpm — ramped setpoint minus measurement. */
int32_t velocity_error(void);

/** @brief Duty the loop last wrote, per-mille. */
int16_t velocity_output(void);

/** @brief The three contributions to that output, per-mille: feedforward,
  *        proportional, integral, derivative. For tuning. */
void velocity_terms(int16_t *ff, int16_t *p, int16_t *i, int16_t *d);

/** @brief True if the last step clamped at the output ceiling. */
bool velocity_saturated(void);

/** @brief Zero the integrator and the derivative state, leaving the setpoint
  *        alone. The thing to do after changing a gain mid-run. */
void velocity_reset(void);

/* --- the published snapshot, for telemetry -------------------------------
 *
 * The accessors above are fine for a human at a console reading one line at a
 * time. They are useless for TUNING: a step response is a one-second event at
 * 50 Hz, and polling nine separate functions from the main loop would sample
 * each at a different moment and stitch together a state the loop never
 * actually had.
 *
 * So the loop publishes instead. At the end of each control step it fills one
 * slot with a coherent snapshot of that step, and the main loop drains it.
 *
 * WHY PER STEP AND NOT ON THE TELEMETRY SCHEDULE. The loop advances only when
 * encoder_velocity_seq() changes — 50 Hz at the default window. Sampling that
 * on the `telem` timer would alias it: at 100 Hz every step appears twice, at
 * 30 Hz they beat against each other. Neither reads as a step response, and
 * the integrator and the derivative only mean anything per step. Publishing
 * per step gives exactly one record per control decision and is self-limiting
 * — the rate is whatever the loop's rate is, by construction.
 *
 * WHEN THE READER FALLS BEHIND. There is one slot, so a main loop that does
 * not drain fast enough loses steps. It is told: the overwrite sets a sticky
 * flag that appears as VELOCITY_SAMPLE_MISSED on the NEXT snapshot taken. A
 * silently decimated stream would look like a slow loop rather than a slow
 * host, which is exactly the wrong conclusion to hand somebody tuning gains.
 */

#define VELOCITY_SAMPLE_SATURATED   0x01u  /*!< output hit the ceiling        */
#define VELOCITY_SAMPLE_FROZEN      0x02u  /*!< integrator did not advance    */
#define VELOCITY_SAMPLE_SLEWING     0x04u  /*!< ...because drive_slewing()    */
#define VELOCITY_SAMPLE_NOBRIDGE    0x08u  /*!< ...because disabled/faulted   */
#define VELOCITY_SAMPLE_WD_EXPIRED  0x10u  /*!< setpoint watchdog has fired   */
#define VELOCITY_SAMPLE_RAMPING     0x20u  /*!< ramped setpoint != commanded  */
#define VELOCITY_SAMPLE_MISSED      0x40u  /*!< a step was lost before this   */

/**
  * @brief One control step, as a coherent set.
  *
  * FROZEN without SLEWING or NOBRIDGE means the integrator was held because
  * the output was clamped and the error pushed further into the clamp — that
  * is anti-windup working. FROZEN with one of the other two means the loop was
  * held off by something outside it. Keeping them apart is the point: the two
  * look identical in the output and want opposite responses.
  */
typedef struct
{
  uint32_t step;       /*!< control-step index; restarts at 0 on enable       */
  uint32_t ms;         /*!< HAL_GetTick() at the step                         */
  int32_t  sp_mrpm;    /*!< the RAMPED setpoint, milli-rpm — what was chased  */
  int32_t  meas_mrpm;  /*!< encoder_rpm() x1000, what the loop acted on       */
  int16_t  out;        /*!< per-mille handed to drive_set_duty()              */
  int16_t  ff;         /*!< feedforward contribution, per-mille               */
  int16_t  p;          /*!< proportional contribution, per-mille              */
  int16_t  i;          /*!< integrator, per-mille                             */
  int16_t  d;          /*!< derivative contribution, per-mille                */
  uint8_t  flags;      /*!< VELOCITY_SAMPLE_* above                           */
} velocity_sample_t;

/**
  * @brief  Take the pending snapshot, if there is one that has not been taken.
  * @param  dst  filled only when the return is true.
  * @retval true if a NEW step was copied; false if nothing has happened since
  *         the last call.
  *
  * Copies under a critical section, so the struct is always one step's worth
  * of state and never a mixture of two. Safe to call from the main loop at any
  * rate; calling faster than the loop runs simply returns false.
  */
bool velocity_take_sample(velocity_sample_t *dst);

/* --- setpoint watchdog -------------------------------------------------- */

/**
  * @brief  Deadline on the SETPOINT, in milliseconds. 0 disables it.
  * @note   This is the watchdog that matters once the loop is enabled, because
  *         the loop itself keeps drive.c's alive. Read the header.
  */
void velocity_set_timeout(uint32_t ms);

/** @brief Current setpoint deadline, ms. 0 means disarmed. */
uint32_t velocity_timeout(void);

/** @brief Milliseconds left before the setpoint goes stale. */
uint32_t velocity_timeout_remaining(void);

/** @brief True if the setpoint watchdog has expired since it was last armed.
  *        Sticky, exactly as drive_timeout_expired() is: a setpoint kick
  *        refreshes the countdown and velocity_set_timeout() restarts it, but
  *        neither clears this. Only velocity_enable() does. */
bool velocity_timeout_expired(void);

/** @brief Expire the setpoint watchdog on the next tick, as if the deadline
  *        had just been missed: setpoint 0, coast, latch set. Ignored while the
  *        loop is off. For bus-off (can_cmds.md §7.2), which must not wait up
  *        to vel_tmo for the countdown. */
void velocity_expire_now(void);

/* --- gains and limits, live ---------------------------------------------- */
/* All in milli-units so they pass through `config` as int32 without losing
   resolution: a Kp of 3.0 o/oo per rpm is stored and set as 3000. */

void     velocity_set_kp(int32_t milli);       /*!< o/oo per rpm          */
int32_t  velocity_kp(void);
void     velocity_set_ki(int32_t milli);       /*!< o/oo per rpm-second   */
int32_t  velocity_ki(void);
void     velocity_set_kd(int32_t milli);       /*!< o/oo-second per rpm   */
int32_t  velocity_kd(void);

void     velocity_set_ff(int32_t slope_milli, int32_t offset_permille);
int32_t  velocity_ff_slope(void);              /*!< milli o/oo per rpm    */
int32_t  velocity_ff_offset(void);             /*!< o/oo, signed by setpoint */

void     velocity_set_i_limit(uint16_t permille);
uint16_t velocity_i_limit(void);
void     velocity_set_max(uint16_t permille);
uint16_t velocity_max(void);
void     velocity_set_slew(int32_t milli_rpm_per_s);  /*!< 0 = no setpoint ramp */
int32_t  velocity_slew(void);

/** @brief Loop period actually in use, in microseconds — derived from
  *        `enc window`, not configured here. */
uint32_t velocity_period_us(void);

#ifdef __cplusplus
}
#endif

#endif /* VELOCITY_H */

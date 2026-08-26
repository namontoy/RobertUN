/**
  ******************************************************************************
  * @file           : encoder.h
  * @brief          : Drive-motor quadrature encoder on TIM2 (PA15 / PB3)
  ******************************************************************************
  *
  * WHAT THIS IS FOR
  * ----------------
  * Turns TIM2's hardware quadrature counter into a position that does not wrap
  * and a velocity that is usable by a control loop. W4's acceptance criterion
  * runs entirely through this module: turn the wheel by hand, read the count,
  * check it matches the rotation.
  *
  *
  * THE COUNTER IS READ AS A DELTA, NEVER AS A POSITION
  * ---------------------------------------------------
  * `TIM2->CNT` free-runs and wraps. Every tick this module reads it, takes the
  * difference from the previous reading as a SIGNED 32-bit value, and adds that
  * to a 64-bit accumulator:
  *
  *     delta = (int32_t)(now - prev);      // wraps correctly, both directions
  *     position += delta;
  *
  * The cast is what makes it work. Unsigned subtraction wraps modulo 2^32, and
  * reinterpreting the result as signed recovers the true difference for any
  * movement smaller than half a counter range. That is 2^31 counts, or about
  * 255 000 output revolutions, per tick — so the only way to break it is to
  * stop calling encoder_on_tick().
  *
  * This code is identical whether the counter is 16- or 32-bit. Choosing TIM2
  * (one of only two 32-bit timers on this part) does not remove the need for
  * it; it moves the horizon at which a STALLED loop starts losing revolutions
  * from 7.8 output revolutions to about 511 000.
  *
  *
  * RESOLUTION
  * ----------
  * 64 CPR at the MOTOR shaft (x4 quadrature, both edges of both channels)
  * through the 131.3:1 gearbox:
  *
  *     64 x 131.3 = 8403.2 counts per revolution of the OUTPUT shaft
  *
  * 131.3:1 is a rounded ratio, so 8403.2 is not an integer and never will be.
  * That matters at W9 (precision calibration), not here.
  *
  *
  * WHY VELOCITY IS NOT COMPUTED EVERY TICK
  * ---------------------------------------
  * At a 1 kHz tick and 8403.2 counts/rev, one count of difference is
  *
  *     1000 x 60 / 8403.2 = 7.14 rpm
  *
  * against a motor whose top speed is about 60 rpm at the 9.5 V rail. Velocity
  * from a single tick would therefore be quantised to roughly 0, 7, 14, 21 rpm
  * — useless as PID feedback.
  *
  * So velocity is measured over a window of N ticks instead. The quantisation
  * scales down with the window:
  *
  *     N =  1   ->  7.14  rpm per count
  *     N = 10   ->  0.71  rpm
  *     N = 20   ->  0.36  rpm      <- default, updates at 50 Hz
  *     N = 50   ->  0.14  rpm
  *
  * 50 Hz is ample for the velocity loop: the mechanical time constant through
  * a 131.3:1 gearbox is far slower than that. Do not confuse this with the
  * CURRENT loop, which will need 5-10 kHz once the DRV8874 arrives, because
  * the electrical time constant (L/R = 1.70 mH / 1.87 ohm = 0.90 ms) is faster
  * than a single tick.
  *
  * Widening the window trades resolution against lag. If W5's PID needs
  * faster velocity updates than 50 Hz, shorten the window and expect coarser
  * numbers rather than reaching for a filter first.
  *
  *
  * CONCURRENCY
  * -----------
  * encoder_on_tick() runs in the TIM6 interrupt. The accumulator is 64-bit and
  * a Cortex-M4 cannot read it atomically, so encoder_position() and anything
  * derived from it briefly mask interrupts. The masked region is a few
  * instructions and nests correctly (PRIMASK is saved and restored, not
  * blindly re-enabled).
  *
  ******************************************************************************
  */
#ifndef ENCODER_H
#define ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/** @brief Counts per revolution of the motor shaft, x4 quadrature decoding. */
#define ENCODER_COUNTS_PER_MOTOR_REV   64.0f

/** @brief Gearbox reduction. Rounded by the vendor; the real train is not
  *        a clean ratio, which is a W9 calibration problem. */
#define ENCODER_GEAR_RATIO             131.3f

/** @brief Counts per revolution of the OUTPUT shaft — 8403.2. The number the
  *        W4 hand-rotation test checks against. */
#define ENCODER_COUNTS_PER_OUTPUT_REV  (ENCODER_COUNTS_PER_MOTOR_REV * ENCODER_GEAR_RATIO)

/** @brief Default velocity window, in ticks. 20 at a 1 kHz tick gives a 50 Hz
  *        update and 0.36 rpm resolution. */
#define ENCODER_VELOCITY_WINDOW_DEFAULT  20u

/**
  * @brief  Start the quadrature counter and zero the accumulator.
  *
  * TIM2 is already configured and started by MX_TIM2_Init(); this seeds the
  * previous-count reference so the first tick does not report a spurious
  * delta, and clears the derived state.
  */
void encoder_init(void);

/**
  * @brief  Sample the counter and accumulate. Call once per control tick.
  *
  * Runs in the TIM6 ISR. Reads one register, does integer arithmetic, and
  * returns — no division, no floating point, nothing that blocks.
  */
void encoder_on_tick(void);

/** @brief Zero the accumulated position and the tick counter. Does not touch
  *        `TIM2->CNT`, so no counts are lost across the call. */
void encoder_zero(void);

/** @brief Accumulated position in counts since the last zero. Signed: negative
  *        means the shaft has net-rotated the other way. */
int64_t encoder_position(void);

/** @brief Counts accumulated during the most recent tick. Signed. */
int32_t encoder_last_delta(void);

/** @brief Accumulated position expressed in revolutions of the OUTPUT shaft. */
float encoder_revolutions(void);

/**
  * @brief  Output-shaft speed in rpm, averaged over the velocity window.
  *
  * Signed — negative is the other direction. Updated once per window, so
  * repeated calls between updates return the same value rather than
  * recomputing from a partial window.
  */
float encoder_rpm(void);

/** @brief Raw `TIM2->CNT`. Diagnostics only — never treat it as a position. */
uint32_t encoder_raw_count(void);

/** @brief Direction of the last hardware count, from TIM2's DIR bit.
  *        true = counting up. */
bool encoder_counting_up(void);

/** @brief Ticks since the last zero. Confirms the TIM6 tick is actually
  *        running, which a stationary encoder cannot tell you on its own. */
uint32_t encoder_tick_count(void);

/**
  * @brief  Set the velocity averaging window, in ticks.
  * @param  ticks  clamped to 1..1000. See the resolution table in the header
  *                comment before shortening it.
  */
void encoder_set_velocity_window(uint16_t ticks);

/** @brief Current velocity window, in ticks. */
uint16_t encoder_velocity_window(void);

/** @brief Result of encoder_probe(). */
typedef struct
{
  uint32_t edges_a;     /*!< transitions seen on channel A during the window  */
  uint32_t edges_b;     /*!< transitions seen on channel B                    */
  bool     level_a;     /*!< channel A level at the end of the window         */
  bool     level_b;     /*!< channel B level                                  */
  int32_t  count_delta; /*!< how far TIM2 moved during the same window        */
  uint32_t samples;     /*!< how many times the pins were read                */
} encoder_probe_t;

/**
  * @brief  Watch the raw A/B pins and TIM2 together for @p ms milliseconds.
  *
  * Exists to split one symptom into two very different faults. Reading the
  * GPIO input register works even though PA15/PB3 are in alternate-function
  * mode, so this observes the actual wire without disturbing the timer:
  *
  *   no edges on either channel  -> the signal never reaches the MCU
  *                                  (encoder power, ground, wiring, pull-ups,
  *                                  or the shaft is not really turning)
  *   edges but count_delta == 0  -> the signal is arriving and TIM2 is not
  *                                  decoding it (AF, timer config, or only
  *                                  one channel connected)
  *
  * @note  BLOCKS for @p ms. Bench diagnostic only — never call it from the
  *        control loop. The CAN heartbeat and the MKS state machine are not
  *        serviced while it runs.
  */
void encoder_probe(uint32_t ms, encoder_probe_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */

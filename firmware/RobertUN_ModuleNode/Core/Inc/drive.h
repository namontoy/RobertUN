/**
  ******************************************************************************
  * @file           : drive.h
  * @brief          : Drive-motor H-bridge control — TIM4 PWM + nSLEEP + nFAULT
  ******************************************************************************
  *
  * WHAT THIS IS FOR
  * ----------------
  * Owns the four pins that reach the motor driver, and nothing else. No PID,
  * no ramping, no policy — those belong above this layer, so that W5 can
  * change how the motor is commanded without touching how the bridge is
  * driven.
  *
  *     PB6  TIM4_CH1  ->  IN1 (+ IN3, paralleled)
  *     PB7  TIM4_CH2  ->  IN2 (+ IN4, paralleled)
  *     PB5  GPIO out  ->  nSLEEP     low = bridge disabled
  *     PB12 GPIO in   ->  nFAULT     open-drain, active low
  *
  *
  * DRIVER-AGNOSTIC BY DESIGN
  * -------------------------
  * The DRV8833 carrier in use today and the DRV8874 arriving ~Sep 15 share the
  * same IN1/IN2 truth table, the same active-low nSLEEP with a 1 ms wake, and
  * the same open-drain active-low nFAULT. Nothing in this module distinguishes
  * them, so the swap is a wiring change and a PMODE strap, not a firmware
  * change.
  *
  *
  * THE TRUTH TABLE, AND WHY BOTH DECAY MODES EXIST
  * -----------------------------------------------
  *     IN1  IN2   result
  *      0    0    coast   — outputs Hi-Z, current decays through the diodes
  *      1    0    forward
  *      0    1    reverse
  *      1    1    brake   — both outputs low, current recirculates
  *
  * Vendor documentation for the DRV8833 carrier calls both 00 and 11 "motor
  * off". They are not the same, and the difference is the whole reason this
  * module has a decay setting:
  *
  *   FAST decay (sign-magnitude) — PWM between drive and COAST.
  *     forward: IN1 = PWM(D), IN2 = 0
  *     Current falls quickly during the off phase, so duty maps less linearly
  *     onto speed at low duty, and ripple is larger.
  *
  *   SLOW decay (drive-brake) — PWM between drive and BRAKE.
  *     forward: IN1 = 1 (always), IN2 = PWM inverted, high for (1 - D)
  *     Current is held up by the shorted winding during the off phase. Better
  *     duty-to-speed linearity at low duty and smaller ripple, which is what a
  *     velocity PID wants. This is the default.
  *
  * Both are reachable because PB6 and PB7 are two independent PWM channels on
  * one timer. That was the reason for choosing two PWM outputs over a
  * PWM+direction pair, and it costs nothing to keep the choice open until W5
  * measures which behaves better.
  *
  *
  * ZERO DUTY IS COAST, IN BOTH MODES
  * ---------------------------------
  * Setting duty 0 in slow-decay mode would, taken literally, mean 100% brake —
  * IN1 and IN2 both high, motor actively held. That is a surprising thing for
  * "stop" to do on a vehicle, so duty 0 always produces coast (00) regardless
  * of decay mode. Call drive_brake() to actually brake.
  *
  *
  * DUTY IS SIGNED PER-MILLE
  * ------------------------
  * -1000..+1000, sign selecting direction. Integer, so the control path never
  * needs floating point, and the resolution (0.1%) is finer than the timer's
  * 4500 steps can resolve anyway. Values outside the range are clamped, not
  * wrapped.
  *
  *
  * nSLEEP NEEDS 1 ms
  * -----------------
  * Both drivers take up to 1 ms to wake. drive_enable() therefore BLOCKS for
  * 2 ms and must not be called from the control loop — enable once at start-up
  * or from the console, then command duty freely.
  *
  *
  * WHAT THIS MODULE DELIBERATELY DOES NOT DO
  * -----------------------------------------
  * It does not slew-limit, and it does not react to nFAULT. A step from 0 to
  * full duty draws stall current (5.0 A at the 9.5 V rail) because back-EMF is
  * zero at t=0 — that is a fact about the motor, and the decision about what to
  * do with it belongs to the control layer in W5, not here. drive_faulted() is
  * provided so that layer can act; this module never disables itself behind the
  * caller's back.
  *
  ******************************************************************************
  */
#ifndef DRIVE_H
#define DRIVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/** @brief Full scale for the signed per-mille duty. */
#define DRIVE_DUTY_MAX  1000

/** @brief PWM carrier frequency, Hz — TIM4 at 90 MHz, PSC 0, ARR 4499.
  *        Chosen so that L/R = 0.90 ms is ~18x the 50 us period, which holds
  *        current ripple near 99 mA peak-to-peak. */
#define DRIVE_PWM_HZ    20000u

typedef enum
{
  DRIVE_DECAY_SLOW = 0,  /*!< drive-brake. Better low-duty linearity. Default */
  DRIVE_DECAY_FAST       /*!< sign-magnitude, PWM against coast               */
} drive_decay_t;

/**
  * @brief  Bring the driver up in its off state.
  *
  * Leaves nSLEEP low and both inputs low. Call after MX_TIM4_Init(); safe to
  * call again at any time.
  */
void drive_init(void);

/**
  * @brief  Raise nSLEEP and wait for the driver to wake.
  * @note   BLOCKS for 2 ms. Never call from the control loop.
  */
void drive_enable(void);

/** @brief Drop duty to zero, then drop nSLEEP. Order matters: the inputs reach
  *        a defined level before the bridge is torn down. */
void drive_disable(void);

/** @brief Whether nSLEEP is currently asserted high. */
bool drive_is_enabled(void);

/**
  * @brief  Command duty and direction.
  * @param  permille  -1000..+1000; clamped to that range and then to the
  *                   configured limit. 0 always produces coast.
  * @note   Does not touch nSLEEP. A non-zero duty with the driver disabled
  *         sets the pins and moves nothing, which is intentional — it lets the
  *         waveform be scoped with the bridge safely off.
  */
void drive_set_duty(int16_t permille);

/** @brief Last commanded duty, after clamping. */
int16_t drive_duty(void);

/** @brief Both inputs high — actively brake. Duty reads back as 0. */
void drive_brake(void);

/** @brief Both inputs low — coast, outputs Hi-Z. Equivalent to duty 0. */
void drive_coast(void);

/** @brief True while nFAULT is pulled low (over-current, over-temperature or
  *        under-voltage). Note that a HIGH reading does not prove the driver
  *        is alive: with VM absent the pin floats up through the pull-up. */
bool drive_faulted(void);

/** @brief Select fast or slow decay. Takes effect on the next duty command. */
void drive_set_decay(drive_decay_t decay);

/** @brief Current decay mode. */
drive_decay_t drive_decay(void);

/**
  * @brief  Cap the magnitude of any commanded duty.
  * @param  permille  0..1000. Applied after the range clamp, so a limit of 400
  *                   turns a command of +1000 into +400.
  *
  * Exists for bench work on the interim DRV8833 carrier, whose paralleled peak
  * is 4 A against a 5.0 A stall at the 9.5 V rail. A limit of 400 keeps stall
  * current near 2 A. Defaults to no limit; set it deliberately.
  */
void drive_set_limit(uint16_t permille);

/** @brief Current duty limit, per-mille. */
uint16_t drive_limit(void);

#ifdef __cplusplus
}
#endif

#endif /* DRIVE_H */

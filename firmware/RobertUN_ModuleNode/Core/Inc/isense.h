/**
  ******************************************************************************
  * @file           : isense.h
  * @brief          : Drive current — measurement (PA2/IPROPI) and limit (PA4/VREF)
  ******************************************************************************
  *
  * WHAT THIS IS FOR
  * ----------------
  * One module for one physical quantity. It turns an ADC reading into
  * milliamps, and it sets the current at which the DRV8874 starts regulating
  * on its own. Those two live together because they are the SAME arithmetic —
  * both are a voltage divided by A_IPROPI x R_IPROPI — and splitting them
  * across two modules would mean two copies of the constant that matters most.
  *
  *
  * THE CARRIER WAS MODIFIED. THIS IS NOT A STOCK BOARD.
  * ----------------------------------------------------
  * Two changes were made by hand on Sep 12, 2026, to the one carrier on the
  * bench (seven spares remain stock):
  *
  *   1. The 10 kOhm between nSLEEP and VREF was REMOVED. VREF is now an
  *      independent input, driven by the MCU's DAC on PA4.
  *   2. R_IPROPI was changed from 2.48 kOhm to 2.0 kOhm || 5.6 kOhm = 1.474 k.
  *
  * Both were done for the same reason: the motor stalls at 5.0 A on the 9.5 V
  * rail, and the stock carrier could neither measure nor permit that. The
  * modification is small, reversible, and the spares are untouched.
  *
  *
  * THE SCALING
  * -----------
  * IPROPI sources a current proportional to the bridge current:
  *
  *     I_IPROPI = I_OUT x A_IPROPI        A_IPROPI = 450 uA/A on the DRV8874
  *
  * R_IPROPI turns that into a voltage:
  *
  *     scale      = 450e-6 x 1474  = 0.6632 V/A
  *     full scale = 3.3 / 0.6632   = 4.975 A      <- ADC ceiling
  *     one LSB    = 4975 / 4096    = 1.215 mA
  *
  * A_IPROPI is 1000 uA/A on the DRV8876 second source, so a swap changes this
  * constant by 2.22x. It is not a transparent substitution.
  *
  * The R_IPROPI default below is the NOMINAL parallel value. If the pair reads
  * differently on a meter, change that one number — `cfg r_ipropi <ohms>` on a
  * live board, no rebuild — because everything else derives from it, and a 1%
  * error there is a 1% error in every current ever logged.
  *
  *
  * WHAT REMOVING THE 10 kOhm BOUGHT, AND WHAT IT COST
  * ---------------------------------------------------
  * On the stock carrier VREF followed nSLEEP, which meant the trip point and
  * the ADC full scale were the same number and could not be moved apart. They
  * are now independent:
  *
  *     R_IPROPI alone sets the CEILING       = 4.975 A, fixed
  *     VREF sets the TRIP anywhere from 0 up to that ceiling, in software
  *
  *     I_TRIP = V_VREF / (A_IPROPI x R_IPROPI)
  *
  * One DAC code moves the trip by 1.215 mA — exactly one ADC LSB, because both
  * converters are 12 bits across the same 3.3 V through the same resistor. The
  * limit and the measurement have the same grain, which is a tidy place to be.
  *
  * THE COST IS THAT THE FAIL-SAFE IS GONE. The 10 kOhm guaranteed VREF could
  * never be wrong while nSLEEP was high; they moved together. Now VREF must be
  * SET BEFORE nSLEEP RISES, every time, including after any reset. isense_init()
  * does this at boot, before anything can command the driver — and if the DAC
  * were somehow not running, the trip is 0 and the motor simply will not turn.
  * That is the safe direction to fail, and it is deliberate.
  *
  *
  * WHY THE DEFAULT TRIP IS 3.0 A AND NOT THE CEILING
  * --------------------------------------------------
  * The stock carrier gave ~2.96 A, and every bench result on record was taken
  * with that limit in place. Booting at 3.0 A keeps that behaviour unchanged,
  * so nothing silently gets more dangerous because a resistor was lifted.
  * Reaching full capacity is then an explicit act: `drv trip 4600`.
  *
  *
  * THE OUTPUT BUFFER COSTS YOU THE TOP OF THE RANGE
  * -------------------------------------------------
  * The F446's DAC output buffer is enabled by default here: it has a low
  * output impedance and will drive VREF regardless of what the pin's input
  * impedance turns out to be. The price is that a buffered output cannot reach
  * either rail — roughly 0.2 V to VDDA-0.2 V:
  *
  *     buffered    0.200 .. 3.100 V  ->  trip  0.30 .. 4.67 A
  *     unbuffered  0.000 .. 3.300 V  ->  trip  0.00 .. 4.98 A
  *
  * 4.67 A is BELOW the 4.92 A the motor draws stalled at the measured 9.35 V
  * motor terminal voltage. So with the buffer on, a genuine stall regulates
  * rather than being measured. That is safe, and it is also not what a
  * full-capacity test is trying to find out — hence isense_set_vref_buffered(),
  * and `drv trip buf off`, to reclaim the top 0.31 A once VREF's input
  * impedance is known to be high enough to leave unbuffered. Check that with a
  * meter on the pin before trusting an unbuffered setting: if the commanded
  * and measured VREF disagree, the buffer belongs back on.
  *
  *
  * THE PLATEAU TEST IS NOW A REAL TEST, NOT AN INFERENCE
  * -----------------------------------------------------
  * I_TRIP above assumes V_IPROPI is compared against VREF directly, with no
  * internal divider. On the stock carrier that could only be guessed at from
  * where a stall happened to land. With VREF under software control it can be
  * measured properly: SET A KNOWN TRIP, STALL THE MOTOR, AND SEE WHERE THE
  * READING PLATEAUS.
  *
  *     drv trip 2000, stall, plateau at ~2000 mA  ->  k = 1, as assumed
  *     drv trip 2000, stall, plateau at ~1000 mA  ->  k = 2, halve every trip
  *     drv trip 2000, stall, plateau at ~667 mA   ->  k = 3
  *
  * Sweep it — 1000, 2000, 3000 — and the relationship should be a straight
  * line through the origin. If it is, the scaling is confirmed end to end, the
  * ADC and the DAC agree, and every number this module reports is trustworthy.
  * That is the single most valuable hour available on this bench right now.
  *
  *
  * WHAT THE READING MEANS - SETTLED ON THE BENCH 2026-09-12
  * ---------------------------------------------------------
  * In slow decay (drive-brake, the chosen scheme) the bridge draws from VM for
  * only D of each 50 us period; the rest of the time current recirculates
  * through the low-side FETs. With the carrier's 20 kOhm IMODE strap, IPROPI is
  * BLANKED during that recirculation. Therefore:
  *
  *   isense_read_ma() returns SUPPLY current - that is, I_motor x D.
  *
  * Measured, not inferred. Stalling the output shaft removes back-EMF, so the
  * motor current is pure Ohm's law and both hypotheses predict a number with no
  * friction model in the way:
  *
  *   20% duty, stalled, Vm 9.35 V, R_motor 1.90 ohm
  *     motor current  = 0.20 x 9.35 / 1.90 = 984 mA   (if continuous)
  *     supply current = 984 x 0.20         = 197 mA   (if drive-phase only)
  *     MEASURED (256-sample average, twice) = 189, 190 mA
  *
  * A 5x discriminator landing within 4% of the supply prediction. The residual
  * is the bridge's own RDS(on) - about 0.16 ohm across the two conducting FETs,
  * so ~0.15 V of the rail never reaches the motor - plus the ~1.4% this module
  * currently reads low from vdda_mv and r_ipropi both being uncorrected. Together those close the gap to ~2.5%.
  *
  * TWO CONSEQUENCES, AND THE SECOND ONE BITES
  *
  * 1. The DRV8874 regulates by comparing the INSTANTANEOUS IPROPI voltage to
  *    VREF, cycle by cycle. Since IPROPI mirrors the drive phase, the quantity
  *    being regulated is true motor current during drive. So the trip does what
  *    you want: `drv trip 3000` really does limit the motor to 3 A.
  *
  * 2. But this module reports the duty-averaged SUPPLY figure, so the trip and
  *    the reading are in different units. During a plateau sweep the reported
  *    current does NOT plateau at the trip value - it plateaus at
  *
  *      trip x D_regulation,  where D_regulation = trip x R_motor / Vm
  *
  *    i.e. at trip^2 x R_motor / Vm, quadratic in the trip. Read a plateau as
  *    though it were the trip itself and the VREF divider will look wrong when
  *    it is not.
  *
  * Callers should keep recording duty alongside the reading regardless -
  * drive_duty() is right there, and `drv current` already prints both.
  *
  *
  * AND THE FREE-RUNNING AVERAGE TURNED OUT TO ALIAS - 2026-09-14
  *
  * >> PARTLY RETRACTED 2026-09-15. Read this whole section with that in mind.
  * >> The aliasing story was built on three low-count readings and does not
  * >> survive the Sep 12 stall test, where this same free-running sampler
  * >> returned 189 and 190 mA on repeat - 0.5%, which a badly-aliasing sampler
  * >> cannot do. The real pattern is steady at high signal, scattered at low
  * >> signal. What IS now established, by sweeping the ADC trigger across the
  * >> period with `drv iscan`: IPROPI reads exactly 0 everywhere outside the
  * >> drive phase, and inside it has reproducible structure - three bumps
  * >> separated by hard zeros within 6.5 us at 13% duty, peaking near 1.1 A.
  * >> So single-point synchronised sampling cannot work here either, and the
  * >> cause of that structure is UNKNOWN pending a scope on PA2. Do not build
  * >> on either story until that measurement exists.
  * -------------------------------------------------------------
  * Everything above is still true of isense_read_avg(). What it missed is that a
  * software loop is not a random sampler. isense_read_raw() runs the same
  * instruction path every call - Start, poll, Stop - so it samples at a very
  * nearly FIXED period, against a PWM carrier that is exactly periodic. That is
  * the textbook aliasing condition: the samples visit a handful of phases of the
  * 50 us period and stay there, however many of them you take.
  *
  * It showed up on the loaded-wheel rig as readings that were not merely noisy but
  * impossible:
  *
  *     12% duty   raw 16     Isup  19 mA     plausible
  *     13% duty   raw  1     Isup   1 mA     while turning a 1 kg wheel
  *     14% duty   raw 12     Isup  14 mA     plausible
  *
  * raw 16 at 12% is exactly 0.12 x 133, the drive-phase peak - so when the samples
  * DO spread across the period, the supply-current model above holds precisely.
  * raw 1 at 13% means essentially no sample landed in the 6.5 us driven window at
  * all. The single stray count is most likely one iteration displaced by the
  * SysTick or TIM6 interrupt.
  *
  * THE DIAGNOSTIC TRAP IS THAT AVERAGING HARDER LOOKS LIKE IT SHOULD HELP AND
  * CANNOT. Immunity to averaging was briefly read as evidence that the scatter was
  * real load modulation. It is the opposite: it is the signature of a sampler
  * locked to the waveform. Anything concluded from free-running current readings
  * before this date should be re-taken.
  *
  *
  * SO THERE ARE TWO READING PATHS, AND THEY RETURN DIFFERENT QUANTITIES
  * --------------------------------------------------------------------
  *   isense_read_avg()      free-running. SUPPLY current, I_motor x D, and aliases
  *                          as described. Kept for isense_zero(), which runs with
  *                          the bridge off and no carrier to alias against, and as
  *                          the fallback below ~4% duty.
  *
  *   isense_read_sync_avg() triggered from TIM4_CH4 at the middle of the drive
  *                          phase. IPROPI is live there, so this is MOTOR current,
  *                          measured directly.
  *
  * The synchronised path is better than the old one by more than it removes the
  * aliasing:
  *
  *   - No divide by D. Recovering motor current from a supply reading amplifies
  *     every error by 1/D, which at 10% duty is a factor of ten. That is why the
  *     inferred figures wandered between 50 and 390 mA at neighbouring duties.
  *   - Eight times the resolution where it is needed. At 12% duty the free average
  *     reads 16 counts of 4096; the synchronised sample reads ~133. The rover
  *     operates near its stiction floor, which is exactly where the old path was
  *     weakest.
  *   - The trip and the reading finally share units. The DRV8874 regulates on
  *     instantaneous IPROPI against VREF - motor current - so a plateau in a
  *     synchronised reading can be compared with the commanded trip directly,
  *     instead of through the quadratic correction two sections above.
  *
  * The cost is time: each sample waits for the next compare event, so one
  * conversion costs a whole 50 us period rather than ~7 us. 64 samples is 3.2 ms.
  * That is why the sync default is 64 and not 1024 - past a point the extra
  * samples are averaging the wheel, not the converter.
  *
  *
  * SAMPLING TIME IS 28 CYCLES ON PURPOSE
  * --------------------------------------
  * At PCLK2/4 = 22.5 MHz a 28-cycle sample plus 12 cycles of conversion is
  * 1.78 us, which fits inside the PWM on-phase even at 25% duty (12.5 us).
  * The 480-cycle maximum would take 21.9 us and straddle half the period,
  * making PWM-synchronised sampling impossible without returning to CubeMX.
  * 28 cycles is still about 4x what a 1.47 kOhm source needs to settle to
  * 12 bits, so nothing was given up to keep that door open.
  *
  * The door was opened on 2026-09-14, a phase earlier than planned, because the
  * aliasing above forced it: the ADC is triggered from TIM4_CH4's compare event.
  * TIM4 already generates the PWM, and a compare channel raises its event with no
  * pin configured - TIM4_CH3/CH4 land on PB8/PB9, which are the CAN pins, but the
  * internal event does not need them, and PB9 stays on AF9 throughout. drive.c
  * places the trigger, because drive.c owns the timer and is the only thing that
  * knows which END of the period is the driven one; this module arms the ADC and
  * reads it. Had the sampling time been left at CubeMX's default the aperture
  * would have been 21.9 us against a 6.5 us window, and none of this would have
  * been reachable without returning to the .ioc.
  *
  *
  * THERMAL, BECAUSE 5 A IS THE POINT OF ALL THIS
  * ----------------------------------------------
  * 5 A through the DRV8874's 200 mOhm is 5 W in an HTSSOP-16 sitting on
  * whatever copper the carrier gives it. The 6 A figure is a peak rating, not
  * a thermal budget. Stall tests are SECONDS, with a hand on the package
  * between runs. The loaded-wheel rig makes this easy to forget: a wheel
  * turning slowly under load looks like nothing much is happening.
  *
  ******************************************************************************
  */
#ifndef ISENSE_H
#define ISENSE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ---------------------------------------------------------------------------
   THESE ARE DEFAULTS, NOT CONSTANTS
   ---------------------------------------------------------------------------
   Each of the four below describes a specific soldered board, not the design,
   and each now lives in FLASH as a config key (see config.h) that the console
   can change without a rebuild. What remains here is the value a blank board
   boots with and what `cfg default` restores — so the REASONING stays next to
   the hardware it is about, which is why they were not simply moved.

   The _DEFAULT suffix is deliberate: it makes reading one of these at runtime,
   where config_get() was meant, read wrong at the call site.
   --------------------------------------------------------------------------- */

/** @brief ADC and DAC reference, mV. VDDA on this board. Live: CFG_VDDA_MV. */
#define ISENSE_VDDA_MV_DEFAULT           3300u

/** @brief IPROPI sense resistor, ohms. MODIFIED CARRIER: 2.0k || 5.6k, the
  *        stock 2.48k having been removed. Nominal parallel value — measure
  *        the pair and set CFG_R_IPROPI_OHM if the meter disagrees, which on
  *        the first board it does: 1465 measured. */
#define ISENSE_R_IPROPI_OHM_DEFAULT      1474u

/** @brief A_IPROPI in uA per A. 450 on the DRV8874, 1000 on the DRV8876 —
  *        changing the part means changing this, and the scale moves 2.22x.
  *        Live: CFG_A_IPROPI_UA_PER_A. */
#define ISENSE_A_IPROPI_UA_PER_A_DEFAULT 450u

/** @brief Raw counts at or above which the ADC itself is clipping. Distinct
  *        from regulating — the bridge regulates at the TRIP, which is now
  *        usually well below this. Live: CFG_ISENSE_SAT_RAW.
  *        See isense_saturated(). */
#define ISENSE_SATURATED_RAW_DEFAULT     4050u

/** @brief Trip set at boot, mA. Matches what the stock carrier enforced, so
  *        lifting the 10k did not quietly make anything more dangerous.
  *        Live: CFG_TRIP_BOOT_MA. */
#define ISENSE_TRIP_DEFAULT_MA           3000u

/** @brief Buffered DAC headroom from each rail, mV. F446 datasheet figure. */
#define ISENSE_VREF_BUF_MARGIN_MV 200u

/** @brief Default number of conversions averaged by isense_read_ma(). At
  *        1.78 us each this is 57 us — just over one PWM period, so a
  *        free-running average covers the whole cycle rather than a slice. */
#define ISENSE_AVG_DEFAULT        32u

/** @brief Narrowest drive phase, in TIM4 ticks, that a synchronised sample will
  *        be taken inside. The 28-cycle aperture is 1.24 us = 112 ticks at
  *        90 MHz; 192 leaves ~40 ticks of clearance at each edge and still
  *        admits any duty at or above 4.3%. The loaded rig stalls below 10%, so
  *        nothing it can actually sustain comes near this floor. */
#define ISENSE_SYNC_MIN_TICKS    192u

/** @brief Conversions averaged by the synchronised path when the caller does
  *        not say. Each costs a whole PWM period because it waits for the next
  *        trigger, so 64 is 3.2 ms - long enough to average commutation ripple,
  *        short enough to feel instant at the console. */
#define ISENSE_SYNC_AVG_DEFAULT   64u

/**
  * @brief  Prepare the module, start the DAC, and apply the default trip.
  * @note   Call after MX_ADC1_Init() and MX_DAC_Init(), and BEFORE anything
  *         can raise nSLEEP. Setting VREF is now a boot-order requirement,
  *         not a convenience — see the header.
  */
void isense_init(void);

/**
  * @brief  One conversion, polled.
  * @return Raw 12-bit count, or 0 if the conversion failed.
  * @note   Blocks ~1.78 us plus HAL overhead.
  */
uint16_t isense_read_raw(void);

/**
  * @brief  Average of @p samples conversions, offset-corrected.
  * @param  samples  1..1024; 0 selects ISENSE_AVG_DEFAULT.
  * @return Raw counts, averaged, with the measured zero offset subtracted.
  */
uint16_t isense_read_avg(uint16_t samples);

/**
  * @brief  Drive current in milliamps.
  * @param  samples  averaging depth; 0 selects ISENSE_AVG_DEFAULT.
  * @return mA. Whether this is supply or motor current is the open question
  *         in the header — record drive_duty() alongside it.
  */
uint32_t isense_read_ma(uint16_t samples);

/**
  * @brief  Whether a synchronised sample can be taken right now.
  * @retval true   the drive phase is at least ISENSE_SYNC_MIN_TICKS wide
  * @retval false  duty is 0, the bridge is braking, or the phase is too narrow
  *                for the sampling aperture to sit inside it
  * @note   Ask before reading rather than interpreting a 0 afterwards - a
  *         genuine 0 mA and a refusal are the same number.
  */
bool isense_sync_ready(void);

/**
  * @brief  Average of @p samples conversions taken at the middle of the PWM
  *         drive phase, offset-corrected.
  * @param  samples  1..1024; 0 selects ISENSE_SYNC_AVG_DEFAULT.
  * @return Raw counts, or 0 if isense_sync_ready() is false.
  * @note   Costs one PWM period (50 us) per sample, not 1.78 us - it waits for
  *         a trigger rather than starting one.
  */
uint16_t isense_read_sync_avg(uint16_t samples);

/**
  * @brief  MOTOR current in milliamps, measured rather than inferred.
  * @param  samples  averaging depth; 0 selects ISENSE_SYNC_AVG_DEFAULT.
  * @return mA, or 0 if no synchronised sample was possible.
  * @note   This is the quantity the DRV8874's trip regulates, so the two are
  *         finally in the same units. Supply current, if a power budget wants
  *         it, is this times the duty - a multiply, not the 1/D divide the old
  *         path needed.
  */
uint32_t isense_read_motor_ma(uint16_t samples);

/**
  * @brief  Average n conversions taken at one fixed tick of the PWM period.
  * @param  tick     where in the 0..4499 period to sample
  * @param  samples  how many periods to average over
  * @retval raw ADC counts, NO offset subtracted
  *
  * DIAGNOSTIC ONLY - the instrument for mapping what IPROPI actually does
  * across a period, rather than reasoning about what it ought to do. Sweeping
  * tick across the whole period plots the waveform the synchronised reader is
  * trying to sample, which is the only way to tell a mis-placed trigger from a
  * mirror that cannot settle inside the drive window from a signal that was
  * never there. Restores the normal trigger placement before returning.
  */
uint16_t isense_read_sync_at(uint16_t tick, uint16_t samples);

/**
  * @brief  Convert a raw count to milliamps.
  * @note   Exact integer form of raw x 3.3 / 4096 / 0.6632. The multiply peaks
  *         at 4095 x 4975 = 20.4e6, comfortably inside uint32.
  */
uint32_t isense_raw_to_ma(uint16_t raw);

/**
  * @brief  Current at raw 4095, mA — 4975 with the resistor as fitted. The ADC
  *         ceiling, and the highest trip the DAC can ask for. No longer the
  *         same as the trip point: the 10k that made them equal is gone.
  * @note   A function rather than the #define it used to be, because it is
  *         computed from three config keys now and must follow them when they
  *         change at runtime.
  */
uint16_t isense_full_scale_ma(void);

/**
  * @brief  Measure and store the zero offset.
  * @note   Call with the driver DISABLED and the motor still. A few counts of
  *         standing offset is several mA of lie at every operating point.
  * @return The offset stored, in raw counts.
  */
uint16_t isense_zero(void);

/** @brief The stored zero offset, raw counts. */
uint16_t isense_offset(void);

/**
  * @brief  Whether the last reading reached the saturation threshold.
  * @note   This is the ADC clipping near 4.98 A, which after the carrier
  *         modification is a DIFFERENT event from the bridge regulating.
  *         Regulation shows up as a plateau at isense_trip_ma(); clipping
  *         shows up here. Reaching both at once means the trip is at the
  *         ceiling.
  */
bool isense_saturated(void);

/* --- current limit: VREF on PA4 / DAC1_OUT1 ------------------------------ */

/**
  * @brief  Set the current-regulation trip point.
  * @param  ma  requested trip, milliamps. Clamped to the achievable range,
  *             which depends on whether the output buffer is enabled.
  * @retval true   the request was met (within one DAC code)
  * @retval false  the request was clamped; isense_trip_ma() says to what
  * @note   Safe to call while the bridge is live — the DAC output moves
  *         immediately and the driver follows it.
  */
bool isense_set_trip_ma(uint32_t ma);

/**
  * @brief  The trip actually in force, mA.
  * @note   Derived back from the DAC code that was written, not from the value
  *         requested, so quantisation and clamping are both visible.
  */
uint32_t isense_trip_ma(void);

/** @brief VREF actually commanded, mV — the same number, before the divide. */
uint16_t isense_vref_mv(void);

/** @brief Raw 12-bit DAC code currently in the output register. */
uint16_t isense_vref_code(void);

/** @brief Highest trip reachable right now, mA. 4.67 A buffered, 4.98 A not. */
uint32_t isense_trip_max_ma(void);

/** @brief Lowest non-zero trip reachable right now, mA. 0.30 A buffered. */
uint32_t isense_trip_min_ma(void);

/**
  * @brief  Enable or disable the DAC output buffer, preserving the trip.
  * @param  on  true = buffered (safe default, ceiling 4.67 A)
  *             false = unbuffered (ceiling 4.98 A, needs a high-impedance
  *             VREF pin — verify with a meter, see the header)
  * @note   Re-applies the current trip afterwards, re-clamping if the new
  *         range cannot hold it.
  */
void isense_set_vref_buffered(bool on);

/** @brief Whether the DAC output buffer is enabled. */
bool isense_vref_buffered(void);

/**
  * @brief  The DAC code a given trip request would end up writing.
  * @note   Exists so a caller can ask "would a reset change the trip?" exactly.
  *         Comparing milliamps cannot answer it: isense_trip_ma() is derived
  *         back from the code and is therefore always quantised, while a stored
  *         config value is not, so the two are almost never equal even when
  *         they mean the same thing. Codes are the only space where the
  *         question has a yes/no answer.
  */
uint16_t isense_code_for_trip_ma(uint32_t ma);

/**
  * @brief  Convert a trip current to the VREF voltage that produces it.
  * @param  ma  milliamps
  * @return millivolts. Unclamped — the callers do the clamping.
  * @note   Was inline; now a function, because A_IPROPI and R_IPROPI are
  *         config keys and an inline would have baked in the defaults.
  */
uint32_t isense_ma_to_vref_mv(uint32_t ma);

/**
  * @brief  Convert a VREF voltage to the trip current it produces.
  * @param  mv  millivolts
  * @return milliamps.
  */
uint32_t isense_vref_mv_to_ma(uint32_t mv);

#ifdef __cplusplus
}
#endif

#endif /* ISENSE_H */

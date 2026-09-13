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
  * ISENSE_R_IPROPI_OHM below is the NOMINAL parallel value. If the pair reads
  * differently on a meter, change that one number — everything else derives
  * from it, and a 1% error there is a 1% error in every current ever logged.
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
  * WHAT THE READING MEANS IS STILL AN OPEN QUESTION - READ THIS
  * ------------------------------------------------------------
  * In slow decay (drive-brake, the chosen scheme) the bridge draws from VM for
  * only D of each 50 us period; the rest of the time current recirculates
  * through the low-side FETs. Whether IPROPI reports during that recirculation
  * decides what this module returns:
  *
  *   reports during drive only  -> the average is SUPPLY current, I_motor x D
  *   reports continuously       -> the average is MOTOR current
  *
  * They differ by a factor of D. At 50% duty one is half the other, so this is
  * not a correction to apply later - it is which quantity is on the pin.
  *
  * The IMODE strap selects this behaviour and the carrier fits 20 kOhm to GND;
  * the datasheet's IMODE table says which mode that is. It has NOT been
  * verified here. Two bench tests settle it faster than the datasheet:
  *
  *   - Scope the IPROPI pin. A 20 kHz square wave means drive-phase only; a
  *     near-DC level means continuous. Ten seconds, and unambiguous.
  *   - Or put a DC ammeter in the VM lead at ~50% duty under load and compare.
  *     The two hypotheses differ by 2x, which cannot be misread.
  *
  * Until it is settled, isense_read_ma() returns what the ADC sees and the
  * caller records the duty alongside it. drive_duty() is right there, and
  * `drv current` already prints both.
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
  * The door leads here, when W5 wants it: trigger the ADC from TIM4_CH4's
  * compare event. TIM4 already generates the PWM, and a compare channel raises
  * its event with no pin configured - TIM4_CH3/CH4 land on PB8/PB9, which are
  * the CAN pins, but the internal event does not need them. That puts the
  * sample at a chosen point in the on-phase and retires the question above.
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

/** @brief ADC and DAC reference, mV. VDDA on this board. */
#define ISENSE_VDDA_MV            3300u

/** @brief IPROPI sense resistor, ohms. MODIFIED CARRIER: 2.0k || 5.6k, the
  *        stock 2.48k having been removed. Nominal parallel value — measure
  *        the pair and correct this if the meter disagrees. */
#define ISENSE_R_IPROPI_OHM       1474u

/** @brief A_IPROPI in uA per A. 450 on the DRV8874, 1000 on the DRV8876 —
  *        changing the part means changing this, and the scale moves 2.22x. */
#define ISENSE_A_IPROPI_UA_PER_A  450u

/** @brief Current at raw 4095, mA — 4975 as fitted. The ADC ceiling, and the
  *        highest trip the DAC can ask for. No longer the same as the trip
  *        point: the 10k that made them equal has been removed. */
#define ISENSE_FULL_SCALE_MA \
  ((uint16_t)(((uint64_t)ISENSE_VDDA_MV * 1000000ull) / \
              ((uint64_t)ISENSE_A_IPROPI_UA_PER_A * ISENSE_R_IPROPI_OHM)))

/** @brief Raw counts at or above which the ADC itself is clipping. Distinct
  *        from regulating — the bridge regulates at the TRIP, which is now
  *        usually well below this. See isense_saturated(). */
#define ISENSE_SATURATED_RAW      4050u

/** @brief Trip set at boot, mA. Matches what the stock carrier enforced, so
  *        lifting the 10k did not quietly make anything more dangerous. */
#define ISENSE_TRIP_DEFAULT_MA    3000u

/** @brief Buffered DAC headroom from each rail, mV. F446 datasheet figure. */
#define ISENSE_VREF_BUF_MARGIN_MV 200u

/** @brief Default number of conversions averaged by isense_read_ma(). At
  *        1.78 us each this is 57 us — just over one PWM period, so a
  *        free-running average covers the whole cycle rather than a slice. */
#define ISENSE_AVG_DEFAULT        32u

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
  * @brief  Convert a raw count to milliamps.
  * @note   Exact integer form of raw x 3.3 / 4096 / 0.6632. The multiply peaks
  *         at 4095 x 4975 = 20.4e6, comfortably inside uint32.
  */
static inline uint32_t isense_raw_to_ma(uint16_t raw)
{
  return ((uint32_t)raw * ISENSE_FULL_SCALE_MA) / 4096u;
}

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
  * @brief  Whether the last reading reached ISENSE_SATURATED_RAW.
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
  * @brief  Convert a trip current to the VREF voltage that produces it.
  * @param  ma  milliamps
  * @return millivolts. Unclamped — the callers do the clamping.
  */
static inline uint32_t isense_ma_to_vref_mv(uint32_t ma)
{
  return (ma * ISENSE_A_IPROPI_UA_PER_A * ISENSE_R_IPROPI_OHM) / 1000000u;
}

/**
  * @brief  Convert a VREF voltage to the trip current it produces.
  * @param  mv  millivolts
  * @return milliamps.
  */
static inline uint32_t isense_vref_mv_to_ma(uint32_t mv)
{
  return (mv * 1000000u) / (ISENSE_A_IPROPI_UA_PER_A * ISENSE_R_IPROPI_OHM);
}

#ifdef __cplusplus
}
#endif

#endif /* ISENSE_H */

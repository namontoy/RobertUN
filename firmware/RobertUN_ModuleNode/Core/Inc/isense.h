/**
  ******************************************************************************
  * @file           : isense.h
  * @brief          : Drive current from the DRV8874's IPROPI output (PA2)
  ******************************************************************************
  *
  * WHAT THIS IS FOR
  * ----------------
  * Turns one ADC reading into milliamps. It closes the W4 task HW4's PDB
  * branch sizing has been waiting on — real drive current, measured rather
  * than calculated from winding resistance — and it is the sensor a W5 current
  * inner loop would read.
  *
  *
  * THE CARRIER DECIDES THE SCALING, AND OURS WAS MEASURED
  * ------------------------------------------------------
  * IPROPI sources a current proportional to the bridge current:
  *
  *     I_IPROPI = I_OUT x A_IPROPI        A_IPROPI = 450 uA/A on the DRV8874
  *
  * R_IPROPI turns that into a voltage. The carrier in hand populates
  * 2.48 kOhm (measured Sep 12, 2026 - NOT the 2.2 kOhm the selection document
  * assumed), so:
  *
  *     scale      = 450e-6 x 2480 = 1.1160 V/A
  *     full scale = 3.3 V / 1.1160 = 2.957 A
  *     one LSB    = 3.3 / 4096 / 1.1160 = 0.7219 mA
  *
  * A_IPROPI is 1000 uA/A on the DRV8876 second source, so a swap changes this
  * constant by 2.22x. It is not a transparent substitution.
  *
  *
  * VREF IS WIRED TO nSLEEP, AND THAT HAS THREE CONSEQUENCES
  * --------------------------------------------------------
  * The carrier ties VREF to nSLEEP through 10 kOhm into a high-impedance
  * input, so VREF = 3.3 V whenever the driver is enabled and 0 V when it is
  * not. This is not a detail; it decides three separate things.
  *
  * 1. CURRENT REGULATION IS ALWAYS ARMED, AT MAXIMUM. The bridge regulates
  *    when V_IPROPI reaches VREF, so
  *
  *        I_TRIP = VREF / (A_IPROPI x R_IPROPI) = 3.3 / 1.1160 = 2.957 A
  *
  *    There is now a HARDWARE current limit at ~2.96 A, where the DRV8833 had
  *    none at all. The 5.0 A stall at the 9.5 V rail can no longer happen:
  *    the driver folds back first. drive.h's warning that a 0-to-full step
  *    draws stall current is, on this carrier, bounded by silicon.
  *
  * 2. FULL SCALE AND THE TRIP POINT ARE THE SAME NUMBER. Both are VREF over
  *    the same scale factor, so the ADC saturates at exactly the current where
  *    regulation begins - 2.957 A, raw 4095. A saturated reading and an
  *    actively-regulating bridge are the same event, which is why
  *    isense_saturated() is worth checking rather than ignoring.
  *
  *    The corollary is that the two CANNOT be traded independently. Lowering
  *    R_IPROPI to raise the trip point lowers the ADC sensitivity by the same
  *    factor. 2.48 kOhm sets both at ~2.96 A, which sits right at the
  *    DRV8874's realistic continuous rating - a sensible place for it.
  *
  * 3. THE PA4 / DAC1 VREF UPGRADE IS FORECLOSED on this carrier. A
  *    software-programmable current limit needs VREF driven independently, and
  *    that 10 kOhm would fight the DAC. It is one lifted resistor away, not
  *    available as shipped. PA4 stays free; nothing is wired to it.
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
  * caller records the duty alongside it. drive_duty() is right there.
  *
  *
  * VERIFY THE VREF COMPARISON WHILE YOU ARE THERE
  * -----------------------------------------------
  * I_TRIP above assumes V_IPROPI is compared against VREF directly, with no
  * internal divider. If the datasheet turns out to divide VREF by k, the trip
  * is k times lower - and the measurement tells you k for free. Stall the
  * motor against the rig and watch where the raw reading PLATEAUS:
  *
  *     plateau at raw ~4095  ->  k = 1, trip 2.96 A, as assumed here
  *     plateau at raw ~2048  ->  k = 2, trip 1.48 A
  *     plateau at raw ~1365  ->  k = 3, trip 0.99 A
  *
  * A plateau well below saturation is not a broken sensor. It is the current
  * limit, and where it sits is the answer.
  *
  *
  * SAMPLING TIME IS 28 CYCLES ON PURPOSE
  * --------------------------------------
  * At PCLK2/4 = 22.5 MHz a 28-cycle sample plus 12 cycles of conversion is
  * 1.78 us, which fits inside the PWM on-phase even at 25% duty (12.5 us).
  * The 480-cycle maximum would take 21.9 us and straddle half the period,
  * making PWM-synchronised sampling impossible without returning to CubeMX.
  * 28 cycles is still about 2.4x what a 2.48 kOhm source needs to settle to
  * 12 bits, so nothing was given up to keep that door open.
  *
  * The door leads here, when W5 wants it: trigger the ADC from TIM4_CH4's
  * compare event. TIM4 already generates the PWM, and a compare channel raises
  * its event with no pin configured - TIM4_CH3/CH4 land on PB8/PB9, which are
  * the CAN pins, but the internal event does not need them. That puts the
  * sample at a chosen point in the on-phase and retires the question above.
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

/** @brief ADC reference, mV. VDDA on this board. */
#define ISENSE_VDDA_MV            3300u

/** @brief IPROPI sense resistor populated on the carrier, ohms.
  *        MEASURED Sep 12, 2026. The selection document assumed 2.2k. */
#define ISENSE_R_IPROPI_OHM       2480u

/** @brief A_IPROPI in uA per A. 450 on the DRV8874, 1000 on the DRV8876 —
  *        changing the part means changing this, and the scale moves 2.22x. */
#define ISENSE_A_IPROPI_UA_PER_A  450u

/** @brief Current at raw 4095, mA. Also the current-regulation trip point,
  *        because VREF is tied to the 3.3 V nSLEEP level — see the header. */
#define ISENSE_FULL_SCALE_MA \
  ((uint16_t)(((uint64_t)ISENSE_VDDA_MV * 1000000ull) / \
              ((uint64_t)ISENSE_A_IPROPI_UA_PER_A * ISENSE_R_IPROPI_OHM)))

/** @brief Raw counts at or above which the bridge is regulating, not measuring.
  *        Slightly below 4095 so a real limit is caught before it clips. */
#define ISENSE_SATURATED_RAW      4050u

/** @brief Default number of conversions averaged by isense_read_ma(). At
  *        1.78 us each this is 57 us — just over one PWM period, so a
  *        free-running average covers the whole cycle rather than a slice. */
#define ISENSE_AVG_DEFAULT        32u

/**
  * @brief  Prepare the module. Call after MX_ADC1_Init().
  * @note   Does not zero the offset; call isense_zero() with the driver
  *         disabled for that.
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
  * @note   Exact integer form of raw x 3.3 / 4096 / 1.116. The multiply peaks
  *         at 4095 x 2956 = 12.1e6, comfortably inside uint32.
  */
static inline uint32_t isense_raw_to_ma(uint16_t raw)
{
  return ((uint32_t)raw * ISENSE_FULL_SCALE_MA) / 4096u;
}

/**
  * @brief  Measure and store the zero offset.
  * @note   Call with the driver DISABLED and the motor still. Leakage and ADC
  *         offset are both small here, but a few counts of standing offset is
  *         6-7 mA of lie at every operating point.
  * @return The offset stored, in raw counts.
  */
uint16_t isense_zero(void);

/** @brief The stored zero offset, raw counts. */
uint16_t isense_offset(void);

/**
  * @brief  Whether the last reading was at or above ISENSE_SATURATED_RAW.
  * @note   On this carrier that means the bridge is current-regulating at
  *         ~2.96 A, not that the sensor is broken. Treat it as a real event.
  */
bool isense_saturated(void);

#ifdef __cplusplus
}
#endif

#endif /* ISENSE_H */

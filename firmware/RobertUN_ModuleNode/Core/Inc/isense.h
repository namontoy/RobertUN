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
  *     scale      = 450e-6 x 1465   = 0.6593 V/A
  *     full scale = 3.325 / 0.6593  = 5.044 A     <- ADC ceiling
  *     one LSB    = 5044 / 4096     = 1.231 mA
  *
  * Those are the MEASURED constants - R_IPROPI 1465 ohm across the fitted pair,
  * VDDA 3325 mV on this board - applied 2026-09-20, once the plateau sweep was
  * finished and nothing was left mid-experiment. On the 1474/3300 nominals the
  * same board computes 0.6632 V/A, 4.975 A and 1.215 mA, which is why every
  * current logged before that date reads ~1.4% low.
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
  *     R_IPROPI alone sets the CEILING       = 5.044 A, fixed
  *     VREF sets the TRIP anywhere from 0 up to a THIRD of that ceiling
  *
  *     I_TRIP = V_VREF / (k x A_IPROPI x R_IPROPI),   k = 3
  *
  * The k is the DRV8874's own internal divider: it compares IPROPI against
  * VREF/3, not VREF. Measured Sep 20, 2026 - see the plateau section below.
  * It lives in config as `vref_div` so a second source with a different
  * divider can be accommodated without a rebuild, but 3 is the measured value
  * for this part and there is no reason to move it.
  *
  * THE CEILING AND THE TRIP ARE STRUCTURALLY COUPLED, AND CANNOT BE SEPARATED
  * IN FIRMWARE. The comparator and the ADC read the same R_IPROPI, so the
  * maximum commandable trip is always exactly one third of the ADC ceiling -
  * 1.681 A here - whatever value R_IPROPI takes. Wanting a 4 A trip means
  * wanting a ~12 A ceiling, which is a resistor change and a hardware trade:
  * the ADC grain coarsens by the same factor.
  *
  * One DAC code moves the trip by 0.410 mA - one third of an ADC LSB, because
  * the two converters are both 12 bits across the same 3.3 V through the same
  * resistor, but the comparator sees VREF divided by three. The limit is
  * therefore finer-grained than the measurement, and can be set to a precision
  * this module cannot read back.
  *
  * THE COST IS THAT THE FAIL-SAFE IS GONE. The 10 kOhm guaranteed VREF could
  * never be wrong while nSLEEP was high; they moved together. Now VREF must be
  * SET BEFORE nSLEEP RISES, every time, including after any reset. isense_init()
  * does this at boot, before anything can command the driver — and if the DAC
  * were somehow not running, the trip is 0 and the motor simply will not turn.
  * That is the safe direction to fail, and it is deliberate.
  *
  *
  * WHY THE DEFAULT TRIP IS 1.0 A AND NOT THE CEILING
  * --------------------------------------------------
  * >> CORRECTED 2026-09-20. This default used to read 3000 and the comment
  * >> used to claim it reproduced the stock carrier's ~2.96 A. With k = 3 that
  * >> number was never 3 A: `drv trip 3000` asked for, and got, 1 A. The
  * >> default is now written as the 1000 mA it always physically was, so the
  * >> printed figure and the enforced one finally agree.
  *
  * 1.0 A is also a sane place to boot regardless of the history: it is above
  * anything the rover draws rolling, below the ~1.26 A this motor pulls
  * stalled at 20% duty on 12 V, and well inside the 1.580 A ceiling. Reaching
  * full capacity is then an explicit act: `drv trip 1550`.
  *
  *
  * THE OUTPUT BUFFER COSTS YOU THE TOP OF THE RANGE
  * -------------------------------------------------
  * The F446's DAC output buffer is enabled by default here: it has a low
  * output impedance and will drive VREF regardless of what the pin's input
  * impedance turns out to be. The price is that a buffered output cannot reach
  * either rail — roughly 0.2 V to VDDA-0.2 V:
  *
  *     buffered    0.200 .. 3.125 V  ->  trip  0.101 .. 1.580 A
  *     unbuffered  0.000 .. 3.325 V  ->  trip  0.000 .. 1.681 A
  *
  * (These were quoted as 0.30 .. 4.67 A and 0.00 .. 4.98 A before k was
  * measured on Sep 20, 2026. Same voltages, same hardware, three times too
  * large.)
  *
  * 1.580 A is far BELOW the ~6.3 A this motor draws stalled cold at 12 V. So
  * with the buffer on - or off, the 0.1 A the buffer costs is not what decides
  * this - a genuine hard stall regulates rather than being measured. That is
  * safe, and it is also not what a full-capacity test is trying to find out;
  * measuring a real stall needs a smaller R_IPROPI, not a different buffer
  * setting. isense_set_vref_buffered() and `drv trip buf off` remain, to
  * reclaim the bottom and top 0.1 A once VREF's input impedance is known to be
  * high enough to leave unbuffered. Check that with a meter on the pin before
  * trusting an unbuffered setting: if the commanded and measured VREF
  * disagree, the buffer belongs back on.
  *
  *
  * THE PLATEAU TEST WAS RUN, AND k IS 3 - 2026-09-20
  * --------------------------------------------------
  * The question this section used to pose - is IPROPI compared against VREF
  * directly, or against some internal fraction of it - was answered on the
  * bench by setting a known trip, stalling the shaft, and reading where the
  * synchronised current plateaued. Five points, 300 to 1500 mA commanded:
  *
  *     commanded   plateau (mA)   ratio
  *        300          101         2.97
  *        600          200         3.00
  *        900          302         2.98
  *       1200          398         3.02
  *       1500          503         2.98
  *
  * A straight line through the origin with slope 1/3, to better than 1%. The
  * comparator sees VREF/3. k = 1 and k = 2 are both refuted by more than the
  * measurement's own spread, and the result is demand-independent: raising the
  * duty 7.6% above what regulation needed moved a plateau by -2.3%, i.e. not
  * at all.
  *
  * Everything commanded before that date was three times smaller than it
  * printed. That is the safe direction to have been wrong in, and it is also
  * why several "the motor is weaker than predicted" results from the same
  * period were really the driver regulating at a third of the intended limit.
  *
  * Two things this test also taught, both about the READING rather than the
  * trip:
  *
  *   - Under deep regulation IPROPI goes blind. The driver chops so hard that
  *     the drive sub-window shrinks below the settle time, and the plateau
  *     figure stops tracking - about 92% of the period is chopped away at the
  *     low end. Regulation is audible before it is visible.
  *   - A plateau is only worth reading from the SETTLED tail of the window.
  *     Taken at the midpoint it reads ~13% low, which is what sent the first
  *     pass of this measurement chasing a non-integer k.
  *
  *
  * WHAT THE READING MEANS - CALIBRATED 2026-09-20
  * -----------------------------------------------
  * >> RETRACTS THE "SETTLED ON THE BENCH 2026-09-12" SECTION THAT STOOD HERE.
  * >> That section concluded isense_read_sync_avg() returned SUPPLY current
  * >> (I_motor x D) and blamed the carrier's 20 kOhm IMODE strap for blanking
  * >> IPROPI during recirculation. Both halves were wrong:
  * >>
  * >>   - The cause. IPROPI does not read zero during recirculation because
  * >>     something blanks it. In slow decay the current recirculates through
  * >>     the HIGH-side FETs, and IPROPI mirrors only the low-side sense
  * >>     element - it is physically blind to that path. No strap involved.
  * >>   - The conclusion. The Sep 12 numbers were taken with the trigger in
  * >>     the wrong place AND with PMODE unstrapped, so the bridge was not in
  * >>     the decay mode the analysis assumed. The commit that moved the
  * >>     trigger into the drive phase (9187dc9, Sep 16) already made this a
  * >>     motor-current reading; the Sep 19 PMODE fix did not change that.
  * >>
  * >> Anything derived from that section - in particular the quadratic
  * >> "plateau = trip^2 x R_motor / Vm" correction - should be discarded
  * >> rather than adjusted.
  *
  * The synchronised path samples inside the drive phase, where IPROPI is live
  * and mirrors the bridge current directly. Therefore:
  *
  *   isense_read_sync_avg() returns MOTOR current, during drive.
  *
  * Calibrated against a stalled shaft, which removes back-EMF and leaves pure
  * Ohm's law with no friction model in the way:
  *
  *   20% duty, stalled, Vm 12.0 V, R_motor 1.90 ohm
  *     predicted  = 0.20 x 12.0 / 1.90  = 1263 mA
  *     MEASURED (settled tail of the window)  = 1290 mA     2% high
  *
  * 2% is inside the rotor-position noise floor: repeating the same reading at
  * different shaft angles moves it by +/-8%, because a stalled motor is a
  * handful of commutator segments, not a resistor. Where the shaft stops is
  * part of the measurement.
  *
  * isense_read_avg() - the free-running path - is a different quantity. It
  * averages the whole period, including the recirculation time where IPROPI
  * reads zero, so it returns something close to I_motor x D. It is kept for
  * isense_zero() and as the sub-15%-duty fallback, and its figure is labelled
  * Isup at the console precisely so the two are never read as one number.
  *
  * The trip and the synchronised reading now share units: the DRV8874
  * regulates on instantaneous IPROPI against VREF/3 - motor current during
  * drive - so a plateau can be compared with the commanded trip directly.
  *
  * Callers should keep recording duty alongside the reading regardless -
  * drive_duty() is right there, and `drv current` already prints both.
  *
  *
  * AND THE FREE-RUNNING AVERAGE TURNED OUT TO ALIAS - 2026-09-14
  *
  * >> PARTLY RETRACTED 2026-09-15. Read this whole section with that in mind.
  * >> SUPERSEDED 2026-09-26: every scan below predates the PMODE strap (Sep 19);
  * >> the driver was in independent half-bridge, so decay was high-side and the
  * >> low-side mirror read 0. In PWM mode the decay phase reads 0.690 x I_motor
  * >> (valid >= 6% duty, +/-4%); see the wheel-FW LOG, 2026-09-26 (night).
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
  *                          the fallback below the ~14.5% duty the synchronised
  *                          path needs.
  *
  *   isense_read_sync_avg() triggered from TIM4_CH4, in the settled tail of the
  *                          drive phase. IPROPI is live there, so this is MOTOR
  *                          current, measured directly.
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
  *     instantaneous IPROPI against VREF/3 - motor current - so a plateau in a
  *     synchronised reading can be compared with the commanded trip directly.
  *
  * The cost is time: each sample waits for the next compare event, so one
  * conversion costs a whole 50 us period rather than ~7 us. 64 samples is 3.2 ms.
  * That is why the sync default is 64 and not 1024 - past a point the extra
  * samples are averaging the wheel, not the converter.
  *
  *
  * WHERE IN THE WINDOW THE SAMPLE IS TAKEN - MEASURED 2026-09-20
  * --------------------------------------------------------------
  * The trigger used to sit at the MIDDLE of the drive window. That is the
  * contaminated half: IPROPI needs 5.6 us to settle after the drive edge -
  * 500 TIM4 ticks, not the datasheet's 1.6 us tDELAY - and all of the ringing
  * is at the leading edge. At 20% duty the midpoint read 994 raw against a
  * settled 1143. Thirteen percent low, silently, at every duty.
  *
  * The budget now lives in drive.h and the placement is END-relative:
  *
  *     settle after the drive edge   500 ticks   measured
  *     ADC aperture                  112 ticks   28 cycles @ 22.5 MHz
  *     guard before the falling edge  40 ticks   chosen
  *     -----------------------------------------
  *     minimum honest drive window   652 ticks = 14.5% duty
  *
  *     trigger = window_end - (aperture + guard),  floored at start + settle
  *
  * so the aperture always sits in settled signal with a guard band, at any
  * duty, and isense_sync_ready() refuses the window when it cannot.
  *
  * THE COST IS THE 14.5% FLOOR. It used to be 4.3%, which was not a smaller
  * floor but a wrong one - it green-lit readings whose entire drive window was
  * shorter than the settle time. Below 14.5% there is no synchronised reading
  * to be had at this carrier frequency, only a free-running Isup figure or a
  * slower carrier.
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

/** @brief ADC and DAC reference, mV. VDDA on this board. Live: CFG_VDDA_MV.
  *        3325 MEASURED on the bench board, not the 3300 nominal. Applied
  *        2026-09-20, held until then so nothing moved underneath the plateau
  *        sweep. A second board must be measured, not assumed. */
#define ISENSE_VDDA_MV_DEFAULT           3325u

/** @brief IPROPI sense resistor, ohms. MODIFIED CARRIER: 2.0k || 5.6k, the
  *        stock 2.48k having been removed. 1465 MEASURED on the fitted pair,
  *        against the 1474 nominal parallel value. Applied 2026-09-20 alongside
  *        VDDA, for the same reason and with the same caveat: this is a
  *        per-board figure. Measure the pair and set CFG_R_IPROPI_OHM on any
  *        other carrier rather than inheriting this one. */
#define ISENSE_R_IPROPI_OHM_DEFAULT      1465u

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
#define ISENSE_TRIP_DEFAULT_MA           1000u

/** @brief Buffered DAC headroom from each rail, mV. F446 datasheet figure. */
#define ISENSE_VREF_BUF_MARGIN_MV 200u

/** @brief Default number of conversions averaged by isense_read_ma(). At
  *        1.78 us each this is 57 us — just over one PWM period, so a
  *        free-running average covers the whole cycle rather than a slice. */
#define ISENSE_AVG_DEFAULT        32u

/* The drive-window floor that used to live here (ISENSE_SYNC_MIN_TICKS, 192)
   moved to drive.h as DRIVE_PHASE_MIN_TICKS on 2026-09-20, and grew to 652.
   192 was derived from the ADC aperture alone and ignored the time IPROPI needs
   to settle, so it admitted readings taken entirely inside the ringing. The
   budget belongs next to place_trigger(), which is the code that has to honour
   it. */

/** @brief How many distinct ticks a synchronised average spreads itself over,
  *        inside the settled part of the drive window. The sample COUNT is
  *        unchanged and simply divided between them, so this costs no extra
  *        time - 4 points is still 64 periods at the default depth. Raising it
  *        buys less than it looks like it should: past a handful the points are
  *        closer together than the ripple they are meant to average over. */
#define ISENSE_SYNC_POINTS         4u

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
  * @return mA of SUPPLY current, I_motor x D - this path averages the whole
  *         period, including the recirculation time where IPROPI reads zero.
  *         Record drive_duty() alongside it.
  */
uint32_t isense_read_ma(uint16_t samples);

/**
  * @brief  Whether a synchronised sample can be taken right now.
  * @retval true   the drive phase is at least DRIVE_PHASE_MIN_TICKS wide
  * @retval false  duty is 0, the bridge is braking, or the phase is too narrow
  *                for the sampling aperture to sit inside it
  * @note   Ask before reading rather than interpreting a 0 afterwards - a
  *         genuine 0 mA and a refusal are the same number.
  */
bool isense_sync_ready(void);

/**
  * @brief  Average of @p samples conversions taken in the settled tail of the
  *         PWM drive phase, offset-corrected.
  * @note   The samples are spread over ISENSE_SYNC_POINTS ticks inside the
  *         settled region, not stacked on one tick. Same count, same cost.
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

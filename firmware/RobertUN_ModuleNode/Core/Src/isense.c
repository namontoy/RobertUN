/**
  ******************************************************************************
  * @file           : isense.c
  * @brief          : Drive current — measurement (PA2/IPROPI) and limit (PA4/VREF)
  ******************************************************************************
  * Rationale — the modified carrier, why measurement and limit share a module,
  * what removing the nSLEEP-to-VREF resistor bought and cost, and the plateau
  * test that validates the whole chain — is in isense.h. This file is the
  * mechanics.
  ******************************************************************************
  */
#include "isense.h"

#include "config.h"
#include "drive.h"
#include "main.h"

extern ADC_HandleTypeDef hadc1;
extern DAC_HandleTypeDef hdac;

static uint16_t zero_offset;    /*!< raw counts read with the bridge off   */
static bool     was_saturated;  /*!< set by the last conversion taken      */
static uint16_t vref_code;      /*!< last code written to the DAC          */
static bool     vref_buffered;  /*!< output buffer state, tracked here     */

/* --- scaling ------------------------------------------------------------- *
 * All three of these used to be macros or inlines in the header, evaluated at
 * compile time. They are functions now because their inputs are config keys:
 * an inline would have frozen whatever the defaults were at build time, which
 * is exactly the freezing the config module exists to undo.
 * -------------------------------------------------------------------------- */

uint16_t isense_full_scale_ma(void)
{
  uint32_t a_ua = (uint32_t)config_get(CFG_A_IPROPI_UA_PER_A);
  uint32_t r    = (uint32_t)config_get(CFG_R_IPROPI_OHM);

  /* Ranges in the key table keep both factors well clear of zero, but this is
     a divisor in the path that sets a CURRENT LIMIT, and a config module that
     fails open is worse than no config module. */
  if ((a_ua == 0u) || (r == 0u))
  {
    return 0u;
  }

  return (uint16_t)(((uint64_t)config_get(CFG_VDDA_MV) * 1000000ull) /
                    ((uint64_t)a_ua * r));
}

uint32_t isense_raw_to_ma(uint16_t raw)
{
  return ((uint32_t)raw * isense_full_scale_ma()) / 4096u;
}

uint32_t isense_ma_to_vref_mv(uint32_t ma)
{
  return (ma * (uint32_t)config_get(CFG_A_IPROPI_UA_PER_A)
             * (uint32_t)config_get(CFG_R_IPROPI_OHM)) / 1000000u;
}

uint32_t isense_vref_mv_to_ma(uint32_t mv)
{
  uint32_t denom = (uint32_t)config_get(CFG_A_IPROPI_UA_PER_A)
                 * (uint32_t)config_get(CFG_R_IPROPI_OHM);

  return (denom == 0u) ? 0u : ((mv * 1000000u) / denom);
}

/* --- measurement --------------------------------------------------------- */

uint16_t isense_read_raw(void)
{
  uint16_t raw = 0u;

  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    return 0u;
  }

  /* 1.78 us at 28 cycles + 12, PCLK2/4. The 2 ms timeout is HAL's minimum
     useful granularity, not an expectation — this never approaches it. */
  if (HAL_ADC_PollForConversion(&hadc1, 2u) == HAL_OK)
  {
    raw = (uint16_t)HAL_ADC_GetValue(&hadc1);
  }

  (void)HAL_ADC_Stop(&hadc1);

  was_saturated = (raw >= (uint16_t)config_get(CFG_ISENSE_SAT_RAW));
  return raw;
}

uint16_t isense_read_avg(uint16_t samples)
{
  uint32_t sum       = 0u;
  bool     saturated = false;
  uint16_t n;
  uint16_t i;

  n = (samples == 0u) ? (uint16_t)config_get(CFG_ISENSE_AVG) : samples;
  if (n > 1024u)
  {
    n = 1024u;
  }

  for (i = 0u; i < n; i++)
  {
    sum += isense_read_raw();
    /* Latch saturation across the whole average. A bridge that clipped for
       part of the window did clip, and averaging would hide it. */
    saturated = saturated || was_saturated;
  }

  was_saturated = saturated;

  {
    uint16_t mean = (uint16_t)(sum / n);
    return (mean > zero_offset) ? (uint16_t)(mean - zero_offset) : 0u;
  }
}

uint32_t isense_read_ma(uint16_t samples)
{
  return isense_raw_to_ma(isense_read_avg(samples));
}

/* --- synchronised measurement -------------------------------------------- *
 * Why this exists rather than "average harder" is in isense.h. drive.c parks
 * TIM4_CH4's compare in the middle of the drive phase; all this end does is
 * point the ADC at that event, take what arrives, and put the trigger back.
 * -------------------------------------------------------------------------- */

/**
  * @brief  Point the ADC at TIM4_CH4, or back at software start.
  *
  * Register level on purpose. Re-running HAL_ADC_Init() to flip two bitfields
  * would re-check and rewrite the entire configuration - including the 28-cycle
  * sampling time that the whole scheme depends on - to change EXTSEL and EXTEN.
  * Two bitfields in CR2 is the actual change, so that is what is written.
  *
  * The handle's Init fields are kept in step with the register. Nothing calls
  * HAL_ADC_Init() again after boot, so it makes no functional difference; it
  * means the handle does not quietly describe a configuration the peripheral is
  * not in, which is the kind of discrepancy that costs an afternoon later.
  */
static void adc_trigger_set(bool hardware)
{
  if (hardware)
  {
    MODIFY_REG(hadc1.Instance->CR2,
               ADC_CR2_EXTSEL | ADC_CR2_EXTEN,
               ADC_EXTERNALTRIGCONV_T4_CC4 | ADC_EXTERNALTRIGCONVEDGE_RISING);

    hadc1.Init.ExternalTrigConv     = ADC_EXTERNALTRIGCONV_T4_CC4;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  }
  else
  {
    CLEAR_BIT(hadc1.Instance->CR2, ADC_CR2_EXTSEL | ADC_CR2_EXTEN);

    hadc1.Init.ExternalTrigConv     = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  }
}

/**
  * @brief  Spin until the next triggered conversion lands.
  * @retval false  nothing arrived within the guard - treated as "no sample"
  *
  * A trigger arrives every 50 us, which is 9000 core cycles at 180 MHz. This
  * loop is a few cycles per pass, so the bound is several periods of patience
  * and still far short of anything a console user would notice. Returning
  * false rather than spinning forever is the point: if the PWM stops while this
  * is sampling - a fault, a duty of 0 racing in from another path - the
  * alternative is a dead console with no way to ask why.
  */
static bool adc_wait_eoc(void)
{
  uint32_t guard = 200000u;

  while (!__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC))
  {
    if (--guard == 0u)
    {
      return false;
    }
  }

  return true;
}

bool isense_sync_ready(void)
{
  return drive_phase_ticks() >= (uint16_t)ISENSE_SYNC_MIN_TICKS;
}

/**
  * @brief  Take n trigger-driven conversions and return their mean, raw.
  *
  * No phase gate and no offset subtraction - both belong to the callers, which
  * disagree about them. Whoever calls this is responsible for having put the
  * trigger somewhere meaningful first.
  */
static uint16_t sync_burst(uint16_t samples)
{
  uint32_t sum       = 0u;
  uint16_t taken     = 0u;
  bool     saturated = false;
  uint16_t n;
  uint16_t i;

  n = (samples == 0u) ? (uint16_t)ISENSE_SYNC_AVG_DEFAULT : samples;
  if (n > 1024u)
  {
    n = 1024u;
  }

  adc_trigger_set(true);

  /* Started ONCE, not once per sample. With an external trigger armed, every
     compare event converts on its own; HAL_ADC_Start() only enables the ADC and
     skips SWSTART. Wrapping each conversion in Start/Stop would disable and
     re-enable the peripheral n times and pay the stabilisation delay for every
     one of them. */
  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    adc_trigger_set(false);
    return 0u;
  }

  for (i = 0u; i < n; i++)
  {
    uint16_t raw;

    if (!adc_wait_eoc())
    {
      break;
    }

    raw = (uint16_t)hadc1.Instance->DR;   /* the read is what clears EOC */

    /* An interrupt landing between two triggers can delay the read past the
       next conversion, which sets OVR and leaves DR holding the older sample.
       That sample is still a legitimate reading at the same phase, one period
       earlier, so it is kept and the flag is cleared. Left uncleared, DR would
       stop updating and the rest of the average would be n copies of one
       number. */
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR);

    sum += raw;
    taken++;
    saturated = saturated || (raw >= (uint16_t)config_get(CFG_ISENSE_SAT_RAW));
  }

  (void)HAL_ADC_Stop(&hadc1);
  adc_trigger_set(false);

  was_saturated = saturated;

  return (taken == 0u) ? 0u : (uint16_t)(sum / taken);
}

uint16_t isense_read_sync_avg(uint16_t samples)
{
  uint16_t mean;

  /* Refuse rather than return a number from the wrong phase. A caller that
     cannot tell 0 mA from "not measurable" would record the first as the
     second, and at these currents that is a plausible-looking lie. */
  if (!isense_sync_ready())
  {
    return 0u;
  }

  mean = sync_burst(samples);

  return (mean > zero_offset) ? (uint16_t)(mean - zero_offset) : 0u;
}

uint16_t isense_read_sync_at(uint16_t tick, uint16_t samples)
{
  uint16_t mean;

  drive_trigger_override(tick);
  mean = sync_burst(samples);
  drive_trigger_restore();

  return mean;
}

uint32_t isense_read_motor_ma(uint16_t samples)
{
  return isense_raw_to_ma(isense_read_sync_avg(samples));
}

uint16_t isense_zero(void)
{
  uint32_t sum = 0u;
  uint16_t i;

  /* Deliberately NOT isense_read_avg() — that subtracts the offset we are
     trying to measure. Raw conversions only, and a long average because this
     number is subtracted from every later reading. */
  zero_offset = 0u;
  for (i = 0u; i < 256u; i++)
  {
    sum += isense_read_raw();
  }

  zero_offset   = (uint16_t)(sum / 256u);
  was_saturated = false;
  return zero_offset;
}

uint16_t isense_offset(void)
{
  return zero_offset;
}

bool isense_saturated(void)
{
  return was_saturated;
}

/* --- current limit ------------------------------------------------------- */

/** @brief Lowest VREF the DAC can hold, mV. The buffer cannot reach GND. */
static uint16_t vref_floor_mv(void)
{
  return vref_buffered ? (uint16_t)ISENSE_VREF_BUF_MARGIN_MV : 0u;
}

/** @brief Highest VREF the DAC can hold, mV. Nor can it reach VDDA. */
static uint16_t vref_ceiling_mv(void)
{
  return vref_buffered
           ? (uint16_t)(config_get(CFG_VDDA_MV) - ISENSE_VREF_BUF_MARGIN_MV)
           : (uint16_t)config_get(CFG_VDDA_MV);
}

/* The DAC divides by 4095, not 4096 — Vout = VDDA x DOR / 4095. The ADC uses
   4096. They are genuinely different in the reference manual and the one-count
   discrepancy is not worth hiding behind a shared constant. */
static uint16_t mv_to_code(uint32_t mv)
{
  uint32_t vdda = (uint32_t)config_get(CFG_VDDA_MV);
  uint32_t code = (mv * 4095u + (vdda / 2u)) / vdda;
  return (code > 4095u) ? 4095u : (uint16_t)code;
}

static uint16_t code_to_mv(uint16_t code)
{
  return (uint16_t)(((uint32_t)code * (uint32_t)config_get(CFG_VDDA_MV)) / 4095u);
}

/** @brief Push vref_code to the hardware. */
static void vref_apply(void)
{
  (void)HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, vref_code);
  (void)HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
}

/**
  * @brief  The DAC code a trip request maps to, and whether the range had to
  *         clamp it. Single-sourced so the "what would this do" and the "do it"
  *         paths cannot drift apart.
  */
static uint16_t trip_code(uint32_t ma, bool *clamped)
{
  uint32_t mv = isense_ma_to_vref_mv(ma);
  bool     c  = false;

  if (mv < vref_floor_mv())   { mv = vref_floor_mv();   c = true; }
  if (mv > vref_ceiling_mv()) { mv = vref_ceiling_mv(); c = true; }

  if (clamped != NULL)
  {
    *clamped = c;
  }

  return mv_to_code(mv);
}

bool isense_set_trip_ma(uint32_t ma)
{
  bool clamped;

  vref_code = trip_code(ma, &clamped);
  vref_apply();

  return !clamped;
}

uint16_t isense_code_for_trip_ma(uint32_t ma)
{
  return trip_code(ma, NULL);
}

uint32_t isense_trip_ma(void)
{
  /* Derived back from the code that was actually written, so quantisation and
     clamping are both visible to the caller rather than assumed away. */
  return isense_vref_mv_to_ma(code_to_mv(vref_code));
}

uint16_t isense_vref_mv(void)
{
  return code_to_mv(vref_code);
}

uint16_t isense_vref_code(void)
{
  return vref_code;
}

uint32_t isense_trip_max_ma(void)
{
  return isense_vref_mv_to_ma(vref_ceiling_mv());
}

uint32_t isense_trip_min_ma(void)
{
  return isense_vref_mv_to_ma(vref_floor_mv());
}

void isense_set_vref_buffered(bool on)
{
  DAC_ChannelConfTypeDef cfg = {0};
  uint32_t               trip;

  if (on == vref_buffered)
  {
    return;
  }

  /* Capture the trip before the range changes, then re-request it: the new
     range may not be able to hold it, and isense_set_trip_ma() will clamp and
     report honestly rather than leaving a stale number in vref_code. */
  trip = isense_trip_ma();

  (void)HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);

  cfg.DAC_Trigger      = DAC_TRIGGER_NONE;
  cfg.DAC_OutputBuffer = on ? DAC_OUTPUTBUFFER_ENABLE : DAC_OUTPUTBUFFER_DISABLE;
  (void)HAL_DAC_ConfigChannel(&hdac, &cfg, DAC_CHANNEL_1);

  vref_buffered = on;
  (void)isense_set_trip_ma(trip);
}

bool isense_vref_buffered(void)
{
  return vref_buffered;
}

/* --- init ---------------------------------------------------------------- */

void isense_init(void)
{
  zero_offset   = 0u;
  was_saturated = false;

  /* Matches MX_DAC_Init()'s generated configuration. Tracked rather than read
     back because the HAL exposes no getter, and isense_set_vref_buffered()
     needs to know what it is changing from. */
  vref_buffered = true;

  /* The trip is armed HERE, before main() can call anything that raises
     nSLEEP. On the modified carrier VREF no longer follows nSLEEP, so a
     driver woken with VREF still at reset would regulate at 0 A. */
  (void)isense_set_trip_ma((uint32_t)config_get(CFG_TRIP_BOOT_MA));
}

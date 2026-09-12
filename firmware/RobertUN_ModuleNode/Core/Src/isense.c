/**
  ******************************************************************************
  * @file           : isense.c
  * @brief          : Drive current from the DRV8874's IPROPI output (PA2)
  ******************************************************************************
  * Rationale — the carrier's 2.48 kOhm sense resistor, why VREF being tied to
  * nSLEEP sets the current limit and the ADC full scale to the same number,
  * and the unresolved question of whether IPROPI reports during recirculation
  * — is in isense.h. This file is the mechanics.
  ******************************************************************************
  */
#include "isense.h"

#include "main.h"

extern ADC_HandleTypeDef hadc1;

static uint16_t zero_offset;   /*!< raw counts read with the bridge off */
static bool     was_saturated; /*!< set by the last conversion taken    */

void isense_init(void)
{
  zero_offset   = 0u;
  was_saturated = false;
}

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

  was_saturated = (raw >= ISENSE_SATURATED_RAW);
  return raw;
}

uint16_t isense_read_avg(uint16_t samples)
{
  uint32_t sum       = 0u;
  bool     saturated = false;
  uint16_t n;
  uint16_t i;

  n = (samples == 0u) ? (uint16_t)ISENSE_AVG_DEFAULT : samples;
  if (n > 1024u)
  {
    n = 1024u;
  }

  for (i = 0u; i < n; i++)
  {
    sum += isense_read_raw();
    /* Latch saturation across the whole average. A bridge that regulated for
       part of the window did regulate, and averaging would hide it. */
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

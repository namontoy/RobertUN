/**
  ******************************************************************************
  * @file           : encoder.c
  * @brief          : Drive-motor quadrature encoder on TIM2 (PA15 / PB3)
  ******************************************************************************
  * Rationale — the delta idiom, the resolution numbers, and why velocity is
  * windowed — is in encoder.h. This file is the mechanics.
  ******************************************************************************
  */
#include "encoder.h"

#include "main.h"

extern TIM_HandleTypeDef htim2;

/* Sampled and written by the TIM6 ISR, read by the main loop. */
static volatile int64_t  position;        /*!< counts since the last zero      */
static volatile int32_t  last_delta;      /*!< counts in the most recent tick  */
static volatile uint32_t ticks;           /*!< ticks since the last zero       */
static uint32_t          prev_count;      /*!< ISR-private, no lock needed     */

/* Velocity is accumulated over a window and only recomputed when it closes. */
static volatile float    rpm;
static volatile int32_t  window_counts;
static volatile uint16_t window_ticks;
static uint16_t          window_len = ENCODER_VELOCITY_WINDOW_DEFAULT;

/** @brief Control-tick rate, from TIM6: 90 MHz / 90 / 1000. Used only to turn
  *        counts-per-window into rpm. */
#define ENCODER_TICK_HZ  1000.0f

/** @brief Enter a critical section that nests correctly — restoring the saved
  *        PRIMASK rather than unconditionally re-enabling interrupts, so
  *        calling from an ISR cannot silently turn them back on. */
static inline uint32_t lock(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static inline void unlock(uint32_t primask)
{
  __set_PRIMASK(primask);
}

void encoder_init(void)
{
  uint32_t primask = lock();

  prev_count    = __HAL_TIM_GET_COUNTER(&htim2);
  position      = 0;
  last_delta    = 0;
  ticks         = 0u;
  rpm           = 0.0f;
  window_counts = 0;
  window_ticks  = 0u;

  unlock(primask);
}

void encoder_on_tick(void)
{
  uint32_t now = __HAL_TIM_GET_COUNTER(&htim2);

  /* Unsigned subtraction wraps modulo 2^32; reinterpreting as signed recovers
     the true movement in either direction. See the header for why this is
     required even on a 32-bit counter. */
  int32_t delta = (int32_t)(now - prev_count);
  prev_count = now;

  position   += delta;
  last_delta  = delta;
  ticks++;

  window_counts += delta;
  window_ticks++;

  if (window_ticks >= window_len)
  {
    /* counts/window -> rev/s -> rpm, in one constant factor. */
    rpm = ((float)window_counts * ENCODER_TICK_HZ * 60.0f)
          / ((float)window_ticks * ENCODER_COUNTS_PER_OUTPUT_REV);

    window_counts = 0;
    window_ticks  = 0u;
  }
}

void encoder_zero(void)
{
  uint32_t primask = lock();

  /* prev_count is deliberately left alone: it tracks the hardware, and
     re-seeding it here would discard whatever the counter did between the
     last tick and this call. */
  position      = 0;
  last_delta    = 0;
  ticks         = 0u;
  rpm           = 0.0f;
  window_counts = 0;
  window_ticks  = 0u;

  unlock(primask);
}

int64_t encoder_position(void)
{
  uint32_t primask = lock();
  int64_t  value   = position;
  unlock(primask);
  return value;
}

int32_t encoder_last_delta(void)
{
  return last_delta;   /* 32-bit, single-copy atomic on Cortex-M4 */
}

float encoder_revolutions(void)
{
  return (float)encoder_position() / ENCODER_COUNTS_PER_OUTPUT_REV;
}

float encoder_rpm(void)
{
  return rpm;
}

uint32_t encoder_raw_count(void)
{
  return __HAL_TIM_GET_COUNTER(&htim2);
}

bool encoder_counting_up(void)
{
  return __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2) == 0u;
}

uint32_t encoder_tick_count(void)
{
  return ticks;
}

void encoder_set_velocity_window(uint16_t new_ticks)
{
  if (new_ticks < 1u)    { new_ticks = 1u;    }
  if (new_ticks > 1000u) { new_ticks = 1000u; }

  uint32_t primask = lock();

  window_len    = new_ticks;
  window_counts = 0;          /* a partial window under the old length would
                                 produce one wrong reading if carried over */
  window_ticks  = 0u;

  unlock(primask);
}

uint16_t encoder_velocity_window(void)
{
  return window_len;
}

void encoder_probe(uint32_t ms, encoder_probe_t *out)
{
  if (out == NULL)
  {
    return;
  }

  uint32_t start_tick  = HAL_GetTick();
  uint32_t start_count = __HAL_TIM_GET_COUNTER(&htim2);

  bool a = (HAL_GPIO_ReadPin(ENC_A_GPIO_Port, ENC_A_Pin) == GPIO_PIN_SET);
  bool b = (HAL_GPIO_ReadPin(ENC_B_GPIO_Port, ENC_B_Pin) == GPIO_PIN_SET);

  out->edges_a = 0u;
  out->edges_b = 0u;
  out->samples = 0u;

  /* Polled rather than interrupt-driven on purpose: this has to keep working
     when the suspicion is that the pins are not configured the way we think
     they are. A tight read of IDR depends on almost nothing. */
  while ((HAL_GetTick() - start_tick) < ms)
  {
    bool now_a = (HAL_GPIO_ReadPin(ENC_A_GPIO_Port, ENC_A_Pin) == GPIO_PIN_SET);
    bool now_b = (HAL_GPIO_ReadPin(ENC_B_GPIO_Port, ENC_B_Pin) == GPIO_PIN_SET);

    if (now_a != a) { out->edges_a++; a = now_a; }
    if (now_b != b) { out->edges_b++; b = now_b; }

    out->samples++;
  }

  out->level_a     = a;
  out->level_b     = b;
  out->count_delta = (int32_t)(__HAL_TIM_GET_COUNTER(&htim2) - start_count);
}

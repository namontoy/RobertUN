/**
  ******************************************************************************
  * @file           : drive.c
  * @brief          : Drive-motor H-bridge control — TIM4 PWM + nSLEEP + nFAULT
  ******************************************************************************
  * The truth table, the decay-mode reasoning and the deliberate omissions are
  * in drive.h. This file is the mechanics.
  ******************************************************************************
  */
#include "drive.h"

#include "main.h"

extern TIM_HandleTypeDef htim4;

static int16_t       duty;                      /*!< signed per-mille, clamped */
static uint16_t      limit = DRIVE_DUTY_MAX;    /*!< magnitude cap             */
static drive_decay_t decay = DRIVE_DECAY_SLOW;
static bool          enabled;

/** @brief Compare value for 100% output. CCR > ARR never matches, so the
  *        channel stays active for the whole period — a true 100%, not
  *        4499/4500. */
#define DRIVE_CCR_FULL  (htim4.Init.Period + 1u)

/** @brief Scale 0..1000 per-mille onto 0..CCR_FULL. */
static uint32_t ccr_from_permille(uint16_t permille)
{
  return ((uint32_t)permille * DRIVE_CCR_FULL) / (uint32_t)DRIVE_DUTY_MAX;
}

/** @brief Write both channels in one place, so no path can leave one stale. */
static void apply(uint32_t ccr1, uint32_t ccr2)
{
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ccr1);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, ccr2);
}

void drive_init(void)
{
  duty    = 0;
  enabled = false;

  apply(0u, 0u);
  HAL_GPIO_WritePin(DRV_nSLEEP_GPIO_Port, DRV_nSLEEP_Pin, GPIO_PIN_RESET);
}

void drive_enable(void)
{
  HAL_GPIO_WritePin(DRV_nSLEEP_GPIO_Port, DRV_nSLEEP_Pin, GPIO_PIN_SET);

  /* Both the DRV8833 and the DRV8874 specify up to 1 ms from nSLEEP rising to
     the outputs being usable. Waiting 2 ms here means a duty command issued
     immediately afterwards is honoured, instead of being silently swallowed by
     a driver that has not finished starting its charge pump. */
  HAL_Delay(2u);

  enabled = true;
}

void drive_disable(void)
{
  drive_coast();
  HAL_GPIO_WritePin(DRV_nSLEEP_GPIO_Port, DRV_nSLEEP_Pin, GPIO_PIN_RESET);
  enabled = false;
}

bool drive_is_enabled(void)
{
  return enabled;
}

void drive_set_duty(int16_t permille)
{
  if (permille >  DRIVE_DUTY_MAX) { permille =  DRIVE_DUTY_MAX; }
  if (permille < -DRIVE_DUTY_MAX) { permille = -DRIVE_DUTY_MAX; }

  if (permille > (int16_t)limit)  { permille =  (int16_t)limit; }
  if (permille < -(int16_t)limit) { permille = -(int16_t)limit; }

  duty = permille;

  if (permille == 0)
  {
    /* Coast, not brake — see drive.h. Slow decay taken literally would hold
       the wheel at zero duty, which is not what "stop" should mean here. */
    apply(0u, 0u);
    return;
  }

  uint16_t magnitude = (uint16_t)((permille < 0) ? -permille : permille);
  bool     forward   = (permille > 0);

  if (decay == DRIVE_DECAY_FAST)
  {
    /* PWM against coast: the driven pin carries the duty, the other sits low. */
    uint32_t ccr = ccr_from_permille(magnitude);
    apply(forward ? ccr : 0u,
          forward ? 0u  : ccr);
  }
  else
  {
    /* PWM against brake: the driven pin sits high and the other is inverted,
       so it is high for the (1 - D) brake fraction of each period. */
    uint32_t ccr = ccr_from_permille((uint16_t)(DRIVE_DUTY_MAX - magnitude));
    apply(forward ? DRIVE_CCR_FULL : ccr,
          forward ? ccr            : DRIVE_CCR_FULL);
  }
}

int16_t drive_duty(void)
{
  return duty;
}

void drive_brake(void)
{
  duty = 0;
  apply(DRIVE_CCR_FULL, DRIVE_CCR_FULL);
}

void drive_coast(void)
{
  duty = 0;
  apply(0u, 0u);
}

bool drive_faulted(void)
{
  return HAL_GPIO_ReadPin(DRV_nFAULT_GPIO_Port, DRV_nFAULT_Pin) == GPIO_PIN_RESET;
}

void drive_set_decay(drive_decay_t new_decay)
{
  decay = new_decay;
  drive_set_duty(duty);   /* re-apply so the change is visible immediately */
}

drive_decay_t drive_decay(void)
{
  return decay;
}

void drive_set_limit(uint16_t permille)
{
  if (permille > DRIVE_DUTY_MAX) { permille = (uint16_t)DRIVE_DUTY_MAX; }

  limit = permille;
  drive_set_duty(duty);   /* a tightened limit takes effect now, not on the
                             next command — otherwise it would not protect
                             against whatever is already running */
}

uint16_t drive_limit(void)
{
  return limit;
}

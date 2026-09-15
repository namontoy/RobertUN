/**
  ******************************************************************************
  * @file           : dipsw.c
  * @brief          : Module identity — 3-bit DIP switch on PB13/PB14/PB15
  ******************************************************************************
  * The electrical convention, the reason 0b111 is the invalid code, and why the
  * halt is deferred are all in dipsw.h. This file is the mechanics.
  ******************************************************************************
  */
#include "dipsw.h"

#include "main.h"

/* PB13 comes from CubeMX. PB14/PB15 do not, and are defined here so this file
   stands alone - but guarded, so that if the .ioc is ever told about them the
   generated main.h definitions win and nothing collides. */
#ifndef DIP_SW_1_Pin
#define DIP_SW_1_Pin        GPIO_PIN_14
#define DIP_SW_1_GPIO_Port  GPIOB
#endif
#ifndef DIP_SW_2_Pin
#define DIP_SW_2_Pin        GPIO_PIN_15
#define DIP_SW_2_GPIO_Port  GPIOB
#endif

static uint8_t latched = DIPSW_CODE_INVALID;   /*!< safe until proven otherwise */
static bool    latched_valid;                  /*!< has dipsw_init() run?       */

/** @brief One switch bit. Pull-up plus switch-to-GND means an open switch reads
  *        high, so the pin level IS the bit - no inversion anywhere. */
static uint8_t bit_of(GPIO_TypeDef *port, uint16_t pin, uint8_t position)
{
  return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET)
         ? (uint8_t)(1u << position) : 0u;
}

uint8_t dipsw_read_live(void)
{
  return (uint8_t)(bit_of(DIP_SW_0_GPIO_Port, DIP_SW_0_Pin, 0u) |
                   bit_of(DIP_SW_1_GPIO_Port, DIP_SW_1_Pin, 1u) |
                   bit_of(DIP_SW_2_GPIO_Port, DIP_SW_2_Pin, 2u));
}

void dipsw_init(void)
{
  if (latched_valid)
  {
    return;   /* the latch is the identity - a second call must not move it */
  }

  /* PB13 is already an input with a pull-up from MX_GPIO_Init(). PB14/PB15 are
     ours; same configuration, so all three bits behave identically. */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef init = {0};
  init.Pin   = DIP_SW_1_Pin | DIP_SW_2_Pin;
  init.Mode  = GPIO_MODE_INPUT;
  init.Pull  = GPIO_PULLUP;
  init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &init);

  /* An internal pull-up is tens of kilohms and has to charge the pin plus
     whatever the switch block and its wiring add. Reading in the same
     microsecond as the HAL_GPIO_Init() above can catch a pin still on its way
     up, which reads as a closed switch that is not closed. One millisecond at
     boot costs nothing and removes the question. */
  HAL_Delay(1u);

  latched       = dipsw_read_live();
  latched_valid = true;
}

uint8_t dipsw_code(void)
{
  return latched;
}

uint8_t dipsw_id(void)
{
  return latched;
}

bool dipsw_valid(void)
{
  return latched_valid && (latched != DIPSW_CODE_INVALID);
}

dipsw_role_t dipsw_role(void)
{
  if (!dipsw_valid())  { return DIPSW_ROLE_INVALID;  }
  if (latched <= 3u)   { return DIPSW_ROLE_CORNER;   }
  if (latched <= 5u)   { return DIPSW_ROLE_CENTER;   }
  return DIPSW_ROLE_RESERVED;
}

uint16_t dipsw_can_id(void)
{
  return (uint16_t)(DIPSW_CAN_ID_BASE + latched);
}

const char *dipsw_role_str(dipsw_role_t role)
{
  switch (role)
  {
    case DIPSW_ROLE_CORNER:   return "corner (steering + drive)";
    case DIPSW_ROLE_CENTER:   return "center (drive only)";
    case DIPSW_ROLE_RESERVED: return "reserved (bench-test)";
    default:                  return "INVALID - identity unconfigured";
  }
}

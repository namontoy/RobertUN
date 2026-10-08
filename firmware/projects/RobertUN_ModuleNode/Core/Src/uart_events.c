/**
  ******************************************************************************
  * @file           : uart_events.c
  * @brief          : Single owner of the HAL UART callbacks, routed by instance
  ******************************************************************************
  *
  * WHY THIS FILE EXISTS
  * --------------------
  * The HAL declares ONE weak callback per event for the entire application —
  * not one per peripheral. Two drivers cannot each define their own copy: the
  * second definition is a duplicate-symbol link error, and if it linked at all
  * only one of them would ever run.
  *
  * So the callbacks live here, once, and fan out by peripheral instance:
  *
  *      HAL_UART_TxCpltCallback    ─┐
  *      HAL_UARTEx_RxEventCallback ─┼─► switch (huart->Instance)
  *      HAL_UART_ErrorCallback     ─┘            │
  *                                               ├─ USART1 ─► debug_uart_on_*()
  *                                               ├─ UART4  ─► mks_on_*()
  *                                               └─ other  ─► ignored
  *
  * Each driver exposes three hooks and knows nothing about the others. Adding
  * a third UART is one more branch here plus three hooks on the new driver;
  * no existing driver changes.
  *
  * These run in interrupt context. Keep the hooks short — the drivers only set
  * flags and advance indices here, deferring real work to their poll functions
  * on the main loop.
  *
  * The alternative would be HAL's registered-callback mechanism
  * (USE_HAL_UART_REGISTER_CALLBACKS=1), which allocates a function pointer per
  * handle. Not used: it costs RAM in every handle and moves the routing to
  * run time for a set of peripherals that is fixed at compile time.
  *
  ******************************************************************************
  */
#include "main.h"

#include "debug_uart.h"
#include "mks_servo.h"

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    debug_uart_on_tx_complete();
  }
  else if (huart->Instance == UART4)
  {
    mks_on_tx_complete();
  }
  else
  {
    /* not ours */
  }
}

/**
  * @brief  Raised on half-transfer, full-transfer and idle-line.
  *
  * Both drivers filter for IDLE themselves — the other two events land
  * mid-stream and would split a reply in two.
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  (void)Size;   /* both drivers read the position from the DMA counter */

  if (huart->Instance == USART1)
  {
    debug_uart_on_rx_event();
  }
  else if (huart->Instance == UART4)
  {
    mks_on_rx_event();
  }
  else
  {
    /* not ours */
  }
}

/**
  * @brief  An overrun/framing/noise/parity error aborts DMA reception, so the
  *         owning driver has to re-arm or its RX stays dead for the rest of
  *         the run — which looks exactly like a wiring fault.
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    debug_uart_on_error();
  }
  else if (huart->Instance == UART4)
  {
    mks_on_error();
  }
  else
  {
    /* not ours */
  }
}

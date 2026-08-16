/**
  ******************************************************************************
  * @file           : debug_uart.c
  * @brief          : Non-blocking DMA console on USART1 (PA9 TX / PA10 RX)
  ******************************************************************************
  * Rationale, data-flow diagram and the CubeMX settings this depends on are in
  * debug_uart.h. This file covers the implementation only.
  *
  * CONCURRENCY MODEL
  * -----------------
  * TX ring: the caller owns `tx_head`, the DMA-completion ISR owns `tx_tail`.
  * One writer each, so reading the other index needs no lock — a 16-bit
  * aligned load is atomic on Cortex-M4. The single genuine race is inside
  * tx_kick(), which both contexts call: without a critical section, two
  * callers can observe "not busy" simultaneously and arm the same DMA stream
  * twice. That check-and-arm is therefore done with interrupts masked. It is a
  * few microseconds at 180 MHz.
  *
  * RX ring: the DMA owns the write position (readable from NDTR), the caller
  * owns `rx_tail`. Nothing is shared mutably, so no locking at all.
  ******************************************************************************
  */
#include "debug_uart.h"

#include "main.h"

#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

#define TX_MASK (DEBUG_UART_TX_BUF_SIZE - 1u)
#define RX_MASK (DEBUG_UART_RX_BUF_SIZE - 1u)

/* Ring sizes must be powers of two for the masking above to be valid. */
_Static_assert((DEBUG_UART_TX_BUF_SIZE & TX_MASK) == 0u, "TX buffer size must be a power of two");
_Static_assert((DEBUG_UART_RX_BUF_SIZE & RX_MASK) == 0u, "RX buffer size must be a power of two");
_Static_assert(DEBUG_UART_RX_BUF_SIZE <= 0xFFFFu, "RX buffer must fit the DMA counter");

/* TX: producer advances head, the DMA completion ISR advances tail. Each index
   has exactly one writer, so no lock is needed to read the other one. */
static uint8_t tx_buf[DEBUG_UART_TX_BUF_SIZE];
static volatile uint16_t tx_head;
static volatile uint16_t tx_tail;
static volatile uint16_t tx_inflight;
static volatile bool tx_busy;

/* RX: the DMA writes continuously; the read index is ours alone. */
static uint8_t rx_buf[DEBUG_UART_RX_BUF_SIZE];
static volatile uint16_t rx_tail;
static volatile bool rx_running;
static volatile bool rx_idle_event;

static debug_uart_stats_t stats;

/* -------------------------------------------------------------------------- */
/* Transmit                                                                    */
/* -------------------------------------------------------------------------- */

/**
  * @brief  Hand the next contiguous run of queued bytes to the DMA.
  *
  * Only the run up to the end of the buffer is submitted; a wrapped region is
  * picked up by the next completion. Runs from both thread and ISR context, so
  * the busy-check and the DMA start have to be atomic with respect to each
  * other — otherwise two callers can arm the same stream twice.
  */
static void tx_kick(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  if (!tx_busy && (tx_head != tx_tail))
  {
    uint16_t tail = tx_tail;
    uint16_t run  = (tx_head > tail) ? (uint16_t)(tx_head - tail)
                                     : (uint16_t)(DEBUG_UART_TX_BUF_SIZE - tail);

    tx_busy     = true;
    tx_inflight = run;

    if (HAL_UART_Transmit_DMA(&huart1, &tx_buf[tail], run) != HAL_OK)
    {
      tx_busy     = false;
      tx_inflight = 0u;
    }
  }

  __set_PRIMASK(primask);
}

size_t debug_uart_write(const void *data, size_t len)
{
  const uint8_t *src = (const uint8_t *)data;
  size_t written = 0u;

  if ((src == NULL) || (len == 0u))
  {
    return 0u;
  }

  while (written < len)
  {
    uint16_t next = (uint16_t)((tx_head + 1u) & TX_MASK);

    /* Leaving one slot empty is what distinguishes full from empty. */
    if (next == tx_tail)
    {
      stats.tx_dropped += (uint32_t)(len - written);
      break;
    }

    tx_buf[tx_head] = src[written];
    tx_head = next;      /* published only after the byte is stored */
    written++;
  }

  stats.tx_bytes += (uint32_t)written;
  tx_kick();
  return written;
}

size_t debug_uart_puts(const char *s)
{
  return (s != NULL) ? debug_uart_write(s, strlen(s)) : 0u;
}

size_t debug_uart_printf(const char *fmt, ...)
{
  char scratch[DEBUG_UART_PRINTF_MAX];
  va_list ap;
  int n;

  va_start(ap, fmt);
  n = vsnprintf(scratch, sizeof(scratch), fmt, ap);
  va_end(ap);

  if (n <= 0)
  {
    return 0u;
  }

  /* vsnprintf reports what it *would* have written; clamp to what it did. */
  if ((size_t)n >= sizeof(scratch))
  {
    n = (int)sizeof(scratch) - 1;
  }

  return debug_uart_write(scratch, (size_t)n);
}

size_t debug_uart_write_hex(const void *data, size_t len)
{
  static const char digits[] = "0123456789ABCDEF";
  const uint8_t *src = (const uint8_t *)data;
  size_t written = 0u;

  for (size_t i = 0u; i < len; i++)
  {
    char pair[3] = { digits[src[i] >> 4], digits[src[i] & 0x0Fu], ' ' };
    written += debug_uart_write(pair, sizeof(pair));
  }

  return written;
}

size_t debug_uart_tx_pending(void)
{
  return (size_t)((tx_head - tx_tail) & TX_MASK) + (tx_busy ? 1u : 0u);
}

bool debug_uart_flush(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();

  while (debug_uart_tx_pending() != 0u)
  {
    if ((HAL_GetTick() - start) >= timeout_ms)
    {
      return false;
    }
  }

  return true;
}

void debug_uart_on_tx_complete(void)
{
  tx_tail     = (uint16_t)((tx_tail + tx_inflight) & TX_MASK);
  tx_inflight = 0u;
  tx_busy     = false;

  tx_kick();   /* pick up anything queued while that transfer was in flight */
}

/* -------------------------------------------------------------------------- */
/* Receive                                                                     */
/* -------------------------------------------------------------------------- */

/**
  * @brief  Where the DMA is currently writing, derived from its own counter.
  *
  * This is what makes reception independent of interrupt latency: the position
  * is a hardware fact, readable at any moment, rather than something a callback
  * has to have run to tell us.
  */
static inline uint16_t rx_head(void)
{
  return (uint16_t)((DEBUG_UART_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx)) & RX_MASK);
}

/** @brief Is DMA reception genuinely still running? Observed from the
  *        hardware, not inferred from HAL's error policy. See mks_servo.c for
  *        the full reasoning — the same handler shape bit us there. */
static bool rx_is_running(void)
{
  return ((huart1.Instance->CR3 & USART_CR3_DMAR) != 0u) &&
         ((hdma_usart1_rx.Instance->CR & DMA_SxCR_EN) != 0u);
}

static bool rx_start(void)
{
  rx_tail = 0u;

  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buf, DEBUG_UART_RX_BUF_SIZE) != HAL_OK)
  {
    rx_running = false;
    return false;
  }

  rx_running = true;
  return true;
}

size_t debug_uart_available(void)
{
  if (!rx_running)
  {
    return 0u;
  }

  return (size_t)((rx_head() - rx_tail) & RX_MASK);
}

size_t debug_uart_read(void *dst, size_t max)
{
  uint8_t *out = (uint8_t *)dst;
  size_t avail = debug_uart_available();
  size_t n     = (avail < max) ? avail : max;

  for (size_t i = 0u; i < n; i++)
  {
    out[i] = rx_buf[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1u) & RX_MASK);
  }

  stats.rx_bytes += (uint32_t)n;
  return n;
}

bool debug_uart_peek(uint8_t *out)
{
  if (debug_uart_available() == 0u)
  {
    return false;
  }

  *out = rx_buf[rx_tail];
  return true;
}

void debug_uart_rx_flush(void)
{
  rx_tail = rx_head();
}

bool debug_uart_take_idle_event(void)
{
  bool had = rx_idle_event;
  rx_idle_event = false;
  return had;
}

/**
  * @brief  Fires on half-transfer, full-transfer, and idle-line.
  *
  * Nothing here is required to capture bytes — the ring is filled by the DMA
  * regardless. Its jobs are to flag the idle event (the frame delimiter) and,
  * because it is guaranteed to run at least every half buffer, to notice a
  * backlog that has grown large enough to be about to overwrite unread data.
  *
  * Only a genuine IDLE means "the sender stopped talking". Half- and
  * full-transfer land mid-stream and would falsely split a reply in two, so
  * they update the bookkeeping but must not raise the frame flag.
  */
void debug_uart_on_rx_event(void)
{
  if (HAL_UARTEx_GetRxEventType(&huart1) == HAL_UART_RXEVENT_IDLE)
  {
    rx_idle_event = true;
  }

  uint32_t backlog = (uint32_t)debug_uart_available();

  if (backlog > stats.rx_high_water)
  {
    stats.rx_high_water = backlog;
  }

  if (backlog >= (DEBUG_UART_RX_BUF_SIZE - 1u))
  {
    stats.rx_overruns++;
  }
}

/**
  * @brief  An overrun/framing/noise/parity error aborts DMA reception, so the
  *         stream has to be re-armed or RX stays dead for the rest of the run —
  *         which looks exactly like a wiring fault.
  */
void debug_uart_on_error(void)
{
  stats.rx_errors++;

  huart1.ErrorCode = HAL_UART_ERROR_NONE;

  if (rx_is_running())
  {
    /* Framing/noise/parity with the transfer still live. Clearing the flags
       here would read DR and steal a byte from the DMA; re-arming would reset
       rx_tail and drop a partly-typed line. Nothing to do. */
    return;
  }

  /* Reception really stopped (overrun or DMA fault). Re-arm — otherwise the
     console goes deaf for the rest of the run, which reads as a dead board. */
  __HAL_UART_CLEAR_PEFLAG(&huart1);   /* one SR-then-DR read clears PE/FE/NE/ORE */

  (void)rx_start();
}

/* -------------------------------------------------------------------------- */
/* Init / diagnostics                                                          */
/* -------------------------------------------------------------------------- */

debug_uart_status_t debug_uart_init(void)
{
  memset(&stats, 0, sizeof(stats));

  tx_head     = 0u;
  tx_tail     = 0u;
  tx_inflight = 0u;
  tx_busy     = false;

  rx_tail       = 0u;
  rx_running    = false;
  rx_idle_event = false;

  /* A non-circular RX stream stops dead at the first idle event and would need
     re-arming from the callback, losing whatever arrived in between. Rather
     than ship that silently, report it and leave TX working so the failure can
     be printed. */
  if (hdma_usart1_rx.Init.Mode != DMA_CIRCULAR)
  {
    return DEBUG_UART_RX_UNAVAILABLE;
  }

  return rx_start() ? DEBUG_UART_OK : DEBUG_UART_RX_UNAVAILABLE;
}

const debug_uart_stats_t *debug_uart_stats(void)
{
  return &stats;
}

void debug_uart_clear_stats(void)
{
  memset(&stats, 0, sizeof(stats));
}

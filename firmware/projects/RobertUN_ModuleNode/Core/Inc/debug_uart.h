/**
  ******************************************************************************
  * @file           : debug_uart.h
  * @brief          : Non-blocking DMA console on USART1 (PA9 TX / PA10 RX)
  ******************************************************************************
  *
  * WHAT THIS IS FOR
  * ----------------
  * The debug probe on this project is an ST-Link/V2 with no virtual COM port,
  * and the WeAct board does not break out SWO. Without this module there is no
  * way to see inside a running node at all — only halt-and-inspect in the
  * debugger, which is useless for anything timing-dependent. This is the
  * project's only window into a live node.
  *
  * Because it is a debug facility, it must never distort what it is observing.
  * That single requirement drives every design decision below: no call blocks,
  * and no data path depends on the application being responsive.
  *
  *
  * DATA FLOW
  * ---------
  *
  *   TX — the caller hands over bytes and returns; the DMA does the waiting
  *   ......................................................................
  *
  *     debug_uart_printf() ─┐
  *     debug_uart_puts()    ├──► [ tx_buf ring, 1 KB ] ──► DMA2_Str7 ──► PA9
  *     debug_uart_write()  ─┘            ▲        │
  *                                       │        └──► USART TC IRQ
  *                                       │                  │
  *                            head (advanced by caller)      ▼
  *                            tail (advanced by ISR) ◄── tx_kick()
  *
  *   Only the contiguous run up to the end of the buffer is submitted per
  *   transfer; a wrapped region is picked up by the next completion.
  *
  *
  *   RX — bytes land whether or not any of our code is running
  *   ......................................................................
  *
  *     PA10 ──► DMA2_Str2 ──► [ rx_buf ring, 512 B, CIRCULAR ] ──► read()
  *                                    ▲                             ▲
  *                       head = SIZE - NDTR                    tail (caller)
  *                       (a hardware fact, readable
  *                        at any instant — no callback
  *                        has to have run)
  *
  *     IDLE line ──► HAL_UARTEx_RxEventCallback ──► rx_idle_event flag
  *                   "the sender stopped talking" = a frame boundary
  *
  *
  * WHY THE RX POSITION COMES FROM THE DMA COUNTER
  * ---------------------------------------------
  * Deriving the write position from NDTR rather than from a callback is what
  * makes reception independent of interrupt latency. Bytes cannot be lost
  * because the CPU was busy in the CAN or PID path — the DMA has already
  * written them, and the position is available whenever anyone asks.
  *
  *
  * WHY IDLE-LINE FRAMING MATTERS
  * -----------------------------
  * The idle line is a hardware frame delimiter: it fires when the wire falls
  * quiet after at least one byte. That is how a reply of unknown length is
  * known to be complete without knowing its length in advance — the exact
  * problem the MKS SERVO42C poses, where one command answers with 8 bytes and
  * another with 3. See mks_servo.h, which uses the same mechanism.
  *
  *
  * CUBEMX SETTINGS THIS DEPENDS ON  (all three fail silently if wrong)
  * ------------------------------------------------------------------
  *   1. USART1 global interrupt ENABLED in NVIC.
  *      HAL_UART_TxCpltCallback is raised from the USART's TC interrupt, not
  *      from the DMA stream interrupt. Without it the TX ring stalls after the
  *      first transfer: the first line of output appears and then nothing,
  *      which reads exactly like a wiring or baud-rate fault. Idle detection
  *      and UART error interrupts are lost too.
  *   2. USART1_RX DMA in CIRCULAR mode.
  *      In Normal mode the stream halts at the first idle event and must be
  *      re-armed from the callback, losing whatever arrives in the gap.
  *      debug_uart_init() checks this and refuses rather than pretending.
  *   3. USART1_TX DMA in NORMAL mode.
  *      Circular TX would retransmit the buffer forever.
  *
  * FLOAT SUPPORT: "%f" needs -Wl,-u,_printf_float at link time. newlib-nano
  * omits float printf by default and prints garbage silently. Already present
  * in CMakeLists.txt.
  *
  ******************************************************************************
  */
#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Both sizes MUST be powers of two — the ring arithmetic masks instead of
   dividing, and RX is additionally bounded by the DMA counter width. */
#define DEBUG_UART_TX_BUF_SIZE   1024u
#define DEBUG_UART_RX_BUF_SIZE   512u

/** Longest single debug_uart_printf() output. Longer output is truncated,
  * never split across calls. */
#define DEBUG_UART_PRINTF_MAX    128u

typedef enum
{
  DEBUG_UART_OK = 0,        /*!< TX and RX both running                       */
  DEBUG_UART_RX_UNAVAILABLE /*!< TX running, RX did not start — see the note
                                 on CIRCULAR mode in the header comment       */
} debug_uart_status_t;

/**
  * @brief Health counters. Nothing here is required for operation; they exist
  *        so that "output looks like it is missing" can be answered with a
  *        number instead of a guess.
  */
typedef struct
{
  uint32_t tx_bytes;      /*!< bytes accepted into the TX ring                */
  uint32_t tx_dropped;    /*!< bytes refused because the TX ring was full     */
  uint32_t rx_bytes;      /*!< bytes handed to the application by read()      */
  uint32_t rx_high_water; /*!< largest unread backlog ever observed           */
  uint32_t rx_overruns;   /*!< times the backlog reached the buffer size      */
  uint32_t rx_errors;     /*!< UART error interrupts (ORE/FE/NE/PE)           */
} debug_uart_stats_t;

/**
  * @brief  Start the console. Call once, after MX_USART1_UART_Init().
  *
  * Resets the counters, clears both rings, and arms the circular receive.
  * Transmission works regardless of the return value, which is deliberate —
  * a failure has to be reportable, and the console is the only way to report
  * anything.
  *
  * @retval DEBUG_UART_OK               TX and RX both running.
  * @retval DEBUG_UART_RX_UNAVAILABLE   The RX DMA stream is not in circular
  *                                     mode, or arming it failed. TX still
  *                                     works; nothing will ever be received.
  */
debug_uart_status_t debug_uart_init(void);

/* ---- Transmit (non-blocking) -------------------------------------------- */

/**
  * @brief  Queue raw bytes for transmission.
  *
  * Copies into the ring and returns immediately — it never waits for the wire,
  * so it is safe from a control loop and from interrupt context.
  *
  * @param  data  bytes to send (not retained; copied)
  * @param  len   how many
  * @retval How many bytes were accepted. A short return means the ring was
  *         full and the remainder was dropped — never silently: the shortfall
  *         is added to stats.tx_dropped. Dropping rather than blocking is the
  *         deliberate choice, because a debug facility that stalls the caller
  *         changes the behaviour it is supposed to be observing.
  */
size_t debug_uart_write(const void *data, size_t len);

/**
  * @brief  Queue a NUL-terminated string. The terminator is not transmitted.
  * @retval Bytes accepted, as for debug_uart_write().
  */
size_t debug_uart_puts(const char *s);

/**
  * @brief  Formatted output into the TX ring.
  *
  * Formats into a stack buffer of DEBUG_UART_PRINTF_MAX bytes, then queues it.
  * Output longer than that is truncated, not split — so a long line arrives
  * short rather than interleaved with something else.
  *
  * @note   "%f" requires -Wl,-u,_printf_float at link time (see header).
  * @retval Bytes accepted, as for debug_uart_write().
  */
size_t debug_uart_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/**
  * @brief  Queue bytes rendered as "DE AD BE EF " — uppercase, space-separated,
  *         with a trailing space.
  *
  * Exists so CAN payloads and MKS packets can be dumped without hand-formatting
  * at every call site, which is where transcription errors creep in.
  * @retval Bytes accepted (3 per input byte when the ring has room).
  */
size_t debug_uart_write_hex(const void *data, size_t len);

/**
  * @brief  Block until the TX ring has drained.
  *
  * Debug and shutdown use only — never call this from a control loop. Its one
  * legitimate use is making sure a final message reaches the wire before a
  * reset, since NVIC_SystemReset() would otherwise cut the DMA mid-transfer.
  *
  * @param  timeout_ms  give up after this long
  * @retval true if drained, false if the timeout elapsed first.
  */
bool debug_uart_flush(uint32_t timeout_ms);

/** @brief Bytes still queued for transmission; 0 means the TX path is idle. */
size_t debug_uart_tx_pending(void);

/* ---- Receive ------------------------------------------------------------ */

/**
  * @brief  Bytes captured by DMA and not yet read.
  *
  * Computed from the DMA counter, so it is accurate even if no interrupt has
  * run since the bytes arrived.
  */
size_t debug_uart_available(void);

/**
  * @brief  Copy up to @p max bytes out of the RX ring, oldest first.
  * @retval Bytes actually copied (0 if nothing was waiting).
  */
size_t debug_uart_read(void *dst, size_t max);

/**
  * @brief  Look at the oldest unread byte without consuming it.
  * @retval true if a byte was available and stored in *out.
  */
bool debug_uart_peek(uint8_t *out);

/** @brief Discard everything currently unread. */
void debug_uart_rx_flush(void);

/**
  * @brief  Consume the "line went idle" flag — the UART's own frame delimiter.
  *
  * Set whenever the line falls quiet after at least one byte. This is how a
  * variable-length reply is known to be complete without knowing its length in
  * advance. Filtered to genuine IDLE events: half- and full-transfer events
  * land mid-stream and would falsely split a frame in two.
  *
  * @retval true if an idle event had occurred since the last call. Reading
  *         clears it, so each idle event is reported exactly once.
  */
bool debug_uart_take_idle_event(void);

/* ---- Diagnostics -------------------------------------------------------- */

/** @brief Borrowed pointer to the live counters — valid for the program's life. */
const debug_uart_stats_t *debug_uart_stats(void);

/** @brief Zero the counters. Useful before a measured run. */
void debug_uart_clear_stats(void);

/* ---- HAL event hooks ---------------------------------------------------- */
/* Called from uart_events.c, which owns the (single, weak) HAL callbacks and
   routes them by peripheral instance. Not for application use — see
   uart_events.c for why the indirection exists. */

/** @brief USART1 transmit-complete: retire the finished run, start the next. */
void debug_uart_on_tx_complete(void);

/** @brief USART1 RX event: raise the idle flag and update backlog counters. */
void debug_uart_on_rx_event(void);

/** @brief USART1 error: clear the sticky flags and re-arm reception. */
void debug_uart_on_error(void);

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_UART_H */

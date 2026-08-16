/**
  ******************************************************************************
  * @file           : console.h
  * @brief          : Line-based command interpreter on the USART1 console
  ******************************************************************************
  *
  * WHAT THIS IS FOR
  * ----------------
  * Turns the board into a bench instrument. Inject arbitrary CAN frames, read
  * the error registers on demand, switch CAN into loopback, drive the steering
  * driver by hand — all without a debugger session and without reflashing.
  *
  * That matters more here than on a typical board: the probe is an ST-Link/V2
  * with no virtual COM port and there is no SWO pin, so the alternative to
  * this is halting the core and reading registers by hand.
  *
  *
  * INPUT PIPELINE
  * --------------
  *
  *   USART1 RX ──► console_poll()
  *                    │
  *                    ├─ printable  ──► line[] buffer, echoed back
  *                    ├─ backspace  ──► erase one, echo "\b \b"
  *                    │
  *                    ├─ CR or LF ────────────┐
  *                    │                       │
  *                    └─ idle line ───────────┤  only when the burst held
  *                       (fallback)           │  more than one byte
  *                                            ▼
  *                                     execute_line()
  *                                            │
  *                                     tokenize(line)
  *                                            │
  *                            argv[0] ──► commands[] ──► handler(argc, argv)
  *                                            │
  *                                       no match ──► "unknown command"
  *
  *   The idle-line fallback exists because a terminal set to send no line
  *   ending (CoolTerm's "Enter Key Emulation = None", among others) produces a
  *   console where commands echo back perfectly and nothing ever runs — a
  *   symptom that looks exactly like a parser bug. The >1 byte guard is what
  *   keeps interactive typing working: a single keystroke arrives alone and
  *   falls quiet, so without it every character would execute as a command.
  *
  *
  * OUTPUT
  * ------
  * Command output and asynchronous output (CAN heartbeat lines, received
  * frames, MKS transaction results) share one wire, so they interleave with
  * whatever is being typed. That is normal for a bare-metal console and is
  * what the `heartbeat off` and `monitor off` toggles are for.
  *
  * Non-blocking throughout: console_poll() consumes whatever bytes have
  * arrived and returns. It never waits for a complete line.
  *
  ******************************************************************************
  */
#ifndef CONSOLE_H
#define CONSOLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/**
  * @brief  Print the banner and the first prompt.
  *
  * Call last in start-up, after debug_uart_init(), can_bus_init() and
  * mks_init(), so that `info` reports live peripheral state and any init
  * failures have already been printed above the prompt.
  */
void console_init(void);

/** @brief Consume pending input, execute any completed line. Call every pass
  *        of the main loop. */
void console_poll(void);

/** @brief Print the outcome of a finished MKS transaction, if one just
  *        completed. Call every main-loop pass, after mks_poll(). */
void console_report_mks(void);

/**
  * @brief  Whether the periodic CAN heartbeat should be sent.
  *
  * Toggled by the `heartbeat` command. Lets the node be silenced while
  * sniffing the bus, or while typing — the main loop still clears its tick
  * flag when muted, so re-enabling waits for the next tick rather than firing
  * immediately on a stale one.
  */
bool console_heartbeat_enabled(void);

/**
  * @brief  Whether received CAN frames should be printed as they arrive.
  *
  * Toggled by the `monitor` command. **Turn this off before any high-rate bus
  * test**: one line per frame at 1000 frames/s vastly exceeds 115200 baud, and
  * the TX ring will simply drop most of it.
  *
  * The receive FIFO is drained either way — muting only suppresses printing,
  * never reception.
  */
bool console_monitor_enabled(void);

#ifdef __cplusplus
}
#endif

#endif /* CONSOLE_H */

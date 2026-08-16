/**
  ******************************************************************************
  * @file           : can_bus.h
  * @brief          : bxCAN bring-up on CAN1 (PB9 TX / PB8 RX) @ 250 kbps
  ******************************************************************************
  *
  * WHAT THIS IS FOR
  * ----------------
  * This node is one of eight on the rover's CAN backbone (six STM32 wheel
  * modules plus the Jetson, plus a CANable sniffer during development). bxCAN
  * implements all of CAN layer 2 in silicon — framing, arbitration, CRC, ACK,
  * fault confinement — so this module is thin by design. It owns only the
  * three things CubeMX does not generate: the acceptance filter, starting the
  * peripheral, and readable access to the error state.
  *
  *
  * SIGNAL PATH
  * -----------
  *
  *   TX                                            RX
  *   ..........................                    ..........................
  *   can_bus_send()                                can_bus_receive()
  *        │                                                ▲
  *        ▼                                                │
  *   [ 3 TX mailboxes ]                          [ FIFO0, only 3 deep ]
  *        │                                                ▲
  *        │ arbitration: lowest ID wins                    │ filter bank 0
  *        │ (loser retries automatically)                  │ ID 0 / mask 0
  *        ▼                                                │ = accept all
  *   PB9 ──► SN65HVD230 ══╗                    ╔══ SN65HVD230 ──► PB8
  *                        ║                    ║
  *      ══════════════════╩════════════════════╩══════════════════
  *          CAN backbone, 250 kbps, 120R at each far end only
  *
  *
  * BIT TIMING  (settled in the .ioc; this module only reads it back)
  * ----------------------------------------------------------------
  *
  *   tq = 1 / (APB1 45 MHz / BRP 12) = 266.67 ns
  *
  *   |<------------------- 1 bit = 15 tq = 4.00 us ------------------>|
  *   | SYNC |            BS1 = 12 tq             |    BS2 = 2 tq      |
  *   | 1 tq |                                    |                    |
  *                                               ^
  *                                        sample point
  *                                     (1 + 12) / 15 = 86.7%
  *
  *   SJW = 2 tq. Not higher because bxCAN's BTR field is 2 bits wide (max 4)
  *   and SJW must also be <= BS2. Orion runs 200 tq from a 50 MHz clock and
  *   lands on the same 250 kbps — the bit rate matches, the tq count does not
  *   have to.
  *
  *
  * TWO TRAPS WORTH KNOWING
  * -----------------------
  *   1. bxCAN receives NOTHING until a filter is configured AND activated.
  *      The reset state is all banks disabled, which presents as "TX works
  *      perfectly, RX is dead" and gets misdiagnosed as wiring or termination.
  *      can_bus_init() handles it; do not remove that call.
  *   2. Error counters read from a node whose transceiver is not connected and
  *      powered are meaningless. A floating CAN_RX produced three different
  *      fault signatures across three runs of identical firmware. See the
  *      "floating CAN_RX lesson" in PROJECT_CONTEXT.md before trusting TEC,
  *      REC or LEC on an unwired bench setup.
  *
  *
  * RX IS POLLED — AND THAT IS STILL AN OPEN DECISION
  * -------------------------------------------------
  * can_bus_receive() is called from the main loop. A load ramp to bus
  * saturation (1,858 frames/s) showed zero FIFO overruns, which bounds the
  * worst-case loop period under 1.6 ms — but that measures throughput only,
  * not latency, and the margin is a property of a loop that is currently
  * almost empty.
  *
  * The API is deliberately context-agnostic: moving to interrupt-driven means
  * enabling CAN1_RX0_IRQn in CubeMX and calling this same can_bus_receive()
  * from HAL_CAN_RxFifo0MsgPendingCallback(). The function body does not
  * change. See the OPEN question in PROJECT_CONTEXT.md; settle it before W6.
  *
  ******************************************************************************
  */
#ifndef CAN_BUS_H
#define CAN_BUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** Heartbeat / telemetry group per the project CAN ID table (0x500–0x5FF).
  * Becomes CAN_ID_HEARTBEAT_BASE + module_id once the DIP switch exists. */
#define CAN_ID_HEARTBEAT_BASE  0x500u

/** @brief One received CAN frame, flattened out of the HAL's split
  *        header/payload representation. */
typedef struct
{
  uint32_t id;        /*!< 11-bit standard, or 29-bit when @ref ext is true  */
  uint8_t  data[8];   /*!< payload; bytes beyond @ref dlc are zero           */
  uint8_t  dlc;       /*!< valid payload length, 0..8                        */
  bool     ext;       /*!< true = 29-bit extended identifier                 */
  bool     rtr;       /*!< true = remote request, carries no data            */
} can_frame_t;

typedef struct
{
  uint32_t tx_frames;    /*!< frames handed to a mailbox                 */
  uint32_t tx_dropped;   /*!< sends refused (all three mailboxes busy)   */
  uint32_t rx_frames;    /*!< frames pulled out of FIFO0                 */
  uint32_t rx_fifo_full; /*!< times FIFO0 reached 3 — margin gone, but
                              nothing lost yet. Leading indicator.       */
  uint32_t rx_overruns;  /*!< overrun *events* — each means at least one
                              frame was lost. See can_bus_receive().     */
} can_bus_stats_t;

/**
  * @brief  Configure the accept-all filter and leave initialisation mode.
  *
  * Call once, after MX_CAN1_Init(). Until this runs the peripheral is not
  * connected to the pins and every incoming frame is discarded by the disabled
  * filter banks.
  *
  * @retval false if the filter could not be configured or the peripheral
  *         refused to start. There is no partial success — treat it as fatal.
  */
bool can_bus_init(void);

/**
  * @brief  Queue a standard-ID data frame into a free TX mailbox.
  *
  * Returns as soon as the frame is handed to hardware; arbitration, retries
  * and ACK checking all happen in silicon afterwards.
  *
  * @param  id    11-bit identifier. Lower value = higher priority.
  * @param  data  payload, may be NULL only when @p len is 0
  * @param  len   0..8
  * @retval false if all three mailboxes are occupied, or the arguments are out
  *         of range. **A run of false here usually means nothing on the bus is
  *         answering**, not that the payload was bad: with automatic
  *         retransmission enabled, an unacknowledged frame holds its mailbox
  *         indefinitely while it retries.
  */
bool can_bus_send(uint32_t id, const void *data, uint8_t len);

/**
  * @brief  Pull one frame out of RX FIFO0.
  *
  * Also samples and clears the FIFO depth flags, so calling it in a
  * drain-until-false loop keeps the overrun counters honest even on the pass
  * that finds the FIFO already empty.
  *
  * @param  frame  destination; untouched when the function returns false
  * @retval false when the FIFO is empty. Call in a while loop to drain.
  */
bool can_bus_receive(can_frame_t *frame);

/**
  * @brief  Switch between CAN_MODE_NORMAL and CAN_MODE_LOOPBACK at runtime.
  *
  * Loopback keeps the peripheral off the wire and self-ACKs, so bit timing,
  * the filter bank and the whole TX/RX path can be proven with **no
  * transceiver, no cable and no second node**. If loopback works and normal
  * mode does not, the fault is downstream of the MCU — which halves the search
  * space before any wiring is touched.
  *
  * Stops, re-initialises and restarts the peripheral, then reinstates the
  * filter. Note this does **not** reset TEC/REC: counters survive a software
  * re-init, so values seen afterwards may predate the mode change.
  *
  * @retval false if any stage of the restart failed.
  */
bool can_bus_set_loopback(bool enable);

/** @brief Whether the peripheral is currently in loopback. */
bool can_bus_is_loopback(void);

/** @brief Live bit timing read back from CAN1->BTR — the silicon's own view,
  *        not the values the source code asked for. Any pointer may be NULL.
  * @param sample_permille Sample point in tenths of a percent (867 = 86.7%). */
void can_bus_get_timing(uint32_t *bitrate, uint32_t *ntq, uint32_t *brp,
                        uint32_t *ts1, uint32_t *ts2, uint32_t *sjw,
                        uint32_t *sample_permille);

/* ---- Error state — the on-chip equivalent of Orion's berr-counter --------
 *
 * How to read these during bring-up:
 *
 *   tec 0, rec 0, lec none        healthy. A zero TEC is proof the ACK came
 *                                 back, so the link is bidirectional even if
 *                                 traffic only went one way.
 *   lec ack                       the frame went out correctly and nothing
 *                                 acknowledged it. A transmitter cannot ACK
 *                                 itself, so: no other node is listening.
 *   lec bit-dominant              transmitted recessive, monitored dominant.
 *                                 CAN_RX held low, TX shorted, or a
 *                                 transceiver holding the bus dominant.
 *   tec parked at exactly 128     a lone node on the bus. The spec exempts an
 *   with passive but not BUS-OFF  error-passive node from further TEC
 *                                 increment on ACK errors, precisely so it
 *                                 cannot take itself bus-off.
 *   tec 0 with BOFF set and       bus-off recovery in progress. Going bus-off
 *   rec counting down             zeroes TEC, and bxCAN reuses REC to count
 *                                 the 128 sequences of 11 recessive bits
 *                                 needed to rejoin. Not a receive problem.
 *
 * LEC is sticky: it holds the last error until a new one overwrites it or
 * software clears it. Read TEC's *trend* for "erroring right now", not LEC.
 */

/** @brief Raw CAN_ESR. Note the accessors below each re-read the register, so
  *        a raw value and a decoded field sampled separately can disagree
  *        while counters are moving. */
uint32_t can_bus_esr(void);

uint8_t     can_bus_tec(void);              /*!< transmit error counter       */
uint8_t     can_bus_rec(void);              /*!< receive error counter        */
uint8_t     can_bus_last_error(void);       /*!< raw LEC field, 0..7          */
const char *can_bus_last_error_str(void);   /*!< LEC decoded, e.g. "ack"      */
bool        can_bus_is_error_warning(void); /*!< a counter passed 96          */
bool        can_bus_is_error_passive(void); /*!< a counter passed 127         */
bool        can_bus_is_bus_off(void);       /*!< TEC passed 255               */

/** @brief Borrowed pointer to the live software counters. TEC/REC are *not*
  *        here — those are hardware-maintained and cannot be written. */
const can_bus_stats_t *can_bus_stats(void);

/** @brief Zero the software counters only. Useful before a measured run. */
void can_bus_clear_stats(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_BUS_H */

/**
  ******************************************************************************
  * @file           : mks_servo.h
  * @brief          : MKS SERVO42C V1.1 UART driver on UART4 (PA0 TX / PA1 RX)
  ******************************************************************************
  *
  * WHAT THIS IS FOR
  * ----------------
  * The SERVO42C is the steering actuator on each of the four corner modules: a
  * NEMA 17 driving a 3D-printed 19:1 cycloidal gearbox. It is UART-only, not
  * CAN-native, which is the entire reason each corner STM32 exists as a
  * bridge — CAN in from Orion, UART out to the driver.
  *
  *      Orion ──CAN 250k──► STM32F446 ──UART 38400──► SERVO42C ──► gearbox
  *                          (this node)                            19:1
  *
  * The link is point-to-point: one MCU, one driver, no shared bus. There is
  * therefore nothing for the address byte to disambiguate, so every unit stays
  * at 0xE0 and the firmware is identical across all four corners. Module
  * identity lives in the CAN ID, never here. That is what makes one binary
  * serve all six modules.
  *
  *
  * PACKET FORMAT
  * -------------
  *
  *   Request                                Reply
  *   ┌──────┬──────┬─ ─ ─ ─┬──────┐         ┌──────┬─ ─ ─ ─ ─┬──────┐
  *   │ 0xE0 │  fn  │ data  │  ck  │         │ 0xE0 │  data   │  ck  │
  *   └──────┴──────┴─ ─ ─ ─┴──────┘         └──────┴─ ─ ─ ─ ─┴──────┘
  *     addr   func   0..n     ▲               addr    0..n      ▲
  *                            │                                │
  *              ck = (sum of ALL preceding bytes) & 0xFF   verified here
  *
  *   The vendor calls `ck` a CRC. It is not — it is a plain 8-bit additive
  *   checksum, address byte included. This driver computes it at runtime
  *   rather than storing per-command constants, because those constants are
  *   the single most error-prone part of this protocol.
  *
  *
  * TRANSACTION STATE MACHINE  (advanced by mks_poll())
  * ---------------------------------------------------
  *
  *                        mks_request()
  *                              │
  *                              ▼
  *                        ┌──────────┐
  *                        │  ST_TX   │  DMA still pushing the request out
  *                        └────┬─────┘
  *                     TX done │
  *                             ▼
  *                     ┌────────────────┐   valid reply,
  *                     │ ST_WAIT_REPLY  ├── not a motion cmd ──┐
  *                     └───┬────────┬───┘                      │
  *      "E0 01 E1" and     │        │  no reply in 100 ms      │
  *      expect_motion      │        └──────────────────────────┤
  *                         ▼                                   │
  *                 ┌────────────────┐    "E0 02 E2"            │
  *                 │ ST_WAIT_MOTION ├──────────────────────────┤
  *                 └────────────────┘    or 20 s timeout       │
  *                                                             ▼
  *                                                       ┌──────────┐
  *                                                       │ ST_IDLE  │
  *                                                       └──────────┘
  *
  *   The two-stage reply is the part of this protocol that is genuinely
  *   unproven on the STM32 side: `FD` answers "accepted" almost immediately
  *   and "complete" only when the motion ends, which on a slow steering move
  *   is seconds later. Treating those as one reply would either time out or
  *   leave a stale frame to corrupt the next transaction.
  *
  *
  * THE DRIVER ECHOES EVERY REQUEST
  * -------------------------------
  * Before answering, the SERVO42C retransmits the bytes it just received. What
  * arrives is therefore:
  *
  *     E0 30 10 | E0 00 00 00 00 00 2A 0A
  *     └─ echo ┘ └──── the actual reply ────┘
  *
  * Confirmed Aug 2026 across two SERVO42C boards and two motors, with
  * connectors and MCU wiring verified — device behaviour, not a wiring loop.
  *
  * Left in place it corrupts everything: the checksum gets computed across
  * both halves and a perfectly good reply is rejected. strip_echo() removes it
  * before validation, counting each one in stats.echoes. The two halves may
  * arrive as a single burst or as two separated by an idle gap; both are
  * handled.
  *
  * A non-zero `echoes` count is normal and expected, not a fault.
  *
  *
  * TRANSPORT
  * ---------
  * Mirrors debug_uart: DMA out, circular DMA in, idle-line framing. The idle
  * line is what makes variable-length replies workable — `E0 30 10` answers
  * with 8 bytes and `E0 F3 01 D4` with 3, and the length is not knowable until
  * the reply has arrived.
  *
  * Nothing blocks. At 38400 a byte is 260 us, so an 8-byte reply is ~2.1 ms of
  * wire time; doing that synchronously would blow the CAN receive margin.
  * Transactions are strictly sequential — one outstanding at a time, enforced
  * by mks_busy() — because the protocol has no way to match a reply to a
  * request other than ordering.
  *
  *
  * BEFORE IT WILL WORK AT ALL
  * --------------------------
  *   - Driver menu: Mode = CR_UART. The factory default CR_vFOC silently
  *     ignores motion commands while still answering reads — so a unit that
  *     reports its encoder but will not move is almost certainly still in
  *     CR_vFOC. This is the first thing to check on a motion timeout.
  *   - UartBaud 38400, UartAddr 0xE0, Mstep 8.
  *   - Wiring: PA0 -> driver RX, PA1 <- driver TX (crossed), common ground.
  *
  ******************************************************************************
  */
#ifndef MKS_SERVO_H
#define MKS_SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** Protocol constant, not an identifier — see the header comment. */
#define MKS_ADDR                   0xE0u

#define MKS_MAX_BODY               12u    /*!< function code + data, no addr/ck */
#define MKS_MAX_FRAME              16u

/** A reply is at most 8 bytes ≈ 2.1 ms of wire time at 38400. 100 ms is far
  * beyond any plausible turnaround and still fast enough to feel instant. */
#define MKS_REPLY_TIMEOUT_MS       100u

/** `FD` answers twice: accepted, then complete when motion ends. A slow move
  * across the full steering range can take seconds. */
#define MKS_MOTION_TIMEOUT_MS      20000u

/** Mstep 8 x 200 steps/rev x 19:1 cycloidal gearbox. 360/30400 = 0.011842 deg
  * per pulse at the output (~43 arcseconds). */
#define MKS_PULSES_PER_OUTPUT_REV  30400u

typedef enum
{
  MKS_RESULT_NONE = 0,       /*!< nothing has completed yet                  */
  MKS_RESULT_OK,
  MKS_RESULT_TIMEOUT,        /*!< no reply at all                            */
  MKS_RESULT_MOTION_TIMEOUT, /*!< accepted, but completion never arrived     */
  MKS_RESULT_BAD_CHECKSUM,
  MKS_RESULT_BAD_ADDR,
  MKS_RESULT_SHORT,          /*!< fewer than 3 bytes — not a valid frame     */
  MKS_RESULT_TX_FAILED,
  MKS_RESULT_ABORTED
} mks_result_t;

typedef struct
{
  uint32_t requests;
  uint32_t replies_ok;
  uint32_t timeouts;
  uint32_t motion_timeouts;
  uint32_t bad_checksum;
  uint32_t bad_addr;
  uint32_t uart_errors;     /*!< HAL_UART_ErrorCallback invocations. One
                                 invocation can carry several flags, so the
                                 breakdown below need not sum to this.     */
  uint32_t echoes;          /*!< requests echoed back by the driver and
                                 discarded — normal, see the note below    */

  /* Which error, broken out — this is what tells you where to look:
       overrun  bytes arrived faster than they were taken from the data
                register. With DMA feeding the transfer this points at a
                DMA/HAL interaction, not at the wire.
       frame    a stop bit was not where it should be — baud mismatch, or a
                line disturbed while idle.
       noise    the three samples inside a bit disagreed — electrical.
       parity   cannot occur here; the link is configured 8N1.
       dma      the DMA controller itself faulted (bus or config error). */
  uint32_t err_overrun;
  uint32_t err_frame;
  uint32_t err_noise;
  uint32_t err_parity;
  uint32_t err_dma;

  /** Times reception was actually re-armed. Far lower than uart_errors is the
    * healthy shape: it means most errors left the transfer running and were
    * handled without disturbing it. */
  uint32_t rx_restarts;
} mks_stats_t;

/**
  * @brief  Start the receiver. Call once, after MX_UART4_Init().
  * @retval false if the UART4_RX DMA stream is not in circular mode, or if
  *         arming reception failed. Nothing will ever be received.
  */
bool mks_init(void);

/**
  * @brief  Advance the transaction state machine. Call every main-loop pass.
  *
  * Does the frame extraction, validation, state transitions and timeout
  * checks. Cheap when idle — a flag test and a switch. All the real work is in
  * hardware; this only reacts to it.
  */
void mks_poll(void);

/**
  * @brief  True while a request is outstanding, including the long wait for a
  *         move to finish. New requests are refused until it clears.
  */
bool mks_busy(void);

/**
  * @brief  Abandon the current transaction and return to idle.
  *
  * For recovering the console when a move is taking longer than expected.
  * **Does not stop the motor** — the driver is still executing whatever it was
  * told to do. Send mks_stop() for that. A completion arriving after an abort
  * is discarded as unsolicited.
  */
void mks_abort(void);

/**
  * @brief  Send an arbitrary request.
  * @param  body       function code followed by its data bytes
  * @param  body_len   length of @p body
  * @param  expect_motion  true for `FD`/`F6`, where an "accepted" reply is
  *                    followed later by a separate "complete" reply
  * @retval false if busy, if @p body_len is out of range, or if the DMA
  *         transmit could not be started.
  */
bool mks_request(const uint8_t *body, uint8_t body_len, bool expect_motion);

/* ---- Read-only diagnostics ----------------------------------------------
 *
 * These answer in ANY driver mode, which makes them the safe way to prove a
 * link before risking motion. mks_read_encoder() is the recommended first
 * command on any bring-up: no movement, no configuration change.
 */

bool mks_read_encoder(void);        /*!< 0x30 -> int32 carry + uint16 value  */
bool mks_read_pulses(void);         /*!< 0x33 -> int32 cumulative pulses     */
bool mks_read_angle_error(void);    /*!< 0x39 -> int16, 65536 = 360 deg      */
bool mks_read_en_status(void);      /*!< 0x3A -> 01 enabled / 02 disabled    */
bool mks_read_protect_state(void);  /*!< 0x3E -> 01 protected / 02 clean     */

/* ---- Motion — requires the driver to be in CR_UART mode -----------------
 *
 * In the factory-default CR_vFOC mode these are accepted by the UART and then
 * ignored, with no error reported. A timeout here while reads still work is
 * the signature.
 */

/** @brief 0xF3 — energise or release the motor. Must be enabled before it will
  *        move, and releasing it lets the steering back-drive under load. */
bool mks_enable(bool on);

/** @brief 0xF7 — stop and hold at the current position. */
bool mks_stop(void);

/**
  * @brief  0xFD — relative move by a pulse count.
  *
  * **Relative, not absolute.** The driver has no position setpoint; it moves
  * by an increment from wherever it currently is. Absolute positioning has to
  * be built on top, either by tracking commanded pulses in software or by
  * reading 0x33 back.
  *
  * @param  ccw     direction. Encoded as bit 7 of the VAL byte, not as a sign.
  * @param  speed   1..127. Use 1-4 for steering — high speeds on a 19:1
  *                 cycloidal gearbox are neither useful nor kind to it.
  * @param  pulses  increment. 1600 = one motor revolution at Mstep 8;
  *                 30400 = one full revolution at the gearbox output.
  * @retval false if busy or if @p speed is out of range.
  *
  * @note   Completion is reported later — see the state machine diagram at the
  *         top of this file.
  */
bool mks_move_pulses(bool ccw, uint8_t speed, uint32_t pulses);

/**
  * @brief  Relative move expressed in degrees at the gearbox OUTPUT.
  *
  * Converts using MKS_PULSES_PER_OUTPUT_REV and rounds to the nearest pulse.
  * Negative is CCW.
  *
  * @retval false if busy, or if the rounded magnitude is zero — one pulse is
  *         0.0118 deg at the output, so anything smaller cannot be commanded.
  */
bool mks_move_degrees(float degrees, uint8_t speed);

/* ---- Results ------------------------------------------------------------ */

/**
  * @brief  True exactly once per completed transaction, then clears.
  *
  * Lets a caller print or act on an outcome without polling mks_busy() and
  * risking either missing a result or reporting the same one twice.
  */
bool          mks_take_completion(void);
mks_result_t  mks_result(void);
const char   *mks_result_str(mks_result_t result);

/** @brief Function code of the last request — lets a caller decode the reply
  *        by what was asked rather than guessing from its length. */
uint8_t       mks_last_function(void);

/** @brief Copy the last reply frame verbatim, address and checksum included. */
size_t        mks_response(uint8_t *dst, size_t max);

/* Decoders for the last reply. All multi-byte fields are big-endian on the
   wire. Each returns false if the stored frame is the wrong length for that
   shape — so pick the decoder from mks_last_function(), not by guessing,
   because different commands produce equal-length replies. */

/** @brief 0x30 reply: 8 bytes. @p carry counts full encoder overflows. */
bool mks_decode_encoder(int32_t *carry, uint16_t *value);

/** @brief 6-byte reply carrying one int32 (0x33 cumulative pulses). */
bool mks_decode_int32(int32_t *value);

/** @brief 4-byte reply carrying one int16 (0x39 shaft angle error). */
bool mks_decode_int16(int16_t *value);

/** @brief 3-byte reply carrying one status byte (0x3A, 0x3E, and the
  *        accepted/complete acknowledgements). */
bool mks_decode_byte(uint8_t *value);

/** @brief Convert a raw 0x39 angle-error reading to degrees at the **motor**
  *        shaft. Divide by 19 for output-referred — see the mixed-unit caveat
  *        in PROJECT_CONTEXT.md before quoting stiffness anywhere. */
float mks_angle_error_degrees(int16_t raw);

const mks_stats_t *mks_stats(void);
void               mks_clear_stats(void);

/* ---- HAL event hooks — called by uart_events.c -------------------------- */
void mks_on_tx_complete(void);
void mks_on_rx_event(void);
void mks_on_error(void);

#ifdef __cplusplus
}
#endif

#endif /* MKS_SERVO_H */

/**
  ******************************************************************************
  * @file           : steer.h
  * @brief          : Absolute steering on top of the relative SERVO42C FD move
  ******************************************************************************
  *
  * The SERVO42C has no position setpoint: FD moves by an increment. CAN STEER
  * (docs/can_cmds.md §4.5) is absolute, so this module keeps the position as
  * the sum of commanded pulses (W6 decision, spec Q3):
  *
  *   - Zero is where the wheel stands at STEER_ENABLE (aligned by hand).
  *   - A target becomes one FD for (target - position).
  *   - The position advances only when an FD reports "complete". A move that
  *     ends any other way (aborted, ESTOP, timeout, link error) leaves the
  *     position unknown: `valid` drops and only a new STEER_ENABLE re-zeroes.
  *   - A target that arrives while a move runs waits for that move to finish,
  *     then one FD goes out for the difference (decided 10-04: defer, so no
  *     move is ever cut short and the sum stays exact). Only the latest
  *     target is kept.
  *
  * Sign: positive degrees = `mks deg +x` = CW (ccw = false), which DECREMENTS
  * the driver's 0x33 counter. So position == -(0x33 now - 0x33 at enable).
  *
  * The servo link carries one transaction at a time and the console uses it
  * too. steer_poll() takes the completion only of a transaction it started
  * (matched with mks_txn_seq()); everything else stays for the console.
  *
  * Main-loop context only. Call steer_poll() after motion_poll() and before
  * console_report_mks().
  ******************************************************************************
  */

#ifndef STEER_H
#define STEER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define STEER_SPEED_DEFAULT  2u       /* MKS speed code when STEER sends 0 */
#define STEER_LIMIT_CDEG     9000     /* ±90.00° at the output (spec Q3)   */

/* STATUS_STEER byte 1 (§4.13). */
#define STEER_F_ENABLED      0x01u
#define STEER_F_MOVING       0x02u    /* a move in flight or a target waiting */
#define STEER_F_STALL        0x04u    /* last move: "complete" never arrived  */
#define STEER_F_POS_VALID    0x08u
#define STEER_F_UART_ERR     0x10u    /* last steer transaction: link error   */

/** @brief STEER_ENABLE / STEER_DISABLE (F3). Enable zeroes the position.
  *        Disable drops a running steer move and invalidates the position,
  *        since a released motor can be back-driven.
  * @retval false if the request could not go out (servo link busy). */
bool    steer_enable(bool on);

bool    steer_enabled(void);
bool    steer_position_valid(void);

/** @brief New absolute target, 0.01° at the output, already range-checked.
  *        Sent at once if the link is free, else after the current move.
  *        Ignored unless enabled and the position is valid. */
void    steer_set_target(int16_t cdeg, uint8_t speed);

int16_t steer_target_cdeg(void);
int16_t steer_position_cdeg(void);
int32_t steer_position_pulses(void);
int32_t steer_target_pulses(void);
uint8_t steer_flags(void);

/** @brief Every servo stop (ESTOP, STOP bit 7): drop a target that is still
  *        waiting. A move in flight is aborted by the stop itself. */
void    steer_cancel(void);

/** @brief The console drove or (de)energised the servo itself: the tracked
  *        position no longer holds. CAN needs STEER_ENABLE again. */
void    steer_external(void);

/** @brief True once per steer transaction that ended in a link error or a
  *        motion timeout (FAULT MKS_ERROR). An abort is not an error. */
bool    steer_take_error(void);

void    steer_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* STEER_H */

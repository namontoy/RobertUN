/**
  ******************************************************************************
  * @file           : motion.h
  * @brief          : Motion guard - the ESTOP latch shared by CAN and console
  ******************************************************************************
  *
  * One place that decides whether a motion command may run, so the CAN command
  * layer and the console cannot disagree about it (docs/can_cmds.md §4.1, §7.3).
  *
  *   motion_estop()        vel off -> drv coast -> mks stop (not on CENTER),
  *                         then latch. Callable any number of times.
  *   motion_estop_clear()  console `estop clear`; refused while the loop is on.
  *   motion_allowed()      false while latched. Console motion handlers and
  *                         can_cmd ask it before acting.
  *
  * The servo stop cannot always go out at once: a steering move keeps
  * mks_busy() true until it completes. motion_poll() keeps retrying - dropping
  * the outstanding transaction first - until the F7 is on the wire.
  *
  * Ownership (§7.3): one owner of motion at a time, NONE / UART / CAN.
 *   motion_may(src)       true if nobody else owns motion. Asked BEFORE a
 *                         motion command acts; claims nothing.
 *   motion_claim(src)     called AFTER the command succeeded: src owns motion.
 *                         A refused command therefore never takes ownership.
 *   motion_release()      every stop, from either side, and every disarm.
 * After boot nobody owns. motion_estop() releases too.
  *
  * Main-loop context only.
  ******************************************************************************
  */

#ifndef MOTION_H
#define MOTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

typedef enum
{
  MOTION_SRC_NONE = 0,
  MOTION_SRC_UART,
  MOTION_SRC_CAN
} motion_src_t;

/** @brief Stop everything and set the ESTOP latch. Acts again when latched. */
void motion_estop(void);

/** @brief True while the ESTOP latch is set. */
bool motion_estop_latched(void);

/**
  * @brief  Clear the ESTOP latch.
  * @retval false if the velocity loop is armed (it cannot be while latched,
  *         but the rule is ARM action 3's and is kept here too).
  */
bool motion_estop_clear(void);

/** @brief True when a motion command may run (not latched). */
bool motion_allowed(void);

/** @brief Who owns motion now (§7.3). */
motion_src_t motion_owner(void);

/** @brief True if @p src may command motion: nobody, or @p src, owns it. */
bool motion_may(motion_src_t src);

/** @brief @p src now owns motion. Call after the motion command succeeded. */
void motion_claim(motion_src_t src);

/** @brief Nobody owns motion. Every stop and disarm, from either side. */
void motion_release(void);

/** @brief Queue an `mks stop` that is retried until it is sent. */
void motion_request_mks_stop(void);

/** @brief Main-loop service: sends a pending servo stop. Call every pass. */
void motion_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_H */

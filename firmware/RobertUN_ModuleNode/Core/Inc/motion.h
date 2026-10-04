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
  * Ownership (UART vs CAN, §7.3) lands here in W6 phase 3.
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

/** @brief Queue an `mks stop` that is retried until it is sent. */
void motion_request_mks_stop(void);

/** @brief Main-loop service: sends a pending servo stop. Call every pass. */
void motion_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_H */

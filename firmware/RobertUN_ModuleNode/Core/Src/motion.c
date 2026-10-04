/**
  ******************************************************************************
  * @file           : motion.c
  * @brief          : Motion guard - see motion.h
  ******************************************************************************
  */

#include "motion.h"

#include "dipsw.h"
#include "drive.h"
#include "mks_servo.h"
#include "velocity.h"

static bool estop_latched;
static bool mks_stop_pending;

void motion_estop(void)
{
  /* Order from the spec: the loop first, or it re-applies duty on its next
     step. velocity_disable() only coasts if the loop was driving, so the
     explicit coast covers a bridge left running by `drv duty`. */
  velocity_disable();
  drive_coast();

  if (dipsw_role() != DIPSW_ROLE_CENTER)
  {
    motion_request_mks_stop();
  }

  estop_latched = true;
}

bool motion_estop_latched(void) { return estop_latched; }

bool motion_estop_clear(void)
{
  if (velocity_enabled())
  {
    return false;
  }

  estop_latched = false;
  return true;
}

bool motion_allowed(void) { return !estop_latched; }

void motion_request_mks_stop(void)
{
  mks_stop_pending = true;
  motion_poll();   /* most of the time it goes out right here */
}

void motion_poll(void)
{
  if (!mks_stop_pending)
  {
    return;
  }

  /* A move holds mks_busy() until the servo reports completion, which can be
     seconds. Drop that transaction so F7 can go out. If the abort lands while
     the previous request is still in DMA, the transmit is refused this pass
     and retried on the next one. */
  if (mks_busy())
  {
    mks_abort();
  }

  if (mks_stop())
  {
    mks_stop_pending = false;
  }
}

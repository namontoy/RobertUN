/**
  ******************************************************************************
  * @file           : steer.c
  * @brief          : Absolute steering on top of the relative FD move - see steer.h
  ******************************************************************************
  */

#include "steer.h"

#include "debug_uart.h"
#include "mks_servo.h"

/* 30400 pulses per output rev / 36000 cdeg = 38/45. */
#define PULSES_NUM  38
#define CDEG_NUM    45

typedef enum
{
  TXN_NONE = 0,
  TXN_ENABLE,
  TXN_DISABLE,
  TXN_MOVE
} txn_kind_t;

static bool       enabled;
static bool       valid = true;      /* zero at boot (spec Q3, W6 decision) */
static bool       pending;           /* a target waits for the link         */
static bool       stall;
static bool       uart_err;
static bool       error_event;
static int32_t    pos;               /* pulses, sum of completed moves      */
static int32_t    target;            /* pulses                              */
static int16_t    target_cdeg;       /* as commanded, echoed in STATUS      */
static uint8_t    speed_code = STEER_SPEED_DEFAULT;
static txn_kind_t txn;
static uint32_t   txn_seq;
static int32_t    move_delta;

static int32_t cdeg_to_pulses(int32_t cdeg)
{
  int32_t n = cdeg * PULSES_NUM;
  return ((n >= 0) ? (n + CDEG_NUM / 2) : (n - CDEG_NUM / 2)) / CDEG_NUM;
}

static int16_t pulses_to_cdeg(int32_t p)
{
  int32_t n = p * CDEG_NUM;
  int32_t c = ((n >= 0) ? (n + PULSES_NUM / 2) : (n - PULSES_NUM / 2)) / PULSES_NUM;

  if (c >  32767) { return  32767; }
  if (c < -32768) { return -32768; }
  return (int16_t)c;
}

static void begin(txn_kind_t k)
{
  txn     = k;
  txn_seq = mks_txn_seq();
}

static bool link_free(void)
{
  return !mks_busy() && !mks_completion_pending();
}

static void start_move(void)
{
  int32_t delta = target - pos;

  pending = false;

  if (delta == 0)
  {
    return;
  }

  /* Positive = CW = ccw false, the same sign as `mks deg`. */
  if (!mks_move_pulses(delta < 0, speed_code,
                       (uint32_t)((delta < 0) ? -delta : delta)))
  {
    pending = true;   /* the link refused it; steer_poll() retries */
    return;
  }

  begin(TXN_MOVE);
  move_delta = delta;
  debug_uart_printf("steer: move %+ld p at speed %u -> target %.2f deg\r\n",
                    (long)delta, (unsigned)speed_code,
                    (double)target_cdeg / 100.0);
}

static void end_txn(mks_result_t r)
{
  txn_kind_t k  = txn;
  bool       ok = (r == MKS_RESULT_OK);

  txn = TXN_NONE;

  if (ok && (k == TXN_MOVE))
  {
    pos += move_delta;
  }
  else if (!ok && (k != TXN_DISABLE))
  {
    /* A move cut short, or an enable that never landed: where the wheel
       stands is no longer the sum of what was commanded. */
    valid   = false;
    pending = false;

    if (k == TXN_ENABLE)
    {
      enabled = false;
    }
  }

  if (r == MKS_RESULT_MOTION_TIMEOUT)
  {
    stall       = true;
    error_event = true;
  }
  else if (!ok && (r != MKS_RESULT_ABORTED))
  {
    uart_err    = true;
    error_event = true;
  }

  if ((k == TXN_MOVE) && ok)
  {
    debug_uart_printf("steer: at %.2f deg (%ld p)\r\n",
                      (double)pulses_to_cdeg(pos) / 100.0, (long)pos);
  }
  else if (!ok)
  {
    static const char *const names[] = { "", "enable", "disable", "move" };

    debug_uart_printf("steer: %s %s%s\r\n", names[k], mks_result_str(r),
                      valid ? "" : " - position lost, STEER_ENABLE to re-zero");
  }
}

bool steer_enable(bool on)
{
  if (on)
  {
    if (!link_free())
    {
      return false;
    }

    if (!mks_enable(true))
    {
      return false;
    }

    begin(TXN_ENABLE);

    /* Optimistic: a STEER right behind the ARM is queued, not refused. If the
       F3 fails, end_txn() takes all of this back. */
    enabled     = true;
    valid       = true;
    pending     = false;
    stall       = false;
    uart_err    = false;
    pos         = 0;
    target      = 0;
    target_cdeg = 0;
    return true;
  }

  /* Disable is stop-like: drop whatever holds the link, ours or not. */
  bool ours = (txn != TXN_NONE);

  if (mks_busy())
  {
    mks_abort();
  }

  if (ours)
  {
    (void)mks_take_completion();
    txn = TXN_NONE;
  }

  enabled = false;
  valid   = false;
  pending = false;

  if (!mks_enable(false))
  {
    return false;
  }

  begin(TXN_DISABLE);
  return true;
}

bool steer_enabled(void)        { return enabled; }
bool steer_position_valid(void) { return valid; }

void steer_set_target(int16_t cdeg, uint8_t speed)
{
  if (!enabled || !valid)
  {
    return;
  }

  target_cdeg = cdeg;
  target      = cdeg_to_pulses(cdeg);
  speed_code  = speed;
  pending     = true;

  if ((txn == TXN_NONE) && link_free())
  {
    start_move();
  }
}

int16_t steer_target_cdeg(void)     { return target_cdeg; }
int16_t steer_position_cdeg(void)   { return pulses_to_cdeg(pos); }
int32_t steer_position_pulses(void) { return pos; }
int32_t steer_target_pulses(void)   { return target; }

uint8_t steer_flags(void)
{
  uint8_t f = 0u;

  if (enabled)                         { f |= STEER_F_ENABLED; }
  if ((txn == TXN_MOVE) || pending)    { f |= STEER_F_MOVING; }
  if (stall)                           { f |= STEER_F_STALL; }
  if (valid)                           { f |= STEER_F_POS_VALID; }
  if (uart_err)                        { f |= STEER_F_UART_ERR; }
  return f;
}

void steer_cancel(void)
{
  pending = false;
}

void steer_external(void)
{
  if (enabled)
  {
    debug_uart_puts("steer: console drove the servo - CAN needs STEER_ENABLE"
                    " (re-zero)\r\n");
  }

  enabled = false;
  valid   = false;
  pending = false;
}

bool steer_take_error(void)
{
  bool e = error_event;
  error_event = false;
  return e;
}

void steer_poll(void)
{
  if (txn != TXN_NONE)
  {
    if (mks_txn_seq() != txn_seq)
    {
      /* Dropped (ESTOP, `mks abort` + a new request): a newer transaction is
         already out and its completion is not ours to take. */
      end_txn(MKS_RESULT_ABORTED);
    }
    else if (!mks_busy())
    {
      (void)mks_take_completion();
      end_txn(mks_result());
    }
    else
    {
      return;
    }
  }

  if (pending && link_free())
  {
    start_move();
  }
}

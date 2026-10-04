/**
  ******************************************************************************
  * @file           : can_cmd.c
  * @brief          : W6 plain-CAN command layer - see can_cmd.h
  ******************************************************************************
  */

#include "can_cmd.h"

#include "main.h"          /* HAL_GetTick */
#include "config.h"
#include "debug_uart.h"
#include "dipsw.h"
#include "drive.h"
#include "encoder.h"
#include "isense.h"
#include "motion.h"
#include "steer.h"
#include "velocity.h"

#define FAULT_CODE_MAX      8u
#define FAULT_MIN_GAP_MS    100u   /* §4.11: at most one per code per 100 ms */

#define SPEED_MAX_MRPM      100000L  /* §4.4, Q4: ±100 rpm */
#define SKIP_FAULT_MIN      5u       /* §5.1: d - 1 >= 5 sends SKIPPED_CTR */
#define STATUS_ISENSE_AVG   16u      /* same depth as telem's T records */
#define STATUS_STEER_MS     100u     /* §4.13: 10 Hz                       */

/* ARM byte 1 (§4.3). */
#define ARM_DISARM          0u
#define ARM_ARM             1u
#define ARM_CLEAR_FAULT     2u
#define ARM_CLEAR_ESTOP     3u
#define ARM_STEER_ENABLE    4u
#define ARM_STEER_DISABLE   5u

/* Counted types ARM..RAMP (0x03-0x07), index = type - CAN_T_ARM. */
#define CTR_FIRST           CAN_T_ARM
#define CTR_TYPES           5u

typedef struct
{
  bool     pending;
  bool     ever_sent;
  uint8_t  flags;
  int16_t  duty;
  uint32_t t_ms;
  uint32_t last_sent_ms;
} fault_slot_t;

/* One §5.1 window: `synced` false means the next frame is taken with any ctr. */
typedef struct
{
  bool    synced;
  uint8_t last;
} ctr_window_t;

typedef enum
{
  CTR_ACCEPT,
  CTR_REPEAT,
  CTR_STALE
} ctr_verdict_t;

static can_cmd_stats_t stats;
static fault_slot_t    faults[FAULT_CODE_MAX + 1u];   /* index = code, 0 unused */
static ctr_window_t    ctr_win[CTR_TYPES][2];         /* [type][0 own, 1 bcast] */
static uint32_t        status_seq;                    /* encoder step last sent */
static bool            wd_was_expired;
static uint32_t        steer_status_ms;               /* last STATUS_STEER  */
static bool            bus_off_was;                   /* §7.2 edges, ESR    */
static bool            passive_was;
static bool            drv_fault_was;
static uint32_t        ring_dropped_last;

/* --- CRC-8/SAE-J1850 ----------------------------------------------------- */

/* Bitwise rather than a 256-byte table: the longest input is 11 bytes. */
static uint8_t crc8_update(uint8_t crc, uint8_t byte)
{
  crc ^= byte;

  for (uint8_t bit = 0u; bit < 8u; bit++)
  {
    crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x1Du) : (uint8_t)(crc << 1);
  }

  return crc;
}

uint8_t can_cmd_crc8(const uint8_t *buf, size_t len)
{
  uint8_t crc = 0xFFu;

  for (size_t i = 0u; i < len; i++)
  {
    crc = crc8_update(crc, buf[i]);
  }

  return (uint8_t)(crc ^ 0xFFu);
}

uint8_t can_cmd_frame_crc(uint16_t id, const uint8_t *payload, size_t len)
{
  uint8_t crc = 0xFFu;

  crc = crc8_update(crc, (uint8_t)(id & 0xFFu));
  crc = crc8_update(crc, (uint8_t)(id >> 8));
  crc = crc8_update(crc, (uint8_t)CAN_PROTO_VER);

  for (size_t i = 0u; i < len; i++)
  {
    crc = crc8_update(crc, payload[i]);
  }

  return (uint8_t)(crc ^ 0xFFu);
}

/* --- transmit ------------------------------------------------------------ */

static uint32_t own_id(uint8_t type)
{
  return ((uint32_t)type << 4) | (uint32_t)dipsw_id();
}

static bool send(uint8_t type, const uint8_t *data, uint8_t len)
{
  /* No identity, no transmit - same gate as the heartbeat. */
  if (!dipsw_valid())
  {
    return false;
  }

  if (!can_bus_send(own_id(type), data, len))
  {
    stats.tx_dropped++;
    return false;
  }

  stats.tx_frames++;
  return true;
}

static void reply(uint8_t type, uint8_t ctr, can_result_t result, uint8_t detail)
{
  const uint8_t d[4] = { type, ctr, (uint8_t)result, detail };

  if (result != CAN_RES_OK)
  {
    stats.rejected++;
  }

  (void)send(CAN_T_CMD_RESULT, d, sizeof(d));
}

/* Snapshot now, send from can_cmd_poll(). Callers raise on a rising edge. */
static void fault_raise(can_fault_t code, uint8_t flags, int16_t duty)
{
  fault_slot_t *s = &faults[code];

  s->pending = true;
  s->flags   = flags;
  s->duty    = duty;
  s->t_ms    = HAL_GetTick();
}

uint8_t can_cmd_flags(void)
{
  bool ramping = drive_slewing() ||
                 (velocity_enabled() &&
                  (velocity_ramped_setpoint() != velocity_setpoint()));

  return (uint8_t)((velocity_enabled()         ? 0x01u : 0u) |
                   (drive_is_enabled()         ? 0x02u : 0u) |
                   (drive_fault_latched()      ? 0x04u : 0u) |
                   (velocity_saturated()       ? 0x08u : 0u) |
                   (velocity_timeout_expired() ? 0x10u : 0u) |
                   (motion_estop_latched()     ? 0x20u : 0u) |
                   (ramping                    ? 0x40u : 0u) |
                   ((motion_owner() == MOTION_SRC_UART) ? 0x80u : 0u));
}

/* --- §5.1 rolling counter ----------------------------------------------- */

static ctr_window_t *ctr_window(uint8_t type, bool bcast)
{
  return &ctr_win[type - CTR_FIRST][bcast ? 1u : 0u];
}

/* Classifies only. The window advances in ctr_commit(), after the frame is
   accepted: a rejected frame never moves the counter (§4). */
static ctr_verdict_t ctr_check(const ctr_window_t *w, uint8_t ctr)
{
  if (!w->synced)
  {
    return CTR_ACCEPT;
  }

  uint8_t d = (uint8_t)(ctr - w->last);

  if (d == 0u)
  {
    return CTR_REPEAT;
  }

  return (d >= 128u) ? CTR_STALE : CTR_ACCEPT;
}

static void ctr_commit(ctr_window_t *w, uint8_t ctr)
{
  if (w->synced)
  {
    uint8_t skipped = (uint8_t)(ctr - w->last - 1u);

    stats.ctr_skipped += skipped;

    if (skipped >= SKIP_FAULT_MIN)
    {
      fault_raise(CAN_FAULT_SKIPPED_CTR, can_cmd_flags(), drive_duty());
    }
  }

  w->synced = true;
  w->last   = ctr;
}

static void ctr_resync(uint8_t type)
{
  ctr_window(type, false)->synced = false;
  ctr_window(type, true)->synced  = false;
}

static void ctr_resync_all(void)
{
  for (uint8_t t = 0u; t < CTR_TYPES; t++)
  {
    ctr_resync((uint8_t)(CTR_FIRST + t));
  }
}

/* --- STATUS_DRIVE -------------------------------------------------------- */

static int16_t clamp_i16(int32_t v)
{
  if (v >  32767) { return  32767; }
  if (v < -32768) { return -32768; }
  return (int16_t)v;
}

static void send_status_drive(void)
{
  /* Asked before reading, as `drv current` does: a zero from the synchronised
     path means "could not measure" as well as "no current". */
  bool     sync = isense_sync_ready();
  uint16_t raw  = sync ? isense_read_sync_avg(STATUS_ISENSE_AVG)
                       : isense_read_avg(STATUS_ISENSE_AVG);
  uint32_t ma   = isense_raw_to_ma(raw);
  int16_t  rpm  = clamp_i16((int32_t)(encoder_rpm() * 100.0f));
  int16_t  out  = drive_duty();
  uint8_t  d[8];

  d[0] = ctr_window(CAN_T_SPEED, false)->last;
  d[1] = can_cmd_flags();
  d[2] = (uint8_t)((uint16_t)rpm & 0xFFu);
  d[3] = (uint8_t)((uint16_t)rpm >> 8);
  d[4] = (uint8_t)((ma > 0xFFFFu) ? 0xFFu : (ma & 0xFFu));
  d[5] = (uint8_t)((ma > 0xFFFFu) ? 0xFFu : ((ma >> 8) & 0xFFu));
  d[6] = (uint8_t)((uint16_t)out & 0xFFu);
  d[7] = (uint8_t)((uint16_t)out >> 8);

  if (send(CAN_T_STATUS_DRIVE, d, sizeof(d)))
  {
    stats.status_tx++;
  }
}

/* Nodes with a steering servo: CORNER, and RESERVED, which acts like one (Q5). */
static bool has_steering(void)
{
  dipsw_role_t r = dipsw_role();
  return (r == DIPSW_ROLE_CORNER) || (r == DIPSW_ROLE_RESERVED);
}

static void send_status_steer(void)
{
  int16_t tgt = steer_target_cdeg();
  int16_t pos = steer_position_cdeg();
  uint8_t d[8];

  d[0] = ctr_window(CAN_T_STEER, false)->last;
  d[1] = steer_flags();
  d[2] = (uint8_t)((uint16_t)tgt & 0xFFu);
  d[3] = (uint8_t)((uint16_t)tgt >> 8);
  d[4] = (uint8_t)((uint16_t)pos & 0xFFu);
  d[5] = (uint8_t)((uint16_t)pos >> 8);
  d[6] = 0u;
  d[7] = 0u;

  if (send(CAN_T_STATUS_STEER, d, sizeof(d)))
  {
    stats.status_tx++;
  }
}

/* --- commands ------------------------------------------------------------ */

static void handle_estop(const can_frame_t *f)
{
  /* Flags and duty as they were when the ESTOP arrived, not after it. */
  uint8_t flags      = can_cmd_flags();
  int16_t duty       = drive_duty();
  bool    was_latched = motion_estop_latched();

  motion_estop();
  ctr_resync_all();
  reply(CAN_T_ESTOP, 0u, CAN_RES_OK, 0u);

  if (!was_latched)
  {
    fault_raise(CAN_FAULT_ESTOP, flags, duty);
  }

  debug_uart_printf("can: ESTOP 0x%03lX - coast, latched ('estop clear' to"
                    " recover)\r\n", (unsigned long)f->id);
}

static void handle_stop(const can_frame_t *f)
{
  /* Acts on the ID: a short frame is a coast, the fail-safe choice. */
  uint8_t ctr  = (f->dlc >= 1u) ? f->data[0] : 0u;
  uint8_t mode = (f->dlc >= 2u) ? f->data[1] : CAN_STOP_COAST;
  uint8_t m    = (uint8_t)(mode & CAN_STOP_MODE_MASK);

  switch (m)
  {
    case CAN_STOP_RAMP:
      /* Loop armed: setpoint 0, the loop stays armed (§4.2). Loop off: the
         bridge's own duty ramp, so a `drv duty` run still stops. */
      if (velocity_enabled())
      {
        velocity_set_setpoint(0);
      }
      else
      {
        drive_set_duty(0);
      }
      break;

    case CAN_STOP_BRAKE:
      velocity_disable();
      drive_brake();
      break;

    default:   /* 1 = coast; 3 is undefined and coasts too */
      velocity_disable();
      drive_coast();
      break;
  }

  motion_release();   /* §7.3: a stop from either side ends ownership */

  if (((mode & CAN_STOP_MKS) != 0u) && (dipsw_role() != DIPSW_ROLE_CENTER))
  {
    motion_request_mks_stop();
  }

  reply(CAN_T_STOP, ctr, CAN_RES_OK, 0u);

  debug_uart_printf("can: STOP mode %u%s ctr %u\r\n",
                    (unsigned)m, ((mode & CAN_STOP_MKS) != 0u) ? "+mks" : "",
                    (unsigned)ctr);
}

/* DLC -> CRC -> counter, shared by every counted frame (§4). On false the
   frame is already answered and must not be acted on. */
static bool frame_checks(const can_frame_t *f, uint8_t type, bool bcast,
                         ctr_window_t **win)
{
  uint8_t ctr = (f->dlc >= 1u) ? f->data[0] : 0u;

  if (f->dlc < 8u)
  {
    reply(type, ctr, CAN_RES_BAD_DLC, 0u);
    return false;
  }

  if (f->data[7] != can_cmd_frame_crc((uint16_t)f->id, f->data, 7u))
  {
    stats.crc_errors++;
    reply(type, ctr, CAN_RES_CRC, 0u);
    return false;
  }

  *win = ctr_window(type, bcast);

  switch (ctr_check(*win, ctr))
  {
    case CTR_REPEAT:
      stats.ctr_repeat++;
      reply(type, ctr, CAN_RES_REPEAT, (*win)->last);
      return false;

    case CTR_STALE:
      stats.ctr_stale++;
      reply(type, ctr, CAN_RES_STALE, (*win)->last);
      return false;

    default:
      return true;
  }
}

static void handle_arm(const can_frame_t *f, bool bcast)
{
  ctr_window_t *win;
  uint8_t       ctr    = f->data[0];
  uint8_t       action = f->data[1];
  can_result_t  res    = CAN_RES_OK;

  if (!frame_checks(f, CAN_T_ARM, bcast, &win))
  {
    return;
  }

  switch (action)
  {
    case ARM_DISARM:
      velocity_disable();
      drive_disable();
      motion_release();
      break;

    case ARM_ARM:
      if (!motion_allowed())
      {
        res = CAN_RES_ESTOP_LATCHED;
      }
      else if (drive_fault_latched())
      {
        res = CAN_RES_FAULT_LATCHED;
      }
      else if (!motion_may(MOTION_SRC_CAN))
      {
        res = CAN_RES_UART_OWNS;
      }
      else
      {
        /* velocity_enable() also clears a latched watchdog (§4.3). */
        drive_enable();
        velocity_enable();
        motion_claim(MOTION_SRC_CAN);
      }
      break;

    case ARM_CLEAR_FAULT:
      drive_clear_fault();
      break;

    case ARM_CLEAR_ESTOP:
      if (!motion_estop_clear())
      {
        res = CAN_RES_BUSY;   /* the loop is on */
      }
      break;

    case ARM_STEER_ENABLE:
    case ARM_STEER_DISABLE:
      if (!has_steering())
      {
        res = CAN_RES_NOT_SUPPORTED;
      }
      else if (!steer_enable(action == ARM_STEER_ENABLE))
      {
        res = CAN_RES_BUSY;   /* the servo link is taken; send it again */
      }
      break;

    default:
      res = CAN_RES_BAD_ACTION;
      break;
  }

  if (res == CAN_RES_OK)
  {
    ctr_commit(win, ctr);
    ctr_resync(CAN_T_SPEED);   /* §4.3: an accepted ARM restarts SPEED/STEER */
    ctr_resync(CAN_T_STEER);
  }

  reply(CAN_T_ARM, ctr, res, 0u);

  debug_uart_printf("can: ARM %u ctr %u -> %u\r\n",
                    (unsigned)action, (unsigned)ctr, (unsigned)res);
}

static void handle_speed(const can_frame_t *f)
{
  ctr_window_t *win;
  uint8_t       ctr = f->data[0];
  int32_t       sp  = (int32_t)((uint32_t)f->data[2]         |
                                ((uint32_t)f->data[3] << 8)  |
                                ((uint32_t)f->data[4] << 16) |
                                ((uint32_t)f->data[5] << 24));
  can_result_t  res = CAN_RES_OK;

  if (!frame_checks(f, CAN_T_SPEED, false, &win))
  {
    return;
  }

  if ((sp > SPEED_MAX_MRPM) || (sp < -SPEED_MAX_MRPM))
  {
    reply(CAN_T_SPEED, ctr, CAN_RES_RANGE, 0x01u);   /* the one field */
    return;
  }

  if (!motion_allowed())
  {
    res = CAN_RES_ESTOP_LATCHED;
  }
  else if (!motion_may(MOTION_SRC_CAN))
  {
    res = CAN_RES_UART_OWNS;   /* checked before NOT_ARMED: say who has it */
  }
  else if (!velocity_enabled() || velocity_timeout_expired())
  {
    /* An expired watchdog needs ARM 1, not just a fresh setpoint (§7.1). */
    res = CAN_RES_NOT_ARMED;
  }

  if (res != CAN_RES_OK)
  {
    reply(CAN_T_SPEED, ctr, res, 0u);
    return;
  }

  ctr_commit(win, ctr);
  velocity_set_setpoint(sp);   /* kicks vel_tmo, as `vel target` does */
  motion_claim(MOTION_SRC_CAN);
  stats.speed_ok++;
  /* No reply on success: ctr comes back in STATUS_DRIVE (§4.4). */
}

static void handle_steer(const can_frame_t *f)
{
  ctr_window_t *win;
  uint8_t       ctr    = f->data[0];
  uint8_t       speed  = f->data[1];
  int16_t       cdeg   = (int16_t)((uint16_t)f->data[2] | ((uint16_t)f->data[3] << 8));
  can_result_t  res    = CAN_RES_OK;
  uint8_t       detail = 0u;

  if (!has_steering())
  {
    reply(CAN_T_STEER, ctr, CAN_RES_NOT_SUPPORTED, 0u);
    return;
  }

  if (!frame_checks(f, CAN_T_STEER, false, &win))
  {
    return;
  }

  /* detail: the field's bit, speed 0x01, angle 0x02. */
  if (speed > 127u)
  {
    res    = CAN_RES_RANGE;
    detail = 0x01u;
  }
  else if ((cdeg > STEER_LIMIT_CDEG) || (cdeg < -STEER_LIMIT_CDEG))
  {
    res    = CAN_RES_RANGE;
    detail = 0x02u;
  }
  else if (!motion_allowed())
  {
    res = CAN_RES_ESTOP_LATCHED;
  }
  else if (!motion_may(MOTION_SRC_CAN))
  {
    res = CAN_RES_UART_OWNS;
  }
  else if (!steer_enabled() || !steer_position_valid())
  {
    /* detail 1: energised, but the position was lost - re-zero with ARM 4. */
    res    = CAN_RES_NOT_ARMED;
    detail = steer_enabled() ? 0x01u : 0x00u;
  }

  if (res == CAN_RES_OK)
  {
    ctr_commit(win, ctr);
    steer_set_target(cdeg, (speed == 0u) ? STEER_SPEED_DEFAULT : speed);
    motion_claim(MOTION_SRC_CAN);
  }

  reply(CAN_T_STEER, ctr, res, detail);   /* always (§4.5) */

  debug_uart_printf("can: STEER ctr %u %.2f deg speed %u -> %u\r\n",
                    (unsigned)ctr, (double)cdeg / 100.0, (unsigned)speed,
                    (unsigned)res);
}

static uint16_t le16(const uint8_t *p)
{
  return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/* LIMITS (§4.6) and RAMP (§4.7) share a layout: ctr, mask, two u16 fields.
   Every field in the mask is checked before anything is applied: an
   out-of-range value rejects the whole frame (RANGE, detail = its mask bit),
   and is never clamped - unlike the console's `drv limit`. Ownership only,
   no latch check, the same gate as the console's limit commands. */
static void handle_limits_ramp(const can_frame_t *f, uint8_t type, bool bcast)
{
  ctr_window_t *win;
  uint8_t       ctr    = f->data[0];
  uint8_t       mask   = f->data[1];
  uint16_t      a      = le16(&f->data[2]);
  uint16_t      b      = le16(&f->data[4]);
  bool          limits = (type == CAN_T_LIMITS);
  can_result_t  res    = CAN_RES_OK;
  uint8_t       detail = 0u;

  if (!frame_checks(f, type, bcast, &win))
  {
    return;
  }

  /* a: duty_limit / ramp_pmps; b: trip_ma / ramp_floor. */
  int32_t a_max = config_max(limits ? CFG_DUTY_LIMIT : CFG_RAMP_PMPS);
  int32_t b_min = limits ? (int32_t)isense_trip_min_ma() : config_min(CFG_RAMP_FLOOR);
  int32_t b_max = limits ? (int32_t)isense_trip_max_ma() : config_max(CFG_RAMP_FLOOR);

  if ((mask == 0u) || ((mask & (uint8_t)~0x03u) != 0u))
  {
    res = CAN_RES_BAD_ACTION;
  }
  else if (((mask & 0x01u) != 0u) && ((int32_t)a > a_max))
  {
    res    = CAN_RES_RANGE;
    detail = 0x01u;
  }
  else if (((mask & 0x02u) != 0u) && (((int32_t)b < b_min) || ((int32_t)b > b_max)))
  {
    res    = CAN_RES_RANGE;
    detail = 0x02u;
  }
  else if (!motion_may(MOTION_SRC_CAN))
  {
    res = CAN_RES_UART_OWNS;
  }
  else if (limits)
  {
    if ((mask & 0x01u) != 0u) { drive_set_limit(a); }
    if ((mask & 0x02u) != 0u) { (void)isense_set_trip_ma(b); }
  }
  else
  {
    if ((mask & 0x01u) != 0u) { drive_set_ramp(a); }
    if ((mask & 0x02u) != 0u) { drive_set_ramp_floor(b); }
  }

  if (res == CAN_RES_OK)
  {
    ctr_commit(win, ctr);
  }

  reply(type, ctr, res, detail);

  if (limits)
  {
    debug_uart_printf("can: LIMITS mask %u ctr %u -> %u (limit %u o/oo, trip"
                      " %lu mA)\r\n", (unsigned)mask, (unsigned)ctr,
                      (unsigned)res, (unsigned)drive_limit(),
                      (unsigned long)isense_trip_ma());
  }
  else
  {
    debug_uart_printf("can: RAMP mask %u ctr %u -> %u (ramp %u o/oo/s, floor"
                      " %u o/oo)\r\n", (unsigned)mask, (unsigned)ctr,
                      (unsigned)res, (unsigned)drive_ramp(),
                      (unsigned)drive_ramp_floor());
  }
}

/* --- CFG_REQ / CFG_RESP (§4.8, §4.9) ------------------------------------ */

static void cfg_resp(uint8_t op, uint8_t key, uint8_t tag, cfg_status_t st,
                     uint32_t value)
{
  const uint8_t d[8] = { op, key, tag, (uint8_t)st,
                         (uint8_t)(value & 0xFFu),
                         (uint8_t)((value >> 8) & 0xFFu),
                         (uint8_t)((value >> 16) & 0xFFu),
                         (uint8_t)(value >> 24) };

  if (st != CFG_ST_OK)
  {
    stats.rejected++;
  }

  (void)send(CAN_T_CFG_RESP, d, sizeof(d));
}

static bool cfg_op_takes_key(uint8_t op)
{
  return (op == CFG_OP_GET) || (op == CFG_OP_SET) || (op == CFG_OP_DEFAULT_KEY) ||
         (op == CFG_OP_GET_MIN) || (op == CFG_OP_GET_MAX) ||
         (op == CFG_OP_GET_DEFAULT);
}

static void handle_cfg(const can_frame_t *f)
{
  uint8_t      op   = (f->dlc >= 1u) ? f->data[0] : 0u;
  uint8_t      key  = (f->dlc >= 2u) ? f->data[1] : 0u;
  uint8_t      tag  = (f->dlc >= 3u) ? f->data[2] : 0u;
  config_key_t k    = (config_key_t)key;
  cfg_status_t st   = CFG_ST_OK;
  uint32_t     out  = 0u;

  /* No counter (§5.1): `tag` pairs request and response. A short frame
     cannot carry a checkable CRC, so it is answered as one that failed it. */
  if (f->dlc < 8u)
  {
    stats.crc_errors++;
    cfg_resp(op, key, tag, CFG_ST_CRC, 0u);
    return;
  }

  /* The CRC sits in byte 3, so the input skips it: bytes 0-2, then 4-7. */
  const uint8_t body[7] = { f->data[0], f->data[1], f->data[2],
                            f->data[4], f->data[5], f->data[6], f->data[7] };

  if (f->data[3] != can_cmd_frame_crc((uint16_t)f->id, body, sizeof(body)))
  {
    stats.crc_errors++;
    cfg_resp(op, key, tag, CFG_ST_CRC, 0u);
    return;
  }

  int32_t value = (int32_t)((uint32_t)f->data[4]         |
                            ((uint32_t)f->data[5] << 8)  |
                            ((uint32_t)f->data[6] << 16) |
                            ((uint32_t)f->data[7] << 24));

  if (op > CFG_OP_GET_DEFAULT)
  {
    cfg_resp(op, key, tag, CFG_ST_BAD_OP, 0u);
    return;
  }

  if (cfg_op_takes_key(op) && (key >= (uint8_t)CFG_KEY_COUNT))
  {
    cfg_resp(op, key, tag, CFG_ST_UNKNOWN_KEY, 0u);
    return;
  }

  switch (op)
  {
    case CFG_OP_GET:
      out = (uint32_t)config_get(k);
      break;

    case CFG_OP_SET:
      /* Rejected, not clamped (config_set()); the reply carries the value
         still in RAM either way. Applied live through the console's path. */
      if (config_set(k, value))
      {
        config_apply_live(k);
      }
      else
      {
        st = CFG_ST_RANGE;
      }
      out = (uint32_t)config_get(k);
      break;

    case CFG_OP_SAVE:
    {
      /* config_save() refuses with the bridge enabled; an armed loop is
         refused here as well, in case the two ever come apart (§4.8). */
      config_save_t r = velocity_enabled() ? CONFIG_SAVE_BUSY : config_save();

      st  = (r == CONFIG_SAVE_BUSY)        ? CFG_ST_BUSY
          : (r == CONFIG_SAVE_FLASH_ERROR) ? CFG_ST_FLASH_ERROR
          :                                  CFG_ST_OK;
      out = (uint32_t)r;
      break;
    }

    case CFG_OP_REVERT:
      out = (uint32_t)config_revert();
      config_apply_all();
      break;

    case CFG_OP_DEFAULT_KEY:
      config_reset_key(k);
      config_apply_all();
      out = (uint32_t)config_get(k);
      break;

    case CFG_OP_DEFAULT_ALL:
      config_reset_all();
      config_apply_all();
      break;

    case CFG_OP_INFO:
    {
      uint16_t used, total;
      config_usage(&used, &total);

      out = (uint32_t)CONFIG_VERSION                          |
            ((uint32_t)CFG_KEY_COUNT << 8)                    |
            ((uint32_t)((used > 255u) ? 255u : used) << 16)   |
            ((uint32_t)(config_dirty() ? 1u : 0u) << 24);
      break;
    }

    case CFG_OP_GET_MIN:     out = (uint32_t)config_min(k);     break;
    case CFG_OP_GET_MAX:     out = (uint32_t)config_max(k);     break;
    default:                 out = (uint32_t)config_default(k); break;
  }

  cfg_resp(op, key, tag, st, out);

  /* Writes are announced on the console; reads are not, so a host polling
     config does not flood it. */
  if ((op >= CFG_OP_SET) && (op <= CFG_OP_DEFAULT_ALL))
  {
    debug_uart_printf("can: CFG op %u key %u -> %u (value %ld)\r\n",
                      (unsigned)op, (unsigned)key, (unsigned)st, (long)(int32_t)out);
  }
}

/* --- dispatch ------------------------------------------------------------ */

static bool is_o2n(uint8_t type)
{
  switch (type)
  {
    case CAN_T_ESTOP:
    case CAN_T_STOP:
    case CAN_T_ARM:
    case CAN_T_SPEED:
    case CAN_T_STEER:
    case CAN_T_LIMITS:
    case CAN_T_RAMP:
    case CAN_T_CFG_REQ:
      return true;

    default:
      return false;
  }
}

void can_cmd_init(void)
{
  stats = (can_cmd_stats_t){ 0 };

  for (uint8_t c = 0u; c <= FAULT_CODE_MAX; c++)
  {
    faults[c] = (fault_slot_t){ 0 };
  }

  ctr_resync_all();
  status_seq      = encoder_velocity_seq();
  wd_was_expired  = velocity_timeout_expired();
  steer_status_ms = HAL_GetTick();

  can_bus_err_t e;
  can_bus_stats_t bs;

  can_bus_errors(&e);
  can_bus_stats_snapshot(&bs);
  bus_off_was       = e.bus_off;
  passive_was       = e.passive;
  drv_fault_was     = drive_fault_latched();
  ring_dropped_last = bs.rx_ring_dropped;
}

void can_cmd_handle(const can_frame_t *f)
{
  if (f->ext || f->rtr)
  {
    stats.ignored++;
    return;
  }

  uint8_t type = (uint8_t)((f->id >> 4) & 0x7Fu);
  uint8_t addr = (uint8_t)(f->id & 0x0Fu);
  bool    bcast = (addr == DIPSW_ADDR_BROADCAST);
  bool    own   = dipsw_valid() && (addr == dipsw_id());

  if (!is_o2n(type) || !(own || bcast))
  {
    stats.ignored++;
    return;
  }

  switch (type)
  {
    case CAN_T_ESTOP:
      stats.handled++;
      handle_estop(f);
      break;

    case CAN_T_STOP:
      stats.handled++;
      handle_stop(f);
      break;

    case CAN_T_ARM:
      if (!dipsw_valid())
      {
        stats.ignored++;   /* no identity: only ESTOP and STOP act */
        break;
      }
      stats.handled++;
      handle_arm(f, bcast);
      break;

    case CAN_T_SPEED:
      /* No broadcast SPEED (§4.4): one setpoint for every wheel is not a
         command anyone means, and six nodes would all answer it. */
      if (!own)
      {
        stats.ignored++;
        break;
      }
      stats.handled++;
      handle_speed(f);
      break;

    case CAN_T_STEER:
      if (!own)   /* no broadcast STEER: every wheel has its own angle */
      {
        stats.ignored++;
        break;
      }
      stats.handled++;
      handle_steer(f);
      break;

    case CAN_T_LIMITS:
    case CAN_T_RAMP:
    case CAN_T_CFG_REQ:
      if (!dipsw_valid())
      {
        stats.ignored++;
        break;
      }
      stats.handled++;
      if (type == CAN_T_CFG_REQ)
      {
        handle_cfg(f);   /* broadcast: every node answers with its own ID */
      }
      else
      {
        handle_limits_ramp(f, type, bcast);
      }
      break;

    default:
      stats.ignored++;
      break;
  }
}

void can_cmd_poll(void)
{
  /* Rising edge of the setpoint watchdog (§7.1). velocity.c has already
     coasted; this only reports it. */
  bool wd = velocity_timeout_expired();

  if (wd && !wd_was_expired)
  {
    fault_raise(CAN_FAULT_VEL_WD_EXPIRED, can_cmd_flags(), drive_duty());
    debug_uart_puts("can: vel_tmo expired - coasted, ARM 1 to recover\r\n");
  }
  wd_was_expired = wd;

  /* §7.2 bus errors, from one ESR read per pass (no SCE interrupt). */
  can_bus_err_t e;

  can_bus_errors(&e);

  if (e.bus_off && !bus_off_was)
  {
    /* Act now, not vel_tmo later. Only a loop CAN armed: a console-owned
       loop is not being driven over this bus. The FAULT goes out on rejoin. */
    if (velocity_enabled() && (motion_owner() == MOTION_SRC_CAN) &&
        !velocity_timeout_expired())
    {
      velocity_expire_now();
      debug_uart_puts("can: bus-off - CAN loop coasted, ARM 1 after rejoin\r\n");
    }
    /* No other print: a shorted bus re-enters bus-off every few ms. The
       count is in `can`. */
    stats.bus_off++;
  }
  else if (!e.bus_off && bus_off_was)
  {
    fault_raise(CAN_FAULT_BUS_OFF_RECOVERED, can_cmd_flags(), drive_duty());
  }
  bus_off_was = e.bus_off;

  if (e.passive && !passive_was)
  {
    stats.passive++;
    fault_raise(CAN_FAULT_ERROR_PASSIVE, can_cmd_flags(), drive_duty());
  }
  passive_was = e.passive;

  /* Ring overflow: any increase. A `stats clear` only resyncs. */
  can_bus_stats_t bs;

  can_bus_stats_snapshot(&bs);
  if (bs.rx_ring_dropped > ring_dropped_last)
  {
    fault_raise(CAN_FAULT_RX_RING_DROPPED, can_cmd_flags(), drive_duty());
  }
  ring_dropped_last = bs.rx_ring_dropped;

  /* nFAULT latch (drive.c samples the pin each tick). Report only; what to
     do about it stays with the caller, as drive.h says. */
  bool drv_fault = drive_fault_latched();

  if (drv_fault && !drv_fault_was)
  {
    fault_raise(CAN_FAULT_DRV_FAULT, can_cmd_flags(), drive_duty());
  }
  drv_fault_was = drv_fault;

  /* STATUS_DRIVE once per control step, phase-locked to it (§4.12). */
  uint32_t seq = encoder_velocity_seq();

  if (seq != status_seq)
  {
    status_seq = seq;
    send_status_drive();
  }

  uint32_t now = HAL_GetTick();

  if (has_steering())
  {
    if (steer_take_error())
    {
      fault_raise(CAN_FAULT_MKS_ERROR, can_cmd_flags(), drive_duty());
    }

    if ((now - steer_status_ms) >= STATUS_STEER_MS)
    {
      steer_status_ms = now;
      send_status_steer();
    }
  }

  for (uint8_t c = 1u; c <= FAULT_CODE_MAX; c++)
  {
    fault_slot_t *s = &faults[c];

    if (!s->pending)
    {
      continue;
    }

    if (!dipsw_valid())
    {
      s->pending = false;   /* never transmitted; nothing to hold on to */
      continue;
    }

    if (s->ever_sent && ((now - s->last_sent_ms) < FAULT_MIN_GAP_MS))
    {
      continue;
    }

    /* Held, not dropped: with no ACK on the bus the mailboxes stay full and
       this would otherwise count a refusal every pass (36 M in one bench
       short, 10-04). It goes out when a mailbox frees. */
    if (can_bus_tx_free() == 0u)
    {
      break;
    }

    uint8_t d[8];

    d[0] = c;
    d[1] = s->flags;
    d[2] = (uint8_t)((uint16_t)s->duty & 0xFFu);
    d[3] = (uint8_t)((uint16_t)s->duty >> 8);
    d[4] = (uint8_t)(s->t_ms);
    d[5] = (uint8_t)(s->t_ms >> 8);
    d[6] = (uint8_t)(s->t_ms >> 16);
    d[7] = (uint8_t)(s->t_ms >> 24);

    if (send(CAN_T_FAULT, d, sizeof(d)))
    {
      s->pending      = false;
      s->ever_sent    = true;
      s->last_sent_ms = now;
    }
  }
}

const can_cmd_stats_t *can_cmd_stats(void) { return &stats; }

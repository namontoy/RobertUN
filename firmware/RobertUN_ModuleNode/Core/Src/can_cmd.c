/**
  ******************************************************************************
  * @file           : can_cmd.c
  * @brief          : W6 plain-CAN command layer - see can_cmd.h
  ******************************************************************************
  */

#include "can_cmd.h"

#include "main.h"          /* HAL_GetTick */
#include "debug_uart.h"
#include "dipsw.h"
#include "drive.h"
#include "motion.h"
#include "velocity.h"

#define FAULT_CODE_MAX      8u
#define FAULT_MIN_GAP_MS    100u   /* §4.11: at most one per code per 100 ms */

typedef struct
{
  bool     pending;
  bool     ever_sent;
  uint8_t  flags;
  int16_t  duty;
  uint32_t t_ms;
  uint32_t last_sent_ms;
} fault_slot_t;

static can_cmd_stats_t stats;
static fault_slot_t    faults[FAULT_CODE_MAX + 1u];   /* index = code, 0 unused */

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

  /* bit 7 (UART owns motion) arrives with ownership, W6 phase 3. */
  return (uint8_t)((velocity_enabled()         ? 0x01u : 0u) |
                   (drive_is_enabled()         ? 0x02u : 0u) |
                   (drive_fault_latched()      ? 0x04u : 0u) |
                   (velocity_saturated()       ? 0x08u : 0u) |
                   (velocity_timeout_expired() ? 0x10u : 0u) |
                   (motion_estop_latched()     ? 0x20u : 0u) |
                   (ramping                    ? 0x40u : 0u));
}

/* --- commands ------------------------------------------------------------ */

static void handle_estop(const can_frame_t *f)
{
  /* Flags and duty as they were when the ESTOP arrived, not after it. */
  uint8_t flags      = can_cmd_flags();
  int16_t duty       = drive_duty();
  bool    was_latched = motion_estop_latched();

  motion_estop();
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

  if (((mode & CAN_STOP_MKS) != 0u) && (dipsw_role() != DIPSW_ROLE_CENTER))
  {
    motion_request_mks_stop();
  }

  reply(CAN_T_STOP, ctr, CAN_RES_OK, 0u);

  debug_uart_printf("can: STOP mode %u%s ctr %u\r\n",
                    (unsigned)m, ((mode & CAN_STOP_MKS) != 0u) ? "+mks" : "",
                    (unsigned)ctr);
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

    default:
      /* ARM, SPEED, STEER, LIMITS, RAMP, CFG_REQ: W6 phases 2-5. A node
         without identity would only ever act on ESTOP and STOP anyway. */
      stats.ignored++;
      break;
  }
}

void can_cmd_poll(void)
{
  uint32_t now = HAL_GetTick();

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

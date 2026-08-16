/**
  ******************************************************************************
  * @file           : mks_servo.c
  * @brief          : MKS SERVO42C V1.1 UART driver on UART4 (PA0 TX / PA1 RX)
  ******************************************************************************
  * Packet format, the transaction state-machine diagram and the driver-side
  * prerequisites (CR_UART mode, 38400, 0xE0, Mstep 8) are in mks_servo.h.
  *
  * STRUCTURE
  * ---------
  *   checksum()        the one arithmetic operation the protocol needs
  *   rx_* helpers      circular-DMA plumbing
  *   ingest()          ring -> accumulator, every poll, never conditional
  *   strip_echo()      remove the driver's echo of our own request
  *   reply_len_for()   expected reply size, keyed on the function code
  *   validate_len()    address and checksum, before anything is believed
  *   mks_request()     build, hand to DMA, arm the state machine
  *   mks_poll()        the state machine itself
  *   wrappers          one per documented command
  *   decoders          big-endian field extraction from the validated reply
  *
  * WHY FRAMING IS NOT DRIVEN BY THE IDLE LINE
  * ------------------------------------------
  * `debug_uart` delimits messages with the hardware IDLE flag, and that is
  * right for a console: a human typing leaves gaps far longer than one
  * character time. It is WRONG here. This driver echoes in software, one byte
  * at a time as it processes them, pausing for longer than a character time
  * *within* a single message — so IDLE fires mid-message and is not a
  * boundary at all.
  *
  * Relying on it cost a bring-up session: an IDLE during transmission caused
  * the first four bytes of an echo to be discarded, leaving `00 00 10 EF ...`
  * — the tail of our own request — to be validated as though it were a reply.
  *
  * So: accumulate unconditionally, and decide a message is complete by its
  * expected length (known for every documented command) or, failing that, by
  * a quiet period long enough to clear this device's inter-byte gaps.
  *
  * WHY A SINGLE TX BUFFER AND NO QUEUE
  * -----------------------------------
  * The protocol has no request/reply tagging — replies are matched to requests
  * purely by ordering. Allowing two in flight would make a reply ambiguous, so
  * the driver enforces one outstanding transaction and a single 16-byte
  * transmit buffer is sufficient. The longest documented packet is 8 bytes.
  ******************************************************************************
  */
#include "mks_servo.h"

#include "main.h"

#include <string.h>

extern UART_HandleTypeDef huart4;
extern DMA_HandleTypeDef hdma_uart4_rx;

#define RX_BUF_SIZE  64u                    /* power of two; replies are <= 8 */
#define RX_MASK      (RX_BUF_SIZE - 1u)

/**
  * Quiet time that ends a reply whose length we cannot predict.
  *
  * Only used for `mks raw` with an undocumented function code. It must exceed
  * this driver's inter-byte gap while echoing, which is why the hardware IDLE
  * flag cannot be used for the job: IDLE fires after one character time
  * (~260 us at 38400) and this device pauses for longer than that *inside* a
  * message.
  */
#define MKS_QUIET_MS 8u

_Static_assert((RX_BUF_SIZE & RX_MASK) == 0u, "RX buffer size must be a power of two");

/** @brief Transaction states. See the diagram in mks_servo.h for transitions.
  *        Only ST_IDLE accepts a new request. */
typedef enum
{
  ST_IDLE = 0,      /* nothing outstanding                             */
  ST_TX,            /* DMA is still pushing the request out            */
  ST_WAIT_REPLY,    /* request sent, waiting for the driver to answer  */
  ST_WAIT_MOTION    /* accepted; waiting for the separate "complete"   */
} state_t;

static uint8_t  tx_buf[MKS_MAX_FRAME];
static uint8_t  tx_len;              /* kept so the echo can be recognised */
static volatile bool tx_busy;

static uint8_t  rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_tail;
static volatile bool rx_running;

static uint8_t  frame[MKS_MAX_FRAME];  /* accumulator, not one burst        */
static uint8_t  frame_len;
static uint8_t  reply_len;             /* validated reply length in frame[]  */
static uint8_t  prev_frame_len;        /* to notice new bytes for quiet-time */
static bool     echo_stripped;         /* strip at most once per transaction */

static volatile state_t state;
static uint32_t deadline;
static uint32_t quiet_deadline;
static uint8_t  last_function;
static uint8_t  expected_len;          /* 0 = unknown, fall back to quiet    */
static bool     expecting_motion;
static bool     completion_pending;
static mks_result_t last_result;

static mks_stats_t stats;

/* -------------------------------------------------------------------------- */
/* Checksum                                                                    */
/* -------------------------------------------------------------------------- */

/**
  * @brief  Plain 8-bit additive checksum over every preceding byte, address
  *         included. Called a CRC in the vendor documentation; it is not one.
  *
  * Computing it here rather than hard-coding per-command constants is
  * deliberate: the constants are the part of this protocol that has been got
  * wrong most often, and a function cannot mis-add.
  */
static uint8_t checksum(const uint8_t *bytes, size_t len)
{
  uint32_t sum = 0u;

  for (size_t i = 0u; i < len; i++)
  {
    sum += bytes[i];
  }

  return (uint8_t)(sum & 0xFFu);
}

/* -------------------------------------------------------------------------- */
/* Receive plumbing                                                            */
/* -------------------------------------------------------------------------- */

/** @brief Where the DMA is currently writing, from its own counter — a
  *        hardware fact readable at any instant, no callback required. */
static inline uint16_t rx_head(void)
{
  return (uint16_t)((RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_uart4_rx)) & RX_MASK);
}

static uint16_t rx_available(void)
{
  if (!rx_running)
  {
    return 0u;
  }

  return (uint16_t)((rx_head() - rx_tail) & RX_MASK);
}

/** @brief Drop anything unread. Called before each request so a late reply
  *        from a previous transaction cannot be mistaken for this one's. */
static void rx_flush(void)
{
  rx_tail = rx_head();
}

/**
  * @brief  Is DMA reception genuinely still running?
  *
  * Observed from the hardware rather than inferred from HAL's error policy:
  * the UART must still be requesting DMA transfers (CR3_DMAR) and the stream
  * must still be enabled (SxCR_EN). HAL clears both when it treats an error as
  * blocking, and leaves both alone when it does not.
  */
static bool rx_is_running(void)
{
  return ((huart4.Instance->CR3 & USART_CR3_DMAR) != 0u) &&
         ((hdma_uart4_rx.Instance->CR & DMA_SxCR_EN) != 0u);
}

static bool rx_start(void)
{
  rx_tail = 0u;

  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rx_buf, RX_BUF_SIZE) != HAL_OK)
  {
    rx_running = false;
    return false;
  }

  rx_running = true;
  return true;
}

/**
  * @brief  Append whatever has arrived to the accumulator.
  *
  * Called on every poll, including while still transmitting. The echo starts
  * coming back before the transmission finishes and arrives with gaps, so
  * bytes must never be dropped on a timing boundary — accumulating
  * unconditionally is what makes the framing immune to how the driver chooses
  * to space its output.
  */
static void ingest(void)
{
  uint16_t n = rx_available();

  while ((frame_len < MKS_MAX_FRAME) && (n > 0u))
  {
    frame[frame_len++] = rx_buf[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1u) & RX_MASK);
    n--;
  }

  /* More than MKS_MAX_FRAME means something is badly out of step; drop the
     excess rather than letting it head the next transaction's reply. */
  if (n > 0u)
  {
    rx_tail = rx_head();
  }
}

/** @brief Drop @p len bytes from the front, keeping anything after them. */
static void consume(uint8_t len)
{
  if (frame_len > len)
  {
    frame_len = (uint8_t)(frame_len - len);
    memmove(frame, &frame[len], frame_len);
  }
  else
  {
    frame_len = 0u;
  }
}

/**
  * @brief  How many bytes this command's reply occupies.
  *
  * Every documented command has a fixed reply size, which makes completion
  * deterministic — no guessing from timing, and no risk of a short frame
  * accidentally checksum-validating as the prefix of a longer one.
  *
  * @retval 0 for an unrecognised function code (reachable only via `mks raw`),
  *         which selects the quiet-time fallback instead.
  */
static uint8_t reply_len_for(uint8_t fn)
{
  switch (fn)
  {
    case 0x30u: return 8u;   /* encoder: addr + int32 carry + uint16 + ck */
    case 0x33u: return 6u;   /* pulses received: addr + int32 + ck        */
    case 0x39u: return 4u;   /* shaft angle error: addr + int16 + ck      */
    case 0x3Au: return 3u;   /* EN pin status                             */
    case 0x3Eu: return 3u;   /* protection state                          */
    case 0x3Fu: return 3u;   /* restore defaults                          */
    case 0xF3u: return 3u;   /* enable                                    */
    case 0xF6u: return 3u;   /* constant speed                            */
    case 0xF7u: return 3u;   /* stop                                      */
    case 0xFDu: return 3u;   /* relative move — the "accepted" ack        */
    default:
      /* Persistent-config writes all acknowledge with E0 01 E1. */
      if (((fn >= 0x81u) && (fn <= 0x8Bu)) ||
          ((fn >= 0xA1u) && (fn <= 0xA5u)))
      {
        return 3u;
      }
      return 0u;
  }
}

/**
  * @brief  Remove the SERVO42C's echo of our own request from the front of the
  *         collected frame.
  *
  * The driver retransmits every byte it receives before answering, so what
  * comes back is [our request][its reply]. Confirmed Aug 2026 against two
  * different SERVO42C boards and two motors, with connectors and MCU wiring
  * verified — it is device behaviour, not a wiring loop.
  *
  * Left in place, the echo turns a valid reply into a malformed frame: the
  * checksum is computed across both halves and fails. Observed as
  * `E0 30 10 | E0 00 00 00 00 00 2A 0A`, where the second half is a perfectly
  * good encoder reading.
  *
  * **The echo arrives with gaps.** This is a software echo — the driver
  * retransmits each byte as it processes it — not a hardware pass-through, so
  * it can pause mid-message for longer than one character time. That is why
  * the accumulator must never be reset on a timing boundary, and why the
  * hardware IDLE flag is not used to delimit anything on this link.
  *
  * Runs at most once per transaction. A no-op on a link that does not echo,
  * so this costs nothing if a future firmware revision drops the behaviour.
  */
static void strip_echo(void)
{
  if (echo_stripped || (tx_len == 0u) || (frame_len < tx_len))
  {
    return;
  }

  if (memcmp(frame, tx_buf, tx_len) != 0)
  {
    return;   /* this device is not echoing, or not echoing verbatim */
  }

  consume(tx_len);
  echo_stripped = true;
  stats.echoes++;
}

/**
  * @brief  Validate address and checksum of the collected frame.
  *
  * A reply always echoes the address in byte 0 and ends with the checksum, so
  * a frame that fails either test is noise or a collision, not a short read.
  */
static mks_result_t validate_len(uint8_t len)
{
  if (len < 3u)
  {
    return MKS_RESULT_SHORT;
  }

  if (frame[0] != MKS_ADDR)
  {
    return MKS_RESULT_BAD_ADDR;
  }

  if (checksum(frame, (size_t)(len - 1u)) != frame[len - 1u])
  {
    return MKS_RESULT_BAD_CHECKSUM;
  }

  return MKS_RESULT_OK;
}

/** @brief `E0 01 E1` — command accepted / run starting. */
static bool frame_is_accepted(void)
{
  return (reply_len == 3u) && (frame[1] == 0x01u);
}

/** @brief `E0 02 E2` — run complete. */
static bool frame_is_complete(void)
{
  return (reply_len == 3u) && (frame[1] == 0x02u);
}

/* -------------------------------------------------------------------------- */
/* Transactions                                                                */
/* -------------------------------------------------------------------------- */

/** @brief Land the transaction: record the outcome, count it, raise the
  *        one-shot completion flag and return to idle. */
static void finish(mks_result_t result)
{
  last_result        = result;
  completion_pending = true;
  state              = ST_IDLE;

  switch (result)
  {
    case MKS_RESULT_OK:             stats.replies_ok++;      break;
    case MKS_RESULT_TIMEOUT:        stats.timeouts++;        break;
    case MKS_RESULT_MOTION_TIMEOUT: stats.motion_timeouts++; break;
    case MKS_RESULT_BAD_CHECKSUM:   stats.bad_checksum++;    break;
    case MKS_RESULT_BAD_ADDR:       stats.bad_addr++;        break;
    default:                                                 break;
  }
}

bool mks_request(const uint8_t *body, uint8_t body_len, bool expect_motion)
{
  if ((body == NULL) || (body_len == 0u) || (body_len > MKS_MAX_BODY))
  {
    return false;
  }

  if (state != ST_IDLE)
  {
    return false;
  }

  rx_flush();

  tx_buf[0] = MKS_ADDR;
  memcpy(&tx_buf[1], body, body_len);
  tx_buf[1u + body_len] = checksum(tx_buf, (size_t)(1u + body_len));

  tx_len = (uint8_t)(body_len + 2u);          /* addr + body + checksum */

  uint16_t len = tx_len;

  tx_busy          = true;
  expecting_motion = expect_motion;
  last_function    = body[0];
  expected_len     = reply_len_for(body[0]);
  frame_len        = 0u;
  reply_len        = 0u;
  prev_frame_len   = 0u;
  echo_stripped    = false;
  quiet_deadline   = HAL_GetTick() + MKS_QUIET_MS;
  state            = ST_TX;

  if (HAL_UART_Transmit_DMA(&huart4, tx_buf, len) != HAL_OK)
  {
    tx_busy = false;
    finish(MKS_RESULT_TX_FAILED);
    return false;
  }

  stats.requests++;
  return true;
}

/**
  * @brief  Advance the state machine: consume any completed frame, then check
  *         the current state's timeout.
  *
  * Frame handling comes first so that a reply arriving in the same pass as its
  * deadline is honoured rather than being declared a timeout.
  *
  * Bytes received while ST_IDLE are discarded — that is a completion arriving
  * after mks_abort(), or line noise, and either way it must not be allowed to
  * head the next transaction's reply.
  */
void mks_poll(void)
{
  uint32_t now = HAL_GetTick();

  if (state == ST_IDLE)
  {
    /* A completion arriving after mks_abort(), or noise. Drop it so it cannot
       head the next transaction's reply. */
    if (rx_available() > 0u)
    {
      rx_tail = rx_head();
    }

    return;
  }

  ingest();
  strip_echo();

  /* Quiet-time bookkeeping, used only when the reply length is unknown. */
  if (frame_len != prev_frame_len)
  {
    prev_frame_len = frame_len;
    quiet_deadline = now + MKS_QUIET_MS;
  }

  if (state == ST_TX)
  {
    if (!tx_busy)
    {
      state    = ST_WAIT_REPLY;
      deadline = now + MKS_REPLY_TIMEOUT_MS;
    }

    return;   /* keep accumulating; the echo is still arriving */
  }

  /* While waiting for a move to finish, the only thing expected is the 3-byte
     completion — the command's own reply length no longer applies. */
  uint8_t want = (state == ST_WAIT_MOTION) ? 3u : expected_len;
  bool    ready;

  if (want > 0u)
  {
    ready = (frame_len >= want);
  }
  else
  {
    /* Unknown reply length (an undocumented code via `mks raw`): the message
       has ended when the line has stayed quiet long enough. */
    ready = (frame_len >= 3u) && ((int32_t)(now - quiet_deadline) >= 0);
    want  = frame_len;
  }

  if (ready)
  {
    reply_len = want;

    mks_result_t valid = validate_len(want);

    if (valid != MKS_RESULT_OK)
    {
      finish(valid);
    }
    else if ((state == ST_WAIT_REPLY) && expecting_motion && frame_is_accepted())
    {
      /* Accepted, not finished. Drop the ack and start over on the
         accumulator — the completion arrives as its own message whenever the
         move ends, which can be seconds later. */
      consume(want);
      prev_frame_len = frame_len;
      reply_len      = 0u;
      state          = ST_WAIT_MOTION;
      deadline       = now + MKS_MOTION_TIMEOUT_MS;
    }
    else if (state == ST_WAIT_MOTION)
    {
      finish(frame_is_complete() ? MKS_RESULT_OK : MKS_RESULT_SHORT);
    }
    else
    {
      finish(MKS_RESULT_OK);
    }
  }
  else if ((int32_t)(now - deadline) >= 0)
  {
    finish((state == ST_WAIT_MOTION) ? MKS_RESULT_MOTION_TIMEOUT
                                     : MKS_RESULT_TIMEOUT);
  }
  else
  {
    /* still accumulating */
  }
}

bool mks_busy(void)
{
  return (state != ST_IDLE);
}

void mks_abort(void)
{
  if (state != ST_IDLE)
  {
    finish(MKS_RESULT_ABORTED);
  }
}

/* -------------------------------------------------------------------------- */
/* Command wrappers                                                            */
/* -------------------------------------------------------------------------- */

bool mks_read_encoder(void)       { const uint8_t b[] = { 0x30u }; return mks_request(b, 1u, false); }
bool mks_read_pulses(void)        { const uint8_t b[] = { 0x33u }; return mks_request(b, 1u, false); }
bool mks_read_angle_error(void)   { const uint8_t b[] = { 0x39u }; return mks_request(b, 1u, false); }
bool mks_read_en_status(void)     { const uint8_t b[] = { 0x3Au }; return mks_request(b, 1u, false); }
bool mks_read_protect_state(void) { const uint8_t b[] = { 0x3Eu }; return mks_request(b, 1u, false); }

bool mks_enable(bool on)
{
  const uint8_t b[] = { 0xF3u, (uint8_t)(on ? 0x01u : 0x00u) };
  return mks_request(b, 2u, false);
}

bool mks_stop(void)
{
  const uint8_t b[] = { 0xF7u };
  return mks_request(b, 1u, false);
}

/**
  * @brief  0xFD — relative move.
  *
  * Wire layout: FD [VAL] [uint32 pulses, big-endian], where VAL packs
  * direction into bit 7 and speed into bits 6-0. So 0x02 is CW at speed 2 and
  * 0x82 is CCW at the same speed — the direction is a bit, never a sign on the
  * pulse count, which is unsigned.
  */
bool mks_move_pulses(bool ccw, uint8_t speed, uint32_t pulses)
{
  /* VAL byte: bit7 = direction (0 CW, 1 CCW), bits6-0 = speed. */
  if ((speed == 0u) || (speed > 0x7Fu))
  {
    return false;
  }

  const uint8_t b[] =
  {
    0xFDu,
    (uint8_t)((ccw ? 0x80u : 0x00u) | speed),
    (uint8_t)(pulses >> 24),
    (uint8_t)(pulses >> 16),
    (uint8_t)(pulses >> 8),
    (uint8_t)pulses
  };

  return mks_request(b, (uint8_t)sizeof(b), true);
}

bool mks_move_degrees(float degrees, uint8_t speed)
{
  bool  ccw = (degrees < 0.0f);
  float mag = ccw ? -degrees : degrees;

  float pulses_f = (mag * (float)MKS_PULSES_PER_OUTPUT_REV) / 360.0f;

  if (pulses_f > 4.0e9f)
  {
    return false;
  }

  uint32_t pulses = (uint32_t)(pulses_f + 0.5f);

  if (pulses == 0u)
  {
    return false;   /* below one pulse — 0.0118 deg at the output */
  }

  return mks_move_pulses(ccw, speed, pulses);
}

/* -------------------------------------------------------------------------- */
/* Results                                                                     */
/* -------------------------------------------------------------------------- */

bool mks_take_completion(void)
{
  bool had = completion_pending;
  completion_pending = false;
  return had;
}

mks_result_t mks_result(void)
{
  return last_result;
}

uint8_t mks_last_function(void)
{
  return last_function;
}

const char *mks_result_str(mks_result_t result)
{
  switch (result)
  {
    case MKS_RESULT_NONE:            return "none";
    case MKS_RESULT_OK:              return "ok";
    case MKS_RESULT_TIMEOUT:         return "timeout (no reply)";
    case MKS_RESULT_MOTION_TIMEOUT:  return "timeout (move never completed)";
    case MKS_RESULT_BAD_CHECKSUM:    return "bad checksum";
    case MKS_RESULT_BAD_ADDR:        return "bad address";
    case MKS_RESULT_SHORT:           return "malformed reply";
    case MKS_RESULT_TX_FAILED:       return "transmit failed";
    case MKS_RESULT_ABORTED:         return "aborted";
    default:                         return "?";
  }
}

size_t mks_response(uint8_t *dst, size_t max)
{
  size_t n = (reply_len < max) ? reply_len : max;

  memcpy(dst, frame, n);
  return n;
}

bool mks_decode_encoder(int32_t *carry, uint16_t *value)
{
  /* E0 [int32 carry] [uint16 value] [ck] */
  if (reply_len != 8u)
  {
    return false;
  }

  *carry = (int32_t)(((uint32_t)frame[1] << 24) | ((uint32_t)frame[2] << 16) |
                     ((uint32_t)frame[3] << 8)  |  (uint32_t)frame[4]);
  *value = (uint16_t)(((uint16_t)frame[5] << 8) | frame[6]);
  return true;
}

bool mks_decode_int32(int32_t *value)
{
  if (reply_len != 6u)
  {
    return false;
  }

  *value = (int32_t)(((uint32_t)frame[1] << 24) | ((uint32_t)frame[2] << 16) |
                     ((uint32_t)frame[3] << 8)  |  (uint32_t)frame[4]);
  return true;
}

bool mks_decode_int16(int16_t *value)
{
  if (reply_len != 4u)
  {
    return false;
  }

  *value = (int16_t)(((uint16_t)frame[1] << 8) | frame[2]);
  return true;
}

bool mks_decode_byte(uint8_t *value)
{
  if (reply_len != 3u)
  {
    return false;
  }

  *value = frame[1];
  return true;
}

float mks_angle_error_degrees(int16_t raw)
{
  return ((float)raw / 65536.0f) * 360.0f;
}

const mks_stats_t *mks_stats(void)
{
  return &stats;
}

void mks_clear_stats(void)
{
  memset(&stats, 0, sizeof(stats));
}

/* -------------------------------------------------------------------------- */
/* Init and HAL hooks                                                          */
/* -------------------------------------------------------------------------- */

bool mks_init(void)
{
  memset(&stats, 0, sizeof(stats));

  tx_busy            = false;
  tx_len             = 0u;
  frame_len          = 0u;
  reply_len          = 0u;
  prev_frame_len     = 0u;
  echo_stripped      = false;
  expected_len       = 0u;
  state              = ST_IDLE;
  expecting_motion   = false;
  completion_pending = false;
  last_result        = MKS_RESULT_NONE;

  if (hdma_uart4_rx.Init.Mode != DMA_CIRCULAR)
  {
    return false;
  }

  return rx_start();
}

void mks_on_tx_complete(void)
{
  tx_busy = false;
}

/**
  * @brief  Deliberately does nothing.
  *
  * This link does not use the IDLE line to delimit messages — see "WHY FRAMING
  * IS NOT DRIVEN BY THE IDLE LINE" at the top of this file. Reception needs no
  * callback at all: the circular DMA keeps filling regardless, and mks_poll()
  * reads the write position straight from the DMA counter.
  *
  * The hook is kept so uart_events.c has a symmetric entry per UART, and so
  * that anyone looking for the idle handler finds this explanation instead of
  * its absence.
  */
void mks_on_rx_event(void)
{
}

void mks_on_error(void)
{
  uint32_t code = huart4.ErrorCode;

  stats.uart_errors++;

  /* A single invocation can carry more than one flag, so these are counted
     independently rather than as a switch. */
  if ((code & HAL_UART_ERROR_ORE) != 0u) { stats.err_overrun++; }
  if ((code & HAL_UART_ERROR_FE)  != 0u) { stats.err_frame++;   }
  if ((code & HAL_UART_ERROR_NE)  != 0u) { stats.err_noise++;   }
  if ((code & HAL_UART_ERROR_PE)  != 0u) { stats.err_parity++;  }
  if ((code & HAL_UART_ERROR_DMA) != 0u) { stats.err_dma++;     }

  huart4.ErrorCode = HAL_UART_ERROR_NONE;

  if (rx_is_running())
  {
    /* Framing, noise or parity with the transfer still live. Two things must
       NOT happen here:
         - clearing the sticky flags, because the clear macro reads DR and
           would consume a byte the DMA is entitled to;
         - re-arming, because rx_start() resets rx_tail and would discard a
           transaction already in flight.
       HAL has already dealt with the flag; there is nothing to do. */
    return;
  }

  /* Reception really has stopped — overrun or a DMA fault, which HAL treats
     as blocking and aborts. Re-arming is now both necessary and safe, and DR
     can be read without competing with the DMA.

     Doing this unconditionally is what let a single seed error turn into a
     cascade: each re-arm could flag another error, which re-armed again. 180
     callbacks across 7 transactions were observed that way on Aug 13, 2026. */
  __HAL_UART_CLEAR_PEFLAG(&huart4);   /* one SR-then-DR read clears PE/FE/NE/ORE */

  stats.rx_restarts++;
  (void)rx_start();
}

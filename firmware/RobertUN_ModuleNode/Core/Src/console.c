/**
  ******************************************************************************
  * @file           : console.c
  * @brief          : Line-based command interpreter on the USART1 console
  ******************************************************************************
  * Input pipeline diagram and the terminal line-ending story are in console.h.
  *
  * ADDING A COMMAND
  * ----------------
  * Write a `static void cmd_x(int argc, char **argv)` and add one row to the
  * commands[] table near the bottom. argv[0] is the command name itself, so a
  * bare invocation has argc == 1. The table is also what `help` prints, so the
  * usage and help strings there are the only documentation a user sees —
  * keep them accurate.
  *
  * Handlers must not block: they run inside console_poll(), which runs in the
  * main loop alongside the CAN drain and the MKS state machine. Anything
  * long-running should start an operation and report its outcome later, the
  * way the `mks` command does via console_report_mks().
  ******************************************************************************
  */
#include "console.h"

#include "can_bus.h"
#include "debug_uart.h"
#include "main.h"
#include "mks_servo.h"
#include "steer.h"
#include "encoder.h"
#include "drive.h"
#include "velocity.h"
#include "isense.h"
#include "config.h"
#include "dipsw.h"
#include "motion.h"
#include "can_cmd.h"

#include <stdlib.h>
#include <string.h>

#define CONSOLE_LINE_MAX    96u
#define CONSOLE_MAX_TOKENS  12u
#define CONSOLE_PROMPT      "> "

static char   line[CONSOLE_LINE_MAX];
static size_t line_len;
static size_t burst_len;            /* bytes since the last idle boundary */
static bool   skip_lf;              /* CRLF terminals send both; act on one */

/* Off at boot. The heartbeat prints a line every 0.5 s, which drowns out the
   drive-motor bench output — `enc watch` runs at 5 Hz and W5's PID telemetry
   will be denser still. Turn it on with `heartbeat on` when the bus is what
   is actually being tested. */
static bool heartbeat_on = false;
static bool monitor_on   = true;
static bool enc_watch_on = false;
static uint32_t enc_watch_last;

/* Telemetry stream. Off at boot: it is a machine format that would make an
   interactive session unreadable, and it costs an ADC conversion per line. */
static bool     telem_on   = false;
static uint16_t telem_ms   = 20u;   /* period, not rate - see cmd_telem()     */
static uint32_t telem_next;         /* HAL_GetTick() at which the next is due */
static uint32_t telem_seq;          /* +1 per line; gaps = dropped lines      */

/* The velocity channel. A SECOND record type on the same wire rather than more
   columns on T: node.py's Telem.parse() rejects any line without exactly eight
   fields, and every committed run directory holds a telemetry.csv with the
   seven-field header, so a widened T would mean two incompatible things
   depending on which reader saw it. A new letter breaks nothing.

   It is gated behind telem_on as well as its own switch, so `telem off` - what
   every host stop sequence already sends - remains a complete stop. */
static bool     telem_vel_on = false;
static uint32_t telem_vseq;         /* +1 per V line; restarts with telem on  */

/** @brief Fastest stream the wire can carry. A ~55-byte line at 100 Hz is
  *        5.5 kB/s against 11.52 kB/s at 115200 8N1 - under half. At 200 Hz it
  *        is ~95%, where the TX ring stops keeping up and lines vanish into
  *        stats.tx_dropped instead of reaching the host. */
#define TELEM_MAX_HZ      100u
#define TELEM_MIN_HZ        1u

/** @brief ADC samples per telemetry line. Deliberately far below
  *        ISENSE_SYNC_AVG_DEFAULT (64): each sample costs one 50 us PWM period,
  *        so 64 would be 3.2 ms of every 10 ms line at 100 Hz. Sixteen is
  *        0.8 ms. The noise this gives up is bought back on the host, which can
  *        average 50 lines a second - far more than 64 samples one line at a
  *        time. Sample fast and thin, average off-board. */
#define TELEM_ISENSE_SAMPLES  16u

/** @brief Command handler. @p argv[0] is the command name, so a bare
  *        invocation arrives with @p argc == 1. Tokens point into the mutable
  *        line buffer and are only valid for the duration of the call. */
typedef void (*cmd_fn_t)(int argc, char **argv);

/** @brief One row of the dispatch table. @ref args and @ref help are printed
  *        verbatim by `help`, and are the only user-facing documentation. */
typedef struct
{
  const char *name;   /*!< exact match, no prefixes or abbreviations */
  const char *args;   /*!< argument summary, e.g. "<id> [hex]"       */
  const char *help;   /*!< one-line description                      */
  cmd_fn_t    fn;
} command_t;

/* -------------------------------------------------------------------------- */
/* Parsing helpers                                                             */
/* -------------------------------------------------------------------------- */

/** @brief Hex digit value, or -1 if @p c is not one. */
static int hex_digit(char c)
{
  if ((c >= '0') && (c <= '9')) { return c - '0'; }
  if ((c >= 'a') && (c <= 'f')) { return (c - 'a') + 10; }
  if ((c >= 'A') && (c <= 'F')) { return (c - 'A') + 10; }
  return -1;
}

/**
  * @brief  Parse a whole token as hex, with an optional "0x" prefix.
  * @retval false on any non-hex character, an empty token, or overflow past
  *         32 bits. @p *out is untouched unless the parse succeeds.
  */
static bool parse_hex_u32(const char *s, uint32_t *out)
{
  uint32_t value = 0u;

  if ((s[0] == '0') && ((s[1] == 'x') || (s[1] == 'X')))
  {
    s += 2;
  }

  if (*s == '\0')
  {
    return false;
  }

  for (; *s != '\0'; s++)
  {
    int digit = hex_digit(*s);

    if ((digit < 0) || (value > (0xFFFFFFFFu >> 4)))
    {
      return false;
    }

    value = (value << 4) | (uint32_t)digit;
  }

  *out = value;
  return true;
}

/**
  * @brief  Collect a CAN payload from the remaining tokens.
  *
  * Digits are taken as one continuous stream, so "DEADBEEF", "DE AD BE EF" and
  * "DEAD BEEF" are all the same eight bytes — whichever is easier to type.
  */
static bool parse_hex_bytes(int argc, char **argv, uint8_t *out, uint8_t *out_len,
                            size_t max)
{
  size_t count = 0u;
  int    high  = -1;

  for (int i = 0; i < argc; i++)
  {
    for (const char *p = argv[i]; *p != '\0'; p++)
    {
      int digit = hex_digit(*p);

      if (digit < 0)
      {
        return false;
      }

      if (high < 0)
      {
        high = digit;
      }
      else
      {
        if (count >= max)
        {
          return false;
        }

        out[count++] = (uint8_t)(((uint32_t)high << 4) | (uint32_t)digit);
        high = -1;
      }
    }
  }

  if (high >= 0)
  {
    return false;   /* odd number of digits — a half byte was typed */
  }

  *out_len = (uint8_t)count;
  return true;
}

/**
  * @brief  Split @p s into tokens in place, overwriting separators with NUL.
  *
  * Runs of spaces and tabs are collapsed, and leading/trailing whitespace is
  * ignored — so "help " and "help" tokenize identically.
  *
  * @retval Token count, capped at @p max. @p s is modified.
  */
static int tokenize(char *s, char **argv, int max)
{
  int argc = 0;

  for (;;)
  {
    while ((*s == ' ') || (*s == '\t'))
    {
      s++;
    }

    if ((*s == '\0') || (argc >= max))
    {
      break;
    }

    argv[argc++] = s;

    while ((*s != '\0') && (*s != ' ') && (*s != '\t'))
    {
      s++;
    }

    if (*s != '\0')
    {
      *s = '\0';
      s++;
    }
  }

  return argc;
}

/** @brief Accept "on"/"off" (and "1"/"0"), leaving *out untouched otherwise. */
static bool parse_on_off(const char *s, bool *out)
{
  if ((strcmp(s, "on") == 0) || (strcmp(s, "1") == 0))
  {
    *out = true;
    return true;
  }

  if ((strcmp(s, "off") == 0) || (strcmp(s, "0") == 0))
  {
    *out = false;
    return true;
  }

  return false;
}

/** @brief Ownership half of the gate (§7.3): refused while CAN owns motion.
  *        On its own for the limit commands, which never latch-check. */
static bool owner_gate(void)
{
  if (motion_may(MOTION_SRC_UART))
  {
    return true;
  }

  debug_uart_puts("CAN owns motion - 'vel off' first\r\n");
  return false;
}

/** @brief Gate for console motion commands (can_cmds.md §4.1, §7.3). Prints
  *        why. The handler calls motion_claim() once the command has acted. */
static bool motion_gate(void)
{
  if (!motion_allowed())
  {
    debug_uart_puts("ESTOP latched - refused. 'estop clear' first\r\n");
    return false;
  }

  return owner_gate();
}

/** @brief CAN fault-confinement state as a word, worst condition first. */
static const char *err_state_str(const can_bus_err_t *e)
{
  if (e->bus_off) { return "BUS-OFF"; }
  if (e->passive) { return "error-passive"; }
  if (e->warning) { return "error-warning"; }
  return "error-active";
}

static const char *can_state_str(void)
{
  can_bus_err_t e;

  can_bus_errors(&e);
  return err_state_str(&e);
}

/* -------------------------------------------------------------------------- */
/* Commands                                                                    */
/* -------------------------------------------------------------------------- */

static void cmd_help(int argc, char **argv);   /* needs the table below */

/**
  * @brief  Clocks, live CAN bit timing, and current modes.
  *
  * The bit timing is read back out of CAN1->BTR rather than printed from the
  * source constants — this reports what the silicon is actually doing, so a
  * CubeMX regeneration that quietly resets a field shows up here in one
  * command instead of as intermittent bus errors weeks later.
  */
static void cmd_info(int argc, char **argv)
{
  uint32_t bitrate, ntq, brp, ts1, ts2, sjw, sample;

  (void)argc;
  (void)argv;

  can_bus_get_timing(&bitrate, &ntq, &brp, &ts1, &ts2, &sjw, &sample);

  debug_uart_printf("firmware  : %s %s\r\n", __DATE__, __TIME__);
  debug_uart_printf("SYSCLK    : %lu Hz\r\n", HAL_RCC_GetSysClockFreq());
  debug_uart_printf("HCLK      : %lu Hz\r\n", HAL_RCC_GetHCLKFreq());
  debug_uart_printf("APB1      : %lu Hz  (bxCAN)\r\n", HAL_RCC_GetPCLK1Freq());
  debug_uart_printf("APB2      : %lu Hz  (USART1)\r\n", HAL_RCC_GetPCLK2Freq());
  debug_uart_printf("CAN1      : %lu bps, %lu tq (BRP %lu, BS1 %lu, BS2 %lu, SJW %lu)\r\n",
                    bitrate, ntq, brp, ts1, ts2, sjw);
  debug_uart_printf("            sample point %lu.%lu%%\r\n", sample / 10u, sample % 10u);
  debug_uart_printf("CAN mode  : %s, %s\r\n",
                    can_bus_is_loopback() ? "LOOPBACK" : "normal", can_state_str());
  debug_uart_printf("heartbeat : %s, ID 0x%03lX%s\r\n",
                    heartbeat_on ? "on" : "off",
                    (unsigned long)dipsw_can_id(),
                    dipsw_valid() ? "" : "  (NOT SENT - no module identity)");
  debug_uart_printf("monitor   : %s\r\n", monitor_on ? "on" : "off");
}

static void cmd_stats(int argc, char **argv)
{
  const debug_uart_stats_t *u = debug_uart_stats();
  can_bus_stats_t           snap;
  const can_bus_stats_t    *c = &snap;

  (void)argc;
  (void)argv;

  can_bus_stats_snapshot(&snap);

  debug_uart_printf("uart tx   : %lu bytes, %lu dropped\r\n", u->tx_bytes, u->tx_dropped);
  debug_uart_printf("uart rx   : %lu bytes, high-water %lu, %lu overruns, %lu errors\r\n",
                    u->rx_bytes, u->rx_high_water, u->rx_overruns, u->rx_errors);
  debug_uart_printf("can  tx   : %lu frames, %lu dropped (no free mailbox)\r\n",
                    c->tx_frames, c->tx_dropped);
  debug_uart_printf("can  rx   : %lu frames, %lu FIFO-full, %lu overrun events\r\n",
                    c->rx_frames, c->rx_fifo_full, c->rx_overruns);
  debug_uart_printf("can  ring : %lu dropped, high-water %lu of %lu\r\n",
                    c->rx_ring_dropped, c->rx_ring_hwm, (unsigned long)CAN_RX_RING_SIZE);

  if (c->rx_ring_dropped != 0u)
  {
    debug_uart_puts("  warning : the RX ring was full - frames were discarded (counted,\r\n"
                    "            never blocked). The main loop is not draining fast enough.\r\n");
  }

  if (c->rx_overruns != 0u)
  {
    debug_uart_puts("  warning : FIFO0 overran - frames were lost. The counter is\r\n"
                    "            events, not frames; the hardware cannot say how many.\r\n");
  }
  else if (c->rx_fifo_full != 0u)
  {
    debug_uart_puts("  note    : FIFO0 hit its 3-message depth but nothing was lost -\r\n"
                    "            the drain loop is keeping up, with no margin to spare.\r\n");
  }
}

static void cmd_errors(int argc, char **argv)
{
  can_bus_err_t e;

  (void)argc;
  (void)argv;

  can_bus_errors(&e);   /* one ESR read; every line below decodes this value */

  debug_uart_printf("CAN_ESR   : 0x%08lX\r\n", e.esr);
  debug_uart_printf("  TEC     : %u\r\n", e.tec);
  debug_uart_printf("  REC     : %u\r\n", e.rec);
  debug_uart_printf("  last err: %s\r\n", can_bus_lec_str(e.lec));
  debug_uart_printf("  state   : %s\r\n", err_state_str(&e));

  if (e.lec == 3u)
  {
    debug_uart_puts("  note    : 'ack' means the frame went out but no other node\r\n"
                    "            acknowledged it - a transmitter cannot ACK itself.\r\n");
  }
}

static void cmd_clear(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  debug_uart_clear_stats();
  can_bus_clear_stats();
  debug_uart_puts("software counters cleared (TEC/REC are hardware-managed)\r\n");
}

/**
  * @brief  Transmit one standard-ID CAN frame.
  *
  * Payload digits are taken as a single stream, so "DEADBEEF", "DE AD BE EF"
  * and "DEAD BEEF" are the same four bytes. Odd digit counts and payloads over
  * 8 bytes are rejected rather than silently truncated.
  */
static void cmd_send(int argc, char **argv)
{
  uint32_t id;
  uint8_t  payload[8] = {0};
  uint8_t  len = 0u;

  if (argc < 2)
  {
    debug_uart_puts("usage: send <id> [hex bytes]   e.g. send 123 DEADBEEF\r\n");
    return;
  }

  if (!parse_hex_u32(argv[1], &id) || (id > 0x7FFu))
  {
    debug_uart_printf("bad id '%s' - expected 11-bit hex, 000..7FF\r\n", argv[1]);
    return;
  }

  if ((argc > 2) && !parse_hex_bytes(argc - 2, &argv[2], payload, &len, sizeof(payload)))
  {
    debug_uart_puts("bad payload - expected up to 8 bytes as hex digit pairs\r\n");
    return;
  }

  if (can_bus_send(id, payload, len))
  {
    debug_uart_printf("sent 0x%03lX [%u] ", (unsigned long)id, len);
    debug_uart_write_hex(payload, len);
    debug_uart_puts("\r\n");
  }
  else
  {
    debug_uart_printf("send failed - no free mailbox (state %s, last err %s)\r\n",
                      can_state_str(), can_bus_last_error_str());
  }
}

static void cmd_heartbeat(int argc, char **argv)
{
  if ((argc >= 2) && !parse_on_off(argv[1], &heartbeat_on))
  {
    debug_uart_puts("usage: heartbeat [on|off]\r\n");
    return;
  }

  debug_uart_printf("heartbeat %s\r\n", heartbeat_on ? "on" : "off");

  /* Say it here, not only in the per-frame line. Turning the heartbeat on and
     watching nothing appear on the bus is a confusing way to discover that the
     board has no identity. */
  if (heartbeat_on && !dipsw_valid())
  {
    debug_uart_puts("  (nothing will be sent - no module identity, see 'id')\r\n");
  }
}

static void cmd_monitor(int argc, char **argv)
{
  if ((argc >= 2) && !parse_on_off(argv[1], &monitor_on))
  {
    debug_uart_puts("usage: monitor [on|off]\r\n");
    return;
  }

  debug_uart_printf("monitor %s\r\n", monitor_on ? "on" : "off");
}

/* TEST ONLY - `canhold <ms>` makes the main loop skip the CAN RX ring drain
   until the deadline, so a bench run can fill the ring on purpose (task 6).
   The ISR keeps running and the rest of the loop stays live. Console commands
   run in the main loop, so plain statics are enough. */
static uint32_t can_hold_until;
static bool     can_hold_set;

static void cmd_canhold(int argc, char **argv)
{
  long ms = 0;

  if (argc < 2)
  {
    debug_uart_printf("canhold %s\r\n", console_can_hold_active() ? "active" : "idle");
    return;
  }

  ms = strtol(argv[1], NULL, 10);
  if ((ms < 0) || (ms > 5000))
  {
    debug_uart_puts("usage: canhold <0..5000 ms> - test only: pauses the RX ring drain\r\n");
    return;
  }

  can_hold_until = HAL_GetTick() + (uint32_t)ms;
  can_hold_set   = (ms > 0);
  debug_uart_printf("canhold %ld ms\r\n", ms);
}

static void cmd_loopback(int argc, char **argv)
{
  bool want = !can_bus_is_loopback();

  if ((argc >= 2) && !parse_on_off(argv[1], &want))
  {
    debug_uart_puts("usage: loopback [on|off]\r\n");
    return;
  }

  if (can_bus_set_loopback(want))
  {
    debug_uart_printf("CAN mode: %s\r\n", want ? "LOOPBACK (off the wire, self-ACK)"
                                               : "normal");
  }
  else
  {
    debug_uart_puts("failed to change CAN mode\r\n");
  }
}

/**
  * @brief  Bench interface to the SERVO42C. Every subcommand is asynchronous —
  *         it starts a transaction and returns; the outcome is printed later by
  *         console_report_mks().
  */
static void cmd_mks(int argc, char **argv)
{
  if (argc < 2)
  {
    debug_uart_puts(
      "usage: mks <sub>\r\n"
      "  encoder | pulses | angle | en | protect   read-only, safe in any mode\r\n"
      "  enable on|off                             F3\r\n"
      "  stop                                      F7\r\n"
      "  move <+/-pulses> [speed]                  FD, sign = direction\r\n"
      "  deg  <+/-degrees> [speed]                 FD at the gearbox output\r\n"
      "  raw  <hex...>                             body only; addr + checksum added\r\n"
      "  stats | clear | abort\r\n");
    return;
  }

  if (strcmp(argv[1], "stats") == 0)
  {
    const mks_stats_t *m = mks_stats();

    debug_uart_printf("mks req   : %lu, ok %lu, %lu echoes stripped (normal)\r\n",
                      m->requests, m->replies_ok, m->echoes);
    debug_uart_printf("mks fail  : %lu timeout, %lu move-timeout, %lu bad-cksum, "
                      "%lu bad-addr\r\n",
                      m->timeouts, m->motion_timeouts, m->bad_checksum,
                      m->bad_addr);
    debug_uart_printf("mks uart  : %lu error callbacks\r\n", m->uart_errors);

    if (m->uart_errors != 0u)
    {
      debug_uart_printf("            overrun %lu, frame %lu, noise %lu, "
                        "parity %lu, dma %lu\r\n",
                        m->err_overrun, m->err_frame, m->err_noise,
                        m->err_parity, m->err_dma);
      debug_uart_printf("            %lu re-arms (only when RX actually stopped)\r\n",
                        m->rx_restarts);
    }

    return;
  }

  if (strcmp(argv[1], "clear") == 0)
  {
    mks_clear_stats();
    debug_uart_puts("mks counters cleared\r\n");
    return;
  }

  if (strcmp(argv[1], "abort") == 0)
  {
    mks_abort();
    debug_uart_puts("mks transaction aborted (motor NOT stopped - use 'mks stop')\r\n");
    return;
  }

  if (mks_busy())
  {
    debug_uart_puts("mks busy - a transaction is outstanding ('mks abort' to drop it)\r\n");
    return;
  }

  bool started = false;

  if      (strcmp(argv[1], "encoder") == 0) { started = mks_read_encoder(); }
  else if (strcmp(argv[1], "pulses")  == 0) { started = mks_read_pulses(); }
  else if (strcmp(argv[1], "angle")   == 0) { started = mks_read_angle_error(); }
  else if (strcmp(argv[1], "en")      == 0) { started = mks_read_en_status(); }
  else if (strcmp(argv[1], "protect") == 0) { started = mks_read_protect_state(); }
  else if (strcmp(argv[1], "stop")    == 0) { started = mks_stop(); motion_release(); }
  else if (strcmp(argv[1], "enable")  == 0)
  {
    bool on = true;

    if ((argc < 3) || !parse_on_off(argv[2], &on))
    {
      debug_uart_puts("usage: mks enable on|off\r\n");
      return;
    }

    started = mks_enable(on);
  }
  else if ((strcmp(argv[1], "move") == 0) || (strcmp(argv[1], "deg") == 0))
  {
    if (!motion_gate())
    {
      return;
    }

    if (argc < 3)
    {
      debug_uart_printf("usage: mks %s <+/-value> [speed 1-127]\r\n", argv[1]);
      return;
    }

    long speed = 2;   /* low speed is what steering wants */

    if (argc >= 4)
    {
      speed = strtol(argv[3], NULL, 10);

      if ((speed < 1) || (speed > 127))
      {
        debug_uart_puts("speed must be 1..127 (use 1-4 for steering)\r\n");
        return;
      }
    }

    if (strcmp(argv[1], "move") == 0)
    {
      long pulses = strtol(argv[2], NULL, 10);
      bool ccw    = (pulses < 0);
      long mag    = ccw ? -pulses : pulses;

      if ((mag == 0) || (mag > 2147483647L))
      {
        debug_uart_puts("pulses must be non-zero\r\n");
        return;
      }

      started = mks_move_pulses(ccw, (uint8_t)speed, (uint32_t)mag);
      debug_uart_printf("moving %ld pulses %s at speed %ld...\r\n",
                        mag, ccw ? "CCW" : "CW", speed);
    }
    else
    {
      float degrees = strtof(argv[2], NULL);

      started = mks_move_degrees(degrees, (uint8_t)speed);

      if (started)
      {
        debug_uart_printf("moving %.4f deg at output (%lu pulses) at speed %ld...\r\n",
                          (double)degrees,
                          (unsigned long)((degrees < 0.0f ? -degrees : degrees) *
                                          (float)MKS_PULSES_PER_OUTPUT_REV / 360.0f + 0.5f),
                          speed);
      }
      else
      {
        debug_uart_puts("value too small - one pulse is 0.0118 deg at the output\r\n");
        return;
      }
    }
  }
  else if (strcmp(argv[1], "raw") == 0)
  {
    uint8_t body[MKS_MAX_BODY];
    uint8_t len = 0u;

    if ((argc < 3) || !parse_hex_bytes(argc - 2, &argv[2], body, &len, sizeof(body)) ||
        (len == 0u))
    {
      debug_uart_puts("usage: mks raw <hex...>   e.g. 'mks raw 30' sends E0 30 10\r\n");
      return;
    }

    started = mks_request(body, len, false);
  }
  else
  {
    debug_uart_printf("unknown subcommand '%s' - try 'mks'\r\n", argv[1]);
    return;
  }

  if (!started)
  {
    debug_uart_puts("could not start transaction\r\n");
  }
  else if ((strcmp(argv[1], "move") == 0) || (strcmp(argv[1], "deg") == 0))
  {
    motion_claim(MOTION_SRC_UART);
    steer_external();   /* CAN's tracked position no longer holds */
  }
  else if (strcmp(argv[1], "enable") == 0)
  {
    steer_external();
  }
}


/* -------------------------------------------------------------------------- */
/* Drive motor — encoder and H-bridge                                          */
/* -------------------------------------------------------------------------- */

/** @brief Print the accumulated position, derived revolutions and speed.
  *
  * Counts are shown as a 32-bit value even though the accumulator is 64-bit:
  * newlib-nano's printf omits long-long support, and 2^31 counts is about
  * 255 000 output revolutions, which no bench session will reach. The
  * accumulator itself is unaffected. */
static void print_encoder_line(void)
{
  int64_t pos = encoder_position();

  if (pos >  2147483647LL) { pos =  2147483647LL; }
  if (pos < -2147483648LL) { pos = -2147483648LL; }

  debug_uart_printf("count %ld  rev %.4f  rpm %.2f  raw %lu  d %ld  %s\r\n",
                    (long)pos,
                    (double)encoder_revolutions(),
                    (double)encoder_rpm(),
                    (unsigned long)encoder_raw_count(),
                    (long)encoder_last_delta(),
                    encoder_counting_up() ? "up" : "down");
}

static void cmd_enc(int argc, char **argv)
{
  if (argc < 2)
  {
    print_encoder_line();
    debug_uart_printf("  %.1f counts per output revolution, window %u ticks\r\n",
                      (double)ENCODER_COUNTS_PER_OUTPUT_REV,
                      (unsigned)encoder_velocity_window());
    debug_uart_printf("  ticks since zero: %lu%s\r\n",
                      (unsigned long)encoder_tick_count(),
                      (encoder_tick_count() == 0u) ? "  <-- TIM6 TICK NOT RUNNING" : "");
    debug_uart_puts("  sub: zero | watch on|off | window <ticks> | probe [ms]\r\n");
    return;
  }

  if (strcmp(argv[1], "zero") == 0)
  {
    encoder_zero();
    debug_uart_puts("encoder zeroed\r\n");
  }
  else if (strcmp(argv[1], "watch") == 0)
  {
    bool on;

    if ((argc < 3) || !parse_on_off(argv[2], &on))
    {
      debug_uart_printf("watch is %s\r\n", enc_watch_on ? "on" : "off");
      return;
    }

    enc_watch_on   = on;
    enc_watch_last = HAL_GetTick();
    debug_uart_printf("watch %s\r\n", on ? "on - turn the shaft" : "off");
  }
  else if (strcmp(argv[1], "window") == 0)
  {
    if (argc >= 3)
    {
      long ticks = strtol(argv[2], NULL, 10);
      encoder_set_velocity_window((uint16_t)((ticks < 1) ? 1 : ticks));
    }

    /* One count of difference over the window, expressed in rpm — the real
       resolution limit of the velocity reading. */
    double res = (1000.0 / (double)encoder_velocity_window())
                 * 60.0 / (double)ENCODER_COUNTS_PER_OUTPUT_REV;

    debug_uart_printf("window %u ticks - %.0f Hz update, %.2f rpm per count\r\n",
                      (unsigned)encoder_velocity_window(),
                      1000.0 / (double)encoder_velocity_window(),
                      res);
  }
  else if (strcmp(argv[1], "probe") == 0)
  {
    uint32_t ms = 2000u;

    if (argc >= 3)
    {
      long requested = strtol(argv[2], NULL, 10);
      if (requested > 0) { ms = (uint32_t)requested; }
      if (ms > 10000u)   { ms = 10000u; }
    }

    encoder_probe_t p;

    debug_uart_printf("watching A/B for %lu ms - TURN THE SHAFT NOW\r\n",
                      (unsigned long)ms);
    (void)debug_uart_flush(100u);   /* get the prompt out before we block */

    encoder_probe(ms, &p);

    debug_uart_printf("  A: %lu edges, now %s     B: %lu edges, now %s\r\n",
                      (unsigned long)p.edges_a, p.level_a ? "HIGH" : "low",
                      (unsigned long)p.edges_b, p.level_b ? "HIGH" : "low");
    debug_uart_printf("  TIM2 moved %ld counts over %lu samples\r\n",
                      (long)p.count_delta, (unsigned long)p.samples);

    if ((p.edges_a == 0u) && (p.edges_b == 0u))
    {
      debug_uart_puts(
        "  -> NOTHING reaches the MCU. Both lines idle "
        );
      debug_uart_printf("%s.\r\n", (p.level_a && p.level_b) ? "HIGH (pull-ups fine, encoder silent)"
                                                             : "LOW (suspect power, ground or a short)");
      debug_uart_puts(
        "     Check: 3.3 V on blue, gray tied to board GND, and that the\r\n"
        "     shaft is really turning. Not a timer problem.\r\n");
    }
    else if ((p.edges_a == 0u) || (p.edges_b == 0u))
    {
      debug_uart_printf("  -> Only channel %s is moving. Quadrature needs both;\r\n",
                        (p.edges_a != 0u) ? "A" : "B");
      debug_uart_puts("     TIM2 will count erratically or not at all.\r\n");
    }
    else if (p.count_delta == 0)
    {
      debug_uart_puts(
        "  -> Both channels are pulsing but TIM2 is NOT counting.\r\n"
        "     The signal is fine; the timer is not seeing it. Suspect the\r\n"
        "     AF setting on PA15/PB3, or a debugger holding the JTAG pins.\r\n");
    }
    else
    {
      debug_uart_puts("  -> Encoder and TIM2 are both working.\r\n");
    }
  }
  else
  {
    debug_uart_printf("unknown subcommand '%s' - try 'enc'\r\n", argv[1]);
  }
}

static void cmd_drv(int argc, char **argv)
{
  if (argc < 2)
  {
    debug_uart_printf("nSLEEP %s  duty %+d o/oo  decay %s  limit %u%%  nFAULT %s\r\n",
                      drive_is_enabled() ? "high (enabled)" : "low (disabled)",
                      drive_duty(),
                      (drive_decay() == DRIVE_DECAY_SLOW) ? "slow (drive-brake)"
                                                          : "fast (sign-magnitude)",
                      (unsigned)(drive_limit() / 10u),
                      drive_faulted() ? "ASSERTED" : "clear");
    debug_uart_printf("trip %lu mA (VREF %u mV, DAC %u, buffer %s)"
                      "  ADC ceiling %u mA\r\n",
                      (unsigned long)isense_trip_ma(),
                      (unsigned)isense_vref_mv(),
                      (unsigned)isense_vref_code(),
                      isense_vref_buffered() ? "on" : "off",
                      (unsigned)isense_full_scale_ma());

    /* The live pin above says what is true now; this says what has happened.
       A driver that tripped and auto-retried during a stall test reads clear
       on the pin and still owes you an explanation. */
    if (drive_fault_latched())
    {
      debug_uart_printf("  FAULT SEEN: %lu ms asserted, first at duty %+d%%"
                        " - 'drv clearfault' to reset\r\n",
                        (unsigned long)drive_fault_ticks(),
                        drive_fault_duty() / 10);
    }

    if (drive_timeout() != 0u)
    {
      /* The latch is sticky and a kick does NOT clear it, so "expired" alone
         is ambiguous: it is true both while the watchdog is down and long
         after traffic resumed. Printed in the present tense beside a live
         countdown it reads as a fault that is happening now. Split on the
         countdown, which is what actually says whether it is down. */
      debug_uart_printf("  watchdog %lu ms armed, %lu ms remaining%s\r\n",
                        (unsigned long)drive_timeout(),
                        (unsigned long)drive_timeout_remaining(),
                        (!drive_timeout_expired())        ? ""
                        : (drive_timeout_remaining() == 0u) ? "  - EXPIRED NOW"
                        : "  - timed out earlier (latched)");
    }
    else if (drive_timeout_expired())
    {
      debug_uart_puts("  WATCHDOG EXPIRED and is now disarmed - the bridge was"
                      " coasted by the watchdog\r\n");
    }

    if (drive_ramp() != 0u)
    {
      debug_uart_printf("  ramp %u o/oo/s (%u.%u%%/s), floor %u o/oo"
                        "  target %+d o/oo%s\r\n",
                        (unsigned)drive_ramp(),
                        (unsigned)(drive_ramp() / 10u),
                        (unsigned)(drive_ramp() % 10u),
                        (unsigned)drive_ramp_floor(),
                        drive_duty_target(),
                        drive_slewing() ? "  - SLEWING" : "");
    }
    else
    {
      debug_uart_puts("  ramp off - duty steps instantly\r\n");
    }

    debug_uart_puts(
      "  sub: enable | disable | duty <+/-pct[p]> | brake | coast\r\n"
      "       decay slow|fast | limit <pct> | current [n] | zero\r\n"
      "       iscan [n] [from] [to] [step]"
      "  (diagnostic: IPROPI vs PWM phase)\r\n"
      "       trip [<mA> | buf on|off] | clearfault | timeout [ms]\r\n"
      "       ramp [<o/oo per s> | floor <o/oo>]\r\n");
    return;
  }

  if (strcmp(argv[1], "clearfault") == 0)
  {
    drive_clear_fault();
    debug_uart_printf("fault latch cleared - nFAULT reads %s right now\r\n",
                      drive_faulted() ? "ASSERTED (still faulting)" : "clear");
    return;
  }

  if (strcmp(argv[1], "enable") == 0)
  {
    if (!motion_gate())
    {
      return;
    }

    drive_enable();
    motion_claim(MOTION_SRC_UART);
    debug_uart_puts("nSLEEP high - driver awake (waited 2 ms)\r\n");
  }
  else if (strcmp(argv[1], "disable") == 0)
  {
    drive_disable();
    motion_release();
    debug_uart_puts("duty 0, nSLEEP low - driver disabled\r\n");
  }
  else if (strcmp(argv[1], "duty") == 0)
  {
    if (argc < 3)
    {
      debug_uart_puts("usage: drv duty <+/-pct>   or  <+/-permille>p\r\n");
      return;
    }

    /* Percent by default, because every procedure and every bench script says
       percent. A trailing 'p' means the argument is already per-mille, which is
       the resolution drive.c actually works in - and the resolution the 1%-step
       stiction bracket needs, since `drv duty 10` and `drv duty 11` are the only
       two commands the percent form can put either side of breakaway. */
    char *end  = NULL;
    long  n    = strtol(argv[2], &end, 10);
    bool  raw  = (end != NULL) && ((*end == 'p') || (*end == 'P'));
    long  pm   = raw ? n : (n * 10);

    if (pm >  DRIVE_DUTY_MAX) { pm =  DRIVE_DUTY_MAX; }
    if (pm < -DRIVE_DUTY_MAX) { pm = -DRIVE_DUTY_MAX; }

    /* Duty 0 is a stop, and a stop always works. */
    if ((pm != 0) && !motion_gate())
    {
      return;
    }

    drive_set_duty((int16_t)pm);

    if (pm != 0)
    {
      motion_claim(MOTION_SRC_UART);
    }

    if (drive_slewing())
    {
      debug_uart_printf("target %+d o/oo, ramping from %+d at %u o/oo/s%s\r\n",
                        drive_duty_target(),
                        drive_duty(),
                        (unsigned)drive_ramp(),
                        drive_is_enabled() ? ""
                                           : "  (driver still disabled)");

      if (drive_duty_target() == 0)
      {
        /* Worth saying every time. An operator who wants the wheel to stop and
           watches it keep turning for several seconds will reach for something
           more drastic than the thing that was already going to work. */
        debug_uart_puts("  ramping down - 'drv coast' stops it immediately"
                        " if you need it now\r\n");
      }
    }
    else
    {
      debug_uart_printf("duty %+d o/oo%s\r\n",
                        drive_duty(),
                        drive_is_enabled() ? ""
                                           : "  (driver still disabled - pins only,"
                                             " which is what you want for a scope check)");
    }
  }
  else if (strcmp(argv[1], "brake") == 0)
  {
    drive_brake();
    motion_release();
    debug_uart_puts("both inputs high - brake\r\n");
  }
  else if (strcmp(argv[1], "coast") == 0)
  {
    drive_coast();
    motion_release();
    debug_uart_puts("both inputs low - coast\r\n");
  }
  else if (strcmp(argv[1], "decay") == 0)
  {
    if (argc >= 3)
    {
      if      (strcmp(argv[2], "slow") == 0) { drive_set_decay(DRIVE_DECAY_SLOW); }
      else if (strcmp(argv[2], "fast") == 0) { drive_set_decay(DRIVE_DECAY_FAST); }
      else { debug_uart_puts("usage: drv decay slow|fast\r\n"); return; }
    }

    debug_uart_printf("decay %s\r\n",
                      (drive_decay() == DRIVE_DECAY_SLOW)
                        ? "slow - IN1 high, IN2 PWM inverted (drive/brake)"
                        : "fast - IN1 PWM, IN2 low (drive/coast)");
  }
  else if (strcmp(argv[1], "current") == 0)
  {
    uint16_t n     = (argc >= 3) ? (uint16_t)strtoul(argv[2], NULL, 10) : 0u;
    int16_t  dperm = drive_duty();
    uint16_t dmag  = (uint16_t)((dperm < 0) ? -dperm : dperm);

    /* Asked before reading, not inferred from the result: isense_read_sync_avg
       returns 0 both for "no current" and for "could not measure", and those
       two must not print the same line. */
    bool     sync  = isense_sync_ready();
    uint16_t raw   = sync ? isense_read_sync_avg(n) : isense_read_avg(n);
    uint32_t ma    = isense_raw_to_ma(raw);

    if (sync)
    {
      /* Sampled at a known point inside the drive phase, where IPROPI is live -
         or, below 14.5% in slow decay, inside the brake phase, scaled by
         1000 / isense_dk. Either way MOTOR current, measured - no divide by D,
         and the same units the DRV8874's trip regulates in. The aliasing that
         made the free-running average unusable at low duty is in isense.h. */
      debug_uart_printf("Imotor %lu mA  (raw %u, offset %u)  at duty %+d%%"
                        "  decay %s\r\n",
                        (unsigned long)ma,
                        (unsigned)raw,
                        (unsigned)isense_offset(),
                        dperm / 10,
                        (drive_decay() == DRIVE_DECAY_SLOW) ? "slow" : "fast");

      /* The phase geometry is printed rather than trusted. A sample taken at
         the wrong tick is the one failure this path can still have, and it is
         invisible in the current figure alone. */
      {
        uint16_t ticks = drive_phase_ticks();
        bool     dec   = isense_sync_is_decay();

        uint16_t depth = (uint16_t)((n == 0u) ? ISENSE_SYNC_AVG_DEFAULT
                                              : ((n > 1024u) ? 1024u : n));
        uint16_t first  = drive_sense_first();
        uint16_t last   = drive_sense_last();
        uint16_t points = (uint16_t)((last > first) ? ISENSE_SYNC_POINTS : 1u);

        if (points > depth) { points = depth; }

        debug_uart_printf("  sync %s: %u samples over %u tick%s in %u..%u, drive"
                          " phase %u ticks = %u.%01u us of 50.0\r\n",
                          dec ? "BRAKE phase" : "drive phase",
                          (unsigned)depth,
                          (unsigned)points,
                          (points == 1u) ? "" : "s",
                          (unsigned)((points == 1u) ? last : first),
                          (unsigned)last,
                          (unsigned)ticks,
                          (unsigned)(ticks / 90u),
                          (unsigned)(((ticks % 90u) * 10u) / 90u));

        if (dec)
        {
          debug_uart_printf("  brake-phase reading scaled x1000/%ld"
                            " (cfg isense_dk); +/-4%% from 6%% duty\r\n",
                            (long)config_get(CFG_ISENSE_DECAY_K));
        }
      }

      debug_uart_printf("  implies Isup %lu mA  (Imotor x D)\r\n",
                        (unsigned long)((ma * dmag) / (uint32_t)DRIVE_DUTY_MAX));
    }
    else
    {
      /* Fallback: a free-running average over the whole period. It is not
         motor current - and since the PMODE strap not clean supply current
         either, because slow decay's brake phase reads 0.690 x I_motor (see
         isense.h). Named Isup because the trip regulates MOTOR current and a
         line that just said "I" invited reading the two as one. */
      debug_uart_printf("Isup %lu mA  (raw %u, offset %u)  at duty %+d%%"
                        "  decay %s\r\n",
                        (unsigned long)ma,
                        (unsigned)raw,
                        (unsigned)isense_offset(),
                        dperm / 10,
                        (drive_decay() == DRIVE_DECAY_SLOW) ? "slow" : "fast");

      debug_uart_printf("  NOT SYNCHRONISED - drive phase is %u ticks, under the"
                        " %u a\r\n"
                        "  drive-phase sample needs, and the brake phase is not"
                        " usable here:\r\n"
                        "  %s\r\n"
                        "  This average runs free across the PWM period and can"
                        " alias against it;\r\n"
                        "  treat it as an order of magnitude, not a"
                        " measurement.\r\n",
                        (unsigned)drive_phase_ticks(),
                        (unsigned)DRIVE_PHASE_MIN_TICKS,
                        (dmag == 0u)
                          ? "duty is 0 (coast) or the bridge is braking."
                          : (drive_decay() == DRIVE_DECAY_FAST)
                            ? "fast decay coasts in the off phase - uncalibrated."
                              " Use 'drv decay slow'."
                            : "duty is under cfg isense_dmin, where the brake-"
                              "phase reading is not trusted.");

      /* Below ~1% the division blows the estimate up into nonsense, so it is
         simply not offered rather than printed with a caveat nobody will read. */
      if (dmag >= 10u)
      {
        debug_uart_printf("  implies Imotor %lu mA  (Isup / D - and 1/D"
                          " multiplies the error too)\r\n",
                          (unsigned long)((ma * (uint32_t)DRIVE_DUTY_MAX) / dmag));
      }
    }

    if (isense_saturated())
    {
      debug_uart_printf(
        "  CLIPPED - the reading hit the ADC ceiling (%u mA). Since the\r\n"
        "  carrier was modified this is NOT the same event as the bridge\r\n"
        "  regulating: regulation shows up as a plateau at the trip"
        " (%lu mA).\r\n"
        "  Both at once just means the trip is sitting at the ceiling.\r\n",
        (unsigned)isense_full_scale_ma(),
        (unsigned long)isense_trip_ma());
    }
    else if (sync && (raw > 0u))
    {
      /* Only offered on the synchronised path, where it is a fair comparison:
         both sides are motor current. Against a free-running supply reading it
         would be the units mismatch isense.h warns about, and the plateau would
         land at trip^2 x R / Vm rather than at the trip. */
      if ((ma * 20u) >= (isense_trip_ma() * 19u))
      {
        debug_uart_printf(
          "  at the TRIP (%lu mA) - if this number stops rising while duty\r\n"
          "  climbs, the bridge is regulating. Where it plateaus against the\r\n"
          "  commanded trip is the VREF divider test in isense.h - and it is\r\n"
          "  now a direct comparison, with no quadratic correction needed.\r\n",
          (unsigned long)isense_trip_ma());
      }
    }

    if (!drive_is_enabled())
    {
      debug_uart_puts("  (driver disabled - no bridge current, so this is an"
                      " offset reading, not a current)\r\n");
    }
  }
  else if (strcmp(argv[1], "iscan") == 0)
  {
    /* Map IPROPI across the whole PWM period instead of arguing about it. A
       mis-placed trigger, a mirror too slow to settle in the drive window and a
       signal that was never there all look identical from one sample; they look
       nothing alike across 36 of them. */
    uint16_t n     = (argc >= 3) ? (uint16_t)strtoul(argv[2], NULL, 10) : 64u;
    uint16_t ticks = drive_phase_ticks();
    uint16_t trig  = drive_phase_trigger();
    uint16_t start = drive_phase_start();
    int16_t  dperm = drive_duty();
    uint16_t peak  = 0u;
    uint16_t peak_t = 0u;
    uint16_t t;

    /* Optional window, so the drive phase can be examined at a resolution the
       full-period sweep cannot reach: at 13% duty it is 585 ticks wide, which a
       125-tick step samples exactly four times. */
    uint16_t from  = (argc >= 4) ? (uint16_t)strtoul(argv[3], NULL, 10) : 0u;
    uint16_t to    = (argc >= 5) ? (uint16_t)strtoul(argv[4], NULL, 10) : 4500u;
    uint16_t step  = (argc >= 6) ? (uint16_t)strtoul(argv[5], NULL, 10) : 125u;

    if (n == 0u)    { n = 64u; }
    if (step == 0u) { step = 1u; }
    if (to > 4500u) { to = 4500u; }
    if (from >= to) { from = 0u; to = 4500u; }

    debug_uart_printf("iscan: duty %+d%%  decay %s  %u samples/point\r\n",
                      (int)(dperm / 10), (drive_decay() == DRIVE_DECAY_SLOW) ? "slow" : "fast",
                      (unsigned)n);

    if (ticks == 0u)
    {
      debug_uart_puts("  no drive phase at this duty - scanning anyway, every"
                      " point should read the same\r\n");
    }
    else
    {
      debug_uart_printf("  drive phase = ticks %u..%u (marked *)\r\n",
                        (unsigned)start, (unsigned)(start + ticks));
    }

    if (!drive_is_enabled())
    {
      debug_uart_puts("  WARNING: driver disabled - this scans the offset,"
                      " not a current\r\n");
    }

    for (t = from; t < to; t = (uint16_t)(t + step))
    {
      bool     in  = (ticks != 0u) && (t >= start) && (t < (uint16_t)(start + ticks));
      uint16_t raw;

      /* Tick 0 is not a measurement and never was. CCR4 = 0 leaves TIM4_CH4
         permanently high, so no compare edge is generated, adc_wait_eoc() times
         out and sync_burst() returns 0 for "took nothing" - which then printed
         as `raw 0` beside 35 real numbers and read as a current. */
      if (t == 0u)
      {
        debug_uart_printf("  t %4u  %2u.%01u us  raw   --  %s\r\n",
                          (unsigned)t,
                          (unsigned)(t / 90u), (unsigned)((t / 9u) % 10u),
                          in ? "*" : "");
        continue;
      }

      raw = isense_read_sync_at(t, n);

      if (raw > peak) { peak = raw; peak_t = t; }

      debug_uart_printf("  t %4u  %2u.%01u us  raw %4u  %s\r\n",
                        (unsigned)t,
                        (unsigned)(t / 90u), (unsigned)((t / 9u) % 10u),
                        (unsigned)raw,
                        in ? "*" : "");
    }

    debug_uart_printf("  peak raw %u at tick %u; trigger currently sits at %u\r\n",
                      (unsigned)peak, (unsigned)peak_t, (unsigned)trig);

    if (from == 0u)
    {
      debug_uart_puts("  (tick 0 reads `--`: CCR4 = 0 raises no compare event,"
                      " so nothing is sampled there)\r\n");
    }
  }
  else if (strcmp(argv[1], "zero") == 0)
  {
    if (drive_is_enabled())
    {
      debug_uart_puts("refusing - 'drv disable' first. Zeroing while the bridge"
                      " is live folds real current into the offset\r\n");
      return;
    }

    debug_uart_printf("offset %u counts (%lu mA equivalent), 256 samples\r\n",
                      (unsigned)isense_zero(),
                      (unsigned long)isense_raw_to_ma(isense_offset()));
  }
  else if (strcmp(argv[1], "trip") == 0)
  {
    if ((argc >= 3) && !owner_gate())
    {
      return;
    }

    if ((argc >= 4) && (strcmp(argv[2], "buf") == 0))
    {
      bool on;
      if      (strcmp(argv[3], "on")  == 0) { on = true;  }
      else if (strcmp(argv[3], "off") == 0) { on = false; }
      else { debug_uart_puts("usage: drv trip buf on|off\r\n"); return; }

      isense_set_vref_buffered(on);

      if (!on)
      {
        debug_uart_puts("buffer OFF - ceiling rises to the full scale, but the"
                        " DAC is now high-impedance.\r\n"
                        "  Put a meter on PA4 and confirm VREF actually reads"
                        " what is commanded below.\r\n"
                        "  If it droops, the VREF pin loads it and the buffer"
                        " belongs back on.\r\n");
      }
    }
    else if (argc >= 3)
    {
      uint32_t want = strtoul(argv[2], NULL, 10);

      if (!isense_set_trip_ma(want))
      {
        debug_uart_printf("clamped - %lu mA is outside the %lu..%lu mA the DAC"
                          " can reach with the buffer %s\r\n",
                          (unsigned long)want,
                          (unsigned long)isense_trip_min_ma(),
                          (unsigned long)isense_trip_max_ma(),
                          isense_vref_buffered() ? "on" : "off");
      }
    }

    debug_uart_printf("trip %lu mA  (VREF %u mV, DAC code %u, buffer %s)\r\n",
                      (unsigned long)isense_trip_ma(),
                      (unsigned)isense_vref_mv(),
                      (unsigned)isense_vref_code(),
                      isense_vref_buffered() ? "on" : "off");
    /* One DAC code is a THIRD of an ADC LSB, not one. Both sides read the same
       R_IPROPI, but the comparator is fed VREF/3, so a code that moves VREF by
       one DAC step moves the trip by one third of a current step. The old text
       here said "1 code = 1 ADC LSB" - true only under k = 1, which the Sep 20
       plateau sweep killed. Held until the re-take was done, then fixed. */
    debug_uart_printf("  range %lu..%lu mA, ADC ceiling %u mA, 1 DAC code ="
                      " 1/3 ADC LSB (VREF/3)\r\n",
                      (unsigned long)isense_trip_min_ma(),
                      (unsigned long)isense_trip_max_ma(),
                      (unsigned)isense_full_scale_ma());

    if (drive_is_enabled())
    {
      debug_uart_puts("  (applied live - the driver follows VREF immediately)\r\n");
    }

    /* This command is the volatile one. Said only when the two have actually
       diverged, so it stays a useful signal instead of noise on every call.
       Compared as DAC CODES, not milliamps: isense_trip_ma() has been through
       quantisation and the stored value has not, so an mA comparison differs
       by a count or two even when the two mean the same thing - which would
       make this fire on every call and render the gate useless. */
    if ((argc >= 3) &&
        (isense_vref_code() !=
         isense_code_for_trip_ma((uint32_t)config_get(CFG_TRIP_BOOT_MA))))
    {
      debug_uart_puts("  (not persistent - 'cfg trip_ma <mA>' then 'cfg save'"
                      " to survive a reset)\r\n");
    }
  }
  else if (strcmp(argv[1], "limit") == 0)
  {
    if ((argc >= 3) && !owner_gate())
    {
      return;
    }

    if (argc >= 3)
    {
      long pct = strtol(argv[2], NULL, 10);
      if (pct < 0) { pct = 0; }
      drive_set_limit((uint16_t)(pct * 10));
    }

    debug_uart_printf("limit %u%%\r\n", (unsigned)(drive_limit() / 10u));

    if ((argc >= 3) && (drive_limit() != (uint16_t)config_get(CFG_DUTY_LIMIT)))
    {
      debug_uart_puts("  (not persistent - 'cfg duty_limit <permille>' then"
                      " 'cfg save' to survive a reset)\r\n");
    }
  }
  else if (strcmp(argv[1], "ramp") == 0)
  {
    if ((argc >= 3) && !owner_gate())
    {
      return;
    }

    if ((argc >= 4) && (strcmp(argv[2], "floor") == 0))
    {
      long pm = strtol(argv[3], NULL, 10);

      if ((pm < 0) || (pm > (long)config_max(CFG_RAMP_FLOOR)))
      {
        debug_uart_printf("floor must be 0..%ld o/oo\r\n",
                          (long)config_max(CFG_RAMP_FLOOR));
        return;
      }

      drive_set_ramp_floor((uint16_t)pm);
    }
    else if (argc >= 3)
    {
      long pmps = strtol(argv[2], NULL, 10);

      if ((pmps < 0) || (pmps > (long)config_max(CFG_RAMP_PMPS)))
      {
        debug_uart_printf("rate must be 0..%ld o/oo per second\r\n",
                          (long)config_max(CFG_RAMP_PMPS));
        return;
      }

      drive_set_ramp((uint16_t)pmps);
    }

    if (drive_ramp() == 0u)
    {
      debug_uart_puts("ramp off - duty steps instantly\r\n");
      debug_uart_puts("  an un-ramped step from rest draws stall current:"
                      " back-EMF is zero at t=0,\r\n"
                      "  so the winding sees the whole terminal voltage."
                      " On the loaded rig that\r\n"
                      "  tripped 1580 mA; the same move at 50 o/oo/s peaked"
                      " at 572 mA\r\n");
    }
    else
    {
      debug_uart_printf("ramp %u o/oo/s (%u.%u%% per second), floor %u o/oo\r\n",
                        (unsigned)drive_ramp(),
                        (unsigned)(drive_ramp() / 10u),
                        (unsigned)(drive_ramp() % 10u),
                        (unsigned)drive_ramp_floor());
      debug_uart_printf("  target %+d o/oo, bridge %+d o/oo%s\r\n",
                        drive_duty_target(),
                        drive_duty(),
                        drive_slewing() ? "  - SLEWING" : "");

      if (drive_ramp_floor() == 0u)
      {
        debug_uart_puts("  no floor: a ramp from rest crawls through the"
                        " sub-breakaway band stalled,\r\n"
                        "  with no back-EMF. 'drv ramp floor <o/oo>' above"
                        " breakaway - ~120 loaded,\r\n"
                        "  ~60 free wheel\r\n");
      }

      debug_uart_puts("  coast and brake are NOT ramped - both are immediate,"
                      " by design\r\n");
    }

    if ((drive_ramp()       != (uint16_t)config_get(CFG_RAMP_PMPS)) ||
        (drive_ramp_floor() != (uint16_t)config_get(CFG_RAMP_FLOOR)))
    {
      debug_uart_puts("  (not persistent - 'cfg ramp_pmps <n>' /"
                      " 'cfg ramp_floor <n>' then 'cfg save')\r\n");
    }
  }
  else if (strcmp(argv[1], "timeout") == 0)
  {
    if (argc >= 3)
    {
      drive_set_timeout(strtoul(argv[2], NULL, 10));
    }

    uint32_t period = drive_timeout();

    if (period == 0u)
    {
      debug_uart_puts("timeout off - the bridge holds its last duty"
                      " indefinitely\r\n");
      debug_uart_puts("  'drv timeout <ms>' before any unattended run:"
                      " a host that dies to SIGKILL\r\n"
                      "  or a cable that falls out runs no cleanup code"
                      " at all\r\n");
    }
    else
    {
      debug_uart_printf("timeout %lu ms, %lu ms remaining\r\n",
                        (unsigned long)period,
                        (unsigned long)drive_timeout_remaining());
      debug_uart_puts("  any 'drv duty' kicks it; re-issuing this command"
                      " kicks it too\r\n");
    }

    if (drive_timeout_expired())
    {
      debug_uart_puts("  EXPIRED SINCE ARMING - the bridge was coasted by the"
                      " watchdog, not by a\r\n"
                      "  command. Whatever was driving it stopped talking."
                      " 'drv timeout <ms>' to re-arm\r\n");
    }
  }
  else
  {
    debug_uart_printf("unknown subcommand '%s' - try 'drv'\r\n", argv[1]);
  }
}

/** @brief Render a milli-rpm integer without dragging the caller through the
  *        sign handling. -1234 must print "-1.234", not "-1.-234". */
static void print_mrpm(const char *label, int32_t mrpm, const char *tail)
{
  int32_t whole = mrpm / 1000;
  int32_t frac  = mrpm % 1000;

  if (frac < 0) { frac = -frac; }

  debug_uart_printf("%s%s%ld.%03ld%s", label,
                    ((mrpm < 0) && (whole == 0)) ? "-" : "",
                    (long)whole, (long)frac, tail);
}

static const char *vel_state_name(velocity_state_t s)
{
  switch (s)
  {
    case VELOCITY_OFF:     return "off";
    case VELOCITY_RUNNING: return "running";
    case VELOCITY_HOLDING: return "holding (coasting)";
    case VELOCITY_TIMEOUT: return "TIMEOUT - coasted";
    default:               return "?";
  }
}

static void cmd_vel(int argc, char **argv)
{
  if (argc < 2)
  {
    int16_t ff, p, i, d;

    velocity_terms(&ff, &p, &i, &d);

    debug_uart_printf("velocity loop %s, state %s\r\n",
                      velocity_enabled() ? "ARMED" : "off",
                      vel_state_name(velocity_state()));

    print_mrpm("  setpoint ", velocity_setpoint(), " rpm");
    print_mrpm(" -> ramped ", velocity_ramped_setpoint(), " rpm\r\n");
    print_mrpm("  measured ", velocity_measured(), " rpm");
    print_mrpm(", error ",    velocity_error(),    " rpm\r\n");

    debug_uart_printf("  output %d o/oo%s   ff %d  p %d  i %d  d %d\r\n",
                      (int)velocity_output(),
                      velocity_saturated() ? " (SATURATED)" : "",
                      (int)ff, (int)p, (int)i, (int)d);

    debug_uart_printf("  step %lu us (encoder window), bridge %s%s\r\n",
                      (unsigned long)velocity_period_us(),
                      drive_is_enabled() ? "enabled" : "DISABLED",
                      drive_fault_latched() ? ", FAULT LATCHED" : "");

    /* The one thing a reader must not have to infer. Arming this loop means
       drive_set_duty() is called at the measurement rate forever, so drv's own
       watchdog stops being able to fire; this one is what is left. */
    if (velocity_timeout() == 0u)
    {
      debug_uart_puts("  setpoint watchdog DISARMED - nothing will stop this"
                      " loop if the host dies\r\n");
    }
    else
    {
      /* Same split as drv's above, and for the same reason: velocity.c's latch
         is cleared only by velocity_enable(), so a loop that timed out once and
         then recovered would otherwise report "HAS EXPIRED" beside a healthy
         countdown for the rest of the arming. Seen on the bench. */
      debug_uart_printf("  setpoint watchdog %lu ms, %lu remaining%s\r\n",
                        (unsigned long)velocity_timeout(),
                        (unsigned long)velocity_timeout_remaining(),
                        (!velocity_timeout_expired())        ? ""
                        : (velocity_timeout_remaining() == 0u) ? "  <-- EXPIRED NOW"
                        : "  <-- timed out earlier (latched)");
    }

    debug_uart_puts("  sub: on | off | target <rpm|Nm> | stop | gains |"
                    " timeout <ms> | reset\r\n");
    return;
  }

  if (strcmp(argv[1], "on") == 0)
  {
    if (!motion_gate())
    {
      return;
    }

    if (!drive_is_enabled())
    {
      debug_uart_puts("bridge is disabled - 'drv enable' first, or the loop"
                      " will wind up\r\n");
      return;
    }

    velocity_enable();
    motion_claim(MOTION_SRC_UART);
    debug_uart_printf("velocity loop ARMED at setpoint 0, watchdog %lu ms\r\n",
                      (unsigned long)velocity_timeout());
    debug_uart_puts("  'drv duty' now fights the loop - use 'vel target'."
                    "  'vel off' hands the bridge back\r\n");
  }
  else if (strcmp(argv[1], "off") == 0)
  {
    velocity_disable();
    motion_release();
    debug_uart_puts("velocity loop off, bridge coasted - 'drv duty' is yours"
                    " again\r\n");
  }
  else if (strcmp(argv[1], "target") == 0)
  {
    char *end = NULL;
    long  v;

    if (argc < 3)
    {
      print_mrpm("target ", velocity_setpoint(), " rpm\r\n");
      return;
    }

    if (!motion_gate())
    {
      return;
    }

    if (!velocity_enabled())
    {
      debug_uart_puts("loop is off - 'vel on' first (the setpoint would be"
                      " ignored)\r\n");
      return;
    }

    v = strtol(argv[2], &end, 10);

    /* Bare number is rpm; an 'm' suffix is milli-rpm, which is the resolution
       the loop actually works in and the only way to ask for a fraction from a
       console that has no float parser. `vel target 1500m` is 1.5 rpm. */
    if ((end != NULL) && ((*end == 'm') || (*end == 'M')))
    {
      velocity_set_setpoint((int32_t)v);
    }
    else
    {
      velocity_set_setpoint((int32_t)v * 1000);
    }

    motion_claim(MOTION_SRC_UART);
    print_mrpm("target ", velocity_setpoint(), " rpm");
    debug_uart_printf(", ramping at %ld m rpm/s\r\n", (long)velocity_slew());
  }
  else if (strcmp(argv[1], "stop") == 0)
  {
    /* Walks the ramp down and then coasts. This is NOT an emergency stop:
       'drv coast' is, and it stays immediate. */
    velocity_set_setpoint(0);
    motion_release();
    debug_uart_puts("setpoint 0 - ramping down, then coast."
                    "  'drv coast' if you want it now\r\n");
  }
  else if (strcmp(argv[1], "reset") == 0)
  {
    velocity_reset();
    debug_uart_puts("integrator and derivative cleared\r\n");
  }
  else if (strcmp(argv[1], "timeout") == 0)
  {
    if (argc >= 3)
    {
      velocity_set_timeout((uint32_t)strtoul(argv[2], NULL, 10));
    }

    if (velocity_timeout() == 0u)
    {
      debug_uart_puts("setpoint watchdog DISARMED - the loop will hold its"
                      " last setpoint forever\r\n");
    }
    else
    {
      debug_uart_printf("setpoint watchdog %lu ms, %lu remaining\r\n",
                        (unsigned long)velocity_timeout(),
                        (unsigned long)velocity_timeout_remaining());
      debug_uart_puts("  every 'vel target' kicks it; expiry forces setpoint 0"
                      " and coasts\r\n");
    }
  }
  else if (strcmp(argv[1], "gains") == 0)
  {
    debug_uart_printf("kp %ld  ki %ld  kd %ld   (all x1000)\r\n",
                      (long)velocity_kp(), (long)velocity_ki(),
                      (long)velocity_kd());
    debug_uart_printf("ff  %ld x1000 o/oo per rpm + %ld o/oo\r\n",
                      (long)velocity_ff_slope(), (long)velocity_ff_offset());
    debug_uart_printf("ilim %u o/oo   max %u o/oo (drv limit %u)   slew %ld m rpm/s\r\n",
                      (unsigned)velocity_i_limit(), (unsigned)velocity_max(),
                      (unsigned)drive_limit(), (long)velocity_slew());
    debug_uart_puts("  change them with 'cfg vel_kp <n>' etc - applied live,"
                    " 'cfg save' to keep\r\n");
  }
  else
  {
    debug_uart_printf("unknown subcommand '%s' - try 'vel'\r\n", argv[1]);
  }
}

static void cmd_reset(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  debug_uart_puts("resetting...\r\n");
  (void)debug_uart_flush(100u);   /* let the message reach the wire first */
  NVIC_SystemReset();
}

/* --- cfg ------------------------------------------------------------------ *
 * The console half of config.c. Everything here is deliberately explicit: a
 * value typed here can end up setting a current limit, so nothing is guessed,
 * nothing is silently clamped, and the difference between "changed" and
 * "changed AND saved" is printed every time rather than assumed.
 * -------------------------------------------------------------------------- */

/** @brief One table row. * marks unsaved, so a forgotten 'cfg save' is visible
  *        without having to remember what was typed. */
/**
  * @brief Let the wire catch up before queueing another line of a listing.
  *
  * The TX ring is 1024 bytes and `debug_uart_write()` DROPS the overflow rather
  * than blocking — the right choice for a control loop, which must never be
  * stalled by a debug facility, and the wrong one for a console listing, which
  * is read by a person or parsed by a tool and is useless with a hole in it.
  *
  * A listing is emitted in a tight loop that fills the ring in microseconds,
  * while the DMA drains it at 11.5 kB/s. Anything past ~1 kB in one command is
  * therefore lost silently. `cfg` crossed that line the day the nine velocity
  * keys were added: 20 keys is ~1.3 kB, and 538 bytes went in the bin — taking
  * the last three keys, the legend, the sub-command line and the prompt with
  * them. Nothing reported it but `stats`.
  *
  * So: before each line, wait until the ring has room for one. Bounded, because
  * a console that can hang is worse than one that truncates, and DMA-driven, so
  * it drains without the main loop. Safe here and only here — this is called
  * from command handlers, which already run at the console's leisure.
  *
  * Call it from any loop that prints one line per item. The per-item printers
  * below do it themselves, so adding a key or a command cannot re-open this.
  */
#define CONSOLE_PACE_TIMEOUT_MS  200u

static void console_pace(size_t headroom)
{
  uint32_t start = HAL_GetTick();

  while ((DEBUG_UART_TX_BUF_SIZE - debug_uart_tx_pending()) < headroom)
  {
    if ((HAL_GetTick() - start) >= CONSOLE_PACE_TIMEOUT_MS)
    {
      break;      /* give up and let it drop rather than hang the console */
    }
  }
}

static void cfg_print_key(config_key_t k)
{
  int32_t now = config_get(k);

  console_pace(96u);            /* one key line is ~60 bytes */

  debug_uart_printf("  %c %-11s %8ld %-5s (default %ld, %ld..%ld)\r\n",
                    (now == config_default(k)) ? ' ' : '*',
                    config_name(k),
                    (long)now,
                    config_units(k),
                    (long)config_default(k),
                    (long)config_min(k),
                    (long)config_max(k));
}

static void cfg_report_save(config_save_t r)
{
  switch (r)
  {
    case CONFIG_SAVE_OK:
    {
      uint16_t used, total;
      config_usage(&used, &total);
      debug_uart_printf("saved - slot %u of %u used\r\n",
                        (unsigned)used, (unsigned)total);
      break;
    }

    case CONFIG_SAVE_BUSY:
      /* Refusing, not queueing. Programming flash stalls the core for up to
         3 s on this part: the control loop stops and a turning motor keeps
         turning through all of it, unsupervised. */
      debug_uart_puts("refusing - 'drv disable' first. Writing flash halts the"
                      " core for up to 3 s,\r\n"
                      "  and a motor already turning keeps turning open-loop"
                      " for the whole stall\r\n");
      break;

    case CONFIG_SAVE_UNCHANGED:
      debug_uart_puts("nothing to save - flash already matches\r\n");
      break;

    case CONFIG_SAVE_FLASH_ERROR:
    default:
      debug_uart_puts("FLASH ERROR - nothing was written. The stored"
                      " configuration is unchanged\r\n");
      break;
  }
}

static void cmd_cfg(int argc, char **argv)
{
  if (argc < 2)
  {
    uint16_t used, total;
    config_usage(&used, &total);

    debug_uart_printf("config v%u, %u keys, slot %u/%u used%s\r\n",
                      (unsigned)CONFIG_VERSION,
                      (unsigned)CFG_KEY_COUNT,
                      (unsigned)used, (unsigned)total,
                      config_dirty() ? "  -- UNSAVED CHANGES" : "");

    for (uint16_t i = 0u; i < (uint16_t)CFG_KEY_COUNT; i++)
    {
      cfg_print_key((config_key_t)i);
    }

    /* The per-key pace above leaves only its own headroom, and this tail is
       ~120 bytes. Pacing the loop but not what follows it is why the first
       version of this fix held for one run and truncated on the next. */
    console_pace(256u);
    debug_uart_puts(
      "  (* = differs from the compiled default)\r\n"
      "  sub: <key> [value] | save | revert | default [<key>] | help\r\n");
    return;
  }

  if (strcmp(argv[1], "save") == 0)
  {
    cfg_report_save(config_save());
    return;
  }

  if (strcmp(argv[1], "revert") == 0)
  {
    config_load_t r = config_revert();
    config_apply_all();

    debug_uart_printf("reverted to stored values - %s\r\n", config_load_str(r));
    debug_uart_printf("  trip %lu mA, limit %u%% - all keys re-applied\r\n",
                      (unsigned long)isense_trip_ma(),
                      (unsigned)(drive_limit() / 10u));
    return;
  }

  if (strcmp(argv[1], "default") == 0)
  {
    if (argc >= 3)
    {
      config_key_t k = config_find(argv[2]);
      if (k == CFG_KEY_COUNT)
      {
        debug_uart_printf("unknown key '%s' - try 'cfg'\r\n", argv[2]);
        return;
      }

      config_reset_key(k);
      config_apply_all();
      cfg_print_key(k);
    }
    else
    {
      config_reset_all();
      config_apply_all();
      debug_uart_puts("all keys back to compiled defaults\r\n");
    }

    debug_uart_printf("  trip %lu mA, limit %u%% - all keys re-applied\r\n",
                      (unsigned long)isense_trip_ma(),
                      (unsigned)(drive_limit() / 10u));
    debug_uart_puts("  (in RAM only - 'cfg save' to make it stick)\r\n");
    return;
  }

  if (strcmp(argv[1], "help") == 0)
  {
    for (uint16_t i = 0u; i < (uint16_t)CFG_KEY_COUNT; i++)
    {
      /* The worst offender of the lot: a help string is far longer than a
         value line, so this listing is several times the ring. */
      console_pace(160u);
      debug_uart_printf("  %-11s %s\r\n",
                        config_name((config_key_t)i),
                        config_help((config_key_t)i));
    }
    return;
  }

  /* Anything else is a key name. */
  config_key_t k = config_find(argv[1]);

  if (k == CFG_KEY_COUNT)
  {
    debug_uart_printf("unknown key '%s' - try 'cfg'\r\n", argv[1]);
    return;
  }

  if (argc < 3)
  {
    cfg_print_key(k);
    debug_uart_printf("  %s\r\n", config_help(k));
    return;
  }

  int32_t want = (int32_t)strtol(argv[2], NULL, 10);

  /* Rejected rather than clamped on purpose - see config_set(). A typo that
     silently becomes the nearest legal value is a typo nobody finds. */
  if (!config_set(k, want))
  {
    debug_uart_printf("out of range - %s accepts %ld..%ld %s\r\n",
                      config_name(k),
                      (long)config_min(k),
                      (long)config_max(k),
                      config_units(k));
    return;
  }

  cfg_print_key(k);

  /* Same path as CAN CFG_REQ SET. What follows only reports what is now in
     force, read back from the module rather than echoed from the request. */
  config_apply_live(k);

  if (k == CFG_TRIP_BOOT_MA)
  {
    debug_uart_printf("  applied now: trip %lu mA\r\n",
                      (unsigned long)isense_trip_ma());
  }
  else if (k == CFG_DUTY_LIMIT)
  {
    debug_uart_printf("  applied now: limit %u%%\r\n",
                      (unsigned)(drive_limit() / 10u));
  }
  else if (k == CFG_RAMP_PMPS)
  {
    debug_uart_printf("  applied now: ramp %u o/oo/s\r\n",
                      (unsigned)drive_ramp());
  }
  else if (k == CFG_RAMP_FLOOR)
  {
    debug_uart_printf("  applied now: ramp floor %u o/oo\r\n",
                      (unsigned)drive_ramp_floor());
  }

  /* 'vel reset' when a clean integrator start is what you want. */
  else if (k == CFG_VEL_KP)
  {
    debug_uart_printf("  applied now: kp %ld x1000\r\n", (long)velocity_kp());
  }
  else if (k == CFG_VEL_KI)
  {
    debug_uart_printf("  applied now: ki %ld x1000\r\n", (long)velocity_ki());
  }
  else if (k == CFG_VEL_KD)
  {
    debug_uart_printf("  applied now: kd %ld x1000\r\n", (long)velocity_kd());
  }
  else if (k == CFG_VEL_FF_SLOPE)
  {
    debug_uart_printf("  applied now: ff slope %ld x1000 o/oo per rpm\r\n",
                      (long)velocity_ff_slope());
  }
  else if (k == CFG_VEL_FF_OFFSET)
  {
    debug_uart_printf("  applied now: ff offset %ld o/oo\r\n",
                      (long)velocity_ff_offset());
  }
  else if (k == CFG_VEL_I_LIMIT)
  {
    debug_uart_printf("  applied now: integrator clamp %u o/oo\r\n",
                      (unsigned)velocity_i_limit());
  }
  else if (k == CFG_VEL_MAX)
  {
    debug_uart_printf("  applied now: vel max %u o/oo (drv limit %u -"
                      " the tighter wins)\r\n",
                      (unsigned)velocity_max(), (unsigned)drive_limit());
  }
  else if (k == CFG_VEL_SLEW)
  {
    debug_uart_printf("  applied now: setpoint ramp %ld m rpm/s\r\n",
                      (long)velocity_slew());
  }
  else if (k == CFG_VEL_TIMEOUT)
  {
    debug_uart_printf("  applied now: setpoint watchdog %lu ms%s\r\n",
                      (unsigned long)velocity_timeout(),
                      (velocity_timeout() == 0u) ? " - DISARMED" : "");
  }

  /* Three keys change the meaning of every current number the board reports,
     so say so at the point of change rather than leaving it to be rediscovered
     when a log stops matching a meter. */
  if ((k == CFG_R_IPROPI_OHM) || (k == CFG_A_IPROPI_UA_PER_A) ||
      (k == CFG_VDDA_MV))
  {
    debug_uart_printf("  current scale is now %u mA full-scale;"
                      " trip re-applied at %lu mA\r\n",
                      (unsigned)isense_full_scale_ma(),
                      (unsigned long)isense_trip_ma());
  }

  debug_uart_puts("  (live now, but in RAM - 'cfg save' to keep it across a"
                  " reset)\r\n");
}

/**
  * @brief  Module identity: what was latched at boot, and what the pins say now.
  * @note   The two can differ, and that is the point - a switch moved since
  *         boot shows up here instead of silently doing nothing.
  */
static void cmd_id(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  uint8_t latched = dipsw_code();
  uint8_t live    = dipsw_read_live();

  debug_uart_printf("module ID : %u  (0b%u%u%u%u, SW3 SW2 SW1 SW0)\r\n",
                    (unsigned)latched,
                    (unsigned)((latched >> 3) & 1u),
                    (unsigned)((latched >> 2) & 1u),
                    (unsigned)((latched >> 1) & 1u),
                    (unsigned)(latched & 1u));
  debug_uart_printf("role      : %s\r\n", dipsw_role_str(dipsw_role()));

  if (dipsw_valid())
  {
    debug_uart_printf("CAN node  : 0x%03lX\r\n", (unsigned long)dipsw_can_id());
  }
  else
  {
    debug_uart_puts("CAN node  : none - transmit disabled\r\n");
    debug_uart_puts("            0b1111 is what an unfitted switch block reads;"
                    " 0 is the broadcast\r\n"
                    "            address. Set PB12..PB15 (SW0..SW3) to an ID"
                    " 1-14; closed = 0, open = 1.\r\n");
  }

  if (live != latched)
  {
    debug_uart_printf("pins now  : %u - CHANGED SINCE BOOT."
                      " Identity is latched; reset to adopt it.\r\n",
                      (unsigned)live);
  }
}

/* --- telem ---------------------------------------------------------------- *
 * The machine-readable counterpart to `enc watch`. That command exists to be
 * read by a person at 5 Hz; this one exists to be parsed by a host at up to
 * 100 Hz, and the two requirements pull in opposite directions - hence a second
 * format rather than a flag on the first.
 *
 * Everything a bench run needs is on ONE line, because the alternative is
 * correlating separate `enc` and `drv current` replies by host arrival time,
 * which is exactly the uncertainty the stream exists to remove.
 *
 *     T,<seq>,<ms>,<duty>,<count>,<milli_rpm>,<mA>,<flags>
 *
 *   seq    uint32, +1 per emitted line. A GAP MEANS LINES WERE DROPPED, which
 *          the host cannot otherwise distinguish from the board being busy.
 *          Cross-check against `stats` tx_dropped.
 *   ms     HAL_GetTick() at emission. BOARD time - the console has never
 *          exposed one, and host arrival time carries the UART's latency plus
 *          the OS's scheduling on top of the jitter actually being measured.
 *   duty   signed per-mille, as commanded (post-clamp, post-limit).
 *   count  int32 encoder position. THIS IS THE REAL MEASUREMENT: it is exact
 *          and unfiltered, so the host can differentiate it at whatever
 *          smoothing an analysis wants.
 *   mrpm   rpm x 1000. A CONVENIENCE, not the primary signal: it comes from
 *          encoder_rpm(), which is a boxcar average over `enc window` ticks
 *          and therefore lags. Fitting a time constant to this column would
 *          measure the FILTER, not the plant. Differentiate count instead.
 *   mA     current. Flags bit 0 says which quantity: motor current when the
 *          sample was phase-synchronised, supply current when it was not.
 *          They are in different units - see isense.h - so a host that ignores
 *          the flag will silently mix them. Bit 5 (32) marks a synchronised
 *          sample taken in the slow-decay BRAKE phase (below 14.5% duty),
 *          already scaled to motor current; +/-4% rather than the drive
 *          phase's ~2%.
 *   flags  1 sync  2 enabled  4 fault latched  8 ADC saturated  16 watchdog
 *          32 brake-phase sample
 *
 * Integer fields throughout. "%f" pulls in newlib's float formatter, which is
 * far too slow to run a hundred times a second, and milli-rpm keeps three
 * decimals without it.
 * -------------------------------------------------------------------------- */

static void print_telem_line(void)
{
  int64_t pos = encoder_position();

  if (pos >  2147483647LL) { pos =  2147483647LL; }
  if (pos < -2147483648LL) { pos = -2147483648LL; }

  /* Asked before reading, not inferred from the result - the same reason
     `drv current` asks: a zero reading means both "no current" and "could not
     measure", and the host has to be able to tell them apart. */
  bool     sync = isense_sync_ready();
  uint16_t raw  = sync ? isense_read_sync_avg(TELEM_ISENSE_SAMPLES)
                       : isense_read_avg(TELEM_ISENSE_SAMPLES);

  uint8_t flags = 0u;
  if (sync)                    { flags |= 0x01u; }
  if (drive_is_enabled())      { flags |= 0x02u; }
  if (drive_fault_latched())   { flags |= 0x04u; }
  if (isense_saturated())      { flags |= 0x08u; }
  if (drive_timeout_expired()) { flags |= 0x10u; }
  if (sync && (drive_sense_kind() == DRIVE_SENSE_DECAY)) { flags |= 0x20u; }

  debug_uart_printf("T,%lu,%lu,%d,%ld,%ld,%lu,%u\r\n",
                    (unsigned long)telem_seq++,
                    (unsigned long)HAL_GetTick(),
                    (int)drive_duty(),
                    (long)pos,
                    (long)(encoder_rpm() * 1000.0f),
                    (unsigned long)isense_raw_to_ma(raw),
                    (unsigned)flags);
}

/* -----------------------------------------------------------------------------
 * THE VELOCITY RECORD
 *
 *   V,<seq>,<ms>,<sp_mrpm>,<meas_mrpm>,<out>,<ff>,<p>,<i>,<d>,<flags>
 *
 * One line per CONTROL STEP, not per telemetry period. velocity_on_tick()
 * advances only when the encoder's velocity window closes - 50 Hz at the
 * default window 20 - and the loop publishes a snapshot each time it does.
 * This drains that. Sampling it on the `telem` timer instead would alias it:
 * at 100 Hz every step would appear twice and at 30 Hz they would beat, and
 * the integrator and the derivative only mean anything per step.
 *
 *   seq   uint32, +1 per LINE, restarting at 0 on `telem on` - exactly T's
 *         semantics, so the same gap detector works. A gap means the line was
 *         dropped on the wire. A dropped STEP is a different failure and is
 *         reported separately, by flag bit 64.
 *   ms    HAL_GetTick() at the step. The join key against the T stream.
 *   sp    the RAMPED setpoint in milli-rpm - what the loop actually chased,
 *         which during a ramp is not what was commanded. Bit 32 says which.
 *   meas  encoder_rpm() x1000 AS THE LOOP SAW IT. This is deliberately the
 *         boxcar-filtered figure and not a fresh differentiation of `count`:
 *         the question this stream answers is what the controller did, and
 *         the controller acted on the filtered value. Fit the PLANT from T's
 *         `count`; judge the LOOP from this.
 *   out   per-mille handed to drive_set_duty(). T's `duty` is what the bridge
 *         then ran - they differ while drv's own slew limiter is armed.
 *   ff p i d  the four contributions, per-mille, summing to the unclamped
 *         output. Separated because "it oscillates" and "it winds up" look
 *         identical in `out` and want opposite corrections.
 *   flags 1 saturated  2 integrator frozen  4 ...slewing  8 ...no bridge
 *         16 watchdog expired  32 setpoint ramping  64 a step was dropped
 *
 * Integer fields throughout, for the reason above print_telem_line().
 * -------------------------------------------------------------------------- */

static void print_velocity_line(const velocity_sample_t *s)
{
  debug_uart_printf("V,%lu,%lu,%ld,%ld,%d,%d,%d,%d,%d,%u\r\n",
                    (unsigned long)telem_vseq++,
                    (unsigned long)s->ms,
                    (long)s->sp_mrpm,
                    (long)s->meas_mrpm,
                    (int)s->out,
                    (int)s->ff,
                    (int)s->p,
                    (int)s->i,
                    (int)s->d,
                    (unsigned)s->flags);
}

static void cmd_telem(int argc, char **argv)
{
  if (argc < 2)
  {
    debug_uart_printf("telem %s, period %u ms (%u Hz), seq %lu\r\n",
                      telem_on ? "on" : "off",
                      (unsigned)telem_ms,
                      (unsigned)(1000u / telem_ms),
                      (unsigned long)telem_seq);
    debug_uart_puts("  T,seq,ms,duty_permille,count,milli_rpm,mA,flags\r\n");
    debug_uart_puts("  flags: 1 sync  2 enabled  4 fault  8 saturated"
                    "  16 watchdog\r\n");
    debug_uart_puts("  count is the measurement; milli_rpm is filtered by"
                    " 'enc window' and lags\r\n");

    debug_uart_printf("velocity channel %s, vseq %lu\r\n",
                      telem_vel_on ? "on" : "off",
                      (unsigned long)telem_vseq);
    debug_uart_puts("  V,seq,ms,sp_mrpm,meas_mrpm,out,ff,p,i,d,flags\r\n");
    debug_uart_puts("  flags: 1 saturated  2 frozen  4 slewing  8 no bridge"
                    "  16 watchdog  32 ramping  64 step dropped\r\n");
    debug_uart_puts("  one line per CONTROL STEP (1000/'enc window' Hz),"
                    " not per telem period\r\n");

    debug_uart_puts("  sub: on | off | rate <1..100 hz> | vel on|off\r\n");
    return;
  }

  if (strcmp(argv[1], "vel") == 0)
  {
    bool von;

    if ((argc < 3) || !parse_on_off(argv[2], &von))
    {
      debug_uart_printf("velocity channel is %s\r\n",
                        telem_vel_on ? "on" : "off");
      return;
    }

    telem_vel_on = von;
    debug_uart_printf("velocity channel %s\r\n", von ? "on" : "off");

    if (von)
    {
      if (!velocity_enabled())
      {
        debug_uart_puts("  the loop is off, so nothing will be emitted -"
                        " 'vel on' to arm it\r\n");
      }

      /* 100 Hz of T plus 50 Hz of V is ~9.7 kB/s of an 11.52 kB/s wire, and
         the echo of anything typed then comes out of what is left. The board
         will not refuse it - a short burst is fine and tx_dropped counts the
         damage - but the pairing that actually fits is said out loud. */
      if (telem_ms < 20u)
      {
        debug_uart_printf("  WARNING: T is at %u Hz. Both channels together"
                          " will outrun 115200.\r\n"
                          "  'telem rate 50' is the pairing that fits;"
                          " check 'stats' for tx_dropped\r\n",
                          (unsigned)(1000u / telem_ms));
      }

      if (!telem_on)
      {
        debug_uart_puts("  'telem on' is still the master switch - nothing"
                        " streams until it is on\r\n");
      }
    }

    return;
  }

  if (strcmp(argv[1], "rate") == 0)
  {
    if (argc < 3)
    {
      debug_uart_puts("usage: telem rate <1..100>\r\n");
      return;
    }

    unsigned long hz = strtoul(argv[2], NULL, 10);

    if ((hz < TELEM_MIN_HZ) || (hz > TELEM_MAX_HZ))
    {
      debug_uart_printf("rate must be %u..%u Hz - above that a %u-byte line"
                        " outruns 115200\r\n",
                        (unsigned)TELEM_MIN_HZ, (unsigned)TELEM_MAX_HZ, 55u);
      return;
    }

    /* The period is what the scheduler actually uses, so it is what gets
       stored and reported. 1000/hz truncates - 60 Hz becomes 16 ms, which is
       62.5 Hz - and rather than hide that, every line carries its own `ms`. */
    telem_ms   = (uint16_t)(1000u / hz);
    telem_next = HAL_GetTick();

    debug_uart_printf("rate %u Hz (period %u ms)\r\n",
                      (unsigned)(1000u / telem_ms), (unsigned)telem_ms);
    return;
  }

  bool on;

  if (!parse_on_off(argv[1], &on))
  {
    debug_uart_puts("usage: telem on|off | telem rate <hz>"
                    " | telem vel on|off\r\n");
    return;
  }

  telem_on = on;

  if (on)
  {
    telem_seq  = 0u;
    telem_vseq = 0u;
    telem_next = HAL_GetTick();

    debug_uart_printf("telem on at %u Hz - seq restarted at 0%s\r\n",
                      (unsigned)(1000u / telem_ms),
                      telem_vel_on ? ", velocity channel on" : "");

    if (monitor_on)
    {
      debug_uart_puts("  WARNING: 'monitor on' interleaves CAN frame lines into"
                      " the stream.\r\n"
                      "  'monitor off' first, or the host parser sees them as"
                      " corrupt telemetry\r\n");
    }
  }
  else
  {
    debug_uart_printf("telem off - %lu T lines, %lu V lines sent\r\n",
                      (unsigned long)telem_seq,
                      (unsigned long)telem_vseq);
  }
}

static void cmd_steer(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  uint8_t f = steer_flags();

  debug_uart_printf("steer %s, position %s | pos %+.2f deg (%+ld p) | target"
                    " %+.2f deg (%+ld p)%s%s%s\r\n",
                    (f & STEER_F_ENABLED)   ? "enabled" : "disabled",
                    (f & STEER_F_POS_VALID) ? "valid"   : "LOST",
                    (double)steer_position_cdeg() / 100.0,
                    (long)steer_position_pulses(),
                    (double)steer_target_cdeg() / 100.0,
                    (long)steer_target_pulses(),
                    (f & STEER_F_MOVING)   ? " | moving"     : "",
                    (f & STEER_F_STALL)    ? " | stall"      : "",
                    (f & STEER_F_UART_ERR) ? " | uart error" : "");
  debug_uart_puts("  0x33 check: pos == -(mks pulses now - mks pulses at enable)\r\n");
}

static void cmd_estop(int argc, char **argv)
{
  if (argc < 2)
  {
    debug_uart_printf("estop %s\r\n",
                      motion_estop_latched() ? "LATCHED - 'estop clear' to recover"
                                             : "clear");
    return;
  }

  if (strcmp(argv[1], "clear") != 0)
  {
    debug_uart_puts("usage: estop [clear]\r\n");
    return;
  }

  if (!motion_estop_clear())
  {
    debug_uart_puts("loop is armed - 'vel off' first\r\n");
    return;
  }

  debug_uart_puts("estop clear - motion commands accepted again\r\n");
}

static void cmd_can(int argc, char **argv)
{
  if ((argc >= 2) && (strcmp(argv[1], "crc") == 0))
  {
    static const uint8_t check[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
    uint8_t crc = can_cmd_crc8(check, sizeof(check));

    debug_uart_printf("crc8 sae-j1850 \"123456789\" = 0x%02X (expect 0x4B) %s\r\n",
                      (unsigned)crc, (crc == 0x4Bu) ? "PASS" : "FAIL");
    return;
  }

  if (argc >= 2)
  {
    debug_uart_puts("usage: can [crc]\r\n");
    return;
  }

  const can_cmd_stats_t *c = can_cmd_stats();

  debug_uart_printf("can cmd: %lu handled, %lu ignored, %lu rejected\r\n",
                    (unsigned long)c->handled, (unsigned long)c->ignored,
                    (unsigned long)c->rejected);
  debug_uart_printf("can tx : %lu queued, %lu dropped (no mailbox)\r\n",
                    (unsigned long)c->tx_frames, (unsigned long)c->tx_dropped);
  debug_uart_printf("can rx : %lu speed ok, %lu crc, %lu repeat, %lu stale,"
                    " %lu skipped\r\n",
                    (unsigned long)c->speed_ok, (unsigned long)c->crc_errors,
                    (unsigned long)c->ctr_repeat, (unsigned long)c->ctr_stale,
                    (unsigned long)c->ctr_skipped);
  debug_uart_printf("status : %lu STATUS_DRIVE sent\r\n",
                    (unsigned long)c->status_tx);
  static const char *const owners[] = { "none", "UART", "CAN" };

  debug_uart_printf("flags  : 0x%02X, estop %s, motion owner %s\r\n",
                    (unsigned)can_cmd_flags(),
                    motion_estop_latched() ? "LATCHED" : "clear",
                    owners[motion_owner()]);
}

static const command_t commands[] =
{
  { "help",      "",             "list these commands",                       cmd_help      },
  { "info",      "",             "clocks, live CAN bit timing, current modes", cmd_info      },
  { "stats",     "",             "UART and CAN counters",                     cmd_stats     },
  { "errors",    "",             "CAN error registers, decoded",              cmd_errors    },
  { "clear",     "",             "zero the software counters",                cmd_clear     },
  { "send",      "<id> [hex]",   "transmit a CAN frame, e.g. send 123 DEADBEEF", cmd_send   },
  { "heartbeat", "[on|off]",     "periodic frame at this module's ID - off to silence the bus", cmd_heartbeat },
  { "monitor",   "[on|off]",     "print received CAN frames as they arrive",  cmd_monitor   },
  { "canhold",   "<ms>",         "TEST: pause the CAN RX ring drain for <ms>", cmd_canhold  },
  { "loopback",  "[on|off]",     "CAN loopback - test with no bus attached",  cmd_loopback  },
  { "mks",       "<sub> [args]", "MKS SERVO42C on UART4 - 'mks' for subcommands", cmd_mks   },
  { "enc",       "[sub]",        "drive encoder - 'enc' for position and speed", cmd_enc   },
  { "drv",       "[sub]",        "drive H-bridge - 'drv' for state",          cmd_drv       },
  { "vel",       "[sub]",        "closed-loop wheel speed - 'vel' for state", cmd_vel   },
  { "telem",     "[sub]",        "machine-readable stream for the bench host", cmd_telem   },
  { "cfg",       "[key] [val]",  "stored tunables - 'cfg' to list",           cmd_cfg       },
  { "id",        "",             "module identity from the DIP switches",     cmd_id        },
  { "can",       "[crc]",        "W6 command layer counters - 'can crc' self-test", cmd_can },
  { "steer",     "",             "CAN steering: tracked position and target",  cmd_steer     },
  { "estop",     "[clear]",      "ESTOP latch state - 'estop clear' to recover", cmd_estop  },
  { "reset",     "",             "reboot the MCU",                            cmd_reset     },
};

#define COMMAND_COUNT (sizeof(commands) / sizeof(commands[0]))

static void cmd_help(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  debug_uart_puts("commands:\r\n");

  for (size_t i = 0u; i < COMMAND_COUNT; i++)
  {
    console_pace(128u);         /* see console_pace(): the ring drops, it does
                                   not block, and this table outgrows it */
    debug_uart_printf("  %-10s %-12s %s\r\n",
                      commands[i].name, commands[i].args, commands[i].help);
  }
}

/* -------------------------------------------------------------------------- */
/* Dispatch and line editing                                                   */
/* -------------------------------------------------------------------------- */

/**
  * @brief  Tokenize and run one command line.
  *
  * Matching is exact — no prefixes or abbreviations — so a typo is reported
  * rather than resolved to something that happens to share a prefix. An empty
  * line is silently ignored.
  */
static void dispatch(char *text)
{
  char *argv[CONSOLE_MAX_TOKENS];
  int   argc = tokenize(text, argv, (int)CONSOLE_MAX_TOKENS);

  if (argc == 0)
  {
    return;
  }

  for (size_t i = 0u; i < COMMAND_COUNT; i++)
  {
    if (strcmp(argv[0], commands[i].name) == 0)
    {
      commands[i].fn(argc, argv);
      return;
    }
  }

  debug_uart_printf("unknown command '%s' - try 'help'\r\n", argv[0]);
}

/** @brief Terminate, dispatch, reset the buffer and print a fresh prompt.
  *        Reached from either terminator path — CR/LF or the idle fallback. */
static void execute_line(void)
{
  debug_uart_puts("\r\n");
  line[line_len] = '\0';
  dispatch(line);
  line_len  = 0u;
  burst_len = 0u;

  /* Pace here and the prompt survives whatever the handler just did to the
     ring, for every command that exists and every one added later. This is
     not cosmetic: the host tooling's ask() keys on the prompt to know a reply
     is complete, so a dropped prompt is the difference between a listing with
     a cosmetic hole in it and a hard timeout that fails the run. Handlers
     should still pace their own long listings -- this only protects the two
     bytes below, not the output above it. */
  console_pace(64u);
  debug_uart_puts(CONSOLE_PROMPT);
}

void console_init(void)
{
  line_len  = 0u;
  burst_len = 0u;
  skip_lf   = false;

  debug_uart_puts("type 'help' for commands\r\n" CONSOLE_PROMPT);
}

void console_poll(void)
{
  uint8_t ch;

  while (debug_uart_read(&ch, 1u) == 1u)
  {
    burst_len++;

    if (skip_lf && (ch == '\n'))
    {
      skip_lf = false;
      continue;
    }

    skip_lf = false;

    if ((ch == '\r') || (ch == '\n'))
    {
      skip_lf = (ch == '\r');
      execute_line();
    }
    else if ((ch == 0x08u) || (ch == 0x7Fu))     /* backspace / delete */
    {
      if (line_len > 0u)
      {
        line_len--;
        debug_uart_puts("\b \b");
      }
    }
    else if ((ch >= 0x20u) && (ch < 0x7Fu))      /* printable only */
    {
      if (line_len < (CONSOLE_LINE_MAX - 1u))
      {
        line[line_len++] = (char)ch;
        (void)debug_uart_write(&ch, 1u);          /* echo so typing is visible */
      }
    }
  }

  /* Terminals that transmit a whole command in one burst and append no CR/LF
     never deliver a terminator, so the idle line is the only frame boundary
     available — the same delimiter the SERVO42C protocol will rely on.
     Requiring more than one byte in the burst is what keeps interactive typing
     working: a keystroke arrives alone and falls quiet, so it would otherwise
     execute a character at a time. */
  if (debug_uart_take_idle_event())
  {
    if ((line_len > 0u) && (burst_len > 1u))
    {
      execute_line();
    }

    burst_len = 0u;
  }
}

/**
  * @brief  Print the outcome of a finished MKS transaction.
  *
  * Replies are decoded by the function code that was sent, not by reply
  * length — different commands produce equal-length replies, so length alone
  * is ambiguous.
  *
  * On failure it prints the most likely cause rather than just the error:
  * a motion command that times out while reads still work is the CR_vFOC
  * signature, and no reply at all points at wiring, baud or power.
  */
void console_report_mks(void)
{
  if (!mks_take_completion())
  {
    return;
  }

  mks_result_t result = mks_result();
  uint8_t      reply[MKS_MAX_FRAME];
  size_t       n = mks_response(reply, sizeof(reply));

  debug_uart_printf("mks %s", mks_result_str(result));

  if (n > 0u)
  {
    debug_uart_puts(" | ");
    debug_uart_write_hex(reply, n);
  }

  debug_uart_puts("\r\n");

  if (result != MKS_RESULT_OK)
  {
    /* The single most common cause, and the one the vendor docs call out:
       reads answer in any mode, motion commands are ignored unless the driver
       is in CR_UART. A unit that reports its encoder but will not move is
       almost certainly still in CR_vFOC. */
    if (result == MKS_RESULT_TIMEOUT)
    {
      uint8_t fn = mks_last_function();

      if ((fn == 0xFDu) || (fn == 0xF6u) || (fn == 0xF3u) || (fn == 0xF7u))
      {
        debug_uart_puts("  hint    : motion commands need Mode = CR_UART. If reads\r\n"
                        "            answer but this does not, the driver is in CR_vFOC.\r\n");
      }
      else
      {
        debug_uart_puts("  hint    : no reply at all - check PA0->RX / PA1<-TX (crossed),\r\n"
                        "            common ground, 38400 baud, and driver power.\r\n");
      }
    }

    return;
  }

  /* Decode by what was asked, not by reply length — lengths collide. */
  switch (mks_last_function())
  {
    case 0x30u:
    {
      int32_t  carry;
      uint16_t value;

      if (mks_decode_encoder(&carry, &value))
      {
        debug_uart_printf("  encoder : carry %ld, value %u\r\n",
                          (long)carry, (unsigned)value);
      }
      break;
    }

    case 0x33u:
    {
      int32_t pulses;

      if (mks_decode_int32(&pulses))
      {
        debug_uart_printf("  pulses  : %ld\r\n", (long)pulses);
      }
      break;
    }

    case 0x39u:
    {
      int16_t raw;

      if (mks_decode_int16(&raw))
      {
        debug_uart_printf("  angle   : %d raw = %.3f deg at the MOTOR shaft "
                          "(/19 => %.4f deg at output)\r\n",
                          (int)raw,
                          (double)mks_angle_error_degrees(raw),
                          (double)(mks_angle_error_degrees(raw) / 19.0f));
      }
      break;
    }

    case 0x3Au:
    {
      uint8_t status;

      if (mks_decode_byte(&status))
      {
        debug_uart_printf("  EN pin  : %s (0x%02X)\r\n",
                          (status == 0x01u) ? "enabled" :
                          (status == 0x02u) ? "disabled" : "?", status);
      }
      break;
    }

    case 0x3Eu:
    {
      uint8_t status;

      if (mks_decode_byte(&status))
      {
        debug_uart_printf("  protect : %s (0x%02X)\r\n",
                          (status == 0x01u) ? "PROTECTED (locked rotor)" :
                          (status == 0x02u) ? "clean" : "?", status);
      }
      break;
    }

    case 0xFDu:
    case 0xF6u:
      debug_uart_puts("  motion  : complete\r\n");
      break;

    default:
      break;
  }
}

bool console_heartbeat_enabled(void)
{
  return heartbeat_on;
}

void console_report_encoder(void)
{
  if (!enc_watch_on)
  {
    return;
  }

  /* 5 Hz. Fast enough to follow a shaft turned by hand, slow enough that the
     output does not swamp the console or the 115200 wire. */
  uint32_t now = HAL_GetTick();

  if ((now - enc_watch_last) < 200u)
  {
    return;
  }

  enc_watch_last = now;
  print_encoder_line();
}

void console_report_telem(void)
{
  if (!telem_on)
  {
    return;
  }

  /* The velocity channel FIRST, and outside the schedule below. It is paced by
     the control loop, not by telem_ms - draining it inside the period check
     would throttle a 50 Hz loop to whatever T happens to be set to, and the
     whole reason this is a separate record is that it must not be resampled.

     One per call: there is only ever one slot, so a second call would return
     false anyway. The main loop runs far faster than any loop rate we can
     configure, and when it does not, the dropped step comes back as flag 64
     rather than as silence. */
  if (telem_vel_on)
  {
    velocity_sample_t vs;

    if (velocity_take_sample(&vs))
    {
      print_velocity_line(&vs);
    }
  }

  uint32_t now = HAL_GetTick();

  /* Signed difference, so the tick wrap at 49.7 days shortens one interval
     instead of stalling the stream for another 49.7. */
  if ((int32_t)(now - telem_next) < 0)
  {
    return;
  }

  /* Advance by the period rather than from `now`, so the schedule does not
     creep later by one main-loop latency on every single line. */
  telem_next += telem_ms;

  /* Unless we have already fallen a whole period behind - then resynchronise
     rather than emit a catch-up burst. A burst would arrive as a clump of
     lines with near-identical `ms`, overflow the TX ring, and show up on the
     host as a seq gap: three lies about the timing, to repay a debt the host
     can already see in the timestamps. */
  if ((int32_t)(now - telem_next) > 0)
  {
    telem_next = now + telem_ms;
  }

  print_telem_line();
}

bool console_monitor_enabled(void)
{
  return monitor_on;
}

bool console_can_hold_active(void)
{
  if (can_hold_set && ((int32_t)(HAL_GetTick() - can_hold_until) < 0))
  {
    return true;
  }

  can_hold_set = false;
  return false;
}

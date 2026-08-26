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
#include "encoder.h"
#include "drive.h"

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

/** @brief CAN fault-confinement state as a word, worst condition first. */
static const char *can_state_str(void)
{
  if (can_bus_is_bus_off())       { return "BUS-OFF"; }
  if (can_bus_is_error_passive()) { return "error-passive"; }
  if (can_bus_is_error_warning()) { return "error-warning"; }
  return "error-active";
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
  debug_uart_printf("heartbeat : %s, ID 0x%03lX\r\n",
                    heartbeat_on ? "on" : "off",
                    (unsigned long)CAN_ID_HEARTBEAT_BASE);
  debug_uart_printf("monitor   : %s\r\n", monitor_on ? "on" : "off");
}

static void cmd_stats(int argc, char **argv)
{
  const debug_uart_stats_t *u = debug_uart_stats();
  const can_bus_stats_t    *c = can_bus_stats();

  (void)argc;
  (void)argv;

  debug_uart_printf("uart tx   : %lu bytes, %lu dropped\r\n", u->tx_bytes, u->tx_dropped);
  debug_uart_printf("uart rx   : %lu bytes, high-water %lu, %lu overruns, %lu errors\r\n",
                    u->rx_bytes, u->rx_high_water, u->rx_overruns, u->rx_errors);
  debug_uart_printf("can  tx   : %lu frames, %lu dropped (no free mailbox)\r\n",
                    c->tx_frames, c->tx_dropped);
  debug_uart_printf("can  rx   : %lu frames, %lu FIFO-full, %lu overrun events\r\n",
                    c->rx_frames, c->rx_fifo_full, c->rx_overruns);

  if (c->rx_overruns != 0u)
  {
    debug_uart_puts("  warning : FIFO0 overran — frames were lost. The counter is\r\n"
                    "            events, not frames; the hardware cannot say how many.\r\n");
  }
  else if (c->rx_fifo_full != 0u)
  {
    debug_uart_puts("  note    : FIFO0 hit its 3-message depth but nothing was lost —\r\n"
                    "            the drain loop is keeping up, with no margin to spare.\r\n");
  }
}

static void cmd_errors(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  debug_uart_printf("CAN_ESR   : 0x%08lX\r\n", can_bus_esr());
  debug_uart_printf("  TEC     : %u\r\n", can_bus_tec());
  debug_uart_printf("  REC     : %u\r\n", can_bus_rec());
  debug_uart_printf("  last err: %s\r\n", can_bus_last_error_str());
  debug_uart_printf("  state   : %s\r\n", can_state_str());

  if (can_bus_last_error() == 3u)
  {
    debug_uart_puts("  note    : 'ack' means the frame went out but no other node\r\n"
                    "            acknowledged it — a transmitter cannot ACK itself.\r\n");
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
    debug_uart_printf("bad id '%s' — expected 11-bit hex, 000..7FF\r\n", argv[1]);
    return;
  }

  if ((argc > 2) && !parse_hex_bytes(argc - 2, &argv[2], payload, &len, sizeof(payload)))
  {
    debug_uart_puts("bad payload — expected up to 8 bytes as hex digit pairs\r\n");
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
    debug_uart_printf("send failed — no free mailbox (state %s, last err %s)\r\n",
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
    debug_uart_puts("mks transaction aborted (motor NOT stopped — use 'mks stop')\r\n");
    return;
  }

  if (mks_busy())
  {
    debug_uart_puts("mks busy — a transaction is outstanding ('mks abort' to drop it)\r\n");
    return;
  }

  bool started = false;

  if      (strcmp(argv[1], "encoder") == 0) { started = mks_read_encoder(); }
  else if (strcmp(argv[1], "pulses")  == 0) { started = mks_read_pulses(); }
  else if (strcmp(argv[1], "angle")   == 0) { started = mks_read_angle_error(); }
  else if (strcmp(argv[1], "en")      == 0) { started = mks_read_en_status(); }
  else if (strcmp(argv[1], "protect") == 0) { started = mks_read_protect_state(); }
  else if (strcmp(argv[1], "stop")    == 0) { started = mks_stop(); }
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
        debug_uart_puts("value too small — one pulse is 0.0118 deg at the output\r\n");
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
    debug_uart_printf("unknown subcommand '%s' — try 'mks'\r\n", argv[1]);
    return;
  }

  if (!started)
  {
    debug_uart_puts("could not start transaction\r\n");
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
    debug_uart_printf("watch %s\r\n", on ? "on — turn the shaft" : "off");
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

    debug_uart_printf("window %u ticks — %.0f Hz update, %.2f rpm per count\r\n",
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

    debug_uart_printf("watching A/B for %lu ms — TURN THE SHAFT NOW\r\n",
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
    debug_uart_printf("unknown subcommand '%s' — try 'enc'\r\n", argv[1]);
  }
}

static void cmd_drv(int argc, char **argv)
{
  if (argc < 2)
  {
    debug_uart_printf("nSLEEP %s  duty %+d%%  decay %s  limit %u%%  nFAULT %s\r\n",
                      drive_is_enabled() ? "high (enabled)" : "low (disabled)",
                      drive_duty() / 10,
                      (drive_decay() == DRIVE_DECAY_SLOW) ? "slow (drive-brake)"
                                                          : "fast (sign-magnitude)",
                      (unsigned)(drive_limit() / 10u),
                      drive_faulted() ? "ASSERTED" : "clear");
    debug_uart_puts(
      "  sub: enable | disable | duty <+/-pct> | brake | coast\r\n"
      "       decay slow|fast | limit <pct>\r\n");
    return;
  }

  if (strcmp(argv[1], "enable") == 0)
  {
    drive_enable();
    debug_uart_puts("nSLEEP high — driver awake (waited 2 ms)\r\n");
  }
  else if (strcmp(argv[1], "disable") == 0)
  {
    drive_disable();
    debug_uart_puts("duty 0, nSLEEP low — driver disabled\r\n");
  }
  else if (strcmp(argv[1], "duty") == 0)
  {
    if (argc < 3)
    {
      debug_uart_puts("usage: drv duty <+/-pct>\r\n");
      return;
    }

    long pct = strtol(argv[2], NULL, 10);
    drive_set_duty((int16_t)(pct * 10));

    debug_uart_printf("duty %+d%%%s\r\n",
                      drive_duty() / 10,
                      drive_is_enabled() ? ""
                                         : "  (driver still disabled — pins only,"
                                           " which is what you want for a scope check)");
  }
  else if (strcmp(argv[1], "brake") == 0)
  {
    drive_brake();
    debug_uart_puts("both inputs high — brake\r\n");
  }
  else if (strcmp(argv[1], "coast") == 0)
  {
    drive_coast();
    debug_uart_puts("both inputs low — coast\r\n");
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
                        ? "slow — IN1 high, IN2 PWM inverted (drive/brake)"
                        : "fast — IN1 PWM, IN2 low (drive/coast)");
  }
  else if (strcmp(argv[1], "limit") == 0)
  {
    if (argc >= 3)
    {
      long pct = strtol(argv[2], NULL, 10);
      if (pct < 0) { pct = 0; }
      drive_set_limit((uint16_t)(pct * 10));
    }

    debug_uart_printf("limit %u%%\r\n", (unsigned)(drive_limit() / 10u));
  }
  else
  {
    debug_uart_printf("unknown subcommand '%s' — try 'drv'\r\n", argv[1]);
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

static const command_t commands[] =
{
  { "help",      "",             "list these commands",                       cmd_help      },
  { "info",      "",             "clocks, live CAN bit timing, current modes", cmd_info      },
  { "stats",     "",             "UART and CAN counters",                     cmd_stats     },
  { "errors",    "",             "CAN error registers, decoded",              cmd_errors    },
  { "clear",     "",             "zero the software counters",                cmd_clear     },
  { "send",      "<id> [hex]",   "transmit a CAN frame, e.g. send 123 DEADBEEF", cmd_send   },
  { "heartbeat", "[on|off]",     "periodic 0x500 frame — off to silence the bus", cmd_heartbeat },
  { "monitor",   "[on|off]",     "print received CAN frames as they arrive",  cmd_monitor   },
  { "loopback",  "[on|off]",     "CAN loopback — test with no bus attached",  cmd_loopback  },
  { "mks",       "<sub> [args]", "MKS SERVO42C on UART4 — 'mks' for subcommands", cmd_mks   },
  { "enc",       "[sub]",        "drive encoder — 'enc' for position and speed", cmd_enc   },
  { "drv",       "[sub]",        "drive H-bridge — 'drv' for state",          cmd_drv       },
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

  debug_uart_printf("unknown command '%s' — try 'help'\r\n", argv[0]);
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
        debug_uart_puts("  hint    : no reply at all — check PA0->RX / PA1<-TX (crossed),\r\n"
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

bool console_monitor_enabled(void)
{
  return monitor_on;
}

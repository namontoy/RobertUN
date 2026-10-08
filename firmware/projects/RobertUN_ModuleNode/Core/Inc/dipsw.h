/**
  ******************************************************************************
  * @file           : dipsw.h
  * @brief          : Module identity — 4-bit DIP switch on PB12..PB15
  ******************************************************************************
  * WHY THIS EXISTS
  *
  * All the wheel nodes run one binary. Identity is a property of the board, not of the
  * build: without this, W7 means one near-identical firmware image per node to
  * keep in sync, and the "which build is on this board?" failure class shows up
  * in W8 disguised as a CAN problem - the most expensive place to meet it.
  *
  * Four switches give a code 0..15, from which both the CAN node ID and the
  * module role are derived. Nothing else in the firmware is allowed to carry
  * identity. The code IS the module ID - plain binary, no inversion:
  *
  *   Pin   PB15 PB14 PB13 PB12
  *   Bit   SW3  SW2  SW1  SW0
  *
  *   ID     Role       Behaviour
  *   0      INVALID    broadcast address - never a node, do not join the bus
  *   1-4    Corner     steering (UART -> MKS SERVO42C) + drive (encoder PID)
  *   5-6    Center     drive only; the steering block stays inactive
  *   7-14   reserved   future modules / bench-test
  *   15     INVALID    unconfigured - do not join the bus
  *
  * ELECTRICAL CONVENTION - do not "simplify" these away
  *
  * Internal pull-ups, switches to GND: closed = 0, open = 1. No external
  * resistors.
  *
  * 0b1111 is deliberately an invalid code, because it is also what you read
  * from a board with no switch block fitted, a broken connection, or a
  * floating input. An unconfigured board therefore fails loudly instead of
  * silently impersonating module 15.
  *
  * 0 is refused for a different reason: address 0 is the broadcast address
  * (as node-ID 0 is in CANopen), and a node that owned it would answer for
  * every node on the bus. A board with every switch closed is a deliberate
  * setting, but never a valid one.
  *
  * WHY INVALID DOES NOT HALT (yet)
  *
  * The recorded decision says an invalid ID halts and blinks. Taken literally
  * that would brick every bench board without a switch block: all four pins
  * float high and the board reads 0b1111. Halting would remove the console -
  * the only way to bring a board up - to protect a bus that has not been built.
  *
  * So the halt is deferred, not dropped. What is implemented is the part that
  * carries the safety: the ID is latched, the boot banner says loudly that
  * identity is unusable and why, and dipsw_valid() is false. The heartbeat is
  * gated on dipsw_valid(), and so must every node transmit that W6 adds - which
  * is what "do not join the bus" actually means.
  *
  * WHY THE PINS ARE ALSO CONFIGURED HERE
  *
  * All four pins are in the .ioc, so MX_GPIO_Init() already sets them up.
  * dipsw_init() configures them again anyway. A regeneration has already
  * silently emptied a USER CODE block on this project once, and the .ioc is the
  * one file we cannot defend. A file we own cannot lose its own init. The pin
  * macros in dipsw.c fall back to PB12..PB15 if main.h ever stops defining them.
  *
  * LATCH ONCE
  *
  * Read in main() before anything that depends on role, and never again.
  * Identity must not change mid-run: a switch nudged with a screwdriver while
  * the nodes are live would otherwise hand two boards the same CAN ID, and the
  * symptom of that is not "wrong ID", it is arbitration chaos on the bus.
  ******************************************************************************
  */
#ifndef __DIPSW_H__
#define __DIPSW_H__

#include <stdbool.h>
#include <stdint.h>

/** @brief Raw code read from a board with nothing fitted (all pins pulled
  *        up). Never a valid identity. */
#define DIPSW_CODE_UNFITTED  0x0Fu

/** @brief The broadcast address. Never a valid identity: no node may own it. */
#define DIPSW_ADDR_BROADCAST 0x00u

/** @brief Base of the module CAN ID range; the node ID is this plus the
  *        module ID. Only meaningful when dipsw_valid(). */
#define DIPSW_CAN_ID_BASE    0x500u

typedef enum
{
  DIPSW_ROLE_CORNER = 0,   /*!< ID 1-4:  steering + drive                  */
  DIPSW_ROLE_CENTER,       /*!< ID 5-6:  drive only                        */
  DIPSW_ROLE_RESERVED,     /*!< ID 7-14: future / bench-test               */
  DIPSW_ROLE_INVALID       /*!< ID 0 (broadcast) or 15 (unconfigured):
                                must not join                              */
} dipsw_role_t;

/**
  * @brief  Configure PB12..PB15, read all four switches, and latch the result.
  * @note   Call once from main(), before any init that depends on role.
  *         Calling again is harmless but re-reads nothing: the latch stands.
  */
void dipsw_init(void);

/** @brief The latched 4-bit code, 0..15, exactly as read. */
uint8_t dipsw_code(void);

/** @brief Module ID 0..15. Same number as the code — separate call so reading
  *        sites say what they mean. Only an identity when dipsw_valid(). */
uint8_t dipsw_id(void);

/** @brief True when the latched code is a usable identity (1..14).
  *        The gate for anything that joins the bus. */
bool dipsw_valid(void);

/** @brief Role derived from the latched ID. */
dipsw_role_t dipsw_role(void);

/** @brief CAN node ID for this module. Undefined unless dipsw_valid(). */
uint16_t dipsw_can_id(void);

/** @brief Role as printable text, for the boot banner and the console. For
  *        DIPSW_ROLE_INVALID the text names the reason (broadcast address or
  *        unconfigured), taken from the latched code. */
const char *dipsw_role_str(dipsw_role_t role);

/**
  * @brief  Live re-read of the pins, without touching the latch.
  * @note   For the console only — so "did that switch actually move?" is
  *         answerable on the bench without a power cycle. Never let control
  *         code call this; identity is the latch, not the pins.
  */
uint8_t dipsw_read_live(void);

#endif /* __DIPSW_H__ */

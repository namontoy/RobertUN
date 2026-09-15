/**
  ******************************************************************************
  * @file           : dipsw.h
  * @brief          : Module identity — 3-bit DIP switch on PB13/PB14/PB15
  ******************************************************************************
  * WHY THIS EXISTS
  *
  * Six nodes run one binary. Identity is a property of the board, not of the
  * build: without this, W7 means six near-identical firmware images to keep in
  * sync, and the "which build is on this board?" failure class shows up in W8
  * disguised as a CAN problem - the most expensive place to meet it.
  *
  * Three switches give a module ID 0..7, from which both the CAN node ID and
  * the module role are derived. Nothing else in the firmware is allowed to
  * carry identity.
  *
  *   ID   Role       Behaviour
  *   0-3  Corner     steering (UART -> MKS SERVO42C) + drive (encoder PID)
  *   4-5  Center     drive only; the steering block stays inactive
  *   6    reserved   future module / bench-test mode
  *   7    INVALID    do not join the bus
  *
  * ELECTRICAL CONVENTION - do not "simplify" these away
  *
  * Internal pull-ups, switches to GND: closed = 0, open = 1. No external
  * resistors.
  *
  * 0b111 is deliberately the invalid code, because it is also what you read
  * from a board with no switch block fitted, a broken connection, or a
  * floating input. An unconfigured board therefore fails loudly instead of
  * silently impersonating module 7. That property is the whole reason the
  * invalid code is 7 and not 0.
  *
  * WHY INVALID DOES NOT HALT (yet)
  *
  * The recorded decision says an invalid ID halts and blinks. Taken literally
  * today that would brick every board on the bench: the switch block is an HW1
  * part that does not exist yet, so all three pins float high and every board
  * reads 0b111. Halting would remove the console - the only way to bring a
  * board up - to protect a bus that has not been built.
  *
  * So the halt is deferred, not dropped. What is implemented now is the part
  * that carries the safety: the ID is latched, the boot banner says loudly
  * that identity is unconfigured, and dipsw_valid() is false. When CAN lands
  * in W6, can_init() refuses on !dipsw_valid() - which is what "do not join
  * the bus" actually means. Nothing that exists today can misbehave on a bad
  * ID, because nothing that exists today reads the ID.
  *
  * WHY THE PINS ARE CONFIGURED HERE, NOT IN THE .ioc
  *
  * PB13 came from CubeMX; PB14 and PB15 are configured by dipsw_init(). Same
  * reasoning as drive_init() starting its own PWM: a regeneration has already
  * silently emptied a USER CODE block on this project once, and the .ioc is
  * the one file we cannot defend. A file we own cannot lose its own init.
  * If CubeMX is ever told about PB14/PB15, the pin macros below fall back to
  * whatever main.h defines and this file keeps working unchanged.
  *
  * LATCH ONCE
  *
  * Read in main() before anything that depends on role, and never again.
  * Identity must not change mid-run: a switch nudged with a screwdriver while
  * six nodes are live would otherwise hand two boards the same CAN ID, and the
  * symptom of that is not "wrong ID", it is arbitration chaos on the bus.
  ******************************************************************************
  */
#ifndef __DIPSW_H__
#define __DIPSW_H__

#include <stdbool.h>
#include <stdint.h>

/** @brief Raw code read from a board with nothing fitted, and the one code
  *        that is never a valid identity. */
#define DIPSW_CODE_INVALID   7u

/** @brief Base of the module CAN ID range; the node ID is this plus the
  *        module ID. Only meaningful when dipsw_valid(). */
#define DIPSW_CAN_ID_BASE    0x500u

typedef enum
{
  DIPSW_ROLE_CORNER = 0,   /*!< ID 0-3: steering + drive            */
  DIPSW_ROLE_CENTER,       /*!< ID 4-5: drive only                  */
  DIPSW_ROLE_RESERVED,     /*!< ID 6:   future / bench-test         */
  DIPSW_ROLE_INVALID       /*!< ID 7:   unconfigured, must not join */
} dipsw_role_t;

/**
  * @brief  Configure PB14/PB15, read all three switches, and latch the result.
  * @note   Call once from main(), before any init that depends on role.
  *         Calling again is harmless but re-reads nothing: the latch stands.
  */
void dipsw_init(void);

/** @brief The latched 3-bit code, 0..7, exactly as read. */
uint8_t dipsw_code(void);

/** @brief Module ID 0..6, or DIPSW_CODE_INVALID. Same number as the code —
  *        separate call so reading sites say what they mean. */
uint8_t dipsw_id(void);

/** @brief True when the latched code is a usable identity (0..6).
  *        The gate for anything that joins the bus. */
bool dipsw_valid(void);

/** @brief Role derived from the latched ID. */
dipsw_role_t dipsw_role(void);

/** @brief CAN node ID for this module. Undefined unless dipsw_valid(). */
uint16_t dipsw_can_id(void);

/** @brief Role as printable text, for the boot banner and the console. */
const char *dipsw_role_str(dipsw_role_t role);

/**
  * @brief  Live re-read of the pins, without touching the latch.
  * @note   For the console only — so "did that switch actually move?" is
  *         answerable on the bench without a power cycle. Never let control
  *         code call this; identity is the latch, not the pins.
  */
uint8_t dipsw_read_live(void);

#endif /* __DIPSW_H__ */

/**
  ******************************************************************************
  * @file           : config.h
  * @brief          : Tunable parameters, persisted in FLASH, edited from the console
  ******************************************************************************
  *
  * WHAT THIS IS FOR
  * ----------------
  * Numbers that describe THIS board rather than the design. R_IPROPI is a
  * resistor somebody soldered; VDDA is whatever the regulator actually makes;
  * the trip is a policy choice that changes between experiments. Compiled in,
  * each of those costs an edit-build-flash cycle to change, cannot differ
  * between the seven nodes without seven builds, and - worst of the three -
  * cannot be READ BACK from a running board, so what is actually in a device
  * is only knowable by finding the commit it was built from.
  *
  * So they live here instead. The `#define`s that used to hold them are still
  * in their own modules and still carry the reasoning, but they are now
  * DEFAULTS: what a blank board boots with, and what `cfg default` restores.
  * The live value comes from FLASH.
  *
  *
  * THE STORAGE IS AN APPEND-ONLY LOG, NOT A VARIABLE
  * --------------------------------------------------
  * Sector 7 (0x08060000, 128 KB, the last on the F446RE) is reserved - see
  * MEMORY in STM32F446xx_FLASH.ld, where FLASH was shortened to 384 KB to
  * carve it out. The image is ~77 KB, so nothing real was given up.
  *
  * Saving APPENDS a record to the next free slot rather than erasing and
  * rewriting one. The reason is endurance: flash is specified for 10 000 erase
  * cycles, and erase granularity here is the whole 128 KB sector. Write in
  * place and every save burns one of those 10 000. Append at a 128-byte stride
  * and the sector holds 1024 saves before it needs erasing at all, which turns
  * 10 000 saves into ~10 000 000. That matters precisely when it is most
  * annoying to be limited: during a tuning session, saving after every trial.
  *
  * On boot every slot is scanned and the one with the highest sequence number
  * AND a valid CRC wins. A partially written record - power lost mid-save -
  * fails its CRC and is skipped, so the previous good record is still live.
  * There is no window in which a reset loses the configuration.
  *
  *
  * A BAD RECORD MUST NEVER STOP THE BOARD BOOTING
  * -----------------------------------------------
  * This module sits upstream of the current limits. If it fails, it fails to
  * COMPILED DEFAULTS and says so on the boot line - loudly, every boot, so a
  * board running on defaults because its config is corrupt cannot be mistaken
  * for one that was deliberately left at defaults.
  *
  * Every value is range-checked on the way in AND on the way out of flash.
  * A record that passes its CRC can still hold a value that is out of range -
  * written by an older firmware with different limits, say - and a trip of 0
  * or a duty limit of 100% arriving from storage is not something to trust
  * because the checksum was fine. Out-of-range on load means that key reverts
  * to its default, individually, and the load still succeeds.
  *
  *
  * WHY SAVING IS REFUSED WHILE THE BRIDGE IS ENABLED
  * --------------------------------------------------
  * Erasing or programming flash STALLS INSTRUCTION FETCH on the F446 - there
  * is one flash bank and the core executes from it, so everything halts,
  * including the 1 kHz control loop and every ISR. A 128 KB sector erase is
  * typically ~1 s and specified up to 3 s.
  *
  * A motor spinning through a 1-second blackout is a motor running open-loop
  * with a frozen controller, which is exactly the hazard __HAL_DBGMCU_FREEZE_TIM4()
  * exists to prevent at a breakpoint. So `cfg save` refuses while the bridge is
  * enabled, the same way `drv zero` does. Disable, save, enable.
  *
  *
  * ADDING A KEY
  * ------------
  * Add an enum entry before CFG_KEY_COUNT and one row to the table in
  * config.c. The row carries name, units, range, default and one line of help,
  * and that row is the only user-facing documentation - `cfg` prints it.
  * Nothing else needs touching; the record grows automatically.
  *
  * Bump CONFIG_VERSION when the MEANING of an existing key changes. A version
  * mismatch discards the stored record and falls back to defaults, which is
  * the right behaviour: a number whose units changed is worse than no number.
  ******************************************************************************
  */
#ifndef CONFIG_H
#define CONFIG_H

#include <stdbool.h>
#include <stdint.h>

/** @brief Bumped when a key's meaning changes, discarding stored records.
  *        Adding or removing keys is handled by the key count, not by this. */
#define CONFIG_VERSION      1u

/**
  * @brief One tunable per entry. Order is free; it is not part of the stored
  *        format, because the record is keyed by name-independent position and
  *        guarded by CONFIG_KEY_COUNT plus a CRC.
  */
typedef enum
{
  CFG_VDDA_MV = 0,        /*!< ADC/DAC reference, mV. Measure it, do not assume */
  CFG_R_IPROPI_OHM,       /*!< IPROPI sense resistor, ohms - the one that matters */
  CFG_A_IPROPI_UA_PER_A,  /*!< 450 on DRV8874, 1000 on DRV8876 */
  CFG_TRIP_BOOT_MA,       /*!< regulation trip applied at boot, mA */
  CFG_DUTY_LIMIT,         /*!< duty magnitude cap at boot, per-mille */
  CFG_MOTOR_RAIL_MV,      /*!< measured motor terminal voltage, mV */
  CFG_ISENSE_AVG,         /*!< conversions averaged by a bare `drv current` */
  CFG_ISENSE_SAT_RAW,     /*!< raw count at or above which the ADC is clipping */
  CFG_KEY_COUNT
} config_key_t;

/** @brief Outcome of the boot-time load. Anything but CONFIG_LOAD_OK means the
  *        board is running on defaults and the user needs to know. */
typedef enum
{
  CONFIG_LOAD_OK = 0,       /*!< a valid record was found and applied        */
  CONFIG_LOAD_EMPTY,        /*!< no record stored yet - normal on a new board */
  CONFIG_LOAD_CORRUPT,      /*!< records present, none passed its CRC        */
  CONFIG_LOAD_VERSION,      /*!< stored version does not match CONFIG_VERSION */
  CONFIG_LOAD_CLAMPED       /*!< loaded, but at least one key was out of range */
} config_load_t;

/** @brief Why a save did not happen. */
typedef enum
{
  CONFIG_SAVE_OK = 0,
  CONFIG_SAVE_BUSY,         /*!< bridge enabled - refused, see the header     */
  CONFIG_SAVE_UNCHANGED,    /*!< identical to what is already stored          */
  CONFIG_SAVE_FLASH_ERROR   /*!< erase or program reported failure            */
} config_save_t;

/**
  * @brief  Load from FLASH, or fall back to compiled defaults.
  * @note   Call FIRST in main(), before any module reads a value - isense_init()
  *         in particular sets the trip from CFG_TRIP_BOOT_MA.
  * @return What happened. Anything but CONFIG_LOAD_OK must be reported.
  */
config_load_t config_init(void);

/** @brief Live value of @p key. Out-of-range keys return 0. */
int32_t config_get(config_key_t key);

/**
  * @brief  Set @p key in RAM. Takes effect immediately; not persistent until
  *         config_save().
  * @return false if @p key is unknown or @p value is outside its range, in
  *         which case nothing is changed.
  */
bool config_set(config_key_t key, int32_t value);

/** @brief Look @p name up in the table. @return CFG_KEY_COUNT if not found. */
config_key_t config_find(const char *name);

/** @brief Restore one key to its compiled default. */
void config_reset_key(config_key_t key);

/** @brief Restore every key to its compiled default. RAM only until saved. */
void config_reset_all(void);

/** @brief True if the live values differ from what is stored in FLASH. */
bool config_dirty(void);

/**
  * @brief  Append the live values to FLASH.
  * @note   Refuses while the bridge is enabled - flash operations stall the
  *         core for up to 3 s. See the header.
  */
config_save_t config_save(void);

/** @brief Reload the live values from FLASH, discarding unsaved edits. */
config_load_t config_revert(void);

/* --- metadata, for the console to print -------------------------------- */

const char *config_name(config_key_t key);
const char *config_units(config_key_t key);
const char *config_help(config_key_t key);
int32_t     config_default(config_key_t key);
int32_t     config_min(config_key_t key);
int32_t     config_max(config_key_t key);

/** @brief Slots used and slots total, for `cfg` to show wear. */
void config_usage(uint16_t *used, uint16_t *total);

/** @brief Human-readable form of a load result, for the boot line. */
const char *config_load_str(config_load_t r);

#endif /* CONFIG_H */

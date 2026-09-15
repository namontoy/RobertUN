/**
  ******************************************************************************
  * @file           : config.c
  * @brief          : Tunable parameters, persisted in FLASH
  ******************************************************************************
  * Why the storage is an append-only log, why a bad record must not stop the
  * boot, and why saving is refused while the bridge is enabled — all in
  * config.h. This file is the mechanics.
  ******************************************************************************
  */
#include "config.h"

#include "main.h"
#include "drive.h"
#include "isense.h"

#include <string.h>

/* ---------------------------------------------------------------------------
   THE KEY TABLE
   ---------------------------------------------------------------------------
   One row per tunable. The defaults deliberately point at the #defines in the
   owning modules rather than repeating the numbers, so the reasoning for each
   value stays next to the hardware it describes and there is exactly one place
   to change a default.
   --------------------------------------------------------------------------- */

typedef struct
{
  const char *name;     /*!< what the user types; keep it short and lowercase */
  const char *units;    /*!< printed after the value, "" if dimensionless     */
  int32_t     min;
  int32_t     max;
  int32_t     def;
  const char *help;     /*!< one line; this IS the user-facing documentation  */
} key_info_t;

static const key_info_t keys[CFG_KEY_COUNT] =
{
  [CFG_VDDA_MV] =
    { "vdda_mv",      "mV",   2000,  3600, (int32_t)ISENSE_VDDA_MV_DEFAULT,
      "ADC/DAC reference - measure VDDA, do not assume 3300" },

  [CFG_R_IPROPI_OHM] =
    { "r_ipropi",     "ohm",   100, 10000, (int32_t)ISENSE_R_IPROPI_OHM_DEFAULT,
      "IPROPI sense resistor as actually fitted" },

  [CFG_A_IPROPI_UA_PER_A] =
    { "a_ipropi",     "uA/A",  100,  2000, (int32_t)ISENSE_A_IPROPI_UA_PER_A_DEFAULT,
      "driver current-mirror gain - 450 DRV8874, 1000 DRV8876" },

  [CFG_TRIP_BOOT_MA] =
    { "trip_ma",      "mA",      0,  6000, (int32_t)ISENSE_TRIP_DEFAULT_MA,
      "regulation trip - applied at boot, and live when set here" },

  [CFG_DUTY_LIMIT] =
    { "duty_limit",   "o/oo",    0,  1000, (int32_t)DRIVE_DUTY_MAX,
      "duty cap, per-mille - at boot, and live when set here" },

  [CFG_MOTOR_RAIL_MV] =
    { "rail_mv",      "mV",      0, 40000, 12000,
      "measured motor terminal voltage - bookkeeping, nothing reads it yet" },

  [CFG_ISENSE_AVG] =
    { "isense_avg",   "",        1,  1024, (int32_t)ISENSE_AVG_DEFAULT,
      "conversions averaged by a bare 'drv current'" },

  [CFG_ISENSE_SAT_RAW] =
    { "sat_raw",      "",     1000,  4095, (int32_t)ISENSE_SATURATED_RAW_DEFAULT,
      "raw count at or above which the ADC itself is clipping" },
};

/* ---------------------------------------------------------------------------
   THE FLASH RECORD
   --------------------------------------------------------------------------- */

/** @brief Sector 7 — the last 128 KB, carved out of the FLASH region in
  *        STM32F446xx_FLASH.ld. Nothing else may be linked here. */
#define CONFIG_SECTOR        FLASH_SECTOR_7
#define CONFIG_BASE          0x08060000u
#define CONFIG_SIZE          0x00020000u   /* 128 KB */

/** @brief Slot pitch. Generous on purpose: the record is 48 bytes today, and
  *        a fixed stride means adding keys later does not change where any
  *        slot begins, so a firmware with more keys can still SCAN a sector
  *        written by one with fewer. 128 bytes leaves room for 28 keys. */
#define CONFIG_SLOT_BYTES    128u
#define CONFIG_SLOTS         (CONFIG_SIZE / CONFIG_SLOT_BYTES)   /* 1024 */

/** @brief "RUNC" — RobertUN config. Present means the slot has been written
  *        to, valid or not; absent (all ones) means erased and free. */
#define CONFIG_MAGIC         0x434E5552u

/** @brief What a word reads as after an erase. A slot whose magic is this has
  *        never been touched, and is where the next save goes. */
#define FLASH_ERASED_WORD    0xFFFFFFFFu

/**
  * @brief Stored form. All fields are word-aligned so the whole thing can be
  *        programmed and CRC'd a word at a time, which is the only granularity
  *        the F446 flash controller and the CRC unit agree on.
  *
  * @note  crc is LAST and is programmed last. Everything before it is covered
  *        by it, so a record interrupted by a power loss cannot pass — which is
  *        the entire reason this scheme is safe to run on a bench supply
  *        somebody might switch off mid-save.
  */
typedef struct
{
  uint32_t magic;
  uint16_t version;
  uint16_t count;                  /*!< CFG_KEY_COUNT of the writing firmware */
  uint32_t seq;                    /*!< monotonic; highest valid one wins     */
  int32_t  values[CFG_KEY_COUNT];
  uint32_t crc;
} record_t;

/* A record that does not fit its slot would silently overlap the next one. */
_Static_assert(sizeof(record_t) <= CONFIG_SLOT_BYTES,
               "config record outgrew its slot - raise CONFIG_SLOT_BYTES and "
               "accept that previously written sectors must be erased");
_Static_assert((sizeof(record_t) % 4u) == 0u,
               "config record must be a whole number of words");

/* --- live state --------------------------------------------------------- */

static int32_t  live[CFG_KEY_COUNT];
static uint32_t last_seq;        /*!< seq of the newest valid stored record  */
static uint16_t next_slot;       /*!< first free slot, == CONFIG_SLOTS if full */
static bool     have_stored;     /*!< a valid record exists in flash          */
static int32_t  stored[CFG_KEY_COUNT];  /*!< what is actually in flash, for
                                             config_dirty() and to avoid
                                             burning a slot on a no-op save */

/** @brief Address of slot @p n. */
static const record_t *slot_at(uint16_t n)
{
  return (const record_t *)(CONFIG_BASE + ((uint32_t)n * CONFIG_SLOT_BYTES));
}

/* ---------------------------------------------------------------------------
   CRC
   ---------------------------------------------------------------------------
   Driven at register level rather than through HAL_CRC_Calculate(). The CRC
   unit needs no pins, no interrupts and no configuration beyond its clock, so
   pulling it in through CubeMX would add a peripheral to the .ioc — and every
   .ioc change is a regeneration, which this project has already lost USER CODE
   blocks to once. Four lines of register poking is the cheaper risk.
   --------------------------------------------------------------------------- */

static uint32_t crc_words(const uint32_t *words, uint32_t count)
{
  __HAL_RCC_CRC_CLK_ENABLE();

  CRC->CR = CRC_CR_RESET;          /* seeds the register with 0xFFFFFFFF */

  for (uint32_t i = 0u; i < count; i++)
  {
    CRC->DR = words[i];
  }

  return CRC->DR;
}

/** @brief CRC over everything in @p r except the trailing crc field itself. */
static uint32_t record_crc(const record_t *r)
{
  return crc_words((const uint32_t *)r, (sizeof(record_t) - 4u) / 4u);
}

/* ---------------------------------------------------------------------------
   LOADING
   --------------------------------------------------------------------------- */

/**
  * @brief  Copy @p src into the live values, substituting the default for any
  *         key that falls outside its range.
  * @return true if every value was accepted as-is.
  * @note   A valid CRC only proves the bytes survived; it says nothing about
  *         whether the numbers still make sense. An older firmware may have
  *         written a trip this build considers out of bounds, and "the checksum
  *         was fine" is not a reason to hand that to the current limiter.
  */
static bool adopt(const int32_t *src)
{
  bool clean = true;

  for (uint16_t i = 0u; i < (uint16_t)CFG_KEY_COUNT; i++)
  {
    if ((src[i] >= keys[i].min) && (src[i] <= keys[i].max))
    {
      live[i] = src[i];
    }
    else
    {
      live[i] = keys[i].def;
      clean   = false;
    }
  }

  return clean;
}

static void load_defaults(void)
{
  for (uint16_t i = 0u; i < (uint16_t)CFG_KEY_COUNT; i++)
  {
    live[i] = keys[i].def;
  }
}

/**
  * @brief  Walk every slot: find the newest valid record and the first free
  *         slot, in one pass.
  * @note   The scan does not stop at the first free slot. It could, since the
  *         log is written in order — but then a single corrupted magic in the
  *         middle would hide every record after it, and a full scan of 1024
  *         slots costs microseconds from flash. Robustness is worth more here
  *         than a boot-time saving nobody can measure.
  */
static config_load_t scan(void)
{
  const record_t *best        = NULL;
  bool            saw_written = false;
  bool            saw_version = false;

  last_seq  = 0u;
  next_slot = CONFIG_SLOTS;

  for (uint16_t n = 0u; n < CONFIG_SLOTS; n++)
  {
    const record_t *r = slot_at(n);

    if (r->magic == FLASH_ERASED_WORD)
    {
      if (next_slot == CONFIG_SLOTS)
      {
        next_slot = n;
      }
      continue;
    }

    if (r->magic != CONFIG_MAGIC)
    {
      continue;                  /* garbage, but the slot is spent */
    }

    saw_written = true;

    if (r->crc != record_crc(r))
    {
      continue;                  /* torn write, or a bit gone bad */
    }

    /* A record written by a firmware with a different key count cannot be
       read positionally — key 5 there is not key 5 here. Same for a version
       bump, which by definition means a key changed meaning. */
    if ((r->version != CONFIG_VERSION) ||
        (r->count   != (uint16_t)CFG_KEY_COUNT))
    {
      saw_version = true;
      continue;
    }

    if ((best == NULL) || (r->seq > best->seq))
    {
      best = r;
    }
  }

  if (best != NULL)
  {
    last_seq = best->seq;

    memcpy(stored, best->values, sizeof(stored));
    have_stored = true;

    return adopt(best->values) ? CONFIG_LOAD_OK : CONFIG_LOAD_CLAMPED;
  }

  have_stored = false;
  load_defaults();
  memcpy(stored, live, sizeof(stored));   /* nothing stored; dirty vs defaults */

  if (saw_version)  { return CONFIG_LOAD_VERSION; }
  if (saw_written)  { return CONFIG_LOAD_CORRUPT; }

  return CONFIG_LOAD_EMPTY;
}

config_load_t config_init(void)
{
  load_defaults();               /* so a fault mid-scan still leaves it sane */
  return scan();
}

config_load_t config_revert(void)
{
  return scan();
}

/* ---------------------------------------------------------------------------
   SAVING
   --------------------------------------------------------------------------- */

static config_save_t program(const record_t *r, uint16_t at_slot)
{
  const uint32_t *w    = (const uint32_t *)r;
  uint32_t        addr = CONFIG_BASE + ((uint32_t)at_slot * CONFIG_SLOT_BYTES);
  config_save_t   res  = CONFIG_SAVE_OK;

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return CONFIG_SAVE_FLASH_ERROR;
  }

  /* Words in declaration order, so crc — the last field — lands last. A power
     loss anywhere before that leaves a record that fails its own checksum and
     is skipped on the next boot, with the previous record still live. */
  for (uint32_t i = 0u; i < (sizeof(record_t) / 4u); i++)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                          addr + (i * 4u), (uint64_t)w[i]) != HAL_OK)
    {
      res = CONFIG_SAVE_FLASH_ERROR;
      break;
    }
  }

  (void)HAL_FLASH_Lock();

  return res;
}

static config_save_t erase_sector(void)
{
  FLASH_EraseInitTypeDef e =
  {
    .TypeErase    = FLASH_TYPEERASE_SECTORS,
    .Banks        = FLASH_BANK_1,
    .Sector       = CONFIG_SECTOR,
    .NbSectors    = 1u,
    /* 32-bit programming, valid for 2.7-3.6 V. The board runs at 3.3 V and
       has no VPP pin, so this is the only correct choice. */
    .VoltageRange = FLASH_VOLTAGE_RANGE_3,
  };

  uint32_t bad = 0u;

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return CONFIG_SAVE_FLASH_ERROR;
  }

  HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&e, &bad);

  (void)HAL_FLASH_Lock();

  return (st == HAL_OK) ? CONFIG_SAVE_OK : CONFIG_SAVE_FLASH_ERROR;
}

config_save_t config_save(void)
{
  /* Refused rather than deferred. Erasing this sector stalls instruction fetch
     for up to 3 s: the 1 kHz control loop stops, the console stops, and a
     motor already turning keeps turning open-loop through all of it. The same
     rule as 'drv zero' - disable, then save. */
  if (drive_is_enabled())
  {
    return CONFIG_SAVE_BUSY;
  }

  if (have_stored && (memcmp(live, stored, sizeof(live)) == 0))
  {
    return CONFIG_SAVE_UNCHANGED;   /* a slot is cheap, but not free */
  }

  if (next_slot >= CONFIG_SLOTS)
  {
    if (erase_sector() != CONFIG_SAVE_OK)
    {
      return CONFIG_SAVE_FLASH_ERROR;
    }

    next_slot = 0u;
    /* last_seq is deliberately NOT reset. The sector is blank, so nothing
       compares against it, but keeping the counter monotonic means a partially
       erased sector - erase interrupted, old records still readable - cannot
       produce a new record that loses to a stale one. */
  }

  record_t r;

  memset(&r, 0, sizeof(r));       /* pads nothing today, but the CRC covers
                                     the whole struct, so any future padding
                                     must be deterministic */
  r.magic   = CONFIG_MAGIC;
  r.version = (uint16_t)CONFIG_VERSION;
  r.count   = (uint16_t)CFG_KEY_COUNT;
  r.seq     = last_seq + 1u;
  memcpy(r.values, live, sizeof(r.values));
  r.crc     = record_crc(&r);

  config_save_t res = program(&r, next_slot);

  if (res != CONFIG_SAVE_OK)
  {
    return res;
  }

  last_seq = r.seq;
  next_slot++;
  memcpy(stored, live, sizeof(stored));
  have_stored = true;

  return CONFIG_SAVE_OK;
}

/* ---------------------------------------------------------------------------
   ACCESS
   --------------------------------------------------------------------------- */

int32_t config_get(config_key_t key)
{
  return (key < CFG_KEY_COUNT) ? live[key] : 0;
}

bool config_set(config_key_t key, int32_t value)
{
  if (key >= CFG_KEY_COUNT)
  {
    return false;
  }

  /* Rejected, not clamped. A clamp would accept "trip 9000" and silently give
     6000, which reads back as success and hides a typo in the one place a typo
     is expensive. */
  if ((value < keys[key].min) || (value > keys[key].max))
  {
    return false;
  }

  live[key] = value;
  return true;
}

config_key_t config_find(const char *name)
{
  if (name == NULL)
  {
    return CFG_KEY_COUNT;
  }

  for (uint16_t i = 0u; i < (uint16_t)CFG_KEY_COUNT; i++)
  {
    if (strcmp(name, keys[i].name) == 0)
    {
      return (config_key_t)i;
    }
  }

  return CFG_KEY_COUNT;
}

void config_reset_key(config_key_t key)
{
  if (key < CFG_KEY_COUNT)
  {
    live[key] = keys[key].def;
  }
}

void config_reset_all(void)
{
  load_defaults();
}

bool config_dirty(void)
{
  return memcmp(live, stored, sizeof(live)) != 0;
}

const char *config_name(config_key_t key)
{
  return (key < CFG_KEY_COUNT) ? keys[key].name : "?";
}

const char *config_units(config_key_t key)
{
  return (key < CFG_KEY_COUNT) ? keys[key].units : "";
}

const char *config_help(config_key_t key)
{
  return (key < CFG_KEY_COUNT) ? keys[key].help : "";
}

int32_t config_default(config_key_t key)
{
  return (key < CFG_KEY_COUNT) ? keys[key].def : 0;
}

int32_t config_min(config_key_t key)
{
  return (key < CFG_KEY_COUNT) ? keys[key].min : 0;
}

int32_t config_max(config_key_t key)
{
  return (key < CFG_KEY_COUNT) ? keys[key].max : 0;
}

void config_usage(uint16_t *used, uint16_t *total)
{
  if (used  != NULL) { *used  = next_slot;     }
  if (total != NULL) { *total = CONFIG_SLOTS;  }
}

const char *config_load_str(config_load_t r)
{
  switch (r)
  {
    case CONFIG_LOAD_OK:      return "loaded";
    case CONFIG_LOAD_EMPTY:   return "none stored, using defaults";
    case CONFIG_LOAD_CORRUPT: return "CORRUPT, using defaults";
    case CONFIG_LOAD_VERSION: return "VERSION MISMATCH, using defaults";
    case CONFIG_LOAD_CLAMPED: return "loaded, SOME KEYS OUT OF RANGE -> default";
    default:                  return "?";
  }
}

/**
  ******************************************************************************
  * @file           : can_cmd.h
  * @brief          : W6 plain-CAN command layer (docs/can_cmds.md)
  ******************************************************************************
  *
  * Decodes the O->N command frames drained from the RX ring and answers them.
  * Everything runs in main-loop context: the ISR only fills the ring.
  *
  *   ID = (type << 4) | addr      addr 1-14 = node, 0 = broadcast
  *
  *   can_cmd_handle(frame)        one call per drained frame
  *     -> type is O->N?  addr is ours or 0?  (else ignored, no reply)
  *     -> ESTOP / STOP act on the ID alone (no DLC, counter or CRC check)
  *     -> other types: DLC -> CRC-8 -> counter -> checks -> action   (phase 2+)
  *     -> CMD_RESULT 0x080 + node
  *
  *   can_cmd_poll()               once per main-loop pass: pending FAULT frames
  *
  * A node without a valid DIP identity transmits nothing and acts only on
  * broadcast ESTOP and STOP.
  *
  * Implemented so far (W6 phases 1-4): ESTOP, STOP, ARM (actions 0-3), SPEED,
 * LIMITS, RAMP, CFG_REQ/CFG_RESP, CMD_RESULT, STATUS_DRIVE, FAULT ESTOP /
 * VEL_WD_EXPIRED / SKIPPED_CTR, and UART/CAN ownership of motion (§7.3,
 * motion.h).
  ******************************************************************************
  */

#ifndef CAN_CMD_H
#define CAN_CMD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "can_bus.h"

/** Folded into every CRC (§5.2); never transmitted. */
#define CAN_PROTO_VER  1u

/* Frame types, ID = (type << 4) | addr (§3). */
#define CAN_T_ESTOP         0x00u
#define CAN_T_STOP          0x01u
#define CAN_T_FAULT         0x02u
#define CAN_T_ARM           0x03u
#define CAN_T_SPEED         0x04u
#define CAN_T_STEER         0x05u
#define CAN_T_LIMITS        0x06u
#define CAN_T_RAMP          0x07u
#define CAN_T_CMD_RESULT    0x08u
#define CAN_T_STATUS_DRIVE  0x10u
#define CAN_T_STATUS_STEER  0x11u
#define CAN_T_HEARTBEAT     0x50u
#define CAN_T_CFG_REQ       0x52u
#define CAN_T_CFG_RESP      0x53u

/* CMD_RESULT byte 2 (§6.1). */
typedef enum
{
  CAN_RES_OK            = 0,
  CAN_RES_REPEAT        = 1,
  CAN_RES_STALE         = 2,
  CAN_RES_RANGE         = 3,
  CAN_RES_NOT_ARMED     = 4,
  CAN_RES_NOT_SUPPORTED = 5,
  CAN_RES_ESTOP_LATCHED = 6,
  CAN_RES_FAULT_LATCHED = 7,
  CAN_RES_CRC           = 8,
  CAN_RES_BAD_DLC       = 9,
  CAN_RES_UART_OWNS     = 10,
  CAN_RES_BAD_ACTION    = 11,
  CAN_RES_BUSY          = 12
} can_result_t;

/* FAULT byte 0 (§4.11). */
typedef enum
{
  CAN_FAULT_DRV_FAULT         = 1,
  CAN_FAULT_VEL_WD_EXPIRED    = 2,
  CAN_FAULT_ESTOP             = 3,
  CAN_FAULT_BUS_OFF_RECOVERED = 4,
  CAN_FAULT_ERROR_PASSIVE     = 5,
  CAN_FAULT_RX_RING_DROPPED   = 6,
  CAN_FAULT_MKS_ERROR         = 7,
  CAN_FAULT_SKIPPED_CTR       = 8
} can_fault_t;

/* CFG_REQ byte 0 (§4.8). */
#define CFG_OP_GET          0u
#define CFG_OP_SET          1u
#define CFG_OP_SAVE         2u
#define CFG_OP_REVERT       3u
#define CFG_OP_DEFAULT_KEY  4u
#define CFG_OP_DEFAULT_ALL  5u
#define CFG_OP_INFO         6u
#define CFG_OP_GET_MIN      7u
#define CFG_OP_GET_MAX      8u
#define CFG_OP_GET_DEFAULT  9u

/* CFG_RESP byte 3 (§4.9). */
typedef enum
{
  CFG_ST_OK          = 0,
  CFG_ST_UNKNOWN_KEY = 1,
  CFG_ST_RANGE       = 2,
  CFG_ST_BUSY        = 3,
  CFG_ST_FLASH_ERROR = 4,
  CFG_ST_BAD_OP      = 5,
  CFG_ST_CRC         = 6
} cfg_status_t;

/* STOP byte 1 (§4.2). */
#define CAN_STOP_MODE_MASK  0x03u
#define CAN_STOP_RAMP       0u
#define CAN_STOP_COAST      1u
#define CAN_STOP_BRAKE      2u
#define CAN_STOP_MKS        0x80u

typedef struct
{
  uint32_t handled;     /*!< frames addressed to this node and acted on/answered */
  uint32_t ignored;     /*!< other nodes' addresses, N->O types, ext/RTR       */
  uint32_t rejected;    /*!< answered with a result other than OK              */
  uint32_t tx_frames;   /*!< every frame queued (CMD_RESULT, FAULT, STATUS)    */
  uint32_t tx_dropped;  /*!< sends refused: no free mailbox                    */
  uint32_t status_tx;   /*!< STATUS_DRIVE frames queued                        */
  uint32_t speed_ok;    /*!< SPEED frames accepted                             */
  uint32_t crc_errors;  /*!< frames rejected with CRC                          */
  uint32_t ctr_repeat;  /*!< frames rejected with REPEAT                       */
  uint32_t ctr_stale;   /*!< frames rejected with STALE                        */
  uint32_t ctr_skipped; /*!< can_ctr_skipped: sum of d - 1 over accepted frames */
} can_cmd_stats_t;

/** @brief Reset the counters and pending state. Call after can_bus_init(). */
void can_cmd_init(void);

/** @brief Decode and act on one received frame. Main-loop context. */
void can_cmd_handle(const can_frame_t *f);

/** @brief STATUS_DRIVE, watchdog edge, pending FAULTs. Call every main-loop pass. */
void can_cmd_poll(void);

/** @brief STATUS_DRIVE `flags` byte (§4.12), also FAULT byte 1. */
uint8_t can_cmd_flags(void);

/**
  * @brief  CRC-8/SAE-J1850: poly 0x1D, init 0xFF, xorout 0xFF, no reflection.
  *         Check value: "123456789" -> 0x4B.
  */
uint8_t can_cmd_crc8(const uint8_t *buf, size_t len);

/**
  * @brief  The frame CRC of §5.2: ID as u16 little-endian, then CAN_PROTO_VER,
  *         then @p len payload bytes (the caller leaves the CRC byte out).
  */
uint8_t can_cmd_frame_crc(uint16_t id, const uint8_t *payload, size_t len);

const can_cmd_stats_t *can_cmd_stats(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_CMD_H */

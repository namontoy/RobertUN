/**
  ******************************************************************************
  * @file           : can_bus.c
  * @brief          : bxCAN bring-up on CAN1 (PB9 TX / PB8 RX) @ 250 kbps
  ******************************************************************************
  * Signal path, bit-timing diagram and the error-counter reading guide are in
  * can_bus.h. This file covers the implementation only.
  *
  * The module is deliberately thin. bxCAN implements all of CAN layer 2 in
  * silicon — framing, arbitration, CRC, ACK generation and fault confinement —
  * so there is no software protocol layer here to get wrong. What is left is
  * configuration that CubeMX does not emit (the acceptance filter), the one
  * call that connects the peripheral to the pins (HAL_CAN_Start), and readable
  * access to registers the debugger would otherwise be needed for.
  *
  * Everything reads or writes hardware directly and returns; nothing here
  * blocks, allocates, or keeps a queue of its own — the three TX mailboxes and
  * the 3-deep RX FIFO are the only buffering, and both live in the peripheral.
  ******************************************************************************
  */
#include "can_bus.h"

#include "main.h"

#include <string.h>

extern CAN_HandleTypeDef hcan1;

static can_bus_stats_t stats;

/* -------------------------------------------------------------------------- */
/* Init                                                                        */
/* -------------------------------------------------------------------------- */

/**
  * @brief  Accept-all mask filter on bank 0, routed to FIFO0.
  *
  * bxCAN discards every frame until at least one filter is configured AND
  * activated — the reset state is all banks disabled. Skipping this presents as
  * "TX works perfectly, RX is dead", which reliably gets misdiagnosed as a
  * wiring or termination fault.
  *
  * ID 0 with mask 0 means "compare nothing", i.e. match everything. Narrow this
  * once the CAN ID table is real; for bring-up, seeing all bus traffic is the
  * point.
  */
static bool filter_accept_all(void)
{
  CAN_FilterTypeDef filter = {0};

  filter.FilterBank           = 0u;
  filter.FilterMode           = CAN_FILTERMODE_IDMASK;
  filter.FilterScale          = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh         = 0x0000u;
  filter.FilterIdLow          = 0x0000u;
  filter.FilterMaskIdHigh     = 0x0000u;
  filter.FilterMaskIdLow      = 0x0000u;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterActivation     = CAN_FILTER_ENABLE;
  filter.SlaveStartFilterBank = 14u;   /* banks 0-13 to CAN1, 14-27 to CAN2 */

  return (HAL_CAN_ConfigFilter(&hcan1, &filter) == HAL_OK);
}

bool can_bus_init(void)
{
  memset(&stats, 0, sizeof(stats));

  if (!filter_accept_all())
  {
    return false;
  }

  /* Leaves initialization mode and connects the peripheral to the pins. */
  return (HAL_CAN_Start(&hcan1) == HAL_OK);
}

bool can_bus_set_loopback(bool enable)
{
  if (HAL_CAN_Stop(&hcan1) != HAL_OK)
  {
    return false;
  }

  hcan1.Init.Mode = enable ? CAN_MODE_LOOPBACK : CAN_MODE_NORMAL;

  /* Re-init skips MspInit when the handle is already out of RESET, so the
     pins and clocks configured by CubeMX are left alone. */
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    return false;
  }

  if (!filter_accept_all())
  {
    return false;
  }

  return (HAL_CAN_Start(&hcan1) == HAL_OK);
}

bool can_bus_is_loopback(void)
{
  return (hcan1.Init.Mode == CAN_MODE_LOOPBACK);
}

void can_bus_get_timing(uint32_t *bitrate, uint32_t *ntq, uint32_t *brp,
                        uint32_t *ts1, uint32_t *ts2, uint32_t *sjw,
                        uint32_t *sample_permille)
{
  uint32_t btr = CAN1->BTR;

  /* Every BTR field stores "value - 1". */
  uint32_t l_brp = ((btr & CAN_BTR_BRP_Msk) >> CAN_BTR_BRP_Pos) + 1u;
  uint32_t l_ts1 = ((btr & CAN_BTR_TS1_Msk) >> CAN_BTR_TS1_Pos) + 1u;
  uint32_t l_ts2 = ((btr & CAN_BTR_TS2_Msk) >> CAN_BTR_TS2_Pos) + 1u;
  uint32_t l_sjw = ((btr & CAN_BTR_SJW_Msk) >> CAN_BTR_SJW_Pos) + 1u;
  uint32_t l_ntq = 1u + l_ts1 + l_ts2;   /* the +1 is the sync segment */

  if (bitrate != NULL)
  {
    *bitrate = HAL_RCC_GetPCLK1Freq() / (l_brp * l_ntq);
  }
  if (sample_permille != NULL)
  {
    *sample_permille = (((1u + l_ts1) * 1000u) + (l_ntq / 2u)) / l_ntq;
  }
  if (ntq != NULL) { *ntq = l_ntq; }
  if (brp != NULL) { *brp = l_brp; }
  if (ts1 != NULL) { *ts1 = l_ts1; }
  if (ts2 != NULL) { *ts2 = l_ts2; }
  if (sjw != NULL) { *sjw = l_sjw; }
}

/* -------------------------------------------------------------------------- */
/* Transmit / receive                                                          */
/* -------------------------------------------------------------------------- */

bool can_bus_send(uint32_t id, const void *data, uint8_t len)
{
  CAN_TxHeaderTypeDef header = {0};
  uint8_t payload[8] = {0};
  uint32_t mailbox;

  if ((len > 8u) || ((data == NULL) && (len > 0u)))
  {
    return false;
  }

  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0u)
  {
    stats.tx_dropped++;
    return false;
  }

  /* Copy rather than cast away const — HAL takes a non-const pointer. */
  if (len > 0u)
  {
    memcpy(payload, data, len);
  }

  header.StdId              = id;
  header.ExtId              = 0u;
  header.IDE                = CAN_ID_STD;
  header.RTR                = CAN_RTR_DATA;
  header.DLC                = len;
  header.TransmitGlobalTime = DISABLE;

  if (HAL_CAN_AddTxMessage(&hcan1, &header, payload, &mailbox) != HAL_OK)
  {
    stats.tx_dropped++;
    return false;
  }

  stats.tx_frames++;
  return true;
}

/**
  * @brief  Sample and clear the FIFO0 depth flags.
  *
  * FIFO0 holds only three messages. `FULL0` says it reached that depth —
  * nothing lost yet, but the margin is gone. `FOVR0` says a message arrived
  * with no room for it, so a frame was lost.
  *
  * Both are rc_w1 and sticky, which is why `rx_overruns` counts *events* and
  * not frames: while the flag stays set, further losses raise it again but
  * cannot be distinguished. Treat a nonzero count as "frames were lost", never
  * as "this many frames were lost".
  *
  * Reading and clearing here — rather than only when a message is waiting —
  * means the flags are sampled on every drain pass of the main loop, including
  * the pass that finds the FIFO already emptied.
  */
static void sample_fifo_flags(void)
{
  uint32_t rf0r  = CAN1->RF0R;
  uint32_t sticky = rf0r & (CAN_RF0R_FULL0 | CAN_RF0R_FOVR0);

  if (sticky == 0u)
  {
    return;
  }

  /* Write-1-to-clear. Only these bits are written, so RFOM0 stays 0 and no
     mailbox is released as a side effect. */
  CAN1->RF0R = sticky;

  if ((sticky & CAN_RF0R_FULL0) != 0u)
  {
    stats.rx_fifo_full++;
  }

  if ((sticky & CAN_RF0R_FOVR0) != 0u)
  {
    stats.rx_overruns++;
  }
}

bool can_bus_receive(can_frame_t *frame)
{
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];

  if (frame == NULL)
  {
    return false;
  }

  sample_fifo_flags();

  if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) == 0u)
  {
    return false;
  }

  if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &header, data) != HAL_OK)
  {
    return false;
  }

  frame->ext = (header.IDE == CAN_ID_EXT);
  frame->id  = frame->ext ? header.ExtId : header.StdId;
  frame->rtr = (header.RTR == CAN_RTR_REMOTE);
  frame->dlc = (uint8_t)header.DLC;
  memcpy(frame->data, data, sizeof(frame->data));

  stats.rx_frames++;
  return true;
}

/* -------------------------------------------------------------------------- */
/* Error state                                                                 */
/* -------------------------------------------------------------------------- */

uint32_t can_bus_esr(void)
{
  return CAN1->ESR;
}

uint8_t can_bus_tec(void)
{
  return (uint8_t)((CAN1->ESR & CAN_ESR_TEC_Msk) >> CAN_ESR_TEC_Pos);
}

uint8_t can_bus_rec(void)
{
  return (uint8_t)((CAN1->ESR & CAN_ESR_REC_Msk) >> CAN_ESR_REC_Pos);
}

uint8_t can_bus_last_error(void)
{
  return (uint8_t)((CAN1->ESR & CAN_ESR_LEC_Msk) >> CAN_ESR_LEC_Pos);
}

/**
  * @brief  Decoded last-error code.
  *
  * "ack" is the one to know during bring-up: the frame went out correctly but
  * nothing on the bus asserted the ACK slot — a transmitter cannot acknowledge
  * itself, so this means no other node is listening, rather than anything being
  * wrong with this node.
  */
const char *can_bus_last_error_str(void)
{
  static const char *const names[8] =
  {
    "none", "stuff", "form", "ack", "bit-recessive", "bit-dominant", "crc", "sw"
  };

  return names[can_bus_last_error() & 0x07u];
}

bool can_bus_is_error_warning(void)
{
  return (CAN1->ESR & CAN_ESR_EWGF_Msk) != 0u;
}

bool can_bus_is_error_passive(void)
{
  return (CAN1->ESR & CAN_ESR_EPVF_Msk) != 0u;
}

bool can_bus_is_bus_off(void)
{
  return (CAN1->ESR & CAN_ESR_BOFF_Msk) != 0u;
}

const can_bus_stats_t *can_bus_stats(void)
{
  return &stats;
}

/* Clears the software counters only. TEC and REC are maintained by the CAN
   fault-confinement state machine in hardware and cannot be written. */
void can_bus_clear_stats(void)
{
  memset(&stats, 0, sizeof(stats));
}

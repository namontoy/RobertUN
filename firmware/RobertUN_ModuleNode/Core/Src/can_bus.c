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
  * Nothing here blocks or allocates. TX goes straight to one of the three
  * mailboxes. RX is interrupt-driven: the CAN1 RX0 ISR empties the 3-deep
  * hardware FIFO0 into a 32-slot software ring (single producer: the ISR;
  * single consumer: the main loop via can_bus_receive()). A full ring drops
  * the frame and counts it — the ISR never waits.
  ******************************************************************************
  */
#include "can_bus.h"

#include "main.h"

#include <string.h>

extern CAN_HandleTypeDef hcan1;

/* Written from the CAN1 RX0 ISR once RX is interrupt-driven (rx_*), and from
   the main loop (tx_*). One writer per field; read and cleared only through
   can_bus_stats_snapshot() / can_bus_clear_stats(), which mask that IRQ. */
static volatile can_bus_stats_t stats;

/* Mask/restore the RX0 interrupt only — never __disable_irq(), which would
   also hold off the 1 kHz control tick. Returns the previous enable state. */
static uint32_t rx_irq_mask(void)
{
  uint32_t was_enabled = NVIC_GetEnableIRQ(CAN1_RX0_IRQn);

  HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  return was_enabled;
}

static void rx_irq_restore(uint32_t was_enabled)
{
  if (was_enabled != 0u)
  {
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  }
}

static void stats_zero(void)
{
  stats = (can_bus_stats_t){0};
}

/* RX ring. Free-running indices; count = head - tail (unsigned wrap is fine
   because CAN_RX_RING_SIZE divides 2^32). head is written only by the ISR,
   tail only by the main loop. The slots are plain memory, so each side puts a
   barrier between touching a slot and publishing its index. */
#define RING_MASK  (CAN_RX_RING_SIZE - 1u)

static can_frame_t          ring[CAN_RX_RING_SIZE];
static volatile uint32_t    ring_head;
static volatile uint32_t    ring_tail;

/* Empty the ring. Call only while the RX0 interrupt is masked or not yet armed. */
static void ring_reset(void)
{
  ring_head = 0u;
  ring_tail = 0u;
}

/* Enable the "message pending" interrupt. HAL_CAN_Init() clears the enable
   bits, so this must follow every HAL_CAN_Start(). */
static bool rx_notify_arm(void)
{
  return (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK);
}

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
  stats_zero();

  if (!filter_accept_all())
  {
    return false;
  }

  ring_reset();

  /* Leaves initialization mode and connects the peripheral to the pins. */
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    return false;
  }

  return rx_notify_arm();
}

bool can_bus_set_loopback(bool enable)
{
  uint32_t irq = rx_irq_mask();
  bool ok = false;

  if (HAL_CAN_Stop(&hcan1) != HAL_OK)
  {
    goto done;
  }

  hcan1.Init.Mode = enable ? CAN_MODE_LOOPBACK : CAN_MODE_NORMAL;

  /* Re-init skips MspInit when the handle is already out of RESET, so the
     pins and clocks configured by CubeMX are left alone. */
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    goto done;
  }

  if (!filter_accept_all())
  {
    goto done;
  }

  ring_reset();   /* no stale frames cross a mode change */

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    goto done;
  }

  ok = rx_notify_arm();

done:
  rx_irq_restore(irq);
  return ok;
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

/**
  * @brief  Pop one frame from hardware FIFO0. ISR context only — this is the
  *         sole caller of sample_fifo_flags(), so the RF0R write-1-to-clear
  *         cannot race a second reader.
  */
static bool fifo_pop(can_frame_t *frame)
{
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];

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

/**
  * @brief  FIFO0 message-pending interrupt (weak override of the HAL stub;
  *         USE_HAL_CAN_REGISTER_CALLBACKS is 0).
  *
  * One bounded job: empty FIFO0 into the ring and return. FIFO0 is only 3
  * deep, so it drains to empty rather than trusting the interrupt to re-fire.
  * A full ring drops the frame and counts it; the FIFO is still emptied so the
  * hardware never backs up. Nothing here waits, prints or calls into the rest
  * of the firmware.
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  can_frame_t f;

  (void)hcan;

  while (fifo_pop(&f))
  {
    uint32_t head  = ring_head;
    uint32_t count = head - ring_tail;

    if (count >= CAN_RX_RING_SIZE)
    {
      stats.rx_ring_dropped++;
      continue;
    }

    ring[head & RING_MASK] = f;
    __DMB();                    /* slot written before head publishes it */
    ring_head = head + 1u;

    count++;
    if (count > stats.rx_ring_hwm)
    {
      stats.rx_ring_hwm = count;
    }
  }
}

bool can_bus_receive(can_frame_t *frame)
{
  uint32_t tail;

  if (frame == NULL)
  {
    return false;
  }

  tail = ring_tail;
  if (ring_head == tail)
  {
    return false;
  }

  __DMB();                      /* head seen before the slot is read */
  *frame = ring[tail & RING_MASK];
  __DMB();                      /* slot copied out before tail frees it */
  ring_tail = tail + 1u;
  return true;
}

/* -------------------------------------------------------------------------- */
/* Error state                                                                 */
/* -------------------------------------------------------------------------- */

/**
  * @brief  One read of CAN_ESR, every field decoded from that single value.
  *
  * The single-field accessors below each re-read the register, so a raw value
  * and a decoded field taken through them can disagree while TEC/REC are moving
  * (seen Aug 10: raw REC 102 printed next to REC 103). Anything that prints
  * more than one field must use this.
  */
void can_bus_errors(can_bus_err_t *out)
{
  uint32_t esr;

  if (out == NULL)
  {
    return;
  }

  esr = CAN1->ESR;

  out->esr     = esr;
  out->tec     = (uint8_t)((esr & CAN_ESR_TEC_Msk) >> CAN_ESR_TEC_Pos);
  out->rec     = (uint8_t)((esr & CAN_ESR_REC_Msk) >> CAN_ESR_REC_Pos);
  out->lec     = (uint8_t)((esr & CAN_ESR_LEC_Msk) >> CAN_ESR_LEC_Pos);
  out->warning = ((esr & CAN_ESR_EWGF_Msk) != 0u);
  out->passive = ((esr & CAN_ESR_EPVF_Msk) != 0u);
  out->bus_off = ((esr & CAN_ESR_BOFF_Msk) != 0u);
}

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
const char *can_bus_lec_str(uint8_t lec)
{
  static const char *const names[8] =
  {
    "none", "stuff", "form", "ack", "bit-recessive", "bit-dominant", "crc", "sw"
  };

  return names[lec & 0x07u];
}

const char *can_bus_last_error_str(void)
{
  return can_bus_lec_str(can_bus_last_error());
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

void can_bus_stats_snapshot(can_bus_stats_t *out)
{
  uint32_t irq;

  if (out == NULL)
  {
    return;
  }

  irq = rx_irq_mask();
  out->tx_frames       = stats.tx_frames;
  out->tx_dropped      = stats.tx_dropped;
  out->rx_frames       = stats.rx_frames;
  out->rx_fifo_full    = stats.rx_fifo_full;
  out->rx_overruns     = stats.rx_overruns;
  out->rx_ring_dropped = stats.rx_ring_dropped;
  out->rx_ring_hwm     = stats.rx_ring_hwm;
  rx_irq_restore(irq);
}

/* Clears the software counters only. TEC and REC are maintained by the CAN
   fault-confinement state machine in hardware and cannot be written. */
void can_bus_clear_stats(void)
{
  uint32_t irq = rx_irq_mask();

  stats_zero();
  rx_irq_restore(irq);
}

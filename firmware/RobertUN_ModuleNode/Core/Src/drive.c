/**
  ******************************************************************************
  * @file           : drive.c
  * @brief          : Drive-motor H-bridge control — TIM4 PWM + nSLEEP + nFAULT
  ******************************************************************************
  * The truth table, the decay-mode reasoning and the deliberate omissions are
  * in drive.h. This file is the mechanics.
  ******************************************************************************
  */
#include "drive.h"

#include "config.h"
#include "main.h"

extern TIM_HandleTypeDef htim4;

static volatile int16_t       duty;                      /*!< APPLIED per-mille, clamped */
static volatile uint16_t      limit = DRIVE_DUTY_MAX;    /*!< magnitude cap             */
static volatile drive_decay_t decay = DRIVE_DECAY_SLOW;
static volatile bool          enabled;

/* Slew limiter. `target` is written by the command path and read by the tick;
   `applied_mpm` the other way round. Single 32-bit-or-smaller objects, so the
   same no-tearing argument as the fault latch below applies and no critical
   section is needed - but see the ordering notes in drive_set_limit() and
   drive_coast(), where WHICH ONE IS WRITTEN FIRST is what makes that safe.

   applied_mpm is in MILLI-per-mille, not per-mille, because the rate that
   matters is sub-unit per tick: 5%/s is 50 per-mille/s is 0.05 per-mille in a
   1 ms tick. An integer accumulator in per-mille cannot represent that and the
   ramp would either stall at zero or run 20x too fast. Full scale is 1 000 000,
   which is nowhere near an int32. */
static volatile int16_t  target;        /*!< last COMMANDED duty, clamped     */
static volatile int32_t  applied_mpm;   /*!< what the bridge runs, x1000      */
static volatile uint16_t ramp_pmps;     /*!< per-mille per second; 0 = off    */
static volatile uint16_t ramp_floor;    /*!< per-mille; 0 = no floor          */

/* Written by drive_on_tick() in the TIM6 ISR, read by the console. Each is a
   single 32-bit-or-smaller object, so a read cannot tear on Cortex-M4 and no
   critical section is needed. They are not consistent with each other under a
   concurrent update — a reader could see the flag set with the duty not yet
   written — but the window is two instructions and the cost of being wrong is
   one misreported number in a diagnostic, not a control decision. */
static volatile bool     fault_latched;
static volatile uint32_t fault_ticks;
static volatile int16_t  fault_duty;

/* Command watchdog. Counted down by drive_on_tick() in the TIM6 ISR, written by
   the caller; the same single-object no-tearing argument as the fault latch. */
static volatile uint32_t wd_period_ms;   /*!< 0 = disabled                    */
static volatile uint32_t wd_remaining;   /*!< ms left; 0 = expired or off     */
static volatile bool     wd_expired;     /*!< sticky; cleared by arming only  */

/* Where the bridge is actually driving, in TIM4 ticks. Maintained alongside
   every CCR write so it can never describe a different duty than the one the
   timer is running - which is the failure mode that would put an ADC sample
   in the blanked phase and look like a real reading. See drive.h. */
static volatile uint16_t phase_ticks;
static volatile uint16_t phase_start;
static volatile uint16_t phase_trigger;

/* Which phase the trigger samples, and the settled region inside it that a
   spread burst may use. Written by place_trigger() alongside the three above,
   for the same reason. */
static volatile drive_sense_t sense_kind;
static volatile uint16_t      sense_first;
static volatile uint16_t      sense_last;

/** @brief Compare value for 100% output. CCR > ARR never matches, so the
  *        channel stays active for the whole period — a true 100%, not
  *        4499/4500. */
#define DRIVE_CCR_FULL  (htim4.Init.Period + 1u)

/** @brief Scale 0..1000 per-mille onto 0..CCR_FULL. */
static uint32_t ccr_from_permille(uint16_t permille)
{
  return ((uint32_t)permille * DRIVE_CCR_FULL) / (uint32_t)DRIVE_DUTY_MAX;
}

/** @brief Write both channels in one place, so no path can leave one stale. */
static void apply(uint32_t ccr1, uint32_t ccr2)
{
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ccr1);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, ccr2);
}

/**
  * @brief  Record where the drive phase is and point TIM4_CH4 at its END.
  * @param  start  first tick of the drive phase
  * @param  ticks  its width; 0 means there is no drive phase
  *
  * CH4 is in PWM mode 2, so OC4REF is low below CCR4 and high at or above it:
  * the RISING edge - the one the ADC triggers on - lands exactly on the
  * compare. PWM mode 1 would put that edge at the counter wrap instead, which
  * is the same instant for every duty and therefore useless for this.
  *
  * A CCR4 of 0 would leave the channel permanently high and never produce an
  * edge at all, so a phase with no width is parked at CCR_FULL: the compare
  * never matches, no trigger is generated, and isense falls back to its
  * free-running path rather than sampling somewhere outside the drive window.
  * Failing to sample is recoverable; sampling at the wrong phase is the bug
  * this whole mechanism exists to remove.
  *
  * THE TRIGGER IS PLACED FROM THE END OF THE WINDOW, NOT ITS MIDDLE - 2026-09-20
  * ---------------------------------------------------------------------------
  * It used to sit at the midpoint, which was wrong by 13% at 20% duty. All the
  * contamination is at the LEADING edge: IPROPI takes DRIVE_IPROPI_SETTLE_TICKS
  * to settle after the bridge turns on - 5.6 us measured, 3.5x the datasheet's
  * 1.6 us tDELAY - and the midpoint is as close to that edge as the window
  * allows. Backing off the trailing edge by exactly the sampling aperture plus a
  * guard band puts the whole aperture in settled signal and self-adjusts at
  * every duty, where any fixed fraction of the window does not.
  *
  * The floor matters as much as the placement. A window narrower than
  * DRIVE_PHASE_MIN_TICKS cannot hold settle + aperture + margin at all;
  * isense_sync_ready() refuses those outright, but `drv iscan` deliberately
  * overrides the gate, so the floor here keeps even an overridden placement from
  * landing before the signal has settled.
  *
  * DECAY PHASE (2026-09-26). In slow decay the brake phase is [0, start) and
  * IPROPI reads a fixed fraction of the motor current there (low-side mirror,
  * PMODE strapped). When @p decay_ok and the drive window is too narrow, the
  * trigger moves into the brake phase instead, by the same end-relative rule:
  * APERTURE + MARGIN before the drive edge, and never before
  * DRIVE_DECAY_SETTLE_TICKS after the falling edge at tick 0. Whether that
  * reading is USED - the minimum duty and the scale factor - is isense.c's
  * call, not this one's; this only keeps the geometry true.
  */
static void place_trigger(uint32_t start, uint32_t ticks, bool decay_ok)
{
  uint32_t back = (uint32_t)DRIVE_ADC_APERTURE_TICKS
                + (uint32_t)DRIVE_TRIGGER_MARGIN_TICKS;

  phase_ticks = (uint16_t)ticks;
  phase_start = (uint16_t)((ticks == 0u) ? 0u : start);
  sense_kind  = DRIVE_SENSE_NONE;
  sense_first = 0u;
  sense_last  = 0u;

  if (ticks == 0u)
  {
    phase_trigger = (uint16_t)DRIVE_CCR_FULL;
  }
  else if ((ticks < (uint32_t)DRIVE_PHASE_MIN_TICKS) && decay_ok &&
           (start >= (uint32_t)DRIVE_DECAY_SETTLE_TICKS + back))
  {
    /* Brake phase [0, start). The settled region runs from the settle floor
       (or one MIN_TICKS back from the end, whichever is later - the plateau
       slopes ~1% per 500 ticks and the calibration was taken near its end)
       to the end-relative trigger. */
    uint32_t last  = start - back;
    uint32_t first = (last > (uint32_t)DRIVE_DECAY_SPAN_TICKS)
                   ? (last - (uint32_t)DRIVE_DECAY_SPAN_TICKS) : 0u;

    if (first < (uint32_t)DRIVE_DECAY_SETTLE_TICKS)
    {
      first = (uint32_t)DRIVE_DECAY_SETTLE_TICKS;
    }

    sense_kind    = DRIVE_SENSE_DECAY;
    sense_first   = (uint16_t)first;
    sense_last    = (uint16_t)last;
    phase_trigger = (uint16_t)last;
  }
  else
  {
    uint32_t tick;

    /* Back off the trailing edge far enough for the aperture plus its guard. */
    tick = (ticks > back) ? (start + ticks - back) : start;

    /* ...but never sample before IPROPI has settled. These two only conflict in
       a window under DRIVE_PHASE_MIN_TICKS, which is refused for measurement -
       the floor exists so an overridden trigger still lands somewhere defensible
       rather than in the ringing. */
    if (tick < (start + (uint32_t)DRIVE_IPROPI_SETTLE_TICKS))
    {
      tick = start + (uint32_t)DRIVE_IPROPI_SETTLE_TICKS;
    }

    /* A trigger at or past CCR_FULL never fires. Clamp to the last tick that
       does, so a pathological width degrades to a late sample rather than to
       silence. */
    if (tick >= (uint32_t)DRIVE_CCR_FULL)
    {
      tick = (uint32_t)DRIVE_CCR_FULL - 1u;
    }

    phase_trigger = (uint16_t)tick;

    /* Only a window that holds settle + aperture + margin is a sense source;
       a narrower one keeps its (floored) trigger for `drv iscan` but reports
       NONE, so isense.c refuses it. */
    if (ticks >= (uint32_t)DRIVE_PHASE_MIN_TICKS)
    {
      sense_kind  = DRIVE_SENSE_DRIVE;
      sense_first = (uint16_t)(start + (uint32_t)DRIVE_IPROPI_SETTLE_TICKS);
      sense_last  = (uint16_t)tick;
    }
  }

  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, (uint32_t)phase_trigger);
}

void drive_init(void)
{
  duty    = 0;
  enabled = false;

  /* The boot cap comes from FLASH, so a board that has been told to stay under
     40% stays under 40% across a power cycle - which is the whole point of a
     cap during bring-up. Requires config_init() to have run first. */
  limit = (uint16_t)config_get(CFG_DUTY_LIMIT);
  if (limit > DRIVE_DUTY_MAX) { limit = (uint16_t)DRIVE_DUTY_MAX; }

  /* The slew limiter comes from FLASH for the same reason the cap does: a board
     told to ramp should still be ramping after a power cycle. Both default to 0
     - off - so a board that has never been told anything behaves exactly as it
     did before the limiter existed. */
  target      = 0;
  applied_mpm = 0;
  ramp_pmps   = (uint16_t)config_get(CFG_RAMP_PMPS);
  ramp_floor  = (uint16_t)config_get(CFG_RAMP_FLOOR);

  apply(0u, 0u);
  HAL_GPIO_WritePin(DRV_nSLEEP_GPIO_Port, DRV_nSLEEP_Pin, GPIO_PIN_RESET);

  /* CH4 exists only to raise a compare event for the ADC - see drive.h. It is
     configured HERE and not in the .ioc for the same reason the PWM is started
     here: a CubeMX regeneration must not be able to take it away silently, and
     this one would fail quietly - current readings would simply go back to
     being averaged across the blanked phase, which still looks like a number.
     PWM mode 2 puts the edge on the compare rather than at the wrap. */
  {
    TIM_OC_InitTypeDef oc = {0};

    oc.OCMode     = TIM_OCMODE_PWM2;
    oc.Pulse      = DRIVE_CCR_FULL;   /* never matches: armed, not yet firing */
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;

    (void)HAL_TIM_PWM_ConfigChannel(&htim4, &oc, TIM_CHANNEL_4);
  }

  place_trigger(0u, 0u, false);

  /* Start the outputs from here, not from a USER CODE block inside
     MX_TIM4_Init(). A CubeMX regeneration silently dropped that block once and
     the bridge went dead with no fault flag and no error - the timer simply was
     never running. This file is ours, so the call cannot be lost that way.
     Both channels are already at 0% and nSLEEP is low, so starting is inert. */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

  /* Enabling CC4 routes nothing to a pin: TIM4_CH4 is PB9, which is configured
     as CAN1_TX (AF9), so the alternate function never selects the timer. The
     compare event still reaches the ADC, which is all it is for. */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  /* Halt the PWM when the core halts. Without this, stopping at a breakpoint
     leaves the motor driven while the control loop is frozen - the wheel keeps
     turning and the encoder delta accumulated on resume is meaningless. */
  __HAL_DBGMCU_FREEZE_TIM4();
}

void drive_enable(void)
{
  HAL_GPIO_WritePin(DRV_nSLEEP_GPIO_Port, DRV_nSLEEP_Pin, GPIO_PIN_SET);

  /* Both the DRV8833 and the DRV8874 specify up to 1 ms from nSLEEP rising to
     the outputs being usable. Waiting 2 ms here means a duty command issued
     immediately afterwards is honoured, instead of being silently swallowed by
     a driver that has not finished starting its charge pump. */
  HAL_Delay(2u);

  enabled = true;
}

void drive_disable(void)
{
  drive_coast();
  HAL_GPIO_WritePin(DRV_nSLEEP_GPIO_Port, DRV_nSLEEP_Pin, GPIO_PIN_RESET);
  enabled = false;
}

bool drive_is_enabled(void)
{
  return enabled;
}

/** @brief Clamp to full scale and then to the configured cap, in that order. */
static int16_t clamp_duty(int32_t permille)
{
  if (permille >  DRIVE_DUTY_MAX) { permille =  DRIVE_DUTY_MAX; }
  if (permille < -DRIVE_DUTY_MAX) { permille = -DRIVE_DUTY_MAX; }

  if (permille >  (int32_t)limit) { permille =  (int32_t)limit; }
  if (permille < -(int32_t)limit) { permille = -(int32_t)limit; }

  return (int16_t)permille;
}

/**
  * @brief  Put @p permille on the bridge. Already clamped; no watchdog kick.
  *
  * Split out of drive_set_duty() when the slew limiter landed, because the two
  * halves of that function now have different callers and different rules. The
  * COMMAND half kicks the watchdog, because a command is evidence of a live
  * host. This half does not, because the tick calling it a thousand times a
  * second is evidence of nothing at all - and a limiter that kept the watchdog
  * permanently fed would quietly disable the one safety property that exists
  * for the unattended case.
  */
static void emit(int16_t permille)
{
  duty = permille;

  if (permille == 0)
  {
    /* Coast, not brake — see drive.h. Slow decay taken literally would hold
       the wheel at zero duty, which is not what "stop" should mean here. */
    apply(0u, 0u);
    place_trigger(0u, 0u, false);
    return;
  }

  uint16_t magnitude = (uint16_t)((permille < 0) ? -permille : permille);
  bool     forward   = (permille > 0);

  if (decay == DRIVE_DECAY_FAST)
  {
    /* PWM against coast: the driven pin carries the duty, the other sits low. */
    uint32_t ccr = ccr_from_permille(magnitude);
    apply(forward ? ccr : 0u,
          forward ? 0u  : ccr);

    /* Fast decay drives from the start of the period: [0, ccr). */
    place_trigger(0u, ccr, false);
  }
  else
  {
    /* PWM against brake: the driven pin sits high and the other is inverted,
       so it is high for the (1 - D) brake fraction of each period. */
    uint32_t ccr = ccr_from_permille((uint16_t)(DRIVE_DUTY_MAX - magnitude));
    apply(forward ? DRIVE_CCR_FULL : ccr,
          forward ? ccr            : DRIVE_CCR_FULL);

    /* Slow decay holds one input high and PWMs the other inverted, so the
       driven window is the TAIL of the period: [ccr, ARR]. Its width is the
       duty magnitude, which is the point - the same 12% duty puts the sample
       at tick 270 in fast decay and tick 4230 in slow. */
    place_trigger(ccr, DRIVE_CCR_FULL - ccr, true);
  }
}

void drive_set_duty(int16_t permille)
{
  /* A duty command is proof the caller is alive, so it refreshes the deadline.
     That makes the watchdog transparent to any active command stream — only a
     genuinely silent host expires. Placed before the clamps so that even a
     command clamped to zero counts as liveness. */
  drive_kick();

  permille = clamp_duty((int32_t)permille);
  target   = permille;

  if (ramp_pmps == 0u)
  {
    /* No limiter: the pre-2026-09-26 path, unchanged. */
    applied_mpm = (int32_t)permille * 1000;
    emit(permille);
    return;
  }

  /* Leaving rest gets a jump to the floor rather than a crawl through the
     sub-breakaway band - see drive_set_ramp_floor(). Conditioned on the BRIDGE
     being at zero, not on the target, so a reversal that ramps through zero
     never re-triggers it: that wheel is still turning and has back-EMF. */
  if ((applied_mpm == 0) && (permille != 0) && (ramp_floor != 0u))
  {
    int16_t jump = clamp_duty((permille > 0) ? (int32_t)ramp_floor
                                             : -(int32_t)ramp_floor);

    /* Never overshoot a command smaller than the floor. */
    if (((permille > 0) && (jump > permille)) ||
        ((permille < 0) && (jump < permille)))
    {
      jump = permille;
    }

    applied_mpm = (int32_t)jump * 1000;
    emit(jump);
    return;
  }

  /* Otherwise the tick owns the bridge from here, and deliberately nothing
     else happens in this function - it does not touch the timer at all.

     That is what keeps this lock-free. While a ramp is IN FLIGHT the ISR is the
     only writer of the CCR pair; the command path writes it only at discrete
     moments (the floor jump above, coast, brake, limit, decay), and each of
     those sets applied_mpm before it emits, so the worst an interleaved tick
     can do is emit the same value twice. That is the same two-instruction
     window this file already accepts for the watchdog's coast, and the cost of
     losing it is one CCR pair corrected within the next period. */
}

int16_t drive_duty(void)
{
  return duty;
}

int16_t drive_duty_target(void)
{
  return target;
}

bool drive_slewing(void)
{
  return applied_mpm != ((int32_t)target * 1000);
}

void drive_set_ramp(uint16_t permille_per_s)
{
  ramp_pmps = permille_per_s;

  if (permille_per_s == 0u)
  {
    /* Disarming mid-ramp adopts where the bridge IS, rather than jumping it to
       where it was heading. Jumping would make a configuration command produce
       exactly the current step this module exists to prevent - and it would do
       it at the moment the operator had just decided they no longer wanted
       ramping, which is the worst possible time to be surprised. The next
       `drv duty` steps normally from here. */
    target = duty;
  }
}

uint16_t drive_ramp(void)
{
  return ramp_pmps;
}

void drive_set_ramp_floor(uint16_t permille)
{
  ramp_floor = permille;
}

uint16_t drive_ramp_floor(void)
{
  return ramp_floor;
}

/**
  * @brief  Walk the bridge one tick toward the target. Called from the ISR.
  *
  * THE ARITHMETIC IS AN IDENTITY, NOT A COINCIDENCE. The step per tick, in
  * milli-per-mille, equals the rate in per-mille per second:
  *
  *     rate [o/oo per s] x 1000 [milli per o/oo] / 1000 [ticks per s] = rate
  *
  * so there is no division here, no rounding per tick and no accumulator residue -
  * a 50 o/oo/s ramp arrives at exactly its target, not near it.
  */
static void ramp_step(void)
{
  int32_t want;
  int32_t step;
  int16_t pm;

  if (ramp_pmps == 0u)
  {
    return;
  }

  want = (int32_t)target * 1000;

  if (applied_mpm == want)
  {
    return;
  }

  step = (int32_t)ramp_pmps;

  if (want > applied_mpm)
  {
    applied_mpm += step;
    if (applied_mpm > want) { applied_mpm = want; }
  }
  else
  {
    applied_mpm -= step;
    if (applied_mpm < want) { applied_mpm = want; }
  }

  /* C integer division truncates toward zero, which is what a signed ramp
     wants: the emitted magnitude never leads the accumulator in either
     direction. */
  pm = (int16_t)(applied_mpm / 1000);

  /* Only touch the timer when the per-mille value actually moved. At 5%/s that
     is once every 20 ms rather than a thousand CCR writes a second. */
  if (pm != duty)
  {
    emit(pm);
  }
}

void drive_brake(void)
{
  /* Zero the ramp BEFORE the bridge, so a tick landing in the middle of this
     sees applied == target and does nothing. Brake is immediate by design -
     see the slew-limiter section in drive.h. */
  target      = 0;
  applied_mpm = 0;

  duty = 0;
  apply(DRIVE_CCR_FULL, DRIVE_CCR_FULL);

  /* Braking draws nothing from VM - the current recirculates - so there is no
     drive phase to sample and no honest synchronised reading to take. */
  place_trigger(0u, 0u, false);
}

void drive_coast(void)
{
  /* Same ordering as drive_brake(), and it matters more here: this is the
     watchdog's action and the first step of drive_disable(), so it runs from
     the ISR as well as from the console. Coast is never ramped - a dead host is
     not the moment to ease off over six seconds. */
  target      = 0;
  applied_mpm = 0;

  duty = 0;
  apply(0u, 0u);
  place_trigger(0u, 0u, false);
}

bool drive_faulted(void)
{
  return HAL_GPIO_ReadPin(DRV_nFAULT_GPIO_Port, DRV_nFAULT_Pin) == GPIO_PIN_RESET;
}

void drive_on_tick(void)
{
  /* The watchdog runs FIRST because the fault path below returns early on the
     common case, and a deadline that only advances while something is wrong is
     not a deadline. */
  if ((wd_period_ms != 0u) && (wd_remaining != 0u))
  {
    if (--wd_remaining == 0u)
    {
      /* Register writes only, so this is safe from the ISR. It can in principle
         race a concurrent apply() from a console command, but the only caller
         that could be mid-command is a host that has just been declared dead,
         and the losing outcome is one torn CCR pair corrected by the next
         command. Coast rather than brake — see drive.h. */
      drive_coast();
      wd_expired = true;
    }
  }

  /* Before the fault check for the same reason the watchdog is: that check
     returns early on the common case, and a ramp that only advances while
     something is wrong is not a ramp. */
  ramp_step();

  if (!drive_faulted())
  {
    return;
  }

  fault_ticks++;

  if (!fault_latched)
  {
    /* Captured on the first tick only. A fault that persists while the duty is
       being changed should report the duty that CAUSED it, not the one it
       happened to end at. */
    fault_duty    = duty;
    fault_latched = true;
  }
}

bool drive_fault_latched(void)
{
  return fault_latched;
}

uint32_t drive_fault_ticks(void)
{
  return fault_ticks;
}

int16_t drive_fault_duty(void)
{
  return fault_duty;
}

void drive_clear_fault(void)
{
  /* Order matters if a tick lands mid-clear: drop the flag last, so the worst
     case is a zeroed counter with the flag still set - which reads as "faulted,
     count unknown" and is recoverable by clearing again. The reverse order
     could leave the flag clear and the counter non-zero, which reads as
     "no fault" while a fault is active. */
  fault_ticks   = 0u;
  fault_duty    = 0;
  fault_latched = false;
}

void drive_set_timeout(uint32_t ms)
{
  /* Order matters. Clear the latch and stop the countdown before publishing the
     new period, so a tick landing mid-update can never see a live period with a
     stale remaining count and fire immediately. */
  wd_expired   = false;
  wd_remaining = 0u;
  wd_period_ms = ms;
  wd_remaining = ms;
}

uint32_t drive_timeout(void)
{
  return wd_period_ms;
}

void drive_kick(void)
{
  if (wd_period_ms != 0u)
  {
    wd_remaining = wd_period_ms;
  }
}

uint32_t drive_timeout_remaining(void)
{
  return wd_remaining;
}

bool drive_timeout_expired(void)
{
  return wd_expired;
}

void drive_set_decay(drive_decay_t new_decay)
{
  decay = new_decay;

  /* Re-apply so the change is visible immediately. emit(), not drive_set_duty():
     the decay mode is not a command, so it must neither kick the watchdog nor
     disturb a ramp in flight - it re-draws whatever the bridge is running now
     using the new truth table. */
  emit(duty);
}

drive_decay_t drive_decay(void)
{
  return decay;
}

void drive_set_limit(uint16_t permille)
{
  if (permille > DRIVE_DUTY_MAX) { permille = (uint16_t)DRIVE_DUTY_MAX; }

  limit = permille;

  /* A tightened limit takes effect now, not on the next command — otherwise it
     would not protect against whatever is already running. Note this goes
     through emit() rather than drive_set_duty(): a cap is protection, not a
     command, so it neither kicks the watchdog nor gets RAMPED down over the
     next few hundred ms.

     Order matters. The target is clamped first, so that a tick landing between
     these two writes can only walk the bridge toward an already-capped value;
     clamping applied_mpm first would leave a window in which the ramp steps
     straight back over the new cap. */
  {
    int32_t cap = (int32_t)limit * 1000;

    target = clamp_duty((int32_t)target);

    if (applied_mpm >  cap) { applied_mpm =  cap; }
    if (applied_mpm < -cap) { applied_mpm = -cap; }

    emit((int16_t)(applied_mpm / 1000));
  }
}

uint16_t drive_limit(void)
{
  return limit;
}

uint16_t drive_phase_ticks(void)
{
  return phase_ticks;
}

uint16_t drive_phase_start(void)
{
  return phase_start;
}

uint16_t drive_phase_trigger(void)
{
  return phase_trigger;
}

drive_sense_t drive_sense_kind(void)
{
  return sense_kind;
}

uint16_t drive_sense_first(void)
{
  return sense_first;
}

uint16_t drive_sense_last(void)
{
  return sense_last;
}

void drive_trigger_override(uint16_t tick)
{
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, (uint32_t)tick);
}

void drive_trigger_restore(void)
{
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, (uint32_t)phase_trigger);
}

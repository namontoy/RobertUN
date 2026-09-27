/**
  ******************************************************************************
  * @file           : velocity.c
  * @brief          : Closed-loop wheel speed control (W5)
  ******************************************************************************
  * The reasoning lives in velocity.h. This file is the arithmetic.
  ******************************************************************************
  */
#include "velocity.h"

#include "main.h"        /* HAL_GetTick() for the published snapshot's stamp */

#include "config.h"
#include "drive.h"
#include "encoder.h"

/* Derivative low-pass, as a first-order coefficient applied once per loop
   step. 0.2 at 50 Hz is roughly a 4-sample smoother — enough to make a
   quantised, boxcar-filtered measurement differentiable at all, short enough
   not to add a pole worth worrying about next to tau_fast = 0.219 s. Fixed
   rather than configurable: Kd is 0 by default and a second knob on a term
   nobody should be using is a trap, not a feature. */
#define D_FILTER_ALPHA   0.2f

/* --- configuration, cached as floats so the tick does no conversion ------
   NOT volatile, and that is the deliberate half of the split below: thread
   context writes these and the tick only ever reads them, each is a single
   word, and a gain that lands one step late is of no consequence. */

static float    kp        = 0.0f;   /* o/oo per rpm                          */
static float    ki        = 0.0f;   /* o/oo per rpm-second                   */
static float    kd        = 0.0f;   /* o/oo-second per rpm                   */
static float    ff_slope  = 0.0f;   /* o/oo per rpm                          */
static int32_t  ff_offset = 0;      /* o/oo, applied with the setpoint's sign */
static float    slew_rpm_s = 0.0f;  /* setpoint ramp; 0 = step the setpoint  */

static int32_t  kp_milli = 0, ki_milli = 0, kd_milli = 0, ff_slope_milli = 0;
static int32_t  slew_milli = 0;
static uint16_t i_limit  = 0;       /* o/oo                                  */
static uint16_t out_max  = 0;       /* o/oo                                  */

/* --- loop state, written by the tick -------------------------------------
   ALL volatile, matching encoder.c and drive.c. The TIM6 ISR writes these and
   thread context - every accessor below, and print_velocity_line() through
   velocity_take_sample() - reads them. Without it the compiler is entitled to
   keep `armed` or `out_pm` in a register across a console call and report a
   value the loop abandoned several steps ago. */

static volatile bool     armed      = false;
static volatile velocity_state_t state = VELOCITY_OFF;

static volatile int32_t  target_mrpm = 0; /* written from thread context      */
static volatile float    sp_rpm      = 0.0f; /* ramped setpoint the PID chases */
static volatile float    meas_rpm    = 0.0f;
static volatile float    integ       = 0.0f; /* o/oo                          */
static volatile float    d_state     = 0.0f; /* filtered d(meas)/dt, rpm/s    */
static volatile float    prev_meas   = 0.0f;
static volatile bool     have_prev   = false;

static volatile int16_t  out_pm  = 0;
static volatile int16_t  t_ff = 0, t_p = 0, t_i = 0, t_d = 0;
static volatile bool     saturated = false;
static volatile bool     coasting  = false;

static volatile uint32_t last_seq = 0u;
static volatile uint32_t period_us = 20000u;

/* --- setpoint watchdog --------------------------------------------------- */

static volatile uint32_t wd_period_ms = 0u;
static volatile uint32_t wd_remaining = 0u;
static volatile bool     wd_expired   = false;  /* sticky; only velocity_enable()
                                                   clears it */

/* --- the published snapshot ----------------------------------------------
   One slot, written at the end of each control step, drained by the main loop
   through velocity_take_sample(). See velocity.h for why this is published per
   STEP rather than sampled on the telemetry schedule. */

static volatile velocity_sample_t pub;
static volatile uint32_t pub_step   = 0u;  /* step index of what is in `pub`  */
static volatile uint32_t pub_taken  = 0u;  /* step index the reader last took */
static volatile bool     pub_missed = false; /* sticky until the next line    */
static volatile uint32_t step_index = 0u;

/* ------------------------------------------------------------------------ */

/** @brief Enter a critical section that nests correctly — restoring the saved
  *        PRIMASK rather than unconditionally re-enabling interrupts, so
  *        calling from an ISR cannot silently turn them back on.
  *
  * Lifted verbatim from encoder.c rather than hoisted into a shared header:
  * two copies of six lines is cheaper to read than one more include, and
  * neither module should have to care that the other exists. */
static inline uint32_t lock(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static inline void unlock(uint32_t primask)
{
  __set_PRIMASK(primask);
}


static float clampf(float v, float lo, float hi)
{
  if (v < lo) { return lo; }
  if (v > hi) { return hi; }
  return v;
}

/** The real output ceiling: this loop's own limit, or drive.c's cap, whichever
    bites first. Taken here rather than left to drive_set_duty() so the
    anti-windup logic knows the actual boundary instead of integrating against
    a clamp it cannot see. */
static float ceiling_pm(void)
{
  uint16_t cap = drive_limit();
  uint16_t lim = out_max;

  /* Unconditionally, including zero: drive_set_limit(0) means "no output",
     not "no cap". Treating 0 as absent would have this loop cheerfully
     command duty through a bridge somebody had just pinned shut. */
  if (cap < lim) { lim = cap; }
  return (float)lim;
}

/**
  * @brief Publish this control step's snapshot. Called from the ISR only.
  *
  * Every field is written through the volatile struct, so the compiler may not
  * reorder them past each other, and `pub_step` is written LAST — a reader that
  * sees a new step index is guaranteed to be looking at a complete record. The
  * reader takes its copy with interrupts off, which is the other half.
  *
  * No lock is taken here. Nothing at this priority can preempt the ISR, and
  * the reader's critical section already excludes it.
  */
static void publish(uint8_t flags)
{
  step_index++;

  /* Overwriting a snapshot nobody took is data loss, and the host has to hear
     about it or it will read a decimated stream as a slow control loop. The
     flag is sticky so it survives to whichever record IS taken next. */
  if (pub_step != pub_taken)
  {
    pub_missed = true;
  }

  if (pub_missed)
  {
    flags |= VELOCITY_SAMPLE_MISSED;
  }

  pub.ms        = HAL_GetTick();
  pub.sp_mrpm   = (int32_t)(sp_rpm   * 1000.0f);
  pub.meas_mrpm = (int32_t)(meas_rpm * 1000.0f);
  pub.out       = out_pm;
  pub.ff        = t_ff;
  pub.p         = t_p;
  pub.i         = t_i;
  pub.d         = t_d;
  pub.flags     = flags;
  pub.step      = step_index;

  pub_step = step_index;      /* LAST — this is what publishes the record */
}

bool velocity_take_sample(velocity_sample_t *dst)
{
  uint32_t primask;
  bool     got = false;

  if (dst == NULL)
  {
    return false;
  }

  primask = lock();

  if (pub_step != pub_taken)
  {
    /* Cast away volatile for the copy: interrupts are off, so nothing can
       change underneath it and the qualifier has no work left to do. Doing it
       as one struct assignment rather than ten field reads keeps the critical
       section to about a dozen instructions. */
    *dst = *(const velocity_sample_t *)&pub;

    pub_taken  = pub_step;
    pub_missed = false;       /* reported on the record now being handed out */
    got        = true;
  }

  unlock(primask);
  return got;
}

static void clear_loop_state(void)
{
  sp_rpm    = 0.0f;
  integ     = 0.0f;
  d_state   = 0.0f;
  prev_meas = 0.0f;
  have_prev = false;
  out_pm    = 0;
  t_ff = t_p = t_i = t_d = 0;
  saturated = false;
}

void velocity_init(void)
{
  velocity_set_kp(config_get(CFG_VEL_KP));
  velocity_set_ki(config_get(CFG_VEL_KI));
  velocity_set_kd(config_get(CFG_VEL_KD));
  velocity_set_ff(config_get(CFG_VEL_FF_SLOPE), config_get(CFG_VEL_FF_OFFSET));
  velocity_set_i_limit((uint16_t)config_get(CFG_VEL_I_LIMIT));
  velocity_set_max((uint16_t)config_get(CFG_VEL_MAX));
  velocity_set_slew(config_get(CFG_VEL_SLEW));
  velocity_set_timeout((uint32_t)config_get(CFG_VEL_TIMEOUT));

  armed       = false;
  state       = VELOCITY_OFF;
  target_mrpm = 0;
  coasting    = false;
  last_seq    = encoder_velocity_seq();
  clear_loop_state();
}

void velocity_on_tick(void)
{
  uint32_t seq;
  float dt, err, unsat, out, d_raw, i_cand, ff_term;
  float lim;
  bool  bridge_ok, push_into_sat, freeze;

  if (!armed)
  {
    return;
  }

  /* The setpoint watchdog runs on the TICK, not on the loop step, and before
     anything else — the same placement drive.c uses for its own. A deadline
     that only advances when the rest of the system is healthy is not a
     deadline. */
  if (wd_period_ms != 0u && wd_remaining != 0u)
  {
    wd_remaining--;
    if (wd_remaining == 0u)
    {
      /* IMMEDIATE, exactly as drive.c's is, and for the same reason: this
         fires only when whatever was commanding the wheel has already stopped
         talking. Walking the setpoint down over the next second would keep the
         motor driven for a second after the last thing watching it died. The
         ramp is for commanded motion; this is not commanded motion.

         sp_rpm is zeroed here too, so the stopped branch below is entered on
         this very step rather than one window later. */
      wd_expired  = true;
      target_mrpm = 0;
      sp_rpm      = 0.0f;
      state       = VELOCITY_TIMEOUT;

      clear_loop_state();
      coasting = true;
      drive_coast();

      /* Published even though this fired on a TICK rather than on a control
         step: the exact millisecond the watchdog acted is the single most
         valuable row in the stream when something has gone wrong, and losing
         it to "that was not a real step" would be pedantry. meas_mrpm here is
         the previous step's — nothing has re-read the encoder this tick. */
      publish(VELOCITY_SAMPLE_WD_EXPIRED);
      return;
    }
  }

  /* Step only when the measurement has actually been refreshed. See the
     header: at 1 kHz against a 50 Hz window the integrator would run 20x
     fast and the derivative would be nineteen zeroes and a spike. */
  seq = encoder_velocity_seq();
  if (seq == last_seq)
  {
    return;
  }
  last_seq = seq;

  {
    uint16_t win = encoder_velocity_window();
    if (win == 0u) { win = 1u; }
    period_us = (uint32_t)win * 1000u;
    dt = (float)win * 0.001f;
  }

  /* --- setpoint ramp, in rpm/s ------------------------------------------ */
  {
    float tgt = (float)target_mrpm * 0.001f;

    if (slew_rpm_s <= 0.0f)
    {
      sp_rpm = tgt;
    }
    else
    {
      float step = slew_rpm_s * dt;
      if (sp_rpm < tgt)       { sp_rpm += step; if (sp_rpm > tgt) { sp_rpm = tgt; } }
      else if (sp_rpm > tgt)  { sp_rpm -= step; if (sp_rpm < tgt) { sp_rpm = tgt; } }
    }
  }

  meas_rpm = encoder_rpm();

  /* --- stopped: coast, do not hold zero with torque --------------------- */
  if (target_mrpm == 0 && sp_rpm == 0.0f)
  {
    state = wd_expired ? VELOCITY_TIMEOUT : VELOCITY_HOLDING;

    /* Rule 4 in the header: an integrator that keeps climbing against
       stiction while the wheel sits still turns the next departure into a
       lurch. Clear it, and coast ONCE rather than re-issuing coast at 50 Hz. */
    clear_loop_state();
    if (!coasting)
    {
      coasting = true;
      drive_coast();
    }

    /* Still a control step, and still worth a row: this is where a coastdown
       is recorded, and where a host watching for the wheel to reach standstill
       gets its answer. */
    publish(wd_expired ? VELOCITY_SAMPLE_WD_EXPIRED : 0u);
    return;
  }

  coasting = false;
  state    = VELOCITY_RUNNING;

  /* --- the controller --------------------------------------------------- */

  err = sp_rpm - meas_rpm;

  /* Feedforward from the measured inverse plant. The offset is friction, so it
     takes the sign of where we are trying to go, not of the error. */
  ff_term = ff_slope * sp_rpm;
  if (sp_rpm > 0.0f)      { ff_term += (float)ff_offset; }
  else if (sp_rpm < 0.0f) { ff_term -= (float)ff_offset; }

  /* Derivative on the MEASUREMENT, low-passed. On the error it would spike on
     every setpoint change; on a quantised boxcar measurement it needs the
     filter to mean anything at all. Negated because d(error)/dt = -d(meas)/dt
     for a setpoint the ramp is already smoothing. */
  if (have_prev)
  {
    d_raw   = (meas_rpm - prev_meas) / dt;
    d_state += (d_raw - d_state) * D_FILTER_ALPHA;
  }
  else
  {
    d_state   = 0.0f;
    have_prev = true;
  }
  prev_meas = meas_rpm;

  i_cand = integ + (ki * err * dt);
  i_cand = clampf(i_cand, -(float)i_limit, (float)i_limit);

  unsat = ff_term + (kp * err) + i_cand - (kd * d_state);

  lim = ceiling_pm();
  out = clampf(unsat, -lim, lim);
  saturated = (out != unsat);

  /* --- anti-windup: the four cases the header enumerates ----------------- */

  bridge_ok = drive_is_enabled() && !drive_fault_latched();

  /* Integrating further in the direction we are already clamped in adds
     nothing to the output and everything to the recovery time. Integrating the
     other way is allowed, so unwinding is immediate. */
  push_into_sat = saturated && ((unsat > out && err > 0.0f) ||
                                (unsat < out && err < 0.0f));

  /* drive_slewing() means the bridge has not reached the last thing we asked
     for. The error is our own actuator's lag; integrating it is integrating
     against ourselves. */
  freeze = push_into_sat || drive_slewing() || !bridge_ok;

  if (!freeze)
  {
    integ = i_cand;
  }

  /* --- write --------------------------------------------------------------
     Only when the bridge can actually act. Writing while it is asleep would
     kick drive.c's command watchdog to no purpose and let the integrator's
     work escape through a disabled driver on the next enable. */
  t_ff = (int16_t)ff_term;
  t_p  = (int16_t)(kp * err);
  t_i  = (int16_t)integ;
  t_d  = (int16_t)(-(kd * d_state));

  if (bridge_ok)
  {
    out_pm = (int16_t)out;
    drive_set_duty(out_pm);
  }
  else
  {
    out_pm = 0;
  }

  {
    uint8_t f = 0u;

    if (saturated)       { f |= VELOCITY_SAMPLE_SATURATED; }
    if (freeze)          { f |= VELOCITY_SAMPLE_FROZEN;    }
    if (drive_slewing()) { f |= VELOCITY_SAMPLE_SLEWING;   }
    if (!bridge_ok)      { f |= VELOCITY_SAMPLE_NOBRIDGE;  }
    if (wd_expired)      { f |= VELOCITY_SAMPLE_WD_EXPIRED; }

    /* RAMPING is the ramped setpoint disagreeing with the commanded one. A
       host reading a step response needs this to know where the command ends
       and the loop's own response begins - without it, the setpoint ramp's
       rate is indistinguishable from a sluggish controller. */
    if ((int32_t)(sp_rpm * 1000.0f) != target_mrpm)
    {
      f |= VELOCITY_SAMPLE_RAMPING;
    }

    publish(f);
  }
}

/* --- arming -------------------------------------------------------------- */

void velocity_enable(void)
{
  target_mrpm = 0;
  clear_loop_state();
  last_seq  = encoder_velocity_seq();
  coasting  = false;
  armed     = true;
  state     = VELOCITY_HOLDING;

  /* The step index restarts with the loop, so a host can tell one arming from
     the next the same way telem's seq marks one stream from the next. Both
     sides of the published slot are cleared too - a snapshot from the previous
     arming is not a step of this one. */
  step_index = 0u;
  pub_step   = 0u;
  pub_taken  = 0u;
  pub_missed = false;

  /* Arm the setpoint watchdog with the loop, not with the first setpoint.
     Enabling is the moment drive.c's watchdog stops being able to fire. */
  wd_remaining = wd_period_ms;
  wd_expired   = false;
}

void velocity_disable(void)
{
  armed    = false;
  state    = VELOCITY_OFF;
  target_mrpm = 0;
  clear_loop_state();
  if (!coasting)
  {
    coasting = true;
    drive_coast();
  }
}

bool velocity_enabled(void)          { return armed; }
velocity_state_t velocity_state(void){ return state; }

/* --- setpoint ------------------------------------------------------------ */

void velocity_set_setpoint(int32_t milli_rpm)
{
  target_mrpm = milli_rpm;

  /* A setpoint is evidence that something upstream is still choosing. That is
     what this watchdog watches, and it is the only thing left watching once
     the loop is keeping drive.c's alive.

     The kick refreshes the countdown but NOT the expired flag - drive.c's
     watchdog behaves the same way. "It stopped talking, then started again"
     is a fact worth surviving until somebody re-arms the loop deliberately;
     a flag that any recovery erases records nothing. */
  wd_remaining = wd_period_ms;
}

int32_t velocity_setpoint(void)        { return target_mrpm; }
int32_t velocity_ramped_setpoint(void) { return (int32_t)(sp_rpm * 1000.0f); }
int32_t velocity_measured(void)        { return (int32_t)(meas_rpm * 1000.0f); }
int32_t velocity_error(void)           { return (int32_t)((sp_rpm - meas_rpm) * 1000.0f); }
int16_t velocity_output(void)          { return out_pm; }
bool    velocity_saturated(void)       { return saturated; }

void velocity_terms(int16_t *ff, int16_t *p, int16_t *i, int16_t *d)
{
  if (ff) { *ff = t_ff; }
  if (p)  { *p  = t_p;  }
  if (i)  { *i  = t_i;  }
  if (d)  { *d  = t_d;  }
}

void velocity_reset(void)
{
  integ     = 0.0f;
  d_state   = 0.0f;
  have_prev = false;
  saturated = false;
}

/* --- setpoint watchdog --------------------------------------------------- */

void velocity_set_timeout(uint32_t ms)
{
  wd_period_ms = ms;
  wd_remaining = ms;         /* a deadline just changed has not been missed
                                yet - but the LATCH stays; only
                                velocity_enable() clears it */
}

uint32_t velocity_timeout(void)           { return wd_period_ms; }
uint32_t velocity_timeout_remaining(void) { return wd_remaining; }
bool     velocity_timeout_expired(void)   { return wd_expired; }

/* --- gains and limits ---------------------------------------------------- */

void velocity_set_kp(int32_t milli) { kp_milli = milli; kp = (float)milli * 0.001f; }
int32_t velocity_kp(void)           { return kp_milli; }
void velocity_set_ki(int32_t milli) { ki_milli = milli; ki = (float)milli * 0.001f; }
int32_t velocity_ki(void)           { return ki_milli; }
void velocity_set_kd(int32_t milli) { kd_milli = milli; kd = (float)milli * 0.001f; }
int32_t velocity_kd(void)           { return kd_milli; }

void velocity_set_ff(int32_t slope_milli, int32_t offset_permille)
{
  ff_slope_milli = slope_milli;
  ff_slope       = (float)slope_milli * 0.001f;
  ff_offset      = offset_permille;
}

int32_t velocity_ff_slope(void)  { return ff_slope_milli; }
int32_t velocity_ff_offset(void) { return ff_offset; }

void velocity_set_i_limit(uint16_t permille) { i_limit = permille; }
uint16_t velocity_i_limit(void)              { return i_limit; }
void velocity_set_max(uint16_t permille)     { out_max = permille; }
uint16_t velocity_max(void)                  { return out_max; }

void velocity_set_slew(int32_t milli_rpm_per_s)
{
  slew_milli = milli_rpm_per_s;
  slew_rpm_s = (float)milli_rpm_per_s * 0.001f;
}

int32_t velocity_slew(void) { return slew_milli; }

uint32_t velocity_period_us(void) { return period_us; }

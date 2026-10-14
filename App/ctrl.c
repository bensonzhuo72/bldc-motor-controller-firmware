/**
 * @file    ctrl.c
 * @brief   Control step in the 20 kHz interrupt: overcurrent trip, output.
 */
#include "ctrl.h"
#include "app_config.h"
#include "motor_params.h"
#include "pwm.h"
#include "debug.h"
#include "main.h"
#if BRINGUP_STAGE >= 8
#include "foc_math.h"
#endif
#if BRINGUP_STAGE >= 11
#include "mt6701.h"
#endif

/* ---- State (shared with the control interrupt) ----------------------------- */

static volatile float    s_trip_a = MOTOR_OC_TRIP_DEFAULT_A;
static volatile bool     s_oc_event;
static volatile uint32_t s_oc_phase;
static volatile float    s_oc_amps;

static volatile float    s_v;          /* Stage 8 vector: amplitude, V */
static volatile float    s_theta;      /* electrical angle, rad */
static volatile float    s_shift;      /* common-mode duty shift */
static volatile bool     s_step_pending;
static volatile float    s_step_v;

static volatile float    s_vph[PHASE_COUNT];
static volatile float    s_dmax;

/* Stage 9 forced angle: target and slew set by the main loop; speed, angle
 * and whole electrical cycles kept by the interrupt. */
#define CTRL_DT_S          50e-6f            /* control period, 20 kHz (plan: TIM1) */
#define CTRL_TWO_PI        6.28318530718f
static volatile float    s_f_target;       /* Hz electrical, signed */
static volatile float    s_accel;          /* Hz/s */
static volatile float    s_f_now;
static volatile float    s_theta_f;        /* rad, [0, 2 pi) */
static volatile int32_t  s_ecycles;
static volatile bool     s_hold;           /* Stage 10: angle from the main loop, not the generator */

/* Stage 11 voltage-mode FOC (plan Stage 11): Vd = 0, Vq from the main loop,
 * angle from the encoder. Electrical angle = DIR x 11 x theta_m - offset
 * (plan: Phase order, angles and direction), computed in counts: 16384 is a
 * power of two, so the wrap is a mask. Latency correction: the angle is
 * sampled at the previous interrupt (50 us), the MT6701 output lags the
 * magnet by up to 5 us (datasheet: propagation delay), and the new duties
 * act from the next PWM period, centred about 50 us later: about 100 us,
 * advanced by the PLL speed. At 2 V no-load (about 210 rpm) that is 1.4 deg
 * electrical. */
static volatile bool     s_foc;
static volatile float    s_vq;
static volatile bool     s_enc_event;
#define CTRL_ENC_COUNTS      16384U
#define CTRL_ENC_MASK        (CTRL_ENC_COUNTS - 1U)
#define CTRL_RAD_PER_ECOUNT  (6.28318530718f / (float)CTRL_ENC_COUNTS)
#define CTRL_ENC_LATENCY_S   100e-6f

/* ---- Control interrupt ---------------------------------------------------- */

APP_RAMFUNC static void ctrl_step(const float amps[PHASE_COUNT])
{
  if (!pwm_is_armed())
  {
    return;
  }

#if BRINGUP_STAGE >= 8
  /* Software overcurrent trip (plan: Firmware protection, Stage 8): any
   * phase beyond the level disarms within this 50 us cycle. The fault is
   * raised by the main loop; the capture keeps the lead-up. */
  float trip = s_trip_a;
  for (uint32_t ph = 0U; ph < PHASE_COUNT; ph++)
  {
    float a = amps[ph];
    if ((a > trip) || (a < -trip))
    {
      motor_disarm();
      debug_capture_trigger();
      s_oc_phase = ph;
      s_oc_amps = a;
      s_oc_event = true;
      return;
    }
  }

  if (s_step_pending)
  {
    s_v = s_step_v;
    s_step_pending = false;
    debug_capture_trigger();   /* this sample is the capture's trigger point */
  }

  float theta;
  float vd = s_v;      /* Stages 8-10: the vector lies on the d axis */
  float vq = 0.0f;
#if BRINGUP_STAGE >= 11
  if (s_foc)
  {
    uint16_t counts;
    float speed;
    if (!mt6701_isr_angle(&counts, &speed))
    {
      /* Bad or unhealthy encoder sample: no valid angle to commutate on. */
      motor_disarm();
      s_enc_event = true;
      return;
    }
    uint32_t e = ((uint32_t)counts * MOTOR_POLE_PAIRS) & CTRL_ENC_MASK;
    if (MOTOR_ENC_DIR < 0)
    {
      e = (CTRL_ENC_COUNTS - e) & CTRL_ENC_MASK;
    }
    float w_e = (float)MOTOR_ENC_DIR * (float)MOTOR_POLE_PAIRS * speed;
    theta = ((float)e * CTRL_RAD_PER_ECOUNT) - MOTOR_ENC_OFFSET_RAD + (w_e * CTRL_ENC_LATENCY_S);
    vd = 0.0f;
    vq = s_vq;
  }
  else
#endif
  {
#if BRINGUP_STAGE >= 9
    if (s_hold)
    {
      theta = s_theta;   /* Stage 10 alignment: angle stepped by the main loop */
    }
    else
    {
      /* Forced angle (plan Stage 9): the speed slews toward the target at the
       * set acceleration and the angle integrates it. */
      float df = s_f_target - s_f_now;
      float dmax_f = s_accel * CTRL_DT_S;
      if (df > dmax_f)
      {
        df = dmax_f;
      }
      else if (df < -dmax_f)
      {
        df = -dmax_f;
      }
      s_f_now += df;
      float th = s_theta_f + (CTRL_TWO_PI * s_f_now * CTRL_DT_S);
      if (th >= CTRL_TWO_PI)
      {
        th -= CTRL_TWO_PI;
        s_ecycles++;
      }
      else if (th < 0.0f)
      {
        th += CTRL_TWO_PI;
        s_ecycles--;
      }
      s_theta_f = th;
      theta = th;
    }
#else
    theta = s_theta;
#endif
  }

  /* Inverse Park (Vd, Vq at theta), then SVPWM. */
  float sn;
  float cs;
  float va;
  float vb;
  float vph[PHASE_COUNT];
  float d[PHASE_COUNT];
  float sh = s_shift;
  foc_sincos(theta, &sn, &cs);
  foc_inv_park(vd, vq, sn, cs, &va, &vb);
  foc_svpwm(va, vb, MOTOR_VBUS_V, vph, d);

  float dmax = 0.0f;
  for (uint32_t ph = 0U; ph < PHASE_COUNT; ph++)
  {
    d[ph] += sh;
    s_vph[ph] = vph[ph];
    if (d[ph] > dmax)
    {
      dmax = d[ph];
    }
  }
  s_dmax = (dmax > PWM_DUTY_MAX) ? PWM_DUTY_MAX : dmax;   /* pwm_set_duty clamps */
  pwm_set_duty(d[PHASE_A], d[PHASE_B], d[PHASE_C]);
#else
  (void)amps;
  pwm_isr_update();
#endif
}

/* Called by the control interrupt. From Stage 11 the encoder is read here:
 * finish the read started last time (so the angle is fresh for this step),
 * run the step, then start the next read whatever the step did (armed or
 * not, trip or not), so the encoder keeps updating while disarmed. */
APP_RAMFUNC void ctrl_isr(const float amps[PHASE_COUNT])
{
#if BRINGUP_STAGE >= 11
  mt6701_isr_collect();
#endif
  ctrl_step(amps);
#if BRINGUP_STAGE >= 11
  mt6701_isr_start();
#endif
}

/* ---- Main loop ------------------------------------------------------------ */

float ctrl_set_trip(float amps)
{
  float t = amps;
  if (!(t > 0.0f))   /* also catches NaN */
  {
    t = MOTOR_OC_TRIP_DEFAULT_A;
  }
  if (t < MOTOR_OC_TRIP_MIN_A)
  {
    t = MOTOR_OC_TRIP_MIN_A;
  }
  if (t > MOTOR_OC_TRIP_STAGE_MAX_A)
  {
    t = MOTOR_OC_TRIP_STAGE_MAX_A;
  }
  s_trip_a = t;
  return t;
}

bool ctrl_take_oc_event(uint32_t *phase, float *amps)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  bool ev = s_oc_event;
  s_oc_event = false;
  *phase = s_oc_phase;
  *amps = s_oc_amps;
  __set_PRIMASK(primask);
  return ev;
}

void ctrl_set_vector(float v, float theta, float shift)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  s_v = v;
  s_theta = theta;
  s_shift = shift;
  __set_PRIMASK(primask);
}

void ctrl_vector_step(float v_step)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  s_step_v = v_step;
  s_step_pending = true;
  __set_PRIMASK(primask);
}

void ctrl_spin_reset(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  s_f_target = 0.0f;
  s_f_now = 0.0f;
  s_theta_f = 0.0f;
  s_ecycles = 0;
  __set_PRIMASK(primask);
}

void ctrl_set_spin(float f_target_hz, float accel_hz_s)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  s_f_target = f_target_hz;
  s_accel = accel_hz_s;
  __set_PRIMASK(primask);
}

void ctrl_hold_angle(bool hold)
{
  s_hold = hold;   /* single word: no lock needed */
}

void ctrl_set_foc(bool on)
{
  s_foc = on;
}

void ctrl_set_vq(float vq)
{
  s_vq = vq;       /* single word */
}

bool ctrl_take_enc_event(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  bool ev = s_enc_event;
  s_enc_event = false;
  __set_PRIMASK(primask);
  return ev;
}

void ctrl_spin_state(float *f_now_hz, float *cycles)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  *f_now_hz = s_f_now;
  *cycles = (float)s_ecycles + (s_theta_f / CTRL_TWO_PI);
  __set_PRIMASK(primask);
}

void ctrl_last_output(float vph[PHASE_COUNT], float *duty_max)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  for (uint32_t ph = 0U; ph < PHASE_COUNT; ph++)
  {
    vph[ph] = s_vph[ph];
  }
  *duty_max = s_dmax;
  __set_PRIMASK(primask);
}

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

/* Encoder electrical angle for this interrupt, from the read collected at
 * its start (Stage 11+): as sampled, without latency correction, and the
 * electrical speed. Interrupt only. */
static float s_th_enc;
static float s_we;
static bool  s_th_ok;

/* Stage 12 rotating-frame currents (plan Stage 12, shadow mode: measured,
 * not controlled). The phase currents are sampled in this interrupt; the
 * encoder sample is one period older (50 us) plus the MT6701 propagation
 * delay (up to 5 us, datasheet), so its angle is advanced by 55 us of
 * speed. Amplitude-invariant Clarke with all three currents and Park as in
 * the plan (Firmware conventions, Transforms): Iq = peak phase current. */
#define CTRL_ENC_AGE_S       55e-6f
#define CTRL_ONE_THIRD       0.333333333f
#define CTRL_INV_SQRT3       0.577350269f
/* Capture encoding in dq mode: mA + 32768 (+/-32 A range), angle as
 * 0..65535 per electrical turn. */
#define CTRL_CAP_MA_OFFSET   32768.0f
#define CTRL_CAP_ANGLE_SCALE (65536.0f / 6.28318530718f)
static volatile bool     s_cap_dq;
static volatile float    s_dq_sum_id;
static volatile float    s_dq_sum_iq;
static volatile float    s_dq_min_id;
static volatile float    s_dq_max_id;
static volatile float    s_dq_min_iq;
static volatile float    s_dq_max_iq;
static volatile uint32_t s_dq_n;
static volatile uint32_t s_dq_bad;     /* interrupts without a valid angle */

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
    if (!s_th_ok)
    {
      /* Bad or unhealthy encoder sample: no valid angle to commutate on. */
      motor_disarm();
      s_enc_event = true;
      return;
    }
    theta = s_th_enc + (s_we * CTRL_ENC_LATENCY_S);
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
#if BRINGUP_STAGE >= 11
/* Electrical angle = DIR x 11 x theta_m - offset, from the encoder sample
 * just collected (plan: Phase order, angles and direction). */
APP_RAMFUNC static void ctrl_enc_angle(void)
{
  uint16_t counts;
  float speed;
  s_th_ok = mt6701_isr_angle(&counts, &speed);
  uint32_t e = ((uint32_t)counts * MOTOR_POLE_PAIRS) & CTRL_ENC_MASK;
  if (MOTOR_ENC_DIR < 0)
  {
    e = (CTRL_ENC_COUNTS - e) & CTRL_ENC_MASK;
  }
  s_we = (float)MOTOR_ENC_DIR * (float)MOTOR_POLE_PAIRS * speed;
  s_th_enc = ((float)e * CTRL_RAD_PER_ECOUNT) - MOTOR_ENC_OFFSET_RAD;
}
#endif

#if BRINGUP_STAGE >= 12
/* Id, Iq from this interrupt's currents and the encoder angle; statistics
 * for the main loop. False (and nothing computed) without a valid angle. */
APP_RAMFUNC static bool ctrl_dq(const float amps[PHASE_COUNT], float *id, float *iq, float *th)
{
  if (!s_th_ok)
  {
    s_dq_bad++;
    return false;
  }
  float sn;
  float cs;
  *th = s_th_enc + (s_we * CTRL_ENC_AGE_S);
  foc_sincos(*th, &sn, &cs);
  float i_alpha = ((2.0f * amps[PHASE_A]) - amps[PHASE_B] - amps[PHASE_C]) * CTRL_ONE_THIRD;
  float i_beta = (amps[PHASE_B] - amps[PHASE_C]) * CTRL_INV_SQRT3;
  float d = (i_alpha * cs) + (i_beta * sn);
  float q = (i_beta * cs) - (i_alpha * sn);
  *id = d;
  *iq = q;
  if (s_dq_n == 0U)
  {
    s_dq_min_id = d;
    s_dq_max_id = d;
    s_dq_min_iq = q;
    s_dq_max_iq = q;
  }
  s_dq_sum_id += d;
  s_dq_sum_iq += q;
  s_dq_min_id = (d < s_dq_min_id) ? d : s_dq_min_id;
  s_dq_max_id = (d > s_dq_max_id) ? d : s_dq_max_id;
  s_dq_min_iq = (q < s_dq_min_iq) ? q : s_dq_min_iq;
  s_dq_max_iq = (q > s_dq_max_iq) ? q : s_dq_max_iq;
  s_dq_n++;
  return true;
}

APP_RAMFUNC static uint16_t ctrl_cap_ma(float a)
{
  float v = (a * 1000.0f) + CTRL_CAP_MA_OFFSET;
  v = (v < 0.0f) ? 0.0f : ((v > 65535.0f) ? 65535.0f : v);
  return (uint16_t)v;
}
#endif

/* Called by the control interrupt. From Stage 11 the encoder is read here:
 * finish the read started last time (so the angle is fresh for this step),
 * compute the angle (and from Stage 12 Id, Iq), record the capture sample,
 * run the step, then start the next read whatever the step did (armed or
 * not, trip or not), so the encoder keeps updating while disarmed. The
 * capture sample is pushed before the step, so a trigger from the step
 * lands after this interrupt's sample (debug.h). */
APP_RAMFUNC void ctrl_isr(const float amps[PHASE_COUNT], const uint16_t raw[PHASE_COUNT])
{
#if BRINGUP_STAGE >= 11
  mt6701_isr_collect();
  ctrl_enc_angle();
#endif
#if BRINGUP_STAGE >= 12
  float id = 0.0f;
  float iq = 0.0f;
  float th = 0.0f;
  bool dq_ok = ctrl_dq(amps, &id, &iq, &th);
  if (s_cap_dq)
  {
    float t = th * CTRL_CAP_ANGLE_SCALE;
    int32_t ti = (int32_t)t;
    debug_capture_push(dq_ok ? ctrl_cap_ma(id) : 0U, dq_ok ? ctrl_cap_ma(iq) : 0U, (uint16_t)(ti & 0xFFFF));
  }
  else
#endif
  {
    debug_capture_push(raw[PHASE_A], raw[PHASE_B], raw[PHASE_C]);
  }
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

void ctrl_capture_dq(bool dq)
{
  s_cap_dq = dq;
}

void ctrl_dq_stats(ctrl_dq_stats_t *out)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  uint32_t n = s_dq_n;
  out->n = n;
  out->bad = s_dq_bad;
  out->id_mean = (n > 0U) ? (s_dq_sum_id / (float)n) : 0.0f;
  out->iq_mean = (n > 0U) ? (s_dq_sum_iq / (float)n) : 0.0f;
  out->id_min = s_dq_min_id;
  out->id_max = s_dq_max_id;
  out->iq_min = s_dq_min_iq;
  out->iq_max = s_dq_max_iq;
  s_dq_sum_id = 0.0f;
  s_dq_sum_iq = 0.0f;
  s_dq_n = 0U;
  s_dq_bad = 0U;
  __set_PRIMASK(primask);
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

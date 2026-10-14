/**
 * @file    aux_sense.c
 * @brief   Throttle pots and supply-current sensor. See aux_sense.h.
 */
#include "aux_sense.h"
#include "app_config.h"
#include "debug.h"
#include "power.h"
#include "adc.h"
#include "main.h"

/* ---- Constants ------------------------------------------------------------ */

#define AUX_THR1_CHANNEL        LL_ADC_CHANNEL_1    /* PA0 = ADC12_IN1 (DS13122 pinout) */
#define AUX_THR2_CHANNEL        LL_ADC_CHANNEL_2    /* PA1 = ADC12_IN2 */
#define AUX_HALL_CHANNEL        LL_ADC_CHANNEL_11   /* PB12 = ADC1_IN11 */

/* 47.5 ADC clocks = 1.1 us at 42.5 MHz: DS13122 Table 61 allows up to 1.8 k
 * source resistance on a slow channel at 12 bits. The pots (100 R + 100 nF)
 * and the sensor (56 R + 2.2 nF) are well inside that. Conversion with
 * 12.5 clocks of SAR: 60 clocks = 1.4 us. */
#define AUX_SAMPLING_TIME       LL_ADC_SAMPLINGTIME_47CYCLES_5
#define AUX_CONV_TIMEOUT_CYCLES 1700U       /* 10 us at 170 MHz, as power.c */

#define AUX_ADC_FULL_SCALE      4095U
#define AUX_VDDA_MV             3300.0f     /* VREF+ = VDDA = 3.3 V LDO */
#define AUX_ADC_COUNTS          4096.0f

/* ACS71240LLCBTR-050B3 (docs/ACS71240.pdf, performance table for this part):
 * VCC 3.3 V, +/-50 A, Sens 26.4 mV/A, VIOUT(Q) 1.65 V at 0 A. The output is
 * not ratiometric, so with VREF+ = 3.3 V the zero is near code 2048. Offset
 * error VOE up to +/-10 mV (+/-0.38 A) at 25 degC and up, hence the zero
 * measured with SW1 off. Noise 78 mA rms at 3.3 V (150 kHz bandwidth). */
#define AUX_ACS_MV_PER_A        26.4f

#define AUX_HALL_SAMPLES        4U          /* averaged per 1 ms tick */
#define AUX_THR_FILTER_SHIFT    3U          /* IIR 1/8 per ms: tau about 8 ms */
#define AUX_I_FILTER_SHIFT      4U          /* IIR 1/16 per ms: tau about 16 ms */
#define AUX_FIXED_SHIFT         4U          /* filters keep codes x16 */

/* Zero current: averaged over this many 1 ms ticks while SW1 is off (relay
 * K1 open, so no current can flow through the sensor). */
#define AUX_ZERO_TICKS          512U

#define AUX_LOG_PERIOD_MS       250U

/* ---- State ---------------------------------------------------------------- */

static uint32_t s_thr_x16[2];        /* filtered throttle codes x16 */
static uint32_t s_hall_x16;          /* filtered sensor code x16 */
static uint32_t s_zero_x16;          /* zero-current code x16 */
static bool     s_zeroed;
static uint32_t s_zero_sum;
static uint32_t s_zero_n;
static bool     s_started;
static uint32_t s_timeouts;

/* Log window: per-tick sensor code (x16) extremes and sum. */
static uint32_t s_win_min;
static uint32_t s_win_max;
static uint32_t s_win_sum;
static uint32_t s_win_n;

/* ---- ADC ------------------------------------------------------------------ */

/* One ADC2 regular conversion. ADC2's regular sequence is otherwise unused
 * (cursense.c uses only its injected one); a conversion started here is
 * paused, not lost, when the TRGO2 injected trigger arrives. */
static bool aux_adc2_read(uint32_t channel, uint16_t *code)
{
  LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, channel);
  LL_ADC_REG_StartConversion(ADC2);
  uint32_t start = DWT->CYCCNT;
  while (!LL_ADC_IsActiveFlag_EOC(ADC2))
  {
    if ((DWT->CYCCNT - start) > AUX_CONV_TIMEOUT_CYCLES)
    {
      return false;
    }
  }
  *code = LL_ADC_REG_ReadConversionData12(ADC2);
  return true;
}

static int32_t aux_code_x16_to_ma(int32_t delta_x16)
{
  float mv = ((float)delta_x16 / (float)(1U << AUX_FIXED_SHIFT)) * (AUX_VDDA_MV / AUX_ADC_COUNTS);
  return (int32_t)(mv * (1000.0f / AUX_ACS_MV_PER_A));
}

/* ---- API ------------------------------------------------------------------ */

bool aux_sense_init(void)
{
  /* SMPRx may only be written with no conversion running or armed. */
  if (LL_ADC_REG_IsConversionOngoing(ADC1) || LL_ADC_INJ_IsConversionOngoing(ADC1) ||
      LL_ADC_REG_IsConversionOngoing(ADC2) || LL_ADC_INJ_IsConversionOngoing(ADC2))
  {
    return false;
  }
  LL_ADC_SetChannelSamplingTime(ADC1, AUX_HALL_CHANNEL, AUX_SAMPLING_TIME);
  LL_ADC_SetChannelSamplingTime(ADC2, AUX_THR1_CHANNEL, AUX_SAMPLING_TIME);
  LL_ADC_SetChannelSamplingTime(ADC2, AUX_THR2_CHANNEL, AUX_SAMPLING_TIME);
  s_hall_x16 = (AUX_ADC_FULL_SCALE + 1U) / 2U << AUX_FIXED_SHIFT;   /* VCC/2 until measured */
  s_zero_x16 = s_hall_x16;
  s_win_min = UINT32_MAX;
  return true;
}

void aux_sense_tick(uint32_t now_ms)
{
  /* Throttles: ADC2 is enabled by cursense_init(), not before. */
  if (LL_ADC_IsEnabled(ADC2))
  {
    static const uint32_t ch[2] = { AUX_THR1_CHANNEL, AUX_THR2_CHANNEL };
    for (uint32_t i = 0U; i < 2U; i++)
    {
      uint16_t code;
      if (!aux_adc2_read(ch[i], &code))
      {
        s_timeouts++;
        continue;
      }
      uint32_t x16 = (uint32_t)code << AUX_FIXED_SHIFT;
      if (!s_started)
      {
        s_thr_x16[i] = x16;
      }
      s_thr_x16[i] = s_thr_x16[i] + ((int32_t)(x16 - s_thr_x16[i]) >> AUX_THR_FILTER_SHIFT);
    }
  }

  /* Supply current: a few conversions per tick, averaged, then filtered. */
  uint32_t sum = 0U;
  uint32_t n = 0U;
  for (uint32_t i = 0U; i < AUX_HALL_SAMPLES; i++)
  {
    uint16_t code;
    if (power_adc1_read(AUX_HALL_CHANNEL, &code))
    {
      sum += code;
      n++;
    }
    else
    {
      s_timeouts++;
    }
  }
  if (n > 0U)
  {
    uint32_t x16 = (sum << AUX_FIXED_SHIFT) / n;
    if (!s_started)
    {
      s_hall_x16 = x16;
    }
    s_hall_x16 = s_hall_x16 + ((int32_t)(x16 - s_hall_x16) >> AUX_I_FILTER_SHIFT);

    /* Zero: re-measured every time SW1 has been off for AUX_ZERO_TICKS. */
    if (!power_motor_en())
    {
      s_zero_sum += x16;
      if (++s_zero_n >= AUX_ZERO_TICKS)
      {
        s_zero_x16 = s_zero_sum / AUX_ZERO_TICKS;
        s_zeroed = true;
        s_zero_sum = 0U;
        s_zero_n = 0U;
      }
    }
    else
    {
      s_zero_sum = 0U;
      s_zero_n = 0U;
    }

    s_win_min = (x16 < s_win_min) ? x16 : s_win_min;
    s_win_max = (x16 > s_win_max) ? x16 : s_win_max;
    s_win_sum += x16;
    s_win_n++;
  }
  s_started = true;

#if SENSOR_TEST
  if (((now_ms % AUX_LOG_PERIOD_MS) == 0U) && (s_win_n > 0U))
  {
    uint32_t mean = s_win_sum / s_win_n;
    int32_t z = (int32_t)s_zero_x16;
    debug_log("AUX thr1=%lu thr2=%lu permille (code %lu %lu) to=%lu",
              aux_throttle_permille(0U), aux_throttle_permille(1U),
              s_thr_x16[0] >> AUX_FIXED_SHIFT, s_thr_x16[1] >> AUX_FIXED_SHIFT, s_timeouts);
    debug_log("AUX isup=%ld mA (min %ld max %ld) code=%lu zero=%lu %s",
              aux_code_x16_to_ma((int32_t)mean - z),
              aux_code_x16_to_ma((int32_t)s_win_min - z), aux_code_x16_to_ma((int32_t)s_win_max - z),
              mean >> AUX_FIXED_SHIFT, s_zero_x16 >> AUX_FIXED_SHIFT,
              s_zeroed ? (power_motor_en() ? "SW1 on" : "SW1 off, zeroing") : "NOT ZEROED: turn SW1 off");
    s_win_min = UINT32_MAX;
    s_win_max = 0U;
    s_win_sum = 0U;
    s_win_n = 0U;
  }
#else
  (void)now_ms;
#endif
}

uint32_t aux_throttle_permille(uint32_t index)
{
  if (index > 1U)
  {
    return 0U;
  }
  uint32_t code = s_thr_x16[index] >> AUX_FIXED_SHIFT;
  return (code * 1000U) / AUX_ADC_FULL_SCALE;
}

int32_t aux_supply_ma(void)
{
  return aux_code_x16_to_ma((int32_t)s_hall_x16 - (int32_t)s_zero_x16);
}

bool aux_supply_zeroed(void)
{
  return s_zeroed;
}

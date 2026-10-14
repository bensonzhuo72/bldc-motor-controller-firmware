/**
 * @file    app.c
 * @brief   BRINGUP_STAGE switch and the 1 kHz main-loop tasks.
 */
#include "app.h"
#include "app_config.h"
#include "board.h"
#include "debug.h"
#if BRINGUP_STAGE >= 1
#include "mt6701.h"
#endif
#if BRINGUP_STAGE >= 2
#include "power.h"
#endif
#if BRINGUP_STAGE >= 3
#include "drv8323.h"
#endif
#if BRINGUP_STAGE >= 4
#include "fault.h"
#endif
#if BRINGUP_STAGE >= 5
#include "pwm.h"
#endif
#if BRINGUP_STAGE >= 7
#include "cursense.h"
#include "ctrl.h"
#endif
#if BRINGUP_STAGE >= 8
#include "foc_math.h"
#include "motor_params.h"
#endif
#if BRINGUP_STAGE >= 2
#include "aux_sense.h"
#endif
#if COMMS_TEST
#include "comms_test.h"
#endif
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#ifndef BRINGUP_STAGE
#error "BRINGUP_STAGE must be defined in app_config.h"
#endif

/* Stages 5-7 drive the PWM from debugger-set test duties; from Stage 8 the
 * control interrupt takes over. */
#define APP_TEST_MODE   ((BRINGUP_STAGE >= 5) && (BRINGUP_STAGE <= 7))

/* Stages 8-12 drive a voltage vector (fixed angle, forced spin, the Stage 10
 * alignment steps) and, from Stage 11, voltage-mode FOC (Stage 12 adds the
 * Id/Iq readings to it). */
#define APP_VEC_MODE    ((BRINGUP_STAGE >= 8) && (BRINGUP_STAGE <= 12))

/* Main-loop tick: 1 kHz (plan: Timing, interrupts and data sharing). */
#define APP_TICK_S             0.001f

/* Encoder update interval: the main-loop tick until Stage 10; from Stage 11
 * the control interrupt reads it (20 kHz, plan: Timing). */
#if BRINGUP_STAGE >= 11
#define ENC_DT_S               50e-6f
#else
#define ENC_DT_S               APP_TICK_S
#endif

/* Expected clock tree (plan: TIM1 and control timing, "Clock"):
 * 24 MHz HSE, PLL M = 6, N = 85, R = 2 -> 170 MHz SYSCLK = HCLK. */
#define CLK_EXPECTED_HZ        170000000UL
#define CLK_EXPECTED_PLLM      6UL
#define CLK_EXPECTED_PLLN      85UL
#define CLK_EXPECTED_PLLR      2UL

/* Cycle-counter self-check: cycles over a HAL_Delay of this many ms. */
#define CYC_CHECK_MS           10UL

/* Main-loop status line period. */
#define ALIVE_PERIOD_MS        1000UL

#define RAD_S_TO_RPM           9.54929658551f   /* 60 / (2*pi) */

#if BRINGUP_STAGE == 0
/* Stage 0 test patterns (plan: Stage 0 procedure).
 * LED4: 500 ms on / 500 ms off, a 1.000 Hz square wave to time on the scope.
 * PA3:  toggled every 1 ms tick, a 500.0 Hz square wave.
 * PA2:  triangle, 100 ms up and 100 ms down (5 Hz), 0 to full scale. */
#define STAGE0_LED_HALF_PERIOD_MS   500UL
#define STAGE0_TRI_HALF_PERIOD_MS   100UL
#define STAGE0_FAULT_TEST_AT_MS     5000UL
#endif

#if BRINGUP_STAGE == 1
/* Stage 1 test mode (plan: Stage 1 procedure).
 * LED4: 1 Hz heartbeat. PA2: mechanical angle as a sawtooth, 0-3.3 V per turn.
 * ENC status line every 200 ms. */
#define STAGE1_LED_HALF_PERIOD_MS   500UL
#define STAGE1_LOG_PERIOD_MS        200UL
#define STAGE1_ANGLE_TO_DAC_SHIFT   2U      /* 14-bit angle -> 12-bit DAC */
#endif

#if BRINGUP_STAGE >= 2
/* LED4 shows the power state (plan Stage 2): NO_VM slow blink (1 Hz),
 * VM_SETTLING fast blink (5 Hz), VM_OK steady on. */
#define LED_NO_VM_HALF_PERIOD_MS     500UL
#define LED_SETTLING_HALF_PERIOD_MS  100UL
#endif

#if BRINGUP_STAGE == 2
/* Stage 2 test mode (plan: Stage 2 procedure).
 * PWR status line every 500 ms. On each MOTOR_EN rising edge (SW1 on), VM is
 * recorded every 1 ms for 500 ms, then printed every 10 ms with the times
 * from SW1 on to VM >= 10 V and to VM_OK (procedure step 4). */
#define STAGE2_LOG_PERIOD_MS         500UL
#define STAGE2_RAMP_SAMPLES          500U
#define STAGE2_RAMP_PRINT_STEP       10U
#define STAGE2_RAMP_10V_MV           10000UL
#endif

#if BRINGUP_STAGE == 3
/* Stage 3 test mode (plan: Stage 3 procedure). After every wake: presence
 * check, all registers read and compared with their defaults, write and
 * verify 0x05, lock test, unlock, CLR_FLT, then sleep/wake and check the
 * defaults came back. Status line every second while running. */
#define STAGE3_LOG_PERIOD_MS        1000UL
#define STAGE3_TEST_OCP_VALUE       0x110U   /* plan register table: bring-up 0x05 */
#define STAGE3_FAULT_TEXT_LEN       96U
#endif

#if BRINGUP_STAGE >= 4
/* DRV configuration readback period (plan: readback check 3, every 100 ms). */
#define DRV_CHECK_PERIOD_MS         100UL
#define DRV_FAULT_TEXT_LEN          96U
#endif

#if BRINGUP_STAGE == 4
/* Stage 4 test mode: status line every second with the configuration
 * counters, for the SW1 cycling test (plan: Stage 4 procedure). */
#define STAGE4_LOG_PERIOD_MS        1000UL
#endif

#if BRINGUP_STAGE >= 5
/* Debugger commands: pause, `set var g_app_cmd = N` in the Debug Console,
 * continue (Stage 5 onward). Arming is always an explicit action (plan:
 * Rules for the bench, 4). */
#define APP_CMD_NONE       0U
#define APP_CMD_ARM        1U   /* arm from the zero command, then slew to the test duties */
#define APP_CMD_DISARM     2U
#define APP_CMD_SW_BREAK   3U   /* TIM1 software break */
#define APP_CMD_SWEEP      4U   /* Stage 6: duty sweep on all phases */
#define APP_CMD_IDRIVE     5U   /* Stage 6: apply g_drv_idrive (sleep, re-wake, reconfigure) */
#define APP_CMD_CAPTURE    6U   /* Stage 7: trigger the capture buffer */
#define APP_CMD_DUMP       7U   /* Stage 7: dump the capture over SWO */
volatile uint32_t g_app_cmd;
#endif

/* Stage 6 bench tools (sweep, IDRIVE, phase hold-off) stay available in the
 * later test-mode stages. */
#define APP_BRIDGE_TOOLS   ((BRINGUP_STAGE >= 6) && APP_TEST_MODE)

#if APP_TEST_MODE
/* PWM test mode. Target duty per phase, 0..1, set from the debugger
 * (`set var g_duty_a = 0.5`). The applied duties slew toward the targets,
 * so every arm starts from the zero command and every change ramps
 * (plan: Hard rules). Targets are clamped to [0, PWM_DUTY_MAX]. */
#define TEST_SLEW_PER_MS     0.001f   /* full scale in 1 s */
#define TEST_LOG_PERIOD_MS   1000UL
#if BRINGUP_STAGE == 5
/* Plan Stage 5 step 1: a different duty per phase to identify each. */
#define TEST_DUTY_A_INIT     0.20f
#define TEST_DUTY_B_INIT     0.50f
#define TEST_DUTY_C_INIT     0.80f
#elif BRINGUP_STAGE == 6
/* Plan Stage 6 step 1: one phase switching at 50 %; the others at 0 %
 * (low side held on, not switching). */
#define TEST_DUTY_A_INIT     0.50f
#define TEST_DUTY_B_INIT     0.00f
#define TEST_DUTY_C_INIT     0.00f
#else
/* Plan Stage 7: PWM at 50 % on all phases (zero current, no motor). */
#define TEST_DUTY_A_INIT     0.50f
#define TEST_DUTY_B_INIT     0.50f
#define TEST_DUTY_C_INIT     0.50f
#endif
volatile float g_duty_a = TEST_DUTY_A_INIT;
volatile float g_duty_b = TEST_DUTY_B_INIT;
volatile float g_duty_c = TEST_DUTY_C_INIT;
#endif

#if BRINGUP_STAGE >= 7
/* Current-sense status line period (plan Stage 7: zero-current noise). */
#define CUR_LOG_PERIOD_MS    1000UL
/* Control interrupt budget: one PWM period, 8,500 cycles at 170 MHz (plan:
 * TIM1 and control timing). */
#define CTRL_PERIOD_CYCLES   8500UL
#define AMPS_TO_MA           1000.0f
#endif

#if APP_BRIDGE_TOOLS
/* Plan Stage 6 step 4: duty sweep, all phases together, each step held
 * long enough to read the switch-node average on a meter. The plan says
 * 5-95 %; the top step is the PWM_DUTY_MAX cap (93 %). */
#define SWEEP_STEP_MS        4000UL
static const float s_sweep_duty[] = { 0.05f, 0.10f, 0.25f, 0.50f, 0.75f, 0.90f, PWM_DUTY_MAX };
#define SWEEP_STEPS          (sizeof(s_sweep_duty) / sizeof(s_sweep_duty[0]))

/* Plan Stage 6 step 2: IDRIVE code to try, IDRIVEP << 4 | IDRIVEN
 * (datasheet Tables 8-16, 8-17), same on both sides. Applied by command 5. */
volatile uint32_t g_drv_idrive = DRV_CFG_IDRIVE;

/* Phases held off at the next arm, bit 0 = A, 1 = B, 2 = C: their INH and
 * INL are driven low, so both FETs stay off and the phase floats. Isolates
 * one half-bridge (2026-10-08, VDS_HA on phase A). */
volatile uint32_t g_phase_off;
#endif

#if APP_VEC_MODE
/* Stages 8-9 test mode: a voltage vector into the motor. Stage 8: fixed
 * angle (plan Stage 8). Stage 9: forced angle at a ramped speed, open loop
 * (plan Stage 9). Set from the debugger: amplitude g_vec_v (phase-voltage
 * amplitude, V), common-mode duty shift g_duty_shift (Stage 8 step 7),
 * overcurrent trip g_oc_trip (1.0 A until the meter check, then 1.5 A);
 * Stage 8: angle g_vec_deg, step g_vstep_v; Stage 9: g_spin_hz, g_spin_accel. */
#define APP_CMD_LSTEP        8U       /* inductance step, g_vec_v -> g_vstep_v */
#if BRINGUP_STAGE == 8
#define VEC_V_INIT           0.5f     /* Stage 8 procedure step 2 */
#else
#define VEC_V_INIT           1.0f     /* Stage 9 procedure step 1: about 1 V */
#endif
#define VEC_V_MAX            1.2f     /* covers the 1.0 V step; 1.2 V / 2.2 Ohm = 0.55 A, under the 1 A trip */
#define VEC_SLEW_V_PER_MS    0.001f   /* 1 V/s: every arm starts at 0 V and ramps */
#define VEC_SHIFT_MAX        0.43f    /* 0.5 + 0.43 = 0.93: largest useful shift */
#define VEC_LOG_PERIOD_MS    1000UL
#define DEG_TO_RAD           0.0174532925199f
#define RAD_PER_TURN         6.28318530718f
/* Inductance fit (step 7): phase A current averaged over 100 samples before
 * the step and over the last 100 of the capture (20-25 ms after it, many
 * time constants), 63.2 % crossing for tau. The step reaches the motor at
 * the PWM update after the trigger interrupt, half a sample before n = 512. */
#define LFIT_AVG_SAMPLES     100U
#define LFIT_STEP_AT         511.5f
#define LFIT_TAU_FRACTION    0.632121f
/* After re-arming the capture, wait for 200 fresh samples (10 ms) so the
 * 100-sample pre-step average contains no old data. */
#define LSTEP_PRETRIGGER_MS  10U
#define CTRL_SAMPLE_S        50e-6f   /* control interrupt period */
/* Rotor-motion check (procedure step 6): encoder position read this long
 * after an angle change, once the rotor has settled. 120 deg electrical =
 * 120 / 11 = 10.9 deg mechanical = 496 counts (16384 counts per turn). */
#define VEC_MOVE_SETTLE_MS   300U
#define VEC_ARM_SETTLE_MS    800U     /* 0.5 V ramp at 1 V/s, then settle */
#define ENC_COUNTS_PER_TURN  16384L
#define ENC_COUNTS_TO_MDEG   (360000.0f / 16384.0f)   /* millidegrees per count */

/* Stage 9 forced spin (plan Stage 9 step 1: about 1 V, ramp 0 to 5 Hz
 * electrical). The limit keeps open loop in sync: at 10 Hz electrical the
 * back-EMF is 0.1562 x 2 pi x 10 / 11 = 0.89 V line-to-line (Ke is
 * line-to-line), 0.51 V phase amplitude against the 1-1.2 V applied
 * (corrected 2026-10-14: the comparison first used the line value).
 * The rotor is held at 0 deg for SPIN_ALIGN_MS after arming (ramp, then
 * settle) before the angle starts to move. */
#define SPIN_HZ_INIT         5.0f
#define SPIN_HZ_MAX          10.0f
#define SPIN_ACCEL_INIT      2.5f     /* Hz/s: 0 to 5 Hz in 2 s */
#define SPIN_ACCEL_MAX       10.0f
#define SPIN_ALIGN_MS        1500U
#define SPIN_MIN_CYCLES_LOG  0.5f     /* electrical cycles per log period to compute counts/cycle */

volatile float g_vec_v = VEC_V_INIT;
volatile float g_vec_deg = 0.0f;
volatile float g_duty_shift = 0.0f;
#if BRINGUP_STAGE >= 11
volatile float g_oc_trip = MOTOR_OC_TRIP_STAGE_MAX_A;   /* Stage 11: 1.5 A (developer, 2026-10-14) */
#else
volatile float g_oc_trip = MOTOR_OC_TRIP_DEFAULT_A;
#endif
volatile float g_vstep_v = 1.0f;      /* Stage 8 procedure step 7 */
volatile float g_spin_hz = SPIN_HZ_INIT;      /* Stage 9: electrical Hz, sign = direction */
volatile float g_spin_accel = SPIN_ACCEL_INIT;

#if BRINGUP_STAGE >= 10
/* Stage 10 encoder-to-electrical alignment (plan Stage 10). The d-axis
 * vector (g_vec_v) is stepped through two electrical cycles forward, then
 * two back; at each step the rotor settles and the encoder is averaged.
 * Offset per point: theta_offset = wrap(DIR x 11 x theta_m - theta_cmd)
 * (plan: Phase order, angles and direction). Forward and backward points
 * are averaged so friction and cogging pull cancel. */
#define APP_CMD_ALIGN         9U
#define ALIGN_STEP_DEG        60.0f
#define ALIGN_FWD_POINTS      13U      /* 0, 60 ... 720 deg */
#define ALIGN_POINTS          25U      /* then 660 ... 0 deg */
#define ALIGN_SLEW_DEG_PER_MS 0.12f    /* 120 deg/s electrical (1 rpm mechanical): the rotor follows */
#define ALIGN_START_MS        2000U    /* at 0 deg: up to 1.2 V ramp at 1 V/s, then settle */
#define ALIGN_SETTLE_MS       400U
#define ALIGN_AVG_MS          64U      /* encoder samples averaged per point */
#define ALIGN_CPC_NOMINAL     (16384.0f / (float)MOTOR_POLE_PAIRS)   /* 1489.5 counts per electrical cycle */
#define ALIGN_CPC_TOL         0.10f    /* the rotor must follow within 10 % */
#define ALIGN_PASS_DEV_DEG    2.0f     /* plan Stage 10 pass: repeat within +/-2 deg electrical */
#define DEG_PER_COUNT_ELEC    (360.0f / 16384.0f)

typedef enum
{
  ALIGN_IDLE = 0,
  ALIGN_START,
  ALIGN_MOVE,
  ALIGN_SETTLE,
  ALIGN_MEASURE
} align_phase_t;
#endif

#if BRINGUP_STAGE >= 11
/* Stage 11 voltage-mode FOC (plan Stage 11): Vd = 0, Vq = g_vq on the
 * encoder angle, no current loop. Limits (developer, 2026-10-14): |Vq| at
 * most 2 V, so at most 2 / 2.2 Ohm = 0.91 A with the rotor held, under the
 * 1.5 A trip. Every arm starts at Vq = 0 and every change is ramped (plan:
 * "changed through a ramp"), which also limits the braking energy pumped
 * into VM when Vq steps down. */
#define APP_CMD_FOC           10U
#define FOC_VQ_MAX            2.0f
#define FOC_VQ_INIT           0.5f     /* plan Stage 11 step 1: first level */
#define FOC_VQ_SLEW_V_PER_MS  0.001f   /* 1 V/s */
/* No-load speed: back-EMF line amplitude = Ke x w (Ke is line-to-line, plan:
 * Motor table, matches Kv 61 rpm/V), phase amplitude Vq = Ke x w / sqrt 3,
 * so w = sqrt 3 x Vq / Ke: 106 rpm per volt of Vq. */
#define FOC_RPM_PER_VQ        (1.7320508f / MOTOR_KE_V_PER_RAD_S * RAD_S_TO_RPM)
volatile float g_vq = FOC_VQ_INIT;
/* Vq from THROTTLE1 instead of g_vq (2026-10-14). Changing g_vq needs a
 * debugger pause, and a pause while armed stops TIM1 with its outputs off
 * (DBG_TIM1_STOP: "as if MOE was reset", RM0440 29.3.34) and stops the
 * encoder reads in the control interrupt; the coasting rotor then moves
 * more than the jump limit before the next read, the interrupt rejects the
 * sample and disarms (FAULT_ENCODER). The pot changes Vq without a pause.
 * Mapping: mid-travel = 0, ends = -/+FOC_VQ_MAX, with a dead band around
 * the middle. Arming needs the pot inside the dead band (zero command). */
volatile uint32_t g_vq_pot = 1U;
#define FOC_POT_MID_PERMILLE  500.0f
#define FOC_POT_DEADBAND      0.05f    /* +/-5 % of half travel around the middle */
#endif

#if BRINGUP_STAGE >= 12
/* Stage 12 (plan): Id/Iq telemetry. g_cap_dq = 1: the capture buffer
 * (g_app_cmd 6, 7) records Id, Iq and the electrical angle instead of the
 * raw ADC counts; 0: raw counts as before. */
volatile uint32_t g_cap_dq = 1U;
/* Capture during FOC without a debugger pause (a pause while armed disarms):
 * set g_cap_auto_ms before arming; that long after the FOC arm the capture
 * is re-armed, triggered 60 ms later (after 1024 fresh samples, 51 ms) and
 * dumped automatically when full. 0 = off. One capture per arm. */
volatile uint32_t g_cap_auto_ms = 0U;
#define DQ_CAP_REFILL_MS  60U
#define DQ_CAP_LABEL   "Id+32768 Iq+32768 in mA, theta_e 0-65535 per elec turn"
#define DQ_RAW_LABEL   "counts A B C"
#endif
#endif

static uint32_t s_last_tick_ms;
static uint32_t s_task_max_cycles;

#if APP_TEST_MODE
static float s_duty[PHASE_COUNT];   /* applied duties */
#endif
#if BRINGUP_STAGE >= 8
static float s_trip_applied;        /* overcurrent trip in force, A */
#endif
#if APP_VEC_MODE
static bool  s_foc_ok;
static float s_vec_v;               /* applied amplitude, slewed */
static uint32_t s_align_ms;         /* Stage 9: ms left holding at 0 deg after arming */
static int32_t s_spin_pos0;         /* Stage 9: encoder position at the last log */
static float s_spin_cyc0;           /* Stage 9: electrical cycles at the last log */
static bool  s_lstep_active;        /* waiting for the step's capture */
#if BRINGUP_STAGE >= 10
static align_phase_t s_al_phase;    /* Stage 10 alignment run */
static uint32_t s_al_timer;
static uint32_t s_al_index;         /* point being taken */
static float    s_al_theta_deg;     /* angle commanded now */
static int64_t  s_al_pos_sum;      /* 64 multi-turn positions: can pass int32 after long runs */
static uint32_t s_al_pos_n;
static int32_t  s_al_pos[ALIGN_POINTS];   /* averaged multi-turn position per point */
static bool     s_al_valid;         /* a run passed this boot: its result is used by TRACK */
static int32_t  s_al_dir;
static float    s_al_offset_deg;
#if BRINGUP_STAGE >= 11
static bool     s_foc_on;           /* armed in FOC mode */
static float    s_vq_applied;       /* ramped Vq, V */
static int32_t  s_foc_pos0;         /* encoder position at the last FOC log */
static uint32_t s_vm_max_mv;        /* highest VM since the last FOC log */
#endif
#if BRINGUP_STAGE >= 12
static uint32_t s_foc_ms;           /* ms since the FOC arm */
static uint32_t s_cap_auto_at;      /* auto-capture time for this arm, 0 = none */
static bool     s_cap_auto_dump;    /* dump when the auto capture is full */
#endif
static float    s_tr_sum;           /* TRACK statistics over the log period */
static float    s_tr_min;
static float    s_tr_max;
static uint32_t s_tr_n;
#endif
#if BRINGUP_STAGE == 8
static uint32_t s_lstep_wait;       /* ms until the step is applied; 0 = none */
static float s_lstep_v1;            /* step target, V */
static float s_move_deg;            /* angle at the last change */
static int32_t s_move_from;         /* encoder position (counts) at the change */
static float s_move_from_deg;       /* angle before the change (< 0: the arm) */
static uint32_t s_move_ms;          /* ms since the change */
static uint32_t s_move_wait;        /* ms to wait before reading */
static bool  s_move_pending;
#endif
#endif
#if APP_BRIDGE_TOOLS
static uint32_t s_sweep_step;
static uint32_t s_sweep_ms;
static bool s_sweeping;
#endif

#if BRINGUP_STAGE >= 4
static uint32_t s_drv_wakes;
static uint32_t s_drv_cfg_ok;
static uint32_t s_drv_cfg_fail;
static uint32_t s_drv_checks;
static uint32_t s_drv_spi_err_seen;
static bool s_fault_logged;
#endif

#if BRINGUP_STAGE >= 1
static uint32_t s_enc_read_max_cycles;
static uint8_t s_enc_logged_status;
static bool s_enc_logged_valid;
#endif

#if BRINGUP_STAGE == 2
static uint16_t s_ramp_mv[STAGE2_RAMP_SAMPLES];
static bool s_ramp_capturing;
static bool s_ramp_dumping;
static uint32_t s_ramp_idx;
static int32_t s_ramp_t10v_ms;
static int32_t s_ramp_tok_ms;
static bool s_en_raw_prev;
#endif

/* ---- Boot checks ---------------------------------------------------------- */

static bool app_check_clock(void)
{
  uint32_t pllcfgr = RCC->PLLCFGR;
  uint32_t pllm = ((pllcfgr & RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1UL;
  uint32_t plln = (pllcfgr & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos;
  uint32_t pllr = (((pllcfgr & RCC_PLLCFGR_PLLR) >> RCC_PLLCFGR_PLLR_Pos) + 1UL) * 2UL;
  bool hse_ready = (__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) != 0U);
  bool pll_from_hse = (__HAL_RCC_GET_PLL_OSCSOURCE() == RCC_PLLSOURCE_HSE);
  bool sys_from_pll = (__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_PLLCLK);
  uint32_t sysclk = HAL_RCC_GetSysClockFreq();
  uint32_t hclk = HAL_RCC_GetHCLKFreq();

  bool ok = hse_ready && pll_from_hse && sys_from_pll &&
            (pllm == CLK_EXPECTED_PLLM) && (plln == CLK_EXPECTED_PLLN) &&
            (pllr == CLK_EXPECTED_PLLR) &&
            (sysclk == CLK_EXPECTED_HZ) && (hclk == CLK_EXPECTED_HZ) &&
            (SystemCoreClock == CLK_EXPECTED_HZ);

  debug_log("CLK hse_rdy=%u pll_src=%s sys_src=%s m=%lu n=%lu r=%lu",
            hse_ready ? 1U : 0U, pll_from_hse ? "HSE" : "OTHER",
            sys_from_pll ? "PLL" : "OTHER", pllm, plln, pllr);
  debug_log("CLK sysclk=%lu hclk=%lu pclk1=%lu pclk2=%lu %s",
            sysclk, hclk, HAL_RCC_GetPCLK1Freq(), HAL_RCC_GetPCLK2Freq(),
            ok ? "OK" : "FAIL");
  return ok;
}

static void app_check_cycle_counter(void)
{
  uint32_t start = debug_cycles();
  HAL_Delay(CYC_CHECK_MS);
  uint32_t elapsed = debug_cycles() - start;

  /* HAL_Delay waits at least CYC_CHECK_MS and up to one tick more. */
  debug_log("CYC %lums=%lu expect>=%lu %s", CYC_CHECK_MS, elapsed,
            CYC_CHECK_MS * (DEBUG_CPU_HZ / 1000UL),
            (elapsed >= CYC_CHECK_MS * (DEBUG_CPU_HZ / 1000UL)) ? "OK" : "FAIL");
}

static void app_check_safe_pins(void)
{
  uint32_t err = board_safe_pins_check();
  debug_log("SAFE pwm_pins=0x%02lX err=0x%02lX %s",
            board_pwm_pins_read(), err, (err == 0U) ? "OK" : "FAIL");
}

/* ---- Stage 0 tick task ---------------------------------------------------- */

#if BRINGUP_STAGE == 0
static void stage0_tick(uint32_t now_ms)
{
  /* PA3: 500 Hz square wave. */
  debug_timing_toggle();

  /* LED4: 1 Hz. */
  if ((now_ms % STAGE0_LED_HALF_PERIOD_MS) == 0U)
  {
    HAL_GPIO_TogglePin(BOARD_LED_PORT, BOARD_LED_PIN);
  }

  /* PA2: triangle wave, full DAC range. */
  uint32_t phase = now_ms % (2UL * STAGE0_TRI_HALF_PERIOD_MS);
  uint32_t ramp = (phase < STAGE0_TRI_HALF_PERIOD_MS) ? phase : (2UL * STAGE0_TRI_HALF_PERIOD_MS - phase);
  debug_scope_write((ramp * DEBUG_SCOPE_FULL_SCALE) / STAGE0_TRI_HALF_PERIOD_MS);

#if STAGE0_FAULT_TEST
  if (now_ms == STAGE0_FAULT_TEST_AT_MS)
  {
    __asm volatile("udf #0");  /* undefined instruction -> HardFault */
  }
#endif
}
#endif

/* ---- Encoder (Stage 1 onward) --------------------------------------------- */

#if BRINGUP_STAGE >= 1
static void enc_log_status_change(const mt6701_snapshot_t *s)
{
  if (!s_enc_logged_valid || (mt6701_field(s->status) != mt6701_field(s_enc_logged_status)))
  {
    debug_log("ENC field=%s", mt6701_field_name(mt6701_field(s->status)));
  }
  if (!s_enc_logged_valid || (mt6701_loss_of_track(s->status) != mt6701_loss_of_track(s_enc_logged_status)))
  {
    debug_log("ENC loss_of_track=%u", mt6701_loss_of_track(s->status) ? 1U : 0U);
  }
  if (!s_enc_logged_valid || (mt6701_push(s->status) != mt6701_push(s_enc_logged_status)))
  {
    debug_log("ENC push=%u", mt6701_push(s->status) ? 1U : 0U);
  }
  s_enc_logged_status = s->status;
  s_enc_logged_valid = true;
}

/* Read the encoder every tick. PA3 is high during the read (scope trigger)
 * until the control interrupt takes over PA3 in Stage 7. */
static void enc_tick(mt6701_snapshot_t *s)
{
#if BRINGUP_STAGE < 7
  debug_timing_high();
#endif
  uint32_t start = debug_cycles();
#if BRINGUP_STAGE < 11
  (void)mt6701_update();         /* from Stage 11 the control interrupt reads it */
#endif
  uint32_t elapsed = debug_cycles() - start;
#if BRINGUP_STAGE < 7
  debug_timing_low();
#endif
  if (elapsed > s_enc_read_max_cycles)
  {
    s_enc_read_max_cycles = elapsed;
  }

  mt6701_snapshot(s);
  if (s->counters.reads > (s->counters.spi_timeouts + s->counters.crc_errors))
  {
    enc_log_status_change(s);
  }
}
#endif

#if BRINGUP_STAGE >= 9
/* Encoder faults disarm from Stage 9 (plan: Firmware protection, Encoder
 * faults): a CRC error, SPI timeout or impossible jump since the last read,
 * or an unhealthy status (field too weak or strong, loss of track), while
 * armed, latches FAULT_ENCODER; the fault response disarms (motor coasts).
 * ctx1 = new CRC | timeouts << 8 | jumps << 16 (low 8 bits each),
 * ctx2 = status bits | healthy << 8. */
static void enc_monitor(const mt6701_snapshot_t *s)
{
  static uint32_t crc;
  static uint32_t to;
  static uint32_t jump;
  uint32_t d_crc = s->counters.crc_errors - crc;
  uint32_t d_to = s->counters.spi_timeouts - to;
  uint32_t d_jump = s->counters.angle_jumps - jump;
  crc = s->counters.crc_errors;
  to = s->counters.spi_timeouts;
  jump = s->counters.angle_jumps;

  if (pwm_is_armed() && ((d_crc | d_to | d_jump) != 0U || !s->healthy))
  {
    (void)fault_raise(FAULT_ENCODER,
                      (d_crc & 0xFFU) | ((d_to & 0xFFU) << 8) | ((d_jump & 0xFFU) << 16),
                      (uint32_t)s->status | (s->healthy ? 0x100U : 0U));
  }
}
#endif

/* ---- Stage 1 tick task ---------------------------------------------------- */

#if BRINGUP_STAGE == 1
static void stage1_tick(uint32_t now_ms, const mt6701_snapshot_t *s)
{
  debug_scope_write((uint32_t)s->angle_counts >> STAGE1_ANGLE_TO_DAC_SHIFT);

  if ((now_ms % STAGE1_LED_HALF_PERIOD_MS) == 0U)
  {
    HAL_GPIO_TogglePin(BOARD_LED_PORT, BOARD_LED_PIN);
  }

  if ((now_ms % STAGE1_LOG_PERIOD_MS) == 0U)
  {
    debug_log("ENC a=%u turns=%ld rpm=%ld ok=%u crc=%lu jump=%lu to=%lu strong=%lu weak=%lu lot=%lu rd_us=%lu",
              (unsigned)s->angle_counts, (long)s->turns,
              (long)(s->speed_rad_s * RAD_S_TO_RPM),
              s->healthy ? 1U : 0U,
              s->counters.crc_errors, s->counters.angle_jumps, s->counters.spi_timeouts,
              s->counters.field_strong, s->counters.field_weak, s->counters.loss_of_track,
              s_enc_read_max_cycles / DEBUG_CYCLES_PER_US);
  }
}
#endif

/* ---- Power (Stage 2 onward) ----------------------------------------------- */

#if BRINGUP_STAGE >= 2
static void power_tick(uint32_t now_ms)
{
  power_state_t prev = power_state();
  if (power_update())
  {
    debug_log("PWR %s -> %s en=%u vm_mv=%lu",
              power_state_name(prev), power_state_name(power_state()),
              power_motor_en() ? 1U : 0U, power_vm_mv());
  }

  /* LED4 shows the power state. */
  switch (power_state())
  {
    case POWER_VM_OK:
      BOARD_LED_PORT->BSRR = BOARD_LED_PIN;
      break;
    case POWER_VM_SETTLING:
      if ((now_ms % LED_SETTLING_HALF_PERIOD_MS) == 0U)
      {
        HAL_GPIO_TogglePin(BOARD_LED_PORT, BOARD_LED_PIN);
      }
      break;
    case POWER_NO_VM:
    default:
      if ((now_ms % LED_NO_VM_HALF_PERIOD_MS) == 0U)
      {
        HAL_GPIO_TogglePin(BOARD_LED_PORT, BOARD_LED_PIN);
      }
      break;
  }
}
#endif

/* ---- Stage 2 tick task ---------------------------------------------------- */

#if BRINGUP_STAGE == 2
static void stage2_ramp(void)
{
  bool en_raw = power_motor_en_raw();

  if (en_raw && !s_en_raw_prev && !s_ramp_capturing && !s_ramp_dumping)
  {
    s_ramp_capturing = true;
    s_ramp_idx = 0U;
    s_ramp_t10v_ms = -1;
    s_ramp_tok_ms = -1;
  }
  s_en_raw_prev = en_raw;

  if (s_ramp_capturing)
  {
    uint32_t mv = power_vm_mv();
    s_ramp_mv[s_ramp_idx] = (uint16_t)((mv > 0xFFFFUL) ? 0xFFFFUL : mv);
    if ((s_ramp_t10v_ms < 0) && (mv >= STAGE2_RAMP_10V_MV))
    {
      s_ramp_t10v_ms = (int32_t)s_ramp_idx;
    }
    if ((s_ramp_tok_ms < 0) && power_vm_ok())
    {
      s_ramp_tok_ms = (int32_t)s_ramp_idx;
    }
    s_ramp_idx++;
    if (s_ramp_idx >= STAGE2_RAMP_SAMPLES)
    {
      s_ramp_capturing = false;
      s_ramp_dumping = true;
      s_ramp_idx = 0U;
    }
  }
  else if (s_ramp_dumping)
  {
    /* One line per tick, so printing never holds up the loop. */
    if (s_ramp_idx < STAGE2_RAMP_SAMPLES)
    {
      debug_log("RAMP t=%lums vm_mv=%u", s_ramp_idx, (unsigned)s_ramp_mv[s_ramp_idx]);
      s_ramp_idx += STAGE2_RAMP_PRINT_STEP;
    }
    else
    {
      debug_log("RAMP sw1_on->10V=%ldms sw1_on->VM_OK=%ldms (-1 = not within %ums)",
                (long)s_ramp_t10v_ms, (long)s_ramp_tok_ms, STAGE2_RAMP_SAMPLES);
      s_ramp_dumping = false;
    }
  }
}

static void stage2_tick(uint32_t now_ms)
{
  stage2_ramp();

  if (((now_ms % STAGE2_LOG_PERIOD_MS) == 0U) && !s_ramp_capturing && !s_ramp_dumping)
  {
    debug_log("PWR st=%s en=%u en_raw=%u vm_mv=%lu code=%u adc_to=%lu",
              power_state_name(power_state()), power_motor_en() ? 1U : 0U,
              power_motor_en_raw() ? 1U : 0U, power_vm_mv(),
              (unsigned)power_vm_code(), power_adc_timeouts());
  }
}
#endif

/* ---- Stage 3 test sequence ------------------------------------------------ */

#if BRINGUP_STAGE == 3
static bool stage3_log_faults(void)
{
  uint16_t f1 = 0U;
  uint16_t f2 = 0U;
  char text[STAGE3_FAULT_TEXT_LEN];
  bool ok = drv8323_read(DRV_REG_FAULT1, &f1) && drv8323_read(DRV_REG_FAULT2, &f2);
  drv8323_fault_text(f1, f2, text, sizeof(text));
  debug_log("DRV faults f1=0x%03X f2=0x%03X %s nfault=%s", (unsigned)f1, (unsigned)f2,
            ok ? text : "READ_FAIL", drv8323_nfault_low() ? "LOW" : "high");
  return ok && (f1 == 0U) && (f2 == 0U);
}

static bool stage3_dump_regs(void)
{
  static const uint16_t defaults[DRV_REG_COUNT] =
  {
    0U, 0U, DRV_DEFAULT_DRIVER_CTRL, DRV_DEFAULT_GATE_HS, DRV_DEFAULT_GATE_LS,
    DRV_DEFAULT_OCP_CTRL, DRV_DEFAULT_CSA_CTRL
  };
  bool all_ok = true;

  for (uint8_t addr = 0U; addr < DRV_REG_COUNT; addr++)
  {
    uint16_t v = 0U;
    bool ok = drv8323_read(addr, &v);
    if (addr <= DRV_REG_FAULT2)
    {
      debug_log("DRV reg 0x%02X = 0x%03X %s", addr, (unsigned)v, ok ? "" : "READ_FAIL");
    }
    else
    {
      bool match = ok && (v == defaults[addr]);
      debug_log("DRV reg 0x%02X = 0x%03X default=0x%03X %s", addr, (unsigned)v,
                (unsigned)defaults[addr], match ? "OK" : "FAIL");
      all_ok = all_ok && match;
    }
  }
  return all_ok;
}

static void stage3_test(void)
{
  bool pass = true;
  bool ok;
  uint16_t v = 0U;

  /* 1. Presence: 0x03 must read 0x3FF (0x7FF = no reply, SDO pull-up). */
  ok = drv8323_present(&v);
  debug_log("DRV present reg03=0x%03X %s", (unsigned)v,
            ok ? "OK" : ((v == DRV_DATA_MASK) ? "FAIL (0x7FF = no reply)" : "FAIL"));
  if (!ok)
  {
    debug_log("DRV test FAIL (not present)");
    return;
  }

  /* 2. All registers at their defaults; no faults after wake. */
  pass &= stage3_dump_regs();
  pass &= stage3_log_faults();

  /* 3. Write and verify 0x05. */
  ok = drv8323_write_verify(DRV_REG_OCP_CTRL, STAGE3_TEST_OCP_VALUE, &v);
  debug_log("DRV write 0x05=0x%03X readback=0x%03X %s", STAGE3_TEST_OCP_VALUE, (unsigned)v, ok ? "OK" : "FAIL");
  pass &= ok;

  /* 4. Lock, then try to restore 0x05: the write must be ignored. */
  ok = drv8323_write_verify(DRV_REG_GATE_HS, DRV_GATE_HS_LOCK_ON | (DRV_DEFAULT_GATE_HS & ~DRV_GATE_HS_LOCK_MASK), &v);
  debug_log("DRV lock reg03=0x%03X %s", (unsigned)v, ok ? "OK" : "FAIL");
  pass &= ok;
  ok = drv8323_write(DRV_REG_OCP_CTRL, DRV_DEFAULT_OCP_CTRL, NULL) && drv8323_read(DRV_REG_OCP_CTRL, &v) &&
       (v == STAGE3_TEST_OCP_VALUE);
  debug_log("DRV locked write to 0x05 ignored: reg05=0x%03X %s", (unsigned)v, ok ? "OK" : "FAIL");
  pass &= ok;

  /* 5. Unlock, then the restore must work. */
  ok = drv8323_write_verify(DRV_REG_GATE_HS, DRV_DEFAULT_GATE_HS, &v);
  debug_log("DRV unlock reg03=0x%03X %s", (unsigned)v, ok ? "OK" : "FAIL");
  pass &= ok;
  ok = drv8323_write_verify(DRV_REG_OCP_CTRL, DRV_DEFAULT_OCP_CTRL, &v);
  debug_log("DRV unlocked write 0x05=0x%03X readback=0x%03X %s", DRV_DEFAULT_OCP_CTRL, (unsigned)v, ok ? "OK" : "FAIL");
  pass &= ok;

  /* 6. CLR_FLT: bit self-clears, so 0x02 reads back 0x000. */
  ok = drv8323_write(DRV_REG_DRIVER_CTRL, DRV_DRIVER_CTRL_CLR_FLT, NULL) &&
       drv8323_read(DRV_REG_DRIVER_CTRL, &v) && (v == DRV_DEFAULT_DRIVER_CTRL);
  debug_log("DRV clr_flt reg02=0x%03X %s", (unsigned)v, ok ? "OK" : "FAIL");
  pass &= ok;
  pass &= stage3_log_faults();

  /* 7. Sleep and wake: registers must return to defaults. */
  ok = drv8323_write_verify(DRV_REG_OCP_CTRL, STAGE3_TEST_OCP_VALUE, &v);
  pass &= ok;
  drv8323_sleep();
  bool nfault_ok = drv8323_wake();
  ok = drv8323_read(DRV_REG_OCP_CTRL, &v) && (v == DRV_DEFAULT_OCP_CTRL);
  debug_log("DRV sleep/wake reset: reg05=0x%03X (was 0x%03X) nfault=%s %s", (unsigned)v,
            STAGE3_TEST_OCP_VALUE, nfault_ok ? "high" : "LOW", ok ? "OK" : "FAIL");
  pass &= ok && nfault_ok;
  pass &= stage3_dump_regs();

  debug_log("DRV test %s spi_err=%lu", pass ? "PASS" : "FAIL", drv8323_spi_errors());
}
#endif

/* ---- Gate driver (Stage 3 onward) ----------------------------------------- */

#if BRINGUP_STAGE >= 4
static fault_code_t drv_cfg_fault_code(drv_cfg_status_t status)
{
  switch (status)
  {
    case DRV_CFG_NOT_PRESENT:    return FAULT_DRV_NOT_PRESENT;
    case DRV_CFG_LOCK_TEST_FAIL: return FAULT_DRV_LOCK_TEST;
    default:                     return FAULT_DRV_CONFIG_WRITE;
  }
}

static void drv_log_registers(const char *prefix)
{
  uint16_t v[DRV_REG_COUNT] = {0U};
  for (uint8_t a = 0U; a < DRV_REG_COUNT; a++)
  {
    (void)drv8323_read(a, &v[a]);
  }
  debug_log("%s f1=0x%03X f2=0x%03X 02=0x%03X 03=0x%03X 04=0x%03X 05=0x%03X 06=0x%03X",
            prefix, (unsigned)v[0], (unsigned)v[1], (unsigned)v[2], (unsigned)v[3],
            (unsigned)v[4], (unsigned)v[5], (unsigned)v[6]);
}

/* After a wake: configure, verify, calibrate, lock (plan Stage 4). */
static void drv_configure_after_wake(bool nfault_released, uint32_t blanked_before)
{
  uint32_t pulses = drv8323_nfault_blanked_events() - blanked_before;

  s_drv_wakes++;
  if (!nfault_released)
  {
    debug_log("DRV wake #%lu nfault still LOW after %s", s_drv_wakes, "1.1 ms");
    (void)fault_raise(FAULT_DRV_WAKE, 0U, 0U);
    return;
  }

  drv_cfg_result_t r;
  if (drv8323_configure(&r))
  {
    s_drv_cfg_ok++;
    debug_log("DRV wake #%lu nfault=high wake_pulses_ignored=%lu cfg OK (cal done, locked, lock test reg05=0x%03X)",
              s_drv_wakes, pulses, (unsigned)r.read);
    drv_log_registers("DRV cfg");
#if BRINGUP_STAGE >= 7
    /* Amplifiers just calibrated, nothing armed: measure the zero-current
     * offsets (plan Stage 7). Arming is refused until they're valid. */
    cursense_offset_start();
#endif
  }
  else
  {
    s_drv_cfg_fail++;
    debug_log("DRV wake #%lu cfg FAIL status=%u reg=0x%02X wrote=0x%03X read=0x%03X",
              s_drv_wakes, (unsigned)r.status, r.addr, (unsigned)r.wrote, (unsigned)r.read);
    (void)fault_raise(drv_cfg_fault_code(r.status),
                      ((uint32_t)r.addr << 16) | r.wrote, r.read);
  }
}

/* Every tick while awake: nFAULT events, SPI errors, 100 ms readback.
 * Then the fault response: log, read the fault registers, put the DRV to
 * sleep (disarm). Faults stay latched until g_fault_clear_request. */
static void drv_monitor(uint32_t now_ms)
{
  if (g_fault_clear_request != 0U)
  {
    g_fault_clear_request = 0U;
    debug_log("FAULT cleared by request (was %s, count=%lu)",
              fault_name(fault_first()), fault_count());
    fault_clear();
    s_fault_logged = false;
  }

  if (drv8323_is_awake() && !fault_active())
  {
    if (drv8323_take_nfault_event())
    {
      uint16_t f1 = 0U;
      uint16_t f2 = 0U;
      (void)drv8323_read(DRV_REG_FAULT1, &f1);
      (void)drv8323_read(DRV_REG_FAULT2, &f2);
      (void)fault_raise(FAULT_DRV_NFAULT, f1, f2);
    }
    else if (drv8323_is_configured() && ((now_ms % DRV_CHECK_PERIOD_MS) == 0U))
    {
      uint8_t addr = 0U;
      uint16_t val = 0U;
      uint16_t f1 = 0U;
      uint16_t f2 = 0U;
      s_drv_checks++;
      if (!drv8323_check_config(&addr, &val))
      {
        (void)fault_raise(FAULT_DRV_CONFIG_MISMATCH, addr, val);
      }
      else if (drv8323_read(DRV_REG_FAULT1, &f1) && drv8323_read(DRV_REG_FAULT2, &f2) &&
               ((f1 != 0U) || (f2 != 0U)))
      {
        (void)fault_raise(FAULT_DRV_FAULT_BITS, f1, f2);
      }
    }

    if (drv8323_spi_errors() != s_drv_spi_err_seen)
    {
      s_drv_spi_err_seen = drv8323_spi_errors();
      (void)fault_raise(FAULT_DRV_SPI, s_drv_spi_err_seen, 0U);
    }
  }

#if BRINGUP_STAGE >= 5
  if (pwm_take_break_event())
  {
    (void)fault_raise(FAULT_PWM_BREAK, pwm_break_events(), board_pwm_pins_read());
  }
#endif

  if (fault_active() && !s_fault_logged)
  {
    uint32_t c1;
    uint32_t c2;
    uint32_t t;
    fault_first_context(&c1, &c2, &t);
    debug_log("FAULT latched %s ctx1=0x%lX ctx2=0x%lX t=%lums", fault_name(fault_first()), c1, c2, t);
#if BRINGUP_STAGE >= 7
    debug_capture_trigger();   /* keep the 25 ms before the fault and the 25 ms after */
#endif
    if (drv8323_is_awake())
    {
      uint16_t f1 = 0U;
      uint16_t f2 = 0U;
      char text[DRV_FAULT_TEXT_LEN];
      bool ok = drv8323_read(DRV_REG_FAULT1, &f1) && drv8323_read(DRV_REG_FAULT2, &f2);
      drv8323_fault_text(f1, f2, text, sizeof(text));
      debug_log("FAULT drv f1=0x%03X f2=0x%03X %s nfault=%s", (unsigned)f1, (unsigned)f2,
                ok ? text : "READ_FAIL", drv8323_nfault_low() ? "LOW" : "high");
      drv8323_sleep();   /* disarms first: outputs off, then ENABLE low */
    }
#if BRINGUP_STAGE >= 5
    motor_disarm();
#endif
    debug_log("FAULT disarmed, DRV asleep. Set g_fault_clear_request=1 in the debugger to clear.");
    s_fault_logged = true;
  }
}
#endif

#if BRINGUP_STAGE >= 3
/* The DRV is awake exactly while VM_OK holds and no fault is latched
 * (plan Stage 3: wake on VM_OK; on VM_OK loss disarm, ENABLE low, release
 * PD2; Stage 4: configure after every wake). Runs after power_tick(). */
static void drv_tick(void)
{
  bool vm_ok = power_vm_ok();

  if (drv8323_sync())
  {
    debug_log("DRV sleep (ENABLE low after disarm); re-wakes if VM_OK and no fault");
  }
#if BRINGUP_STAGE == 5
  /* Stage 5: PWM into a sleeping DRV, so ENABLE is held low throughout. */
  bool may_wake = false;
#elif BRINGUP_STAGE >= 4
  bool may_wake = vm_ok && !fault_active();
#else
  bool may_wake = vm_ok;
#endif

  if (may_wake && !drv8323_is_awake())
  {
#if BRINGUP_STAGE >= 4
    uint32_t blanked_before = drv8323_nfault_blanked_events();
    bool nfault_released = drv8323_wake();
    drv_configure_after_wake(nfault_released, blanked_before);
#else
    bool nfault_released = drv8323_wake();
    debug_log("DRV wake nfault=%s", nfault_released ? "high" : "LOW");
#endif
#if BRINGUP_STAGE == 3
    stage3_test();
#endif
  }
  else if (!vm_ok && drv8323_is_awake())
  {
    drv8323_sleep();
    debug_log("DRV sleep (VM_OK lost) vm_mv=%lu", power_vm_mv());
  }
}
#endif

#if BRINGUP_STAGE == 4
static void stage4_tick(uint32_t now_ms)
{
  if ((now_ms % STAGE4_LOG_PERIOD_MS) != 0U)
  {
    return;
  }
  const char *st = drv8323_is_configured() ? "CONFIGURED" : (drv8323_is_awake() ? "AWAKE" : "ASLEEP");
  uint16_t f1 = 0U;
  uint16_t f2 = 0U;
  if (drv8323_is_awake())
  {
    (void)drv8323_read(DRV_REG_FAULT1, &f1);
    (void)drv8323_read(DRV_REG_FAULT2, &f2);
  }
  debug_log("DRV %s pwr=%s nfault=%s f1=0x%03X f2=0x%03X wakes=%lu cfg_ok=%lu cfg_fail=%lu checks=%lu nf_ev=%lu nf_ignored=%lu fault=%s",
            st, power_state_name(power_state()), drv8323_nfault_low() ? "LOW" : "high",
            (unsigned)f1, (unsigned)f2, s_drv_wakes, s_drv_cfg_ok, s_drv_cfg_fail, s_drv_checks,
            drv8323_nfault_events(), drv8323_nfault_blanked_events(), fault_name(fault_first()));
}
#endif

#if BRINGUP_STAGE == 3
static void stage3_tick(uint32_t now_ms)
{
  if ((now_ms % STAGE3_LOG_PERIOD_MS) != 0U)
  {
    return;
  }
  if (drv8323_is_awake())
  {
    uint16_t f1 = 0U;
    uint16_t f2 = 0U;
    char text[STAGE3_FAULT_TEXT_LEN];
    bool ok = drv8323_read(DRV_REG_FAULT1, &f1) && drv8323_read(DRV_REG_FAULT2, &f2);
    drv8323_fault_text(f1, f2, text, sizeof(text));
    debug_log("DRV awake pwr=%s vm_mv=%lu nfault=%s f1=0x%03X f2=0x%03X %s spi_err=%lu",
              power_state_name(power_state()), power_vm_mv(),
              drv8323_nfault_low() ? "LOW" : "high", (unsigned)f1, (unsigned)f2,
              ok ? text : "READ_FAIL", drv8323_spi_errors());
  }
  else
  {
    debug_log("DRV asleep pwr=%s vm_mv=%lu en=%u", power_state_name(power_state()),
              power_vm_mv(), power_motor_en() ? 1U : 0U);
  }
}
#endif

/* ---- Control interrupt monitor (Stage 7 onward) --------------------------- */

#if BRINGUP_STAGE >= 7
static void ctrl_tick(uint32_t now_ms)
{
  float means[PHASE_COUNT];
  bool ok;

  /* Heartbeat against real time (HAL_GetTick), so the main loop's catch-up
   * ticks can't trip it; stops with the core under the debugger. A stale
   * count latches a fault, and the fault response disarms (plan: Control-
   * loop heartbeat). */
  if (!cursense_heartbeat_ok(HAL_GetTick()))
  {
    (void)fault_raise(FAULT_CTRL_HEARTBEAT, cursense_late_count(), 0U);
  }

  if (cursense_offset_poll(means, &ok))
  {
    debug_log("CUR offsets A=%u B=%u C=%u counts (expect 2048 +/-250) %s",
              (unsigned)(means[PHASE_A] + 0.5f), (unsigned)(means[PHASE_B] + 0.5f),
              (unsigned)(means[PHASE_C] + 0.5f), ok ? "OK" : "FAIL");
    if (!ok)
    {
      (void)fault_raise(FAULT_CUR_OFFSET,
                        ((uint32_t)means[PHASE_A] << 16) | (uint32_t)means[PHASE_B], (uint32_t)means[PHASE_C]);
    }
  }

#if BRINGUP_STAGE >= 8
  uint32_t oc_phase;
  float oc_amps;
  if (ctrl_take_oc_event(&oc_phase, &oc_amps))
  {
    /* The interrupt has already disarmed; latch and log (ctx: phase 0-2 = A-C,
     * current in mA as a signed 32-bit value). */
    debug_log("OVERCURRENT phase %c %ld mA (trip %ld mA): disarmed in the control interrupt",
              (char)('A' + oc_phase), (long)(oc_amps * AMPS_TO_MA), (long)(s_trip_applied * AMPS_TO_MA));
    (void)fault_raise(FAULT_OVERCURRENT, oc_phase, (uint32_t)(int32_t)(oc_amps * AMPS_TO_MA));
  }
#endif

  (void)debug_capture_dump_tick();

#if BRINGUP_STAGE >= 7
  /* Current level and noise per phase over the last second (plan Stage 7
   * step 3; Stage 8 steps 3-5), control interrupt time and load. Silent
   * during the comms test. */
  if (!COMMS_TEST && ((now_ms % CUR_LOG_PERIOD_MS) == 0U))
  {
    cursense_stats_t st;
    float off[PHASE_COUNT];
    int32_t ma[PHASE_COUNT];
    cursense_stats_take(&st);
    cursense_offsets(off);
    if (st.samples == 0U)
    {
      debug_log("CUR no samples (control interrupt not running)");
      return;
    }
    for (uint32_t ph = 0U; ph < PHASE_COUNT; ph++)
    {
      ma[ph] = (int32_t)((st.mean[ph] - off[ph]) * CURSENSE_AMPS_PER_COUNT * AMPS_TO_MA);
    }
    /* Interrupt time over this second: mean and max; load from the max. */
    uint32_t isr_mean = st.isr_mean_cycles;
    uint32_t isr = st.isr_max_cycles;
    uint32_t worst = cursense_isr_max_cycles();
    uint32_t load_pm = (isr * 1000UL) / CTRL_PERIOD_CYCLES;   /* permille */
    uint32_t rms10[PHASE_COUNT];
    for (uint32_t ph = 0U; ph < PHASE_COUNT; ph++)
    {
      rms10[ph] = (uint32_t)(st.rms[ph] * 10.0f + 0.5f);       /* tenths of a count */
    }
    debug_log("CUR mA=%ld/%ld/%ld sum=%ld pp=%u/%u/%u rms=%lu.%lu/%lu.%lu/%lu.%lu off=%u/%u/%u valid=%u n=%lu",
              (long)ma[PHASE_A], (long)ma[PHASE_B], (long)ma[PHASE_C],
              (long)(ma[PHASE_A] + ma[PHASE_B] + ma[PHASE_C]),
              (unsigned)(st.max[PHASE_A] - st.min[PHASE_A]), (unsigned)(st.max[PHASE_B] - st.min[PHASE_B]),
              (unsigned)(st.max[PHASE_C] - st.min[PHASE_C]),
              rms10[PHASE_A] / 10UL, rms10[PHASE_A] % 10UL, rms10[PHASE_B] / 10UL, rms10[PHASE_B] % 10UL,
              rms10[PHASE_C] / 10UL, rms10[PHASE_C] % 10UL,
              (unsigned)(off[PHASE_A] + 0.5f), (unsigned)(off[PHASE_B] + 0.5f), (unsigned)(off[PHASE_C] + 0.5f),
              cursense_offsets_valid() ? 1U : 0U, st.samples);
    debug_log("CTRL isr mean=%lu.%luus max=%lu.%luus (1 s) worst=%lu.%luus (since boot) load=%lu.%lu%% late=%lu",
              isr_mean / DEBUG_CYCLES_PER_US, (isr_mean % DEBUG_CYCLES_PER_US) * 10UL / DEBUG_CYCLES_PER_US,
              isr / DEBUG_CYCLES_PER_US, (isr % DEBUG_CYCLES_PER_US) * 10UL / DEBUG_CYCLES_PER_US,
              worst / DEBUG_CYCLES_PER_US, (worst % DEBUG_CYCLES_PER_US) * 10UL / DEBUG_CYCLES_PER_US,
              load_pm / 10UL, load_pm % 10UL, cursense_late_count());
  }
#endif
}
#endif

/* ---- Stages 8-9 test mode: voltage vector (fixed angle, then forced spin) -- */

#if APP_VEC_MODE
static float vec_target(void)
{
  float v = g_vec_v;
  if (!(v >= 0.0f))   /* also catches NaN */
  {
    return 0.0f;
  }
  return (v > VEC_V_MAX) ? VEC_V_MAX : v;
}

static float vec_theta(void)
{
  float deg = g_vec_deg;
  if (!(deg >= -360.0f) || !(deg <= 360.0f))   /* NaN or out of range: 0 */
  {
    deg = 0.0f;
  }
  return deg * DEG_TO_RAD;
}

static float vec_shift(void)
{
  float s = g_duty_shift;
  if (!(s >= 0.0f))
  {
    return 0.0f;
  }
  return (s > VEC_SHIFT_MAX) ? VEC_SHIFT_MAX : s;
}

/* Multi-turn encoder position, counts. */
static int32_t vec_enc_position(void)
{
  mt6701_snapshot_t s;
  mt6701_snapshot(&s);
  return ((int32_t)s.turns * ENC_COUNTS_PER_TURN) + (int32_t)s.angle_counts;
}

#if BRINGUP_STAGE == 8
static void vec_move_start(float from_deg, uint32_t wait_ms)
{
  s_move_from = vec_enc_position();
  s_move_from_deg = from_deg;
  s_move_ms = 0U;
  s_move_wait = wait_ms;
  s_move_pending = true;
}

/* Log how far the shaft moved after an arm or an angle change (plan Stage 8
 * step 6, "watch the rotor snap", measured by the encoder). */
static void vec_move_tick(void)
{
  if (!s_move_pending || (++s_move_ms < s_move_wait))
  {
    return;
  }
  s_move_pending = false;
  int32_t d = vec_enc_position() - s_move_from;
  int32_t tenths = (int32_t)((float)d * ENC_COUNTS_TO_MDEG / 100.0f);   /* 0.1 deg mech */
  int32_t mag = (tenths < 0) ? -tenths : tenths;
  if (s_move_from_deg < 0.0f)
  {
    debug_log("MOVE arm -> %ld deg: shaft moved %ld counts = %c%ld.%ld deg mech (0 if already aligned)",
              (long)s_move_deg, (long)d, (tenths < 0) ? '-' : '+', (long)(mag / 10), (long)(mag % 10));
  }
  else
  {
    debug_log("MOVE %ld -> %ld deg elec: shaft moved %ld counts = %c%ld.%ld deg mech (120 elec = 496 = 10.9 deg)",
              (long)s_move_from_deg, (long)s_move_deg, (long)d, (tenths < 0) ? '-' : '+',
              (long)(mag / 10), (long)(mag % 10));
  }
}
#endif

#if BRINGUP_STAGE >= 9
/* Stage 9 spin target (Hz electrical, sign = direction) and slew, bounded. */
static float spin_target(void)
{
  float f = g_spin_hz;
  if (!(f >= -SPIN_HZ_MAX) || !(f <= SPIN_HZ_MAX))   /* NaN or beyond the limit */
  {
    return (f > 0.0f) ? SPIN_HZ_MAX : ((f < 0.0f) ? -SPIN_HZ_MAX : 0.0f);
  }
  return f;
}

static float spin_accel(void)
{
  float a = g_spin_accel;
  if (!(a > 0.0f))
  {
    return SPIN_ACCEL_INIT;
  }
  return (a > SPIN_ACCEL_MAX) ? SPIN_ACCEL_MAX : a;
}

/* Signed value with two decimals, for the log. */
static const char *fmt2(char *buf, size_t len, float x)
{
  int32_t v = (int32_t)((x * 100.0f) + ((x < 0.0f) ? -0.5f : 0.5f));
  uint32_t m = (uint32_t)((v < 0) ? -v : v);
  (void)snprintf(buf, len, "%c%lu.%02lu", (v < 0) ? '-' : '+', m / 100UL, m % 100UL);
  return buf;
}
#endif

static void vec_arm(bool align)
{
  const char *reason;
  if (!s_foc_ok)
  {
    debug_log("VEC arm refused: CORDIC not running (CubeMX: activate CORDIC)");
    return;
  }
  s_vec_v = 0.0f;                                  /* zero voltage vector, then ramp */
  ctrl_set_vector(0.0f, vec_theta(), 0.0f);
#if BRINGUP_STAGE >= 9
  ctrl_spin_reset();                               /* forced angle at 0 deg, not moving */
#endif
#if BRINGUP_STAGE >= 11
  ctrl_set_foc(false);
  s_foc_on = false;
#endif
#if BRINGUP_STAGE >= 10
  ctrl_set_vector(0.0f, 0.0f, 0.0f);               /* alignment starts at 0 deg */
  ctrl_hold_angle(align);
  s_al_phase = ALIGN_IDLE;
#else
  (void)align;
#endif
  pwm_zero_command();
  if (motor_can_arm(&reason))
  {
#if BRINGUP_STAGE == 8
    debug_log("VEC armed from 0 V; ramping to %lu mV at %ld deg, trip %lu mA",
              (uint32_t)(vec_target() * 1000.0f), (long)g_vec_deg, (uint32_t)(s_trip_applied * 1000.0f));
    s_move_deg = vec_theta() / DEG_TO_RAD;
    vec_move_start(-1.0f, VEC_ARM_SETTLE_MS);
#else
#if BRINGUP_STAGE >= 10
    if (align)
    {
      s_al_phase = ALIGN_START;
      s_al_timer = 0U;
      s_al_index = 0U;
      s_al_theta_deg = 0.0f;
      debug_log("ALIGN armed: %lu mV d-axis, %u points (0..720..0 deg elec, %u deg steps), about 25 s",
                (uint32_t)(vec_target() * 1000.0f), ALIGN_POINTS, (unsigned)ALIGN_STEP_DEG);
      return;
    }
#endif
    char b[16];
    s_align_ms = SPIN_ALIGN_MS;
    s_spin_pos0 = vec_enc_position();
    s_spin_cyc0 = 0.0f;
    debug_log("SPIN armed: %lu mV at 0 deg for %u ms (align), then to %s Hz at %lu mHz/s; trip %lu mA",
              (uint32_t)(vec_target() * 1000.0f), SPIN_ALIGN_MS, fmt2(b, sizeof(b), spin_target()),
              (uint32_t)(spin_accel() * 1000.0f), (uint32_t)(s_trip_applied * 1000.0f));
#endif
  }
  else
  {
#if BRINGUP_STAGE >= 10
    ctrl_hold_angle(false);
#endif
    debug_log("VEC arm refused: %s", reason);
  }
}

#if BRINGUP_STAGE == 8

/* Plan Stage 8 step 7: tau from the phase A step response in the capture,
 * L = tau x R. Assumes the vector is at 0 deg (phase A carries the full
 * current, the rotor is aligned so there's no torque). */
static void vec_lstep_fit(void)
{
  float off[PHASE_COUNT];
  uint16_t s[3];
  float i0 = 0.0f;
  float i1 = 0.0f;
  cursense_offsets(off);

  for (uint32_t n = DEBUG_CAPTURE_TRIGGER - LFIT_AVG_SAMPLES; n < DEBUG_CAPTURE_TRIGGER; n++)
  {
    (void)debug_capture_get(n, s);
    i0 += ((float)s[PHASE_A] - off[PHASE_A]) * CURSENSE_AMPS_PER_COUNT;
  }
  for (uint32_t n = DEBUG_CAPTURE_LEN - LFIT_AVG_SAMPLES; n < DEBUG_CAPTURE_LEN; n++)
  {
    (void)debug_capture_get(n, s);
    i1 += ((float)s[PHASE_A] - off[PHASE_A]) * CURSENSE_AMPS_PER_COUNT;
  }
  i0 /= (float)LFIT_AVG_SAMPLES;
  i1 /= (float)LFIT_AVG_SAMPLES;

  float target = i0 + (LFIT_TAU_FRACTION * (i1 - i0));
  float prev = i0;
  float t_cross = -1.0f;
  for (uint32_t n = DEBUG_CAPTURE_TRIGGER; n < DEBUG_CAPTURE_LEN; n++)
  {
    (void)debug_capture_get(n, s);
    float i = ((float)s[PHASE_A] - off[PHASE_A]) * CURSENSE_AMPS_PER_COUNT;
    if (i >= target)
    {
      float frac = ((i - prev) > 0.0f) ? ((target - prev) / (i - prev)) : 0.0f;
      t_cross = ((float)n - 1.0f + frac) - LFIT_STEP_AT;
      break;
    }
    prev = i;
  }

  if ((t_cross <= 0.0f) || ((i1 - i0) < 0.05f))
  {
    debug_log("LFIT failed: I %ld -> %ld mA, no clean 63%% crossing (vector at 0 deg? rotor aligned?)",
              (long)(i0 * AMPS_TO_MA), (long)(i1 * AMPS_TO_MA));
    return;
  }
  float tau = t_cross * CTRL_SAMPLE_S;
  float l_h = tau * MOTOR_R_PHASE_OHM;
  float r_dc = (i1 > 0.01f) ? (g_vstep_v / i1) : 0.0f;
  debug_log("LFIT I %ld -> %ld mA, tau=%lu us, L=%lu uH (R=%lu mOhm from motor_params; V/I=%lu mOhm)",
            (long)(i0 * AMPS_TO_MA), (long)(i1 * AMPS_TO_MA), (uint32_t)(tau * 1e6f),
            (uint32_t)(l_h * 1e6f), (uint32_t)(MOTOR_R_PHASE_OHM * 1000.0f), (uint32_t)(r_dc * 1000.0f));
}
#endif

#if BRINGUP_STAGE >= 10
/* ---- Stage 10: alignment and angle tracking ---- */

static float wrap180(float deg)
{
  while (deg >= 180.0f)
  {
    deg -= 360.0f;
  }
  while (deg < -180.0f)
  {
    deg += 360.0f;
  }
  return deg;
}

/* DIR x 11 x theta_m in electrical degrees [0, 360), no offset, computed in
 * counts so it stays exact. */
static float enc_elec_deg(int32_t pos, int32_t dir)
{
  int32_t c = pos % ENC_COUNTS_PER_TURN;
  if (c < 0)
  {
    c += ENC_COUNTS_PER_TURN;
  }
  int32_t x = (c * (int32_t)MOTOR_POLE_PAIRS) % ENC_COUNTS_PER_TURN;
  if (dir < 0)
  {
    x = (ENC_COUNTS_PER_TURN - x) % ENC_COUNTS_PER_TURN;
  }
  return (float)x * DEG_PER_COUNT_ELEC;
}

/* Commanded angle of point i: 0, 60 ... 720 forward, then 660 ... 0. */
static float align_point_deg(uint32_t i)
{
  uint32_t k = (i < ALIGN_FWD_POINTS) ? i : ((ALIGN_POINTS - 1U) - i);
  return (float)k * ALIGN_STEP_DEG;
}

/* Signed value with four decimals, for the radian offset. */
static const char *fmt4(char *buf, size_t len, float x)
{
  int32_t v = (int32_t)((x * 10000.0f) + ((x < 0.0f) ? -0.5f : 0.5f));
  uint32_t m = (uint32_t)((v < 0) ? -v : v);
  (void)snprintf(buf, len, "%s%lu.%04lu", (v < 0) ? "-" : "", m / 10000UL, m % 10000UL);
  return buf;
}

static void align_finish(void)
{
  char b1[16];
  char b2[16];
  char b3[16];
  char b4[16];
  float span_cycles = ((float)(ALIGN_FWD_POINTS - 1U) * ALIGN_STEP_DEG) / 360.0f;
  float cpc_f = (float)(s_al_pos[ALIGN_FWD_POINTS - 1U] - s_al_pos[0]) / span_cycles;
  float cpc_b = (float)(s_al_pos[ALIGN_POINTS - 1U] - s_al_pos[ALIGN_FWD_POINTS - 1U]) / -span_cycles;
  int32_t dir = (cpc_f > 0.0f) ? 1 : -1;
  float lo = ALIGN_CPC_NOMINAL * (1.0f - ALIGN_CPC_TOL);
  float hi = ALIGN_CPC_NOMINAL * (1.0f + ALIGN_CPC_TOL);
  float af = (float)dir * cpc_f;
  float ab = (float)dir * cpc_b;
  debug_log("ALIGN cnt/ecyc fwd=%ld bwd=%ld (expect +/-1489.5) -> dir=%+ld (motor_params %+d)",
            (long)cpc_f, (long)cpc_b, (long)dir, MOTOR_ENC_DIR);
  if (!(af > lo) || !(af < hi) || !(ab > lo) || !(ab < hi))
  {
    debug_log("ALIGN FAIL: rotor did not follow the steps (stuck, slipping, or too little torque: raise g_vec_v)");
    return;
  }

  /* Offsets relative to the first point, so the average can't straddle
   * the +/-180 deg wrap. */
  float off0 = wrap180(enc_elec_deg(s_al_pos[0], dir) - align_point_deg(0U));
  float rel[ALIGN_POINTS];
  float sum = 0.0f;
  float sum_f = 0.0f;
  float sum_b = 0.0f;
  for (uint32_t i = 0U; i < ALIGN_POINTS; i++)
  {
    float off = wrap180(enc_elec_deg(s_al_pos[i], dir) - align_point_deg(i));
    rel[i] = wrap180(off - off0);
    sum += rel[i];
    if (i < ALIGN_FWD_POINTS)
    {
      sum_f += rel[i];
    }
    else
    {
      sum_b += rel[i];
    }
  }
  float mean = sum / (float)ALIGN_POINTS;
  float dev = 0.0f;
  for (uint32_t i = 0U; i < ALIGN_POINTS; i++)
  {
    float d = rel[i] - mean;
    d = (d < 0.0f) ? -d : d;
    dev = (d > dev) ? d : dev;
  }
  float off_deg = wrap180(off0 + mean);
  if (off_deg < 0.0f)
  {
    off_deg += 360.0f;
  }
  float fwd = wrap180(off0 + (sum_f / (float)ALIGN_FWD_POINTS));
  float bwd = wrap180(off0 + (sum_b / (float)(ALIGN_POINTS - ALIGN_FWD_POINTS)));
  s_al_valid = true;
  s_al_dir = dir;
  s_al_offset_deg = off_deg;
  debug_log("ALIGN offset=%s deg elec (fwd %s, bwd %s; max point dev %s)",
            fmt2(b1, sizeof(b1), off_deg), fmt2(b2, sizeof(b2), fwd), fmt2(b3, sizeof(b3), bwd),
            fmt2(b4, sizeof(b4), dev));
  debug_log("ALIGN result: MOTOR_ENC_DIR %ld, MOTOR_ENC_OFFSET_RAD %sf (used by TRACK until reset)",
            (long)dir, fmt4(b1, sizeof(b1), off_deg * DEG_TO_RAD));
}

/* One alignment step per 1 ms tick, while armed. */
static void align_tick(void)
{
  switch (s_al_phase)
  {
    case ALIGN_START:
      if (++s_al_timer >= ALIGN_START_MS)
      {
        s_al_phase = ALIGN_MEASURE;
        s_al_pos_sum = 0;
        s_al_pos_n = 0U;
      }
      break;
    case ALIGN_MOVE:
    {
      float target = align_point_deg(s_al_index);
      float d = target - s_al_theta_deg;
      if ((d > ALIGN_SLEW_DEG_PER_MS) || (d < -ALIGN_SLEW_DEG_PER_MS))
      {
        s_al_theta_deg += (d > 0.0f) ? ALIGN_SLEW_DEG_PER_MS : -ALIGN_SLEW_DEG_PER_MS;
      }
      else
      {
        s_al_theta_deg = target;
        s_al_phase = ALIGN_SETTLE;
        s_al_timer = 0U;
      }
      break;
    }
    case ALIGN_SETTLE:
      if (++s_al_timer >= ALIGN_SETTLE_MS)
      {
        s_al_phase = ALIGN_MEASURE;
        s_al_pos_sum = 0;
        s_al_pos_n = 0U;
      }
      break;
    case ALIGN_MEASURE:
      s_al_pos_sum += vec_enc_position();
      if (++s_al_pos_n >= ALIGN_AVG_MS)
      {
        float avg = (float)s_al_pos_sum / (float)s_al_pos_n;
        int32_t pos = (int32_t)(avg + ((avg < 0.0f) ? -0.5f : 0.5f));
        char b[16];
        s_al_pos[s_al_index] = pos;
        debug_log("ALIGN %2lu/%u cmd=%ld deg pos=%ld off=%s deg (dir %+d)", s_al_index + 1U, ALIGN_POINTS,
                  (long)align_point_deg(s_al_index), (long)pos,
                  fmt2(b, sizeof(b), wrap180(enc_elec_deg(pos, MOTOR_ENC_DIR) - align_point_deg(s_al_index))),
                  MOTOR_ENC_DIR);
        if (++s_al_index >= ALIGN_POINTS)
        {
          motor_disarm();
          ctrl_hold_angle(false);
          s_al_phase = ALIGN_IDLE;
          debug_log("ALIGN steps done, disarmed");
          align_finish();
        }
        else
        {
          s_al_phase = ALIGN_MOVE;
        }
      }
      break;
    default:
      break;
  }
}

/* Encoder electrical angle against the forced angle during the open-loop
 * spin (plan Stage 10 step 2). Uses this boot's alignment result if there
 * is one, otherwise motor_params.h. The error is the rotor's load angle
 * plus any offset error: it should be steady at a steady speed. */
static void track_tick(void)
{
  mt6701_snapshot_t e;
  float f_now;
  float cyc;
  mt6701_snapshot(&e);
  ctrl_spin_state(&f_now, &cyc);
  float frac = cyc - (float)(int32_t)cyc;
  if (frac < 0.0f)
  {
    frac += 1.0f;
  }
  int32_t dir = s_al_valid ? s_al_dir : MOTOR_ENC_DIR;
  float off = s_al_valid ? s_al_offset_deg : (MOTOR_ENC_OFFSET_RAD / DEG_TO_RAD);
  float err = wrap180(enc_elec_deg((int32_t)e.angle_counts, dir) - off - (frac * 360.0f));
  if (s_tr_n == 0U)
  {
    s_tr_min = err;
    s_tr_max = err;
    s_tr_sum = 0.0f;
  }
  s_tr_sum += err;
  s_tr_min = (err < s_tr_min) ? err : s_tr_min;
  s_tr_max = (err > s_tr_max) ? err : s_tr_max;
  s_tr_n++;
}
#endif

#if BRINGUP_STAGE >= 11
/* ---- Stage 11: voltage-mode FOC ---- */

/* THROTTLE1 position as a Vq command (0 inside the dead band). */
static float foc_pot_vq(void)
{
  float x = ((float)aux_throttle_permille(0U) - FOC_POT_MID_PERMILLE) / FOC_POT_MID_PERMILLE;
  float m = (x < 0.0f) ? -x : x;
  if (m <= FOC_POT_DEADBAND)
  {
    return 0.0f;
  }
  m = (m - FOC_POT_DEADBAND) / (1.0f - FOC_POT_DEADBAND);
  m = (m > 1.0f) ? 1.0f : m;
  return ((x < 0.0f) ? -m : m) * FOC_VQ_MAX;
}

static float foc_vq_target(void)
{
  if (g_vq_pot != 0U)
  {
    return foc_pot_vq();
  }
  float v = g_vq;
  if (!(v >= -FOC_VQ_MAX) || !(v <= FOC_VQ_MAX))   /* NaN or beyond the limit */
  {
    return (v > 0.0f) ? FOC_VQ_MAX : ((v < 0.0f) ? -FOC_VQ_MAX : 0.0f);
  }
  return v;
}

static void foc_arm(void)
{
  const char *reason;
  char b[16];
  if (!s_foc_ok)
  {
    debug_log("FOC arm refused: CORDIC not running (CubeMX: activate CORDIC)");
    return;
  }
  if ((g_vq_pot != 0U) && (foc_pot_vq() != 0.0f))
  {
    debug_log("FOC arm refused: THROTTLE1 not at the middle (%lu permille; dead band %lu-%lu)",
              aux_throttle_permille(0U),
              (uint32_t)(FOC_POT_MID_PERMILLE * (1.0f - FOC_POT_DEADBAND)),
              (uint32_t)(FOC_POT_MID_PERMILLE * (1.0f + FOC_POT_DEADBAND)));
    return;
  }
  s_vq_applied = 0.0f;                     /* zero command, then ramp */
  ctrl_set_vq(0.0f);
  ctrl_hold_angle(false);
  ctrl_set_vector(0.0f, 0.0f, 0.0f);
  ctrl_set_foc(true);
  pwm_zero_command();
  if (motor_can_arm(&reason))
  {
    s_foc_on = true;
    s_foc_pos0 = vec_enc_position();
    s_vm_max_mv = 0U;
#if BRINGUP_STAGE >= 12
    s_foc_ms = 0U;
    s_cap_auto_at = g_cap_auto_ms;
    s_cap_auto_dump = false;
#endif
    debug_log("FOC armed: Vd 0, Vq from %s (now %s V), ramp 1 V/s, trip %lu mA. Don't pause the debugger while armed",
              (g_vq_pot != 0U) ? "THROTTLE1" : "g_vq", fmt2(b, sizeof(b), foc_vq_target()),
              (uint32_t)(s_trip_applied * 1000.0f));
  }
  else
  {
    ctrl_set_foc(false);
    debug_log("FOC arm refused: %s", reason);
  }
}

/* While in FOC mode: ramp Vq, log once a second. */
static void foc_tick(uint32_t now_ms)
{
  if (pwm_is_armed())
  {
    float d = foc_vq_target() - s_vq_applied;
    if (d > FOC_VQ_SLEW_V_PER_MS)
    {
      d = FOC_VQ_SLEW_V_PER_MS;
    }
    else if (d < -FOC_VQ_SLEW_V_PER_MS)
    {
      d = -FOC_VQ_SLEW_V_PER_MS;
    }
    s_vq_applied += d;
    ctrl_set_vq(s_vq_applied);
#if BRINGUP_STAGE >= 12
    s_foc_ms++;
    if ((s_cap_auto_at != 0U) && (s_foc_ms == s_cap_auto_at))
    {
      (void)debug_capture_rearm();
    }
    else if ((s_cap_auto_at != 0U) && (s_foc_ms == (s_cap_auto_at + DQ_CAP_REFILL_MS)))
    {
      debug_capture_trigger();
      s_cap_auto_dump = true;
      debug_log("CAP auto-triggered %lu ms after the FOC arm; dump follows", s_foc_ms);
    }
#endif
  }
#if BRINGUP_STAGE >= 12
  if (s_cap_auto_dump && debug_capture_done())
  {
    s_cap_auto_dump = false;
    (void)debug_capture_dump_start();
  }
#endif

  if (!COMMS_TEST && ((now_ms % VEC_LOG_PERIOD_MS) == 0U))
  {
    char b1[16];
    char b2[16];
    mt6701_snapshot_t e;
    cursense_snapshot_t cs;
    mt6701_snapshot(&e);
    cursense_snapshot(&cs);
    int32_t pos = vec_enc_position();
    int32_t dpos = pos - s_foc_pos0;
    s_foc_pos0 = pos;
    /* Positive = the direction positive Vq drives (electrical angle rising). */
    float rpm = (float)MOTOR_ENC_DIR * ((float)dpos / (float)ENC_COUNTS_PER_TURN) * 60000.0f / (float)VEC_LOG_PERIOD_MS;
    float pll = (float)MOTOR_ENC_DIR * e.speed_rad_s * RAD_S_TO_RPM;
    float i_alpha = cs.amps[PHASE_A];
    float i_beta = (cs.amps[PHASE_B] - cs.amps[PHASE_C]) * 0.577350269f;   /* 1/sqrt(3) */
    float i_amp = foc_sqrtf((i_alpha * i_alpha) + (i_beta * i_beta));
    debug_log("FOC Vq=%sV tgt=%s rpm=%ld pll=%ld noload=%ld Iamp=%lumA vm=%lu max=%lumV %s",
              fmt2(b1, sizeof(b1), s_vq_applied), fmt2(b2, sizeof(b2), foc_vq_target()),
              (long)rpm, (long)pll, (long)(s_vq_applied * FOC_RPM_PER_VQ), (uint32_t)(i_amp * AMPS_TO_MA),
              power_vm_mv(), s_vm_max_mv, fault_name(fault_first()));
    s_vm_max_mv = 0U;
  }

  if (!pwm_is_armed())
  {
    s_foc_on = false;
    s_vq_applied = 0.0f;
    ctrl_set_vq(0.0f);
    ctrl_set_foc(false);
    debug_log("FOC off: disarmed");
  }
}
#endif

#if BRINGUP_STAGE >= 12
/* Once a second: Id and Iq over the last second (mean, min, max). Steady DC
 * with little spread = the angle and transforms are right; ripple at the
 * electrical frequency would show as a wide min-max. */
static void dq_tick(uint32_t now_ms)
{
  static uint32_t s_cap_mode = 2U;      /* forces the first label update */
  uint32_t mode = (g_cap_dq != 0U) ? 1U : 0U;
  if (mode != s_cap_mode)
  {
    s_cap_mode = mode;
    ctrl_capture_dq(mode != 0U);
    debug_capture_label((mode != 0U) ? DQ_CAP_LABEL : DQ_RAW_LABEL);
  }
  if (!COMMS_TEST && ((now_ms % VEC_LOG_PERIOD_MS) == 0U))
  {
    ctrl_dq_stats_t st;
    ctrl_dq_stats(&st);
    debug_log("DQ id=%ld iq=%ld mA (id %ld..%ld, iq %ld..%ld) n=%lu bad=%lu",
              (long)(st.id_mean * AMPS_TO_MA), (long)(st.iq_mean * AMPS_TO_MA),
              (long)(st.id_min * AMPS_TO_MA), (long)(st.id_max * AMPS_TO_MA),
              (long)(st.iq_min * AMPS_TO_MA), (long)(st.iq_max * AMPS_TO_MA), st.n, st.bad);
  }
}
#endif

static void vec_command(uint32_t cmd)
{
  switch (cmd)
  {
    case APP_CMD_ARM:
#if BRINGUP_STAGE >= 11
      if (pwm_is_armed())
      {
        debug_log("ARM refused: already armed; g_app_cmd = 2 first");
        break;
      }
#endif
      vec_arm(false);
      break;
#if BRINGUP_STAGE >= 11
    case APP_CMD_FOC:
      if (pwm_is_armed())
      {
        debug_log("FOC refused: already armed; g_app_cmd = 2 first");
      }
      else
      {
        foc_arm();
      }
      break;
#endif
#if BRINGUP_STAGE >= 10
    case APP_CMD_ALIGN:
      if (pwm_is_armed())
      {
        debug_log("ALIGN refused: armed; g_app_cmd = 2 first");
      }
      else
      {
        vec_arm(true);
      }
      break;
#endif
    case APP_CMD_DISARM:
      motor_disarm();
      s_lstep_active = false;
      debug_log("VEC disarmed by command: moe=%u pins=0x%02lX", pwm_is_armed() ? 1U : 0U, board_pwm_pins_read());
      break;
    case APP_CMD_SW_BREAK:
      pwm_software_break();
      s_lstep_active = false;
      debug_log("PWM software break: moe=%u pins=0x%02lX", pwm_is_armed() ? 1U : 0U, board_pwm_pins_read());
      break;
    case APP_CMD_CAPTURE:
      debug_capture_trigger();
      debug_log("CAP triggered; g_app_cmd = 7 dumps it once full (25 ms)");
      break;
    case APP_CMD_DUMP:
      if (!debug_capture_dump_start())
      {
        debug_log("CAP dump refused: no finished capture (g_app_cmd = 6 first)");
      }
      break;
#if BRINGUP_STAGE == 8
    case APP_CMD_LSTEP:
    {
      float v1 = g_vstep_v;
      float v0 = vec_target();
      float settle = s_vec_v - v0;
      if (!pwm_is_armed())
      {
        debug_log("LSTEP refused: not armed (g_app_cmd = 1 first)");
      }
      else if ((settle > VEC_SLEW_V_PER_MS) || (settle < -VEC_SLEW_V_PER_MS))
      {
        debug_log("LSTEP refused: vector still ramping; wait 1 s");
      }
      else if (!(v1 > v0) || (v1 > VEC_V_MAX))
      {
        debug_log("LSTEP refused: g_vstep_v must be above g_vec_v (%lu mV) and at most %lu mV",
                  (uint32_t)(v0 * 1000.0f), (uint32_t)(VEC_V_MAX * 1000.0f));
      }
      else if ((s_lstep_wait != 0U) || s_lstep_active)
      {
        debug_log("LSTEP refused: a step is already running");
      }
      else if (debug_capture_done() && !debug_capture_rearm())
      {
        debug_log("LSTEP refused: a capture dump is still printing; wait for CAP end");
      }
      else
      {
        /* Capture re-armed (any earlier recording discarded); step after
         * enough fresh samples for the pre-step average. */
        s_lstep_v1 = v1;
        s_lstep_wait = LSTEP_PRETRIGGER_MS;
        debug_log("LSTEP %lu -> %lu mV at %ld deg in %u ms; fit follows 25 ms later",
                  (uint32_t)(v0 * 1000.0f), (uint32_t)(v1 * 1000.0f), (long)g_vec_deg, LSTEP_PRETRIGGER_MS);
      }
      break;
    }
#endif
    default:
      debug_log("CMD %lu unknown", cmd);
      break;
  }
}

static void vec_tick(uint32_t now_ms)
{
  float t = ctrl_set_trip(g_oc_trip);
  if (t != s_trip_applied)
  {
    s_trip_applied = t;
    debug_log("OC trip now %lu mA (asked %ld mA; range %lu-%lu mA)", (uint32_t)(t * 1000.0f),
              (long)(g_oc_trip * 1000.0f), (uint32_t)(MOTOR_OC_TRIP_MIN_A * 1000.0f),
              (uint32_t)(MOTOR_OC_TRIP_STAGE_MAX_A * 1000.0f));
  }

  uint32_t cmd = g_app_cmd;
  if (cmd != APP_CMD_NONE)
  {
    g_app_cmd = APP_CMD_NONE;
    vec_command(cmd);
  }

#if BRINGUP_STAGE >= 12
  dq_tick(now_ms);
#endif
#if BRINGUP_STAGE >= 11
  if (ctrl_take_enc_event())
  {
    /* The interrupt already disarmed; latch the fault like enc_monitor does. */
    (void)fault_raise(FAULT_ENCODER, 0xFFU, 0U);
    debug_log("FOC disarmed by the control interrupt: bad encoder sample");
  }
  uint32_t vm_now = power_vm_mv();
  s_vm_max_mv = (vm_now > s_vm_max_mv) ? vm_now : s_vm_max_mv;
  if (s_foc_on)
  {
    foc_tick(now_ms);
    return;
  }
#endif

#if BRINGUP_STAGE == 8
  if (pwm_is_armed() && (s_lstep_wait != 0U) && (--s_lstep_wait == 0U))
  {
    s_vec_v = s_lstep_v1;
    g_vec_v = s_lstep_v1;          /* hold the new level after the step */
    ctrl_vector_step(s_lstep_v1);  /* applied and captured in the same control interrupt */
    s_lstep_active = true;
  }
#endif

  if (pwm_is_armed())
  {
    float target = vec_target();
    float step = target - s_vec_v;
    if (step > VEC_SLEW_V_PER_MS)
    {
      step = VEC_SLEW_V_PER_MS;
    }
    else if (step < -VEC_SLEW_V_PER_MS)
    {
      step = -VEC_SLEW_V_PER_MS;
    }
    s_vec_v += step;
#if BRINGUP_STAGE >= 10
    bool aligning = (s_al_phase != ALIGN_IDLE);
    ctrl_set_vector(s_vec_v, aligning ? (s_al_theta_deg * DEG_TO_RAD) : vec_theta(), vec_shift());
#else
    ctrl_set_vector(s_vec_v, vec_theta(), vec_shift());
#endif

#if BRINGUP_STAGE == 8
    float deg = vec_theta() / DEG_TO_RAD;
    if (deg != s_move_deg)
    {
      vec_move_start(s_move_deg, VEC_MOVE_SETTLE_MS);
      s_move_deg = deg;
    }
    vec_move_tick();
#else
#if BRINGUP_STAGE >= 10
    if (aligning)
    {
      align_tick();
    }
    else
#endif
    {
      /* Hold at 0 deg until aligned, then spin toward the target (the
       * interrupt slews the speed at spin_accel()). */
      float f_t = 0.0f;
      if (s_align_ms > 0U)
      {
        s_align_ms--;
      }
      else
      {
        f_t = spin_target();
      }
      ctrl_set_spin(f_t, spin_accel());
#if BRINGUP_STAGE >= 10
      track_tick();
#endif
    }
#endif
  }
  else
  {
    s_vec_v = 0.0f;
#if BRINGUP_STAGE >= 10
    if (s_al_phase != ALIGN_IDLE)
    {
      s_al_phase = ALIGN_IDLE;
      ctrl_hold_angle(false);
      debug_log("ALIGN aborted at point %lu: disarmed (command, trip or fault)", s_al_index + 1U);
    }
#endif
#if BRINGUP_STAGE == 8
    s_move_pending = false;
    s_lstep_wait = 0U;              /* a disarm or trip cancels a pending step */
#endif
  }

#if BRINGUP_STAGE == 8
  if (s_lstep_active && debug_capture_done())
  {
    s_lstep_active = false;
    vec_lstep_fit();
  }
#endif

#if BRINGUP_STAGE >= 9
  /* Encoder against the forced angle over the last second (plan Stage 9
   * step 3): counts per electrical cycle = 16384 / pole pairs, signed by
   * the direction; current amplitude from the latest sample (Clarke). */
#if BRINGUP_STAGE >= 10
  if (!COMMS_TEST && (s_tr_n > 0U) && ((now_ms % VEC_LOG_PERIOD_MS) == 0U))
  {
    char t1[16];
    char t2[16];
    char t3[16];
    debug_log("TRACK err=%s deg elec (min %s max %s) over %lu ms, %s offset",
              fmt2(t1, sizeof(t1), s_tr_sum / (float)s_tr_n), fmt2(t2, sizeof(t2), s_tr_min),
              fmt2(t3, sizeof(t3), s_tr_max), s_tr_n, s_al_valid ? "measured" : "motor_params");
    s_tr_n = 0U;
  }
  if (!COMMS_TEST && (s_al_phase == ALIGN_IDLE) && ((now_ms % VEC_LOG_PERIOD_MS) == 0U))
#else
  if (!COMMS_TEST && ((now_ms % VEC_LOG_PERIOD_MS) == 0U))   /* silent during the comms test */
#endif
  {
    float f_now;
    float cyc;
    char b1[16];
    char b2[16];
    char b3[16];
    mt6701_snapshot_t e;
    cursense_snapshot_t cs;
    ctrl_spin_state(&f_now, &cyc);
    mt6701_snapshot(&e);
    cursense_snapshot(&cs);
    int32_t pos = vec_enc_position();
    float dcyc = cyc - s_spin_cyc0;
    int32_t dpos = pos - s_spin_pos0;
    s_spin_cyc0 = cyc;
    s_spin_pos0 = pos;

    float cpc = 0.0f;
    float pp = 0.0f;
    if ((dcyc > SPIN_MIN_CYCLES_LOG) || (dcyc < -SPIN_MIN_CYCLES_LOG))
    {
      cpc = (float)dpos / dcyc;
      float acpc = (cpc < 0.0f) ? -cpc : cpc;
      pp = (acpc > 1.0f) ? ((float)ENC_COUNTS_PER_TURN / acpc) : 0.0f;
    }
    float i_alpha = cs.amps[PHASE_A];
    float i_beta = (cs.amps[PHASE_B] - cs.amps[PHASE_C]) * 0.577350269f;   /* 1/sqrt(3) */
    float i_amp = foc_sqrtf((i_alpha * i_alpha) + (i_beta * i_beta));
    /* meas: average over the log period from the encoder position (the
     * pass check); pll: the speed estimate at this instant, which at 5 Hz is
     * sampled at the same point of every electrical cycle and shows the
     * open-loop speed ripple (2026-10-08: -18 against 27 rpm average). */
    float rpm_avg = ((float)dpos / (float)ENC_COUNTS_PER_TURN) * 60000.0f / (float)VEC_LOG_PERIOD_MS;
    debug_log("SPIN armed=%u V=%lumV f=%sHz tgt=%s rpm forced=%ld meas=%ld pll=%ld cnt/ecyc=%ld pp=%s Iamp=%lumA %s",
              pwm_is_armed() ? 1U : 0U, (uint32_t)(s_vec_v * 1000.0f), fmt2(b1, sizeof(b1), f_now),
              fmt2(b2, sizeof(b2), spin_target()),
              (long)(f_now * 60.0f / (float)MOTOR_POLE_PAIRS), (long)rpm_avg, (long)(e.speed_rad_s * RAD_S_TO_RPM),
              (long)cpc, fmt2(b3, sizeof(b3), pp), (uint32_t)(i_amp * AMPS_TO_MA), fault_name(fault_first()));
  }
#else
  if (!COMMS_TEST && ((now_ms % VEC_LOG_PERIOD_MS) == 0U))   /* silent during the comms test */
  {
    float vph[PHASE_COUNT];
    float dmax;
    ctrl_last_output(vph, &dmax);
    /* Expected currents from Ohm's law on the commanded phase voltages
     * (plan Stage 8 pass: V/R within about 15 %). */
    debug_log("VEC armed=%u V=%lumV deg=%ld enc=%ld shift=%lu dmax=%lu expect mA=%ld/%ld/%ld trip=%lu fault=%s",
              pwm_is_armed() ? 1U : 0U, (uint32_t)(s_vec_v * 1000.0f), (long)g_vec_deg,
              (long)vec_enc_position(),
              (uint32_t)(vec_shift() * 1000.0f), (uint32_t)(dmax * 1000.0f),
              (long)(vph[PHASE_A] / MOTOR_R_PHASE_OHM * AMPS_TO_MA),
              (long)(vph[PHASE_B] / MOTOR_R_PHASE_OHM * AMPS_TO_MA),
              (long)(vph[PHASE_C] / MOTOR_R_PHASE_OHM * AMPS_TO_MA),
              (uint32_t)(s_trip_applied * 1000.0f), fault_name(fault_first()));
  }
#endif
}
#endif

/* ---- PWM test mode (Stages 5 to 7) ---------------------------------------- */

#if APP_TEST_MODE
static float test_target(float t)
{
  if (!(t >= 0.0f))   /* also catches NaN from a mistyped debugger value */
  {
    return 0.0f;
  }
  return (t > PWM_DUTY_MAX) ? PWM_DUTY_MAX : t;
}

static float test_slew(float now, float target)
{
  float step = target - now;
  if (step > TEST_SLEW_PER_MS)
  {
    step = TEST_SLEW_PER_MS;
  }
  else if (step < -TEST_SLEW_PER_MS)
  {
    step = -TEST_SLEW_PER_MS;
  }
  return now + step;
}

static void test_arm(void)
{
  const char *reason;
#if APP_BRIDGE_TOOLS
  uint32_t off = g_phase_off & 0x7UL;
  for (uint32_t ph = 0U; ph < PHASE_COUNT; ph++)
  {
    (void)pwm_phase_output((board_phase_t)ph, (off & (1UL << ph)) == 0U);
  }
  if (off != 0U)
  {
    debug_log("PWM phases held off (INH = INL = low): A=%u B=%u C=%u",
              (unsigned)(off & 1UL), (unsigned)((off >> 1) & 1UL), (unsigned)((off >> 2) & 1UL));
  }
#endif
#if BRINGUP_STAGE >= 7
  /* The control interrupt applies the command once armed: zero it first. */
  pwm_command_duty(PWM_DUTY_ZERO_CMD, PWM_DUTY_ZERO_CMD, PWM_DUTY_ZERO_CMD);
#endif
  pwm_zero_command();
  s_duty[PHASE_A] = PWM_DUTY_ZERO_CMD;
  s_duty[PHASE_B] = PWM_DUTY_ZERO_CMD;
  s_duty[PHASE_C] = PWM_DUTY_ZERO_CMD;
  if (motor_can_arm(&reason))
  {
    debug_log("PWM armed from zero command; slewing to A=%u B=%u C=%u permille",
              (unsigned)(test_target(g_duty_a) * 1000.0f), (unsigned)(test_target(g_duty_b) * 1000.0f),
              (unsigned)(test_target(g_duty_c) * 1000.0f));
  }
  else
  {
    debug_log("PWM arm refused: %s", reason);
  }
}

#if APP_BRIDGE_TOOLS
static void sweep_apply_step(void)
{
  float d = s_sweep_duty[s_sweep_step];
  g_duty_a = d;
  g_duty_b = d;
  g_duty_c = d;
  s_sweep_ms = 0U;
  debug_log("SWEEP %lu/%u duty=%u permille: expect MOTA/B/C avg = %lu mV (duty x VM)",
            s_sweep_step + 1UL, (unsigned)SWEEP_STEPS, (unsigned)(d * 1000.0f),
            (uint32_t)(d * (float)power_vm_mv()));
}

static void sweep_tick(void)
{
  if (!s_sweeping || (++s_sweep_ms < SWEEP_STEP_MS))
  {
    return;
  }
  if (++s_sweep_step < SWEEP_STEPS)
  {
    sweep_apply_step();
    return;
  }
  s_sweeping = false;
  g_duty_a = PWM_DUTY_ZERO_CMD;
  g_duty_b = PWM_DUTY_ZERO_CMD;
  g_duty_c = PWM_DUTY_ZERO_CMD;
  debug_log("SWEEP done; back to 50 %% on all phases");
}

/* Disarm and sleep, set the new IDRIVE, then drv_tick() wakes the DRV and
 * configure() writes, verifies and locks it. Re-arm by command to test. */
static void test_apply_idrive(void)
{
  uint32_t code = g_drv_idrive;
  uint32_t src_ma;
  uint32_t snk_ma;
  if (code > 0xFFUL)
  {
    debug_log("IDRIVE 0x%lX invalid (0x00-0xFF: IDRIVEP << 4 | IDRIVEN)", code);
    return;
  }
  s_sweeping = false;
  drv8323_sleep();                       /* disarms first */
  (void)drv8323_set_idrive((uint8_t)code);
  drv8323_idrive_ma((uint8_t)code, &src_ma, &snk_ma);
  debug_log("IDRIVE 0x%02lX source=%lumA sink=%lumA; DRV re-waking, re-arm to test", code, src_ma, snk_ma);
}
#endif

static void test_command(uint32_t cmd)
{
  switch (cmd)
  {
    case APP_CMD_ARM:
      test_arm();
      break;
    case APP_CMD_DISARM:
      motor_disarm();
      debug_log("PWM disarmed by command: moe=%u pins=0x%02lX", pwm_is_armed() ? 1U : 0U, board_pwm_pins_read());
      break;
    case APP_CMD_SW_BREAK:
      pwm_software_break();
      debug_log("PWM software break: moe=%u pins=0x%02lX (expect moe=0 pins=0x00)",
                pwm_is_armed() ? 1U : 0U, board_pwm_pins_read());
      break;
#if APP_BRIDGE_TOOLS
    case APP_CMD_SWEEP:
      if (!pwm_is_armed())
      {
        debug_log("SWEEP refused: arm first (g_app_cmd = 1)");
        break;
      }
      s_sweeping = true;
      s_sweep_step = 0U;
      sweep_apply_step();
      break;
    case APP_CMD_IDRIVE:
      test_apply_idrive();
      break;
#endif
#if BRINGUP_STAGE >= 7
    case APP_CMD_CAPTURE:
      debug_capture_trigger();
      debug_log("CAP triggered; g_app_cmd = 7 dumps it once full (25 ms)");
      break;
    case APP_CMD_DUMP:
      if (!debug_capture_dump_start())
      {
        debug_log("CAP dump refused: no finished capture (g_app_cmd = 6 first)");
      }
      break;
#endif
    default:
      debug_log("CMD %lu unknown", cmd);
      break;
  }
}

static void test_tick(uint32_t now_ms)
{
  uint32_t cmd = g_app_cmd;
  if (cmd != APP_CMD_NONE)
  {
    g_app_cmd = APP_CMD_NONE;
    test_command(cmd);
  }

  if (pwm_is_armed())
  {
#if APP_BRIDGE_TOOLS
    sweep_tick();
#endif
    s_duty[PHASE_A] = test_slew(s_duty[PHASE_A], test_target(g_duty_a));
    s_duty[PHASE_B] = test_slew(s_duty[PHASE_B], test_target(g_duty_b));
    s_duty[PHASE_C] = test_slew(s_duty[PHASE_C], test_target(g_duty_c));
#if BRINGUP_STAGE >= 7
    pwm_command_duty(s_duty[PHASE_A], s_duty[PHASE_B], s_duty[PHASE_C]);
#else
    pwm_set_duty(s_duty[PHASE_A], s_duty[PHASE_B], s_duty[PHASE_C]);
#endif
  }
#if APP_BRIDGE_TOOLS
  else
  {
    s_sweeping = false;   /* a break or fault ends the sweep */
  }
#endif

  if ((now_ms % TEST_LOG_PERIOD_MS) == 0U)
  {
    uint32_t a;
    uint32_t b;
    uint32_t c;
    const char *drv = drv8323_is_configured() ? "CFG" : (drv8323_is_awake() ? "AWAKE" : "ASLEEP");
    pwm_get_duty_permille(&a, &b, &c);
    debug_log("PWM armed=%u A=%lu B=%lu C=%lu vm=%lumV drv=%s nf=%s idrive=0x%02X brk=%lu fault=%s",
              pwm_is_armed() ? 1U : 0U, a, b, c, power_vm_mv(), drv,
              drv8323_nfault_low() ? "LOW" : "high", (unsigned)drv8323_idrive(),
              pwm_break_events(), fault_name(fault_first()));
  }
}
#endif

/* ---- Entry points --------------------------------------------------------- */

/* CRS_CR TRIM is 7 bits with its centre and reset value at 64 (RM0440 8.7.1;
 * HAL RCC_CRS_HSI48CALIBRATION_DEFAULT). CubeMX only offers 0-63 and writes
 * 32, about 32 x 0.14 % = 4.5 % slow (DS13122 Table 43): too far off for USB
 * to decode the SOF the CRS needs to correct it (Bring-up log 2026-10-14).
 * TRIM is read-only while AUTOTRIMEN is set, so pause auto-trimming. */
#define APP_HSI48_TRIM_CENTRE   64U

void app_clock_init(void)
{
  CLEAR_BIT(CRS->CR, CRS_CR_AUTOTRIMEN);
  MODIFY_REG(CRS->CR, CRS_CR_TRIM, APP_HSI48_TRIM_CENTRE << CRS_CR_TRIM_Pos);
  SET_BIT(CRS->CR, CRS_CR_AUTOTRIMEN);
}

void app_init(void)
{
  /* Safe pins first, before anything is logged. */
  board_safe_pins();

  bool debug_ok = debug_init();

  debug_log("BOOT stage=%d", BRINGUP_STAGE);
#if BRINGUP_STAGE >= 2
  debug_log_reset_cause();
#endif
  debug_log("DAC3 %s (scope pin retired: PA2/PA3 are USART2)", debug_ok ? "OK" : "FAIL");
  (void)app_check_clock();
  app_check_cycle_counter();
  app_check_safe_pins();

#if BRINGUP_STAGE >= 1
  bool enc_cfg_ok = mt6701_init(ENC_DT_S);
  debug_log("ENC spi1 mode1 12bit div32 %s", enc_cfg_ok ? "OK" : "FAIL");
  (void)mt6701_update();
  mt6701_snapshot_t s;
  mt6701_snapshot(&s);
  debug_log("ENC first raw=0x%06lX a=%u st=0x%X crc=%s field=%s lot=%u",
            s.raw_word, (unsigned)s.angle_counts, (unsigned)s.status,
            (s.counters.crc_errors == 0U) ? "OK" : "FAIL",
            mt6701_field_name(mt6701_field(s.status)),
            mt6701_loss_of_track(s.status) ? 1U : 0U);
#endif

#if BRINGUP_STAGE >= 2
  bool pwr_ok = power_init();
  debug_log("PWR adc1 cal+enable %s en=%u vm_mv=%lu", pwr_ok ? "OK" : "FAIL",
            power_motor_en() ? 1U : 0U, power_vm_mv());
  /* After power_init() (ADC1 enabled), before cursense_init() (injected
   * conversions armed: sampling times can't be changed after). */
  debug_log("AUX thr1 PA0/ADC2_IN1 thr2 PA1/ADC2_IN2 isup PB12/ADC1_IN11 smp 47.5 %s",
            aux_sense_init() ? "OK" : "FAIL (ADC busy)");
#endif
#if BRINGUP_STAGE == 2
  /* Start from the current SW1 level, so booting with SW1 already on
   * doesn't count as a switch-on edge. */
  s_en_raw_prev = power_motor_en_raw();
#endif

#if BRINGUP_STAGE >= 3
  bool drv_cfg_ok = drv8323_init();
  debug_log("DRV spi3 mode1 16bit div256 %s, asleep (ENABLE low, PD2 released)",
            drv_cfg_ok ? "OK" : "FAIL");
#endif

#if BRINGUP_STAGE >= 5
  const char *what;
  bool pwm_ok = pwm_init(&what);
  debug_log("PWM tim1 center 20kHz dt=100ns brk=PB10 low trgo2=OC4REF ccr4=4200 %s (%s); dbg freeze on, counter running, moe=%u",
            pwm_ok ? "OK" : "FAIL", what, pwm_is_armed() ? 1U : 0U);
  debug_log("PWM sysbrk lockup+sram_parity+flash_ecc cfgr2=0x%03lX", SYSCFG->CFGR2);
#endif
#if BRINGUP_STAGE >= 7
  /* After power_init() (ADC1 enabled) and pwm_init() (TRGO2 running). */
  const char *cur_what;
  bool cur_ok = cursense_init(&cur_what);
  debug_log("CUR adc1/2/3 injected on TRGO2, control irq on ADC1 JEOS %s (%s)", cur_ok ? "OK" : "FAIL", cur_what);
#endif
#if BRINGUP_STAGE == 5
  debug_log("STAGE5 DRV held asleep. Commands: set g_app_cmd = 1 arm, 2 disarm, 3 software break");
#elif APP_BRIDGE_TOOLS
  debug_log("STAGE%d g_app_cmd: 1 arm, 2 disarm, 3 sw break, 4 sweep, 5 apply g_drv_idrive", BRINGUP_STAGE);
#if BRINGUP_STAGE >= 7
  debug_log("STAGE%d g_app_cmd: 6 capture trigger, 7 capture dump", BRINGUP_STAGE);
#endif
  debug_log("STAGE%d targets g_duty_a/b/c (0..0.93), now A=%u B=%u C=%u permille", BRINGUP_STAGE,
            (unsigned)(g_duty_a * 1000.0f), (unsigned)(g_duty_b * 1000.0f), (unsigned)(g_duty_c * 1000.0f));
  debug_log("STAGE%d g_phase_off: 1 = A, 2 = B, 4 = C held off (both FETs) from the next arm", BRINGUP_STAGE);
#elif APP_VEC_MODE
  s_foc_ok = foc_init();
  debug_log("FOC cordic cos/sin q1.31 24 iterations %s", s_foc_ok ? "OK" : "FAIL (CubeMX: activate CORDIC)");
#if BRINGUP_STAGE == 8
  debug_log("STAGE8 g_app_cmd: 1 arm, 2 disarm, 3 sw break, 6 capture, 7 dump, 8 L step");
  debug_log("STAGE8 g_vec_v (V, max 1.2) g_vec_deg g_duty_shift (max 0.43) g_oc_trip (A, 0.1-1.5) g_vstep_v");
#else
  debug_log("STAGE%d g_app_cmd: 1 arm (align, then spin), 2 disarm, 3 sw break, 6 capture, 7 dump", BRINGUP_STAGE);
  debug_log("STAGE%d g_vec_v (V, max 1.2) g_spin_hz (elec Hz, +/-10, 0 = hold) g_spin_accel (Hz/s, max 10) g_oc_trip",
            BRINGUP_STAGE);
#if BRINGUP_STAGE >= 10
  debug_log("STAGE10 g_app_cmd = 9: encoder alignment at g_vec_v (disarmed first); TRACK line while spinning");
#endif
#if BRINGUP_STAGE >= 11
  debug_log("STAGE11 g_app_cmd = 10: voltage-mode FOC (Vd 0), Vq from THROTTLE1 (g_vq_pot = 1; middle = 0, ends = +/-2 V)");
  debug_log("STAGE11 or g_vq_pot = 0: Vq = g_vq (V, +/-2). Ramp 1 V/s. Pausing while armed disarms (encoder jump)");
#endif
#if BRINGUP_STAGE >= 12
  debug_log("STAGE12 DQ line every second; g_cap_dq = 1: capture (6, 7) holds Id, Iq, theta_e");
#endif
#endif
#endif

#if COMMS_TEST
  comms_test_init();
#endif

  s_last_tick_ms = HAL_GetTick();
}

void app_loop(void)
{
  uint32_t now = HAL_GetTick();

  /* Run every 1 ms tick once, catching up if the loop was late. */
  while (s_last_tick_ms != now)
  {
    s_last_tick_ms++;

    uint32_t start = debug_cycles();
#if BRINGUP_STAGE >= 1
    mt6701_snapshot_t enc;
    enc_tick(&enc);
#endif
#if BRINGUP_STAGE >= 9
    enc_monitor(&enc);           /* encoder faults disarm from Stage 9 */
#endif
#if BRINGUP_STAGE >= 2
    power_tick(s_last_tick_ms);
    aux_sense_tick(s_last_tick_ms);
#endif
#if BRINGUP_STAGE >= 3
    drv_tick();
#endif
#if BRINGUP_STAGE >= 7
    ctrl_tick(s_last_tick_ms);   /* before drv_monitor: a fault here gets the same-tick response */
#endif
#if BRINGUP_STAGE >= 4
    drv_monitor(s_last_tick_ms);
#endif

#if BRINGUP_STAGE == 0
    stage0_tick(s_last_tick_ms);
#elif BRINGUP_STAGE == 1
    stage1_tick(s_last_tick_ms, &enc);
#elif BRINGUP_STAGE == 2
    stage2_tick(s_last_tick_ms);
#elif BRINGUP_STAGE == 3
    stage3_tick(s_last_tick_ms);
#elif BRINGUP_STAGE == 4
    stage4_tick(s_last_tick_ms);
#elif APP_TEST_MODE
    test_tick(s_last_tick_ms);
#elif APP_VEC_MODE
    vec_tick(s_last_tick_ms);
#endif
#if COMMS_TEST
    comms_test_tick(s_last_tick_ms);
#endif
    uint32_t elapsed = debug_cycles() - start;
    if (elapsed > s_task_max_cycles)
    {
      s_task_max_cycles = elapsed;
    }

    if ((s_last_tick_ms % ALIVE_PERIOD_MS) == 0U)
    {
#if BRINGUP_STAGE >= 1
      debug_log("ALIVE t=%lus task_max_us=%lu enc_ok=%u enc_crc=%lu enc_jump=%lu",
                s_last_tick_ms / 1000UL, s_task_max_cycles / DEBUG_CYCLES_PER_US,
                enc.healthy ? 1U : 0U, enc.counters.crc_errors, enc.counters.angle_jumps);
#else
      debug_log("ALIVE t=%lus task_max_us=%lu", s_last_tick_ms / 1000UL,
                s_task_max_cycles / DEBUG_CYCLES_PER_US);
#endif
    }
  }
}

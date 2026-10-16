/**
 * @file    ctrl.h
 * @brief   Control step run inside the 20 kHz control interrupt, after the
 *          currents are read: software overcurrent trip, then the output
 *          (Stage 7: duty command; Stage 8: fixed voltage vector).
 *
 * Not in the plan's original module list: added in Stage 8 to keep the
 * control logic out of the ADC driver (cursense) and the PWM driver.
 */
#ifndef CTRL_H
#define CTRL_H

#include "board.h"
#include <stdbool.h>
#include <stdint.h>

/** Called by the control interrupt with the phase currents (A, positive
 *  into the motor) and the raw ADC counts (for the capture buffer).
 *  Register access only. */
void ctrl_isr(const float amps[PHASE_COUNT], const uint16_t raw[PHASE_COUNT]);

/** Overcurrent trip level, A. Clamped to [MOTOR_OC_TRIP_MIN_A,
 *  MOTOR_OC_TRIP_STAGE_MAX_A]; NaN gives the default. Returns the level
 *  applied. Main loop. */
float ctrl_set_trip(float amps);

/** True once after the interrupt tripped on overcurrent (it has already
 *  disarmed, latched the fault and triggered the capture); *phase and *amps
 *  say where. Consuming this diagnostic event does not clear the fault. */
bool ctrl_take_oc_event(uint32_t *phase, float *amps);

/** Stage 8: voltage vector command, phase-voltage amplitude v (V) at
 *  electrical angle theta (rad), plus a common-mode duty shift added to
 *  all three duties. Applied by the interrupt while armed. Main loop. */
void ctrl_set_vector(float v, float theta, float shift);

/** Stage 8: apply v_step at the next interrupt and trigger the capture in
 *  that same interrupt, so the capture's trigger marks the step. */
void ctrl_vector_step(float v_step);

/** Stage 9 forced angle. reset: angle, speed and cycle count to 0 (call
 *  while disarmed). set_spin: target electrical frequency (Hz, sign =
 *  direction) and the slew toward it (Hz/s). spin_state: present speed and
 *  electrical cycles since the reset (fractional). Main loop. */
void ctrl_spin_reset(void);
void ctrl_set_spin(float f_target_hz, float accel_hz_s);
void ctrl_spin_state(float *f_now_hz, float *cycles);

/** Stage 10: true = the interrupt uses the angle from ctrl_set_vector()
 *  (alignment steps), false = the Stage 9 forced-angle generator. */
void ctrl_hold_angle(bool hold);

/** Stage 11 voltage-mode FOC. set_foc: true = the interrupt commutates on
 *  the encoder angle with Vd = 0 and Vq from set_vq (V, phase amplitude;
 *  set before arming). take_enc_event: true once after the interrupt
 *  disarmed on a bad encoder sample. Main loop. */
void ctrl_set_foc(bool on);
void ctrl_set_vq(float vq);
bool ctrl_take_enc_event(void);

/** Stage 12 rotating-frame currents (A), accumulated by the interrupt since
 *  the last call: mean, min, max of every sample with a valid encoder
 *  angle, enabled driver and valid offsets; n accepted samples, bad = rejected
 *  samples. n=0 means no valid measurement, not verified zero current.
 *  Resets the statistics.
 *  capture_dq: true = the capture buffer records Id, Iq (mA + 32768) and
 *  the electrical angle (0..65535 per turn) instead of raw counts. Main
 *  loop. */
typedef struct
{
  float id_mean;
  float iq_mean;
  float id_min;
  float id_max;
  float iq_min;
  float iq_max;
  uint32_t n;
  uint32_t bad;
} ctrl_dq_stats_t;
void ctrl_dq_stats(ctrl_dq_stats_t *out);
void ctrl_capture_dq(bool dq);

/** Phase voltages last commanded (V, before injection) and the highest
 *  duty, for logging. */
void ctrl_last_output(float vph[PHASE_COUNT], float *duty_max);

#endif /* CTRL_H */

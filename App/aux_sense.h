/**
 * @file    aux_sense.h
 * @brief   Slow analog inputs: the two on-board throttle pots and the
 *          ACS71240 total supply-current sensor, read in the 1 kHz main-loop
 *          tick by software-started regular conversions.
 *
 *  - THROTTLE1 PA0 = ADC2_IN1, THROTTLE2 PA1 = ADC2_IN2: 10k pots from 3V3
 *    to GND, wiper through 100 R with 100 nF (schematic, peripherals sheet).
 *  - HALL_SENSE PB12 = ADC1_IN11: ACS71240 output through 56 R / 2.2 nF;
 *    the sensor is in series with the supply, before the K1 relay (schematic,
 *    power sheet), so it reads the whole board's VM current.
 *
 * Must be initialised after power_init() and before cursense_init(): the
 * sampling times can only be written while no injected conversion is armed
 * (JADSTART = 0, RM0440 ADC_SMPRx).
 */
#ifndef AUX_SENSE_H
#define AUX_SENSE_H

#include <stdbool.h>
#include <stdint.h>

/** Sets the three channels' sampling times. False if ADC1/ADC2 were busy. */
bool aux_sense_init(void);

/** Every 1 ms main-loop tick: converts, filters, tracks the current zero. */
void aux_sense_tick(uint32_t now_ms);

/** Throttle position 0..1000 permille (index 0 or 1). */
uint32_t aux_throttle_permille(uint32_t index);

/** Supply current in mA (positive into the board), and whether its zero
 *  has been measured yet (taken while SW1 is off and the relay is open). */
int32_t aux_supply_ma(void);
bool aux_supply_zeroed(void);

#endif /* AUX_SENSE_H */

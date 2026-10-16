/**
 * @file    app_config.h
 * @brief   Bring-up stage selection and build-time test switches.
 *
 * Plan section "Stage selection": each stage's test mode is compiled in only
 * when selected; modules from earlier stages stay active underneath it.
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* Current bring-up stage (plan: "Stages 0-4" onward). */
#define BRINGUP_STAGE 12

/* Stage 0 one-off: execute an undefined instruction 5 s after boot to check
 * that the HardFault dump appears over SWO. Set to 1 only for that test
 * (needs BRINGUP_STAGE 0). */
#define STAGE0_FAULT_TEST 0

/* Communication interface bench test (I2C IMU, USART2, FDCAN2, USB CDC),
 * run alongside the selected stage; see comms_test.h. Run with SW1 off and
 * the motor disconnected. 1 also silences the 1 s CUR/CTRL/SPIN lines. */
#define COMMS_TEST 0

/* Per-interface switches for the comms test (1 = run, 0 = left idle: the
 * peripheral is initialised by CubeMX but never started, so FDCAN2 stays in
 * init mode and drives nothing, USART2 neither sends nor arms reception, and
 * I2C1 is not touched). */
#define COMMS_TEST_I2C  0
#define COMMS_TEST_UART 0
#define COMMS_TEST_CAN  0
#define COMMS_TEST_USB  1

/* Standalone diagnostics owns USART2 instead of the legacy UART bench test.
 * Existing CubeMX setup: PA2 TX / PA3 RX, 115200 8N1, IRQ priority 6. */
#define UART_DIAG_ENABLE 1
#if UART_DIAG_ENABLE && COMMS_TEST && COMMS_TEST_UART
#error "USART2 has two owners: disable UART_DIAG_ENABLE or COMMS_TEST_UART"
#endif

/* Throttle pots and supply-current sensor test (aux_sense.c): logs both
 * pots and the ACS71240 supply current every 250 ms. The module itself runs
 * from Stage 2 regardless; this switch only adds the log lines. */
#define SENSOR_TEST 0

/* Code on the 20 kHz control-interrupt path runs from SRAM: the linker
 * script's .RamFunc input section goes into .data, which the startup code
 * copies from flash (STM32G491XX_FLASH.ld). From flash, with 4 wait states,
 * prefetch off and a small cache, the interrupt took 7.7 us armed (Stage 8,
 * 2026-10-08). */
#define APP_RAMFUNC   __attribute__((section(".RamFunc")))

#endif /* APP_CONFIG_H */

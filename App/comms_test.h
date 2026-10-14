/**
 * @file    comms_test.h
 * @brief   Communication interface bench test (enabled by COMMS_TEST in
 *          app_config.h): I2C IMU read, UART and CAN send/receive with
 *          another MCU, USB CDC (virtual COM port) send/receive with a PC.
 *
 * Main loop only (HAL allowed here by developer decision, 2026-10-13); the
 * receive callbacks run in the peripherals' interrupts and only copy data.
 * The motor stays idle: run with SW1 off (no VM_OK, so arming is refused).
 *
 *  - I2C1 (PB9 SDA, PA15 SCL): LSM6DSOX, WHO_AM_I then accel/gyro/temp
 *    every 200 ms.
 *  - USART2 (PA2 TX, PA3 RX on J15), 115200 8N1: a counter line every
 *    100 ms; each received line is logged and echoed back ("echo: ...").
 *  - FDCAN2 (PB6 TX, PB5 RX on J10), classic CAN 500 kbit/s: ID 0x101 every
 *    100 ms (counter, uptime); every received frame is logged; a frame on
 *    ID 0x200 is answered on ID 0x201 with the same data (no blanket echo,
 *    so two echoing nodes can't ping-pong).
 *  - USB CDC (PA11/PA12, J1): a counter line every 1 s on the PC's COM port;
 *    typed text is logged and echoed back.
 */
#ifndef COMMS_TEST_H
#define COMMS_TEST_H

#include <stdint.h>

void comms_test_init(void);
void comms_test_tick(uint32_t now_ms);

/** Called from CDC_Receive_FS (USB interrupt): copies the received bytes. */
void comms_test_usb_rx(const uint8_t *buf, uint32_t len);

/** Called first in USB_LP_IRQHandler (USER CODE USB_LP_IRQn 0): counts the
 *  interrupt causes for the enumeration diagnostics. */
void comms_test_usb_irq(void);

#endif /* COMMS_TEST_H */

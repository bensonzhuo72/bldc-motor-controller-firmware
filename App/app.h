/**
 * @file    app.h
 * @brief   Application entry points called from main.c USER CODE blocks.
 */
#ifndef APP_H
#define APP_H

/** Once, after SystemClock_Config and before the peripheral init
 *  (main.c USER CODE SysInit): clock corrections CubeMX can't express. */
void app_clock_init(void);

/** Once, after the CubeMX peripheral init (main.c USER CODE 2). */
void app_init(void);

/** Every main-loop pass (main.c USER CODE 3); runs the 1 kHz tick tasks. */
void app_loop(void);

#endif /* APP_H */

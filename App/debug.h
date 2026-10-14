/**
 * @file    debug.h
 * @brief   SWO logging, CPU cycle counter, capture buffer, HardFault/NMI
 *          dump. (Scope DAC and PA3 timing pin retired 2026-10-13.)
 *
 * Logging is main-loop only (plan: Timing, interrupts and data sharing).
 * The scope and timing-pin helpers are register writes, safe in interrupts.
 */
#ifndef DEBUG_H
#define DEBUG_H

#include "main.h"
#include "board.h"
#include <stdbool.h>
#include <stdint.h>

/* HCLK = 170 MHz (plan: TIM1 and control timing, "Clock"). */
#define DEBUG_CPU_HZ             170000000UL
#define DEBUG_CYCLES_PER_US      (DEBUG_CPU_HZ / 1000000UL)

/* DAC3 is 12-bit; 4095 is full scale. */
#define DEBUG_SCOPE_FULL_SCALE   4095UL

/** Start the cycle counter and DAC3. Returns false if the DAC failed. */
bool debug_init(void);

/** printf-style log line over SWO (ITM port 0); a newline is appended.
 *  Main loop only. Integers only: newlib-nano printf has no float support. */
void debug_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/** Capture buffer (Stage 7 onward). push: control interrupt, every sample.
 *  trigger: keeps 512 samples before it, records 512 after, then freezes.
 *  dump_start / dump_tick: main loop, one SWO line per call; the capture
 *  re-arms after the last line. */
void debug_capture_push(uint16_t a, uint16_t b, uint16_t c);
void debug_capture_trigger(void);
bool debug_capture_done(void);
/** Discard a finished capture and start recording again. Refused (false)
 *  while a dump is printing. Main loop. */
bool debug_capture_rearm(void);
/** Sample n of a finished capture, oldest first. A trigger from the control
 *  interrupt lands after that interrupt's sample (n = 511); n = 512 is the
 *  first sample after it. */
bool debug_capture_get(uint32_t n, uint16_t out[3]);
#define DEBUG_CAPTURE_LEN      1024U
#define DEBUG_CAPTURE_TRIGGER  512U
bool debug_capture_dump_start(void);
bool debug_capture_dump_tick(void);

/** Log the reset cause from RCC_CSR, then clear the flags (plan Stage 2). */
void debug_log_reset_cause(void);

/** NMI response, called from NMI_Handler: outputs off, then ENABLE low,
 *  then FLASH_ECCR and SYSCFG_CFGR2 over SWO; halts (debugger) or blinks. */
void debug_nmi_report(void) __attribute__((noreturn));

/** Raw SWO output without printf, usable from fault handlers. */
void debug_put_str(const char *s);
void debug_put_hex32(uint32_t v);

/** Free-running CPU cycle count (DWT CYCCNT), wraps every ~25 s at 170 MHz. */
static inline uint32_t debug_cycles(void)
{
  return DWT->CYCCNT;
}

/** Timing pin: none since 2026-10-13 (PA3 is USART2 RX on J15). Kept as
 *  no-ops so callers stay unchanged; interrupt time is logged on the CTRL
 *  line instead. */
static inline void debug_timing_high(void) {}
static inline void debug_timing_low(void) {}
static inline void debug_timing_toggle(void) {}

/** Scope DAC: 12-bit code to DAC3 CH1, internal only since 2026-10-13
 *  (OPAMP1 to PA2 removed). Harmless; kept so callers stay unchanged. */
static inline void debug_scope_write(uint32_t code)
{
  DAC3->DHR12R1 = (code > DEBUG_SCOPE_FULL_SCALE) ? DEBUG_SCOPE_FULL_SCALE : code;
}

#endif /* DEBUG_H */

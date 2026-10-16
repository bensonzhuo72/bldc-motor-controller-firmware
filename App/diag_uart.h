#ifndef DIAG_UART_H
#define DIAG_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Single main-loop producer/consumer; IRQ owns TX consumption/RX production.
 * No allocation, waiting, logging, motor calls or global IRQ masking here. */
typedef struct
{
  uint32_t tx_bytes, rx_bytes, tx_dropped, rx_overflow, rx_errors, rx_stale, tx_pending;
} diag_uart_stats_t;

bool diag_uart_init(void);  /* after CubeMX USART2 init; no peripheral reconfiguration */
void diag_uart_irq(void);   /* USART2 USER CODE hook; replaces HAL handler when enabled */
/* Enqueue entire record or drop it. reserve keeps room for command responses. */
bool diag_uart_write(const char *data, size_t len, size_t reserve);
/* 0..255 byte, -1 empty, -2 RX gap/error (queued bytes flushed). */
int diag_uart_read(void);
void diag_uart_stats(diag_uart_stats_t *out); /* counters are approximate, nonblocking */

#endif

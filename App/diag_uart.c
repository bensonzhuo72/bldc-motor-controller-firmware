#include "diag_uart.h"
#include "app_config.h"
#include "usart.h"
#include "stm32g4xx_ll_usart.h"

/* Design budgets for 115200 8N1 (11,520 B/s), not guaranteed telemetry rates.
 * Powers of two permit unsigned monotonic indexes, including wraparound. */
#define TX_SIZE 2048U
#define RX_SIZE 256U
#define RX_MAX_AGE_MS 250U /* discard commands buffered across a long main-loop stall */
static volatile uint8_t s_tx[TX_SIZE], s_rx[RX_SIZE];
static volatile uint32_t s_rx_ms[RX_SIZE];
static volatile uint32_t s_tx_head, s_tx_tail, s_rx_head, s_rx_tail;
static volatile uint32_t s_tx_bytes, s_rx_bytes, s_rx_overflow, s_rx_errors, s_rx_epoch;
static uint32_t s_tx_dropped, s_rx_seen, s_rx_stale;
static bool s_ready;

bool diag_uart_init(void)
{
#if UART_DIAG_ENABLE
  /* Generated configuration owns pins/clock/baud/NVIC. Refuse mismatches.
   * IRQ priority must remain below the priority-0 ADC/control interrupt. */
  if ((huart2.Instance != USART2) || (huart2.Init.BaudRate != 115200U) ||
      (huart2.Init.WordLength != UART_WORDLENGTH_8B) ||
      (huart2.Init.StopBits != UART_STOPBITS_1) ||
      (huart2.Init.Parity != UART_PARITY_NONE) ||
      (huart2.Init.Mode != UART_MODE_TX_RX) ||
      (huart2.Init.HwFlowCtl != UART_HWCONTROL_NONE) ||
      ((USART2->CR1 & USART_CR1_FIFOEN) != 0U) ||
      (NVIC_GetPriority(USART2_IRQn) == 0U) || !NVIC_GetEnableIRQ(USART2_IRQn))
  {
    return false;
  }
  /* RM0440 USART ICR/RQR; corresponding vendor LL clear/read/IT helpers.
   * Discard boot noise before accepting commands. RX/TX IT are runtime IO. */
  LL_USART_ClearFlag_ORE(USART2);
  LL_USART_ClearFlag_FE(USART2);
  LL_USART_ClearFlag_NE(USART2);
  LL_USART_ClearFlag_PE(USART2);
  LL_USART_RequestRxDataFlush(USART2);
  s_ready = true;
  LL_USART_EnableIT_ERROR(USART2);
  LL_USART_EnableIT_RXNE_RXFNE(USART2);
  return true;
#else
  return false;
#endif
}

bool diag_uart_write(const char *data, size_t len, size_t reserve)
{
  uint32_t head = s_tx_head;
  uint32_t used = head - s_tx_tail;
  if (!s_ready || len > TX_SIZE || reserve > TX_SIZE ||
      len + reserve > TX_SIZE - used)
  {
    s_tx_dropped++;
    return false;
  }
  for (size_t i = 0U; i < len; i++)
  {
    s_tx[(head + (uint32_t)i) & (TX_SIZE - 1U)] = (uint8_t)data[i];
  }
  __DMB();
  s_tx_head = head + (uint32_t)len;
  /* Atomic CR1 update from vendor LL: UART IRQ may clear TXEIE concurrently. */
  LL_USART_EnableIT_TXE_TXFNF(USART2);
  return true;
}

int diag_uart_read(void)
{
  uint32_t epoch = s_rx_epoch;
  if (epoch != s_rx_seen)
  {
    s_rx_tail = s_rx_head;
    s_rx_seen = epoch;
    return -2;
  }
  uint32_t tail = s_rx_tail;
  if (tail == s_rx_head) { return -1; }
  __DMB();
  if (uwTick - s_rx_ms[tail & (RX_SIZE - 1U)] > RX_MAX_AGE_MS)
  {
    s_rx_tail = s_rx_head;
    s_rx_stale++;
    return -2;
  }
  int value = s_rx[tail & (RX_SIZE - 1U)];
  __DMB();
  s_rx_tail = tail + 1U;
  /* Do not return a byte from a stream that developed a gap during read. */
  if (s_rx_epoch != epoch) { return -2; }
  return value;
}

void diag_uart_irq(void)
{
  /* At most one RX and one TX byte per IRQ (FIFO disabled by CubeMX).
   * Priority-0 motor control can preempt every operation below. */
  uint32_t flags = USART2->ISR;
  bool error = (flags & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) != 0U;
  if (error)
  {
    USART2->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
    s_rx_errors++;
    s_rx_epoch++;
  }
  if ((flags & USART_ISR_RXNE_RXFNE) != 0U)
  {
    uint8_t byte = LL_USART_ReceiveData8(USART2);
    s_rx_bytes++;
    uint32_t head = s_rx_head;
    if (!error)
    {
      if (head - s_rx_tail < RX_SIZE)
      {
        s_rx[head & (RX_SIZE - 1U)] = byte;
        s_rx_ms[head & (RX_SIZE - 1U)] = uwTick;
        __DMB();
        s_rx_head = head + 1U;
      }
      else
      {
        s_rx_overflow++;
        s_rx_epoch++;
      }
    }
  }
  if (((flags & USART_ISR_TXE_TXFNF) != 0U) &&
      LL_USART_IsEnabledIT_TXE_TXFNF(USART2))
  {
    uint32_t tail = s_tx_tail;
    if (tail != s_tx_head)
    {
      __DMB();
      LL_USART_TransmitData8(USART2, s_tx[tail & (TX_SIZE - 1U)]);
      s_tx_tail = tail + 1U;
      s_tx_bytes++;
    }
    if (s_tx_tail == s_tx_head) { LL_USART_DisableIT_TXE_TXFNF(USART2); }
  }
}

void diag_uart_stats(diag_uart_stats_t *out)
{
  out->tx_bytes = s_tx_bytes;
  out->rx_bytes = s_rx_bytes;
  out->tx_dropped = s_tx_dropped;
  out->rx_overflow = s_rx_overflow;
  out->rx_errors = s_rx_errors;
  out->rx_stale = s_rx_stale;
  out->tx_pending = s_tx_head - s_tx_tail;
}

/**
 * @file    comms_test.c
 * @brief   I2C / UART / CAN / USB bench test. See comms_test.h.
 */
#include "comms_test.h"
#include "debug.h"
#include "i2c.h"
#include "usart.h"
#include "fdcan.h"
#include "usbd_cdc_if.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;

/* ---- IMU: LSM6DSOX (datasheet DS12814) ------------------------------------- */

#define IMU_ADDR_SA0_LOW        0x6AU   /* 7-bit address 110101xb, SA0 = 0 (6.1.1) */
#define IMU_ADDR_SA0_HIGH       0x6BU   /* SA0 = 1 */
#define IMU_REG_WHO_AM_I        0x0FU
#define IMU_WHO_AM_I_VALUE      0x6CU   /* 9.14: fixed at 6Ch */
#define IMU_REG_CTRL1_XL        0x10U
#define IMU_REG_CTRL2_G         0x11U
#define IMU_REG_CTRL3_C         0x12U
#define IMU_REG_OUT_TEMP_L      0x20U   /* temp, gyro X/Y/Z, accel X/Y/Z: 14 bytes to 2Dh */
#define IMU_CTRL3_C_BDU_IF_INC  0x44U   /* BDU = 1 (bit 6), IF_INC = 1 (bit 2, the default) */
#define IMU_CTRL1_XL_104HZ_2G   0x40U   /* ODR_XL 0100 = 104 Hz, FS_XL 00 = +/-2 g (Tables 51, 52) */
#define IMU_CTRL2_G_104HZ_250   0x40U   /* ODR_G 0100 = 104 Hz, FS_G 00 = +/-250 dps (Tables 54, 55) */
#define IMU_ACC_MG_PER_LSB      0.061f  /* LA_So at +/-2 g (Table 2) */
#define IMU_GYRO_MDPS_PER_LSB   8.75f   /* G_So at +/-250 dps (Table 2) */
#define IMU_TEMP_LSB_PER_C      256.0f  /* TSen (Table 4) */
#define IMU_TEMP_ZERO_C         25.0f   /* 0 LSB = 25 degC; offset up to +/-15 degC (Table 4) */
#define IMU_DATA_BYTES          14U
#define IMU_I2C_TIMEOUT_MS      5U
#define IMU_PERIOD_MS           200U
#define IMU_RETRY_MS            1000U   /* re-probe period while not found */

/* ---- UART: USART2, 115200 8N1 (CubeMX) -------------------------------------- */

#define UART_TX_PERIOD_MS       500U
#define UART_BUF                64U     /* one received burst (up to an idle line) */
#define UART_TX_BUF             96U

/* ---- CAN: FDCAN2, classic, 500 kbit/s (CubeMX: 170 MHz / 17, 1 + 15 + 4 tq) -- */

#define CAN_TX_ID               0x101U  /* our periodic frame */
#define CAN_REQ_ID              0x200U  /* the other MCU asks... */
#define CAN_REPLY_ID            0x201U  /* ...and we answer with the same data */
#define CAN_TX_PERIOD_MS        100U
#define CAN_RX_QUEUE            8U      /* power of two */

/* ---- USB CDC -------------------------------------------------------------- */

#define USB_TX_PERIOD_MS        1000U
#define USB_BUF                 64U

#define STATUS_PERIOD_MS        2000U

/* ---- State ---------------------------------------------------------------- */

static uint8_t  s_imu_addr;              /* 7-bit address found, 0 = none */
static uint32_t s_imu_reads;
static uint32_t s_imu_errors;

static uint8_t  s_uart_rx[UART_BUF];     /* filled by HAL in the USART2 interrupt */
static char     s_uart_line[UART_BUF + 1U];
static volatile uint16_t s_uart_line_len;  /* 0 = nothing pending */
static volatile uint32_t s_uart_rx_bytes;
static volatile uint32_t s_uart_errors;
static volatile uint32_t s_uart_overruns;  /* a burst arrived before the last was logged */
static char     s_uart_tx[UART_TX_BUF];
static uint32_t s_uart_tx_count;
static uint32_t s_uart_tx_busy;

typedef struct
{
  uint32_t id;
  uint32_t len;
  uint8_t  data[8];
} can_frame_t;

static can_frame_t s_can_q[CAN_RX_QUEUE];
static volatile uint32_t s_can_head;       /* written by the FDCAN interrupt */
static uint32_t s_can_tail;                /* main loop */
static volatile uint32_t s_can_rx_count;
static volatile uint32_t s_can_rx_dropped;
static uint32_t s_can_tx_count;
static uint32_t s_can_tx_full;
static bool     s_can_ok;

static uint8_t  s_usb_rx[USB_BUF];
static volatile uint32_t s_usb_rx_len;     /* 0 = nothing pending */
static volatile uint32_t s_usb_rx_bytes;
static char     s_usb_tx[USB_BUF + 16U];
static uint32_t s_usb_tx_count;
static uint8_t  s_usb_state;

/* ---- Helpers -------------------------------------------------------------- */

/* Signed fixed-point text with the given decimals (no float printf in
 * newlib-nano). */
static const char *fmt_fixed(char *buf, size_t len, float x, uint32_t decimals)
{
  uint32_t scale = 1U;
  for (uint32_t i = 0U; i < decimals; i++)
  {
    scale *= 10U;
  }
  float s = x * (float)scale;
  int32_t v = (int32_t)(s + ((s < 0.0f) ? -0.5f : 0.5f));
  uint32_t m = (uint32_t)((v < 0) ? -v : v);
  (void)snprintf(buf, len, "%c%lu.%0*lu", (v < 0) ? '-' : '+', m / scale, (int)decimals, m % scale);
  return buf;
}

/* Copy for the log: printable ASCII kept, line endings dropped, others '.'. */
static void printable(char *dst, size_t len, const uint8_t *src, uint32_t n)
{
  uint32_t j = 0U;
  for (uint32_t i = 0U; (i < n) && (j + 1U < len); i++)
  {
    uint8_t c = src[i];
    if ((c == '\r') || (c == '\n'))
    {
      continue;
    }
    dst[j++] = ((c >= 0x20U) && (c < 0x7FU)) ? (char)c : '.';
  }
  dst[j] = '\0';
}

/* ---- IMU ------------------------------------------------------------------ */

static bool imu_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t n)
{
  return HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(addr << 1), reg, I2C_MEMADD_SIZE_8BIT, buf, n,
                          IMU_I2C_TIMEOUT_MS) == HAL_OK;
}

static bool imu_write(uint8_t addr, uint8_t reg, uint8_t v)
{
  return HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(addr << 1), reg, I2C_MEMADD_SIZE_8BIT, &v, 1U,
                           IMU_I2C_TIMEOUT_MS) == HAL_OK;
}

/* Try SA0 low, then high; configure the first that answers with 6Ch. */
static void imu_probe(void)
{
  static const uint8_t addrs[2] = { IMU_ADDR_SA0_LOW, IMU_ADDR_SA0_HIGH };
  for (uint32_t i = 0U; i < 2U; i++)
  {
    uint8_t id = 0U;
    bool ok = imu_read(addrs[i], IMU_REG_WHO_AM_I, &id, 1U);
    debug_log("IMU probe 0x%02X: %s WHO_AM_I=0x%02X %s", addrs[i], ok ? "ACK" : "no ACK", id,
              (ok && (id == IMU_WHO_AM_I_VALUE)) ? "OK (LSM6DSOX)" : "");
    if (ok && (id == IMU_WHO_AM_I_VALUE))
    {
      bool cfg = imu_write(addrs[i], IMU_REG_CTRL3_C, IMU_CTRL3_C_BDU_IF_INC) &&
                 imu_write(addrs[i], IMU_REG_CTRL1_XL, IMU_CTRL1_XL_104HZ_2G) &&
                 imu_write(addrs[i], IMU_REG_CTRL2_G, IMU_CTRL2_G_104HZ_250);
      debug_log("IMU config 104 Hz, +/-2 g, +/-250 dps: %s", cfg ? "OK" : "FAIL");
      if (cfg)
      {
        s_imu_addr = addrs[i];
      }
      return;
    }
  }
}

static void imu_tick(uint32_t now_ms)
{
  if (s_imu_addr == 0U)
  {
    if ((now_ms % IMU_RETRY_MS) == 0U)
    {
      imu_probe();
    }
    return;
  }
  if ((now_ms % IMU_PERIOD_MS) != 0U)
  {
    return;
  }

  uint8_t b[IMU_DATA_BYTES];
  if (!imu_read(s_imu_addr, IMU_REG_OUT_TEMP_L, b, IMU_DATA_BYTES))
  {
    s_imu_errors++;
    debug_log("IMU read FAIL (errors %lu)", s_imu_errors);
    return;
  }
  s_imu_reads++;

  int16_t raw[7];   /* temp, gx, gy, gz, ax, ay, az; little-endian */
  for (uint32_t i = 0U; i < 7U; i++)
  {
    raw[i] = (int16_t)((uint16_t)b[2U * i] | ((uint16_t)b[(2U * i) + 1U] << 8));
  }
  char t[12];
  char g[3][12];
  char a[3][12];
  for (uint32_t i = 0U; i < 3U; i++)
  {
    (void)fmt_fixed(g[i], sizeof(g[i]), (float)raw[1U + i] * IMU_GYRO_MDPS_PER_LSB / 1000.0f, 1U);
    (void)fmt_fixed(a[i], sizeof(a[i]), (float)raw[4U + i] * IMU_ACC_MG_PER_LSB / 1000.0f, 3U);
  }
  (void)fmt_fixed(t, sizeof(t), IMU_TEMP_ZERO_C + ((float)raw[0] / IMU_TEMP_LSB_PER_C), 1U);
  debug_log("IMU acc g=%s/%s/%s gyro dps=%s/%s/%s T=%sC", a[0], a[1], a[2], g[0], g[1], g[2], t);
}

/* ---- UART ----------------------------------------------------------------- */

static void uart_arm_rx(void)
{
  if (HAL_UARTEx_ReceiveToIdle_IT(&huart2, s_uart_rx, UART_BUF) != HAL_OK)
  {
    s_uart_errors++;
  }
}

/* USART2 interrupt: a burst ended (idle line) or the buffer filled. */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  if (huart != &huart2)
  {
    return;
  }
  s_uart_rx_bytes += size;
  if (s_uart_line_len == 0U)
  {
    memcpy(s_uart_line, s_uart_rx, size);
    s_uart_line_len = (size > 0U) ? size : 0U;
  }
  else
  {
    s_uart_overruns++;
  }
  uart_arm_rx();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2)
  {
    s_uart_errors++;
    uart_arm_rx();
  }
}

static void uart_send(const char *text)
{
  if (huart2.gState != HAL_UART_STATE_READY)
  {
    s_uart_tx_busy++;
    return;
  }
  size_t n = strlen(text);
  if (n > (sizeof(s_uart_tx) - 1U))
  {
    n = sizeof(s_uart_tx) - 1U;
  }
  memcpy(s_uart_tx, text, n);
  if (HAL_UART_Transmit_IT(&huart2, (uint8_t *)s_uart_tx, (uint16_t)n) == HAL_OK)
  {
    s_uart_tx_count++;
  }
}

static void uart_tick(uint32_t now_ms)
{
  uint16_t n = s_uart_line_len;
  if (n != 0U)
  {
    char shown[UART_BUF + 1U];
    char echo[UART_TX_BUF];
    printable(shown, sizeof(shown), (const uint8_t *)s_uart_line, n);
    s_uart_line_len = 0U;   /* free for the next burst */
    debug_log("UART rx %u B: '%s'", (unsigned)n, shown);
    (void)snprintf(echo, sizeof(echo), "echo: %s\r\n", shown);
    uart_send(echo);
  }
  else if ((now_ms % UART_TX_PERIOD_MS) == 0U)
  {
    char line[UART_TX_BUF];
    (void)snprintf(line, sizeof(line), "G491 uart n=%lu t=%lums\r\n", s_uart_tx_count, now_ms);
    uart_send(line);
  }
}

/* ---- CAN ------------------------------------------------------------------ */

/* FDCAN2 interrupt line 0: queue every frame in RX FIFO 0 for the log. */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t its)
{
  if ((hfdcan != &hfdcan2) || ((its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U))
  {
    return;
  }
  FDCAN_RxHeaderTypeDef h;
  uint8_t d[8];
  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
  {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &h, d) != HAL_OK)
    {
      break;
    }
    s_can_rx_count++;
    uint32_t head = s_can_head;
    if ((head - s_can_tail) >= CAN_RX_QUEUE)
    {
      s_can_rx_dropped++;
      continue;
    }
    can_frame_t *f = &s_can_q[head & (CAN_RX_QUEUE - 1U)];
    f->id = h.Identifier;
    f->len = (h.DataLength <= FDCAN_DLC_BYTES_8) ? h.DataLength : 8U;   /* classic: DLC = bytes */
    memcpy(f->data, d, f->len);
    s_can_head = head + 1U;
  }
}

static bool can_send(uint32_t id, const uint8_t *data, uint32_t len)
{
  FDCAN_TxHeaderTypeDef h = {0};
  h.Identifier = id;
  h.IdType = FDCAN_STANDARD_ID;
  h.TxFrameType = FDCAN_DATA_FRAME;
  h.DataLength = len;                       /* FDCAN_DLC_BYTES_0..8 = 0..8 */
  h.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  h.BitRateSwitch = FDCAN_BRS_OFF;
  h.FDFormat = FDCAN_CLASSIC_CAN;
  h.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  if ((HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0U) ||
      (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &h, (uint8_t *)data) != HAL_OK))
  {
    s_can_tx_full++;
    return false;
  }
  s_can_tx_count++;
  return true;
}

static void can_init(void)
{
  /* No filters configured: accept every standard and extended data frame
   * into RX FIFO 0, reject remote frames. */
  s_can_ok = (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                                           FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) == HAL_OK) &&
             (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) == HAL_OK) &&
             (HAL_FDCAN_Start(&hfdcan2) == HAL_OK);
  debug_log("CAN fdcan2 classic 500k: %s; tx id 0x%03X every %u ms, rx all, 0x%03X -> reply 0x%03X",
            s_can_ok ? "started" : "FAIL", CAN_TX_ID, CAN_TX_PERIOD_MS, CAN_REQ_ID, CAN_REPLY_ID);
}

static void can_tick(uint32_t now_ms)
{
  while (s_can_tail != s_can_head)
  {
    const can_frame_t *f = &s_can_q[s_can_tail & (CAN_RX_QUEUE - 1U)];
    char hex[3U * 8U + 1U];
    uint32_t j = 0U;
    for (uint32_t i = 0U; i < f->len; i++)
    {
      j += (uint32_t)snprintf(&hex[j], sizeof(hex) - j, "%02X ", f->data[i]);
    }
    hex[j] = '\0';
    debug_log("CAN rx id=0x%03lX len=%lu data=%s", f->id, f->len, hex);
    if (f->id == CAN_REQ_ID)
    {
      (void)can_send(CAN_REPLY_ID, f->data, f->len);
    }
    s_can_tail++;
  }

  if (s_can_ok && ((now_ms % CAN_TX_PERIOD_MS) == 0U))
  {
    uint8_t d[8];
    uint32_t n = s_can_tx_count;
    memcpy(&d[0], &n, 4U);        /* counter, little-endian */
    memcpy(&d[4], &now_ms, 4U);   /* uptime in ms */
    (void)can_send(CAN_TX_ID, d, 8U);
  }
}

/* ---- USB CDC -------------------------------------------------------------- */

void comms_test_usb_rx(const uint8_t *buf, uint32_t len)
{
  s_usb_rx_bytes += len;
  if (s_usb_rx_len == 0U)
  {
    uint32_t n = (len > USB_BUF) ? USB_BUF : len;
    memcpy(s_usb_rx, buf, n);
    s_usb_rx_len = n;
  }
}

/* CDC_Transmit_FS dereferences the class data, which exists only once the
 * PC has configured the device: never call it before. */
static bool usb_send(const char *text)
{
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
  {
    return false;
  }
  size_t n = strlen(text);
  if (n > sizeof(s_usb_tx))
  {
    n = sizeof(s_usb_tx);
  }
  memcpy(s_usb_tx, text, n);
  if (CDC_Transmit_FS((uint8_t *)s_usb_tx, (uint16_t)n) != USBD_OK)
  {
    return false;   /* previous transfer still in progress, or no terminal reading */
  }
  s_usb_tx_count++;
  return true;
}

static void usb_tick(uint32_t now_ms)
{
  uint8_t st = hUsbDeviceFS.dev_state;
  if (st != s_usb_state)
  {
    debug_log("USB state %u -> %u (%s)", s_usb_state, st,
              (st == USBD_STATE_CONFIGURED) ? "configured: COM port ready" : "not configured");
    s_usb_state = st;
  }

  uint32_t n = s_usb_rx_len;
  if (n != 0U)
  {
    char shown[USB_BUF + 1U];
    char echo[USB_BUF + 16U];
    printable(shown, sizeof(shown), s_usb_rx, n);
    s_usb_rx_len = 0U;
    debug_log("USB rx %lu B: '%s'", n, shown);
    (void)snprintf(echo, sizeof(echo), "echo: %s\r\n", shown);
    (void)usb_send(echo);
  }
  else if ((now_ms % USB_TX_PERIOD_MS) == 0U)
  {
    char line[USB_BUF];
    (void)snprintf(line, sizeof(line), "G491 usb n=%lu t=%lums\r\n", s_usb_tx_count, now_ms);
    (void)usb_send(line);
  }
}

/* ---- API ------------------------------------------------------------------ */

void comms_test_init(void)
{
  debug_log("COMMS test: I2C1 IMU, USART2 115200 (J15), FDCAN2 500k (J10), USB CDC (J1)");
  imu_probe();
  uart_arm_rx();
  debug_log("UART usart2 115200 8N1: tx every %u ms, rx logged and echoed", UART_TX_PERIOD_MS);
  can_init();
  s_usb_state = hUsbDeviceFS.dev_state;
  debug_log("USB cdc: state %u, plug into the PC and open the COM port", s_usb_state);
}

void comms_test_tick(uint32_t now_ms)
{
  imu_tick(now_ms);
  uart_tick(now_ms);
  can_tick(now_ms);
  usb_tick(now_ms);

  if ((now_ms % STATUS_PERIOD_MS) == 0U)
  {
    FDCAN_ProtocolStatusTypeDef ps;
    FDCAN_ErrorCountersTypeDef ec;
    (void)HAL_FDCAN_GetProtocolStatus(&hfdcan2, &ps);
    (void)HAL_FDCAN_GetErrorCounters(&hfdcan2, &ec);
    debug_log("UART tx=%lu rx=%lu B err=%lu ovr=%lu busy=%lu | IMU reads=%lu err=%lu addr=0x%02X",
              s_uart_tx_count, s_uart_rx_bytes, s_uart_errors, s_uart_overruns, s_uart_tx_busy,
              s_imu_reads, s_imu_errors, s_imu_addr);
    debug_log("CAN tx=%lu full=%lu rx=%lu drop=%lu tec=%lu rec=%lu passive=%lu busoff=%lu lec=%lu | USB tx=%lu rx=%lu B",
              s_can_tx_count, s_can_tx_full, s_can_rx_count, s_can_rx_dropped, ec.TxErrorCnt, ec.RxErrorCnt,
              ps.ErrorPassive, ps.BusOff, ps.LastErrorCode, s_usb_tx_count, s_usb_rx_bytes);
  }
}

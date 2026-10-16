/* Host-only protocol tests. No STM32 headers/hardware, no changes to target build.
 * See docs/UART-Diagnostics.md for the developer-run compile command. */
#include "diag.h"
#include "diag_uart.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static int rx[8192];
static size_t head, tail;
static char tx[32768];
static size_t tx_len;
static uint32_t now, commands, settings, last_command, pending;
static int32_t last_value;
static bool fail_tx;

bool diag_uart_init(void) { return true; }
int diag_uart_read(void) { return tail == head ? -1 : rx[tail++]; }
bool diag_uart_write(const char *data, size_t len, size_t reserve)
{
  (void)reserve;
  if (fail_tx) { return false; }
  assert(tx_len + len < sizeof(tx));
  memcpy(tx + tx_len, data, len);
  tx_len += len;
  tx[tx_len] = '\0';
  return true;
}
void diag_uart_stats(diag_uart_stats_t *out)
{
  memset(out, 0, sizeof(*out));
  out->tx_pending = pending;
}
static const char *command(uint32_t n) { commands++; last_command = n; return NULL; }
static const char *setting(const char *name, int32_t value)
{
  assert(strcmp(name, "vq_mv") == 0);
  settings++; last_value = value; return NULL;
}
static void snapshot(char *out, size_t size) { (void)snprintf(out, size, "armed=0"); }
static void feed(const char *s)
{
  while (*s) { assert(head < sizeof(rx)/sizeof(rx[0])); rx[head++] = (unsigned char)*s++; }
}
static void drain(void) { while (tail != head) { diag_poll(++now); } }
static void clear_output(void) { tx_len = 0U; tx[0] = '\0'; }

int main(void)
{
  const diag_ops_t ops = {command, setting, snapshot};
  assert(diag_init(&ops));
  feed("cmd 10"); drain(); assert(commands == 0U);
  feed("\r\n"); drain(); assert(commands == 1U && last_command == 10U);
  assert(strstr(tx, "QUEUED") != NULL);
  feed("set vq_mv -500\n"); drain(); assert(settings == 1U && last_value == -500);
  feed("cmd 10junk\ncmd -1\ncmd 2147483648\ncmd 1 extra\nset vq_mv NaN\n");
  drain(); assert(commands == 1U && settings == 1U);
  feed("cmd 1\r0\n"); drain(); assert(commands == 1U); /* embedded CR must not become cmd 10 */
  feed("cmd 1\ncmd 2\n");
  diag_poll(++now); assert(commands == 2U);
  diag_poll(now); assert(commands == 2U); /* same millisecond does no additional work */
  drain(); assert(commands == 3U && last_command == 2U);
  for (unsigned i = 0; i < 120; i++) { feed("x"); }
  feed("cmd 10\n"); drain(); assert(commands == 3U); /* suffix of long line not executed */
  feed("cmd "); drain(); rx[head++] = -2; feed("10\n"); drain(); assert(commands == 3U);
  feed("cmd 10\n"); drain(); assert(commands == 4U); /* delimiter resynchronization */
  feed("cmd "); drain(); now += 1001U; diag_poll(now);
  feed("10\n"); drain(); assert(commands == 4U); /* expired partial line rejected */
  rx[head++] = 0; feed("cmd 10\n"); drain(); assert(commands == 4U);
  feed("logs 0\n"); drain(); clear_output(); diag_log("test", 4U); assert(tx_len == 0U);
  feed("logs 1\n"); drain(); clear_output(); diag_log("test", 4U); assert(strcmp(tx, "LOG test\n") == 0);
  feed("stream 50\n"); drain(); clear_output();
  pending = 200U; now += 500U; diag_poll(now); assert(strstr(tx, "TEL ") == NULL);
  pending = 0U; now += 50U; diag_poll(now); assert(strstr(tx, "TEL v=1") != NULL);
  clear_output(); diag_poll(now); assert(tx_len == 0U);
  now += 500U; diag_poll(now);
  char *first = strstr(tx, "TEL "); assert(first && !strstr(first + 1, "TEL ")); /* no catch-up */
  fail_tx = true; now += 50U; diag_poll(now); /* dropped transmission never blocks */
  puts("diag protocol tests passed");
  return 0;
}

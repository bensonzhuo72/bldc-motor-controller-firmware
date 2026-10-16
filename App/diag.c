#include "diag.h"
#include "diag_uart.h"
#include "app_config.h"
#include <stdio.h>
#include <string.h>

/* Protocol v1 budgets: bounded ASCII commands; no dynamic allocation.
 * Responses reserve space ahead of lossy logs; telemetry never catches up. */
#define LINE_SIZE 96U
#define RECORD_SIZE 512U
#define SNAPSHOT_SIZE 384U
#define RX_BUDGET 32U
#define RESPONSE_RESERVE 640U
#define STREAM_MIN_MS 50U
#define STREAM_MAX_MS 60000U
#define PARTIAL_TIMEOUT_MS 1000U
#define TELEMETRY_BACKLOG_MAX 128U /* do not queue a dashboard snapshot behind bulk output */
static const diag_ops_t *s_ops;
static bool s_ready, s_logs = true, s_discard, s_cr;
static char s_line[LINE_SIZE];
static uint32_t s_len, s_poll_ms, s_byte_ms, s_stream_ms, s_stream_at;
static uint32_t s_request, s_sample, s_bad_lines, s_skipped;

static bool integer(const char *s, int32_t *out)
{
  if (!s || !*s) { return false; }
  bool negative = *s == '-';
  if (negative) { s++; }
  if (!*s) { return false; }
  uint32_t v = 0U;
  /* Bounded protocol integer range; sufficient for engineering-unit settings. */
  for (; *s; s++)
  {
    if (*s < '0' || *s > '9' || v > 214748364U) { return false; }
    uint32_t digit = (uint32_t)(*s - '0');
    if (v == 214748364U && digit > 7U) { return false; }
    v = v * 10U + digit;
  }
  *out = negative ? -(int32_t)v : (int32_t)v;
  return true;
}

static void reply(const char *result, const char *detail)
{
  char record[RECORD_SIZE];
  int n = snprintf(record, sizeof(record), "RSP v=1 seq=%lu %s %s\n",
                   (unsigned long)s_request, result, detail);
  if (n > 0 && (size_t)n < sizeof(record))
  {
    (void)diag_uart_write(record, (size_t)n, 0U);
  }
}

static void execute(char *line, uint32_t now)
{
  char *args[4];
  unsigned count = 0U;
  char *p = line;
  while (*p == ' ' || *p == '\t') { p++; }
  if (!*p) { return; }
  s_request++;
  while (*p)
  {
    while (*p == ' ' || *p == '\t') { p++; }
    if (!*p) { break; }
    if (count == 4U) { reply("ERR", "syntax"); return; }
    args[count++] = p;
    while (*p && *p != ' ' && *p != '\t') { p++; }
    if (*p) { *p++ = '\0'; }
  }
  if (!count) { return; }
  int32_t value;
  if (count == 1U && strcmp(args[0], "help") == 0)
  {
    reply("OK", "help | ping | status | stats | cmd N | set NAME INTEGER | logs 0/1 | stream MS(0=off,50..60000)");
  }
  else if (count == 1U && strcmp(args[0], "ping") == 0) { reply("OK", "pong"); }
  else if (count == 1U && strcmp(args[0], "status") == 0)
  {
    char snapshot[SNAPSHOT_SIZE];
    s_ops->snapshot(snapshot, sizeof(snapshot));
    reply("OK", snapshot);
  }
  else if (count == 1U && strcmp(args[0], "stats") == 0)
  {
    diag_uart_stats_t st;
    char text[256];
    diag_uart_stats(&st);
    (void)snprintf(text, sizeof(text),
        "tx=%lu rx=%lu tx_drop=%lu rx_ovf=%lu rx_err=%lu rx_stale=%lu pending=%lu bad=%lu tel_skip=%lu",
        (unsigned long)st.tx_bytes, (unsigned long)st.rx_bytes,
        (unsigned long)st.tx_dropped, (unsigned long)st.rx_overflow,
        (unsigned long)st.rx_errors, (unsigned long)st.rx_stale, (unsigned long)st.tx_pending,
        (unsigned long)s_bad_lines, (unsigned long)s_skipped);
    reply("OK", text);
  }
  else if (count == 2U && strcmp(args[0], "cmd") == 0 && integer(args[1], &value) && value >= 0)
  {
    const char *err = s_ops->command((uint32_t)value);
    reply(err ? "ERR" : "QUEUED", err ? err : "check LOG/status for execution result");
  }
  else if (count == 3U && strcmp(args[0], "set") == 0 && integer(args[2], &value))
  {
    const char *err = s_ops->set(args[1], value);
    reply(err ? "ERR" : "OK", err ? err : "setting accepted");
  }
  else if (count == 2U && strcmp(args[0], "logs") == 0 && integer(args[1], &value) && (value == 0 || value == 1))
  {
    s_logs = value != 0;
    reply("OK", s_logs ? "logs=1" : "logs=0");
  }
  else if (count == 2U && strcmp(args[0], "stream") == 0 && integer(args[1], &value) &&
           (value == 0 || (value >= (int32_t)STREAM_MIN_MS && value <= (int32_t)STREAM_MAX_MS)))
  {
    s_stream_ms = (uint32_t)value;
    s_stream_at = now;
    reply("OK", "stream interval accepted; best effort");
  }
  else { s_bad_lines++; reply("ERR", "syntax/range; use help"); }
}

bool diag_init(const diag_ops_t *ops)
{
  s_ops = ops;
  s_ready = ops && ops->command && ops->set && ops->snapshot && diag_uart_init();
  return s_ready;
}

void diag_log(const char *line, size_t len)
{
  if (!s_ready || !s_logs) { return; }
  /* Matches debug's <=127-byte line; reject oversize instead of partial frames. */
  char record[160];
  if (len + 5U > sizeof(record)) { return; }
  memcpy(record, "LOG ", 4U);
  memcpy(record + 4U, line, len);
  record[len + 4U] = '\n';
  (void)diag_uart_write(record, len + 5U, RESPONSE_RESERVE);
}

void diag_poll(uint32_t now_ms)
{
  if (!s_ready || now_ms == s_poll_ms) { return; }
  s_poll_ms = now_ms;
  if (s_len && now_ms - s_byte_ms > PARTIAL_TIMEOUT_MS)
  {
    s_len = 0U;
    s_discard = true;
    s_bad_lines++;
  }
  for (uint32_t i = 0U; i < RX_BUDGET; i++)
  {
    int c = diag_uart_read();
    if (c == -1) { break; }
    if (c == -2)
    {
      s_discard = true;
      s_len = 0U;
      s_bad_lines++;
      continue;
    }
    s_byte_ms = now_ms;
    if (c == '\r' && !s_cr) { s_cr = true; continue; } /* CR must be followed by LF */
    if (s_cr && c != '\n') { s_discard = true; s_bad_lines++; }
    s_cr = false;
    if (c == '\n')
    {
      if (s_discard)
      {
        s_discard = false;
        reply("ERR", "RX line discarded; resend complete command");
      }
      else
      {
        s_line[s_len] = '\0';
        execute(s_line, now_ms);
      }
      s_len = 0U;
      break; /* at most one command per real millisecond */
    }
    if (s_discard) { continue; }
    if ((c < 32 && c != '\t') || c > 126 || s_len == LINE_SIZE - 1U)
    {
      s_discard = true;
      s_len = 0U;
      s_bad_lines++;
      continue;
    }
    s_line[s_len++] = (char)c;
  }
  if (s_stream_ms && now_ms - s_stream_at >= s_stream_ms)
  {
    s_stream_at = now_ms; /* no catch-up burst */
    s_sample++;
    diag_uart_stats_t st;
    diag_uart_stats(&st);
    /* Never add stale telemetry behind a log/capture backlog. */
    if (st.tx_pending > TELEMETRY_BACKLOG_MAX) { s_skipped++; return; }
    char snapshot[SNAPSHOT_SIZE], record[RECORD_SIZE];
    s_ops->snapshot(snapshot, sizeof(snapshot));
    int n = snprintf(record, sizeof(record), "TEL v=1 seq=%lu ms=%lu %s\n",
                     (unsigned long)s_sample, (unsigned long)now_ms, snapshot);
    if (n <= 0 || (size_t)n >= sizeof(record) ||
        !diag_uart_write(record, (size_t)n, RESPONSE_RESERVE)) { s_skipped++; }
  }
}

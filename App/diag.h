#ifndef DIAG_H
#define DIAG_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Protocol knows no motor globals. Extend the application adapter for new
 * commands/settings/fields; keep motor policy outside the byte transport. */
typedef struct
{
  const char *(*command)(uint32_t command); /* NULL = queued, else refusal reason */
  const char *(*set)(const char *name, int32_t value);
  void (*snapshot)(char *out, size_t size);
} diag_ops_t;

bool diag_init(const diag_ops_t *ops);
void diag_poll(uint32_t now_ms); /* main only; once per real ms; bounded work */
void diag_log(const char *line, size_t len); /* mirror existing debug_log lines */
#endif

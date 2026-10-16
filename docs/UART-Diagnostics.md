# UART diagnostics and command protocol v1

Implemented for Nathaniel Su on `post-12b-uartdev`, 2026-10-16. **Source-reviewed only; build, host tests and board validation pending.** Stage 12 and its existing motor limits remain selected.

## Connect and inspect

Use the existing USART2 connection on J15: **PA2 = MCU TX, PA3 = MCU RX**, crossed to the adapter/Teensy RX/TX with common ground and appropriate 3.3 V logic. The existing configuration is **115200 baud, 8 data bits, no parity, 1 stop bit, no flow control**. No CubeMX changes are required for this checkout. UART TX reception was previously unresolved on hardware; implementation does not establish that it now works.

Send ASCII commands terminated by **LF** or **CRLF**. CR alone does not execute a command. Start with:

```text
help
ping
status
stats
logs 0
stream 250
```

`logs 0` suppresses UART log mirroring only; SWO stays available. `stream 250` requests a best-effort snapshot every 250 ms. Streaming is **off at boot**. `stream 0` stops it; `logs 1` restores mirroring.

SWO itself is an output channel. Existing commands were debugger writes to `g_app_cmd`, not inbound SWO packets. UART queues the same application commands and mirrors the existing `debug_log` text rather than introducing another motor-control implementation.

## Commands

Send **one request at a time**, consume its response, then send the next. `QUEUED` means the request reached the existing command mailbox; it does not mean the motor armed or the action succeeded. Read the subsequent `LOG` lines or query `status` for the actual result. Existing arm guards, zero start, ramps, limits and latched faults still apply.

| Request | Meaning |
| --- | --- |
| `help` | Protocol command summary |
| `ping` | `OK pong` |
| `status` | One dashboard/state snapshot |
| `stats` | Transport loss/error/backlog counters |
| `cmd 0` | Explicit fault-clear request; disarm first; no automatic arm |
| `cmd 1` | Existing arm command (at Stage 12: alignment/forced spin, **not** voltage FOC) |
| `cmd 2` | Existing disarm command; may replace a pending command |
| `cmd 3` | Existing software-break test |
| `cmd 6` | Capture trigger |
| `cmd 7` | Capture dump; must have a finished capture |
| `cmd 9` | Existing electrical alignment procedure |
| `cmd 10` | Voltage-mode FOC arm |
| `set NAME INTEGER` | Allowlisted parameter, using units below |
| `logs 0` / `logs 1` | Disable/enable UART mirror of ordinary SWO lines |
| `stream MS` | 0 disables; otherwise 50–60,000 ms, best effort |

Commands unavailable at the selected stage return an error. Earlier diagnostic stages retain their supported command numbers (including 4/5 bridge tools and Stage 8 command 8). There is no arbitrary memory/register-write command.

| Setting | Units / accepted range | Behavior |
| --- | --- | --- |
| `vq_mv` | signed mV, -2000..2000 | Requires `vq_pot=0`; existing Vq ramp remains in force |
| `vq_pot` | 0 or 1 | Disarmed only; changing source first resets the stored manual Vq to zero |
| `oc_ma` | mA, 100..1500 | Sets the existing phase-current trip request; default startup/fallback behavior is unchanged |
| `vec_mv` | mV, 0..1200 | Existing stationary/open-loop vector amplitude |
| `spin_mhz` | **millihertz**, -10000..10000 | Existing forced electrical frequency, not mechanical rpm |
| `cap_auto_ms` | ms, 0..60000 | Disarmed only; 0 disables the existing automatic FOC capture schedule |

Settings are stage-gated and refused while an application command/clear is pending. Capture format is deliberately not writable through UART: `set cap_dq ...` returns an error because changing it can mix ring history and labels. Select `g_cap_dq` through the existing debugger setup before the test and refill the capture as documented in the plan.

A manual-voltage session would select `set vq_pot 0` while disarmed, then separately request FOC with `cmd 10`; only after verifying the arm result would it change `vq_mv` within the accepted test envelope. `cmd 2` disarms. These are explicit motor commands, not actions performed automatically on connection.

## Output framing and dashboard fields

All records end with LF. Parsers should tolerate additional keys and unknown record types for future versions.

```text
LOG <unchanged existing SWO log text>
RSP v=1 seq=<request counter> OK|ERR|QUEUED <detail or key=value fields>
TEL v=1 seq=<sample-attempt counter> ms=<uptime> <key=value fields>
```

`RSP seq` is a device-generated counter, not a host transaction ID or replay token. Do not blindly retry an arm/alignment/action after a missing reply; query state first. There is no automatic retransmission, checksum, authentication, persistent session or exactly-once execution guarantee in v1. This is a local bench diagnostic protocol, not a safety-rated stop channel. Disconnecting UART does not disarm the motor or change an existing command; use the established hardware controls/protection paths for immediate shutdown.

| Snapshot key | Interpretation |
| --- | --- |
| `stage`, `armed`, `fault` | Selected stage, PWM armed flag, `fault_code_t` numeric code (0 = none); source enum is in `fault.h` |
| `vm_mv` | Measured bus voltage in mV |
| `rpm` | Signed **encoder-direction mechanical** rpm; with DIR=-1, its sign differs from the positive-Vq convention in the existing FOC log |
| `valid` | Bit 0: encoder healthy; bit 1: calibrated currents and configured driver. These are prerequisites, not a certification that every field is fresh |
| `ia_ma`, `ib_ma`, `ic_ma` | Latest phase samples in mA; ignore when current-valid bit is clear |
| `id_ma`, `iq_ma`, `dq_n`, `dq_age_ms` | Cached one-second dq means, number of accepted samples and age since statistics update. Zero accepted samples means unavailable. These are **not** newly measured values at the requested stream rate |
| `vq_mv`, `vq_target_mv`, `pot` | Last ramped FOC voltage command/target in mV and pot/manual selector; these are commands, not measured terminal voltages, and are not the voltage command for open-loop mode |
| `trip_ma` | Last applied phase-trip setting, mA |
| `swo_drop` | Normal SWO queue-full line drops |

Snapshots copy existing encoder/current state using their existing brief critical sections; different fields are not sampled simultaneously. The dashboard must consider age, validity, fault and operating mode. Dq statistics are updated by the normal once-per-second task, not consumed/reset by a dashboard request. In `COMMS_TEST` mode that normal logging/statistics task can be suppressed; dq will remain absent or stale. `snapshot_error=truncated` replaces an oversized snapshot instead of emitting a broken record.

`stats` returns byte counts (`tx`, `rx`), dropped TX records (`tx_drop`), dropped RX bytes (`rx_ovf`), RX error events (`rx_err`), stale-buffer discards (`rx_stale`), queued TX bytes (`pending`), parser/discard events (`bad`) and skipped telemetry attempts (`tel_skip`). Counters are approximate individual 32-bit reads and wrap naturally. Sequence gaps and loss counters are expected under overload; missing packets must not be interpreted as measured zeros.

## Scheduling and load policy

- No UART code is added to the 20 kHz ADC/control ISR. The existing USART2 IRQ priority is 6; ADC/control is priority 0. Motor control can preempt UART work.
- The UART IRQ handles at most one RX byte and one TX byte per invocation, using vendor LL/direct register access. No parsing, formatting, HAL transfers, motor commands or logging occur there. No new global interrupt masking is used by the transport.
- Main-loop parsing runs after due supervision/application tasks, at most once per actual millisecond, with a 32-byte budget and at most one complete command. Missed telemetry intervals are skipped, never replayed in a burst.
- UART TX has a 2,048-byte ring. Logs/telemetry leave 640 bytes reserved for responses. Records are enqueued whole or dropped; never wait for serial capacity. Responses share the FIFO and cannot jump ahead of already queued bytes. Repeated requests can exhaust even reserved capacity, so response delivery is not guaranteed under a flood.
- UART RX has a 256-byte ring plus receive timestamps. Overflow or UART framing/noise/overrun errors flush queued input and discard through the next LF. Buffered bytes older than 250 ms are rejected; partial command lines time out after 1 s between serviced bytes. Overlong (>95-character), non-ASCII/control-byte, malformed integer and embedded-CR input cannot execute a truncated suffix. Send a blank LF to resynchronize after an error, then a complete fresh command.
- Streaming is limited to a minimum 50 ms request interval and skipped when TX backlog exceeds 128 bytes. It is designed to drop data rather than chase stable 100 Hz telemetry. At 115200 8N1, maximum wire capacity is approximately 11,520 B/s before sharing with logs and responses.
- Normal SWO logging uses a separate 1,024-byte ring and checks ITM readiness without waiting. Each drain handles at most 64 bytes. Fatal HardFault/NMI raw SWO handling is unchanged and is **not mirrored to UART**.
- No heap or DMA is required. Fixed transport/SWO buffers use about 4.25 KiB, plus protocol state and bounded formatting stack. Confirm actual RAM/stack/timing in the target build; there is no measured latency guarantee yet. New transport/protocol sources compile at `-O2` even in Debug.

Large `CAP` dumps can exceed serial bandwidth. Keep the indexed samples and verify all 1,024 are present before treating a dump as complete; retry a fresh acquisition or use the existing debugger RAM capture if records were dropped. The capture re-arms according to its existing behavior, so a lossy UART dump is not a lossless download of a permanently retained buffer.

## Files and extension points

| File | Responsibility |
| --- | --- |
| `App/diag_uart.c/h` | USART2 rings, bounded IRQ, errors, counters; no motor dependencies |
| `App/diag.c/h` | ASCII framing, parser, responses, log prefix and best-effort scheduling |
| `App/app.c` diagnostic adapter | Command allowlist, parameter units/limits, dashboard state and existing mailbox integration |
| `App/debug.c/h` | Existing formatted logger fan-out and nonblocking normal SWO queue |
| `Core/Src/stm32g4xx_it.c` USER CODE blocks | UART IRQ dispatch to standalone transport |
| `App/app_config.h` | `UART_DIAG_ENABLE`; compile-time rejection of simultaneous legacy UART-test ownership |

Set `UART_DIAG_ENABLE=0` to return USART2 IRQ handling to the generated HAL/legacy test path. To run `COMMS_TEST_UART`, disable diagnostics first; the conflicting combination is a compile error. The normal SWO nonblocking queue remains enabled independently.

Add motor features in the adapter and extend its explicit setting/command allowlists. Extend dashboard keys while keeping schema/version/units documented. A later binary/CRC protocol or DMA transport can replace the corresponding layer without moving motor policy into the ISR. Any DMA/FIFO/baud/NVIC configuration change belongs in CubeMX. Do not assume adding a setting bypasses the existing controller guards or checkoff requirements.

## Developer verification

The agent did not run builds, tests, scripts or hardware operations under `CLAUDE.md`. The host test source exercises framing, fragmented/CRLF commands, invalid/range/extra-token input, overlong-line and RX-gap recovery, partial timeout, log toggling, per-ms command budget and dropped/no-catch-up telemetry. It mocks the byte transport; it does not prove ring concurrency, register behavior, application safety or physical timing.

With a host C compiler, from the repository root:

```text
cc -std=c11 -Wall -Wextra -Werror -IApp tests/diag_protocol_test.c App/diag.c -o diag_protocol_test
```

Run the resulting executable on the host; expect `diag protocol tests passed`. Build/flash the target separately through the normal developer workflow. Use the UART section of [Stage-12B-Checkoff.md](Stage-12B-Checkoff.md) for the board checks, including continuous RX/TX stress, control ISR timing, malformed-command recovery, limits and arm/fault behavior. No new stage pass is asserted.

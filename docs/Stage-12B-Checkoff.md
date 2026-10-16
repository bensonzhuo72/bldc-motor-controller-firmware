# Stage 12B checkoff

**Tick a box when it passes. No status codes or per-test form needed.** In a Markdown editor, change `[ ]` to `[x]`. Leave failed or unfinished tests unticked. Tell Codex your results to have them saved in the bring-up log.

Build/board (optional, fill once): ____________________

Keep existing limits: nominal 12 V, |Vq| <= 2 V, nominal phase trip 1.5 A, duty <= 93%. Follow the [original test procedures](BLDC%20Controller%20Firmware%20Bring-Up%20Test%20Plan.md). The trip threshold is not a continuous-current rating.

## Startup and arming

- [ ] Latest firmware builds and the correct ELF is flashed.
- [ ] Boot shows Stage 12; startup and DRV configuration checks pass.
- [ ] Fresh current offsets report OK after driver wake.
- [ ] Arming starts at zero command and voltage ramps smoothly.
- [ ] Normal disarm works; re-arming requires a separate command.
- [ ] Missing VM, invalid offsets/encoder, unconfigured driver or latched fault prevent arming.
- [ ] Driver wake repeats configuration and offset calibration.

## Protection

Use the original Stage 8 reduced-threshold procedure, then restore 1.5 A. Reuse unaffected prior evidence explicitly; repeat affected checks.

- [ ] Software overcurrent shuts PWM off within one 50 us control cycle.
- [ ] PWM goes off before ENABLE goes low; shutdown sequence meets the 40 us requirement.
- [ ] SWO reports the correct overcurrent phase/current and latched fault, without double-counting.
- [ ] Driver stays asleep and PWM stays off while the fault is latched.
- [ ] Fault clear leaves PWM disarmed; separate zero-command arm required.
- [ ] Overcurrent threshold restored to 1.5 A.
- [ ] Prior nFAULT/break pass still applies, or the original test was repeated successfully.
- [ ] Encoder-loss, heartbeat and debugger-stop protection evidence still applies, or affected checks were repeated.
- [ ] Power/enable removal disarms; restoration does not automatically re-arm.

## Remaining bridge checks

Use the Stage 6 diagnostic setup: **motor disconnected, 12 V, supply limit 0.5 A**. Identify any earlier-stage diagnostic build and return to Stage 12 afterward.

- [ ] Falling-edge dead time checked: no overlap; clean gate/switch-node behavior.
- [ ] Switch-node average voltage follows duty × actual VM over the 5–93% sweep.
- [ ] PWM-on/off supply-current difference is explained; no unexpected DC load or faults.

## Current sensing and captures

- [ ] Driver awake and calibrated: zero-current readings/noise acceptable; offsets pass 2048 +/-250 counts.
- [ ] Driver off or offsets invalid: dq shows n=0, increasing bad, and zero extrema for an empty window.
- [ ] Valid dq readings resume after calibration with a healthy encoder.
- [ ] Phase mapping/signs correct; Ia+Ib+Ic approximately zero; scaling consistent with stationary vectors.
- [ ] Sampling remains settled and current readings consistent up to 93% duty.
- [ ] No unexplained ADC late samples over the test interval.
- [ ] Raw capture works: 1,024 samples, 50 us spacing, trigger boundary at index 512.
- [ ] Dq capture agrees with telemetry; invalid 0 0 pairs excluded.
- [ ] Automatic capture works without pausing the debugger while armed.
- [ ] Fault capture contains the trip sample and expected post-trip behavior.
- [ ] Full ISR maximum below 25 us with capture/logging active; main-loop delays acceptable.

Capture reminders: choose raw/dq before testing, allow 60 ms of fresh history, and do not change format mid-capture/dump. Raw current = (count-offset) × 0.00402832 A. Dq current = (word-32768)/1000 A; angle = word × 2π/65536. CUR rms excludes DC. PA2/PA3 are no longer timing outputs; use current cycle-count telemetry.

## Voltage FOC and encoder

- [ ] Smooth forward rotation at Vq = +0.5, +1 and +2 V.
- [ ] Smooth reverse rotation at Vq = -0.5, -1 and -2 V.
- [ ] Speed roughly follows Vq (about 106 rpm/V no-load, allowing for losses).
- [ ] Iq sign appropriate in both directions; current responds appropriately to bounded load.
- [ ] Id and ripple physically reasonable; unexplained ripple resolved.
- [ ] VM stays below about 14 V during ramped Vq reductions.
- [ ] Encoder healthy while running; no unexplained CRC/jump/loss-of-track errors.
- [ ] Existing direction/offset still applies, or Stage 10 repeated after leads/magnet/encoder moved.

Shadow mode does not require Id exactly zero. Agree unspecified noise/ripple tolerances before judging results. No firmware overvoltage trip is implied.

## Load rig

Follow [characterization procedures C0/C1](Motor-Controller-Characterization-Plan.md). If unavailable/unsuitable, leave tests unticked and note an explicit deferral below.

- [ ] Generator/load/scale ratings, polarity, fixture and temperature monitoring checked.
- [ ] Torque arm/scale calibrated; tare, zero return, friction, repeatability and uncertainty checked.
- [ ] Generator/electronic load regulate within the existing speed/current envelope.
- [ ] Open-circuit drag baseline measured separately from fixture tare.
- [ ] Repeatable bounded loaded points measured: speed, Vq, Id/Iq, phase current, VM, generator V/I, torque and temperatures.

## Auxiliary hardware

Test or explicitly defer below; a deferral is not a pass.

- [ ] THROTTLE2 responds correctly.
- [ ] MCU UART TX received by Teensy.
- [ ] External CAN communication works, beyond loopback.
- [ ] USB orientation issue resolved; enumeration, echo and reliability checked in supported orientations.

## Status LED

- [ ] Disarmed/waiting: fast blink, 5 Hz (100 ms on/off).
- [ ] Armed: solid on.
- [ ] Normal disarm: fast blink again.
- [ ] Latched controller fault: slow blink, 1 Hz (500 ms on/off), until cleared.
- [ ] Fault clear: waiting indication, no automatic arm.

Fatal HardFault/NMI halt/blink behavior remains separate and unchanged.

## UART diagnostics branch regression

Added 2026-10-16 for the authorized UART feature on `post-12b-uartdev`; not a claim of Stage 12B acceptance. Follow [UART-Diagnostics.md](UART-Diagnostics.md) for commands, limits and test instructions.

- [ ] Host protocol tests build and pass.
- [ ] Target builds with diagnostics enabled; boot reports DIAG USART2 OK.
- [ ] PA2 TX / PA3 RX work both ways at 115200 8N1: help, ping, status and stats respond.
- [ ] Existing normal SWO output is mirrored as LOG records; logs 0/1 affects UART only.
- [ ] UART command numbers use the existing guarded paths; queued/refused/successful actions are distinguished.
- [ ] Overcurrent, fault latch/clear, zero-command arm, disarm and LED behavior still work through UART commands.
- [ ] Vq/current limits and pot/manual source-change restrictions reject invalid settings.
- [ ] Fragmented/CRLF input works; malformed, overlong, timed-out and corrupted/overflowed lines do not execute partial commands.
- [ ] RX resynchronizes after a blank LF; fresh complete commands work afterward.
- [ ] TX overload drops complete records and increments counters without blocking control or main-loop supervision.
- [ ] Stream on/off works; congestion skips samples without catch-up bursts; timestamps/sequence gaps visible.
- [ ] Dashboard field units, encoder-rpm sign, current validity and cached dq age match the reference logs.
- [ ] Capture dump completeness checked by sample indexes; loss detected rather than treated as a complete capture.
- [ ] Continuous RX/TX, capture output and stalled/absent SWO viewer keep the full control ISR below 25 us with no unexplained ADC late samples.
- [ ] Main-loop command/stop response and memory/stack usage are acceptable under the same stress test.
- [ ] Buffered input older than 250 ms and partial lines idle for over 1 s are discarded after a stall/reconnect; fresh commands work after LF resynchronization. Disconnect alone does not stop the motor.
- [ ] Diagnostics-disabled build retains legacy UART testing; simultaneous UART owners are rejected at build time.

## Final checkoff

- [ ] Continuous-rating discrepancy resolved, or bounded interim test envelope explicitly accepted.
- [ ] Required current/angle/PWM/protection checks pass; failures resolved.
- [ ] Any deferred rig/peripheral checks and their next-stage impact explicitly accepted.
- [ ] Nathaniel accepts Stage 12B complete; decision saved in the bring-up log.

**Already waived:** independent phase-current meter check, 2026-10-14. No repeat required; common gain uncertainty remains. Prior Stage 8 protection and Stage 10 alignment passes remain historical evidence, not new passes of this build. Current PI and later-stage tests are outside this checklist.

## Optional notes / deferrals

Only write something here if you want to explain a result, link a capture, or explicitly defer a test. No entry is needed for each tick.

____________________________________________________________

____________________________________________________________

## Maintaining this checklist

When implementing a feature or changing firmware behavior, add/update short unchecked tests here for the new behavior and affected regressions in the same change. Keep existing passes and waivers; add an unchecked recheck when a change invalidates prior evidence. Later-stage features get a separate labeled stage checklist linked here, not additional 12B prerequisites. See [AGENTS.md](../AGENTS.md) for the standing instruction. No per-test form is required.

Updated 2026-10-16 at Nathaniel Su's request; simplified by Codex. No tests marked passed by this edit. Detailed requirements/results remain in the [bring-up plan](BLDC%20Controller%20Firmware%20Bring-Up%20Test%20Plan.md); current status is in the [handoff](Firmware-Development-Handoff.md).

# Firmware development handoff and bring-up record

Snapshot: 2026-10-16. This document preserves context for future developers and agents. It is separate from the owner's local `Firmware-Walkthrough.md`, which explains how the code works and is gitignored.

**Developer for this session: Nathaniel Su.** Requests, decisions, and setup clarifications originating in this agent's 2026-10-15 conversation are Nathaniel's; documentation was written by the Codex agent. Earlier bench history summarized here retains its original provenance and is not retroactively attributed to Nathaniel. Use his name for subsequent updates from this conversation, as requested.

## Current task boundary

**Latest authorized work (2026-10-16):** Nathaniel Su requested standalone UART TX/RX diagnostics, commands and logging on `post-12b-uartdev`, prioritizing motor-control execution over stable telemetry. Implemented `diag_uart` transport, `diag` protocol and an application adapter over existing command/settings paths; normal SWO logs are mirrored and normal SWO writes are now buffered/nonblocking. See [UART-Diagnostics.md](UART-Diagnostics.md) and the new UART checklist section. USART2's existing PA2/PA3, 115200 8N1, IRQ-priority-6 configuration is retained; no CubeMX change. Builds, host tests and hardware validation have not been run under `CLAUDE.md`. This does not implement Stage 13 or assert Stage 12B acceptance.

Nathaniel Su authorized **Stage 12B implementation/protection fixes on 2026-10-16** and explicitly confirmed **12B only**, excluding Stage 13 current regulation. The agent hardened overcurrent fault latching and shadow-current diagnostics; no limits, stage selection, or generated configuration changed. Source reviewed only; builds, flashing and hardware testing remain with the developer under `CLAUDE.md`. Stage 12B hardware acceptance is still pending.

Read the user's latest instructions before taking action. Historical approvals below describe decisions already made; they are not blanket authorization for new bench procedures or later stages.

## Read first and source precedence

**Feature checkoff policy (Nathaniel Su, 2026-10-16):** every implemented feature or behavior change must update the relevant short tick-box tests and affected regressions in the same turn. Start with `docs/Stage-12B-Checkoff.md`; link separate stage checklists there when later-stage work is authorized. Preserve prior passes/waivers and leave new or invalidated checks pending. Repository `AGENTS.md` contains the persistent instruction.

**Ongoing update policy (user request, 2026-10-15):** substantive future bring-up reports are saved in the original plan's dated Bring-up log during the reporting turn, and this handoff is refreshed when current state changes. Repository `AGENTS.md` supplies the persistent instructions for future chats. Record provenance, measurements, decisions, missing evidence, and unresolved checks; never convert an inference or waiver into a verified bench pass. The personal walkthrough remains separate and ignored. Shared files are local until explicitly committed/synchronized; separate checkouts may be stale.

1. Repository `CLAUDE.md`: development workflow, file ownership, bench constraints, and safety conventions.
2. [BLDC Controller Firmware Bring-Up Test Plan.md](BLDC%20Controller%20Firmware%20Bring-Up%20Test%20Plan.md): specification and detailed dated bench record. Read the relevant stage plus the latest log entries, not just the original procedure.
3. `App/app_config.h`: actual selected stage and diagnostic switches.
4. `App/motor_params.h`, `board.h`, and the implementation touched by the task.
5. Relevant schematic screenshots/datasheets in `docs/` before changing peripheral timing, registers, or hardware assumptions.

This handoff is a dated summary, not a replacement specification. Current source establishes implemented behavior; recorded developer measurements establish bench evidence. A stage being compiled or implemented does not mean it passed. If evidence conflicts, identify the conflict rather than silently choosing an interpretation.

`CLAUDE.md` prescribes human builds/flashing/tests and one stage at a time. It prohibits command execution except documentation reads, owns generated configuration through CubeMX, and requires new findings to be added to the dated bring-up log. Follow applicable current user authorization and repository instructions. Do not infer a bench pass from an agent's successful code generation.

## Snapshot of implemented functionality

- MCU: STM32G491RE, 170 MHz from 24 MHz HSE, C11, no RTOS or dynamic allocation in the application design.
- Driver: DRV8323S, SPI3, six-input PWM mode.
- Motor: 4015 gimbal, 11 pole pairs, operated at nominal 12 V.
- Encoder: MT6701 SSI on SPI1, 14-bit angle, CRC/status validation, unwrap, PLL speed estimation.
- `BRINGUP_STAGE = 12`, `COMMS_TEST = 0`, `SENSOR_TEST = 0`.
- Voltage-mode FOC is implemented and a successful first spin is recorded.
- Stage 12 calculates Id/Iq in shadow mode; they are telemetry, not feedback to a current PI controller.
- Current PI, speed PI, the planned motor state-machine module, thermal model, VM overvoltage trip, stall detector, and independent watchdog are not implemented merely because they appear in the plan.
- No Stage 12 bench pass appears in the inspected final log entry.
- Stage 12 is now organized as **12A: existing baseline and recorded evidence**, and **12B: remaining hardware checkout before Stage 13** (user request, 2026-10-15). These are planning labels, not changes to the numeric firmware macro. Proposed branches are `stage/12a-existing-baseline` then `stage/12b-hardware-checkout`; neither was created. Preserve existing passes/waivers without treating 12A as a newly validated build. See the temporary implementation outline for the 12B closure matrix. Stage 13 software can progress on dependent branches, but energized tests and validated promotion await 12B acceptance.
- Earlier open-loop spin and electrical-alignment commands remain available at Stage 12.
- **UART implementation, 2026-10-16:** `UART_DIAG_ENABLE=1` owns USART2 instead of the legacy UART bench test (conflicting owners fail compilation). Bounded low-priority IRQ byte movement, fixed TX/RX rings, main-only command parsing and allowlisted integer settings; versioned text `LOG`/`RSP`/`TEL` records. Streaming defaults off, requests >=50 ms are best effort, congestion drops rather than catches up. Normal SWO logging no longer waits for ITM readiness; fatal raw SWO remains unchanged and is not UART mirrored. Cmd 0 explicitly requests clear, other supported numbers retain debugger meanings; responses report QUEUED, not successful execution. UART disconnect is not a motor stop. Cached dq values remain one-second means with age/validity, not a high-rate control signal. All changes remain source-reviewed only.
- **Status LED update (Nathaniel Su request, 2026-10-16):** Stage 5 onward uses LED4/PC3 for controller status: disarmed/waiting 5 Hz (100 ms on/off), PWM armed solid on, latched fault 1 Hz (500 ms on/off), with fault priority until explicit clear. Waiting includes missing arm prerequisites; solid indicates PWM armed, not shaft motion. Main-loop indication replaces power-state indication in these stages; Stages 0–4 and fatal HardFault/NMI halt/blink diagnostics retain their earlier behavior. Source-reviewed only; build and bench confirmation pending.

Do not advance `BRINGUP_STAGE` to 13 as a substitute for implementing Stage 13. `APP_VEC_MODE` currently ends at Stage 12; changing this macro alone would not create a functional current-loop application.

## Hardware context that must survive a handoff

| Connection | Actual mapping |
| --- | --- |
| PWM A | TIM1 CH3/CH3N, PA10/PB15 |
| PWM B | TIM1 CH2/CH2N, PA9/PB14 |
| PWM C | TIM1 CH1/CH1N, PA8/PB13 |
| Current A / B / C | ADC2 / ADC1 / ADC3, all injected channel 12, PB2 / PB1 / PB0 |
| Encoder | CS PA4, CLK PA5, DO PA6; PA7 dummy MOSI unused by encoder board |
| Driver SPI | CS PD2, SCLK PC10, SDO PC11, SDI PC12 |
| Driver enable/calibration | PC8 / PC9 |
| nFAULT | PB10, both hardware TIM1 break and EXTI |
| MOTOR_EN | PC1 input sensing hardware relay-coil signal |
| VM sense | PB11, ADC1 channel 14, 180k/10k divider |
| Throttle 1 / 2 | PA0 / PA1, ADC2 regular channels 1/2 |
| Supply-current sense | PB12, ADC1 regular channel 11 |

SW1 operates relay K1 in hardware; firmware senses it. Logic can run from USB with VM absent. The ACS71240 is a separate supply-current sensor before K1, not the FOC phase-current sensor. FOC uses three 5 mΩ low-side shunts and the DRV's gain-40 amplifiers.

Schematic screenshots cover system blocks, power, MCU/peripherals, gate driver, connectors, and a readable MCU pin detail. The detailed power-switch sheet and MOSFET datasheet were not present in the inspected `docs/` inventory. Do not invent test-point mappings from missing sheets. The STM32 PDFs have misleading `STM32G419E` filenames; the actual target is STM32G491RE.

**Two boards appear in the history.** The original PCB developed/revealed a phase-A high-side FET problem and was retired. The developer switched to a new PCB during Stage 6. Original Stage 0–5 results concern the old board; later switching and motor results concern the replacement board.

## Chronological development and evidence

### 2026-10-04: foundation, Stages 0–4

**Stage 0: core and safe boot.** Added application hooks, `board.h`, debugging, stage selection, clock/cycle-counter checks, safe GPIO/TIM1 state, and a custom HardFault dump. SWO was made to work through Cortex-Debug with ST OpenOCD's `stlink-dap.cfg`. The older HLA path had recursive initialization problems. Developer reported Stage 0 passed: 170 MHz, SWO, safe PWM pins, debug outputs, and fault dump.

The custom HardFault handler requires CubeMX not to generate a duplicate handler. Safe boot actively drives gate inputs low using MOE=0, OSSI/OSSR, low idle states, and enabled channel output paths; leaving inputs floating was deliberately avoided.

**Stage 1: encoder.** Added SPI1 mode-1 reads as two 12-bit frames, CRC-6, field/loss-of-track checks, angle unwrap, and a 50 Hz PLL estimator. Early logs showed zero CRC/jump/timeout errors while hand-turning and about 1–2 counts stationary jitter. Developer called the stage passed; initial read time was about 16 µs in unoptimized Debug, substantially longer than the 4.5 µs wire time. Encoder assembly/magnet prerequisite was later explicitly closed Oct 8 based on the assembled motor/encoder and clean reads.

**Stage 2: power.** Added calibrated ADC1 regular VM readings, five-tick switch debounce, and `NO_VM -> VM_SETTLING -> VM_OK`. VM_OK needs at least 10 V for 20 ticks; below 9 V or switch removal loses readiness. Stage passed. A rapid charge of the roughly 1,000 µF VM bank exceeded what the supply current-limit setting alone would suggest; the log attributed this plausibly to supply output capacitance. Do not equate supply limit with an instantaneous surge cap.

**Stage 3: driver link.** Added wake/sleep timing and SPI3 register access. Presence detection deliberately reads register 0x03 expecting 0x3FF: an unpowered pull-up reads 0x7FF, which is also the reset value of register 0x04 and would be a misleading presence check. Read/write, lock/unlock, fault clear, and sleep-reset checks passed.

**Stage 4: configured driver and fault monitoring.** Added configure/readback/calibration/locking and periodic verification, fault latches, nFAULT event handling, and reconfiguration on every wake. Developer reported pass.

### 2026-10-08: PWM and physical bridge, Stages 5–6

**Stage 5: PWM into a sleeping driver.** Added timer checks, single arm/disarm interface, 20 kHz center-aligned complementary PWM, 100 ns timer dead time, zero-command arming, debug freeze, break handling, and ramped diagnostic duties. Developer verified A/B/C mapping using 20/50/80% commands, approximately 99 ns dead time, and low outputs after halt/break/disarm. Passed.

Important fixes retained from this work:

- Break interrupt BIE is armed only with PWM and disabled after disarm/break. A held-low nFAULT otherwise caused repeated interrupts and starved main-loop diagnostics. Hardware break remains configured independently.
- System break sources connect core lockup, SRAM parity, and flash double-ECC errors to shutdown; NMI reporting was added. HSE CSS was not enabled.
- `pwm_init()` no longer identifies success by a message starting with the letter O, which had incorrectly accepted an `OCxM` mismatch message.
- MOE readback allows a brief asynchronous propagation delay and forces disarm on a failed arm confirmation.

**Stage 6: bridge switching.** Added duty sweep, IDRIVE tuning, and per-phase hold-off tools. An original-board fault repeatedly reported `0x620 = FAULT | VDS_OCP | VDS_HA`. Holding phase A off allowed B/C to run. Scope evidence showed full high-side gate drive but no useful phase-A channel conduction; the drain supply was confirmed. The log concluded a dead/nonconducting phase-A high-side FET. Its original failure cause was not established.

The developer chose to replace the PCB rather than repair that FET. On the replacement, phase A and then all phases switched successfully. Approximately 60 ns non-overlap was reported on one transition. IDRIVE was retained at 0x22 (60 mA source / 120 mA sink).

A separate firmware bug was fixed: disarm lowered ENABLE, but driver software still claimed awake/configured. `drv8323_is_awake/is_configured()` now require ENABLE high, and `drv8323_sync()` completes sleep bookkeeping before normal wake/reconfiguration. Preserve this relationship.

Stage 6 was not formally closed in full. Falling-edge dead time, duty-versus-switch-node-average sweep, and supply-current on/off comparison remained open in the log. The nFAULT short test was later reported passed during Stage 8, so that specific item is no longer outstanding.

### 2026-10-08: sensing and motor characterization, Stages 7–8

**Stage 7: phase-current acquisition.** Added the custom ADC1/2 ISR, synchronized injected sampling, measured zero offsets, heartbeat, statistics, and 1,024-sample capture. CubeMX's generated ADC1_2 handler was disabled to avoid duplicate definitions. Initial offsets were 2023/2029/2042 counts; these are historical examples, not constants to hardcode.

At 93% duty, scope evidence placed sampling about 1.8 µs after the low-side gate turned on, exceeding the cited 1.2 µs amplifier settling. Initial ISR timing was too slow for the sensing-only budget at -O0. Selected sources were switched to -O2; developer then reported Stage 7 passed. Do not confuse its 10% sensing-only budget with the full-control target below 50%.

**Stage 8: stationary voltage vectors.** Added CORDIC sin/cos, inverse transforms/SVPWM, `ctrl.c`, software overcurrent shutdown, voltage-step capture, and R/L characterization. CORDIC activation required CubeMX regeneration. Fast functions were moved into SRAM to reduce flash overhead; reported complete ISR time fell from 7.7 to 6.8 µs.

Measured R line-to-line was 4.4 Ω, so the wye-equivalent model became 2.2 Ω per phase, replacing the earlier 1.6 Ω estimate. Three inductance steps produced 525/611/483 µH, mean 540 µH. Trip testing passed. Hardware nFAULT short/break testing passed. Stage 8 was accepted except for an independent phase-current meter check, deferred to before Stage 11 and later explicitly waived.

### 2026-10-08: Stage 9 first rotation

Added a forced electrical-angle generator with speed slew and initial hold. The encoder was observed, not used to determine commutation angle in this mode. Stage passed with forward/reverse tests and about -1489 encoder counts per electrical cycle. This confirmed 11 pole pairs and DIR=-1 for the present motor lead order.

Later logs at 10 Hz electrical and 1.2 V showed about 54 rpm, 10.99 inferred pole pairs, and approximately 442 mA phase-current vector amplitude. This is open-loop test evidence, not closed-loop speed regulation.

### 2026-10-13 to 14: interfaces and auxiliary sensing

- CubeMX labels changed. PA2/PA3 returned to USART2; the old external scope DAC/timing functions were retired. DAC3 remains internal, timing helpers are no-ops.
- Added optional IMU/I2C, UART, CAN, and USB bench tests with individual switches.
- IMU test passed: address 0x6A, WHO_AM_I 0x6C, plausible flat-board acceleration and gyro/temperature.
- UART RX from Teensy worked after crossing wires; TX reception at the Teensy remained unresolved in the log.
- CAN loopback diagnostics were added; do not infer completed external CAN validation.
- USB initially did not enumerate. `app_clock_init()` corrects generated CRS trim 32 to 64, temporarily disabling AUTOTRIMEN during the write. The generated field/tool limitation was documented; do not remove the override because CubeMX already initializes CRS.
- USB later enumerated in one plug orientation. Final developer clarification was orientation-dependent connector-row wiring, superseding the earlier intermittent-contact hypothesis. Working enumeration does not establish both orientations or completed echo/reliability tests.
- Added `aux_sense.c`: filtered throttle channels and ACS71240 supply current, with zero calibration when SW1 is off. Supply-current results around 74–80 mA agreed plausibly with bench consumption after other loads. THROTTLE2 read zero and remained an investigation item.

### 2026-10-14: decisions and Stage 10 alignment

The developer explicitly waived the independent phase-current meter check. Supporting evidence: phase RMS/vector amplitude agreement, independent supply sensor versus bench supply, and approximate power balance. Accepted residual uncertainty: common gain error in the three phase amplifiers. Do not rewrite this as a completed meter test.

The developer selected Stage 11 limits **1.5 A phase trip and |Vq| <= 2 V**, superseding the earlier proposal of 3 A/4 V. On choosing Stage 12, the developer explicitly kept the 2 V cap instead of increasing speed through a higher voltage limit.

Stage 10 added automated bidirectional electrical-angle sweeps and offset averaging. Five runs at 1 V gave 334.56/334.65/334.59/334.63/334.58 degrees: mean 334.60 degrees, spread 0.09 degrees. DIR=-1 was confirmed. Stored offset is 5.8400 rad. Forward/backward mean separation around 9 degrees and individual deviations up to about 16.5 degrees were attributed to friction/cogging. Stage passed.

### 2026-10-14: Stage 11 voltage-mode FOC

Moved encoder acquisition into the 20 kHz control path using split FIFO transfers. An earlier proposal that SPI DMA/CubeMX changes were necessary was superseded: the implemented method queues the transaction in one ISR and collects it in the next, without DMA.

Added encoder-angle commutation, Vd=0, ramped Vq, latency advance, and immediate encoder-invalid disarm. A first attempt faulted with an encoder jump after a debugger pause; the rotor had moved while sampling stopped. Pot control was added to avoid live debugger changes. Developer reported successful FOC spin up to the 2 V cap. Quantitative speed/reverse/VM step-down logs were still outstanding.

### 2026-10-14: Stage 12 shadow current transforms

Added amplitude-invariant three-current Clarke and Park, per-second Id/Iq statistics, dq capture encoding, and scheduled automatic capture. Encoder angle is advanced 55 µs for sampled currents, versus the 100 µs advance used for voltage actuation. dq results do not change the voltage command. This is the latest implemented stage; no subsequent bench acceptance is recorded.

## Current constants and conventions

| Parameter | Current value / provenance |
| --- | --- |
| Phase resistance | 2.2 Ω, from measured 4.4 Ω line-to-line |
| Phase inductance | 0.54 mH, mean of three step estimates, about ±12% spread |
| Pole pairs | 11, supported by open-loop encoder motion |
| Encoder direction / offset | -1 / 5.8400 rad, Stage 10 |
| Back-EMF constant | 0.1562 V/(rad/s), application's line-to-line convention |
| Modulation Vbus | Hardcoded 12 V; measured VM not yet fed into SVPWM |
| Vq limit / ramp | ±2 V / 1 V per second |
| Current trip at Stage 12 | Default 1.5 A, setter bounds 0.1–1.5 A |
| PWM | 20 kHz, ARR 4250, PSC 0, center-aligned mode 1 |
| Current trigger | TIM1 TRGO2=OC4REF, CCR4 4200, rising on down-count |
| Duty maximum | 0.93, preserves sampling window |
| ADC / CSA / shunt | 12-bit, 3.3 V, 40 V/V, 5 mΩ |
| Current conversion | `(raw-offset) * 0.00402832 A`, positive into motor |
| Offset measurement | 10 ms settle + 1,024 samples; valid within 2048±250 counts |
| Encoder SPI / driver SPI | Approximately 5.31 MHz / 664 kHz, both mode 1 |
| Gate drive | IDRIVE 0x22, 60/120 mA; separate from motor current |

Use amplitude-invariant transforms. With Id=0, |Iq| is peak phase/line current, not RMS. Mechanical encoder speed is converted to electrical speed using DIR*11. Phase A is electrical zero and positive forced rotation advances A->B->C. Recalibrate if the magnet, encoder, or lead order changes.

The continuous motor rating appears as both 0.55 A and 0.65 A in the existing record. The 4 A maximum's peak/RMS and line/winding interpretation is acknowledged as uncertain. Future thermal/current-limit work must resolve these rather than treating an old table as verified.

## Runtime integration and invariants to preserve

The fast path is `ADC1_2_IRQHandler -> ctrl_isr -> overcurrent check/shutdown/latch -> encoder collect -> electrical angle -> dq/capture -> ctrl_step -> PWM duties -> encoder start`. Sampling occurs once every 50 µs independently of main-loop scheduling. Since the 2026-10-16 Stage 12B fix, overcurrent shuts down and latches before encoder/dq work; the main loop only consumes the diagnostic event and logs it. This closes the prior window where `drv_tick` could begin waking the driver before `ctrl_tick` latched the fault. Fault context retains phase and signed mA, and capture triggers after the trip sample as before.

Stage 12B dq statistics require an enabled driver, valid calibrated offsets and a valid angle. `bad` now includes failed current prerequisites; `n=0` is unavailable data, not a measured zero. Empty windows return zero extrema; mean divisions run outside the interrupt-disabled snapshot. Capture stores invalid dq as `0 0` (not -32.768 A). Validity is saved before shutdown so the trip sample can still be captured. Raw capture is unaffected. Late ADC completion remains counted but not rejected by this change; Stage 12B must establish clean sampling, and explicit stale-sample handling remains assigned to Stage 13A.

Keep SPI3 and ordinary logging out of this path. The encoder pipeline has only a short CS setup wait; it does not block for the complete transfer. ADC2/3 completion uses a bounded wait. Some older comments say absolutely no waiting; describe the actual bounded mechanisms accurately.

Only the normal arm path sets MOE. Keep zero-command arming, command ramps, safe timer off-state settings, debug freeze, no automatic output re-enable, and PWM-off-before-ENABLE-low shutdown. Hardware break clears outputs independently of CPU execution. HardFault/NMI have deliberate direct shutdown exceptions.

Faults remain latched until explicitly cleared. Re-waking the driver after normal disarm is allowed; automatically re-arming PWM is not. Every wake resets/reapplies/verifies driver settings and starts fresh phase-current offsets.

Generated peripheral changes belong in CubeMX, not arbitrary edits to generated init code. Keep application hooks in USER CODE blocks. Preserve custom handler ownership to avoid duplicate HardFault/ADC handlers. Keep fast-path optimization and SRAM placement unless measured evidence supports a change.

## Open checks and known limitations

**Consolidated checkoff (Nathaniel Su request, 2026-10-16):** [Stage-12B-Checkoff.md](Stage-12B-Checkoff.md) lists pending build, protection, bridge, sensing/dq/capture/timing, voltage FOC, load-rig, auxiliary and LED checks. At Nathaniel's follow-up request it now uses short tick-to-pass items, with no mandatory per-test form/status codes; notes and capture links are optional in the checklist. Explicit deferrals remain separate from passes. It preserves prior evidence and waivers; substantive results reported in chat still belong in the original Bring-up log. No new test or stage pass is implied.

**Planned characterization rig (developer, 2026-10-15):** BLDC coupled to a DC generator feeding an electronic load in CC mode. Generator housing is free to rotate; a rod glued to the housing reacts against a scale and nominally holds it stationary. This mounting detail is user-confirmed. See `docs/Motor-Controller-Characterization-Plan.md` for calibration, torque/power/dq equations, efficiency boundaries, load compliance, stage integration, and required evidence. DC generator type/model, scale/load specs, arm geometry, ratings, and measured results are not yet provided. Do not assume constant load-current setpoint guarantees calibrated torque or that this rig can load a stationary position loop. The temporary branch outline links this characterization work; no controller implementation or limits changed.

1. Stage 12 requires actual dq captures and quantitative logs before claiming pass: driver awake/valid offsets for zero tests; positive/negative Vq signs; load response; angle health; timing after added transforms.
2. Stage 11 reverse, measured rpm versus prediction, and VM on voltage reduction lack recorded quantitative closure.
3. Stage 6 falling-edge dead time, average switch-node voltage sweep, and supply-current comparison remain unclosed in the inspected log. Hardware nFAULT short test is already passed.
4. Independent phase-current meter calibration was waived, not performed. Do not silently remove the recorded uncertainty or restore it as an unresolved approval question without new reason.
5. Actual VM is not used for modulation normalization. Voltage scaling is nominal-12-V dependent.
6. ADC late completion is counted but still followed by register reads; this is not currently robust stale-sample rejection.
7. Main-loop SWO bursts caused recorded delays up to about 7.2 ms. The fast ISR keeps running, but supervision and command ramps can be delayed. Some snapshot/statistic functions mask interrupts while calculating results; evaluate actual latency if making timing claims.
8. A sleeping driver can make current telemetry appear strongly negative using old offsets. Do not interpret this as real phase current without checking driver/offset state.
9. `CUR rms` is standard deviation about the mean, not total RMS including DC. dq invalid captures use zero encoding, not a real -32.768 A result.
10. Pot dead band is 475–525 permille (±5% of half travel), despite less precise prose in older logs.
11. THROTTLE2, UART TX, external CAN validation, and USB connector orientation/reliability need their respective bench follow-ups.
12. Boot configuration checks do not all universally latch faults; this is not a complete safety validation. Planned thermal/OV/stall/watchdog protections are absent.

## Future stages: reference only, not authorization

**2026-10-15 proposal for review:** `docs/TEMP-Current-Control-Branch-Plan.md` preserves a proposed Stage 12 / 13A–13E breakdown and stacked-branch promotion workflow, followed by torque, speed, position, and field-weakening work. The user requested an outline and a separate temporary document only. Branch names and subdivision labels are proposals; no branches, controller implementation, or new hardware acceptance were created. Read it alongside the original plan before adopting the workflow.

**Before Stage 13:** finish Stage 12 evidence, confirm which existing open checks the developer wants closed, and read current instructions. Preserve accepted limits until explicitly changed. Confirm R/L and their measurement uncertainty; do not reuse old estimated gains.

**Stage 13, current-loop torque control:** add d/q PI regulation, vector saturation and anti-windup, and appropriate command/state integration. The plan uses Kp=L*omega_c and Ki=R*omega_c; measured R/L give approximately 3.4 V/A and 13,800 V/(A*s) at 1 kHz bandwidth. These are starting calculations, not tuned results. Plan calls for 25–50% initial bandwidth, limited locked-rotor tests (Id/Iq commands capped at 0.3 A and automatic disarm after 2 s), captured step responses, then higher bandwidth only with clean response. Verify ISR stays below 50% of the cycle.

**Stage 14, speed control:** planned state machine IDLE->ALIGN->RUN->FAULT, 2 kHz speed PI generating Iq demand, acceleration/ramp handling, thermal/I²t limits, stop/coast sequence, and VM overvoltage response. Original plan mentions 4 A peak command, approximately 4.5 A trip, and 16 V OV shutdown. These are future design values and must not override the current 1.5 A/2 V settings without stage-specific authorization and validation.

**Stage 14A, field-weakening characterization (added to the plan 2026-10-15):** the user requested test-plan coverage, not implementation. This is a future extension after validated current and baseline speed loops, not a new compile-time stage number. The original plan now specifies negative-Id effect tests, measured base-speed identification, matched voltage/speed/load comparisons, current-circle torque tradeoff, heating, smooth entry/exit, and repeatability. Read that section before proposing code or tests.

No motor-specific safe negative-Id or overspeed limit is established. Prerequisites include manufacturer-supported negative-Id/temperature and mechanical-speed bounds, resolution of the 0.55/0.65 A continuous-rating discrepancy, measured-VM modulation, total-current and thermal budgeting using Id²+Iq², validated sampling near saturation, and bus-energy-safe exit/fault response. A spinning motor can feed VM through bridge diodes after coast; do not assume disabling gates resolves high-speed bus overvoltage. Keep Stage 12 limits unchanged. The existing radial SVPWM limiter is not a verified d-axis-priority field-weakening allocator. A negligible useful speed extension on this high-resistance motor is an acceptable characterization outcome; do not raise limits to force a gain.

**Stage 15, hardening:** watchdog, stall detection, explicit enable-loss behavior, and fault-injection tests. If field weakening is later implemented, include its saturation/negative-Id limits, overspeed/temperature, safe exit on angle/enable loss, bus energy, and no automatic re-entry after faults. Verify behavior against actual hardware; do not claim completion from implementing a handler alone.

For each authorized future stage, record files changed, required CubeMX changes, expected logs/signals, actual developer results, measured constants, unresolved items, and pass/waiver decisions in the original dated log. Update this handoff when it would otherwise mislead the next agent.

## Practical recovery and debugging context

- **2026-10-16 current debugger setup (supersedes four-variable setup below):** Nathaniel requested PATH discovery. Removed launch serverpath/armToolchainPath overrides, retaining BLDC_OPENOCD_SCRIPTS and BLDC_SVD. Verified all six tool executables through saved Windows PATH, both supporting user variables, OpenOCD scripts, valid launch JSON, and existing Debug ELF. Local setup script is no longer needed. Developer must fully restart VS Code and try F5; actual debug launch remains unverified. See `docs/Debug-Setup.md`.

- **2026-10-16 follow-up:** Nathaniel's next screenshot reports bare `arm-none-eabi-gdb.exe` missing. Local GDB exists; likely missing inherited `BLDC_ARM_TOOLCHAIN` in VS Code after the environment-based launch change. Fully close VS Code and start it from the external shell that dot-sources the local setup script. No successful debug session yet established.

- **2026-10-16 portable launch fix (Nathaniel Su request):** GDB/nm failed on a hardcoded `benso` path. Shared `.vscode/launch.json` now uses `BLDC_ARM_TOOLCHAIN`, `BLDC_OPENOCD`, `BLDC_OPENOCD_SCRIPTS`, and `BLDC_SVD`. See `docs/Debug-Setup.md`; a gitignored local PowerShell setup file contains verified installed paths. JSON and file existence checked; no launch/flash/hardware verification. Restart VS Code from that environment before debugging.

- **2026-10-16 configure-button diagnosis confirmed:** Nathaniel's VS Code screenshot and installed extension code identify `travnsam.vscode-stm32cube-configurator` 0.0.1 as the source of `Configuration failed. Makefile not found!`. That extension reads a root Makefile to generate IntelliSense settings; it is incompatible with this project's CMake/Ninja workflow. Disable it for this workspace or avoid its button; no generator change is needed. This supersedes the UI uncertainty and `.cproject` hypothesis below. No extension change was performed.

- **2026-10-16 UI follow-up (Nathaniel Su):** "stm32cubemx configure" reports a missing Makefile; exact app/action and diagnostic pending. The `.ioc` selects CMake and presets use Ninja, so a Makefile is not expected from this configuration. Investigate the UI's selected project/build action before changing generators; terminal configuration/linker evidence below remains separate.

- **2026-10-16 linker output (Nathaniel Su):** RAM 20,776 B (18.12%), FLASH 121,396 B (23.15%), with an RWX LOAD-segment warning. The linker script and generated map confirm intentional `.RamFunc` code mixed with writable `.data` in RAM, explaining the warning. Supplied excerpt has no fatal error; exit status and hardware results were not supplied. No linker change made.

- **2026-10-16 resolution (Nathaniel Su's supplied output):** Debug configuration and generation succeeded using GNU ARM C/C++ 14.3.1 and its assembler, producing `build/Debug`. The Ninja discovery failure below is resolved for that run. Compilation/linking and hardware results remain unverified; configuration success is not a stage pass.

- **2026-10-16 follow-up:** Nathaniel supplied the exact Ninja/CMAKE_MAKE_PROGRAM discovery error. Ninja is installed at `C:/Users/natha/AppData/Local/stm32cube/bundles/ninja/1.13.2+st.1/bin/ninja.exe`; make that bin directory available in the configuring process's PATH or explicitly set CMAKE_MAKE_PROGRAM. This narrows the report below to confirmed Ninja discovery failure; configuration success is still unverified.

- **2026-10-16 build-setup report (Nathaniel Su):** CMake fails on `main`; actual error/command pending. Static inspection found the referenced source/include paths, startup file, and linker script present. Presets require Ninja and ARM GCC; editor settings use `cube-cmake`. The agent shell finds CMake but not Ninja, ARM GCC/G++, `cube`, or `cube-cmake`, and has no `CUBE_BUNDLE_PATH`. Likely tool-discovery issue, not yet a reproduced configure failure; editor environment may differ. See the dated Bring-up log. No build or firmware change performed.

- Historical launch configuration did not build before F5; an old ELF was once flashed. Developer should verify build freshness and `BOOT stage` rather than infer it from editor contents. This is historical context; inspect current launch configuration if that workflow is revisited.
- GDB command syntax used was `set var g_app_cmd = N`; the debug console rejected bare assignment, `-exec`, or pasted trailing-newline variants in that setup.
- Do not pause while armed to vary Vq. The pot is the intended live input; automatic capture was added to avoid pauses.
- `g_app_cmd=10` arms voltage FOC; 1 is the retained open-loop mode; 9 runs alignment. 2 disarms, 3 tests software break, 6/7 trigger/dump capture. `g_fault_clear_request=1` clears the latch but does not arm.
- `g_vq_pot=1` is default. Center pot before arming. `g_cap_auto_ms` schedules capture after arm; `g_cap_dq=1` records dq current/angle.
- Capture buffer is 1,024 samples, 512 before/after trigger, one recording at a time. A finished fault capture can block a new step capture until handled.
- SWO is ITM port 0 at the recorded 2 MHz setup with 170 MHz core. The old external scope DAC/timing pins are no longer available on PA2/PA3.

## Documentation produced in this session

- `docs/Firmware-Walkthrough.md`: owner's detailed explanation, explicitly ignored through the repository `.gitignore`.
- `docs/Firmware-Development-Handoff.md`: this shared development context, intentionally not added to `.gitignore` so it can be versioned.
- No firmware source, `.ioc`, generated initialization, or operating limit was changed. No new bench result or stage pass was asserted.

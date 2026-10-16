# Temporary reference: current-control stages and branch workflow

Date: 2026-10-15. Status: **proposal for review, not implementation authorization**.

Developer/requester: **Nathaniel Su**. This outline and its Stage 12A/12B revision were documented by the Codex agent at Nathaniel's request. Setup clarifications from this conversation are attributed to Nathaniel; historical bench evidence retains its existing provenance.

This document preserves the discussion outline for cross-reference during implementation. No firmware changes, branch creation, builds, or hardware tests are implied. The original bring-up plan remains the specification and dated results record; the development handoff remains the current-state summary. Reconcile this proposal with those documents before adopting it.

## Starting point and objective

Current source selects Stage 12: encoder-based voltage-mode FOC with Id/Iq measured in shadow mode, not regulated. Stage 12 hardware acceptance is not yet recorded in the inspected bring-up log.

**Revised Stage 12 split (user request, 2026-10-15):** 12A preserves everything already implemented and all previously recorded results; 12B contains the remaining hardware checkout and evidence closure before Stage 13 hardware operation. This replaces the earlier single `stage/12-dq-validation` branch proposal. It does not retroactively mark existing implementation as hardware-validated.

The next objective is a validated inner current loop, developed in small stages. Later branches may contain implementation while earlier hardware validation is pending, but the validated integration branch stays at the last accepted stage.

Closed-loop Iq regulation is already the basis of torque control. A later torque-mode layer mainly adds command handling, limits, and potentially conversion from N·m to Iq. Speed and position loops then sit outside the current loop.

## Proposed current-control stages

These suffixes are planning labels, not existing numeric values for `BRINGUP_STAGE`. Do not change the stage macro as a substitute for implementing a stage.

| Stage / proposed branch | Future implementation scope | Hardware acceptance before promotion |
| --- | --- | --- |
| 12A — `stage/12a-existing-baseline` | Preserve the current implementation through Stage 12 and its recorded hardware history, including voltage FOC, shadow dq, capture tools, measured R/L, and encoder alignment. Identify the exact baseline revision and inherited open checks. | Baseline accounted for; implementation and historical pass/waiver evidence distinguished. This is a development snapshot, not a new Stage 12 bench pass. |
| 12B — `stage/12b-hardware-checkout` | Complete outstanding hardware checkout and Stage 12 dq validation on that baseline; add only separately authorized diagnostic fixes needed for checkout. Preserve voltage-mode operation. | Checkout matrix below closed with evidence or explicit disposition; valid current/angle/timing results, calibrated load-rig baseline where applicable, and documented decision to proceed to Stage 13. |
| 13A — `stage/13a-current-foundation` | Define current-controller interfaces and units. Feed measured VM into modulation. Define current/voltage limits, sample validity, controller reset behavior, and telemetry. | Measured VM gives correct voltage scaling; sampling remains valid near duty limits; stale/invalid inputs cause the intended response; existing operation still works. |
| 13B — `stage/13b-current-pi` | Implement reusable PI calculation, saturation, anti-windup, and integrator reset. Keep it disconnected from bridge commands initially. Validate using numerical tests and replayed data. | Confirm added computation and telemetry meet timing requirements. Numerical-test success alone does not qualify closed-loop operation. |
| 13C — `stage/13c-id-regulation` | Connect a deliberately limited current regulator for standstill characterization. Use conservative gains derived from measured R/L. Add automatic capture and timed shutdown. | Small Id steps track with correct sign, stable settling, and little overshoot; cross-axis behavior is understood; saturation and shutdown work. Retain the plan's locked-rotor command cap and two-second timeout. |
| 13D — `stage/13d-dq-regulation` | Enable both current regulators. Start with low-speed, small-Iq tests and Id reference zero. Define coordinated voltage saturation and total-current limiting. | Positive/negative Iq commands behave correctly; Id remains controlled; load changes remain stable; limits and anti-windup recover cleanly. |
| 13E — `stage/13e-current-acceptance` | Consolidate startup, stop, re-arm, fault recovery, current derating, and repeatable regression captures. Establish the validated operating envelope. | Current control passes the agreed matrix across permitted speeds, loads, and supply conditions. No unexplained faults or timing regressions. |

Before energizing a new mode, record numerical current, speed, temperature, duration, and tracking-error acceptance thresholds. The existing 1.5 A phase trip is a shutdown threshold, not a continuous operating-current allowance. The current plan specifies locked-rotor test commands capped at 0.3 A and automatic disarm after two seconds; also enforce an appropriate total-current budget rather than interpreting independent axis caps as permission to exceed it.

## Resolve before tuning

### Stage 12A: existing baseline inventory

- Preserve implemented startup/power/driver/PWM/current-sampling protections and previous stage diagnostics.
- Preserve voltage-mode FOC, THROTTLE1 control, split encoder SPI, latency compensation, shadow Id/Iq statistics, and raw/dq automatic capture.
- Carry forward measured R=2.2 Ω, L=0.54 mH, DIR=-1, and offset=5.8400 rad with provenance; retain the original-board/replacement-board distinction.
- Carry forward recorded passes and the explicit independent phase-current meter-check waiver. Pending results stay pending, including Stage 12 hardware acceptance.
- Preserve current |Vq| <= 2 V, 1.5 A nominal phase trip, and 93% duty cap. Record the exact source/build baseline when branch work is authorized.

### Stage 12B: hardware checkout closure matrix

| Checkout | Required evidence / closure |
| --- | --- |
| Baseline and re-arm | Exact tested build and board identity, boot checks, driver configuration/readback, offsets after wake, explicit arm/disarm and fault-clear behavior. Reuse unaffected prior evidence with its provenance rather than declaring all tests repeated. |
| Remaining bridge checks | Resolve the recorded falling-edge dead-time check, duty versus switch-node-average sweep, and PWM-on/off supply-current comparison. Retain the passed nFAULT test; repeat if relevant hardware/firmware changed. |
| Current sensing and timing | Validate zero with driver awake/offsets valid, phase signs/sum/scaling consistency, sample window at the accepted duty envelope, late-sample count, full Stage 12 ISR timing, and capture decoding. The independent phase-current check remains explicitly waived unless new measurements are supplied. |
| Voltage FOC / dq | Forward/reverse plateaus and bounded loading: correct Iq sign/load response, physically plausible Id including voltage-mode cross-coupling, bounded ripple, healthy encoder, and recorded speed/VM response on Vq reduction. Do not require mathematically exact Id=0 in voltage mode. |
| Characterization fixture | Calibrate the housing reaction arm/scale, record geometry/uncertainty, establish generator/electronic-load compliance, and obtain bounded voltage-mode baseline points per C0/C1 in the characterization plan. If the rig cannot operate within current limits, record the limitation and explicit alternative/deferment; do not raise limits to force compliance. |
| Auxiliary hardware | Close or explicitly defer THROTTLE2, UART TX, external CAN, and USB orientation/reliability checks. Distinguish unrelated peripheral deferrals from motor-control prerequisites; no blanket claim of all hardware passing. |
| Motor/test envelope | Record current, temperature, speed, duration, and abort bounds for the next stage; resolve the continuous-rating discrepancy or explicitly document a bounded interim test envelope and the outstanding uncertainty. |
| Exit review | Every item marked pass, fail, previously passed with applicable evidence, waived, or explicitly deferred with rationale and next-stage impact. No unresolved failure of a required current/angle/PWM/protection prerequisite may be labeled ready. |

**Gate:** Stage 13 software implementation may proceed on dependent branches while 12B testing is pending. Energized Stage 13 tests and promotion of Stage 13 to the validated integration branch wait for the 12B acceptance decision. Closing 12B does not implement current PI, measured-VM modulation, or the future sample-validity changes assigned to Stage 13A.

### Stage 13 design items after the checkout baseline

- Modulation currently assumes 12 V despite measuring VM. Define how measured bus voltage reaches the fast loop, including freshness and invalid-reading behavior.
- The ADC handler counts late samples but still reads their result registers. Establish explicit sample-validity behavior.
- Define a coordinated voltage-allocation policy and anti-windup tied to the voltage actually applied. Distinguish requested, limited, and applied voltages in telemetry.
- Account for total current magnitude `sqrt(Id² + Iq²)` in limits and thermal reasoning.
- Establish encoder timing compensation and current polarity before interpreting PI performance.
- Keep the unresolved motor continuous-rating discrepancy and the explicitly waived independent phase-current check visible. A waiver is not a successful measurement.
- Preserve the 93% PWM duty cap and verify the low-side sampling window across the accepted envelope.

Start with a stable baseline without extra decoupling/feedforward terms unless measurements show they are needed. Add complexity as a separately reviewable change with evidence.

## Proposed branch structure

Use a validated integration branch and stack development branches above it:

```text
main — last hardware-accepted firmware (target policy)
 └─ stage/12a-existing-baseline
     └─ stage/12b-hardware-checkout
         └─ stage/13a-current-foundation
             └─ stage/13b-current-pi
                 └─ stage/13c-id-regulation
                     └─ stage/13d-dq-regulation
                         └─ stage/13e-current-acceptance
```

This is a proposed policy, not a claim about the current branch name or validation state of its HEAD. Before adopting it, identify and record the last hardware-accepted commit. Preserve the current work if it is ahead of that baseline; do not reset or discard it merely to match the diagram. If a different integration branch name is used, substitute it consistently.

The structure allows implementation of 13D while testing is still on 13A. Each branch includes its prerequisites; its review should show only its own changes relative to its parent.

Stage 12A is the preserved development baseline and may contain unvalidated Stage 12 work. Do not promote it to a hardware-validated `main` solely because it is called “already done.” The 12B evidence closes the applicable 12A acceptance gaps; promote that accepted baseline/checkout together or in dependency order after the gate passes. Do not duplicate or reset existing commits to manufacture the proposed branch layout.

### Stage lifecycle

1. Create a draft PR targeting the predecessor branch, once branch/PR work is authorized.
2. Record implementation checks separately from hardware results. Use explicit states such as planned, implemented, software-checked, hardware-pending, hardware-failed, and hardware-accepted.
3. Test an exact commit/build and record board identity, settings, limits, and captures. If the tested build includes uncommitted changes, preserve and identify those changes as part of the test record; do not label it as the clean commit alone.
4. Promote into the validated integration branch only after prerequisites and that stage's acceptance criteria pass. Retarget dependent PRs as predecessors are promoted; do not merge later-stage work by accidentally merging an entire development stack.
5. Bring fixes forward into dependent branches. Repeat affected software and hardware checks before promotion.

A hardware pass belongs to the tested revision. Changes affecting timing, sensing, modulation, gains, or limits make the relevant acceptance tests pending again. Later implementation may proceed, but its behavior is provisional until its dependencies are accepted.

For simultaneous work on multiple stages, separate worktrees can keep checkouts/build artifacts isolated. Branching alone does not prevent flashing the wrong ELF; identify the active stage and exact build in the test record. Do not assume uncommitted files or captures are shared between worktrees.

## Stage-status record template

Each stage branch should carry a short status document, with a distinct filename per stage to avoid overwriting its predecessor's record:

```text
Stage and branch:
Prerequisite stages and revisions:
Scope and excluded features:
Implementation status:
Software checks and results:
Hardware-tested commit/build:
Uncommitted changes included in build, if any:
Board/motor/encoder configuration:
Bench configuration and operating limits:
Acceptance criteria:
Results and evidence paths:
Outstanding issues and invalidated checks:
Promotion decision and date:
```

These stage records supplement the canonical dated Bring-up log; they must not become competing project histories. Save substantive user reports in that log and refresh the development handoff when current state changes, as required by repository `AGENTS.md`.

## Motor/controller characterization alongside implementation

The developer's planned load rig couples the BLDC shaft to a DC motor used as a generator feeding an electronic load in constant-current mode. As clarified 2026-10-15, the generator housing is free to rotate; a rod attached to its housing reacts against a scale and holds it approximately stationary. This is a reaction-torque fixture. At steady speed, calibrated torque is scale force times perpendicular lever distance; housing/support friction and wire torque must be accounted for.

Use [Motor-Controller-Characterization-Plan.md](Motor-Controller-Characterization-Plan.md) for fixture calibration, generator compliance, motor/controller tests, physics equations, telemetry, and evidence fields. It covers torque from mass readings, mechanical/electrical power, generator CC behavior, dq torque/current conventions, copper loss, efficiency measurement boundaries, and uncertainty.

Integrate the work as follows:

| Stage | Characterization work |
| --- | --- |
| Before powered loading | Calibrate scale/arm, quantify fixture bias, verify generator/load voltage compliance at permitted speed, and record hardware ratings and test bounds. |
| 12A | Preserve existing implementation and reported characterization evidence; no new measurement implied. |
| 12B | Calibrate the rig and record bounded voltage-mode loaded baselines; no current/speed regulation is implied. |
| 13A | Plan timestamped telemetry, bounded profiles, actual/requested voltage reporting, and external-instrument association. |
| 13D–13E | Validate rotating current tracking under load, torque versus measured Iq, repeatability, thermal behavior, and saturation response. Torque mode does not guarantee constant speed. |
| 14 | Fixed-speed load sweeps, torque-speed maps, speed disturbance response, and drive/generator efficiency at explicitly defined measurement boundaries. |
| Later position/field weakening | Generator CC cannot supply controlled static torque at zero speed. Position holding needs another fixture; field weakening retains Stage 14A prerequisites. |

No generator specifications, calibrated torque results, or new accepted limits have been provided yet. Characterization remains pending independently of branch implementation progress.

## Outer-loop development after current control

| Future branch scope | Relationship to validated current control |
| --- | --- |
| Torque mode | Bounded Iq commands, ramps, command timeout behavior, thermal limits, and torque calibration if commands use N·m. |
| Speed control | Outer speed PI generates a limited Iq reference. Validate inner-loop behavior before tuning the outer loop. |
| Position control | Outer position controller generates a limited speed reference, with position limits and trajectory generation. |
| Field weakening | Separate extension after current and ordinary speed control pass, following Stage 14A's motor-specific limits, total-current budgeting, and bus-energy prerequisites. |

Later controllers can be developed ahead on dependent branches. Hardware validation proceeds from the inner current loop outward. Do not describe speed, position, or field weakening as accepted merely because code exists or a draft PR is ready.

## Related documents

- [Original bring-up plan and dated results](BLDC%20Controller%20Firmware%20Bring-Up%20Test%20Plan.md)
- [Current development handoff](Firmware-Development-Handoff.md)
- [Personal firmware walkthrough](Firmware-Walkthrough.md) — local, gitignored; may not exist in other checkouts.
- [Persistent agent instructions](../AGENTS.md)

This temporary outline is not added to `.gitignore` by this request. Only the personal walkthrough was previously requested to be ignored. No commit or push has been performed.

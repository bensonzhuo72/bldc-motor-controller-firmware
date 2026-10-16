# Motor/controller characterization: generator load and reaction arm

Date: 2026-10-15. Status: proposed test/implementation specification; no firmware changes or bench results established here.

Developer/setup source: **Nathaniel Su**. Documentation and proposed analysis prepared by the Codex agent from Nathaniel's requests and clarifications; this does not imply completed hardware validation.

## Intended setup and measurement boundary

Nathaniel Su proposes coupling the BLDC motor to a DC motor connected to an electronic load in constant-current (CC) mode, with a glued rod acting as a lever arm against a scale to measure torque.

**Nathaniel Su's clarification, 2026-10-15:** the DC load motor housing is free to rotate about the shaft axis; only the rod attached to that housing reacts against the scale and nominally keeps the housing still. This is the intended reaction-torque arrangement, not a rigidly bolted generator housing. The support construction, friction, wiring torque, and attachment rigidity still require characterization. “Free spinning” here describes the housing's rotational freedom, not an electrically unloaded generator rotor.

This plan assumes the DC load machine is a **permanent-magnet brushed DC motor used as a generator**. Confirm its model and construction before applying the generator equations. A wound-field machine needs its field specified; a brushless generator needs suitable rectification and a different terminal-voltage/current interpretation.

```text
DC supply -> controller -> BLDC under test -> shaft coupling -> DC generator
                                                               |
                                                    voltage/current measurement
                                                               |
                                                     electronic load, CC mode

DC generator housing -> reaction arm -> scale force -> torque measurement
```

The reaction arm belongs on the **generator housing/stator**, not on the rotating shaft. Support that housing in a cradle/pivot concentric with the shaft so its reaction torque is carried by the arm and scale. Keep the BLDC housing fixed. A generator housing rigidly bolted to the bench provides an unmeasured parallel torque path, so its scale reading cannot be assumed to represent total torque.

The proposed glued connection must remain rigid under the planned torque and temperature. Record arm attachment, material, adhesive/cure, and observed slip/creep. Provide restraint for unexpected housing rotation and keep the arm separate from rotating parts. Validate the attachment with known static torque before powered measurements; the proposal itself does not establish its load capacity.

Support machine weight independently of the scale where practical. Route flexible generator wires so they do not act as torsion springs. Measure the perpendicular distance from shaft axis to the scale force line, not simply rod length. Maintain contact without side loading the scale. Reverse rotation may lift the arm off a compression-only scale: reconfigure/preload and recalibrate the fixture rather than assuming it measures both signs.

At steady speed, a correctly supported generator reaction measurement approximates torque entering the generator shaft, including the generator's internal mechanical/electromagnetic losses reacted through its housing. External cradle friction, wire torque, and support paths can bias it. Coupling losses between the BLDC shaft and generator are outside that boundary. During acceleration, generator rotor inertia adds shaft torque not represented by a simple stationary housing-reaction reading; use steady plateaus for the first torque/power map.

## Confirm and record before selecting numerical test points

- Generator model: PM brushed or other construction, terminal polarity, resistance, torque/back-EMF constants and units, speed/current/temperature ratings, cooling, and any gearbox ratio/backdrive suitability.
- Electronic-load model: CC regulation range, minimum input voltage, maximum voltage/current/power, transient behavior, reverse-voltage tolerance, and input protection. Never assume generator polarity reversal is acceptable to a unipolar load.
- Scale: resolution, capacity, update rate/filtering, repeatability, calibration, and the force direction it measures. Arm perpendicular moment distance and its uncertainty.
- Mechanical assembly: coupling type/alignment, shaft/cradle supports, attachment proof check, motor/encoder identity, permitted speed, and rotor guarding.
- Electrical measurement points and instrument accuracy, including controller supply versus VM and generator terminals versus electronic-load terminals.
- Temperature sensing for the BLDC, controller, and generator; permitted current, voltage, speed, duration, temperature, and shutdown response. The existing 0.55/0.65 A continuous-motor-rating discrepancy remains unresolved.

Current firmware limits remain unchanged: Stage 12 voltage mode, |Vq| <= 2 V, nominal 1.5 A phase trip, 93% duty cap. A trip threshold is not a continuous-current rating. No numerical load-current sweep is authorized or established by this document.

## Physics reference and units

### Scale force and torque

For scale readings expressed as mass, subtract the fixture tare at the same geometry:

```text
delta_m = m_loaded - m_tare                    [kg]
F = delta_m * g                               [N], g = 9.80665 m/s²
T = |r x F| = r * F * sin(phi) = F * r_perp   [N*m]
```

`phi` is the angle between arm vector and force; `r_perp` is the shortest distance from shaft axis to the force line. With a horizontal arm pressing vertically on a scale, the horizontal shaft-to-contact distance is the moment arm. Divide a gram reading by 1,000 before applying the equation. If the instrument already reports newtons, do not multiply by g again. Assign torque sign from the fixture direction.

Example only: a 100 g tare-subtracted reading at 0.10 m perpendicular distance gives `T = 0.100 * 9.80665 * 0.10 = 0.0981 N*m`.

Calibrate with known torques rather than relying solely on nominal scale accuracy:

```text
T_known = m_known * g * r_cal_perp
T_calibrated = a * (scale_reading - zero) + b
```

Fit/check several points in increasing and decreasing order; record hysteresis and zero return. Use either the validated calibration relationship or the geometric conversion consistently. Do not automatically subtract the spinning generator's open-circuit drag as “tare”: that drag is real shaft load. Record static fixture tare and dynamic no-load loss separately.

For independent errors, a useful first estimate is:

```text
u_T ≈ sqrt((r_perp * u_F)² + (F * u_r)²)
```

Add fixture friction, hysteresis, contact-angle error, and drift as separate uncertainty contributions. Near zero torque, report absolute uncertainty rather than misleading relative percentages.

### Speed, mechanical power, and dynamics

```text
omega_m = 2*pi*n/60                         [rad/s], n in rpm
omega_e = p * omega_m                      [electrical rad/s], p = 11
P_shaft = T_shaft * omega_m                [W]
T_motor - T_load - T_other_loss = J_total * d(omega_m)/dt
```

Use consistent signed coordinates; positive delivered power requires matching torque and speed signs. In ordinary steady motoring, report positive magnitudes with direction recorded separately. The dynamic equation is a balance, not a justification for fitting inertia from filtered kitchen-scale transients. Shaft torque at the generator also includes its acceleration torque `J_generator * alpha` during transients.

### DC generator and constant-current loading

For a PM brushed generator in one direction, using positive generated voltage/current magnitudes:

```text
E_g = Ke_g * omega_g
V_g ≈ E_g - I_g * R_g - V_brush              (steady state)
T_em_g = Kt_g * I_g
T_load_shaft ≈ Kt_g * I_g + T_loss_g(omega_g, temperature)
P_load_terminal = V_g * I_g
P_copper_g = I_g² * R_g
```

With compatible SI definitions, `Kt_g [N*m/A] = Ke_g [V/(rad/s)]`. A gearbox or rectifier changes the relationship between measured shaft/terminal quantities and machine constants.

CC mode holds generator current approximately constant only within the load's voltage/power compliance. This makes electromagnetic braking torque approximately constant, not precisely measured shaft torque and not constant power. Generator friction, magnetic losses, temperature, and load regulation still matter. Always record actual current rather than only the CC setpoint.

The approximate minimum-speed condition for maintaining the requested current is:

```text
Ke_g * omega_g >= V_load_min + I_g*(R_g + R_wires) + V_brush
```

Include any rectifier voltage drops if present. At zero speed there is no generated EMF, so this passive generator/electronic-load setup cannot provide a controlled static holding torque. Below compliance, mark the point out of regulation instead of interpreting it as a controller torque error. The rig is therefore unsuitable by itself for static position-loop load tests.

### BLDC torque and current convention

For the amplitude-invariant dq convention used by this firmware:

```text
T_em = (3/2) * p * [psi_m * Iq + (Ld - Lq) * Id * Iq]
T_em ≈ (3/2) * p * psi_m * Iq = Kt_q * Iq   (Id = 0; simplified model)
I_vector = sqrt(Id² + Iq²)
I_phase_rms = I_vector/sqrt(2)               (balanced steady sinusoidal currents)
P_copper_motor = 3*R_phase*I_phase_rms²
               = (3/2)*R_phase*(Id² + Iq²)
T_shaft ≈ T_em - T_internal_loss             (steady speed)
```

The stored phase-equivalent values are R=2.2 Ω and L=0.54 mH; Ld/Lq and flux linkage are not independently established. The stored Ke=0.1562 uses a line-to-line convention whose peak/RMS interpretation must be resolved before deriving Kt_q. Do not equate that number directly to torque per amp of Iq.

At fixed speed and comparable temperature, fit measured reaction/shaft torque against measured Iq:

```text
T_measured ≈ Kt_q_fit * Iq + b(speed, temperature)
```

The intercept can contain motor losses, coupling losses, and fixture bias. Prefer a multi-point slope and residuals over `T/Iq` at one low-current point. This is a shaft-output characterization, not automatically a calibrated electromagnetic torque constant. If current scaling is still uncertain, the fitted slope inherits that uncertainty; the previously waived phase-current meter check is not retroactively satisfied by this fit.

### Electrical power and efficiency boundaries

```text
P_controller_input = mean(V_input(t) * I_input(t))
P_generator_output = mean(V_g(t) * I_g(t))
eta_drive_to_generator_shaft = P_shaft_at_generator / P_controller_input
eta_generator = P_generator_output / P_shaft_at_generator
eta_end_to_end = P_generator_output / P_controller_input
```

At steady, low-ripple plateaus, products of averaged voltage/current are approximations; for ripple/transients use synchronized products. State whether the controller input includes logic, relay coil, wiring, and auxiliary loads. The board ACS71240 measures the VM supply path and has approximately 30.5 mA/count resolution, so low-power efficiency measurements may need an external instrument.

Generator terminal power is **not** BLDC shaft power: their difference includes generator losses. Separating controller efficiency from BLDC motor efficiency additionally requires calibrated motor electrical power measurement:

```text
P_motor_electrical = mean(va*ia + vb*ib + vc*ic)
                  = (3/2)*mean(vd*id + vq*iq)  (compatible actual voltages/currents)
eta_controller = P_motor_electrical / P_controller_input
eta_motor = P_motor_shaft / P_motor_electrical
```

Use actual phase voltages with a consistent reference; when phase currents sum to zero, a common-mode voltage shift cancels in total power. Commanded Vd/Vq do not include every dead-time, switch-drop, bus, and saturation error. Firmware command-based power is an estimate, not an independent inverter-efficiency measurement. Do not report separate controller/motor efficiencies without those additional measurements and matching boundaries. Report efficiency only for valid nonzero input power and with uncertainty; apparent values above 100% indicate a boundary, synchronization, or measurement problem.

## Characterization sequence and stage integration

### C0 — Fixture calibration and generator suitability

Before a powered sweep, verify arm geometry, scale calibration/zero return, cradle friction, wire torque, coupling alignment, attachment rigidity, and contact direction. Determine whether the chosen generator produces enough voltage at the BLDC's permitted speed for CC regulation. Record minimum compliant speed/current and instrument limits. A different generator or ratio may be needed; do not increase motor speed merely to make an unsuitable load regulate.

Record an open-circuit/no-command-load speed sweep within already accepted operating limits, including torque, generator voltage, and temperature. Distinguish generator no-load drag from fixture tare. Calibrate the rig before using it to declare Stage 13 acceptance.

### C1 — Stage 12B voltage-mode hardware baseline

Stage 12A preserves the already implemented firmware and existing evidence. Stage 12B performs the remaining hardware checkout; fixture calibration C0 and the bounded baseline below feed its acceptance matrix. No new bench result is implied by this split.

Within the current ±2 V/1.5 A-trip envelope, establish repeatable steady points with small generator loads only where the electronic load regulates. Record actual speed, Id/Iq, phase current, Vq, VM, generator V/I, scale force, torque, and temperatures. Voltage mode has no current regulation: use bounded test duration and the existing shutdown path; do not interpret speed droop as a speed-loop failure. Abort a point if the setup leaves its defined envelope.

### C2 — Stage 13D/13E current tracking and torque map

After limited standstill current tests pass, use the calibrated rig for low-speed rotating current-loop tests. Sweep approved Iq commands with Id reference zero and bounded load changes. Verify tracking, saturation recovery, phase-current limits, temperature, and ISR timing. Current/torque mode does not regulate speed; supervise overspeed and do not expect arbitrary motor-Iq and load-current settings to yield a stable plateau. If speed cannot settle within the allowed envelope, use another operating point or defer fixed-speed mapping to the speed-loop stage.

Collect increasing/decreasing load points and repeats to expose heating/hysteresis. Fit torque versus measured Iq at matched speed/temperature where possible. Use automatic high-rate firmware capture for current transients; the scale normally supports average torque only, not 20 kHz ripple or current-loop bandwidth claims.

### C3 — Stage 14 speed/load and drive-efficiency map

With speed control accepted, hold speed at several permitted setpoints and step actual generator load current through a bounded range. Record torque-speed curves, Iq demand, speed error/recovery, saturation headroom, input/output power, temperature, and plateau variance. Reverse tests require an appropriate electrical-load polarity arrangement and scale reaction geometry. Repeat selected points to quantify drift.

Include combined controller/motor/coupling efficiency at the measured boundary and generator efficiency; separate controller/motor values only if additional calibrated electrical measurements exist. Begin with time-limited points, then use approved thermal-duration tests to establish an operating envelope rather than assuming peak ratings are continuous.

### C4 — Later position and field-weakening work

This rig supplies speed-dependent generator braking, not controlled static load at standstill. Position hold/stiffness tests require a separately specified static torque fixture or active dynamometer. Field-weakening comparisons may use the generator map only after Stage 14A prerequisites pass; use matched speed/load/temperature, account for Id heating and torque headroom, and keep the validated bus-energy exit strategy.

## Future firmware facilities and evidence

Proposed facilities, not currently implemented by this document:

- Timestamped stage/mode/build identity and synchronized snapshots of Id/Iq commands and measurements, phase-current peaks, requested/applied Vd/Vq, VM, duty extrema, saturation and limit states, encoder speed/health, and ISR/late-sample counts.
- Bounded command profiles, timed plateaus/captures, and explicit abort limits; no debugger pause required while armed. A PC load-control/scale interface, if desired, is a separate task and must use confirmed device protocols.
- Associate external scale, load, and power-meter readings with stable plateau timestamps. Record scale update/filter delay; use settle windows long enough for both mechanical and measurement response.

Suggested record columns:

```text
run_id, timestamp, stage, branch, tested_commit/build, board_id, motor_id,
generator_id, coupling/ratio, direction, arm_r_perp_m, tare, calibration_id,
speed_target_rpm, speed_mean/min/max, Id_ref_A, Id_mean_A, Iq_ref_A, Iq_mean_A,
phase_peak_A, VM_mean/max_V, controller_input_V/A/W, Vd/Vq_requested/applied,
load_CC_set_A, generator_actual_V/A/W, load_in_regulation,
scale_mean/stddev, force_N, reaction_torque_Nm, shaft_power_W,
BLDC/controller/generator_temperature, duration_s, duty_max, saturation,
ISR_max_us, late_samples, faults, uncertainty, evidence_paths, result/notes
```

Accept the characterization dataset when calibration and repeatability are recorded, each point has valid load regulation and measurement boundaries, prescribed tracking/thermal/mechanical/electrical limits hold, and exclusions/failures are retained. Define numerical tracking and uncertainty tolerances before testing; no pass is claimed here.

## Branch integration

Add fixture definition, calibration records, and generator compliance data to the relevant stage's documentation. Preserve existing work on proposed `stage/12a-existing-baseline`; C0/C1 checkout evidence belongs to proposed `stage/12b-hardware-checkout`. Future telemetry/profile support fits `stage/13a-current-foundation`; loaded current validation fits 13D/13E; regulated-speed maps fit Stage 14. Keep characterized results attached to exact tested revisions, as described in [the temporary branch plan](TEMP-Current-Control-Branch-Plan.md). Implementation may progress on dependent branches while measurements remain pending; energized Stage 13 tests and validated promotion await 12B acceptance.

## References

- [maxon: Motors as Generators](https://support.maxongroup.com/hc/en-us/articles/360004496254-maxon-Motors-as-Generators): generator voltage/current behavior, torque demand, loss, and rating considerations. Supports the generator model; does not qualify this unspecified DC motor.
- [Magtrol: reaction torque measurement on a dynamometer stator](https://www.magtrol.com/product/wb-23-and-wb-27-high-speed-eddy-current-dynamometers/): reference for the reaction-measurement principle, not validation of this custom fixture.
- [Original bring-up plan](BLDC%20Controller%20Firmware%20Bring-Up%20Test%20Plan.md): firmware stages, sign/unit conventions, existing motor measurements, and current limits.

Equations and the procedure above are a proposed engineering model/test plan. The housing reaction arrangement is confirmed by the developer; generator type, detailed arm/support geometry, scale/load specifications, and numerical test envelopes still need recording from the actual setup.

# RocketFusion

Virtual rocket + virtual sensors + a benchmark for **onboard operational-state-transition detection**
in 1- and 2-stage rockets, with and without multisensor feature-level fusion.

Developed within Minerva Rockets (UFRJ). Companion code for the paper
*"Feature-Level Multisensor Fusion for Onboard Operational State Transition Detection in a Two-Stage Rocket"*
(title/citation to be updated).

## What it does

1. **Flight simulator** (`dynamics.py`): planar point-mass dynamics with an optional altitude-sheared crosswind,
   ISA atmosphere, Mach-dependent drag, staging (burnout, separation, 2nd-stage ignition), drogue + main parachutes.
   The zero-wind default preserves the vertical baseline; ground-truth event times are recorded.
2. **Virtual sensors** (`sensors.py`): an explicit transducer chain (see *Sensors* below) for the IMU (low-g and high-g
   accelerometers + gyroscope, LSM6DSK320X) and a barometer (MS5611), plus the physical effects that act on them (motor vibration,
   separation pyro-shock, static-port error, **transonic pressure disturbance**, port lag, dropouts).
3. **Processing and estimation** (`preprocess.py`, `kalman.py`, `estimators.py`, `features.py`): common-rate resampling, pad
   calibration, accelerometer channel merging, three vertical filters (fixed-R Kalman, Mach-adaptive Kalman, adaptive Kalman with an
   aerodynamic pseudo-measurement) and causal windowed features grouped as `accel`, `gyro` (together `imu`), `baro`, `fusion`.
4. **Detectors** (`detectors.py`), all sequential and with a persistence criterion (temporal consistency):

   | name | description |
   |---|---|
   | `B1_imu_rules` | single-sensor threshold rules (accelerometer + dead reckoning) |
   | `B1_baro_rules` | single-sensor rules (barometer only; unobservable events are not reported) |
   | `B2_kalman_fsm`, `B3_adaptive_kf_fsm`, `B4_aero_ekf_fsm` | accelerometer rules + altitude/velocity of the fixed-R, adaptive and adaptive+aero estimators for apogee and main |
   | `tree_accel_only`, `tree_imu_only`, `tree_baro_only` | decision trees on accelerometer, accelerometer+gyro, barometer features |
   | `tree_imu_baro` | decision trees on IMU and barometer features |
   | `tree_fusion` | decision trees on IMU + baro + fusion (estimator outputs) features |
  | `tree_extra_fusion` | ExtraTrees ensembles, one event classifier each, on the same fused features |
  | `tree_boost_fusion` | gradient-boosted trees, one event classifier each, on the same fused features |
  | `mlp_fusion` | dense neural network (MLP) predicts the operating state from fused features |
  | `knn_fusion_anomaly` | KNN state prediction with distance-based rejection of unfamiliar feature windows |

  Learned state predictions and event candidates pass through the same persistence and mission-graph decoder. Trees
  classify one transition at a time; the MLP classifies the current state; KNN classifies the state but rejects a sample
  when it is too far from training examples for that state.
5. **Monte Carlo experiment** (`experiment.py`): parameter jitter per flight, eight sensor-degradation
   scenarios plus vehicle-failure cases and mixed sensor/vehicle failures, metrics and figures.

## Sensors (virtual LSM6DSK320X + MS5611)

Each virtual sensor channel applies, in order: dynamic-range clipping -> sensitivity threshold -> hysteresis ->
linearity error -> sensitivity (gain) and offset -> noise -> range clipping and quantisation.
`python -m rocketfusion.characterize --preset minerva` measures all of them with static up/down sweeps and writes
`sensor_characteristics.json`, `fig_static_error.png` (error vs input, increasing and decreasing) and
`fig_transducer_characteristics_table.png` (the same measurements as a table image).

* **IMU = LSM6DSK320X**, three signals: a **low-g accelerometer** (+/-16 g, precise but it saturates), a **high-g accelerometer**
  (+/-64 g here, device up to +/-320 g; coarser but it does not saturate at thrust or shock peaks) and a **gyroscope**
  (+/-4000 dps). While the low-g channel is saturated the software uses the high-g one (`SensorConfig.imu_dual`);
  each channel is calibrated on the pad so that switching does not create a step.
* **Barometer = MS5611-01BA03**, plus a static-port error modelled as a pressure coefficient (see *Transonic barometer model*).

| Characteristic | Definition used | Where it is set (`SensorConfig`) |
|---|---|---|
| **Sensitivity** | slope output/input (gain error, unit to unit) and the smallest input change that moves the output (threshold / dead-band) | `*_scale_err`, `*_threshold_*` |
| **Dynamic range** | min/max measurable input; beyond it the output saturates | `imu_range_g`, `hg_range_g`, `gyro_range_dps`, `baro_min_pa`, `baro_max_pa` |
| **Precision** | scatter of repeated readings of a constant input (white noise 1 sigma) + turn-on offset repeatability | `*_noise_*`, `*_bias_*` |
| **Resolution** | quantisation step (ADC LSB) | `imu_bits`, `hg_bits`, `gyro_bits`, `baro_res_pa` |
| **Accuracy** | max abs(output - true input) over the range; *emergent* (gain, offset, linearity, hysteresis, noise), measured by `characterize` | (not a parameter) |
| **Linearity** | max deviation from the end-point line, % of full span | `*_nonlin_pct_fs` |
| **Hysteresis** | width of the up/down loop, % of full span | `*_hyst_pct_fs` |

Measured characteristics of the Minerva preset (seed 0; `results/sensor_characterisation/`):

| | low-g accel. | high-g accel. | gyroscope | barometer |
|---|---|---|---|---|
| dynamic range | [-16, 16] g | [-64, 64] g | [-4000, 4000] dps | [1000, 120000] Pa |
| sensitivity error [%] | 0.63 | 0.624 | 0.42 | -0.186 |
| sensitivity threshold | 0.005 g | 0.02 g | 0.05 dps | 1 Pa |
| resolution | 0.000488281 g | 0.00195312 g | 0.12207 dps | 1.2 Pa |
| precision (1 sigma) | 0.03048 g | 0.1401 g | 0.1537 dps | 4.147 Pa |
| linearity [% FS] | 0.2754 | 0.459 | 0.0918 | 0.0459 |
| hysteresis [% FS] | 0.1015 | 0.2041 | 0.0203 | 0.0201 |
| accuracy (max error) | 0.1122 g | 0.6121 g | 24.65 dps | 329.5 Pa |

The ranges and the resolution of the barometer come from the team's slides; **noise, linearity, hysteresis, bias and the
high-g/gyro figures are ASSUMED** plausible values, not datasheet numbers: replace them with datasheet or bench values via `--sensor-config`.
The generated Minerva table is also available at
[`results/sensor_characterisation/fig_transducer_characteristics_table.png`](./results/sensor_characterisation/fig_transducer_characteristics_table.png).

### Scripted attitude motion (gyroscope input)

The point-mass model does not integrate attitude, so the body rates seen by the gyroscope are **scripted, not simulated** (`RocketConfig`):
roll rate = `roll_dps_per_ms` x airspeed (canted fins; damped by the parachute after apogee), a transverse-rate **tip-off** at
stage separation (`tipoff_dps`, absent when the separation fails), a **pitch-over** pulse after apogee (`pitchover_dps`) and
weathercocking noise proportional to dynamic pressure (`weathercock_dps`). These rates do not feed back into the trajectory.

### Transonic barometer model

The static-port error is a pressure coefficient, `p_meas = p_static + Cp(M) q`, with
`Cp = dyn + trans * exp(-((M - c) / w)^2) + sup * smoothstep(M > c + 1.5 w)`: a spike around Mach 1 (default 0.02 q, about 90 m at 3 km)
and a residual supersonic offset (0.006 q). The altitude error therefore grows with v^2 (about `Cp v^2 / 2g`). **The shape and the
magnitudes are an assumption** (no altimeter logs were available): calibrate them with COTS-altimeter flight logs before claiming realism.

The virtual barometer does **not** automatically shut off at Mach 1 or at the Minerva target apogee. Its pressure
range is 1,000-120,000 Pa; ISA pressure at 10 km is about 26,400 Pa, comfortably inside that range. Near Mach 1,
the model instead perturbs the static-port pressure by `Cp(M) q`, which can bias the inferred altitude; this is a
modeled aerodynamic measurement error, not a sensor dropout. A dropout is separately injected in the `baro_dropout`
scenario (sample-and-hold from 7 to 14 s after liftoff) and in training scenario `sep_baro_dropout`. The Minerva
rocket preset is calibrated to about 10 km in nominal flight; vehicle-failure scenarios can have lower apogees.
The [nominal example plot](./results/tree_fusion_comparison/fig_1_sensors_nominal_val_seen.png) shows barometer
altitude error alongside Mach.

## State estimators (`estimators.py`)

All estimate altitude h, vertical velocity v and accelerometer bias, using the accelerometer as the input of a kinematic
model (in 1-D, a = f_specific - g exactly) and the barometric altitude as the measurement.

| estimator | idea |
|---|---|
| `Kalman (fixed R)` | baseline: constant trust in the barometer |
| `adaptive Kalman` | barometer noise R grows (i) around the **estimated** Mach 1, (ii) with the estimated dynamic pressure (error ~ `cp v^2 / 2g`) and (iii) with the normalised innovation (chi-square test, Huber-like): the barometer is down-weighted instead of switched off |
| `adaptive Kalman + aero` | adds a velocity pseudo-measurement during coast: with no thrust the accelerometer reads pure drag, `v = sqrt(2 m abs(f) / (rho CdA(M)))`, which needs no barometer. It uses the *design* drag table and mass (with an 8 % Cd error, ASSUMED) and is chi-square gated |
| `baro only (raw)`, `IMU dead reckoning` | reference: differentiated pressure altitude, integrated accelerometer |

`cp_sigma` of the adaptive filter was chosen knowing the order of magnitude of the *assumed* transonic model: with real data
it must be re-identified.

## Figures: what each one measures

Every figure carries its own caption; `FIGURES.md` in each results folder repeats them in English and Portuguese.

| figure | what it measures / how to read it |
|---|---|
| `fig_1_sensors_*` | what each sensor reads vs the truth for one flight: both accelerometers and the merged signal, gyroscope roll and transverse rate, barometer altitude error with the Mach number. Red band = transonic window, grey bands = motor burning |
| `fig_2_fusion_*` | velocity estimate, velocity error and altitude error for the three Kalman estimators (0 = perfect) |
| `fig_3_estimation_error_by_phase` | RMSE of the three Kalman estimators by flight phase; lower is better |
| `fig_4_detection_success_heatmap` | event success within +/-1 s for selected Kalman, tree and KNN detectors; one panel per test scenario |
| `fig_5_detection_delay` | event detection time minus true event time for those detectors; grey band = +/-1 s tolerance |
| `fig_6_overall_success` | fraction of occurred events detected within +/-1 s, per scenario |
| `fig_7_sequence_correct` | fraction of flights with every event right and nothing spurious |
| `fig_8_timeline_*` | one flight: where each selected detector placed each event relative to the truth |
| `fig_9_stage_detection_summary` | on-time event detection by flight-stage transition for the selected detectors |
| `fig_11_inference_latency` | median and p95 wall-clock time of the selected detectors' full-flight `detect()` calls |
| `fig_12_tree_fusion_diagram` | visual flow of tree_fusion inputs, event trees, persistence and mission decoder |
| `fig_13_experiment_methodology` | English workflow of the simulation, feature preparation, training, decoding and held-out evaluation, with current validation gaps |
| `fig_14_flight_environment_table` | English table of Minerva vehicle mass, sensor settings, atmosphere, wind-model status and nominal flight example |
| `fig_15_tree_fusion_pipeline_en` | English end-to-end feature, classifier, persistence and mission-decoder pipeline |
| `fig_16_tree_model_comparison_en` | English visual comparison of single-tree, ExtraTrees and boosted-tree event classifiers |
| `fig_wind_detector_comparison` | held-out detector metrics and Python-host latency across the wind sweep |
| `fig_wind_trajectory` | nominal altitude and crosswind drift under the configured wind profiles |

For readability, detector-result figures show only the three Kalman-FSMs, `tree_fusion`,
`knn_fusion_anomaly`, and (when `--compare-tree-ensembles` is enabled) the ExtraTrees and boosted-tree
fusion candidates. Estimation figures show only the three Kalman
estimators because tree and KNN detectors predict events, not altitude/velocity. Full metrics for all detectors
remain in the CSV outputs.

### Research scope vs. the abstract

The repository is a valid **simulation-based prototype** for comparing event-transition detectors and exploring
sensor-failure robustness. It does not, by itself, substantiate claims of STM32 deployment, hardware-in-the-loop
or real-flight validation, nor embedded memory/latency measurements. An optional planar point-mass crosswind
model is now available, but it does not simulate attitude, lift, or wind-driven body rotation; some sensor and
transonic-port characteristics are explicitly assumed. The archived held-out benchmark also does not show tree
fusion outperforming the Kalman-FSM baseline (sequence-correct: 0.56 vs. 1.00), and the KNN result was 0.00 in
that earlier run. These figures are evidence about the previous simulation setup, not evidence that the
approach cannot work with improved features, training coverage or a different validation setup.

Accordingly, the abstract is defensible as a **research objective and proposed architecture**, but should describe
STM32/HIL and real-flight validation as future work unless results from those activities exist elsewhere. A
stronger empirical claim requires repeated, sufficiently large held-out runs, calibrated wind/trajectory models,
sensor parameters calibrated against datasheets or bench/flight data, and target-hardware measurements.

## Minerva Rockets vehicle preset (`--preset minerva`)

A preset built from the team's slides (motor data sheets, CONOPS, parachute sizing, sensor choices).
Run: `python -m rocketfusion.experiment --preset minerva --out results/minerva`.

| Item | Value | Status |
|---|---|---|
| Booster motor | Olympus v2.0 (O3135): 27 446 N.s, 3.45 s, Isp 130.5 s -> 21.5 kg propellant | from slide; thrust-curve shape read off the OpenMotor plot |
| Booster motor (alt.) | Motor O V2.0: 29 409 N.s, 4.53 s, Isp 128.2 s -> 23.4 kg propellant | from slide; curve shape ASSUMED |
| Sustainer motor | Notus v2.0 (N3112): 12 885 N.s, 4.11 s, Isp 116 s -> 11.3 kg propellant | from slide; curve shape ASSUMED |
| Timeline | separation 9.53 s, sustainer ignition ~12.6 s, burnout 16.68 s, apogee ~46.8 s | CONOPS (some digits are blurry in the slide) |
| Recovery | pilot at apogee, reefed main +1 s, main at 457.2 m (1500 ft); D_R = 0.289 m, D = 2.729 m | slides; reefed CdA fitted to CONOPS main time (~158 s); inflation time ASSUMED |
| Masses | booster dry 29.8 kg, sustainer dry 26.1 kg, Cd0 0.204 | **FITTED** to a 10 000 m apogee (`calibrate_minerva.py`); area ASSUMED (200 mm body) |
| IMU | LSM6DSK320X low-g channel, +/-16 g, 16 bit | slide; noise/linearity/hysteresis ASSUMED |
| Barometer | MS5611-01BA03, 10-1200 mbar, ~10 cm resolution (1.2 Pa) | slide; accuracy ~+/-1.5 mbar from memory, **check the datasheet** |

**Which booster?** With the **Motor O V2.0** the CONOPS points (burnout altitude/velocity, separation, sustainer burnout)
are reproduced within ~2 tolerances; with Olympus v2.0 the booster-burnout altitude is ~6 tolerances off.
`--preset minerva` therefore aliases `minerva_motor_o`; use `minerva_olympus` for the Olympus booster.
Please confirm with the propulsion team which motor the CONOPS used.

**Which apogee?** `minerva` (default) is fitted to **10 000 m**, `minerva_conops` to the 9490 m of the CONOPS slide. The fit
changes only dry masses and Cd0; with 10 km all CONOPS points remain within ~2 tolerances (apogee 9995 m, apogee time 48.1 s vs 46.8 s).

`python calibrate_minerva.py [motor_o|olympus] [--apogee 10000]` prints the model-vs-CONOPS table and writes
`reference_results/minerva_validation_<booster>_<apogee>m.png`. Mach max 1.52 (CONOPS: ~1.5), peak specific force
9 g under thrust and ~12 g at main-parachute opening, i.e. below the 16 g low-g range in nominal flight.

## Train / validation / test protocol (`--protocol`)

Three roles, never mixed (`rocketfusion/splits.py`, seed ranges asserted disjoint at start-up):

| role | scenarios | flights | purpose |
|---|---|---|---|
| **train** | nominal, high_noise, baro_transonic_severe, low_resolution, sep_failure, sep_baro_dropout | 120 (vehicle spread +/-3 %) | fit trees, MLP and KNN with a separation failure and a mixed barometer-dropout failure |
| **validation** | the *same* scenarios, **new flights** | 40 | choose the tree depth (candidates 3, 4, 5, 6, 8); never used for fitting |
| **test (held out)** | imu_saturation, baro_dropout, low_cost_sensors, ign2_failure, combined, ign2_imu_saturation | 30 per scenario (vehicle spread +/-5 %) | final evaluation on held-out scenarios, including an unseen mixed ignition/IMU failure |

The held-out split keeps seeds and scenario names disjoint. Training now includes `sep_baro_dropout` (separation
failure plus barometer sample-and-hold dropout); testing includes `ign2_imu_saturation` (second-stage ignition
failure plus IMU saturation), an unseen mixed case. All learned detectors, including the event trees, MLP and KNN,
are fit using the training flights. Other protocols:
`--protocol nominal_only` (train on nominal flights only, test on all) and `--protocol all_seen`
(optimistic upper bound: test scenarios are in training). Lists can be overridden with
`--train-scenarios` / `--test-scenarios`.

```bash
python -m rocketfusion.experiment --preset minerva --protocol held_out --out results/minerva_held_out
```

Each run writes `split_summary.csv` (macro averages per role), `depth_selection_validation.csv`,
`flight_summary.csv` and `mission_check.txt`.

### Apogee check (expected: about 10 km)

The default Minerva vehicle is calibrated to a **10 000 m** apogee (RASAero in the team slides: 10 001 m with 5 m/s
wind; the CONOPS slide gives 9490 m and RocketPy 8673 m, so the tools disagree by about 1.3 km; `--preset minerva_conops`
reproduces 9490 m). In the reference run:

```
[test_unseen] apogee without vehicle failure (n=120): mean 10095 m, std 530 m, range [8895, 11662] m; 69% within 10000 +/- 500 m; Mach max 1.53
[test_unseen] vehicle-failure apogees: ign2_failure: 4395 m
[val_seen] apogee without vehicle failure (n=32): mean 9963 m, std 307 m, range [9412, 10653] m; 91% within 10000 +/- 500 m; Mach max 1.52
[val_seen] vehicle-failure apogees: sep_failure: 8401 m
```

The ignition failure reaches less than half the nominal apogee and the separation failure about 8.4 km.
The spread comes from the +/-3 % (train/validation) and +/-5 % (test) variation of masses, thrust, burn time and Cd; with the
earlier +/-10 % the standard deviation was 1.1 km, too large for a calibrated vehicle. Change it with `--jitter` / `--test-jitter`.

### Results (Minerva vehicle, held-out protocol)

The tables below are the archived 120/40/30 reference run, before the mixed failure scenarios were added. The
updated preview figures are in `results/tree_fusion_comparison`; that run uses only 12 training flights, 6
validation flights and 2 test flights per scenario to keep it quick, so treat its metrics as a smoke/demo run,
not as a statistically reliable benchmark.

**Detection.** `ok` = fraction of occurred events detected within +/-1 s; `seq` = fraction of flights with a fully correct event sequence (macro average over scenarios):

| detector | validation (seen scen.): ok / seq | test (unseen scen.): ok / seq |
|---|---|---|
| B1_baro_rules | 0.41 / 0.00 | 0.45 / 0.00 |
| B1_imu_rules | 0.84 / 0.00 | 0.77 / 0.07 |
| B2_kalman_fsm | 1.00 / 1.00 | 1.00 / 1.00 |
| B3_adaptive_kf_fsm | 1.00 / 1.00 | 1.00 / 1.00 |
| B4_aero_ekf_fsm | 1.00 / 1.00 | 1.00 / 1.00 |
| tree_accel_only | 1.00 / 0.97 | 0.65 / 0.20 |
| tree_imu_only | 1.00 / 1.00 | 0.90 / 0.71 |
| tree_baro_only | 0.92 / 0.47 | 0.79 / 0.34 |
| tree_imu_baro | 1.00 / 1.00 | 0.89 / 0.56 |
| tree_fusion | 1.00 / 0.97 | 0.88 / 0.56 |

Fully correct sequences, validation scenarios (seen, new flights):

| detector | baro_transonic_severe | high_noise | low_resolution | nominal | sep_failure |
|---|---|---|---|---|---|
| B1_baro_rules | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| B1_imu_rules | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| B2_kalman_fsm | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 |
| B3_adaptive_kf_fsm | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 |
| B4_aero_ekf_fsm | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 |
| tree_accel_only | 1.00 | 1.00 | 1.00 | 1.00 | 0.88 |
| tree_imu_only | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 |
| tree_baro_only | 0.75 | 0.38 | 0.75 | 0.50 | 0.00 |
| tree_imu_baro | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 |
| tree_fusion | 1.00 | 1.00 | 1.00 | 1.00 | 0.88 |

Fully correct sequences, **held-out test scenarios**:

| detector | baro_dropout | combined | ign2_failure | imu_saturation | low_cost_sensors |
|---|---|---|---|---|---|
| B1_baro_rules | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| B1_imu_rules | 0.07 | 0.00 | 0.30 | 0.00 | 0.00 |
| B2_kalman_fsm | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 |
| B3_adaptive_kf_fsm | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 |
| B4_aero_ekf_fsm | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 |
| tree_accel_only | 1.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| tree_imu_only | 0.97 | 0.97 | 0.00 | 1.00 | 0.63 |
| tree_baro_only | 0.00 | 0.37 | 0.00 | 0.67 | 0.67 |
| tree_imu_baro | 0.20 | 0.77 | 0.00 | 0.93 | 0.90 |
| tree_fusion | 0.10 | 0.97 | 0.00 | 0.97 | 0.77 |

Occurred events detected within +/-1 s, held-out test scenarios:

| detector | baro_dropout | combined | ign2_failure | imu_saturation | low_cost_sensors |
|---|---|---|---|---|---|
| B1_baro_rules | 0.42 | 0.36 | 0.60 | 0.43 | 0.43 |
| B1_imu_rules | 0.87 | 0.71 | 0.86 | 0.71 | 0.71 |
| B2_kalman_fsm | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 |
| B3_adaptive_kf_fsm | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 |
| B4_aero_ekf_fsm | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 |
| tree_accel_only | 1.00 | 0.43 | 0.60 | 0.43 | 0.81 |
| tree_imu_only | 1.00 | 1.00 | 0.60 | 1.00 | 0.93 |
| tree_baro_only | 0.64 | 0.87 | 0.57 | 0.93 | 0.95 |
| tree_imu_baro | 0.89 | 0.97 | 0.60 | 0.99 | 0.99 |
| tree_fusion | 0.87 | 0.99 | 0.60 | 0.99 | 0.97 |

### How fusion and the state machine work together

The current `tree_fusion` detector does **not** directly classify the rocket's current operating state. It detects
candidate **events**, and the mission state machine turns those event signals into an ordered state sequence:

1. At each processing sample, causal features are computed from the accelerometers, gyroscope, barometer and estimators.
   The `fusion` group contains fixed-R Kalman altitude, velocity and innovation features, plus adaptive-Kalman and
   aerodynamic-estimator altitude/velocity (and adaptive-filter NIS).
2. `tree_fusion` uses those features together with IMU and barometer features. It has a separate binary decision tree
   for each event; each tree answers "does this event appear to have just happened?", not "which state are we in?".
   Training positives are the 0.5 s after a ground-truth event; samples before it are negatives.
3. The shared sequence decoder starts at `PRELAUNCH`, evaluates only transitions allowed by `NEXT`, and accepts a
   candidate after the configured number of consecutive positive samples (`persist`, 3 by default). It can follow
   defined skip paths for failures, including a confirmation wait before concluding that second-stage ignition was
   skipped. Its output is the detected event sequence, which implies the operating states between transitions.

So fusion features contribute evidence for event detection, while the FSM supplies ordering and mission context. The
current figures report event timing and sequence correctness, not a per-sample state-classification confusion matrix or
which fusion feature caused a tree decision. The exported `tree_fusion_rules.txt` can be inspected to see the learned
feature splits.

### A `tree_fusion` usa Kalman? O modelo é puro?

**Não é uma árvore pura baseada somente em sinais brutos.** O classificador em si é uma árvore de decisão
(`DecisionTreeClassifier`), com uma árvore binária separada para cada evento. Porém, suas entradas incluem IMU,
barômetro e features do grupo `fusion`: altitude, velocidade e inovação do Kalman fixo, além de altitude/velocidade
dos filtros adaptativo e aero-adaptativo. Portanto, o Kalman participa **indiretamente**, fornecendo features; quem
decide o evento é a árvore, não um filtro Kalman. O `knn_fusion_anomaly` também usa essas features de estimadores.
Veja [fig_12_tree_fusion_diagram.png](./results/tree_fusion_comparison/fig_12_tree_fusion_diagram.png) para o fluxo.

### What is ExtraTrees Fusion?

`tree_extra_fusion` uses the same mixed IMU, barometer and Kalman-estimator feature vector as `tree_fusion`.
It does **not** make a separate tree for each sensor. Instead, it replaces the single decision tree for each
event with an ensemble of 16 randomized decision trees; their class predictions are combined into one event
candidate. At each split, the estimator considers a randomized subset of 80% of the features. The candidates
then share the same persistence rule and mission-graph decoder. `tree_boost_fusion` is a different alternative:
it adds 24 shallow trees sequentially, with later trees correcting errors left by earlier stages.

These are alternative classifiers, not evidence that a learned tree has eliminated Kalman filtering: the fused
input vector still includes Kalman-derived altitude, velocity and innovation features. The English diagrams
[`fig_15_tree_fusion_pipeline_en.png`](./results/wind_tree_comparison/fig_15_tree_fusion_pipeline_en.png) and
[`fig_16_tree_model_comparison_en.png`](./results/wind_tree_comparison/fig_16_tree_model_comparison_en.png)
show the shared pipeline and the difference between the three tree-based classifiers.

### What triggers each transition?

The hand-written rules below are thresholds on processed measurements, not direct observations of a motor command or
stage position. `f` is axial specific force in g; `v` and `h` are estimated vertical velocity and altitude. Rule
thresholds are defined in `RuleConfig` and applied at the common 50 Hz processing rate by default.

| transition | IMU / Kalman-FSM rules | barometer-only rules | what the signal means here |
|---|---|---|---|
| `LIFTOFF` | `f > 2 g` | barometric altitude `h_b > 5 m` | sustained acceleration or altitude rise after pad calibration |
| `BURNOUT_01`, `BURNOUT_02` | `f < 0.5 g` | not observable / not reported | thrust has dropped; this is the motor-burnout cue |
| `BECO` | `abs(delta f) > 4 g` between adjacent processed samples; accepted on one sample | not observable / not reported | a sudden axial acceleration change used as a proxy for the separation shock |
| `IGNITION_02` | `f > 2 g` | not observable / not reported | renewed acceleration from second-stage thrust |
| `APOGEE` | estimated vertical velocity `v < 0` | barometric altitude trend `v_b < 0` | velocity changes from ascent to descent |
| `MAIN` | `h < main_alt` and `v < 0` | `h_b < main_alt` and `v_b < 0` | descending below the configured main-deployment altitude |

For `B1_imu_rules`, velocity and altitude are dead-reckoned from the accelerometer: the code subtracts gravity from
specific force, integrates to get vertical velocity, then integrates velocity to get altitude. That is why this
detector can estimate apogee without a barometer: it looks for the integrated velocity to cross below zero. Its
altitude will drift over time, so the `MAIN` decision is less reliable. `B2_kalman_fsm`, `B3_adaptive_kf_fsm` and
`B4_aero_ekf_fsm` use the same acceleration thresholds for powered-flight transitions, but use their respective fused
state estimator's velocity/altitude for `APOGEE` and `MAIN`.

**Important BECO distinction:** the simulator already has a separate `BURNOUT_01` event, then schedules `BECO` at the
stage-separation instant (`burnout + sep_delay`). In other words, although BECO conventionally means booster engine
cut-off, this benchmark's `BECO` label currently means **separation**. The rule detector does not measure a speed
change or verify that the booster physically detached; it looks for the acceleration jump caused by the simulated
separation shock. The simulator also scripts a gyroscope tip-off at that instant. On a real rocket, a shock can occur
without successful separation, and a quiet separation could lack this cue; an independent separation switch or other
hardware confirmation would be needed to establish physical detachment.

The learned detectors use different criteria: each `tree_*` model predicts whether one event has just happened from
its selected causal feature group (`tree_imu_only` includes accelerometer and gyroscope; `tree_fusion` adds barometer
and estimator features). Training labels mark the 0.5 s after a simulated event as positive; the trees learn feature
splits rather than using the hand-written thresholds above. `mlp_fusion` and `knn_fusion_anomaly` instead predict the
current mission state from IMU, barometer and estimator features; KNN suppresses predictions that are too far from
training examples. For these models, an event is implied when the decoded state advances to its successor, not when
an independent sensor directly reports “separated”.

All candidates are then filtered by the mission graph and persistence rule: normally a cue must remain positive for
`persist=3` consecutive samples (60 ms at 50 Hz); the abrupt `BECO` shock uses one sample (20 ms). Only legal next
transitions are considered, and the earliest eligible candidate advances the state. This enforces event order; it
does not independently prove that the physical event occurred.

The archived held-out results below do not show a general advantage from adding the estimator features: `tree_fusion` and
`tree_imu_baro` both achieve 0.56 fully correct sequences, with event success of 0.88 and 0.89 respectively. Both
struggle on the unseen `ign2_failure` scenario (0.00 fully correct sequences), while the hand-written Kalman/FSM
baselines score 1.00 in this simulator. These results are evidence about this simulation and split, not real-flight
performance. These checked-in tables predate `mlp_fusion` and `knn_fusion_anomaly`; fresh experiment runs include their
scores in the same CSV metrics and the stage summary figure.

### Combining fusion with a dense neural network and KNN anomaly detection

Both methods are now implemented as causal state detectors over IMU, barometer and estimator features:

* `mlp_fusion` uses a dense MLP with two hidden layers (48 and 24 units) to predict the current mission state. Its
  standardized input is balanced and capped per state using training flights only; predicted states are decoded into
  events by the existing persistence and FSM logic.
* `knn_fusion_anomaly` predicts the state from nearby training windows and rejects predictions whose nearest same-state
  distance exceeds a threshold calibrated from training examples. Rejected windows cannot trigger an event. This is a
  state classifier with an out-of-distribution rejection rule, not an unsupervised detector that names arbitrary faults.
* Both methods use labels from this rocket's mission graph, not the transportation classes in the example material.
  Feature scaling and distance thresholds are fit on training flights only. They are compared on the held-out scenarios
  using event success, detection delay, false/spurious transitions and fully correct sequences.

The new `fig_9_stage_detection_summary.png` averages on-time detection by event across all applicable test flights.
Its left panel groups test scenarios without vehicle failure (sensor-degradation cases may be included); its right panel
groups vehicle-failure scenarios. The default held-out test set has no nominal-only scenario, so the left panel is not a
pure nominal baseline. Events that do not occur are excluded. The figure compares transition detection, not how long a
model occupies each state. Main comparison figures show the five core detectors; full results for all detectors remain in
the CSV outputs. The historical event tables above predate the MLP/KNN methods.

### Compact held-out comparison

Reference run: Minerva preset, 120 training flights, 40 validation flights and 30 flights in each of five unseen test
scenarios (150 test flights total, seed 2024). The compact figures show:

| detector | on-time events | fully correct sequences | inference median / p95 [ms per flight] |
|---|---:|---:|---:|
| `B2_kalman_fsm` | 1.000 | 1.000 | 1.419 / 4.053 |
| `tree_imu_only` | 0.905 | 0.713 | 14.800 / 19.433 |
| `tree_fusion` | 0.882 | 0.560 | 19.686 / 28.125 |
| `mlp_fusion` | 0.673 | 0.167 | 19.361 / 26.962 |
| `knn_fusion_anomaly` | 0.488 | 0.000 | 451.127 / 639.514 |

On this simulator, the hand-written Kalman/FSM baseline is both the most accurate and among the fastest. The learned
models generalized poorly to these unseen degradations/failures; KNN was especially expensive because it compares each
feature window with its training set. Inference times are host-specific (Windows 10, Intel64 Family 6 Model 142,
Python 3.13.14), cover the complete `detect()` call for the processed flight, and exclude sensor simulation, feature
generation and estimator computation. They do not establish onboard real-time performance. Electrical energy was not
measured; power telemetry on the target flight computer is required to report joules or energy per inference.

The selected detector comparison figures contain the three Kalman-based detectors, `tree_fusion`,
`knn_fusion_anomaly`, and the ExtraTrees / boosted-tree candidates when `--compare-tree-ensembles` is enabled.
Full detector timing and detection records are in `inference_latency_per_flight.csv`,
`inference_latency_summary.csv` and `raw_detections.csv`.

**Estimation** (RMSE; validation scenarios = nominal, high_noise, baro_transonic_severe, low_resolution; test = imu_saturation, baro_dropout, low_cost_sensors, combined; vehicle failures excluded):

| estimator | transonic, velocity [m/s] (val / test) | transonic, altitude [m] (val / test) | whole ascent, velocity [m/s] (val / test) | whole ascent, altitude [m] (val / test) |
|---|---|---|---|---|
| baro only (raw) | 96.1 / 280.1 | 107 / 153 | 42.7 / 119.5 | 59 / 143 |
| IMU dead reckoning | 4.5 / 88.6 | 61 / 1178 | 3.8 / 95.4 | 83 / 2232 |
| Kalman (fixed R) | 26.4 / 54.1 | 102 / 195 | 16.2 / 51.5 | 61 / 154 |
| adaptive Kalman | 7.5 / 47.2 | 68 / 339 | 4.2 / 37.6 | 42 / 247 |
| adaptive Kalman + aero | 7.7 / 44.4 | 70 / 311 | 4.2 / 35.2 | 43 / 227 |

Transonic-window velocity RMSE [m/s] on each held-out scenario:

| estimator | baro_dropout | combined | imu_saturation | low_cost_sensors |
|---|---|---|---|---|
| baro only (raw) | 850 | 129 | 70 | 72 |
| IMU dead reckoning | 4 | 169 | 164 | 17 |
| Kalman (fixed R) | 116 | 47 | 32 | 21 |
| adaptive Kalman | 73 | 50 | 48 | 19 |
| adaptive Kalman + aero | 63 | 50 | 47 | 18 |

How to read this (please do not over-interpret):

* **Adaptive barometer trust works when the accelerometer is healthy.** On the validation scenarios it cuts the transonic-window velocity error of the fixed-R Kalman filter from 26.4 to 7.5 m/s. On held-out scenarios that corrupt the accelerometer (`imu_saturation`, `combined`: low-g range 6 g and no high-g fallback) it can be **worse**, because it trusts the IMU more; fusion would also have to detect a bad IMU.
* **The aerodynamic pseudo-measurement adds almost nothing here** (7.5 vs 7.7 m/s on validation): once the barometer is down-weighted the accelerometer integration is already good. It might matter with worse IMU drift or longer coast phases.
* **In clean scenarios, plain IMU dead reckoning has a lower velocity error than every fused estimate** (4.5 m/s) because the simulated IMU is good (pad-calibrated bias, short flight). Fusion pays off when the IMU degrades or for the altitude, not for the velocity of a clean IMU. Real IMU drift is probably worse than simulated.
* **Gyroscope helps the learned detector on unseen scenarios:** fully correct sequences 0.20 -> 0.71 when the gyroscope features are added to the accelerometer ones (`tree_accel_only` -> `tree_imu_only`), mostly through the separation tip-off.
* **The trees still do not generalise to unseen scenarios as well as the rule baselines** (`ign2_failure` was never seen in training and every tree fails it), while the Kalman-based rule detectors (B2-B4) get a fully correct sequence in every scenario. Their thresholds are set by hand for a clean, simplified simulator, so this says little about real flights; the benchmark does **not** show an advantage of the learned fusion.
* B3 and B4 equal B2 in event detection: the estimation improvements do not change *when* apogee is detected here (all use the same velocity sign test).

## Install

```bash
python -m venv .venv && source .venv/bin/activate
pip install -r requirements-dev.txt      # or: pip install -e .
pytest -q
```

## Run

```bash
# Minerva vehicle, train / validation / held-out test (recommended)
python -m rocketfusion.experiment --preset minerva --protocol held_out --jobs 4 --out results/minerva_held_out

# same vehicle, trained on nominal flights only
python -m rocketfusion.experiment --preset minerva --protocol nominal_only --n-train 80 --out results/minerva_nominal_only

# generic 1-stage rocket
python -m rocketfusion.experiment --stages 1 --protocol held_out --out results/one_stage

# custom rocket / sensors (JSON overrides, see configs/)
python -m rocketfusion.experiment --rocket-config configs/example_rocket.json \
  --sensor-config configs/example_sensor.json --out results/custom
```

Everything is seeded (`--seed`): flight *i* derives rocket parameters, sensor parameters and noise from
independent `numpy.random.SeedSequence` streams, so results are reproducible and independent of `--jobs`.
Training, validation and test seeds are disjoint (asserted). Each run writes its full configuration and library versions to `run_config.json`.

### Outputs (per run)

`raw_detections.csv` (one row per flight × detector × event, with its split), `per_event_summary.csv`, `overall_summary.csv`,
`split_summary.csv`, `depth_selection_validation.csv`, `estimation_error_summary.csv`, `FIGURES.md`, `flight_summary.csv` (apogee, Mach, peak g, wind, lateral drift), `mission_check.txt`,
`tree_fusion_rules.txt` (human-readable trees), `fig_4_detection_success_heatmap.png`,
`fig_5_detection_delay.png`, `fig_6_overall_success.png`, `fig_7_sequence_correct.png`,
`fig_9_stage_detection_summary.png`,
`fig_11_inference_latency.png`, `fig_12_tree_fusion_diagram.png`, `fig_13_experiment_methodology.png`,
`fig_14_flight_environment_table.png`, `inference_latency_per_flight.csv`,
`inference_latency_summary.csv`, `fig_sequence_correct.png`, `fig_example_<scenario>_<split>.png`, `run_config.json`.

### Crosswind and tree-ensemble comparison

The simulator can apply a signed crosswind at a reference height, a power-law shear profile limited to the
100 m boundary layer, a linear transition to the aloft wind at 1,000 m, and a sinusoidal gust with a reproducible
per-flight phase. Above the aloft height the configured aloft wind is held constant, and wind magnitude is capped
at 40 m/s. Air-relative velocity drives Mach and drag; the rocket remains a planar point mass with vertical thrust
and no attitude/lift response. These are configurable synthetic profiles, not measured launch-site weather. The
default experiment uses 0 m/s, preserving the no-wind baseline. A sensitivity sweep trains and tests a separate
model set at each wind level:

```powershell
python -m rocketfusion.wind_sweep --speeds 0 5 10 15 --n-train 12 --n-val 6 --n-test 2 `
  --out results/wind_tree_comparison
```

The sweep compares the three Kalman-FSMs, the current event trees, ExtraTrees, gradient-boosted event trees and
KNN on the held-out sensor/vehicle-failure scenarios. Each wind level is represented in its own train and test
run; this is a wind-sensitivity comparison, **not** a claim of generalization to unseen wind profiles. The small
counts above are a smoke/demo run; increase them for research conclusions. `wind_comparison.csv` and
`fig_wind_detector_comparison.png` summarize sequence correctness, on-time detection and Python-host latency.
The rerun also creates scenario-level tables (`wind_scenario_accuracy.csv`, `wind_event_accuracy.csv`),
one event-accuracy heatmap per wind level, `fig_wind_sequence_heatmap.png`, a compact sequence score table,
and `fig_wind_detection_delay_boxplot.png`, which separates on-time detection delays by event and method.
`wind_detection_delay_summary.csv` and `wind_detection_delay_by_event.csv` report misses and detections outside
the ±1 s tolerance separately, plus median and p95 absolute delay among all detected events.
`wind_experiment_tables.md` collects the tabular results, and
[`presentation_script_en.md`](./results/wind_tree_comparison/presentation_script_en.md) is a timed English
talk script intended to fit within ten minutes. To regenerate these presentation outputs without rerunning
training, use `python -m rocketfusion.wind_sweep --speeds 0 5 10 15 --reuse-runs --out results/wind_tree_comparison`.
In the current 12/6/2-per-wind preview, ExtraTrees matched the Kalman-FSM sequence score at 1.00 for 0, 5 and
10 m/s, then fell to 0.67 at 15 m/s; the original single-tree fusion scored 0.42-0.67. The Kalman-FSM remained
at 1.00 in this small run. ExtraTrees host inference was about 53-60 ms per full processed flight versus
11-13 ms for the single tree; these are Python-host measurements, not MCU timings. The nominal 15 m/s case
reaches 9,985.6 m apogee and about 3,034 m crosswind displacement in this simplified model. See
[`fig_wind_detector_comparison.png`](./results/wind_tree_comparison/fig_wind_detector_comparison.png),
[`fig_wind_trajectory.png`](./results/wind_tree_comparison/fig_wind_trajectory.png), and
[`wind_trajectory_summary.csv`](./results/wind_tree_comparison/wind_trajectory_summary.csv). These are
exploratory outputs with only 2 test flights per scenario and wind level.

### STM32H562RG physical HIL

The requested target is STM32H562RG, but this workspace currently has no detected serial port, ARM cross-compiler,
STM32 programmer, or PySerial installation, so no physical HIL result has been produced. In a first classifier-only
HIL, the host would stream the 50 Hz fused feature vectors to the board and compare event-tree outputs. That would
not yet be a complete onboard implementation: the current features include Kalman estimates, so sensor processing,
estimation and temporal decoding would also need MCU firmware before claiming end-to-end onboard validation.
Python-host inference latency is not an STM32 timing measurement.

### Metrics

An event that occurred is **ok** if detected within ±`tol_s` (default 1 s) of the true time, **false_early** if detected earlier
than that, **late** if later, **missed** if never detected. An event that did *not* occur (vehicle failure) is **spurious** if reported.
A flight is *sequence_correct* when every occurred event is ok and nothing spurious was reported. Reported: success rate, false transitions per flight,
median latency (ok events), per event and per scenario. Detectors walk the mission graph, so a missed event can stall
the later ones (visible in the per-event heat map).

Two latency measurements are distinct. Event latency is detection time minus true event time (`latency` in the detection
metrics); inference time is the wall-clock duration of one detector `detect()` call over the complete processed flight.
The latter is measured after feature generation and state estimation, reported per flight and summarized by median/p95;
`ms_per_sample` is provided as a throughput normalization, not a streaming deadline guarantee. It depends on the host
CPU and software stack. Electrical energy is not measured: reliable joules-per-inference requires power telemetry on the
target hardware, and cannot be inferred from wall-clock time alone.

### Parameters you can change

* `RocketConfig` (`config.py`): number of stages, masses, thrust, burn times, drag, areas, staging delays, recovery.
* `SensorConfig`: rates, ranges, noise, bias, vibration, shock, transonic disturbance, dropouts.
* `ProcConfig`: processing rate, calibration time, feature window, persistence, tolerance.
* Scenarios (`apply_scenario`, `apply_failure`): `nominal`, `high_noise`, `imu_saturation`, `baro_transonic_severe`, `baro_dropout`,
  `low_cost_sensors` (poor linearity, hysteresis, sensitivity error and threshold), `low_resolution` (10-bit IMU, 12 Pa barometer), `combined`,
  and the vehicle failures `sep_failure`, `ign2_failure`, `sep_baro_dropout` (separation + barometer dropout), and
  `ign2_imu_saturation` (stage-2 ignition failure + IMU saturation; 2-stage only).

## Generic vehicle

Without `--preset`, a small generic 1- or 2-stage rocket is used (see `config.py`); results for it are not
shipped, regenerate them with `reproduce.sh`.

## Modelling assumptions and limitations

* **Attitude is scripted**, not simulated (the gyroscope sees rolls, tip-off and pitch-over that do not affect the trajectory).
* **Planar point-mass flight** when wind is enabled (the booster after separation is not simulated). The crosswind changes
  air-relative drag and horizontal drift, but thrust stays vertical; pitch/yaw, lift, structural bending and attitude
  feedback are not modeled. This is not a full 3/6-DoF or launch-site meteorology simulation.
* **BECO is mapped to the stage-separation instant.** Rename or remap in `config.EVENTS` if your definition differs.
* Linearity is a cubic bow, hysteresis a play operator and the threshold a dead-band: simple, standard
  static models; temperature drift, cross-axis sensitivity and frequency response of the accelerometer are not modelled.
* The separation shock, transonic barometer disturbance, noise levels and motor profiles are **plausible
  parameterisations, not calibrated to a specific sensor or motor**. Calibrate them with bench/flight data
  before claiming realism.
* Simulation-only: models trained here may not transfer to real flights (sim-to-real gap).
* The rule baselines use thresholds set by hand once (`RuleConfig`); they are not tuned per scenario.

## Layout

```
calibrate_minerva.py   fits the unknown masses/Cd to the CONOPS points / a target apogee
rocketfusion/   splits (train/val/test protocols), estimators, estimation, atmosphere, config, dynamics, sensors (transducer chain), characterize, preprocess,
                kalman, features, detectors, metrics, pipeline, experiment, plots
tests/          unit/smoke tests
configs/        example JSON overrides
results/              curated figures and tables for the sensor, tree-fusion and wind experiments
reference_results/    local-only archived outputs (ignored by Git)
```

## Citation / License

Add your paper citation and choose a license (e.g. MIT or Apache-2.0) before publishing.

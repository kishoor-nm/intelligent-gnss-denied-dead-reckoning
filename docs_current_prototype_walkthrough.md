# SIH 2026 PS-168: Intelligent Dead Reckoning (IDR) Prototype
## Comprehensive Technical Walkthrough & Software-in-the-Loop Architecture Report

---

## SECTION 1 — WHAT IS THIS PROTOTYPE?

### Problem Overview
Modern vehicle navigation systems rely heavily on Global Navigation Satellite Systems (GNSS), such as GPS, GLONASS, or NavIC. However, GNSS signals are vulnerable to interference, multipath drift, physical blockages (tunnels, urban canyons, dense forests, parking garages), and signal spoofing/jamming. When GNSS signals drop, conventional navigation displays freeze or jump erratically.

**SIH 2026 Problem Statement 168 (PS-168)** challenges us to develop an **Intelligent Dead Reckoning (IDR)** system capable of continuously estimating vehicle position during extended GNSS outages without relying on external location updates.

### What is GNSS-Denied Navigation & Dead Reckoning?
- **GNSS-Denied Navigation**: Navigating in an environment where direct absolute location measurements (latitude, longitude, altitude) are completely unavailable.
- **Dead Reckoning (DR)**: The mathematical process of calculating one's current position by advancing a previously known position ("anchor") using estimated speeds, accelerations, and headings over elapsed time.

### Prototype Capabilities
Our prototype fuses multi-rate sensor streams from two distinct sources:
1. **Vehicle CAN Bus ECU**: Wheel/ECU indicated speed, longitudinal acceleration, lateral acceleration, and vehicle yaw rate.
2. **Smartphone IMU**: Gyroscope roll rate ($\omega_{\text{roll}}$) captured by a smartphone mounted inside the vehicle cabin.

The system uses an **Extended Kalman Filter (EKF)** to estimate position drift, vehicle velocity, heading, roll angle, and gyroscope bias in real time.

### Prototype Operational Mode: Software-in-the-Loop (SIL) Dataset Replay
> [!IMPORTANT]
> **This prototype is currently operating in Software-in-the-Loop (SIL) Real-Time Dataset Replay Mode.**
> It is **NOT** connected to live vehicle CAN hardware or live Bluetooth IMU streaming in physical real-time. Instead, previously recorded, synchronized multi-sensor datasets are replayed sample-by-sample at a nominal 10 Hz rate (100 ms/sample) to simulate a live physical sensor feed.

---

## SECTION 2 — CURRENT SYSTEM FLOW

### End-to-End System Pipeline Diagram

```
+------------------------------------+        +------------------------------------+
|  Vehicle CAN Bus CSV Dataset       |        |  Smartphone IMU CSV Dataset        |
|  (Speed, Long/Lat Accel, Yaw Rate) |        |  (Gyroscope Roll Rate)             |
+------------------------------------+        +------------------------------------+
                  |                                             |
                  +---------------------+-----------------------+
                                        |
                                        v
                    +---------------------------------------+
                    |  Schema & Unit Validation Layer       |
                    |  (schema_validation.py)               |
                    +---------------------------------------+
                                        |
                                        v
                    +---------------------------------------+
                    |  CSV Replay Streamer (10 Hz Pacing)   |
                    |  (csv_replay_streamer.py)             |
                    +---------------------------------------+
                                        | Yields SingleSensorSample
                                        v
                    +---------------------------------------+
                    |  GNSS Outage Boundary & Provenance    |
                    |  (Position Anchor: Lat/Lon -> ENU)    |
                    +---------------------------------------+
                                        |
                                        v
                    +---------------------------------------+
                    |  GNSS Attribute Masking Layer         |
                    |  (Zero Leakage: Reference ONLY)       |
                    +---------------------------------------+
                                        |
                                        v
                    +---------------------------------------+
                    |  M5.1 Baseline Estimator (5D EKF)     |
                    |  State: [East, North, V, psi, b_z]    |
                    +---------------------------------------+
                                        |
                                        v
                    +---------------------------------------+
                    |  M9.3 Adaptive Switching Logic        |
                    |  Condition: t >= 20s AND             |
                    |  (sigma_psi >= 8.0 deg OR cum_yaw)    |
                    +---------------------------------------+
                                        | (Continuous Handoff)
                                        v
                    +---------------------------------------+
                    |  M9.1 Roll-Aware Estimator (6D EKF)   |
                    |  State: [East, North, V, psi, phi, bz]|
                    |  Measurement: ECU Speed + NHC Roll    |
                    +---------------------------------------+
                                        |
                                        v
                    +---------------------------------------+
                    |  Estimated State Vector (x_k)         |
                    |  (East, North, Speed, Heading, Roll)  |
                    +---------------------------------------+
                                        |
                                        v
                    +---------------------------------------+
                    |  Live Terminal & Matplotlib Visualizer|
                    |  (cli_realtime_replay.py)             |
                    +---------------------------------------+
                                        |
                                        v
                    +---------------------------------------+
                    |  Post-Inference RMSE Evaluation       |
                    |  (final_navigation.py)                |
                    +---------------------------------------+
```

### Component Breakdown

| Component | Input | Processing | Output | Source File |
| :--- | :--- | :--- | :--- | :--- |
| **Schema Validation** | Raw DataFrames | Validates column names, units, non-empty bounds | Boolean flag, validated DataFrame | [`schema_validation.py`](file:///d:/prototype/src/iovnbd/preprocessing/schema_validation.py) |
| **Replay Streamer** | Multi-Sensor DataFrames | Synchronizes streams; yields 1 sample per 100ms | `SingleSensorSample` | [`csv_replay_streamer.py`](file:///d:/prototype/src/iovnbd/navigation/csv_replay_streamer.py) |
| **State Anchor** | Outage Start Row Index | Converts initial Lat/Lon to local ENU coordinate origin | `InitialState` | [`initialization.py`](file:///d:/prototype/src/iovnbd/navigation/initialization.py) |
| **GNSS Masking** | `SingleSensorSample` | Marks GNSS lat/lon as evaluation-only; excludes from EKF | Masked sample stream | [`streaming_runner.py`](file:///d:/prototype/src/iovnbd/navigation/streaming_runner.py) |
| **M5.1 EKF** | ECU Speed, Long Accel, Yaw Rate | Propagates 5D state $\mathbf{x}_5 = [E, N, V, \psi, b_z]^T$ | Updated 5D state & covariance $P_5$ | [`ekf_m5_1.py`](file:///d:/prototype/src/iovnbd/navigation/ekf_m5_1.py) |
| **Adaptive Switch** | Elapsed outage $t_{\text{rel}}$, $P_5[3,3]$, cumulative yaw | Checks confidence thresholds; triggers handoff at $t \ge 20\text{s}$ | Regime switch event flag | [`fusion_m9_3.py`](file:///d:/prototype/src/iovnbd/navigation/fusion_m9_3.py) |
| **M9.1 EKF** | ECU Speed, Accel, Yaw Rate, Smartphone Roll | Propagates 6D state $\mathbf{x}_6 = [E, N, V, \psi, \phi, b_z]^T$ + Roll NHC | Updated 6D state & covariance $P_6$ | [`ekf_m9.py`](file:///d:/prototype/src/iovnbd/navigation/ekf_m9.py) |
| **Streaming Runner** | `SingleSensorSample` stream | Maintains state persistent memory across iterations | `FusedStatePointM9_3` | [`streaming_runner.py`](file:///d:/prototype/src/iovnbd/navigation/streaming_runner.py) |
| **CLI Visualizer** | `FusedStatePointM9_3` stream | Prints live console table; updates Matplotlib plot | PNG plots, metrics dict | [`cli_realtime_replay.py`](file:///d:/prototype/src/iovnbd/cli_realtime_replay.py) |

---

## SECTION 3 — INPUT DATA & SENSOR FIELDS

### Active Causal Sensors Used for Navigation Inference

1. **Vehicle CAN Bus ECU Sensors**:
   - `indicated_speed_m_s` (or `Indicated Vehicle Speed (km/hr)`): Vehicle ECU wheel speed ($\text{m/s}$). Used in EKF measurement update.
   - `longitudinal_accel_m_s2` (or `Indicated Longitudinal Acceleration (g)`): Vehicle acceleration along body X-axis ($\text{m/s}^2$). Used in state prediction.
   - `lateral_accel_m_s2` (or `Indicated Lateral Acceleration (g)`): Vehicle cornering acceleration along body Y-axis ($\text{m/s}^2$). Used in roll-aware Non-Holonomic Constraint (NHC) updates.
   - `yaw_rate_rad_s` (or `Yaw Rate (deg/sec)`): Gyroscope turning rate about vertical Z-axis ($\text{rad/s}$). Used in heading integration & bias tracking.
   - `t_rel_sec`: Relative timestamp ($\text{s}$).

2. **Smartphone IMU Sensors**:
   - `roll_rate_rad_s` (or `GYROSCOPE Roll (rad/s)`): Smartphone angular velocity about longitudinal X-axis ($\text{rad/s}$). Used in M9.1 roll integration.

3. **GNSS Reference Fields (STRICTLY EVALUATION-ONLY)**:
   - `Latitude (degrees)` & `Longitude (degrees)`: High-precision VBOX GNSS position reference.
   - **Zero Leakage Rule**: During outage inference ($t > t_0$), these fields are **100% ignored** by the EKF state update functions. They are used exclusively post-hoc to calculate ground-truth position drift error (RMSE).

---

## SECTION 4 — WHAT HAPPENS WHEN GNSS IS LOST?

### Outage Sequence Timeline

```
t < 100.0s (Pre-Outage)           t = 100.0s (Outage Onset)              t > 100.0s (Dead Reckoning)
GNSS Available                    GNSS Lost                              GNSS Masked
+-----------------------+         +-----------------------+              +-----------------------+
| Absolute Lat/Lon      | ------> | Save Initial Anchor   | -----------> | EKF State Propagation |
| Fix updates position  |         | (Lat 52.403°, Lon -1.507°)           | using CAN + IMU only |
+-----------------------+         +-----------------------+              +-----------------------+
```

1. **Pre-Outage Calibration ($t < t_0$)**: The GNSS receiver maintains absolute position fixes.
2. **Outage Onset ($t = t_0 = 100.0\text{s}$)**: GNSS signal is intentionally masked. The system captures the last valid GNSS coordinate as the **ENU Local Origin Anchor** ($E_0 = 0.0\text{m}, N_0 = 0.0\text{m}$).
3. **GNSS Masking Verification**: The `StreamingNavigationRunner` blocks all latitude and longitude access. Overwriting reference GNSS columns with zeros produces 100% identical estimated position trajectories, proving zero leakage.
4. **Dead Reckoning Execution**: EKF propagates position using wheel speed and yaw rate, adjusting for gyro bias $b_z$ and roll tilt $\phi$.

---

## SECTION 5 — EXPLAIN M5.1 LIKE I'M A BEGINNER

### What is M5.1?
**M5.1** is our primary 5-Dimensional Extended Kalman Filter. It treats the vehicle as moving on a flat 2D plane.

### Sensor Inputs & Estimated State
- **Inputs**: ECU Wheel Speed, Longitudinal Accel, Yaw Rate.
- **Estimated State Vector $\mathbf{x}_5$**:
  1. $E$: East position offset ($\text{m}$)
  2. $N$: North position offset ($\text{m}$)
  3. $V$: Forward speed ($\text{m/s}$)
  4. $\psi$: Heading yaw angle ($\text{rad}$)
  5. $b_z$: Gyroscope Z-axis bias ($\text{rad/s}$)

### Why M5.1 is Great for Short Outages ($T \le 30\text{s}$)
For short time windows, vehicle roll (chassis lean) is minimal. M5.1 does not try to estimate roll, making it mathematically stable, highly robust, and computationally fast.

---

## SECTION 6 — EXPLAIN M9.1 LIKE I'M A BEGINNER

### What is M9.1?
**M9.1** is a 6-Dimensional EKF that explicitly models vehicle chassis roll tilt angle ($\phi$).

### Why Roll Matters During Cornering
When a vehicle turns at high speed, centrifugal force causes the body to lean (roll). This roll tilts the accelerometer, causing gravity ($g = 9.81\text{ m/s}^2$) to leak into lateral acceleration measurements:
$$\text{Measured Lateral Acceleration} = V \cdot \omega_z + g \cdot \sin(\phi)$$

If roll is ignored, the EKF mistakes gravity tilt for vehicle turning, causing severe position drift over long outages ($T > 60\text{s}$).

### Estimated State & Speed-Adaptive Restoring Torque
- **Estimated State Vector $\mathbf{x}_6$**: $[E, N, V, \psi, \phi, b_z]^T$
- **Speed-Adaptive Damping ($K_{\text{roll}}$)**: M9.1 applies a restoring force to prevent roll drift during straight driving:
  $$K_{\text{roll}}(V) = K_{\text{base}} \cdot \left(1 - e^{-V / V_0}\right) \quad (K_{\text{base}} = 0.02, V_0 = 10.0\text{ m/s})$$

---

## SECTION 7 — EXPLAIN M9.3 ADAPTIVE FUSION

### Architecture & Switching Mechanism
M9.3 combines M5.1 and M9.1 into a unified dual-regime estimator.

```
       [ Outage Start t = 0s ]
                  |
                  v
       +---------------------+
       |  Regime 1: M5.1 EKF |  <-- High stability for initial 20s
       +---------------------+
                  |
                  | Check Condition at t >= 20s:
                  |   • Heading Std Dev (sigma_psi) >= 8.0° OR
                  |   • Cumulative Yaw Turn >= 0.5 rad OR
                  |   • Elapsed Outage Time >= 40.0s
                  v
       +---------------------+
       |  Regime 2: M9.1 EKF |  <-- Activated for extended cornering / long duration
       +---------------------+
```

### Continuous State Handoff
When switching from M5.1 to M9.1, position $(E, N)$, speed $V$, heading $\psi$, and bias $b_z$ are transferred seamlessly while initializing roll $\phi = 0.0$. This guarantees **zero position jump** ($< 10^{-4}\text{ m}$ continuity error).

---

## SECTION 8 — WHAT IS HAPPENING IN THE TERMINAL?

When running the 30-second replay command:
```powershell
C:\Users\kisho\AppData\Local\Programs\Python\Python312\python.exe -m src.iovnbd.cli_realtime_replay --vehicle_csv "d:/prototype/data/processed/S1/V-S1_processed.csv" --smartphone_csv "d:/prototype/data/processed/S1/S-S1_processed.csv" --duration 30 --replay_speed 1.0
```

### Terminal Output Explanation

1. `MODE: DATASET REPLAY — NOT LIVE PHYSICAL HARDWARE`: Confirms Software-in-the-Loop simulation mode.
2. `SENSOR SAMPLING RATE: 10 Hz Nominal`: Sensors deliver 1 payload every 100 milliseconds.
3. `Outage Start Timestamp: t = 100.0s`: GNSS masking begins at dataset time 100.0s (Row 1000).
4. `Initial ENU Position: East=0.0m, North=0.0m`: Sets local origin at outage start.
5. `t = 0.9s | Active Estimator: [M5.1] | Speed: 0.62 m/s | Heading: 302.6° | Roll: 0.0° | Pos Error: 0.09m`: Initial 1-second update showing baseline M5.1 tracking.
6. `t = 20.0s | Active Estimator: [M9.1] ... *** ADAPTIVE SWITCH TO M9.1 ***`: Adaptive condition triggers transition from M5.1 to M9.1 at $t = 20.0\text{s}$.
7. `Streaming Fused Prototype RMSE: 21.60 meters`: Root Mean Square Error over the entire 30-second outage.
8. `Streaming Final Position Error: 43.83 meters`: Absolute position error at $t = +30.0\text{s}$.
9. `Total Wall Clock Runtime: 30.35 seconds`: Replay pacing matches actual physical time elapsed.

---

## SECTION 9 — WHAT THE USER ACTUALLY SEES

During execution, the user sees:
1. **Interactive Terminal Output**: Live updating progress log displaying current timestamp, active regime, speed, heading, roll, and position error.
2. **Matplotlib Live Trajectory Plot**: A two-panel GUI window:
   - **Left Panel (Trajectory Map)**: Shows ground-truth GNSS reference path (black dashed), outage start point (blue dot), and streaming estimated trajectory (green line).
   - **Right Panel (Drift Growth)**: Real-time plot of position error growth over elapsed outage seconds.
3. **Artifact Output**: High-resolution plot saved to `d:/prototype/results/realtime_replay/realtime_replay_trajectory.png`.

---

## SECTION 10 — 30-SECOND DEMO WALKTHROUGH

### Live Demonstration Script (Step-by-Step)

| Step | Action | Command / Say |
| :--- | :--- | :--- |
| **1** | Open Terminal | Open PowerShell in `d:/prototype` |
| **2** | Run Replay | `python -m src.iovnbd.cli_realtime_replay --duration 30 --replay_speed 1.0` |
| **3** | Explain Mode | *"Notice that this is Software-in-the-Loop dataset replay at 10 Hz real-time pacing."* |
| **4** | Explain Masking | *"At t = 100s, GNSS position is masked. The system relies strictly on vehicle speed and smartphone IMU."* |
| **5** | Show M5.1 | *"For the first 20 seconds, the M5.1 5D EKF handles flat driving efficiently."* |
| **6** | Show Switch | *"At t = 20s, heading uncertainty triggers an adaptive switch to M9.1 roll compensation."* |
| **7** | Show Results | *"After 30 seconds of total GNSS outage, position drift RMSE is just 21.60 meters."* |

---

## SECTION 11 — 120-SECOND DEMO

### Extended Outage Command
```powershell
C:\Users\kisho\AppData\Local\Programs\Python\Python312\python.exe -m src.iovnbd.cli_realtime_replay --duration 120 --replay_speed 2.0
```

### Why 120-Second Outage Matters
Over 120 seconds, uncompensated roll drift causes flat EKFs (M5.1) to accumulate 148.57 m RMSE. M9.3 adaptive fusion maintains roll stability, achieving **88.76 m RMSE** (a **40.3% error reduction** over baseline).

---

## SECTION 12 — CURRENT VERIFIED PERFORMANCE RESULTS

*Data evaluated on S1 Canonical Outage Benchmark ($t_0 = 100.0\text{s}$):*

| Outage Duration | M5.1 Baseline RMSE | M9.1 Pure Roll RMSE | M9.3 Fused Prototype RMSE | M9.3 Accuracy Gain |
| :---: | :---: | :---: | :---: | :---: |
| **10 seconds** | 3.19 m | 3.25 m | **3.16 m** | **+0.9%** |
| **30 seconds** | 22.48 m | 22.28 m | **22.28 m** | **+0.9%** |
| **60 seconds** | 86.25 m | 78.40 m | **73.20 m** | **+15.1%** |
| **120 seconds** | 148.57 m | 133.25 m | **88.76 m** | **+40.3%** |

---

## SECTION 13 — WHAT IS ACTUALLY "REAL-TIME"?

### Real-Time Dataset Replay (Current Status)
- **Data Flow**: Pre-recorded CSV rows loaded into RAM $\rightarrow$ `CSVReplayStreamer` yields 1 sample payload every 100 ms $\rightarrow$ EKF state updated sample-by-sample.
- **Why it's SIL Real-Time**: The navigation core processes data incrementally without forward dataset access, exactly as it would in a vehicle ECU.

### What is NOT Implemented (Hardware Limitations)
- Live OBD-II / CAN hardware adapter.
- Live Bluetooth / USB smartphone IMU receiver.
- Hardware clock synchronization / CAN bus packet drop handling.

---

## SECTION 14 — FUTURE HARDWARE ARCHITECTURE

```
[ Vehicle OBD-II / CAN Adapter ]     [ Smartphone Sensor Daemon ]
                |                                 |
                +----------------+----------------+
                                 | (Socket / Serial Stream)
                                 v
                +---------------------------------+
                | Live Sensor Ingestion Interface |
                +---------------------------------+
                                 | (SingleSensorSample @ 10Hz)
                                 v
                +---------------------------------+
                | Production Navigation Core      |
                | (StreamingNavigationRunner)     |
                +---------------------------------+
                                 |
                                 v
                +---------------------------------+
                | Live Vehicle Head-Unit Display  |
                +---------------------------------+
```

---

## SECTION 15 — FILE-BY-FILE MAP

| File Path | Purpose | Key Inputs | Key Outputs | Used By |
| :--- | :--- | :--- | :--- | :--- |
| [`final_navigation.py`](file:///d:/prototype/src/iovnbd/navigation/final_navigation.py) | Main competition wrapper & benchmark evaluator | Vehicle & Smartphone CSVs | `FusedResultM9_3`, `NavigationPerformanceMetrics` | CLI, Test Suite |
| [`fusion_m9_3.py`](file:///d:/prototype/src/iovnbd/navigation/fusion_m9_3.py) | Dual-regime EKF switching core | Multi-sensor inputs | `FusedResultM9_3` | Batch runner |
| [`streaming_runner.py`](file:///d:/prototype/src/iovnbd/navigation/streaming_runner.py) | Sample-by-sample incremental EKF state manager | `SingleSensorSample` | `FusedStatePointM9_3` | Replay CLI |
| [`csv_replay_streamer.py`](file:///d:/prototype/src/iovnbd/navigation/csv_replay_streamer.py) | 10Hz dataset streamer & synchronizer | Processed CSVs | `SingleSensorSample` generator | Replay CLI |
| [`schema_validation.py`](file:///d:/prototype/src/iovnbd/preprocessing/schema_validation.py) | Input data contract & schema validator | DataFrames / Dicts | Validation boolean & error list | All Entry Points |
| [`cli_realtime_replay.py`](file:///d:/prototype/src/iovnbd/cli_realtime_replay.py) | Demonstration CLI & live Matplotlib visualizer | Command line args | Live console, PNG plots | Reviewers / Demo |

---

## SECTION 16 — ONE-PAGE FRIEND EXPLANATION

> *"Our SIH 2026 project is an Intelligent Dead Reckoning system for vehicles when GPS/GNSS is completely lost (like in long tunnels or urban canyons). Normally, navigation apps stop working when GPS drops. Our system takes sensor data from the vehicle's ECU (speed and turning rate) and combines it with a smartphone's IMU gyroscope (roll tilt rate). We use a two-stage Extended Kalman Filter: for short GPS outages under 20 seconds, it uses a lightweight 5D model (M5.1); for longer outages or sharp turns, it automatically switches to a 6D roll-aware model (M9.1) to correct for vehicle tilt gravity leakage. Currently, our prototype operates in Software-in-the-Loop mode—replaying recorded sensor data at 10 Hz sample-by-sample while masking GPS position. Over a 2-minute GPS outage, our adaptive fusion system reduces navigation position error from 148.57 meters down to 88.76 meters—a 40.3% improvement."*

---

## SECTION 17 — CURRENT PROTOTYPE CAPABILITY STATUS

| Capability / Interface | Current Implementation Status |
| :--- | :---: |
| **Dataset Replay Engine** | **YES (Implemented & Validated)** |
| **Sample-by-Sample Incremental Processing** | **YES (Implemented & Validated)** |
| **10 Hz Real-Time Pacing** | **YES (Implemented & Validated)** |
| **Zero GNSS Data Leakage Masking** | **YES (Verified)** |
| **M5.1 5D Flat EKF** | **YES (Implemented & Validated)** |
| **M9.1 6D Speed-Adaptive Roll EKF** | **YES (Implemented & Validated)** |
| **M9.3 Adaptive Regime Switching** | **YES (Implemented & Validated)** |
| **Trajectory Plot Generation** | **YES (Implemented & Validated)** |
| **Live Console & Visual Progress Output** | **YES (Implemented & Validated)** |
| **Live Vehicle CAN/OBD-II Hardware Connection** | **NO (Software-in-the-Loop Simulation Only)** |
| **Live Smartphone Bluetooth IMU Stream** | **NO (Software-in-the-Loop Simulation Only)** |
| **Physical Vehicle Hardware Testbed** | **NO (Software-in-the-Loop Simulation Only)** |

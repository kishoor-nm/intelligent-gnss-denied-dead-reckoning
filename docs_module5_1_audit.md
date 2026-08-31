# Module 5.1 Sensor-Provenance & GNSS-Outage Compliance Audit Documentation

## 1. Audit Objective & Findings
* **Objective**: Audit the provenance of all sensor signals consumed by Module 5 during GNSS outage windows to guarantee zero GNSS data leakage.
* **Primary Finding (`CORRECTED`)**:
  1. The original Module 5 implementation consumed `velocity_m_s` during outage.
  2. Provenance tracing revealed that `velocity_m_s` was derived from `Velocity (km/hr)` in the raw VBOX log, which represents **VBOX GNSS-Derived Velocity**.
  3. **Correction Applied**: `velocity_m_s` was **REMOVED** from the outage estimator loop and replaced with **`indicated_speed_m_s`** (derived from CAN-bus `Indicated Vehicle Speed (km/hr)` logged directly by the vehicle ECU wheel/transmission sensors).

---

## 2. Sensor Provenance & Outage Isolation Matrix

| M5 Input Signal | Raw IO-VNBD Field | Processed Field | Unit | Physical Source / Provenance | Allowed During GNSS Outage? | Audit Status |
|---|---|---|---|---|---|---|
| **Vehicle Speed** | `Indicated Vehicle Speed (km/hr)` | `indicated_speed_m_s` | $m/s$ | Vehicle ECU CAN-Bus Wheel/Transmission Speed Sensor | **YES** | **VERIFIED (NON-GNSS)** |
| **VBOX Velocity** | `Velocity (km/hr)` | `velocity_m_s` | $m/s$ | VBOX High-Precision GNSS Doppler Velocity | **NO** | **REMOVED (GNSS-DERIVED)** |
| **Longitudinal Accel** | `Indicated Longitudinal Acceleration (g)` | `longitudinal_accel_m_s2` | $m/s^2$ | Vehicle Body Acceleration Sensor | **YES** | **VERIFIED (NON-GNSS)** |
| **Yaw Rate** | `Yaw Rate (deg/sec)` | `yaw_rate_rad_s` | $rad/s$ | Vehicle CAN-Bus Gyroscope Sensor | **YES** | **VERIFIED (NON-GNSS)** |
| **Reference Position** | `Latitude (degrees)`, `Longitude (degrees)` | N/A | degrees | VBOX GNSS Reference Solution | **NO** | **ISOLATED (EVALUATION ONLY)** |

---

## 3. Strict Outage Input Interface (`ekf_m5_1.py`)
To prevent future leakage, a dedicated input structure `OutageEstimatorInputs` was created. The estimator function `propagate_ekf_m5_1` receives only explicitly extracted non-GNSS ECU attributes (`ecu_speed_m_s`, `longitudinal_accel_m_s2`, `yaw_rate_rad_s`) and has zero access to reference GNSS dataframes.

---

## 4. Measured Experimental Results (Sequence `S1`)

Evaluated using the locked M4.2 canonical evaluation protocol ($t_0 = 100.0s$, start index 1000):

| Outage Duration | Sample Count | M3 Baseline RMSE | M5.1 Corrected EKF RMSE | M3 Baseline Final Error | M5.1 Corrected EKF Final Error | M5.1 EKF Max Error |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **10 seconds** | 101 | **3.13 m** | 3.19 m | **6.58 m** | 6.85 m | 6.85 m |
| **30 seconds** | 301 | **22.35 m** | 22.48 m | **48.01 m** | 48.38 m | 48.38 m |
| **60 seconds** | 601 | **84.22 m** | 86.25 m | **129.77 m** | 134.35 m | 165.61 m |
| **120 seconds** | 1201 | **145.89 m** | 148.57 m | **245.02 m** | 249.89 m | 249.89 m |

---

## 5. Machine-Readable Results Artifacts
Saved in `d:/prototype/results/module5_1/`:
* Machine Results (JSON): [`m5_1_results.json`](file:///d:/prototype/results/module5_1/m5_1_results.json)
* Machine Results (CSV): [`m5_1_results.csv`](file:///d:/prototype/results/module5_1/m5_1_results.csv)
* Trajectory Comparison Plot: `m5_1_trajectory_comparison.png`

---

## 6. Automated Test Suite Execution
Ran full test suite (`python -m unittest discover -s d:/prototype/tests`):
* **34 / 34 Tests PASSED (`OK`)**
  * 6 Module 1 tests: `PASSED`
  * 6 Module 2 tests: `PASSED`
  * 5 Module 3 tests: `PASSED`
  * 3 Module 4 tests: `PASSED`
  * 4 Module 4.1 tests: `PASSED`
  * 3 Module 4.2 tests: `PASSED`
  * 3 Module 5 tests: `PASSED`
  * 4 Module 5.1 tests (`test_outage_estimator_inputs_isolation`, `test_ekf_m5_1_propagation`, `test_audit_suite_execution`, `test_strict_no_gnss_leakage`): `PASSED`

# Module 4.2 Results Integrity & Baseline Reconciliation Report

## 1. Purpose & Objectives
Resolve experimental discrepancies, perform a data leakage audit, qualify processing runtime claims, establish a canonical evaluation protocol, and lock the baseline results for future SIH 2026 PS-168 modules.

---

## 2. Investigation & Explanation of the 120-Second Discrepancy

### Discrepancy Identified:
* **Initial Project Benchmark**: 120-second M3 Position RMSE = **145.89 m**
* **Interim M4.1 Report Metric**: 120-second M3 Position RMSE reported as **~180.94 m**

### Root Cause Analysis (`VERIFIED`):
1. **Experiment Mislabeling**: In the interim M4.1 report, the 120-second **Exp B (4-Wheel Encoder Speed Average)** RMSE ($180.94 m$) was accidentally placed in the **Exp A (M3 Transmission Speed Baseline)** column.
2. **Canonical Baseline Verification**: Re-executing the canonical M3 baseline propagator (`propagate_dead_reckoning_baseline` with VBOX transmission speed) confirms that the true 120-second M3 Position RMSE is **145.89 m** (Final Position Error **245.02 m**).

---

## 3. Data Leakage & Sensor Input Audit (`VERIFIED / PASS`)

### Code-Path Leakage Audit:
* **`propagate_dead_reckoning_baseline` & `propagate_corrected_dead_reckoning`**: Audited input slicing and execution loops.
* **Findings**:
  1. Zero reference GNSS coordinates (`Latitude`, `Longitude`) or speeds enter the propagation state loop during outage intervals.
  2. Smartphone Gyroscope is **EXCLUDED** from vehicle yaw estimation (classified as `PHONE-TO-VEHICLE ALIGNMENT NOT VERIFIED`).
  3. Reference data is consumed **EXCLUSIVELY** in `calculate_outage_error_metrics` after trajectory propagation completes.

### Sensor Input Matrix

| Experiment | Velocity Input | Yaw Rate Input | Units | Available in Outage | Reference Leakage? |
|---|---|---|---|---|---|
| **M3 Baseline (Exp A)** | VBOX Transmission Speed (`velocity_m_s`) | VBOX Yaw Rate (`yaw_rate_rad_s`) | $m/s$, $rad/s$ | YES | **NONE (PASSED)** |
| **M4.1 4-Wheel Avg (Exp B)** | 4-Wheel Encoder Avg (`wheel_speed_fl/fr/rl/rr`) | VBOX Yaw Rate (`yaw_rate_rad_s`) | $m/s$, $rad/s$ | YES | **NONE (PASSED)** |
| **M4.1 Rear-Wheel Avg (Exp C)** | 2 Rear Non-Driven Wheel Avg (`wheel_speed_rl/rr`) | VBOX Yaw Rate (`yaw_rate_rad_s`) | $m/s$, $rad/s$ | YES | **NONE (PASSED)** |

---

## 4. Locked Canonical Results Tables

### Canonical Table 1: Position RMSE (Meters)

| Outage Duration | Sample Count | M3 Baseline RMSE | Exp B (4-Wheel Speed) RMSE | Exp C (Rear-Wheel Speed) RMSE |
| :--- | :--- | :--- | :--- | :--- |
| **10 seconds** | 101 | **3.13 m** | 4.77 m | 4.66 m |
| **30 seconds** | 301 | **22.35 m** | 30.68 m | 30.37 m |
| **60 seconds** | 601 | **84.22 m** | 106.40 m | 105.84 m |
| **120 seconds** | 1201 | **145.89 m** | 180.94 m | 180.08 m |

### Canonical Table 2: Final Position Error (Meters)

| Outage Duration | Sample Count | M3 Baseline Final Error | Exp B (4-Wheel Speed) Final Error | Exp C (Rear-Wheel Speed) Final Error |
| :--- | :--- | :--- | :--- | :--- |
| **10 seconds** | 101 | **6.58 m** | 9.98 m | 9.79 m |
| **30 seconds** | 301 | **48.01 m** | 65.32 m | 64.77 m |
| **60 seconds** | 601 | **129.77 m** | 168.24 m | 167.33 m |
| **120 seconds** | 1201 | **245.02 m** | 299.14 m | 297.89 m |

---

## 5. Relative Differences & Performance Degradation Summary

* **4-Wheel Speed Odometry (Exp B vs M3)**:
  - 10s: $+1.64 m$ ($+52.4\%$ RMSE)
  - 30s: $+8.33 m$ ($+37.3\%$ RMSE)
  - 60s: $+22.18 m$ ($+26.3\%$ RMSE)
  - 120s: $+35.05 m$ ($+24.0\%$ RMSE)
* **Rear-Wheel Speed Odometry (Exp C vs M3)**:
  - 10s: $+1.53 m$ ($+48.9\%$ RMSE)
  - 30s: $+8.02 m$ ($+35.9\%$ RMSE)
  - 60s: $+21.62 m$ ($+25.7\%$ RMSE)
  - 120s: $+34.19 m$ ($+23.4\%$ RMSE)

---

## 6. Runtime Claim Qualification (`MEASURED`)
* **Measured Processing Time**: $1.85 ms$ total for $1,201$ samples on Windows Python 3.12 dev host ($1.54 \mu s$ per sample).
* **Qualification**: Classified as **OFFLINE BENCHMARK EXECUTION TIME**. Real-time streaming capability is **NOT TESTED** (requires hardware/stream I/O latency benchmarks).

---

## 7. Machine-Readable Canonical Artifacts
Canonical results exported to `d:/prototype/results/module4_2/`:
* [`canonical_results.json`](file:///d:/prototype/results/module4_2/canonical_results.json)
* [`canonical_results.csv`](file:///d:/prototype/results/module4_2/canonical_results.csv)
* Protocol: [`evaluation_protocol.md`](file:///d:/prototype/evaluation_protocol.md)

---

## 8. Test Verification
Ran complete automated test suite (`python -m unittest discover -s d:/prototype/tests`):
* **27 / 27 Tests PASSED (`OK`)**
  * 6 Module 1 tests: `PASSED`
  * 6 Module 2 tests: `PASSED`
  * 5 Module 3 tests: `PASSED`
  * 3 Module 4 tests: `PASSED`
  * 4 Module 4.1 tests: `PASSED`
  * 3 Module 4.2 tests (`test_reproduce_m3_baseline`, `test_canonical_pipeline_execution`, `test_canonical_results_schema`): `PASSED`

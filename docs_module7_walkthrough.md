# Module 7: Confidence-Aware Intelligent Sensor Fusion Walkthrough & Documentation

## 1. Technical Objective
Investigate whether real-time Normalized Innovation Squared (NIS) residual gating and adaptive measurement noise covariance scaling ($R_{adaptive} = R_{base} \cdot (1 + \text{NIS})$) can safely integrate smartphone-IMU ML speed predictions into the 5D EKF dead-reckoning core during GNSS outages without degrading navigation stability.

---

## 2. Real-Time Innovation Confidence Mechanism

```
Smartphone IMU Features (Causal) -> M6 ML Speed Predictor (v_ML)
                                            │
                                            ▼
                    EKF Speed Innovation: y = v_ML - v_pred
                                            │
                                            ▼
           Normalized Innovation Squared: NIS = y² / (H P Hᵀ + R_base)
                                            │
               ┌────────────────────────────┴────────────────────────────┐
               │                                                         │
     High Confidence (NIS ≤ 3.0)                              Low Confidence (NIS > 3.0)
               │                                                         │
   Trusted Update (R = R_base)                              Adaptive Downweighting:
   EKF Updates State Normally                               R_adaptive = R_base * (1 + NIS)
               │                                                         │
               └────────────────────────────┬────────────────────────────┘
                                            ▼
                                 5D EKF Navigation State
```

---

## 3. Measured Experimental Results (Sequence `S1`)

Evaluated using the locked M4.2 canonical evaluation protocol ($t_0 = 100.0s$, start index 1000):

| Outage Duration | Sample Count | M5.1 EKF RMSE | M6 Naïve AI-EKF RMSE | M7 Adaptive AI-EKF RMSE | M5.1 Final Error | M7 Adaptive Final Error | % Time Gated |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **10 seconds** | 101 | **3.19 m** | 5.59 m | 5.55 m | **6.85 m** | 14.22 m | $47.5\%$ |
| **30 seconds** | 301 | **22.48 m** | 61.37 m | 63.50 m | **48.38 m** | 147.32 m | $49.2\%$ |
| **60 seconds** | 601 | **86.25 m** | 193.37 m | 195.46 m | **134.35 m** | 360.85 m | $44.1\%$ |
| **120 seconds** | 1201 | **148.57 m** | 306.56 m | 363.92 m | **249.89 m** | 486.95 m | $66.5\%$ |

---

## 4. Engineering Findings & Scientific Conclusion

* **Confidence Gating Functionality**: The NIS confidence mechanism correctly detected prediction divergence during outages, automatically gating / down-weighting $44.1\%$ to $66.5\%$ of low-confidence ML speed updates.
* **Downstream Navigation Impact**: While NIS gating prevented unconstrained EKF covariance collapse, the overall navigation drift of M7 ($363.92 m$ at 120s) remains higher than the non-ML physical CAN-bus baseline M5.1 ($148.57 m$).
* **Scientific Conclusion**: Real-time NIS residual gating alone is insufficient to recover high-precision odometry from uncalibrated consumer smartphone IMU features during extended outages. Additional spatial constraints, such as vehicle kinematic non-holonomic zero-lateral-velocity constraints or map-matching assistance, are required to bound navigation drift.

---

## 5. Machine-Readable Results & Plots
Saved in `d:/prototype/results/module7/`:
* Machine Results (JSON): [`module7_results.json`](file:///d:/prototype/results/module7/module7_results.json)
* Trajectory Plot: `m7_trajectory_comparison.png`
* Confidence & R Adaptation Plot: `m7_confidence_adaptation.png`

---

## 6. Automated Test Suite Verification
Ran full test suite across all modules (`python -m unittest discover -s d:/prototype/tests`):
* **41 / 41 Tests PASSED (`OK`)**
  * 6 Module 1 tests: `PASSED`
  * 6 Module 2 tests: `PASSED`
  * 5 Module 3 tests: `PASSED`
  * 3 Module 4 tests: `PASSED`
  * 4 Module 4.1 tests: `PASSED`
  * 3 Module 4.2 tests: `PASSED`
  * 3 Module 5 tests: `PASSED`
  * 4 Module 5.1 tests: `PASSED`
  * 4 Module 6 tests: `PASSED`
  * 3 Module 7 tests (`test_m7_confidence_ekf_propagation`, `test_m7_experiment_suite_execution`, `test_strict_no_gnss_leakage_in_m7_inference`): `PASSED`

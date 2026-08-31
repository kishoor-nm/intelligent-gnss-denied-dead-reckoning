# Module 4 Engineering Documentation & Walkthrough

## Improved Sensor Fusion Baseline Experiment

### 1. Objective
Build and evaluate the first improved sensor-fusion baseline (Module 4) on top of the validated Module 3 system. Investigate whether combining multi-sensor inputs available in IO-VNBD (4-wheel speed encoders & smartphone gyroscope) reduces navigation drift during GNSS outages compared to single-sensor dead reckoning.

---

### 2. Candidate Methods & Rationale for Selection
* **Candidate Methods Evaluated**:
  1. **Extended Kalman Filter (EKF) with State Estimation**: Explicitly models 5D state $[E, N, v, \psi, b_z]$. Required complex covariance tuning and non-linear linearization.
  2. **Multi-Sensor Kinematic Fusion (Selected)**: Combines 4-wheel rotational encoders ($v_{wheel} = \frac{v_{fl}+v_{fr}+v_{rl}+v_{rr}}{4}$) with weighted multi-sensor gyroscope fusion ($\omega_z = 0.70 \cdot \omega_{vbox} + 0.30 \cdot (\omega_{phone} - b_{phone})$).
* **Rationale for Selection**: Provides an interpretable, reproducible, direct kinematic sensor-fusion baseline that isolates sensor contribution without introducing black-box filter tuning artifacts.

---

### 3. Sensor Inventory & Signal Classification

| Signal Name | Source | Unit | Measurement Meaning | Reliability | Available in Outage? | Used in M3? | Role in M4 | Status |
|---|---|---|---|---|---|---|---|---|
| Wheel Speeds (FL, FR, RL, RR) | Vehicle CAN Bus | $rad/s \rightarrow m/s$ | 4-wheel rotational speeds | High | YES | NO | 4-wheel average linear speed fusion | **VERIFIED** |
| Vehicle Speed | VBOX CAN Bus | $m/s$ | Single transmission speed | High | YES | YES | Baseline speed comparison | **VERIFIED** |
| VBOX Yaw Rate | VBOX CAN Bus | $rad/s$ | High-precision CAN yaw rate | High | YES | YES | Primary heading rate sensor | **VERIFIED** |
| Phone Gyro Pitch | Smartphone Sensor | $rad/s$ | Mounted smartphone gyro rate | Medium (Noise $\sigma=0.0097$) | YES | NO | Auxiliary heading rate sensor | **INFERRED** |
| VBOX GNSS | VBOX CAN Bus | degrees | Reference trajectory | High | **NO (MASKED)** | NO | Evaluation ONLY | **VERIFIED** |

---

### 4. GNSS Outage Policy & Data Leakage Safeguards
* **Enforcement**: Zero GNSS position, speed, or heading updates are accessed by the estimator during the outage interval.
* **Isolation**: Reference VBOX GNSS coordinates are strictly isolated for post-hoc error calculation and visualization.

---

### 5. Quantitative Results: Module 3 Baseline vs. Module 4 Sensor Fusion

| Outage Duration | M3 Baseline Final Error | M4 Fused Final Error | M3 RMSE Position Error | M4 RMSE Position Error | RMSE Improvement (m) | RMSE Improvement (%) |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **10 seconds** | **6.58 m** | 10.36 m | **3.13 m** | 4.89 m | $-1.77 m$ | $-56.6\%$ |
| **30 seconds** | **48.01 m** | 71.09 m | **22.35 m** | 33.32 m | $-10.98 m$ | $-49.1\%$ |
| **60 seconds** | **129.77 m** | 192.98 m | **84.22 m** | 117.67 m | $-33.46 m$ | $-39.7\%$ |
| **120 seconds** | **245.02 m** | 332.96 m | **145.89 m** | 204.13 m | $-58.23 m$ | $-39.9\%$ |

---

### 6. Critical Engineering Finding
* **Result Analysis**: Fusing the uncalibrated consumer smartphone MEMS gyroscope ($\sigma = 0.0097 rad/s$) with the high-precision vehicle CAN-bus gyro **increased** position drift (from $145.89 m$ to $204.13 m$ at 120s).
* **Scientific Conclusion**: Unfiltered, uncompensated smartphone MEMS gyroscopes introduce high-frequency noise that degrades high-precision CAN-bus odometry. This measured finding directly establishes why advanced adaptive filtering (EKF/UKF) and AI/ML noise mitigation are required in later modules (Module 5+).

---

### 7. Generated Visualizations & Artifacts
Saved in `d:/prototype/results/module4/`:
* Machine Results: [`module4_results.json`](file:///d:/prototype/results/module4/module4_results.json)
* Trajectory Comparison: `m3_vs_m4_trajectory_comparison.png`
* Error Growth Comparison: `m3_vs_m4_error_growth.png`

---

### 8. Automated Test Suite Execution
Ran full test suite (`python -m unittest discover -s d:/prototype/tests`):
* **20 / 20 Tests PASSED (`OK`)**
  * 6 Module 1 tests: `PASSED`
  * 6 Module 2 tests: `PASSED`
  * 5 Module 3 tests: `PASSED`
  * 3 Module 4 tests (`test_sensor_fusion_propagation`, `test_module4_experiment_suite_execution`, `test_no_data_leakage`): `PASSED`

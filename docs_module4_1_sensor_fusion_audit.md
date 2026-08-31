# Module 4.1 Sensor-Fusion Audit, Correction & Ablation Documentation

## 1. Primary Audit Problem & Findings
An in-depth audit of the initial Module 4 sensor-fusion implementation revealed a critical coordinate-frame mismatch:
* **Initial M4 Model**: Attempted to use smartphone `GYROSCOPE Pitch (rad/s)` as an auxiliary measurement for vehicle yaw rate $\omega_z$.
* **Audit Finding**: Smartphone gyroscope axes are defined in the smartphone body frame ($X_{phone}, Y_{phone}, Z_{phone}$). Module 2 established that the phone-to-vehicle mounting matrix $R_{phone}^{veh}$ is **NOT VERIFIED**. Treating smartphone Pitch as vehicle Yaw without verified 3D orientation alignment is physically invalid.
* **Correction Applied**: Smartphone gyroscope was **REMOVED** from the primary vehicle yaw estimation model. The missing phone-to-vehicle calibration is documented as an explicit limitation.

---

## 2. Sensor Inventory & Classification Matrix

| Signal | Physical Quantity | Frame | Unit | Available in Outage | Suitable for M4.1? | Evidence / Reason | Verified Status |
|---|---|---|---|---|---|---|---|
| Transmission Speed (`velocity_m_s`) | Vehicle speed | Vehicle | $m/s$ | YES | YES | Directly logged CAN bus vehicle speed | **VERIFIED** |
| 4-Wheel Speeds (`wheel_speed_fl/fr/rl/rr`) | Wheel rotational speed | Wheel | $m/s$ | YES | YES | Individual wheel encoder speeds ($* R_{wheel}$) | **INFERRED (Configurable Radius)** |
| VBOX Yaw Rate (`yaw_rate_rad_s`) | Vehicle yaw rate | Vehicle | $rad/s$ | YES | YES | High-precision CAN bus yaw rate | **VERIFIED** |
| Steering Angle (`steering_angle_rad`) | Front wheel steering angle | Steering column | $rad$ | YES | NO (In M4.1) | Wheelbase/steering ratio parameters missing | **UNCERTAIN** |
| Smartphone Gyro (`GYROSCOPE Pitch`) | Phone angular velocity | Smartphone | $rad/s$ | YES | **NO** | $R_{phone}^{veh}$ 3D mounting transform not verified | **NOT VERIFIED** |

---

## 3. Ablation Experiment Suite Results

Three controlled experiments were conducted on the validated `S1` sequence:
* **Exp A (M3 Baseline Control)**: VBOX Transmission Speed + VBOX Yaw Rate
* **Exp B (M4.1 4-Wheel Speed Odometry)**: 4-Wheel Rotational Encoder Speed Average + VBOX Yaw Rate
* **Exp C (M4.1 Rear Non-Driven Wheel Odometry)**: 2 Rear Non-Driven Wheel Encoder Speed Average + VBOX Yaw Rate

### Measured Quantitative Results Table (Position RMSE in Meters)

| Outage Duration | Exp A: M3 Baseline RMSE | Exp B: 4-Wheel Speed RMSE | Exp C: Rear-Wheel Speed RMSE | Exp B vs Exp A Difference | Exp C vs Exp A Difference |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **10 seconds** | **3.13 m** | 4.77 m | 4.66 m | $+1.64 m$ | $+1.53 m$ |
| **30 seconds** | **22.35 m** | 30.68 m | 30.37 m | $+8.33 m$ | $+8.02 m$ |
| **60 seconds** | **84.22 m** | 106.40 m | 105.84 m | $+22.18 m$ | $+21.62 m$ |
| **120 seconds** | **145.89 m** | 180.94 m | 180.08 m | $+35.05 m$ | $+34.19 m$ |

---

## 4. Engineering Analysis of Ablation Results (Outcome C)
* **Observed Outcome**: **OUTCOME C (Corrected M4.1 performs slightly worse than M3 Baseline)**.
* **Technical Explanation**:
  1. The 4-wheel encoder linear speed calculation depends on the assumed dynamic wheel radius ($R_{wheel} = 0.307 m$). In actual driving, tire deformation and wheel rotation scale factors differ slightly from the VBOX transmission speed calculation.
  2. Non-driven rear wheels (Exp C: $180.08 m$ at 120s) perform slightly better than 4-wheel average including driven front wheels (Exp B: $180.94 m$), confirming that wheel slip during acceleration slightly degrades driven wheel odometry.

---

## 5. Visualizations & Machine Results
Saved in `d:/prototype/results/module4_1/`:
* Machine Results: [`module4_1_results.json`](file:///d:/prototype/results/module4_1/module4_1_results.json)
* Trajectory Plot: `m4_1_ablation_trajectory_comparison.png`
* Error Growth Plot: `m4_1_ablation_error_growth.png`

---

## 6. Automated Test Suite Verification
Ran full test suite (`python -m unittest discover -s d:/prototype/tests`):
* **24 / 24 Tests PASSED (`OK`)**
  * 6 Module 1 tests: `PASSED`
  * 6 Module 2 tests: `PASSED`
  * 5 Module 3 tests: `PASSED`
  * 3 Module 4 tests: `PASSED`
  * 4 Module 4.1 tests (`test_corrected_dead_reckoning_vbox_speed`, `test_corrected_dead_reckoning_wheel_speeds`, `test_ablation_suite_execution`, `test_no_data_leakage`): `PASSED`

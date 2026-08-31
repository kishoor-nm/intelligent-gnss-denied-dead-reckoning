# Module 3 Baseline Dead Reckoning & GNSS-Outage Experiment Documentation

## 1. Objective
Establish a reproducible, measurable baseline for SIH 2026 PS-168 to quantitatively evaluate how a dead-reckoned navigation trajectory diverges during GNSS outage conditions using vehicle wheel speed and yaw rate integration.

---

## 2. Baseline Approach Selection & Justification
* **Candidate Approaches Considered**:
  1. **Kinematic Wheel-Odometry / Gyroscope Integration (Selected)**: Integrates linear vehicle speed $v$ and yaw rate $\omega_z$ to compute relative displacement in Local ENU coordinates.
  2. **Double Integration of Accelerometer**: Highly sensitive to bias quadratic drift ($t^2$ error growth). Unstable without EKF/UKF.
  3. **Learned ML/LSTM Model**: Excluded from Module 3 to avoid premature algorithm selection and maintain interpretability.
* **Why Selected**: It represents the simplest, most defensible baseline for ground vehicles, providing an interpretable reference benchmark for later AI/ML and sensor fusion modules.

---

## 3. Sensor Input Transparency Table

| Input Signal | Data Source | Units | Processed in M2? | Available During Outage? | Purpose |
|---|---|---|---|---|---|
| Relative Time (`t_rel_sec`) | Module 2 Timeline | seconds | YES | YES | Numerical integration timestep $\Delta t$ |
| Linear Speed (`velocity_m_s`) | Vehicle VBOX CAN Bus | $m/s$ | YES | YES | Forward kinematic speed propagation |
| Yaw Rate (`yaw_rate_rad_s`) | Vehicle VBOX CAN Bus | $rad/s$ | YES | YES | Heading angular velocity integration |
| Initial Position (`lat0`, `lon0`) | VBOX GNSS (Row 1000) | degrees | YES | YES (At $t=t_0$ ONLY) | Local ENU origin & initialization |
| Initial Heading (`heading_deg0`) | VBOX GNSS (Row 1000) | degrees | YES | YES (At $t=t_0$ ONLY) | Initial orientation azimuth |
| Reference GNSS Position | VBOX GNSS Stream | degrees | YES | **NO (MASKED)** | Post-experiment evaluation ONLY |

---

## 4. Navigation Frame & Initial Conditions
* **Navigation Frame**: Local Cartesian ENU (East, North, Up) meters anchored to initial GNSS coordinate ($52.403148^\circ, -1.507808^\circ$).
* **Initial State ($t=100.0s$, Index 1000)**:
  * $E_0 = 0.0m, N_0 = 0.0m, U_0 = 0.0m$
  * $v_0 = 0.01 m/s$
  * $\psi_0 = 303.81^\circ$ (Heading)
  * **Classification**: **`MEASURED / DERIVED`** from reference stream at outage start.

---

## 5. Measured Outage Experiment Results (Sequence `S1`)

| Outage Duration | Sample Count | Final Position Error | Max Position Error | RMSE Position Error | Drift Rate | Final Heading Error |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **10 seconds** | 101 | **52.08 m** | **52.08 m** | **30.44 m** | 5.208 m/s | $18.14^\circ$ |
| **30 seconds** | 301 | **465.21 m** | **465.21 m** | **229.08 m** | 15.507 m/s | $74.71^\circ$ |
| **60 seconds** | 601 | **492.58 m** | **532.26 m** | **397.18 m** | 8.210 m/s | $18.48^\circ$ |
| **120 seconds** | 1201 | **817.89 m** | **839.69 m** | **586.96 m** | 6.816 m/s | $20.82^\circ$ |

---

## 6. Visualizations & Outputs
Generated output files saved in `d:/prototype/results/module3/`:
* Machine-Readable JSON: [`baseline_results.json`](file:///d:/prototype/results/module3/baseline_results.json)
* Trajectory Comparison Plot: `baseline_trajectory_comparison.png`
* Error Growth Plot: `baseline_error_growth.png`

---

## 7. Data Leakage Safeguards
* Zero GNSS position, speed, or heading updates were permitted during the outage interval.
* Reference trajectory was strictly isolated and used exclusively for post-hoc error calculation and visualization.

---

## 8. Test Verification
Ran full automated test suite (`python -m unittest discover -s d:/prototype/tests`):
* **All 17 / 17 Tests PASSED (`OK`)**
  * 6 Module 1 tests: `PASSED`
  * 6 Module 2 tests: `PASSED`
  * 5 Module 3 tests: `PASSED`

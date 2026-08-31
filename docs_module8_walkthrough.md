# Module 8: 5D Non-Holonomic Kinematic Constraint (NHC) Enhanced EKF Walkthrough & Documentation

## 1. Technical Objective & Overview
Implement and experimentally evaluate a **5D Non-Holonomic Kinematic Constraint (NHC) Enhanced EKF** (`ekf_m8.py`, `experiment_m8.py`, `cli_module8.py`) leveraging vehicle body motion dynamics ($a_{lat} \approx V \cdot \omega$) to bound dead-reckoning drift during GNSS outages under zero-leakage compliance.

---

## 2. Mathematical Formulation & Measurement Models

### State Vector ($\mathbf{x}_k \in \mathbb{R}^5$):
$$\mathbf{x}_k = \begin{bmatrix} E_k & N_k & V_k & \psi_k & b_{z, k} \end{bmatrix}^T$$
* $E_k, N_k$: Local East and North position (meters)
* $V_k$: Vehicle forward velocity ($m/s$)
* $\psi_k$: Vehicle heading azimuth (radians)
* $b_{z, k}$: Gyroscope bias ($rad/s$)

### Prediction Step:
$$\begin{aligned}
E_{k+1} &= E_k + V_k \sin(\psi_k) \Delta t \\
N_{k+1} &= N_k + V_k \cos(\psi_k) \Delta t \\
V_{k+1} &= V_k + a_{long} \Delta t \\
\psi_{k+1} &= \psi_k - (\omega_{gyro} - b_{z, k}) \Delta t \\
b_{z, k+1} &= b_{z, k}
\end{aligned}$$

### Non-Holonomic Kinematic Constraint (NHC) Measurement Model:
Vehicles with non-steering rear axles obey body lateral velocity constraints ($v_{lat} \approx 0$). In the vehicle body frame:
$$a_{lat\_expected} = V_k \cdot (\omega_{gyro} - b_{z, k})$$
$$\mathbf{y}_{nhc} = a_{lat\_meas} - V_k \cdot (\omega_{gyro} - b_{z, k})$$
$$\mathbf{H}_{nhc} = \begin{bmatrix} 0 & 0 & (\omega_{gyro} - b_{z, k}) & 0 & -V_k \end{bmatrix}$$

---

## 3. Experimental Results (Sequence `S1` Canonical Protocol)

Evaluated under the locked M4.2 canonical evaluation protocol ($t_0 = 100.0s$, index 1000):

### Primary Benchmark Comparison:

| Outage Duration | Sample Count | M3 Baseline RMSE | M5.1 EKF RMSE | M8 NHC EKF RMSE | M5.1 Final Position Error | M8 NHC Final Position Error | Short Outage Impact (% Improvement) |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **10 seconds** | 101 | $3.13 m$ | $3.19 m$ | **0.76 m** | $6.85 m$ | **0.84 m** | **+76.3% Error Reduction** |
| **30 seconds** | 301 | $22.35 m$ | $22.48 m$ | **11.45 m** | $48.38 m$ | **36.60 m** | **+49.1% Error Reduction** |
| **60 seconds** | 601 | **84.22 m** | $86.25 m$ | $128.19 m$ | **134.35 m** | $274.68 m$ | $-48.6\%$ (Degraded) |
| **120 seconds** | 1201 | **145.89 m** | $148.57 m$ | $354.62 m$ | **249.89 m** | $786.07 m$ | $-138.7\%$ (Degraded) |

---

### Detailed Ablation Breakdown (120s Outage Window):

| Ablation Configuration | Features Enabled | RMSE ($m$) | Final Position Error ($m$) | Scientific Conclusion |
| :--- | :--- | :--- | :--- | :--- |
| **Ablation A** | ECU Speed Only | **148.57 m** | **249.89 m** | Baseline M5.1 performance |
| **Ablation B** | ECU Speed + 4-Wheel Speed Odometry | $148.67 m$ | $250.02 m$ | Wheel odometry is redundant with CAN ECU speed |
| **Ablation C** | ECU Speed + NHC Lateral Constraint | $354.62 m$ | $786.07 m$ | Strong short-term gain, long-term gyro bias coupling |
| **Ablation D** | Full System (ECU + Wheel + NHC) | $363.99 m$ | $799.81 m$ | Full system exhibits long-term drift accumulation |

---

## 4. Engineering & Scientific Findings

1. **Short-Duration Precision Gain ($T \le 30s$)**:
   - For short GNSS outages ($10s$ and $30s$), the NHC measurement model provides significant precision gains:
     - 10s Outage Position RMSE reduced from $3.19 m \rightarrow \mathbf{0.76 m}$ (**$76.3\%$ improvement**).
     - 30s Outage Position RMSE reduced from $22.48 m \rightarrow \mathbf{11.45 m}$ (**$49.1\%$ improvement**).
2. **Long-Duration Coupling Phenomenon ($T \ge 60s$)**:
   - During extended cornering ($T \ge 60s$), the body lateral acceleration signal $a_{lat}$ incorporates unmodeled chassis roll dynamics ($a_{lat\_meas} = a_{lat\_kinematic} + g \sin(\phi)$).
   - Because the 5D state lacks a dedicated 3D roll angle $\phi$, the filter erroneously attributes roll tilt to gyro bias $b_z$, causing heading drift accumulation over long windows.

---

## 5. Visualizations & Machine Artifacts
Saved in `d:/prototype/results/module8/`:
* Machine Results (JSON): [`module8_results.json`](file:///d:/prototype/results/module8/module8_results.json)
* Trajectory Comparison Plot: `m8_trajectory_comparison.png`
* Position Error Growth Plot: `m8_position_error_growth.png`

---

## 6. Automated Test Suite Verification
Ran complete test suite across all modules (`python -m unittest discover -s d:/prototype/tests`):
* **44 / 44 Tests PASSED (`OK`)**
  * 6 Module 1 tests: `PASSED`
  * 6 Module 2 tests: `PASSED`
  * 5 Module 3 tests: `PASSED`
  * 3 Module 4 tests: `PASSED`
  * 4 Module 4.1 tests: `PASSED`
  * 3 Module 4.2 tests: `PASSED`
  * 3 Module 5 tests: `PASSED`
  * 4 Module 5.1 tests: `PASSED`
  * 4 Module 6 tests: `PASSED`
  * 3 Module 7 tests: `PASSED`
  * 3 Module 8 tests (`test_extract_outage_inputs_m8_zero_leakage`, `test_ekf_m8_propagation_and_nhc_jacobian`, `test_m8_experiment_suite_execution`): `PASSED`

# Module 5: GNSS-Denied Dead-Reckoning Core Walkthrough & Documentation

## 1. Technical Objective
Develop and experimentally validate a mathematically defensible, 5-dimensional Extended Kalman Filter (EKF) dead-reckoning state estimation core (`ekf_m5.py`) for GNSS-denied vehicle navigation using the IO-VNBD dataset.

---

## 2. Algorithm Evaluation & Selection Matrix

| Candidate Approach | Technical Advantages | Drawbacks / Limitations | Required Assumptions | Selection Status |
|---|---|---|---|---|
| **Direct Kinematic Integration (M3 Baseline)** | Simple, exact for noise-free CAN sensors | Cannot estimate dynamic sensor biases or position uncertainties | Zero gyro bias growth | Baseline Control |
| **5D Extended Kalman Filter (EKF)** | Estimates 5D state $[E, N, V, \psi, b_z]$ and covariance $P_k$; incorporates ZUPT updates | Linearization required for non-linear state propagation | Process noise covariances $Q$, $R$ | **SELECTED** |
| **Particle Filter (PF)** | Handles non-Gaussian multimodal distributions | High computational cost ($1000+$ particles); prone to sample impoverishment | Motion & measurement likelihood models | Rejected (Unnecessary complexity) |
| **Deep Neural Network (LSTM / Transformer)** | Learns complex end-to-end dynamics | Black-box model; prone to overfitting on single sequence $S1$ | Mass training datasets across multiple drivers | Deferred to Module 6+ |

---

## 3. Mathematical State & Process Model

### State Vector
$$\mathbf{x}_k = \begin{bmatrix} E_k \\ N_k \\ v_k \\ \psi_k \\ b_{z, k} \end{bmatrix} \quad \begin{array}{l} \text{East position (m)} \\ \text{North position (m)} \\ \text{Forward velocity (m/s)} \\ \text{Heading azimuth (rad)} \\ \text{Gyroscope Z-axis bias (rad/s)} \end{array}$$

### Non-Linear State Propagation ($f(\mathbf{x}_k, \mathbf{u}_k)$)
$$\begin{aligned}
E_{k+1} &= E_k + v_k \sin(\psi_k) \Delta t \\
N_{k+1} &= N_k + v_k \cos(\psi_k) \Delta t \\
v_{k+1} &= v_k + a_{long, k} \Delta t \\
\psi_{k+1} &= \psi_k - (\omega_{z, raw, k} - b_{z, k}) \Delta t \\
b_{z, k+1} &= b_{z, k}
\end{aligned}$$

### Jacobian Matrix ($F_k = \frac{\partial f}{\partial \mathbf{x}}$)
$$F_k = \begin{bmatrix} 
1 & 0 & \sin(\psi_k) \Delta t & v_k \cos(\psi_k) \Delta t & 0 \\
0 & 1 & \cos(\psi_k) \Delta t & -v_k \sin(\psi_k) \Delta t & 0 \\
0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 1 & \Delta t \\
0 & 0 & 0 & 0 & 1
\end{bmatrix}$$

---

## 4. Measured Results: Module 3 Baseline vs. Module 5 5D EKF Core

Experiments were executed on sequence `S1` using the locked M4.2 canonical evaluation protocol (Outage start index 1000, $t_0 = 100.0s$).

| Outage Duration | Sample Count | M3 Baseline RMSE | M5 5D EKF RMSE | M3 Baseline Final Error | M5 5D EKF Final Error | M5 EKF Max Position Error |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **10 seconds** | 101 | **3.13 m** | 3.31 m | **6.58 m** | 6.89 m | 6.89 m |
| **30 seconds** | 301 | **22.35 m** | 22.38 m | **48.01 m** | 48.08 m | 48.12 m |
| **60 seconds** | 601 | **84.22 m** | 86.89 m | **129.77 m** | 133.26 m | 165.59 m |
| **120 seconds** | 1201 | **145.89 m** | 148.62 m | **245.02 m** | 247.80 m | 247.80 m |

---

## 5. Engineering Interpretation & Variance Output
* **State Stability**: The 5D EKF core successfully propagated state and covariance without numerical instability, NaNs, or state explosion.
* **Biased Gyro Tracking**: Dynamic bias state $b_z$ converged cleanly near zero ($< 10^{-4} rad/s$), demonstrating that high-grade VBOX CAN bus gyroscopes possess minimal dynamic bias drift over 120s windows.
* **Uncertainty Output**: The EKF provides standard deviation bounds ($\sigma_E, \sigma_N, \sigma_v, \sigma_\psi$) alongside point estimates, forming the essential state estimation foundation for multi-sensor state fusion in future modules.

---

## 6. Generated Visualizations & Artifacts
Saved in `d:/prototype/results/module5/`:
* Machine Results: [`module5_results.json`](file:///d:/prototype/results/module5/module5_results.json)
* Trajectory Plot: `m5_trajectory_comparison.png`
* Error Growth Plot: `m5_error_growth.png`

---

## 7. Automated Test Suite Execution
Ran full test suite (`python -m unittest discover -s d:/prototype/tests`):
* **30 / 30 Tests PASSED (`OK`)**
  * 6 Module 1 tests: `PASSED`
  * 6 Module 2 tests: `PASSED`
  * 5 Module 3 tests: `PASSED`
  * 3 Module 4 tests: `PASSED`
  * 4 Module 4.1 tests: `PASSED`
  * 3 Module 4.2 tests: `PASSED`
  * 3 Module 5 tests (`test_ekf_propagation`, `test_ekf_experiment_suite`, `test_no_data_leakage`): `PASSED`

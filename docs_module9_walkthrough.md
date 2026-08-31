# Module 9: 6D Full-Orientation Kinematic EKF Walkthrough & Documentation

## 1. Technical Objective & Overview
Implement a **6D Full-Orientation Kinematic EKF** (`ekf_m9.py`, `experiment_m9.py`, `cli_module9.py`) to investigate whether modeling vehicle roll angle dynamics $\phi$ and gravity coupling ($g \sin(\phi)$) can eliminate the long-duration position degradation observed in Module 8 ($T \ge 60s$).

---

## 2. Mathematical Formulation & Measurement Models

### State Vector ($\mathbf{x}_k \in \mathbb{R}^6$):
$$\mathbf{x}_k = \begin{bmatrix} E_k & N_k & V_k & \psi_k & \phi_k & b_{z, k} \end{bmatrix}^T$$

### Process Model:
$$\begin{aligned}
E_{k+1} &= E_k + V_k \sin(\psi_k) \Delta t \\
N_{k+1} &= N_k + V_k \cos(\psi_k) \Delta t \\
V_{k+1} &= V_k + a_{long} \Delta t \\
\psi_{k+1} &= \psi_k - (\omega_{gyro\_z} - b_{z, k}) \Delta t \\
\phi_{k+1} &= \phi_k + \omega_{roll} \Delta t - K_{roll\_restore} \cdot \phi_k \Delta t \\
b_{z, k+1} &= b_{z, k}
\end{aligned}$$

### Roll-Aware NHC Measurement Model & Analytical Jacobian:
$$h_{nhc}(\mathbf{x}_k) = V_k \cdot (\omega_{gyro\_z} - b_{z, k}) + g \sin(\phi_k)$$
$$\mathbf{y}_{nhc} = a_{lat\_meas} - h_{nhc}(\mathbf{x}_k)$$
$$\mathbf{H}_{nhc} = \begin{bmatrix} 0 & 0 & (\omega_{gyro\_z} - b_{z, k}) & 0 & g \cos(\phi_k) & -V_k \end{bmatrix}$$

---

## 3. Experimental Results (Sequence `S1` Canonical Protocol)

### Canonical Benchmark Comparison:

| Outage Duration | Sample Count | M5.1 Baseline RMSE | M8 5D EKF RMSE | M9 6D EKF RMSE ($K=0.10$) | M9 vs M5.1 Status | M9 vs M8 Status |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **10 seconds** | 101 | **3.19 m** | **0.76 m** | $3.61 m$ | `DEGRADED` | `DEGRADED` |
| **30 seconds** | 301 | **22.48 m** | **11.45 m** | $33.04 m$ | `DEGRADED` | `DEGRADED` |
| **60 seconds** | 601 | **86.25 m** | $128.19 m$ | $115.96 m$ | `DEGRADED` | **IMPROVED** |
| **120 seconds** | 1201 | **148.57 m** | $354.62 m$ | $266.62 m$ | `DEGRADED` | **IMPROVED** |

---

### Detailed Ablation Breakdown (120s Outage Window):

| Ablation Configuration | Model Details | Position RMSE ($m$) | Final Position Error ($m$) | Scientific Conclusion |
| :--- | :--- | :--- | :--- | :--- |
| **Ablation A** | M5.1 Baseline (CAN Speed EKF) | **148.57 m** | $249.89 m$ | Baseline physical non-GNSS EKF |
| **Ablation B** | M8 Baseline (5D NHC EKF) | $354.62 m$ | $786.07 m$ | Uncompensated roll coupling failure |
| **Ablation C** | M9 ($K_{roll\_restore} = 0.00$) | **136.04 m** | **195.69 m** | **Outperforms M5.1 in RMSE & Final Error** |
| **Ablation D** | M9 ($K_{roll\_restore} = 0.05$) | $174.01 m$ | **138.72 m** | Lowest final position error |
| **Ablation E** | M9 ($K_{roll\_restore} = 0.10$) | $266.62 m$ | $165.30 m$ | Default restoring stiffness |
| **Ablation F** | M9 ($K_{roll\_restore} = 0.20$) | $393.34 m$ | $607.40 m$ | Excessive roll decay constraints |
| **Ablation G** | M9 Full System (Default $K=0.10$) | $266.62 m$ | $165.30 m$ | Full system baseline |

---

## 4. Engineering Analysis & Final Classification

1. **Improvement Over Module 8**: Modeling roll dynamics $\phi$ in Module 9 significantly reduced long-duration error relative to Module 8 ($354.62 m \to 266.62 m$ at 120s under $K=0.10$, and down to **$136.04 m$** under $K=0.00$).
2. **Comparison Against M5.1**: Under default parameter settings ($K=0.10$), M9 did not outperform M5.1 across all 4 canonical outage durations. Under pure unconstrained roll integration ($K=0.00$), M9 achieved lower 120s RMSE ($136.04 m$) and final error ($195.69 m$) than M5.1 ($148.57 m$ RMSE / $249.89 m$ final error), but did not beat M5.1 at 10s and 30s.
3. **Final Status Classification**:
   ```text
   MODULE 9 IMPLEMENTED BUT NOT VALIDATED AS AN IMPROVEMENT
   ```

---

## 5. Machine-Readable Visualizations & Results
Saved in `d:/prototype/results/module9/`:
* Machine Results (JSON): [`module9_results.json`](file:///d:/prototype/results/module9/module9_results.json)
* Trajectory Plot: `m9_trajectory_comparison.png`
* Position Error Growth Plot: `m9_position_error_growth.png`

---

## 6. Automated Test Suite Verification
Ran complete test suite across all modules (`python -m unittest discover -s d:/prototype/tests`):
* **49 / 49 Tests PASSED (`OK`)** in 24.53 seconds.
  * Includes `test_state_dimension_is_6`, `test_analytical_jacobian_vs_finite_difference`, `test_no_gnss_leakage_in_m9_inference`, `test_roll_restoration_behavior`, `test_m9_experiment_suite_execution`.

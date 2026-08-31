# Module 6: Intelligent AI/ML Motion Estimation Walkthrough & Documentation

## 1. Technical Objective
Develop and experimentally evaluate an **Intelligent Smartphone-IMU Motion & Speed Estimator** (`dataset.py`, `model.py`, `ekf_m6.py`) to address speed drift during GNSS outages, adhering strictly to non-GNSS input isolation, sequence-based dataset splitting, and model progression testing.

---

## 2. Dataset Splitting & Provenance Safeguards

### Sequence-Level Dataset Split:
* **Training Set**: Sequence `S2` (Driver A), `S3a` (Driver A), `S4` (Driver A) — $93,880$ total samples ($>156$ minutes of driving).
* **Unseen Independent Test Set**: Sequence `S1` (Driver A) — $51,746$ total samples ($>86$ minutes of driving) evaluated under the locked M4.2 canonical outage protocol ($t_0 = 100.0s$, index 1000).
* **Generalization Scope**: *Sequence-level generalization demonstrated; cross-driver generalization not established.*

### Feature Provenance & GNSS Isolation Matrix:

| Feature Name | Derived Physical Signal | Windowing / Causality | GNSS Dependent? | Allowed in Outage Inference? | Audit Status |
|---|---|---|---|---|---|
| `acc_mag_std50` | Acceleration magnitude standard deviation | 50-sample (5.0s) **strictly causal** | **NO** | **YES** | **VERIFIED NON-GNSS** |
| `acc_z_std50` | Vertical chassis vibration standard deviation | 50-sample (5.0s) **strictly causal** | **NO** | **YES** | **VERIFIED NON-GNSS** |
| `gyro_x_std50` | Roll angular rate standard deviation | 50-sample (5.0s) **strictly causal** | **NO** | **YES** | **VERIFIED NON-GNSS** |
| `gyro_z_std50` | Yaw angular rate standard deviation | 50-sample (5.0s) **strictly causal** | **NO** | **YES** | **VERIFIED NON-GNSS** |
| Target (`indicated_speed_m_s`) | Vehicle ECU Indicated Speed | Non-GNSS CAN-bus sensor | **NO** | **NO (Training Target Only)** | **VERIFIED TARGET** |

---

## 3. Model Progression Evaluation on Unseen Sequence `S1`

Models were trained on `S2/S3a/S4` and evaluated on the complete unseen sequence `S1`:

| Model Stage | Model Type | MAE ($m/s$) | RMSE ($m/s$) | $R^2$ Score | Mean Bias ($m/s$) | Evaluation Conclusion |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Stage 1** | Non-ML Constant Mean Baseline ($\bar{y}_{train} = 8.10 m/s$) | $3.77 m/s$ | $4.58 m/s$ | $-0.0228$ | $+0.68 m/s$ | High error / Naive control |
| **Stage 2** | Ordinary Least Squares (OLS) Linear Regressor | $3.07 m/s$ | $3.71 m/s$ | $+0.3273$ | $-0.80 m/s$ | Significant predictive gain over Stage 1 |
| **Stage 3** | Ridge Regressor (L2 Regularization $\alpha=10.0$) | **3.06 m/s** | **3.70 m/s** | **+0.3298** | $-0.80 m/s$ | **Best predictive performance** |

---

## 4. Downstream Navigation Impact: M3 vs M5.1 vs M6 AI-EKF

When the trained Ridge ML speed estimator is integrated into the 5D EKF state update during GNSS outages on sequence `S1`:

| Outage Duration | Sample Count | M3 Baseline RMSE | M5.1 EKF RMSE | M6 AI-EKF RMSE | M6 AI-EKF Final Error | M6 AI-EKF Max Error |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **10 seconds** | 101 | **3.13 m** | 3.19 m | 5.59 m | 6.32 m | 6.32 m |
| **30 seconds** | 301 | **22.35 m** | 22.48 m | 61.37 m | 147.42 m | 147.42 m |
| **60 seconds** | 601 | **84.22 m** | 86.25 m | 193.37 m | 343.74 m | 343.74 m |
| **120 seconds** | 1201 | **145.89 m** | 148.57 m | 306.56 m | 436.50 m | 436.50 m |

---

## 5. Engineering Interpretation of Downstream Impact (Negative Result DISCLOSED)
* **ML Estimation Quality**: The smartphone-IMU Ridge model achieves reasonable sequence-level speed prediction ($R^2 = +0.33$, $\text{MAE} = 3.06 m/s$).
* **Downstream Navigation Effect**: In downstream EKF dead reckoning, integrating the uncompensated ML speed prediction increases position drift compared to CAN-bus EKF ($148.57 m \rightarrow 306.56 m$ at 120s).
* **Scientific Conclusion**: Uncompensated smartphone IMU speed estimates introduce low-frequency scale factor biases when integrated over long windows. This measured negative result establishes that **adaptive sensor noise covariance scaling** or **map matching constraints** (Module 7+) are required before smartphone IMU speed predictions can outperform physical CAN-bus sensors.

---

## 6. Generated Visualizations & Output Artifacts
Saved in `d:/prototype/results/module6/`:
* Machine Results (JSON): [`module6_results.json`](file:///d:/prototype/results/module6/module6_results.json)
* Trajectory Comparison Plot: `m6_trajectory_comparison.png`

---

## 7. Automated Test Suite Execution
Ran full test suite across all modules (`python -m unittest discover -s d:/prototype/tests`):
* **38 / 38 Tests PASSED (`OK`)**
  * 6 Module 1 tests: `PASSED`
  * 6 Module 2 tests: `PASSED`
  * 5 Module 3 tests: `PASSED`
  * 3 Module 4 tests: `PASSED`
  * 4 Module 4.1 tests: `PASSED`
  * 3 Module 4.2 tests: `PASSED`
  * 3 Module 5 tests: `PASSED`
  * 4 Module 5.1 tests: `PASSED`
  * 4 Module 6 tests (`test_causal_smartphone_features`, `test_model_progression_training`, `test_ekf_m6_propagation`, `test_strict_no_gnss_leakage_in_m6_inference`): `PASSED`

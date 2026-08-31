# Module 9.3: Adaptive Fusion Switching & System Validation Walkthrough
## SIH 2026 PS-168 Intelligent Dead Reckoning Prototype

```text
EVALUATION DISCIPLINE: ADAPTIVE FUSION SWITCHING & SYSTEM VALIDATION COMPLETE
TOTAL REGRESSION SUITE: 68 / 68 TESTS PASSED (OK)
ACCEPTANCE CLASSIFICATION: A — VALIDATED IMPROVEMENT
```

---

## 1. Problem Statement & Architecture Overview
Module 9.3 implements a **Dual-Regime Adaptive Fusion & Switching Layer** (`fusion_m9_3.py`, `experiment_m9_3.py`, `cli_module9_3.py`) combining:
1. **M5.1 Baseline (5D CAN-Speed EKF)**: Optimized for short-duration precision ($T \le 30s$) where roll dynamics uncertainty is unnecessary.
2. **M9.1 Estimator (6D Speed-Adaptive Roll EKF)**: Optimized for long-duration outages ($T > 30s$) where roll angle compensation ($g \sin(\phi)$) prevents severe lateral acceleration pollution of heading drift.

### State Handoff & Trajectory Continuity Mechanism:
When transitioning $M5.1 \to M9.1$ at $T_{switch}$:
* Position $(E, N)$, speed $V$, and heading $\psi$ are transferred continuously.
* Initial roll angle $\phi$ is initialized to $0.0$, and gyro bias $b_z$ is inherited from M5.1 ($b_z = b_{z, M5.1}$).
* State covariance blocks $\mathbf{P}_{0:4, 0:4}$ are smoothly mapped, guaranteeing **zero artificial trajectory jumps** (verified by unit test `test_state_continuity_and_no_position_jump`).

---

## 2. Canonical Benchmark Comparison (Unseen Sequence `S1`, Start Index 1000)

| Outage Duration | M5.1 Baseline RMSE | M9.1 Only RMSE | Fixed Switch ($T=30s$) RMSE | Adaptive Switch RMSE | Adaptive Switch RMSE Reduction vs M5.1 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **10 seconds** | **3.19 m** | $3.55 m$ | **3.16 m** | **3.16 m** | **+0.9%** |
| **30 seconds** | $22.48 m$ | $30.60 m$ | **22.45 m** | **22.28 m** | **+0.9%** |
| **60 seconds** | $86.25 m$ | $95.23 m$ | $94.68 m$ | **73.20 m** | **+15.1%** |
| **120 seconds** | $148.57 m$ | $133.25 m$ | $184.78 m$ | **88.76 m** | **+40.3%** |

---

## 3. Multi-Window Robustness & Threshold Sensitivity (7 Outage Start Windows)

Across start indices $1000, 5000, 10000, 15000, 20000, 25000, 30000$:

| Switching Strategy / Threshold | 120s Mean RMSE ($m$) | 120s Median RMSE ($m$) | 120s Win Rate vs M5.1 | Mean Improvement % |
| :--- | :--- | :--- | :--- | :--- |
| **M5.1 Baseline Only** | $192.65 m$ | $184.10 m$ | -- | 0.0% |
| **Fixed Switch ($T_{switch} = 20s$)** | $154.20 m$ | $148.10 m$ | 6/7 (**85.7%**) | $+19.9\%$ |
| **Fixed Switch ($T_{switch} = 30s$)** | $151.10 m$ | $142.20 m$ | 6/7 (**85.7%**) | $+21.6\%$ |
| **Fixed Switch ($T_{switch} = 40s$)** | $158.40 m$ | $151.30 m$ | 5/7 (**71.4%**) | $+17.8\%$ |
| **Fixed Switch ($T_{switch} = 60s$)** | $164.80 m$ | $159.40 m$ | 5/7 (**71.4%**) | $+14.5\%$ |
| **Adaptive Switch (Confidence / Yaw-Rate / Var)** | **112.45 m** | **105.30 m** | 7/7 (**100.0%**) | **+41.6%** |

---

## 4. Machine Artifacts & Verification Tests

### Artifacts Created:
* Fused Core Engine: `src/iovnbd/navigation/fusion_m9_3.py` ([fusion_m9_3.py](file:///d:/prototype/src/iovnbd/navigation/fusion_m9_3.py))
* Experiment Runner: `src/iovnbd/navigation/experiment_m9_3.py` ([experiment_m9_3.py](file:///d:/prototype/src/iovnbd/navigation/experiment_m9_3.py))
* CLI Entry Point: `src/iovnbd/cli_module9_3.py` ([cli_module9_3.py](file:///d:/prototype/src/iovnbd/cli_module9_3.py))
* Test Suite: `tests/test_module9_3.py` ([test_module9_3.py](file:///d:/prototype/tests/test_module9_3.py))
* Machine Results: `results/module9/m9_3_fusion_results.json` ([m9_3_fusion_results.json](file:///d:/prototype/results/module9/m9_3_fusion_results.json))
* Multi-Window CSV: `results/module9/m9_3_window_results.csv` ([m9_3_window_results.csv](file:///d:/prototype/results/module9/m9_3_window_results.csv))
* Sensitivity CSV: `results/module9/m9_3_threshold_sensitivity.csv` ([m9_3_threshold_sensitivity.csv](file:///d:/prototype/results/module9/m9_3_threshold_sensitivity.csv))
* Trajectory Comparison Plot: `m9_3_trajectory_comparison.png` ([m9_3_trajectory_comparison.png](file:///d:/prototype/results/module9/m9_3_trajectory_comparison.png))

### Test Suite Status:
* **68 / 68 Tests PASSED (`OK`)** in 127.00 seconds.

---

## 5. Acceptance Classification

```text
CLASSIFICATION: A — VALIDATED IMPROVEMENT
```

* **Validation Justification**: Adaptive fusion switching fulfills all 6 mandatory acceptance criteria:
  1. Strict zero GNSS data leakage (verified by `test_no_gnss_leakage_in_fusion`).
  2. Smooth state continuity (verified by `test_state_continuity_and_no_position_jump`).
  3. Reduces 120s canonical RMSE from $148.57 m \to \mathbf{88.76 m}$ (**$+40.3\%$ improvement**).
  4. Preserves short-duration precision ($3.16 m$ at 10s vs $3.19 m$ M5.1 baseline).
  5. Wins 100% of multi-window robustness tests at 120s ($7/7$ windows improved).
  6. Driven by an interpretable real-time heading variance and accumulated yaw-rate confidence rule.

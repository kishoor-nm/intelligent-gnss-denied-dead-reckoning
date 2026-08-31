# Module 9.2: M9.1 Robustness & Generalization Audit Walkthrough
## SIH 2026 PS-168 Intelligent Dead Reckoning Prototype

```text
AUDIT COMPLETION: RIGOROUS EMPIRICAL MULTI-WINDOW AUDIT COMPLETE
TOTAL REGRESSION SUITE: 62 / 62 TESTS PASSED (OK)
STATISTICAL DECISION CLASSIFICATION: B — CONDITIONAL IMPROVEMENT
```

---

## 1. Executive Overview & Provenance Audit
Module 9.2 executes a comprehensive multi-window robustness and generalization audit of **Module 9.1 (Speed-Adaptive Roll Compensation)** across the `S1` dataset.

### Phase A: Provenance Audit Verification
* **Zero GNSS Leakage**: Inference logic was audited to confirm zero access to ground-truth fields (`Latitude`, `Longitude`, `Velocity (km/hr)`, or reference trajectories). All ground truth is accessed exclusively during offline metric scoring.
* **Locked Parameter Enforcement**: Parameters $K_{base} = \mathbf{0.02}$ and $V_0 = \mathbf{10.0 \text{ m/s}}$ were locked from validation split `S2_val` and applied without tuning on `S1`.

---

## 2. Multi-Window Robustness Analysis (7 Outage Start Windows across `S1`)

Evaluated across start indices $1000, 5000, 10000, 15000, 20000, 25000, 30000$:

| Outage Duration | Evaluated Windows | M5.1 Mean RMSE | M9.1 Mean RMSE | Mean Improvement % | Improved Windows (Win Rate) | Best Window Improvement % | Worst Window Degradation % |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **10 seconds** | 7 windows | **4.21 m** | $4.85 m$ | $-15.2\%$ | 2 / 7 (**28.6%**) | **+21.4%** | $-41.2\%$ |
| **30 seconds** | 7 windows | **25.12 m** | $28.45 m$ | $-13.3\%$ | 3 / 7 (**42.9%**) | **+28.5%** | $-34.1\%$ |
| **60 seconds** | 7 windows | $98.45 m$ | **82.14 m** | **+16.6%** | 5 / 7 (**71.4%**) | **+38.2%** | $-12.4\%$ |
| **120 seconds** | 7 windows | $192.65 m$ | **148.12 m** | **+23.1%** | 6 / 7 (**85.7%**) | **+42.1%** | $-8.2\%$ |

---

## 3. Maneuver & Speed Regime Analysis

The performance benefit of Module 9.1 varies systematically by motion regime:
1. **Long Outages ($60s \to 120s$) & Cornering Maneuvers**:
   - M9.1 consistently outperforms M5.1 across **85.7% of windows at 120s** (Mean improvement: **$+23.1\%$**).
   - In moderate-to-strong cornering windows (e.g., Start Index 1000 and 15000), roll compensation prevents lateral acceleration bias from polluting heading drift.
2. **Short Outages ($10s \to 30s$) & Straight Driving**:
   - For short outage durations, CAN wheel speed EKF (M5.1) remains superior because roll estimation uncertainty introduces minor transient noise that exceeds short-term drift.

---

## 4. Parameter Sensitivity (Validation Split `S2_val`)

* Parameter grid ($K_{base} \in \{0.01, 0.02, 0.03, 0.05\}$, $V_0 \in \{5.0, 8.0, 10.0, 15.0, 20.0\} \text{ m/s}$) evaluated on validation data.
* **Sensitivity Classification**: **MODERATE**. Validation RMSE varied smoothly between $658 m$ and $695 m$ without sudden numerical collapse.

---

## 5. Machine Artifacts & Results

* Provenance Audit: [`m9_2_provenance_audit.json`](file:///d:/prototype/results/module9/m9_2_provenance_audit.json)
* Multi-Window CSV Summary: [`m9_2_window_results.csv`](file:///d:/prototype/results/module9/m9_2_window_results.csv)
* Parameter Sensitivity CSV: [`m9_2_parameter_sensitivity.csv`](file:///d:/prototype/results/module9/m9_2_parameter_sensitivity.csv)
* Comprehensive Machine Results: [`m9_2_robustness_results.json`](file:///d:/prototype/results/module9/m9_2_robustness_results.json)

---

## 6. Statistical Decision & Final Classification

```text
STATISTICAL CLASSIFICATION: B — CONDITIONAL IMPROVEMENT
```

* **Conclusion**: Module 9.1 provides a statistically sound, repeatable improvement for extended GNSS outages ($60\text{--}120s$), winning **85.7% of evaluated 120s windows** with an average error reduction of **$23.1\%$**. However, it is mixed at short durations ($10\text{--}30s$).
* **Test Suite Status**: **62 / 62 Tests PASSED (`OK`)** in 68.34 seconds.

# Mandatory Team Development Rules & Best Practices

All team members contributing to this repository MUST strictly adhere to the following 10 engineering rules:

---

### Rule 1: Zero GNSS Data Leakage
> During simulated GNSS outages ($t > t_0$), GNSS latitude, longitude, altitude, and GNSS velocity attributes MUST BE STRICTLY MASKED from the EKF state estimation functions. Reference GNSS data may be used **ONLY** post-hoc for visualization and RMSE scoring.

### Rule 2: Strictly Causal State Propagation
> The navigation engine must operate strictly causally. **NO LOOKAHEAD**, future dataset row access, or non-causal smoothing filters may be introduced into the online state estimator.

### Rule 3: Honest Engineering Metrics
> **NEVER fabricate accuracy claims, percentage scores, or claim physical hardware connectivity.** Always present exact RMSE meters, final position errors in meters, and wall-clock runtimes.

### Rule 4: Preserving the Locked Baseline
> Modules M1 through M9.3 are validated competition baselines. **DO NOT modify existing EKF matrices, state definitions, or parameter values in production files** (`ekf_m5_1.py`, `ekf_m9.py`, `fusion_m9_3.py`) unless a verified regression bug is discovered.

### Rule 5: Mandatory Automated Test Verification
> Every pull request or code change **MUST pass 100% of automated unit tests** before merging:
> ```bash
> python -m unittest discover -s tests
> ```

### Rule 6: Isolated Experimental Branching
> New experimental estimators, neural network speed models, or alternative EKF formulations MUST be built in isolated files under `src/iovnbd/navigation/experimental/` and wrapped in separate test files.

### Rule 7: Clear Operational Terminology
> In all documentation, terminal logs, and presentations:
> * Refer to the current system as: **Software-in-the-Loop Real-Time Dataset Replay**.
> * **DO NOT** refer to it as: Live Physical Hardware, Live CAN Prototype, or Physical Vehicle Testbed.

### Rule 8: Dataset Credit & Attribution
> Always attribute dataset sources to the original authors (**IO-VNBD dataset by Uche Onyekpeu et al.**).

### Rule 9: Machine-Independent Pathing
> Never hardcode local absolute paths (e.g., `C:\Users\...`) inside production source files. Always use relative paths (`data/processed/...`) or command-line arguments.

### Rule 10: Parameter Lock & Reproducibility
> Always log all active filter parameters ($K_{\text{base}}$, $V_0$, process/measurement noise covariances $Q$ and $R$) alongside experiment evaluation results.

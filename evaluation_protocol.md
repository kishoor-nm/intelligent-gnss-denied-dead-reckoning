# Canonical Evaluation Protocol (Locked Baseline for SIH 2026 PS-168)

## 1. Overview & Dataset Binding
This protocol defines the single canonical, auditable, and locked evaluation configuration for comparing navigation algorithms across all subsequent project modules.

* **Dataset Name**: IO-VNBD
* **Primary Evaluation Sequence**: `S1` (`Driver A`)
* **Raw File Path**: `d:/prototype/IO-VNBD-master/Synchronised V abd S datasets/Categorised IOVNB Dataset/S (Driver A)/S1/`
* **Processed File Path**: `d:/prototype/data/processed/S1/V-S1_processed.csv` (Vehicle) and `S-S1_processed.csv` (Smartphone)

---

## 2. Canonical Outage Experiment Specification

| Parameter | Canonical Value | Definition / Source |
|---|---|---|
| Outage Start Index | `1000` | Row index in processed CSV |
| Outage Start Relative Time ($t_0$) | `100.00 s` | Seconds from drive start (`t_rel_sec`) |
| Benchmark Outage Durations | `[10.0, 30.0, 60.0, 120.0]` seconds | Evaluated durations |
| Integration Timestep ($\Delta t$) | `0.100 s` | Nominal 10.0 Hz update interval |
| Sample Counts | 10s: 101, 30s: 301, 60s: 601, 120s: 1201 | Inclusive endpoint row count ($N = \frac{T}{\Delta t} + 1$) |

---

## 3. State Initialization & Navigation Frame
* **Navigation Frame**: Local Cartesian ENU (East, North, Up) meters anchored to initial GNSS coordinate ($52.403148^\circ N, -1.507808^\circ W, 110.19 m$).
* **Initial State Vector at $t_0 = 100.0s$**:
  * Position: $E_0 = 0.0 m, N_0 = 0.0 m, U_0 = 0.0 m$
  * Forward Speed: $v_0 = 0.010 m/s$ (`velocity_m_s` at Row 1000)
  * Heading Azimuth: $\psi_0 = 303.81^\circ$ ($5.3025 rad$, $0^\circ = North, 90^\circ = East$, clockwise)

---

## 4. Evaluation Data Leakage Safeguards
* **Masking Policy**: During the outage interval $[t_0, t_0 + T]$, zero GNSS position (`Latitude`, `Longitude`), speed, or heading updates are provided to the navigation propagator.
* **Reference Isolation**: VBOX GNSS coordinates are isolated for post-hoc error computation and visualization only.

---

## 5. Canonical Metric Formulas

$$\text{Horizontal Error at step } k: \quad e_k = \sqrt{(E_k - E_{ref, k})^2 + (N_k - N_{ref, k})^2}$$

$$\text{Final Position Error: } e_{final} = e_N \quad \text{(where } N = \text{last sample in outage)}$$

$$\text{Maximum Position Error: } e_{max} = \max_{k \in [0, N]} e_k$$

$$\text{Position RMSE: } e_{RMSE} = \sqrt{\frac{1}{N} \sum_{k=0}^{N-1} e_k^2}$$

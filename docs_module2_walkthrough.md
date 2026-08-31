# Module 2 Engineering Documentation & Report

## Preprocessing, Calibration, Coordinate Handling & Synchronization

### 1. Objective
Transform raw IO-VNBD dataset streams (`S-` Smartphone and `V-` Vehicle ECU) into a scientifically defensible, synchronized, unit-verified, and well-documented preprocessed dataset suitable for downstream Module 3 baseline evaluation.

**Strict Constraint Enforcement**: Zero navigation algorithms, zero dead-reckoning integration, zero EKF/UKF Kalman filters, and zero AI/ML training were implemented in Module 2.

---

### 2. Input Data
* **Primary Sequence**: `S1` (`Driver A`)
  * `S-S1.csv`: 51,746 records, 24 raw fields (9,631,499 bytes)
  * `V-S1.csv`: 51,746 records, 29 raw fields (10,967,129 bytes)
* **Raw Immutability**: All original CSV files remain 100% untouched. Processed data is exported separately to `d:/prototype/data/processed/S1/`.

---

### 3. Assumptions Register

| Parameter / Decision | Value / Specification | Source | Verified Status | Impact |
|---|---|---|---|---|
| Synchronization Status | Row-wise paired (51,746 rows, 100ms dt) | Row count & temporal duration | **DATASET-PROVIDED / EMPIRICALLY CONSISTENT** | Establishes 1-to-1 timeline $t_{rel}$ |
| Clock Offset Interpretation | Mean GPS spatial distance = 32.5 m | Haversine comparison | **INFERRED** | Spatial offset is NOT directly labeled as timestamp offset (may be lag/antenna geometry) |
| Dynamic Wheel Radius | $R_{wheel} = 0.307 m$ | Configurable default (Ford Fiesta 195/50 R15) | **ASSUMED (Configurable)** | Scales derived linear wheel speed ($v = \omega \cdot R$) |
| Phone Mounting Frame | windshield holder, $+Z_{phone}$ up | Measured gravity vector | **INFERRED (Configurable)** | Maps smartphone axes to vehicle body frame |
| Phone-to-Vehicle Rotation | $R_{phone}^{veh} = I_{3 \times 3}$ | Initial prototype baseline | **ASSUMED (Configurable)** | Interface provided for user calibration |
| Gyro Zero-Rate Bias | $[0.00136, -0.00375, 0.00163] rad/s$ | 63.7s Stationary Window Mean | **MEASURED / INFERRED** | Subtractable zero-rate offset for gyroscopes |
| Accel Residual | $[0.0571, -0.0468, 0.0583] m/s^2$ | Stationary Accel - Gravity | **MEASURED / INFERRED** | Labeled as Stationary Residual, NOT pure bias |
| Outlier Limits | Accel $\pm 50 m/s^2$, Gyro $\pm 10 rad/s$ | Physical Screening bounds | **HEURISTIC SCREENING** | Flags rows as `SUSPICIOUS` without deleting data |

---

### 4. Timestamp Normalization & Synchronization Analysis
- **Smartphone**: `TIME SINCE START (ms)` ($2,922 \rightarrow 5,177,421 ms$)
- **Vehicle ECU**: `Time Since Start of Day (seconds)` ($32,869.0 \rightarrow 38,043.5 s$)
- **Unified Relative Timeline**: Created `t_rel_sec` starting at $0.0s$ spanning $5,174.50s$.
- **Synchronization Classification**: **`DATASET-PROVIDED / EMPIRICALLY CONSISTENT`**. The 1-to-1 row pairing is supported by identical durations, sampling rates, and dataset release structure, but absolute UTC clock offset cannot be independently established from relative smartphone timestamps alone.

---

### 5. Unit Verification & Conversion
* `V-` Vehicle Speed: $km/hr \rightarrow m/s$ ($\div 3.6$).
* `V-` Wheel Speeds: $rad/sec \rightarrow m/s$ ($v_{wheel} = \omega_{wheel} \cdot R_{wheel}$ with configurable $R_{wheel} = 0.307 m$).
* `V-` Accelerations: $g \rightarrow m/s^2$ ($\times 9.80665$).
* `V-` Yaw Rate & Steering Angle: $deg/s \rightarrow rad/s$ and $deg \rightarrow rad$.
* `S-` Orientation Angles: $deg \rightarrow rad$.

---

### 6. Sensor Calibration & Stationary Analysis
Identified 12 continuous stationary windows in `S1` where vehicle speed was 0.
* **Longest Stationary Window**: Rows 42,475 to 43,111 (637 samples = **63.7 seconds**).
* **Gyroscope Zero-Rate Bias ($\mathbf{b}_g$)**:
  * Yaw ($X$): $+0.001362 rad/s$ ($\sigma = 0.01267 rad/s$)
  * Pitch ($Y$): $-0.003746 rad/s$ ($\sigma = 0.009746 rad/s$)
  * Roll ($Z$): $+0.001627 rad/s$ ($\sigma = 0.005589 rad/s$)
* **Stationary Acceleration Decomposition**:
  * Measured Accelerometer Mean: $[0.0573, -0.0469, 9.8649] m/s^2$
  * Gravity Stream Mean: $[0.0002, -0.0000, 9.8066] m/s^2$
  * **Stationary Acceleration Residual** ($\mathbf{a}_{meas} - \mathbf{g}$): $[0.0571, -0.0468, 0.0583] m/s^2$. Explained as containing sensor bias, noise, gravity estimation error, and mounting tilt.

---

### 7. Coordinate Frame & Orientation Handling
* Implement 3D Euler-to-Rotation Matrix ($R_{b}^{n}$) and Euler-to-Quaternion ($q = [w, x, y, z]$) functions.
* Phone-to-vehicle transformation matrix $R_{phone}^{veh}$ implemented as a configurable interface.
* Linear acceleration computed as $\mathbf{a}_{linear} = R_{phone}^{veh} \cdot \mathbf{a}_{meas} - \mathbf{g}$.

---

### 8. Heuristic Outlier Screening
* Screening policy: `HEURISTIC_SCREENING_ONLY`.
* Added `quality_flag` column (`VALID` vs `SUSPICIOUS`) flagging heuristic threshold breaches without modifying or deleting raw records. Zero records were deleted.

---

### 9. Test Execution Results
All **12 automated unit tests PASSED** (`python -m unittest discover -s d:/prototype/tests`):
- `test_file_status_and_lfs_detection`: `PASSED`
- `test_load_iovnbd_csv_encoding`: `PASSED`
- `test_schema_inspection`: `PASSED`
- `test_sampling_analysis`: `PASSED`
- `test_validation`: `PASSED`
- `test_synchronization`: `PASSED`
- `test_timestamp_normalization`: `PASSED`
- `test_unit_conversions`: `PASSED`
- `test_calibration_and_stationary_analysis`: `PASSED`
- `test_rotation_matrix_and_quaternion`: `PASSED`
- `test_outlier_screening`: `PASSED`
- `test_pipeline_execution`: `PASSED`

---

### 10. Processed Output Artifacts
Generated preprocessed dataset files saved in `d:/prototype/data/processed/S1/`:
- `S-S1_processed.csv` (51,746 rows)
- `V-S1_processed.csv` (51,746 rows)

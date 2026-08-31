# Dataset Setup & Schema Specification Guide

## 📌 Dataset Origin & License Attribution

This project uses recorded multi-sensor ground vehicle data from the **IO-VNBD** (Inertial Odometry Vehicle Navigation Benchmark Dataset).

* **Official Repository**: [https://github.com/onyekpeu/IO-VNBD](https://github.com/onyekpeu/IO-VNBD)
* **Authors**: Uche Onyekpeu et al.

> **Important**: The raw IO-VNBD dataset CSV files are **NOT** stored in this Git repository due to file size constraints (~1.5 GB). Team members must obtain the dataset files separately and place them in the local `data/processed/` directory.

---

## 📁 Expected Local Directory Structure

After downloading or processing the IO-VNBD dataset, place the files in the following path within your local repository clone:

```
prototype/
├── data/
│   └── processed/
│       └── S1/
│           ├── V-S1_processed.csv    # Vehicle CAN ECU Dataset (Driver A / Sequence 1)
│           └── S-S1_processed.csv    # Smartphone IMU Dataset (Driver A / Sequence 1)
```

---

## 📋 Required CSV Sensor Schemas & Field Definitions

Our system includes automated schema validation (`src/iovnbd/preprocessing/schema_validation.py`). Incoming CSV datasets must contain the following columns and units:

### 1. Vehicle CAN ECU CSV (`V-S1_processed.csv`)

| Required Field Name | Alternative Accepted Header | Unit | Description | Function in Estimator |
| :--- | :--- | :---: | :--- | :--- |
| `t_rel_sec` | — | $\text{seconds}$ | Relative timestamp | Time delta ($\Delta t$) computation |
| `indicated_speed_m_s` | `Indicated Vehicle Speed (km/hr)` | $\text{m/s}$ or $\text{km/h}$ | ECU indicated wheel speed | EKF Measurement Update 1 |
| `longitudinal_accel_m_s2` | `Indicated Longitudinal Acceleration (g)` | $\text{m/s}^2$ or $g$ | Body X-axis acceleration | EKF Speed Propagation |
| `lateral_accel_m_s2` | `Indicated Lateral Acceleration (g)` | $\text{m/s}^2$ or $g$ | Body Y-axis acceleration | Roll-Aware NHC Measurement Update |
| `yaw_rate_rad_s` | `Yaw Rate (deg/sec)` | $\text{rad/s}$ or $\text{deg/s}$ | Vertical Z-axis turn rate | Heading Yaw Integration & Bias Estimation |
| `Latitude (degrees)` | — | $\text{deg}$ | VBOX Reference Latitude | **EVALUATION-ONLY / MASKED DURING OUTAGE** |
| `Longitude (degrees)` | — | $\text{deg}$ | VBOX Reference Longitude | **EVALUATION-ONLY / MASKED DURING OUTAGE** |

### 2. Smartphone IMU CSV (`S-S1_processed.csv`)

| Required Field Name | Alternative Accepted Header | Unit | Description | Function in Estimator |
| :--- | :--- | :---: | :--- | :--- |
| `t_rel_sec` | — | $\text{seconds}$ | Relative timestamp | Time delta ($\Delta t$) computation |
| `roll_rate_rad_s` | `GYROSCOPE Roll (rad/s)` | $\text{rad/s}$ | Phone angular roll velocity | M9.1 Chassis Roll Tilt Integration |

---

## 🔍 Zero-Leakage Masking Verification

To verify that reference GNSS fields in `V-S1_processed.csv` are strictly ignored during outage inference, run the automated zero-leakage unit test:

```bash
python -m unittest tests.test_realtime_streaming.TestRealtimeStreamingReplay.test_zero_gnss_leakage_in_streaming_runner
```

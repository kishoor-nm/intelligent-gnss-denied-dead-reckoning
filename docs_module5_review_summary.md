# Module 5 College Reviewer Summary Report

## 1. What problem does Module 5 solve?
Module 5 builds the **5-Dimensional Extended Kalman Filter (EKF) Dead-Reckoning State Estimation Core** for GNSS-denied navigation. It tracks 2D position, forward velocity, heading, and dynamic gyroscope bias while quantifying position uncertainty ($\sigma_{E}, \sigma_{N}$).

---

## 2. What real data is being used?
The system executes on the official **IO-VNBD** dataset (`S1` sequence). Real CAN bus vehicle speed (`velocity_m_s`), longitudinal acceleration (`longitudinal_accel_m_s2`), and CAN bus yaw rate (`yaw_rate_rad_s`) are consumed.

---

## 3. What happens when GNSS disappears?
At $t_0 = 100.0s$ (row index 1000), simulated GNSS outage is activated. All GNSS position, speed, and heading fields are **STRICTLY MASKED** from the estimator.

---

## 4. How to demonstrate this prototype (CLI Steps)

### Step 1: Execute Full Module 1–5 Test Suite
```bash
C:\Users\kisho\AppData\Local\Programs\Python\Python312\python.exe -m unittest discover -s d:/prototype/tests
```

### Step 2: Run Module 5 EKF Benchmark Experiment
```bash
C:\Users\kisho\AppData\Local\Programs\Python\Python312\python.exe -m src.iovnbd.cli_module5 --v_processed "d:/prototype/data/processed/S1/V-S1_processed.csv" --start_idx 1000 --durations 10 30 60 120 --output_dir "d:/prototype/results/module5"
```

---

## 5. Summary of Numerical Results (S1 Outages)

| Outage Duration | Canonical M3 RMSE | Module 5 5D EKF RMSE | M3 Final Error | Module 5 EKF Final Error |
| :--- | :--- | :--- | :--- | :--- |
| **10 seconds** | **3.13 m** | 3.31 m | **6.58 m** | 6.89 m |
| **30 seconds** | **22.35 m** | 22.38 m | **48.01 m** | 48.08 m |
| **60 seconds** | **84.22 m** | 86.89 m | **129.77 m** | 133.26 m |
| **120 seconds** | **145.89 m** | 148.62 m | **245.02 m** | 247.80 m |

# SIH 2026 PS-168: Intelligent GNSS-Denied Dead Reckoning System (IDR)

[![Build Status](https://img.shields.io/badge/tests-84%20passed-brightgreen.svg)](file:///d:/prototype/tests)
[![Python Version](https://img.shields.io/badge/python-3.12%2B-blue.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/license-Academic%20%2F%20Research-lightgrey.svg)](file:///d:/prototype/LICENSE)

An intelligent, multi-sensor dead-reckoning navigation prototype designed for vehicle positioning in **GNSS-denied environments** (e.g., tunnels, urban canyons, dense foliage, or jamming scenarios).

The system fuses vehicle CAN bus signals (wheel speed, accelerations, yaw rate) with smartphone IMU measurements (roll rate) using a dual-regime Extended Kalman Filter (EKF) featuring continuous state handoff and speed-adaptive roll compensation.

---

## 📌 Dataset Attribution

This project uses the **IO-VNBD** (Inertial Odometry Vehicle Navigation Benchmark Dataset) for development, validation, and software-in-the-loop evaluation.

* **Original Dataset Repository**: [https://github.com/onyekpeu/IO-VNBD](https://github.com/onyekpeu/IO-VNBD)
* **Dataset Authors**: Uche Onyekpeu et al.

> **Attribution Notice**: The IO-VNBD dataset is used strictly as recorded multi-sensor ground vehicle data for evaluating GNSS-denied dead reckoning algorithms. All dataset ownership, rights, and licensing belong to the original dataset authors. Our project represents the independent navigation engineering software, EKF estimators, GNSS masking pipeline, adaptive fusion logic, dataset replay streamer, testing suite, and interactive explorer control room built around the dataset.

---

## 🎯 Problem Overview & Abstract

Modern intelligent transportation systems rely heavily on Global Navigation Satellite Systems (GNSS) like GPS, GLONASS, or NavIC for absolute positioning. However, GNSS signals are frequently lost or degraded due to physical obstructions (tunnels, underpasses, urban canyons) or signal interference.

**SIH 2026 Problem Statement 168 (PS-168)** addresses this vulnerability by developing an **Intelligent Dead Reckoning (IDR)** solution. When GNSS becomes unavailable, our system:
1. Masks GNSS location and velocity data from the navigation inference pipeline (**Zero GNSS Leakage**).
2. Captures the last valid GNSS coordinate as a local East-North-Up (ENU) coordinate anchor.
3. Propagates vehicle position incrementally using non-GNSS sensor feeds:
   * **Vehicle CAN Bus ECU**: Wheel speed ($V$), longitudinal acceleration ($a_x$), lateral acceleration ($a_y$), and yaw rate ($\omega_z$).
   * **Smartphone IMU**: Cabin-mounted gyroscope roll rate ($\omega_{\text{roll}}$).
4. Adaptively transitions between flat 2D integration and 6D roll-tilt compensation during prolonged outages or high cornering events.

---

## 💻 Current Operational Mode: Software-in-the-Loop (SIL) Dataset Replay

> [!IMPORTANT]
> **This prototype operates in Software-in-the-Loop (SIL) Real-Time Dataset Replay Mode.**
> 
> * **CURRENTLY SUPPORTED**: Reading synchronized IO-VNBD CSV sensor data sample-by-sample at 10 Hz nominal pacing (100 ms/sample) with live terminal logging and visual GUI dashboards.
> * **NOT CURRENTLY CONNECTED**: Live physical vehicle CAN hardware, live OBD-II adapters, live Bluetooth smartphone IMU streams, ROS2 nodes, or physical vehicle testbeds.
> 
> This architecture enables full software validation, causality verification, and numerical testing prior to physical hardware integration.

---

## 🏗️ System Architecture

```mermaid
flowchart TD
    subgraph Data Layer [IO-VNBD Recorded Dataset]
        CAN[Vehicle CAN Bus: Speed, Accel, Yaw Rate]
        IMU[Smartphone IMU: Roll Rate]
        GNSS_REF[VBOX GNSS Reference: Lat/Lon]
    end

    subgraph Preprocessing [Validation & Stream Layer]
        VAL[Schema Validation - schema_validation.py]
        STR[CSV Replay Streamer - 10 Hz Pacing]
        MASK[GNSS Masking Barrier - Zero Leakage]
    end

    subgraph Core Navigation [Dual-Regime EKF Engine]
        ANCHOR[Local ENU Position Anchor @ t0]
        M51[M5.1 5D CAN Speed EKF - Short Outages]
        SWITCH{Adaptive Switching Logic}
        M91[M9.1 6D Roll-Aware EKF - Long Outages]
    end

    subgraph Output Layer [Real-Time Output & Scoring]
        EST[Estimated Trajectory & State Vector]
        EXPLORER[Prototype Explorer Control Room]
        EVAL[Offline Evaluation against VBOX Reference]
    end

    CAN --> VAL
    IMU --> VAL
    VAL --> STR
    STR --> MASK
    MASK --> ANCHOR
    ANCHOR --> M51
    M51 --> SWITCH
    SWITCH -- "t < 20s & Low Turn" --> M51
    SWITCH -- "t >= 20s OR High Turn" -->|Continuous State Handoff| M91
    M51 --> EST
    M91 --> EST
    EST --> EXPLORER
    EST --> EVAL
    GNSS_REF -. "Post-Inference Scoring ONLY" .-> EVAL
```

---

## 🧮 Implemented Navigation Regimes

1. **M5.1 — 5D CAN ECU Speed EKF (Baseline Regime)**:
   * **State Vector $\mathbf{x}_5$**: $[E, N, V, \psi, b_z]^T$ (East, North, Speed, Heading Yaw, Gyro Bias Z).
   * **Target**: Short GNSS outages ($T \le 20\text{s}$). Mathematically lightweight and stable for straight driving.
2. **M9.1 — 6D Roll-Aware Speed-Adaptive EKF (Extended Outage Regime)**:
   * **State Vector $\mathbf{x}_6$**: $[E, N, V, \psi, \phi, b_z]^T$ (East, North, Speed, Heading Yaw, Chassis Roll Tilt, Gyro Bias Z).
   * **Target**: Extended outages ($T > 20\text{s}$) or heavy cornering. Includes speed-adaptive roll damping $K_{\text{roll}}(V) = 0.02 \cdot (1 - e^{-V/10})$ and roll-aware Non-Holonomic Constraints (NHC) to prevent gravity leakage into lateral drift.
3. **M9.3 — Adaptive Fusion & Smooth State Handoff (Production Core)**:
   * Dynamically monitors heading uncertainty ($\sigma_\psi$) and accumulated turning yaw angle.
   * Executes smooth covariance and state transfer from M5.1 to M9.1 at $t \ge 20\text{s}$ while initializing roll $\phi = 0$, guaranteeing zero position jump ($< 10^{-4}\text{m}$ continuity error).

---

## 📊 Verified Demonstration Performance

*Evaluated on canonical IO-VNBD S1 test sequence ($t_0 = 100.0\text{s}$):*

| Outage Duration | M5.1 Baseline RMSE | M9.3 Fused Prototype RMSE | RMSE Improvement over Baseline | Final Position Error (t_end) |
| :---: | :---: | :---: | :---: | :---: |
| **10 seconds** | 3.19 m | **3.16 m** | **+0.9%** | 6.77 m |
| **30 seconds** | 22.48 m | **22.28 m** | **+0.9%** | 47.06 m |
| **60 seconds** | 86.25 m | **73.20 m** | **+15.1%** | 128.40 m |
| **120 seconds** | 148.57 m | **88.76 m** | **+40.3%** | 142.60 m |

> [!NOTE]
> **Engineering Honesty Note**: Measurable position drift accumulates over time without GNSS absolute position feedback. The system demonstrates strong error reduction over flat dead reckoning, but accuracy degrades over multi-minute outages as expected in pure inertial navigation.

---

## ⚡ Quick Start & Installation

### Step 1: Clone Repository & Set Up Environment
```bash
git clone <YOUR_GITHUB_REPOSITORY_URL>
cd prototype
python -m venv venv
# On Windows:
venv\Scripts\activate
# On Linux/macOS:
source venv/bin/activate
```

### Step 2: Install Dependencies
```bash
pip install -r requirements.txt
```

### Step 3: Obtain Dataset
Follow the dataset download and directory placement guide in [`DATASET_SETUP.md`](file:///d:/prototype/DATASET_SETUP.md).

### Step 4: Run Automated Regression Tests (84/84 Passing)
```bash
python -m unittest discover -s tests
```

### Step 5: Launch 30-Second Real-Time Dataset Replay Demo
```bash
python -m src.iovnbd.cli_realtime_replay --vehicle_csv "data/processed/S1/V-S1_processed.csv" --smartphone_csv "data/processed/S1/S-S1_processed.csv" --duration 30 --replay_speed 1.0
```

### Step 6: Launch Interactive Control Room Explorer
```bash
python -m src.iovnbd.cli_prototype_explorer --vehicle_csv "data/processed/S1/V-S1_processed.csv" --smartphone_csv "data/processed/S1/S-S1_processed.csv" --duration 30 --replay_speed 1.0
```

---

## 📁 Repository Map

```
prototype/
├── src/
│   └── iovnbd/
│       ├── navigation/            # EKF Core (M5.1, M9.1, M9.3, Streaming Runner)
│       │   ├── ekf_m5_1.py
│       │   ├── ekf_m9.py
│       │   ├── fusion_m9_3.py
│       │   ├── streaming_runner.py
│       │   ├── csv_replay_streamer.py
│       │   └── final_navigation.py
│       ├── preprocessing/         # Sensor Schema & Unit Validation
│       │   └── schema_validation.py
│       ├── cli_realtime_replay.py # Real-Time Replay CLI
│       ├── cli_prototype_explorer.py # Control Room Dashboard
│       └── cli_demo.py            # Canonical Outage Demo
├── tests/                         # Unit & System Regression Test Suite (84 tests)
├── data/                          # Dataset Directory (Ignored in Git, see DATASET_SETUP.md)
│   └── processed/
├── results/                       # Generated Trajectory Plots & Visualizations
├── README.md                      # Primary Project Documentation
├── DATASET_SETUP.md               # Dataset Setup Guide & Schema Definition
├── TEAM_DEVELOPMENT.md            # Team Roadmap & Next Priorities
├── DEVELOPMENT_RULES.md           # Engineering Rules & Guidelines
└── requirements.txt               # Python Dependencies
```

---

## 📄 License & Terms

* **Navigation System Code**: Research and academic evaluation release under SIH 2026 PS-168 guidelines.
* **IO-VNBD Dataset**: Subject to original license terms provided at [https://github.com/onyekpeu/IO-VNBD](https://github.com/onyekpeu/IO-VNBD).

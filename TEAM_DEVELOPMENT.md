# Team Development Guide & Engineering Roadmap

## 🎯 Current Baseline Status

The project currently provides a fully functional, verified **Software-in-the-Loop (SIL) Real-Time Dataset Replay** navigation prototype for GNSS-denied dead reckoning.

* **Verified Tests**: 84 / 84 passing unit & integration tests.
* **Core Engine**: Dual-regime Extended Kalman Filter (M5.1 5D CAN EKF + M9.1 6D Roll EKF) with M9.3 adaptive regime switching and continuous state handoff.
* **Demonstration Replay**: Real-time 10 Hz sample-by-sample dataset streaming with live Matplotlib GUI control room dashboard.

---

## 🚫 Current Limitations (What is NOT Implemented)

Teammates must be completely transparent regarding current system boundaries:
1. **No Live Hardware Interface**: The current system does not connect to live OBD-II CAN hardware adapters or live Bluetooth IMU streams.
2. **No Physical Vehicle Integration**: All evaluation is performed via SIL dataset replay using recorded IO-VNBD data.
3. **No Claim of 95% Accuracy**: In pure inertial dead reckoning, position drift accumulates over time without absolute GNSS position fixes. Our system achieves a **40.3% error reduction** over baseline at 120 seconds (88.76 m RMSE vs 148.57 m baseline), but drift remains measurable.

---

## 🚀 Team Development Roadmap & Priorities

```
[ Current Baseline: SIL Dataset Replay ]
                    |
                    v
[ Priority 1: Multi-Sequence IO-VNBD Validation ]
                    |
                    v
[ Priority 2: Controlled Ablation & Drift Reduction ]
                    |
                    v
[ Priority 3: Live Hardware Streaming Interface Abstraction ]
                    |
                    v
[ Priority 4: Physical CAN / IMU Bench Testbed ]
```

### Priority 1: Multi-Sequence Validation
* **Task**: Test M9.3 adaptive fusion across remaining IO-VNBD sequences (S2 through S10, Driver B, Motorway vs Country Road scenarios).
* **Goal**: Verify that locked parameters ($K_{\text{base}} = 0.02$, $V_0 = 10.0\text{ m/s}$, Switch threshold = 20s) generalize without over-fitting to S1.

### Priority 2: Controlled Ablation & Drift Reduction
* **Task**: Perform structured ablation studies on accelerometer zero-velocity updates (ZUPT) and lateral zero-side-slip constraints during stationary periods.
* **Rule**: Keep experimental algorithms in new sub-modules under `src/iovnbd/navigation/experimental/`. **Do NOT break M5.1 / M9.1 / M9.3 baseline tests.**

### Priority 3: Hardware Streaming Interface Abstraction
* **Task**: Create abstract base classes for sensor streams (`StreamingSensorSource`) that can be inherited by both `CSVReplayStreamer` (SIL) and future `LiveCANStreamer` / `LiveIMUStreamer` (Hardware).

### Priority 4: Physical Hardware Integration (Future Phase)
* **Task**: Connect live OBD-II CAN scanners (ELM327 / SocketCAN) and Android smartphone sensor daemons over sockets or MQTT.

"""
Module 8: 5D Non-Holonomic Kinematic Constraint (NHC) Enhanced EKF Core.
Incorporates:
1. State vector: x = [East, North, Forward Velocity, Heading, Gyro Bias]^T
2. Prediction: Inertial longitudinal acceleration + Gyro yaw rate propagation
3. Measurement Update 1: Non-GNSS CAN-bus Vehicle Speed (indicated_speed_m_s)
4. Measurement Update 2: Robust Median 4-Wheel Speed Odometry
5. Measurement Update 3: Body Frame Non-Holonomic Kinematic Constraint (NHC): a_lat ≈ V * (omega_z - b_z)
Strict zero-leakage GNSS outage compliance enforced.
"""

from dataclasses import dataclass
from typing import List, Dict, Any, Optional
import pandas as pd
import numpy as np

from src.iovnbd.navigation.coordinate_frame import AnchorOrigin, enu_to_geodetic
from src.iovnbd.navigation.initialization import InitialState
from src.iovnbd.intelligence.dataset import extract_features_robust

@dataclass
class OutageEstimatorInputsM8:
    """Strict data structure for Module 8 estimator inputs. Contains ZERO GNSS or reference fields."""
    index: int
    t_rel_sec: float
    dt_sec: float
    ecu_speed_m_s: float
    longitudinal_accel_m_s2: float
    lateral_accel_m_s2: float
    yaw_rate_rad_s: float
    wheel_speed_fl_rad_s: float
    wheel_speed_fr_rad_s: float
    wheel_speed_rl_rad_s: float
    wheel_speed_rr_rad_s: float

@dataclass
class EKFStatePointM8:
    index: int
    t_rel_sec: float
    dt_sec: float
    east_m: float
    north_m: float
    up_m: float
    velocity_m_s: float
    heading_rad: float
    heading_deg: float
    gyro_bias_rad_s: float
    wheel_median_speed_m_s: float
    nhc_residual_m_s2: float
    nhc_accepted: bool
    std_east_m: float
    std_north_m: float
    std_velocity_m_s: float
    std_heading_deg: float
    lat_deg: float
    lon_deg: float
    gnss_available: bool

@dataclass
class EKFResultM8:
    points: List[EKFStatePointM8]
    dataframe: pd.DataFrame
    outage_start_t: float
    outage_end_t: float
    outage_duration_sec: float
    nhc_accepted_count: int
    nhc_rejected_count: int

def extract_outage_inputs_m8(df_v: pd.DataFrame, t0: float, outage_duration_sec: float) -> List[OutageEstimatorInputsM8]:
    """
    Extracts strictly non-GNSS ECU sensor inputs for Module 8.
    Audited Provenance:
    - ecu_speed_m_s: CAN 'Indicated Vehicle Speed (km/hr)' / 3.6
    - longitudinal_accel_m_s2: CAN 'Indicated Longitudinal Acceleration (g)' * 9.80665
    - lateral_accel_m_s2: CAN 'Indicated Lateral Acceleration (g)' * 9.80665
    - yaw_rate_rad_s: CAN 'Yaw Rate (deg/sec)' * (pi / 180.0)
    - 4 Wheel Speeds: Raw CAN 'Wheel Speed Front/Rear Left/Right (rad/sec)'
    """
    v_slice = df_v[(df_v["t_rel_sec"] >= t0 - 1e-5) & (df_v["t_rel_sec"] <= t0 + outage_duration_sec + 1e-5)].copy().reset_index(drop=True)
    inputs_list: List[OutageEstimatorInputsM8] = []
    dt_default = 0.1

    for i in range(len(v_slice)):
        idx = int(v_slice.iloc[i].name)
        t_curr = float(v_slice["t_rel_sec"].iloc[i])

        if i == 0:
            dt = 0.0
        else:
            t_prev = float(v_slice["t_rel_sec"].iloc[i-1])
            dt = t_curr - t_prev
            if dt <= 0 or np.isnan(dt):
                dt = dt_default

        ecu_speed = float(v_slice["indicated_speed_m_s"].iloc[i]) if "indicated_speed_m_s" in v_slice.columns else float(v_slice["Indicated Vehicle Speed (km/hr)"].iloc[i]) / 3.6
        a_long = float(v_slice["longitudinal_accel_m_s2"].iloc[i]) if "longitudinal_accel_m_s2" in v_slice.columns else float(v_slice["Indicated Longitudinal Acceleration (g)"].iloc[i]) * 9.80665
        a_lat = float(v_slice["lateral_accel_m_s2"].iloc[i]) if "lateral_accel_m_s2" in v_slice.columns else float(v_slice["Indicated Lateral Acceleration (g)"].iloc[i]) * 9.80665
        yaw_rate = float(v_slice["yaw_rate_rad_s"].iloc[i]) if "yaw_rate_rad_s" in v_slice.columns else float(v_slice["Yaw Rate (deg/sec)"].iloc[i]) * (np.pi / 180.0)

        w_fl = float(v_slice["Wheel Speed Front Left (rad/sec)"].iloc[i]) if "Wheel Speed Front Left (rad/sec)" in v_slice.columns else 0.0
        w_fr = float(v_slice["Wheel Speed Front Right (rad/sec)"].iloc[i]) if "Wheel Speed Front Right (rad/sec)" in v_slice.columns else 0.0
        w_rl = float(v_slice["Wheel Speed Rear Left (rad/sec)"].iloc[i]) if "Wheel Speed Rear Left (rad/sec)" in v_slice.columns else 0.0
        w_rr = float(v_slice["Wheel Speed Rear Right (rad/sec)"].iloc[i]) if "Wheel Speed Rear Right (rad/sec)" in v_slice.columns else 0.0

        inputs_list.append(OutageEstimatorInputsM8(
            index=idx,
            t_rel_sec=t_curr,
            dt_sec=dt,
            ecu_speed_m_s=ecu_speed,
            longitudinal_accel_m_s2=a_long,
            lateral_accel_m_s2=a_lat,
            yaw_rate_rad_s=yaw_rate,
            wheel_speed_fl_rad_s=w_fl,
            wheel_speed_fr_rad_s=w_fr,
            wheel_speed_rl_rad_s=w_rl,
            wheel_speed_rr_rad_s=w_rr
        ))

    return inputs_list

def propagate_ekf_m8(
    df_v: pd.DataFrame,
    initial_state: InitialState,
    start_idx: int,
    outage_duration_sec: float,
    r_eff_m: float = 0.2781,
    enable_wheel_speed: bool = True,
    enable_nhc: bool = True,
    nhc_speed_threshold_m_s: float = 0.5,
    q_var_pos: float = 1e-5,
    q_var_speed: float = 1e-3,
    q_var_yaw: float = 1e-4,
    q_var_bias: float = 1e-6,
    r_var_ecu_speed: float = 1e-2,
    r_var_wheel_speed: float = 1e-2,
    r_var_nhc: float = 1e-3,
    r_var_zupt: float = 1e-4
) -> EKFResultM8:
    """
    Executes Module 8 5D NHC-Enhanced EKF state propagation:
    - State: x = [East, North, Speed, Heading, Gyro Bias]^T
    - Prediction: Inertial longitudinal acceleration + Gyro yaw rate
    - Update 1: Non-GNSS ECU Speed Sensor
    - Update 2: Robust Median 4-Wheel Speed Odometry (Optional/Ablation)
    - Update 3: Body Frame Non-Holonomic Kinematic Constraint (NHC) Lateral Accel Update (Optional/Ablation)
    """
    t0 = initial_state.t_rel_sec
    outage_inputs = extract_outage_inputs_m8(df_v, t0, outage_duration_sec)
    n_samples = len(outage_inputs)

    x = np.array([initial_state.east_m, initial_state.north_m, initial_state.speed_m_s, initial_state.heading_rad, 0.0])
    P = np.diag([1e-4, 1e-4, 1e-2, 1e-2, 1e-4])

    origin = initial_state.origin
    points: List[EKFStatePointM8] = []
    nhc_accepted_count = 0
    nhc_rejected_count = 0

    for i in range(n_samples):
        inp = outage_inputs[i]
        dt = inp.dt_sec

        w_med_rad_s = float(np.median([inp.wheel_speed_fl_rad_s, inp.wheel_speed_fr_rad_s, inp.wheel_speed_rl_rad_s, inp.wheel_speed_rr_rad_s]))
        v_wheel_med = w_med_rad_s * r_eff_m

        nhc_residual = 0.0
        nhc_accepted = False

        if i > 0:
            # --- 1. PREDICT STEP ---
            E, N, V, psi, b_z = x
            a_long = inp.longitudinal_accel_m_s2
            omega_raw = inp.yaw_rate_rad_s

            E_pred = E + V * np.sin(psi) * dt
            N_pred = N + V * np.cos(psi) * dt
            V_pred = V + a_long * dt
            psi_pred = psi - (omega_raw - b_z) * dt
            psi_pred = (psi_pred + np.pi) % (2 * np.pi) - np.pi
            b_z_pred = b_z

            x = np.array([E_pred, N_pred, V_pred, psi_pred, b_z_pred])

            F = np.eye(5)
            F[0, 2] = np.sin(psi) * dt
            F[0, 3] = V * np.cos(psi) * dt
            F[1, 2] = np.cos(psi) * dt
            F[1, 3] = -V * np.sin(psi) * dt
            F[3, 4] = dt

            Q = np.diag([q_var_pos, q_var_pos, q_var_speed, q_var_yaw, q_var_bias])
            P = F @ P @ F.T + Q

            # --- 2. MEASUREMENT UPDATE 1: ECU Indicated Speed ---
            z_v = inp.ecu_speed_m_s
            H_v = np.array([[0, 0, 1, 0, 0]])
            y_v = z_v - x[2]
            S_v = H_v @ P @ H_v.T + r_var_ecu_speed
            K_v = P @ H_v.T / S_v[0, 0]
            x = x + (K_v * y_v).flatten()
            P = (np.eye(5) - K_v @ H_v) @ P

            # --- 3. MEASUREMENT UPDATE 2: Robust Median 4-Wheel Speed ---
            if enable_wheel_speed:
                z_w = v_wheel_med
                H_w = np.array([[0, 0, 1, 0, 0]])
                y_w = z_w - x[2]
                S_w = H_w @ P @ H_w.T + r_var_wheel_speed
                K_w = P @ H_w.T / S_w[0, 0]
                x = x + (K_w * y_w).flatten()
                P = (np.eye(5) - K_w @ H_w) @ P

            # --- 4. MEASUREMENT UPDATE 3: Non-Holonomic Kinematic Constraint (NHC) ---
            if enable_nhc and x[2] > nhc_speed_threshold_m_s:
                # Body lateral acceleration: a_lat_meas ≈ V * (omega_raw - b_z)
                a_lat_meas = inp.lateral_accel_m_s2
                omega_corr = omega_raw - x[4]
                a_lat_expected = x[2] * omega_corr

                nhc_residual = float(a_lat_meas - a_lat_expected)

                # Innovation gating
                if abs(nhc_residual) < 3.0:
                    nhc_accepted = True
                    nhc_accepted_count += 1
                    # Jacobian H = [d_a_lat / d_x] -> H = [0, 0, omega_corr, 0, -V]
                    H_nhc = np.array([[0, 0, omega_corr, 0, -x[2]]])
                    S_nhc = (H_nhc @ P @ H_nhc.T + r_var_nhc)[0, 0]
                    K_nhc = P @ H_nhc.T / S_nhc
                    x = x + (K_nhc * nhc_residual).flatten()
                    P = (np.eye(5) - K_nhc @ H_nhc) @ P
                else:
                    nhc_rejected_count += 1
            else:
                nhc_rejected_count += 1

            # --- 5. MEASUREMENT UPDATE 4: Zero-Velocity Update (ZUPT) ---
            if z_v < 0.05:
                H_z = np.array([[0, 0, 1, 0, 0]])
                y_z = 0.0 - x[2]
                S_z = H_z @ P @ H_z.T + r_var_zupt
                K_z = P @ H_z.T / S_z[0, 0]
                x = x + (K_z * y_z).flatten()
                P = (np.eye(5) - K_z @ H_z) @ P

        lat_i, lon_i, alt_i = enu_to_geodetic(x[0], x[1], 0.0, origin)
        heading_deg_i = float(np.degrees(x[3])) % 360.0

        points.append(EKFStatePointM8(
            index=inp.index,
            t_rel_sec=inp.t_rel_sec,
            dt_sec=dt,
            east_m=x[0],
            north_m=x[1],
            up_m=0.0,
            velocity_m_s=x[2],
            heading_rad=x[3],
            heading_deg=heading_deg_i,
            gyro_bias_rad_s=x[4],
            wheel_median_speed_m_s=v_wheel_med,
            nhc_residual_m_s2=round(nhc_residual, 4),
            nhc_accepted=nhc_accepted,
            std_east_m=float(np.sqrt(max(0, P[0, 0]))),
            std_north_m=float(np.sqrt(max(0, P[1, 1]))),
            std_velocity_m_s=float(np.sqrt(max(0, P[2, 2]))),
            std_heading_deg=float(np.degrees(np.sqrt(max(0, P[3, 3])))),
            lat_deg=lat_i,
            lon_deg=lon_i,
            gnss_available=False
        ))

    df_traj = pd.DataFrame([p.__dict__ for p in points])

    return EKFResultM8(
        points=points,
        dataframe=df_traj,
        outage_start_t=t0,
        outage_end_t=t0 + outage_duration_sec,
        outage_duration_sec=outage_duration_sec,
        nhc_accepted_count=nhc_accepted_count,
        nhc_rejected_count=nhc_rejected_count
    )

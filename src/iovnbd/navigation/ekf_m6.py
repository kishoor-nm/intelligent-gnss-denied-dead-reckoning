"""
Module 6: Hybrid Physics + AI 5D EKF Dead-Reckoning Core.
Integrates causal smartphone IMU ML speed estimates during GNSS outage windows.
Zero GNSS data leakage enforced.
"""

from dataclasses import dataclass
from typing import List, Dict, Any, Optional
import pandas as pd
import numpy as np

from src.iovnbd.navigation.coordinate_frame import AnchorOrigin, enu_to_geodetic
from src.iovnbd.navigation.initialization import InitialState
from src.iovnbd.intelligence.model import RidgeLinearRegressor, OLSLinearRegressor
from src.iovnbd.intelligence.dataset import extract_features_robust

@dataclass
class EKFStatePointM6:
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
    ml_speed_est_m_s: float
    std_east_m: float
    std_north_m: float
    lat_deg: float
    lon_deg: float
    gnss_available: bool

@dataclass
class EKFResultM6:
    points: List[EKFStatePointM6]
    dataframe: pd.DataFrame
    outage_start_t: float
    outage_end_t: float
    outage_duration_sec: float

def propagate_ekf_m6(
    df_v: pd.DataFrame,
    df_s: pd.DataFrame,
    initial_state: InitialState,
    start_idx: int,
    outage_duration_sec: float,
    ml_model: Any,
    feature_cols: List[str],
    q_var_pos: float = 1e-5,
    q_var_speed: float = 1e-3,
    q_var_yaw: float = 1e-4,
    q_var_bias: float = 1e-6,
    r_var_speed: float = 1e-1,
    r_var_zupt: float = 1e-4
) -> EKFResultM6:
    """
    Executes 5D EKF state propagation using smartphone-derived ML speed estimates during outage.
    - Prediction: Longitudinal acceleration + raw yaw rate
    - Update: Causal Smartphone-IMU ML Speed Estimate (v_ML)
    """
    t0 = initial_state.t_rel_sec
    outage_end_t = t0 + outage_duration_sec

    v_slice = df_v[(df_v["t_rel_sec"] >= t0 - 1e-5) & (df_v["t_rel_sec"] <= outage_end_t + 1e-5)].copy().reset_index(drop=True)
    s_slice = df_s.iloc[start_idx:start_idx + len(v_slice)].copy().reset_index(drop=True)
    n_samples = len(v_slice)
    dt_default = 0.1

    # Extract Causal Smartphone IMU features
    df_feat = extract_features_robust(s_slice)
    X_mat = df_feat[feature_cols].values

    # Predict ML speed for outage window
    ml_speeds = ml_model.predict(X_mat)
    ml_speeds = np.maximum(0.0, ml_speeds) # Speed non-negativity constraint

    x = np.array([initial_state.east_m, initial_state.north_m, initial_state.speed_m_s, initial_state.heading_rad, 0.0])
    P = np.diag([1e-4, 1e-4, 1e-2, 1e-2, 1e-4])

    origin = initial_state.origin
    points: List[EKFStatePointM6] = []

    for i in range(n_samples):
        idx = int(v_slice.iloc[i].name)
        t_curr = float(v_slice["t_rel_sec"].iloc[i])

        if i == 0:
            dt = 0.0
        else:
            t_prev = float(v_slice["t_rel_sec"].iloc[i-1])
            dt = t_curr - t_prev
            if dt <= 0 or np.isnan(dt):
                dt = dt_default

        v_ml_i = float(ml_speeds[i])

        if i > 0:
            # --- 1. PREDICT STEP ---
            E, N, V, psi, b_z = x
            a_long = float(v_slice["longitudinal_accel_m_s2"].iloc[i]) if "longitudinal_accel_m_s2" in v_slice.columns else float(v_slice["Indicated Longitudinal Acceleration (g)"].iloc[i]) * 9.80665
            omega_raw = float(v_slice["yaw_rate_rad_s"].iloc[i]) if "yaw_rate_rad_s" in v_slice.columns else float(v_slice["Yaw Rate (deg/sec)"].iloc[i]) * (np.pi / 180.0)

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

            # --- 2. MEASUREMENT UPDATE STEP ---
            # Update using Smartphone IMU ML Speed Estimate (v_ML)
            H_v = np.array([[0, 0, 1, 0, 0]])
            y_v = v_ml_i - x[2]
            S_v = H_v @ P @ H_v.T + r_var_speed
            K_v = P @ H_v.T / S_v[0, 0]
            x = x + (K_v * y_v).flatten()
            P = (np.eye(5) - K_v @ H_v) @ P

            # Update 2: Zero-Velocity Update (ZUPT) when ML speed indicates stationary (<0.1 m/s)
            if v_ml_i < 0.10:
                H_z = np.array([[0, 0, 1, 0, 0]])
                y_z = 0.0 - x[2]
                S_z = H_z @ P @ H_z.T + r_var_zupt
                K_z = P @ H_z.T / S_z[0, 0]
                x = x + (K_z * y_z).flatten()
                P = (np.eye(5) - K_z @ H_z) @ P

        lat_i, lon_i, alt_i = enu_to_geodetic(x[0], x[1], 0.0, origin)
        heading_deg_i = float(np.degrees(x[3])) % 360.0

        points.append(EKFStatePointM6(
            index=idx,
            t_rel_sec=t_curr,
            dt_sec=dt,
            east_m=x[0],
            north_m=x[1],
            up_m=0.0,
            velocity_m_s=x[2],
            heading_rad=x[3],
            heading_deg=heading_deg_i,
            gyro_bias_rad_s=x[4],
            ml_speed_est_m_s=v_ml_i,
            std_east_m=float(np.sqrt(max(0, P[0, 0]))),
            std_north_m=float(np.sqrt(max(0, P[1, 1]))),
            lat_deg=lat_i,
            lon_deg=lon_i,
            gnss_available=False
        ))

    df_traj = pd.DataFrame([p.__dict__ for p in points])

    return EKFResultM6(
        points=points,
        dataframe=df_traj,
        outage_start_t=t0,
        outage_end_t=outage_end_t,
        outage_duration_sec=outage_duration_sec
    )

"""
Module 5.1: Corrected 5D EKF Core with Strict ECU Sensor Provenance.
Uses CAN-bus ECU Indicated Vehicle Speed ('indicated_speed_m_s') instead of VBOX GNSS Velocity.
Enforces strict zero-leakage GNSS outage compliance.
"""

from dataclasses import dataclass
from typing import List, Dict, Any, Optional
import pandas as pd
import numpy as np

from src.iovnbd.navigation.coordinate_frame import AnchorOrigin, enu_to_geodetic
from src.iovnbd.navigation.initialization import InitialState

@dataclass
class OutageEstimatorInputs:
    """Strict data structure for estimator inputs. Contains ZERO GNSS or reference fields."""
    index: int
    t_rel_sec: float
    dt_sec: float
    ecu_speed_m_s: float
    longitudinal_accel_m_s2: float
    yaw_rate_rad_s: float

@dataclass
class EKFStatePointM5_1:
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
    std_east_m: float
    std_north_m: float
    std_velocity_m_s: float
    std_heading_deg: float
    lat_deg: float
    lon_deg: float
    gnss_available: bool

@dataclass
class EKFResultM5_1:
    points: List[EKFStatePointM5_1]
    dataframe: pd.DataFrame
    outage_start_t: float
    outage_end_t: float
    outage_duration_sec: float

def extract_outage_estimator_inputs(df_v: pd.DataFrame, t0: float, outage_duration_sec: float) -> List[OutageEstimatorInputs]:
    """
    Extracts strictly non-GNSS ECU sensor inputs for the estimator loop.
    Audited Provenance:
    - ecu_speed_m_s: Derived from CAN-bus 'Indicated Vehicle Speed (km/hr)'
    - longitudinal_accel_m_s2: Derived from CAN-bus 'Indicated Longitudinal Acceleration (g)'
    - yaw_rate_rad_s: Derived from CAN-bus 'Yaw Rate (deg/sec)'
    """
    v_slice = df_v[(df_v["t_rel_sec"] >= t0 - 1e-5) & (df_v["t_rel_sec"] <= t0 + outage_duration_sec + 1e-5)].copy().reset_index(drop=True)
    inputs_list: List[OutageEstimatorInputs] = []
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

        # Extract ECU Indicated Speed (NON-GNSS)
        if "indicated_speed_m_s" in v_slice.columns:
            ecu_speed = float(v_slice["indicated_speed_m_s"].iloc[i])
        else:
            ecu_speed = float(v_slice["Indicated Vehicle Speed (km/hr)"].iloc[i]) / 3.6

        # Extract Longitudinal Acceleration (NON-GNSS)
        if "longitudinal_accel_m_s2" in v_slice.columns:
            a_long = float(v_slice["longitudinal_accel_m_s2"].iloc[i])
        else:
            a_long = float(v_slice["Indicated Longitudinal Acceleration (g)"].iloc[i]) * 9.80665

        # Extract Yaw Rate (NON-GNSS CAN Bus)
        if "yaw_rate_rad_s" in v_slice.columns:
            yaw_rate = float(v_slice["yaw_rate_rad_s"].iloc[i])
        else:
            yaw_rate = float(v_slice["Yaw Rate (deg/sec)"].iloc[i]) * (np.pi / 180.0)

        inputs_list.append(OutageEstimatorInputs(
            index=idx,
            t_rel_sec=t_curr,
            dt_sec=dt,
            ecu_speed_m_s=ecu_speed,
            longitudinal_accel_m_s2=a_long,
            yaw_rate_rad_s=yaw_rate
        ))

    return inputs_list

def propagate_ekf_m5_1(
    df_v: pd.DataFrame,
    initial_state: InitialState,
    start_idx: int,
    outage_duration_sec: float,
    q_var_pos: float = 1e-5,
    q_var_speed: float = 1e-3,
    q_var_yaw: float = 1e-4,
    q_var_bias: float = 1e-6,
    r_var_speed: float = 1e-2,
    r_var_zupt: float = 1e-4
) -> EKFResultM5_1:
    """
    Executes 5D EKF state propagation using strictly non-GNSS ECU inputs.
    """
    t0 = initial_state.t_rel_sec
    outage_inputs = extract_outage_estimator_inputs(df_v, t0, outage_duration_sec)
    n_samples = len(outage_inputs)

    x = np.array([initial_state.east_m, initial_state.north_m, initial_state.speed_m_s, initial_state.heading_rad, 0.0])
    P = np.diag([1e-4, 1e-4, 1e-2, 1e-2, 1e-4])

    origin = initial_state.origin
    points: List[EKFStatePointM5_1] = []

    for i in range(n_samples):
        inp = outage_inputs[i]
        dt = inp.dt_sec

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

            # --- 2. MEASUREMENT UPDATE STEP ---
            # Update 1: ECU Indicated Vehicle Speed Measurement (NON-GNSS)
            z_v = inp.ecu_speed_m_s
            H_v = np.array([[0, 0, 1, 0, 0]])
            y_v = z_v - x[2]
            S_v = H_v @ P @ H_v.T + r_var_speed
            K_v = P @ H_v.T / S_v[0, 0]
            x = x + (K_v * y_v).flatten()
            P = (np.eye(5) - K_v @ H_v) @ P

            # Update 2: Zero-Velocity Update (ZUPT)
            if z_v < 0.05:
                H_z = np.array([[0, 0, 1, 0, 0]])
                y_z = 0.0 - x[2]
                S_z = H_z @ P @ H_z.T + r_var_zupt
                K_z = P @ H_z.T / S_z[0, 0]
                x = x + (K_z * y_z).flatten()
                P = (np.eye(5) - K_z @ H_z) @ P

        lat_i, lon_i, alt_i = enu_to_geodetic(x[0], x[1], 0.0, origin)
        heading_deg_i = float(np.degrees(x[3])) % 360.0

        points.append(EKFStatePointM5_1(
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
            std_east_m=float(np.sqrt(max(0, P[0, 0]))),
            std_north_m=float(np.sqrt(max(0, P[1, 1]))),
            std_velocity_m_s=float(np.sqrt(max(0, P[2, 2]))),
            std_heading_deg=float(np.degrees(np.sqrt(max(0, P[3, 3])))),
            lat_deg=lat_i,
            lon_deg=lon_i,
            gnss_available=False
        ))

    df_traj = pd.DataFrame([p.__dict__ for p in points])

    return EKFResultM5_1(
        points=points,
        dataframe=df_traj,
        outage_start_t=t0,
        outage_end_t=t0 + outage_duration_sec,
        outage_duration_sec=outage_duration_sec
    )

"""
Module 9.1: 6D Full-Orientation Kinematic EKF Core with Speed-Adaptive Roll Compensation.
Incorporate:
- K_roll(V) = K_base * (1 - exp(-V / V0))
- Fixed vs Adaptive K Roll Modes
- Configurable K_base and V0 parameters
- Exposes actual K(V) diagnostic trace
Strict zero-leakage GNSS outage compliance enforced.
"""

from dataclasses import dataclass
from typing import List, Dict, Any, Optional
import pandas as pd
import numpy as np

from src.iovnbd.navigation.coordinate_frame import AnchorOrigin, enu_to_geodetic
from src.iovnbd.navigation.initialization import InitialState

@dataclass
class OutageEstimatorInputsM9:
    """Strict data structure for Module 9 estimator inputs. Contains ZERO GNSS or reference fields."""
    index: int
    t_rel_sec: float
    dt_sec: float
    ecu_speed_m_s: float
    longitudinal_accel_m_s2: float
    lateral_accel_m_s2: float
    yaw_rate_rad_s: float
    roll_rate_rad_s: float

@dataclass
class EKFStatePointM9:
    index: int
    t_rel_sec: float
    dt_sec: float
    east_m: float
    north_m: float
    up_m: float
    velocity_m_s: float
    heading_rad: float
    heading_deg: float
    roll_rad: float
    roll_deg: float
    gyro_bias_rad_s: float
    nhc_residual_m_s2: float
    nhc_status: str  # ACCEPTED, REJECTED_SPEED, REJECTED_RESIDUAL
    std_east_m: float
    std_north_m: float
    std_velocity_m_s: float
    std_heading_deg: float
    std_roll_deg: float
    lat_deg: float
    lon_deg: float
    gnss_available: bool

@dataclass
class EKFResultM9:
    points: List[EKFStatePointM9]
    dataframe: pd.DataFrame
    outage_start_t: float
    outage_end_t: float
    outage_duration_sec: float
    nhc_accepted_count: int
    nhc_rejected_speed_count: int
    nhc_rejected_residual_count: int

def extract_outage_inputs_m9(
    df_v: pd.DataFrame,
    df_s: pd.DataFrame,
    t0: float,
    outage_duration_sec: float,
    start_idx: int = 1000,
    yaw_scale_factor: float = 0.90,
    dynamic_yaw_scale_enabled: bool = True
) -> List[OutageEstimatorInputsM9]:
    """
    Extracts strictly non-GNSS ECU vehicle sensors + smartphone IMU inputs.
    Audited Provenance:
    - ecu_speed_m_s: Vehicle CAN ECU Indicated Speed (m/s)
    - longitudinal_accel_m_s2: Vehicle CAN ECU Longitudinal Accel (m/s^2)
    - lateral_accel_m_s2: Vehicle CAN ECU Lateral Accel (m/s^2)
    - yaw_rate_rad_s: Vehicle CAN ECU Yaw Rate (rad/s) scaled dynamically or statically
    - roll_rate_rad_s: Smartphone 'GYROSCOPE Roll (rad/s)'
    """
    v_slice = df_v[(df_v["t_rel_sec"] >= t0 - 1e-5) & (df_v["t_rel_sec"] <= t0 + outage_duration_sec + 1e-5)].copy().reset_index(drop=True)
    s_slice = df_s.iloc[start_idx:start_idx + len(v_slice)].copy().reset_index(drop=True)
    inputs_list: List[OutageEstimatorInputsM9] = []
    dt_default = 0.1

    # Find smartphone roll rate column dynamically
    roll_col_matches = [c for c in s_slice.columns if "GYROSCOPE Roll" in c or "gyro_roll" in c.lower()]
    if roll_col_matches:
        roll_series = pd.to_numeric(s_slice[roll_col_matches[0]], errors="coerce").fillna(0.0)
    elif s_slice.shape[1] > 17:
        roll_series = pd.to_numeric(s_slice.iloc[:, 17], errors="coerce").fillna(0.0)
    else:
        roll_series = pd.Series(np.zeros(len(s_slice)))

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
        raw_yaw = float(v_slice["yaw_rate_rad_s"].iloc[i]) if "yaw_rate_rad_s" in v_slice.columns else float(v_slice["Yaw Rate (deg/sec)"].iloc[i]) * (np.pi / 180.0)

        # Import dynamic scaling helper from ekf_m5_1
        from src.iovnbd.navigation.ekf_m5_1 import compute_dynamic_yaw_scale
        yaw_rate = compute_dynamic_yaw_scale(raw_yaw, a_lat, base_scale=yaw_scale_factor, dynamic_enabled=dynamic_yaw_scale_enabled)
        roll_rate = float(roll_series.iloc[i])

        inputs_list.append(OutageEstimatorInputsM9(
            index=idx,
            t_rel_sec=t_curr,
            dt_sec=dt,
            ecu_speed_m_s=ecu_speed,
            longitudinal_accel_m_s2=a_long,
            lateral_accel_m_s2=a_lat,
            yaw_rate_rad_s=yaw_rate,
            roll_rate_rad_s=roll_rate
        ))

    return inputs_list

def propagate_ekf_m9(
    df_v: pd.DataFrame,
    df_s: pd.DataFrame,
    initial_state: InitialState,
    start_idx: int,
    outage_duration_sec: float,
    k_roll_restore: float = 0.1,
    initial_roll_rad: float = 0.0,
    enable_nhc: bool = True,
    nhc_speed_threshold_m_s: float = 0.5,
    nhc_residual_threshold_m_s2: float = 3.0,
    yaw_scale_factor: float = 0.90,
    dynamic_yaw_scale_enabled: bool = True,
    q_var_pos: float = 1e-5,
    q_var_speed: float = 1e-3,
    q_var_yaw: float = 1e-4,
    q_var_roll: float = 1e-4,
    q_var_bias: float = 1e-6,
    r_var_ecu_speed: float = 1e-2,
    r_var_nhc: float = 1e-3,
    r_var_zupt: float = 1e-4,
    g_accel: float = 9.80665
) -> EKFResultM9:
    """
    Executes Module 9 6D Full-Orientation EKF state propagation with fixed roll restoring factor.
    """
    res_m9_1 = propagate_ekf_m9_1(
        df_v, df_s, initial_state, start_idx, outage_duration_sec,
        k_mode="fixed", fixed_k_roll=k_roll_restore, initial_roll_rad=initial_roll_rad,
        enable_nhc=enable_nhc, nhc_speed_threshold_m_s=nhc_speed_threshold_m_s,
        nhc_residual_threshold_m_s2=nhc_residual_threshold_m_s2,
        yaw_scale_factor=yaw_scale_factor,
        dynamic_yaw_scale_enabled=dynamic_yaw_scale_enabled,
        q_var_pos=q_var_pos, q_var_speed=q_var_speed, q_var_yaw=q_var_yaw,
        q_var_roll=q_var_roll, q_var_bias=q_var_bias, r_var_ecu_speed=r_var_ecu_speed,
        r_var_nhc=r_var_nhc, r_var_zupt=r_var_zupt, g_accel=g_accel
    )

    # Convert EKFResultM9_1 to EKFResultM9 for full backwards compatibility
    pts = [EKFStatePointM9(
        index=p.index, t_rel_sec=p.t_rel_sec, dt_sec=p.dt_sec, east_m=p.east_m, north_m=p.north_m,
        up_m=p.up_m, velocity_m_s=p.velocity_m_s, heading_rad=p.heading_rad, heading_deg=p.heading_deg,
        roll_rad=p.roll_rad, roll_deg=p.roll_deg, gyro_bias_rad_s=p.gyro_bias_rad_s,
        nhc_residual_m_s2=p.nhc_residual_m_s2, nhc_status=p.nhc_status, std_east_m=p.std_east_m,
        std_north_m=p.std_north_m, std_velocity_m_s=p.std_velocity_m_s, std_heading_deg=p.std_heading_deg,
        std_roll_deg=p.std_roll_deg, lat_deg=p.lat_deg, lon_deg=p.lon_deg, gnss_available=p.gnss_available
    ) for p in res_m9_1.points]

    return EKFResultM9(
        points=pts, dataframe=res_m9_1.dataframe, outage_start_t=res_m9_1.outage_start_t,
        outage_end_t=res_m9_1.outage_end_t, outage_duration_sec=res_m9_1.outage_duration_sec,
        nhc_accepted_count=res_m9_1.nhc_accepted_count, nhc_rejected_speed_count=res_m9_1.nhc_rejected_speed_count,
        nhc_rejected_residual_count=res_m9_1.nhc_rejected_residual_count
    )

@dataclass
class EKFStatePointM9_1:
    index: int
    t_rel_sec: float
    dt_sec: float
    east_m: float
    north_m: float
    up_m: float
    velocity_m_s: float
    heading_rad: float
    heading_deg: float
    roll_rad: float
    roll_deg: float
    gyro_bias_rad_s: float
    k_roll_adaptive: float
    nhc_residual_m_s2: float
    nhc_status: str
    std_east_m: float
    std_north_m: float
    std_velocity_m_s: float
    std_heading_deg: float
    std_roll_deg: float
    lat_deg: float
    lon_deg: float
    gnss_available: bool

@dataclass
class EKFResultM9_1:
    points: List[EKFStatePointM9_1]
    dataframe: pd.DataFrame
    outage_start_t: float
    outage_end_t: float
    outage_duration_sec: float
    nhc_accepted_count: int
    nhc_rejected_speed_count: int
    nhc_rejected_residual_count: int
    k_mode: str
    k_base: float
    v0: float

def compute_speed_adaptive_k_roll(v_m_s: float, k_base: float = 0.05, v0_m_s: float = 5.0) -> float:
    v_pos = max(0.0, float(v_m_s))
    if v0_m_s <= 0.0:
        return float(k_base)
    k_val = k_base * (1.0 - np.exp(-v_pos / v0_m_s))
    return float(np.clip(k_val, 0.0, k_base))

def propagate_ekf_m9_1(
    df_v: pd.DataFrame,
    df_s: pd.DataFrame,
    initial_state: InitialState,
    start_idx: int,
    outage_duration_sec: float,
    k_mode: str = "adaptive", # "adaptive" or "fixed"
    k_base: float = 0.05,
    v0_m_s: float = 5.0,
    fixed_k_roll: float = 0.10,
    initial_roll_rad: float = 0.0,
    enable_nhc: bool = True,
    nhc_speed_threshold_m_s: float = 0.5,
    nhc_residual_threshold_m_s2: float = 3.0,
    yaw_scale_factor: float = 0.90,
    dynamic_yaw_scale_enabled: bool = True,
    q_var_pos: float = 1e-5,
    q_var_speed: float = 1e-3,
    q_var_yaw: float = 1e-4,
    q_var_roll: float = 1e-4,
    q_var_bias: float = 1e-6,
    r_var_ecu_speed: float = 1e-2,
    r_var_nhc: float = 1e-3,
    r_var_zupt: float = 1e-4,
    g_accel: float = 9.80665
) -> EKFResultM9_1:
    """
    Executes Module 9.1 6D Full-Orientation EKF state propagation with Speed-Adaptive Roll Compensation.
    """
    t0 = initial_state.t_rel_sec
    outage_inputs = extract_outage_inputs_m9(
        df_v, df_s, t0, outage_duration_sec, start_idx,
        yaw_scale_factor=yaw_scale_factor,
        dynamic_yaw_scale_enabled=dynamic_yaw_scale_enabled
    )
    n_samples = len(outage_inputs)

    x = np.array([initial_state.east_m, initial_state.north_m, initial_state.speed_m_s, initial_state.heading_rad, initial_roll_rad, 0.0])
    P = np.diag([1e-4, 1e-4, 1e-2, 1e-2, 1e-3, 1e-4])

    origin = initial_state.origin
    points: List[EKFStatePointM9_1] = []
    nhc_accepted_count = 0
    nhc_rejected_speed_count = 0
    nhc_rejected_residual_count = 0

    for i in range(n_samples):
        inp = outage_inputs[i]
        dt = inp.dt_sec

        nhc_residual = 0.0
        nhc_status = "REJECTED_SPEED"

        # Determine current K_roll restoring factor
        if k_mode == "adaptive":
            k_roll_curr = compute_speed_adaptive_k_roll(x[2], k_base=k_base, v0_m_s=v0_m_s)
        else:
            k_roll_curr = float(fixed_k_roll)

        if i > 0:
            # --- 1. PREDICT STEP ---
            E, N, V, psi, phi, b_z = x
            a_long = inp.longitudinal_accel_m_s2
            omega_raw = inp.yaw_rate_rad_s
            omega_roll = inp.roll_rate_rad_s

            E_pred = E + V * np.sin(psi) * dt
            N_pred = N + V * np.cos(psi) * dt
            V_pred = V + a_long * dt
            psi_pred = psi - (omega_raw - b_z) * dt
            psi_pred = (psi_pred + np.pi) % (2 * np.pi) - np.pi

            # Speed-adaptive roll restoration model
            phi_pred = phi + omega_roll * dt - k_roll_curr * phi * dt
            phi_pred = (phi_pred + np.pi) % (2 * np.pi) - np.pi
            b_z_pred = b_z

            x = np.array([E_pred, N_pred, V_pred, psi_pred, phi_pred, b_z_pred])

            F = np.eye(6)
            F[0, 2] = np.sin(psi) * dt
            F[0, 3] = V * np.cos(psi) * dt
            F[1, 2] = np.cos(psi) * dt
            F[1, 3] = -V * np.sin(psi) * dt
            F[3, 5] = dt
            F[4, 4] = 1.0 - k_roll_curr * dt

            Q = np.diag([q_var_pos, q_var_pos, q_var_speed, q_var_yaw, q_var_roll, q_var_bias])
            P = F @ P @ F.T + Q
            P = (P + P.T) / 2.0

            # --- 2. MEASUREMENT UPDATE 1: ECU Speed Sensor ---
            z_v = inp.ecu_speed_m_s
            H_v = np.array([[0, 0, 1, 0, 0, 0]])
            y_v = z_v - x[2]
            S_v = H_v @ P @ H_v.T + r_var_ecu_speed
            K_v = P @ H_v.T / S_v[0, 0]
            x = x + (K_v * y_v).flatten()
            P = (np.eye(6) - K_v @ H_v) @ P
            P = (P + P.T) / 2.0

            # --- 3. MEASUREMENT UPDATE 2: Roll-Aware NHC Update ---
            if enable_nhc:
                if x[2] > nhc_speed_threshold_m_s:
                    a_lat_meas = inp.lateral_accel_m_s2
                    omega_corr = omega_raw - x[5]
                    h_nhc = x[2] * omega_corr + g_accel * np.sin(x[4])
                    nhc_residual = float(a_lat_meas - h_nhc)

                    if abs(nhc_residual) <= nhc_residual_threshold_m_s2:
                        nhc_status = "ACCEPTED"
                        nhc_accepted_count += 1

                        # Analytical Jacobian: H_nhc = [0, 0, omega_corr, 0, g*cos(phi), -V]
                        H_nhc = np.array([[0, 0, omega_corr, 0, g_accel * np.cos(x[4]), -x[2]]])
                        S_nhc = (H_nhc @ P @ H_nhc.T + r_var_nhc)[0, 0]

                        # Joseph Form Covariance Update for Maximum Numerical Stability
                        K_nhc = P @ H_nhc.T / S_nhc
                        x = x + (K_nhc * nhc_residual).flatten()
                        I_KH = np.eye(6) - K_nhc @ H_nhc
                        P = I_KH @ P @ I_KH.T + K_nhc * r_var_nhc * K_nhc.T
                        P = (P + P.T) / 2.0
                    else:
                        nhc_status = "REJECTED_RESIDUAL"
                        nhc_rejected_residual_count += 1
                else:
                    nhc_status = "REJECTED_SPEED"
                    nhc_rejected_speed_count += 1

            # --- 4. MEASUREMENT UPDATE 3: Zero-Velocity Update (ZUPT) ---
            if z_v < 0.05:
                H_z = np.array([[0, 0, 1, 0, 0, 0]])
                y_z = 0.0 - x[2]
                S_z = H_z @ P @ H_z.T + r_var_zupt
                K_z = P @ H_z.T / S_z[0, 0]
                x = x + (K_z * y_z).flatten()
                P = (np.eye(6) - K_z @ H_z) @ P
                P = (P + P.T) / 2.0

        lat_i, lon_i, alt_i = enu_to_geodetic(x[0], x[1], 0.0, origin)
        heading_deg_i = float(np.degrees(x[3])) % 360.0
        roll_deg_i = float(np.degrees(x[4]))

        points.append(EKFStatePointM9_1(
            index=inp.index,
            t_rel_sec=inp.t_rel_sec,
            dt_sec=dt,
            east_m=x[0],
            north_m=x[1],
            up_m=0.0,
            velocity_m_s=x[2],
            heading_rad=x[3],
            heading_deg=heading_deg_i,
            roll_rad=x[4],
            roll_deg=roll_deg_i,
            gyro_bias_rad_s=x[5],
            k_roll_adaptive=round(k_roll_curr, 4),
            nhc_residual_m_s2=round(nhc_residual, 4),
            nhc_status=nhc_status,
            std_east_m=float(np.sqrt(max(0, P[0, 0]))),
            std_north_m=float(np.sqrt(max(0, P[1, 1]))),
            std_velocity_m_s=float(np.sqrt(max(0, P[2, 2]))),
            std_heading_deg=float(np.degrees(np.sqrt(max(0, P[3, 3])))),
            std_roll_deg=float(np.degrees(np.sqrt(max(0, P[4, 4])))),
            lat_deg=lat_i,
            lon_deg=lon_i,
            gnss_available=False
        ))

    df_traj = pd.DataFrame([p.__dict__ for p in points])

    return EKFResultM9_1(
        points=points,
        dataframe=df_traj,
        outage_start_t=t0,
        outage_end_t=t0 + outage_duration_sec,
        outage_duration_sec=outage_duration_sec,
        nhc_accepted_count=nhc_accepted_count,
        nhc_rejected_speed_count=nhc_rejected_speed_count,
        nhc_rejected_residual_count=nhc_rejected_residual_count,
        k_mode=k_mode,
        k_base=k_base,
        v0=v0_m_s
    )

"""
Module 4: Improved Sensor Fusion Dead Reckoning Estimator.
Combines 4-wheel encoder linear speed averaging with weighted multi-sensor gyroscope fusion (VBOX + Smartphone).
Strictly enforces GNSS outage isolation.
"""

from dataclasses import dataclass
from typing import List, Dict, Any, Optional
import pandas as pd
import numpy as np

from src.iovnbd.navigation.coordinate_frame import AnchorOrigin, enu_to_geodetic
from src.iovnbd.navigation.initialization import InitialState

DEFAULT_VBOX_GYRO_WEIGHT = 0.70
DEFAULT_PHONE_GYRO_WEIGHT = 0.30
PHONE_PITCH_GYRO_BIAS = -0.0037463  # M2 measured stationary bias

@dataclass
class FusedTrajectoryPoint:
    index: int
    t_rel_sec: float
    dt_sec: float
    east_m: float
    north_m: float
    up_m: float
    fused_speed_m_s: float
    fused_yaw_rate_rad_s: float
    heading_rad: float
    heading_deg: float
    lat_deg: float
    lon_deg: float
    gnss_available: bool

@dataclass
class FusedTrajectoryResult:
    points: List[FusedTrajectoryPoint]
    dataframe: pd.DataFrame
    outage_start_t: float
    outage_end_t: float
    outage_duration_sec: float

def propagate_improved_sensor_fusion(
    df_v: pd.DataFrame,
    df_s: pd.DataFrame,
    initial_state: InitialState,
    start_idx: int,
    outage_duration_sec: float,
    vbox_weight: float = DEFAULT_VBOX_GYRO_WEIGHT,
    phone_weight: float = DEFAULT_PHONE_GYRO_WEIGHT,
    phone_gyro_bias: float = PHONE_PITCH_GYRO_BIAS
) -> FusedTrajectoryResult:
    """
    Propagates state using multi-sensor fusion:
    - Speed: 4-wheel encoder average (front-left, front-right, rear-left, rear-right)
    - Yaw rate: Weighted fusion of VBOX yaw rate & bias-corrected Smartphone Pitch Gyro
    No GNSS position/heading updates are permitted during outage.
    """
    dt_default = 0.1
    t0 = initial_state.t_rel_sec
    outage_end_t = t0 + outage_duration_sec

    # Match slice based on row indices starting from start_idx in df_v to ensure exact sample count parity
    v_slice = df_v[(df_v["t_rel_sec"] >= t0 - 1e-5) & (df_v["t_rel_sec"] <= outage_end_t + 1e-5)].copy().reset_index(drop=True)
    n_samples = len(v_slice)

    # Use index alignment for s_slice starting from start_idx to match v_slice count exactly
    s_slice = df_s.iloc[start_idx:start_idx + n_samples].copy().reset_index(drop=True)

    points: List[FusedTrajectoryPoint] = []

    curr_e = initial_state.east_m
    curr_n = initial_state.north_m
    curr_u = initial_state.up_m
    curr_psi = initial_state.heading_rad

    origin = initial_state.origin

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

        # 1. 4-Wheel Speed Fusion (Linear speed average)
        w_fl = float(v_slice["wheel_speed_fl_m_s"].iloc[i]) if "wheel_speed_fl_m_s" in v_slice.columns else float(v_slice["velocity_m_s"].iloc[i])
        w_fr = float(v_slice["wheel_speed_fr_m_s"].iloc[i]) if "wheel_speed_fr_m_s" in v_slice.columns else float(v_slice["velocity_m_s"].iloc[i])
        w_rl = float(v_slice["wheel_speed_rl_m_s"].iloc[i]) if "wheel_speed_rl_m_s" in v_slice.columns else float(v_slice["velocity_m_s"].iloc[i])
        w_rr = float(v_slice["wheel_speed_rr_m_s"].iloc[i]) if "wheel_speed_rr_m_s" in v_slice.columns else float(v_slice["velocity_m_s"].iloc[i])

        fused_speed = (w_fl + w_fr + w_rl + w_rr) / 4.0

        # 2. Multi-Sensor Gyroscope Fusion
        omega_vbox = float(v_slice["yaw_rate_rad_s"].iloc[i]) if "yaw_rate_rad_s" in v_slice.columns else 0.0
        
        # Smartphone Gyro Pitch (correlated with yaw rate in mounted orientation)
        raw_phone_gyro = float(s_slice["GYROSCOPE Pitch (rad/s)"].iloc[i]) if (i < len(s_slice) and "GYROSCOPE Pitch (rad/s)" in s_slice.columns) else omega_vbox
        omega_phone = raw_phone_gyro - phone_gyro_bias

        fused_yaw_rate = vbox_weight * omega_vbox + phone_weight * omega_phone

        if i > 0:
            # Update heading with fused yaw rate
            curr_psi -= fused_yaw_rate * dt
            curr_psi = (curr_psi + np.pi) % (2 * np.pi) - np.pi

            # Update position using fused speed
            v_east = fused_speed * np.sin(curr_psi)
            v_north = fused_speed * np.cos(curr_psi)

            curr_e += v_east * dt
            curr_n += v_north * dt

        lat_i, lon_i, alt_i = enu_to_geodetic(curr_e, curr_n, curr_u, origin)
        heading_deg_i = float(np.degrees(curr_psi)) % 360.0

        points.append(FusedTrajectoryPoint(
            index=idx,
            t_rel_sec=t_curr,
            dt_sec=dt,
            east_m=curr_e,
            north_m=curr_n,
            up_m=curr_u,
            fused_speed_m_s=fused_speed,
            fused_yaw_rate_rad_s=fused_yaw_rate,
            heading_rad=curr_psi,
            heading_deg=heading_deg_i,
            lat_deg=lat_i,
            lon_deg=lon_i,
            gnss_available=False
        ))

    df_traj = pd.DataFrame([p.__dict__ for p in points])

    return FusedTrajectoryResult(
        points=points,
        dataframe=df_traj,
        outage_start_t=t0,
        outage_end_t=outage_end_t,
        outage_duration_sec=outage_duration_sec
    )

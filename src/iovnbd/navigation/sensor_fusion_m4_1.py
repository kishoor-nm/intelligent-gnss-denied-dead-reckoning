"""
Module 4.1: Corrected Sensor Fusion & Sensor Selection Model.
Removes uncalibrated smartphone gyro from vehicle yaw estimation to avoid coordinate frame mismatch.
Permits explicit ablation comparing:
- Exp A: Module 3 Baseline (VBOX Transmission Speed + VBOX Yaw Rate)
- Exp B: Wheel Speed Odometry (4-Wheel Rotational Encoder Speed Average + VBOX Yaw Rate)
- Exp C: Rear Non-Driven Wheel Odometry (Rear-Wheel Rotational Encoder Speed Average + VBOX Yaw Rate)
"""

from dataclasses import dataclass
from typing import List, Dict, Any, Optional
import pandas as pd
import numpy as np

from src.iovnbd.navigation.coordinate_frame import AnchorOrigin, enu_to_geodetic
from src.iovnbd.navigation.initialization import InitialState

# Default dynamic wheel radius assumption (Ford Fiesta 195/50 R15)
DEFAULT_WHEEL_RADIUS_METERS = 0.307

@dataclass
class CorrectedTrajectoryPoint:
    index: int
    t_rel_sec: float
    dt_sec: float
    east_m: float
    north_m: float
    up_m: float
    speed_m_s: float
    yaw_rate_rad_s: float
    heading_rad: float
    heading_deg: float
    lat_deg: float
    lon_deg: float
    gnss_available: bool

@dataclass
class CorrectedTrajectoryResult:
    points: List[CorrectedTrajectoryPoint]
    dataframe: pd.DataFrame
    outage_start_t: float
    outage_end_t: float
    outage_duration_sec: float
    speed_mode: str

def propagate_corrected_dead_reckoning(
    df_v: pd.DataFrame,
    initial_state: InitialState,
    start_idx: int,
    outage_duration_sec: float,
    speed_mode: str = "4wheel_avg",  # 'vbox', '4wheel_avg', 'rear_wheel_avg'
    wheel_radius_m: float = DEFAULT_WHEEL_RADIUS_METERS
) -> CorrectedTrajectoryResult:
    """
    Propagates dead reckoning state with explicit sensor selection:
    - Smartphone Gyroscope is EXCLUDED from vehicle yaw estimation (PHONE-TO-VEHICLE ALIGNMENT NOT VERIFIED).
    - Speed options:
        * 'vbox': Direct VBOX transmission speed
        * '4wheel_avg': Average of 4 wheel encoders (* R_wheel ASSUMED)
        * 'rear_wheel_avg': Average of 2 non-driven rear wheel encoders (* R_wheel ASSUMED)
    """
    dt_default = 0.1
    t0 = initial_state.t_rel_sec
    outage_end_t = t0 + outage_duration_sec

    v_slice = df_v[(df_v["t_rel_sec"] >= t0 - 1e-5) & (df_v["t_rel_sec"] <= outage_end_t + 1e-5)].copy().reset_index(drop=True)
    n_samples = len(v_slice)

    points: List[CorrectedTrajectoryPoint] = []

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

        # Speed Selection
        if speed_mode == "vbox":
            speed = float(v_slice["velocity_m_s"].iloc[i]) if "velocity_m_s" in v_slice.columns else float(v_slice["Velocity (km/hr)"].iloc[i]) / 3.6
        elif speed_mode == "4wheel_avg":
            if "wheel_speed_fl_m_s" in v_slice.columns:
                w_fl = float(v_slice["wheel_speed_fl_m_s"].iloc[i])
                w_fr = float(v_slice["wheel_speed_fr_m_s"].iloc[i])
                w_rl = float(v_slice["wheel_speed_rl_m_s"].iloc[i])
                w_rr = float(v_slice["wheel_speed_rr_m_s"].iloc[i])
            else:
                w_fl = float(v_slice["Wheel Speed Front Left (rad/sec)"].iloc[i]) * wheel_radius_m
                w_fr = float(v_slice["Wheel Speed Front Right (rad/sec)"].iloc[i]) * wheel_radius_m
                w_rl = float(v_slice["Wheel Speed Rear Left (rad/sec)"].iloc[i]) * wheel_radius_m
                w_rr = float(v_slice["Wheel Speed Rear Right (rad/sec)"].iloc[i]) * wheel_radius_m
            speed = (w_fl + w_fr + w_rl + w_rr) / 4.0
        elif speed_mode == "rear_wheel_avg":
            if "wheel_speed_rl_m_s" in v_slice.columns:
                w_rl = float(v_slice["wheel_speed_rl_m_s"].iloc[i])
                w_rr = float(v_slice["wheel_speed_rr_m_s"].iloc[i])
            else:
                w_rl = float(v_slice["Wheel Speed Rear Left (rad/sec)"].iloc[i]) * wheel_radius_m
                w_rr = float(v_slice["Wheel Speed Rear Right (rad/sec)"].iloc[i]) * wheel_radius_m
            speed = (w_rl + w_rr) / 2.0
        else:
            speed = float(v_slice["velocity_m_s"].iloc[i])

        # Yaw Rate: Validated VBOX CAN Bus Yaw Rate (deg/s -> rad/s)
        yaw_rate = float(v_slice["yaw_rate_rad_s"].iloc[i]) if "yaw_rate_rad_s" in v_slice.columns else float(v_slice["Yaw Rate (deg/sec)"].iloc[i]) * (np.pi / 180.0)

        if i > 0:
            # Update heading (Corrected Sign Alignment for Clockwise ENU)
            curr_psi -= yaw_rate * dt
            curr_psi = (curr_psi + np.pi) % (2 * np.pi) - np.pi

            # Update position
            v_east = speed * np.sin(curr_psi)
            v_north = speed * np.cos(curr_psi)

            curr_e += v_east * dt
            curr_n += v_north * dt

        lat_i, lon_i, alt_i = enu_to_geodetic(curr_e, curr_n, curr_u, origin)
        heading_deg_i = float(np.degrees(curr_psi)) % 360.0

        points.append(CorrectedTrajectoryPoint(
            index=idx,
            t_rel_sec=t_curr,
            dt_sec=dt,
            east_m=curr_e,
            north_m=curr_n,
            up_m=curr_u,
            speed_m_s=speed,
            yaw_rate_rad_s=yaw_rate,
            heading_rad=curr_psi,
            heading_deg=heading_deg_i,
            lat_deg=lat_i,
            lon_deg=lon_i,
            gnss_available=False
        ))

    df_traj = pd.DataFrame([p.__dict__ for p in points])

    return CorrectedTrajectoryResult(
        points=points,
        dataframe=df_traj,
        outage_start_t=t0,
        outage_end_t=outage_end_t,
        outage_duration_sec=outage_duration_sec,
        speed_mode=speed_mode
    )

"""
Module 2: Unit conversions and explicit verification.
Preserves raw original fields and creates explicit converted fields.
"""

from typing import Tuple, Optional
import pandas as pd
import numpy as np

# Configurable dynamic wheel radius default (Ford Fiesta 195/50 R15 Tyre Spec)
DEFAULT_WHEEL_RADIUS_METERS = 0.307

def convert_vehicle_units(
    df_v: pd.DataFrame,
    wheel_radius_m: float = DEFAULT_WHEEL_RADIUS_METERS
) -> pd.DataFrame:
    """
    Converts vehicle ECU fields:
    - Speed (km/hr -> m/s)
    - Wheel rotational speed (rad/sec -> m/s using configurable wheel radius)
    - Accelerations (g -> m/s²)
    - Yaw rate (deg/sec -> rad/s)
    """
    df_out = df_v.copy()

    # 1. Vehicle speed: km/hr -> m/s
    if "Velocity (km/hr)" in df_out.columns:
        df_out["velocity_m_s"] = df_out["Velocity (km/hr)"] / 3.6
    if "Indicated Vehicle Speed (km/hr)" in df_out.columns:
        df_out["indicated_speed_m_s"] = df_out["Indicated Vehicle Speed (km/hr)"] / 3.6

    # 2. Wheel speeds: rad/sec -> linear speed m/s (ASSUMED radius)
    wheel_cols = [
        ("Wheel Speed Front Left (rad/sec)", "wheel_speed_fl_m_s"),
        ("Wheel Speed Front Right (rad/sec)", "wheel_speed_fr_m_s"),
        ("Wheel Speed Rear Left (rad/sec)", "wheel_speed_rl_m_s"),
        ("Wheel Speed Rear Right (rad/sec)", "wheel_speed_rr_m_s")
    ]
    for orig_col, new_col in wheel_cols:
        if orig_col in df_out.columns:
            df_out[new_col] = df_out[orig_col] * wheel_radius_m

    # 3. Accelerations: g -> m/s²
    if "Indicated Longitudinal Acceleration (g)" in df_out.columns:
        df_out["longitudinal_accel_m_s2"] = df_out["Indicated Longitudinal Acceleration (g)"] * 9.80665
    if "Indicated Lateral Acceleration (g)" in df_out.columns:
        df_out["lateral_accel_m_s2"] = df_out["Indicated Lateral Acceleration (g)"] * 9.80665

    # 4. Yaw rate: deg/sec -> rad/s
    if "Yaw Rate (deg/sec)" in df_out.columns:
        df_out["yaw_rate_rad_s"] = df_out["Yaw Rate (deg/sec)"] * (np.pi / 180.0)

    # 5. Steering Angle: deg -> rad
    if "Steering Angle (degrees)" in df_out.columns:
        df_out["steering_angle_rad"] = df_out["Steering Angle (degrees)"] * (np.pi / 180.0)

    return df_out

def convert_smartphone_units(df_s: pd.DataFrame) -> pd.DataFrame:
    """
    Converts smartphone fields:
    - GPS speed (km/h -> m/s)
    - Orientation angles (deg -> rad)
    """
    df_out = df_s.copy()

    if "GPS SPEED (Kmh)" in df_out.columns:
        df_out["gps_speed_m_s"] = df_out["GPS SPEED (Kmh)"] / 3.6

    orientation_cols = [
        ("ORIENTATION (Yaw) (°)", "orientation_yaw_rad"),
        ("ORIENTATION (Pitch) (°)", "orientation_pitch_rad"),
        ("ORIENTATION (Roll ) (°)", "orientation_roll_rad"),
        ("GPS ORIENTATION (°)", "gps_orientation_rad")
    ]
    for orig_col, new_col in orientation_cols:
        if orig_col in df_out.columns:
            df_out[new_col] = np.radians(df_out[orig_col])

    return df_out

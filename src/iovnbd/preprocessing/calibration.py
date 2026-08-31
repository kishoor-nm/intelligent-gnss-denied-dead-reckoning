"""
Module 2: Calibration and stationary segment analysis.
Detects stationary windows and calculates zero-rate gyroscope bias and stationary acceleration residual.
"""

from dataclasses import dataclass
from typing import Dict, Any, List, Optional, Tuple
import pandas as pd
import numpy as np

@dataclass
class StationaryAnalysisResult:
    stationary_windows_found: int
    longest_window_samples: int
    longest_window_seconds: float
    window_start_idx: int
    window_end_idx: int
    gyro_bias_rad_s: Tuple[float, float, float]  # (yaw, pitch, roll)
    gyro_noise_std_rad_s: Tuple[float, float, float]
    accel_stat_mean_m_s2: Tuple[float, float, float]
    gravity_stat_mean_m_s2: Tuple[float, float, float]
    stationary_accel_residual_m_s2: Tuple[float, float, float]
    residual_explanation: str

def find_stationary_windows(
    df_v: pd.DataFrame,
    speed_col: str = "Indicated Vehicle Speed (km/hr)",
    min_window_samples: int = 100
) -> List[Tuple[int, int, int]]:
    """
    Identifies contiguous indices where vehicle speed is zero.
    Returns list of (start_idx, end_idx, sample_count).
    """
    if speed_col not in df_v.columns:
        return []

    speed = df_v[speed_col]
    stat_mask = (speed == 0)
    stat_indices = np.where(stat_mask)[0]

    if len(stat_indices) == 0:
        return []

    blocks = []
    start = stat_indices[0]
    prev = stat_indices[0]

    for idx in stat_indices[1:]:
        if idx == prev + 1:
            prev = idx
        else:
            count = prev - start + 1
            if count >= min_window_samples:
                blocks.append((start, prev, count))
            start = idx
            prev = idx

    count = prev - start + 1
    if count >= min_window_samples:
        blocks.append((start, prev, count))

    # Sort descending by sample count
    return sorted(blocks, key=lambda x: x[2], reverse=True)

def analyze_sensor_calibration(
    df_s: pd.DataFrame,
    df_v: pd.DataFrame
) -> StationaryAnalysisResult:
    """
    Analyzes sensor behavior during candidate stationary periods.
    Estimates gyroscope zero-rate bias and stationary acceleration residual.
    """
    blocks = find_stationary_windows(df_v, min_window_samples=100)

    if len(blocks) == 0:
        return StationaryAnalysisResult(
            stationary_windows_found=0,
            longest_window_samples=0,
            longest_window_seconds=0.0,
            window_start_idx=0,
            window_end_idx=0,
            gyro_bias_rad_s=(0.0, 0.0, 0.0),
            gyro_noise_std_rad_s=(0.0, 0.0, 0.0),
            accel_stat_mean_m_s2=(0.0, 0.0, 0.0),
            gravity_stat_mean_m_s2=(0.0, 0.0, 0.0),
            stationary_accel_residual_m_s2=(0.0, 0.0, 0.0),
            residual_explanation="No stationary segment >= 10s found in sequence."
        )

    best = blocks[0]
    start_i, end_i, count = best

    sub_s = df_s.iloc[start_i:end_i+1]

    # Gyroscope fields
    gx_col = [c for c in sub_s.columns if "GYROSCOPE Yaw" in c][0]
    gy_col = [c for c in sub_s.columns if "GYROSCOPE Pitch" in c][0]
    gz_col = [c for c in sub_s.columns if "GYROSCOPE Roll" in c][0]

    b_gx = float(sub_s[gx_col].mean())
    b_gy = float(sub_s[gy_col].mean())
    b_gz = float(sub_s[gz_col].mean())

    std_gx = float(sub_s[gx_col].std())
    std_gy = float(sub_s[gy_col].std())
    std_gz = float(sub_s[gz_col].std())

    # Accelerometer fields
    ax_col = [c for c in sub_s.columns if "ACCELEROMETER X" in c][0]
    ay_col = [c for c in sub_s.columns if "ACCELEROMETER Y" in c][0]
    az_col = [c for c in sub_s.columns if "ACCELEROMETER Z" in c][0]

    a_x = float(sub_s[ax_col].mean())
    a_y = float(sub_s[ay_col].mean())
    a_z = float(sub_s[az_col].mean())

    # Gravity fields
    g_x_col = [c for c in sub_s.columns if "GRAVITY X" in c][0]
    g_y_col = [c for c in sub_s.columns if "GRAVITY Y" in c][0]
    g_z_col = [c for c in sub_s.columns if "GRAVITY Z" in c][0]

    g_x = float(sub_s[g_x_col].mean())
    g_y = float(sub_s[g_y_col].mean())
    g_z = float(sub_s[g_z_col].mean())

    # Stationary Acceleration Residual: a_measured - gravity
    res_x = a_x - g_x
    res_y = a_y - g_y
    res_z = a_z - g_z

    explanation = (
        "Stationary Acceleration Residual (a_measured - gravity) is reported. "
        "It includes accelerometer sensor bias, measurement noise, gravity estimation errors, and mounting alignment offsets. "
        "It is NOT classified as a pure accelerometer bias because individual error components cannot be isolated from single-position stationary data."
    )

    return StationaryAnalysisResult(
        stationary_windows_found=len(blocks),
        longest_window_samples=count,
        longest_window_seconds=count * 0.1,
        window_start_idx=start_i,
        window_end_idx=end_i,
        gyro_bias_rad_s=(b_gx, b_gy, b_gz),
        gyro_noise_std_rad_s=(std_gx, std_gy, std_gz),
        accel_stat_mean_m_s2=(a_x, a_y, a_z),
        gravity_stat_mean_m_s2=(g_x, g_y, g_z),
        stationary_accel_residual_m_s2=(res_x, res_y, res_z),
        residual_explanation=explanation
    )

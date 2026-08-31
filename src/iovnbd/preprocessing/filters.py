"""
Module 2: Heuristic screening and outlier classification.
Flags records as VALID or SUSPICIOUS using explicit heuristic screening thresholds without deleting raw records.
"""

from dataclasses import dataclass
from typing import Dict, Any, List, Tuple
import pandas as pd
import numpy as np

# Explicit HEURISTIC / SCREENING Thresholds (Not sensor ground truth)
MAX_HEURISTIC_ACCEL_M_S2 = 50.0  # ~5g
MAX_HEURISTIC_GYRO_RAD_S = 10.0   # ~573 deg/s

@dataclass
class OutlierScreeningResult:
    total_records: int
    suspicious_accel_count: int
    suspicious_gyro_count: int
    suspicious_gps_count: int
    total_flagged_suspicious: int
    screening_policy: str

def screen_outliers(df_s: pd.DataFrame, df_v: pd.DataFrame) -> Tuple[pd.DataFrame, pd.DataFrame, OutlierScreeningResult]:
    """
    Screens sensor streams against heuristic physical bounds.
    Adds 'quality_flag' column ('VALID' or 'SUSPICIOUS') without silently deleting data.
    """
    df_s_out = df_s.copy()
    df_v_out = df_v.copy()

    df_s_out["quality_flag"] = "VALID"
    df_v_out["quality_flag"] = "VALID"

    # Smartphone Accel check
    ax_col = [c for c in df_s_out.columns if "ACCELEROMETER X" in c][0]
    ay_col = [c for c in df_s_out.columns if "ACCELEROMETER Y" in c][0]
    az_col = [c for c in df_s_out.columns if "ACCELEROMETER Z" in c][0]

    acc_norm = np.sqrt(df_s_out[ax_col]**2 + df_s_out[ay_col]**2 + df_s_out[az_col]**2)
    susp_accel = acc_norm > MAX_HEURISTIC_ACCEL_M_S2
    df_s_out.loc[susp_accel, "quality_flag"] = "SUSPICIOUS"

    # Smartphone Gyro check
    gx_col = [c for c in df_s_out.columns if "GYROSCOPE Yaw" in c][0]
    gy_col = [c for c in df_s_out.columns if "GYROSCOPE Pitch" in c][0]
    gz_col = [c for c in df_s_out.columns if "GYROSCOPE Roll" in c][0]

    gyro_max = np.maximum.reduce([np.abs(df_s_out[gx_col]), np.abs(df_s_out[gy_col]), np.abs(df_s_out[gz_col])])
    susp_gyro = gyro_max > MAX_HEURISTIC_GYRO_RAD_S
    df_s_out.loc[susp_gyro, "quality_flag"] = "SUSPICIOUS"

    # GPS validity check (0,0 coordinate lock loss)
    susp_gps_s = (df_s_out["GPS LATITUDE (degrees)"] == 0) | (df_s_out["GPS LONGITUDE (degrees)"] == 0)
    df_s_out.loc[susp_gps_s, "quality_flag"] = "SUSPICIOUS"

    susp_gps_v = (df_v_out["Latitude (degrees)"] == 0) | (df_v_out["Longitude (degrees)"] == 0)
    df_v_out.loc[susp_gps_v, "quality_flag"] = "SUSPICIOUS"

    result = OutlierScreeningResult(
        total_records=len(df_s_out),
        suspicious_accel_count=int(susp_accel.sum()),
        suspicious_gyro_count=int(susp_gyro.sum()),
        suspicious_gps_count=int(susp_gps_s.sum()),
        total_flagged_suspicious=int((df_s_out["quality_flag"] == "SUSPICIOUS").sum()),
        screening_policy="HEURISTIC_SCREENING_ONLY (No records deleted or modified)"
    )

    return df_s_out, df_v_out, result

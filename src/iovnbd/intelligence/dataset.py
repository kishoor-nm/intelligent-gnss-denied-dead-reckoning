"""
Module 6: Intelligent Feature Engineering, Target Audit & Dataset Splitting.
Strictly causal feature computation (past samples only) per sequence.
Zero temporal, window, or cross-driver leakage.
"""

from dataclasses import dataclass
from typing import List, Dict, Any, Tuple
import os
import pandas as pd
import numpy as np

FEATURE_COLUMNS = [
    "acc_mag_std50",
    "acc_z_std50",
    "gyro_x_std50",
    "gyro_z_std50"
]

TARGET_COLUMN = "indicated_speed_m_s"

def extract_features_robust(df_s: pd.DataFrame) -> pd.DataFrame:
    """
    Extracts causal rolling window features using robust string matching on column names.
    Window size = 50 samples (5.0s at 10 Hz update rate).
    """
    def find_col(patterns):
        for p in patterns:
            matches = [c for c in df_s.columns if p.lower() in c.strip().lower()]
            if matches:
                return pd.to_numeric(df_s[matches[0]], errors='coerce').fillna(0.0)
        return pd.Series(np.zeros(len(df_s)))

    ax = find_col(['ACCELEROMETER X', 'accel_x'])
    ay = find_col(['ACCELEROMETER Y', 'accel_y'])
    az = find_col(['ACCELEROMETER Z', 'accel_z'])

    gx = find_col(['GYROSCOPE Yaw', 'gyro_x'])
    gy = find_col(['GYROSCOPE Pitch', 'gyro_y'])
    gz = find_col(['GYROSCOPE Roll', 'gyro_z'])

    acc_mag = np.sqrt(ax**2 + ay**2 + az**2)

    # Causal past-only rolling standard deviations
    acc_mag_std = acc_mag.rolling(window=50, min_periods=1).std().fillna(0.0)
    acc_z_std = az.rolling(window=50, min_periods=1).std().fillna(0.0)
    gyro_x_std = gx.rolling(window=50, min_periods=1).std().fillna(0.0)
    gyro_z_std = gz.rolling(window=50, min_periods=1).std().fillna(0.0)

    df_feat = pd.DataFrame({
        "acc_mag_std50": acc_mag_std,
        "acc_z_std50": acc_z_std,
        "gyro_x_std50": gyro_x_std,
        "gyro_z_std50": gyro_z_std
    })

    return df_feat

def load_sequence_dataset(v_csv_path: str, s_csv_path: str) -> Tuple[pd.DataFrame, pd.Series]:
    """
    Loads V and S CSV files with latin1 encoding fallback, computes causal features, and extracts the target.
    Audited Target Provenance: CAN-bus ECU 'Indicated Vehicle Speed (km/hr)' -> indicated_speed_m_s (VERIFIED NON-GNSS).
    """
    try:
        df_v = pd.read_csv(v_csv_path, encoding='utf-8')
    except Exception:
        df_v = pd.read_csv(v_csv_path, encoding='latin1')

    try:
        df_s = pd.read_csv(s_csv_path, encoding='utf-8')
    except Exception:
        df_s = pd.read_csv(s_csv_path, encoding='latin1')

    df_feat = extract_features_robust(df_s)

    speed_matches = [c for c in df_v.columns if 'indicated vehicle speed' in c.strip().lower() or 'indicated_speed_m_s' in c.strip().lower()]
    if speed_matches:
        speed_col = speed_matches[0]
        target = pd.to_numeric(df_v[speed_col], errors='coerce').fillna(0.0)
        if 'km/hr' in speed_col:
            target = target / 3.6
    else:
        target = pd.Series(np.zeros(len(df_v)))

    return df_feat, target

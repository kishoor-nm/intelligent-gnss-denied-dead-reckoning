"""
Dataset Schema Validation and Streaming Interface Contracts for SIH 2026 PS-168 Prototype.
Enforces input data contracts, missing value checks, unit assertions, and defines a clean
StreamingSensorSource abstraction for future real-time sensor integration.
"""

from dataclasses import dataclass
from typing import Dict, Any, List, Optional, Protocol, Tuple
import pandas as pd
import numpy as np

REQUIRED_VEHICLE_COLUMNS = [
    "t_rel_sec",
    "indicated_speed_m_s",
    "longitudinal_accel_m_s2",
    "lateral_accel_m_s2",
    "yaw_rate_rad_s"
]

REQUIRED_SMARTPHONE_COLUMNS = [
    "t_rel_sec",
    "roll_rate_rad_s"
]

EVALUATION_ONLY_GNSS_COLUMNS = [
    "Latitude (degrees)",
    "Longitude (degrees)",
    "velocity_m_s",
    "Heading (degrees)"
]

@dataclass
class SingleSensorSample:
    """Causal, single-timestamp multi-sensor payload entering navigation step."""
    t_rel_sec: float
    indicated_speed_m_s: float
    longitudinal_accel_m_s2: float
    lateral_accel_m_s2: float
    yaw_rate_rad_s: float
    roll_rate_rad_s: float
    # Optional reference GNSS attributes (strictly evaluation-only)
    gnss_lat_deg: Optional[float] = None
    gnss_lon_deg: Optional[float] = None
    gnss_speed_m_s: Optional[float] = None

class StreamingSensorSource(Protocol):
    """Abstract interface contract for future real-time sensor streams (e.g. Serial, MQTT, ROS2)."""
    def is_connected(self) -> bool:
        ...
    def read_next_sample(self) -> Optional[SingleSensorSample]:
        ...

def validate_vehicle_dataframe_schema(df: pd.DataFrame) -> Tuple[bool, List[str]]:
    """Validates required vehicle CAN bus sensor columns and checks for missing/NaN values."""
    errors = []
    for col in REQUIRED_VEHICLE_COLUMNS:
        if col not in df.columns:
            # Check for alternative raw column names if processed column is missing
            if col == "indicated_speed_m_s" and "Indicated Vehicle Speed (km/hr)" in df.columns:
                continue
            if col == "longitudinal_accel_m_s2" and "Indicated Longitudinal Acceleration (g)" in df.columns:
                continue
            if col == "lateral_accel_m_s2" and "Indicated Lateral Acceleration (g)" in df.columns:
                continue
            if col == "yaw_rate_rad_s" and "Yaw Rate (deg/sec)" in df.columns:
                continue
            errors.append(f"Missing required vehicle column: '{col}'")

    if errors:
        return False, errors

    # Check for excessive NaNs in required columns
    for col in REQUIRED_VEHICLE_COLUMNS:
        if col in df.columns:
            nan_cnt = df[col].isna().sum()
            if nan_cnt > 0:
                errors.append(f"Vehicle column '{col}' contains {nan_cnt} NaN values.")

    return (len(errors) == 0), errors

def validate_smartphone_dataframe_schema(df: pd.DataFrame) -> Tuple[bool, List[str]]:
    """Validates required smartphone IMU sensor columns."""
    errors = []
    for col in REQUIRED_SMARTPHONE_COLUMNS:
        if col not in df.columns:
            if col == "roll_rate_rad_s" and "GYROSCOPE Roll (rad/s)" in df.columns:
                continue
            errors.append(f"Missing required smartphone column: '{col}'")

    if errors:
        return False, errors

    for col in REQUIRED_SMARTPHONE_COLUMNS:
        if col in df.columns:
            nan_cnt = df[col].isna().sum()
            if nan_cnt > 0:
                errors.append(f"Smartphone column '{col}' contains {nan_cnt} NaN values.")

    return (len(errors) == 0), errors

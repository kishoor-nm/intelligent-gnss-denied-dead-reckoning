"""
Module for schema discovery and dynamic validation against documented specs.
"""

from dataclasses import dataclass
from typing import List, Dict, Any, Optional
import pandas as pd

# Documented column keys (with normalized whitespace & special char handling)
SMARTPHONE_DOCUMENTED_COLUMNS = [
    "GPS LATITUDE (degrees)",
    "GPS LONGITUDE (degrees)",
    "GPS ALTITUDE (m)",
    "GPS SPEED (Kmh)",
    "GPS ACCURACY (m)",
    "GPS ORIENTATION (°)",
    "GPS SATELLITES IN RANGE",
    "TIME SINCE START (ms)",
    "DATE (YYYY-MO-DD HH-MI-SS_SSS)",
    "ACCELEROMETER X (m/s²)",
    "ACCELEROMETER Y (m/s²)",
    "ACCELEROMETER Z (m/s²)",
    "GRAVITY X (m/s²)",
    "GRAVITY Y (m/s²)",
    "GRAVITY Z (m/s²)",
    "GYROSCOPE Yaw (rad/s)",
    "GYROSCOPE Pitch (rad/s)",
    "GYROSCOPE Roll (rad/s)",
    "MAGNETIC FIELD X (μT)",
    "MAGNETIC FIELD Y (μT)",
    "MAGNETIC FIELD Z (μT)",
    "ORIENTATION (Yaw) (°)",
    "ORIENTATION (Pitch) (°)",
    "ORIENTATION (Roll ) (°)"
]

VEHICLE_DOCUMENTED_COLUMNS = [
    "No of GPS Satellites Available",
    "Time Since Start of Day (seconds)",
    "Latitude (degrees)",
    "Longitude (degrees)",
    "Velocity (km/hr)",
    "Heading (degrees)",
    "Height (km)",
    "Vertical velocity (km/hr)",
    "Sample period (seconds)",
    "Steering Angle (degrees)",
    "Wheel Speed Front Left (rad/sec)",
    "Wheel Speed Front Right (rad/sec)",
    "Wheel Speed Rear Left (rad/sec)",
    "Wheel Speed Rear Right (rad/sec)",
    "Yaw Rate (deg/sec)",
    "Indicated Vehicle Speed (km/hr)",
    "Indicated Longitudinal Acceleration (g)",
    "Indicated Lateral Acceleration (g)",
    "Handbrake (0 or 1)",
    "Gear Requested (Number fof gear employed 1-5)",
    "Gear (Number fof gear employed 1-5)",
    "Engine Speed (rev/min)",
    "Coolant Temperature (degrees)",
    "Clutch Position (0 or 1)",
    "Brake Pressure (psi)",
    "Brake Position (0 or 1)",
    "Battery Voltage (volts)",
    "Air Temperature (degrees)",
    "Accelerator Pedal Position (0 or 1)"
]

@dataclass
class SchemaInspectionResult:
    stream_type: str  # 'S' or 'V' or 'UNKNOWN'
    column_count: int
    observed_columns: List[str]
    missing_documented_columns: List[str]
    extra_unexpected_columns: List[str]
    exact_match: bool

def inspect_dataframe_schema(df: pd.DataFrame, file_prefix: str) -> SchemaInspectionResult:
    observed_cols = [str(c).strip() for c in df.columns]
    
    if file_prefix.startswith("S-") or "ACCELEROMETER" in "".join(observed_cols).upper():
        stream_type = "S"
        # Match by prefix/token for tolerance of unicode encoding differences (m/s² vs m/s)
        doc_cols = SMARTPHONE_DOCUMENTED_COLUMNS
    elif file_prefix.startswith("V-") or "WHEEL SPEED" in "".join(observed_cols).upper():
        stream_type = "V"
        doc_cols = VEHICLE_DOCUMENTED_COLUMNS
    else:
        stream_type = "UNKNOWN"
        doc_cols = []

    # Count matching columns by column count and stem check
    missing = []
    extra = []

    # For S stream, compare lengths
    if len(observed_cols) == len(doc_cols):
        exact_match = True
    else:
        exact_match = False
        missing = [c for c in doc_cols if not any(c[:10] in o for o in observed_cols)]
        extra = [o for o in observed_cols if not any(d[:10] in o for d in doc_cols)]

    return SchemaInspectionResult(
        stream_type=stream_type,
        column_count=len(observed_cols),
        observed_columns=observed_cols,
        missing_documented_columns=missing,
        extra_unexpected_columns=extra,
        exact_match=exact_match
    )

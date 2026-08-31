"""
Module 2: Complete end-to-end preprocessing pipeline wrapper.
Executes timestamp normalization, unit conversions, calibration analysis, frame transforms, and outlier screening.
"""

import os
from dataclasses import dataclass
from typing import Dict, Any, Optional
import pandas as pd

from src.iovnbd.loader import load_iovnbd_csv
from src.iovnbd.preprocessing.timestamp import normalize_timestamps
from src.iovnbd.preprocessing.units import convert_vehicle_units, convert_smartphone_units
from src.iovnbd.preprocessing.calibration import analyze_sensor_calibration, StationaryAnalysisResult
from src.iovnbd.preprocessing.coordinates import apply_phone_to_vehicle_transform
from src.iovnbd.preprocessing.filters import screen_outliers

@dataclass
class PipelineExecutionResult:
    sequence_name: str
    processed_s_path: str
    processed_v_path: str
    record_count: int
    duration_sec: float
    stationary_analysis: StationaryAnalysisResult
    sync_status: str

def run_preprocessing_pipeline(
    s_csv_path: str,
    v_csv_path: str,
    output_dir: str,
    sequence_name: str = "S1",
    wheel_radius_m: float = 0.307
) -> PipelineExecutionResult:
    """
    Runs full Module 2 preprocessing on input sequence CSVs and exports clean processed files.
    Raw source files remain completely untouched.
    """
    stream_s = load_iovnbd_csv(s_csv_path)
    stream_v = load_iovnbd_csv(v_csv_path)

    if stream_s.dataframe is None or stream_v.dataframe is None:
        raise ValueError(f"Failed to load input CSVs for sequence {sequence_name}: S={stream_s.error}, V={stream_v.error}")

    df_s_raw = stream_s.dataframe
    df_v_raw = stream_v.dataframe

    # 1. Timestamp Normalization
    df_s_ts, df_v_ts, ts_res = normalize_timestamps(df_s_raw, "TIME SINCE START (ms)", df_v_raw, "Time Since Start of Day (seconds)")

    # 2. Unit Conversions
    df_s_unit = convert_smartphone_units(df_s_ts)
    df_v_unit = convert_vehicle_units(df_v_ts, wheel_radius_m=wheel_radius_m)

    # 3. Calibration & Stationary Analysis
    stat_res = analyze_sensor_calibration(df_s_raw, df_v_raw)

    # 4. Coordinate Frame & Gravity Transform
    df_s_coord = apply_phone_to_vehicle_transform(df_s_unit)

    # 5. Outlier Screening
    df_s_final, df_v_final, screen_res = screen_outliers(df_s_coord, df_v_unit)

    # Export Processed Data
    os.makedirs(output_dir, exist_ok=True)
    processed_s_path = os.path.join(output_dir, f"S-{sequence_name}_processed.csv")
    processed_v_path = os.path.join(output_dir, f"V-{sequence_name}_processed.csv")

    df_s_final.to_csv(processed_s_path, index=False)
    df_v_final.to_csv(processed_v_path, index=False)

    return PipelineExecutionResult(
        sequence_name=sequence_name,
        processed_s_path=processed_s_path,
        processed_v_path=processed_v_path,
        record_count=len(df_s_final),
        duration_sec=ts_res.duration_sec,
        stationary_analysis=stat_res,
        sync_status=ts_res.sync_status
    )

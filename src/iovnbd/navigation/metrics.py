"""
Module 3: Error Metrics & Accuracy Analysis.
Calculates position error, horizontal error, max error, RMSE, and error vs outage duration.
"""

from dataclasses import dataclass
from typing import Dict, Any, List, Optional
import pandas as pd
import numpy as np

from src.iovnbd.navigation.coordinate_frame import geodetic_to_enu
from src.iovnbd.navigation.baseline import TrajectoryResult

@dataclass
class OutageMetricsResult:
    outage_duration_sec: float
    outage_start_t: float
    outage_end_t: float
    sample_count: int
    final_position_error_m: float
    max_position_error_m: float
    mean_position_error_m: float
    rmse_position_error_m: float
    final_heading_error_deg: float
    max_heading_error_deg: float
    drift_rate_m_per_sec: float
    error_series_m: List[float]

def calculate_outage_error_metrics(
    traj_res: TrajectoryResult,
    df_ref: pd.DataFrame,
    ref_lat_col: str = "Latitude (degrees)",
    ref_lon_col: str = "Longitude (degrees)",
    ref_heading_col: str = "Heading (degrees)"
) -> OutageMetricsResult:
    """
    Computes rigorous error metrics comparing the dead-reckoned trajectory against reference data.
    """
    df_dr = traj_res.dataframe
    origin = traj_res.points[0].index  # use start point origin

    errors_m = []
    heading_errors_deg = []

    # Map reference rows by t_rel_sec matching
    t_start = traj_res.outage_start_t
    t_end = traj_res.outage_end_t

    ref_slice = df_ref[(df_ref["t_rel_sec"] >= t_start) & (df_ref["t_rel_sec"] <= t_end)].copy().reset_index(drop=True)

    # Use first reference row to set ENU anchor
    lat0 = ref_slice[ref_lat_col].iloc[0]
    lon0 = ref_slice[ref_lon_col].iloc[0]
    from src.iovnbd.navigation.coordinate_frame import AnchorOrigin
    ref_origin = AnchorOrigin(lat0_rad=float(np.radians(lat0)), lon0_rad=float(np.radians(lon0)), alt0_m=0.0)

    for i in range(min(len(df_dr), len(ref_slice))):
        dr_e = df_dr["east_m"].iloc[i]
        dr_n = df_dr["north_m"].iloc[i]
        dr_head = df_dr["heading_deg"].iloc[i]

        ref_lat = ref_slice[ref_lat_col].iloc[i]
        ref_lon = ref_slice[ref_lon_col].iloc[i]
        ref_e, ref_n, _ = geodetic_to_enu(ref_lat, ref_lon, 0.0, ref_origin)

        # Horizontal position error (meters)
        err = float(np.sqrt((dr_e - ref_e)**2 + (dr_n - ref_n)**2))
        errors_m.append(err)

        if ref_heading_col in ref_slice.columns and pd.notna(ref_slice[ref_heading_col].iloc[i]):
            ref_head = ref_slice[ref_heading_col].iloc[i]
            head_err = abs((dr_head - ref_head + 180.0) % 360.0 - 180.0)
            heading_errors_deg.append(float(head_err))
        else:
            heading_errors_deg.append(0.0)

    final_err = float(errors_m[-1]) if len(errors_m) > 0 else 0.0
    max_err = float(np.max(errors_m)) if len(errors_m) > 0 else 0.0
    mean_err = float(np.mean(errors_m)) if len(errors_m) > 0 else 0.0
    rmse_err = float(np.sqrt(np.mean(np.square(errors_m)))) if len(errors_m) > 0 else 0.0

    final_head_err = float(heading_errors_deg[-1]) if len(heading_errors_deg) > 0 else 0.0
    max_head_err = float(np.max(heading_errors_deg)) if len(heading_errors_deg) > 0 else 0.0

    drift_rate = final_err / traj_res.outage_duration_sec if traj_res.outage_duration_sec > 0 else 0.0

    return OutageMetricsResult(
        outage_duration_sec=traj_res.outage_duration_sec,
        outage_start_t=t_start,
        outage_end_t=t_end,
        sample_count=len(errors_m),
        final_position_error_m=final_err,
        max_position_error_m=max_err,
        mean_position_error_m=mean_err,
        rmse_position_error_m=rmse_err,
        final_heading_error_deg=final_head_err,
        max_heading_error_deg=max_head_err,
        drift_rate_m_per_sec=drift_rate,
        error_series_m=errors_m
    )

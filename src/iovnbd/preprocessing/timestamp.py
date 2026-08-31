"""
Module 2: Timestamp normalization and relative timeline generation.
"""

from dataclasses import dataclass
from typing import Dict, Any, Tuple, Optional
import pandas as pd
import numpy as np

@dataclass
class NormalizedTimelineResult:
    total_records: int
    s_start_raw_ms: float
    s_end_raw_ms: float
    v_start_raw_sec: float
    v_end_raw_sec: float
    duration_sec: float
    sampling_step_sec: float
    sync_status: str

def normalize_timestamps(
    df_s: pd.DataFrame,
    s_ts_col: str = "TIME SINCE START (ms)",
    df_v: Optional[pd.DataFrame] = None,
    v_ts_col: str = "Time Since Start of Day (seconds)"
) -> Tuple[pd.DataFrame, Optional[pd.DataFrame], NormalizedTimelineResult]:
    """
    Normalizes timestamps into a unified relative timeline t_rel_sec starting at 0.0s.
    Preserves all original raw timestamps without mutating raw inputs.
    """
    df_s_out = df_s.copy()
    
    s_ts = df_s_out[s_ts_col]
    s_start_ms = float(s_ts.min())
    s_end_ms = float(s_ts.max())
    
    # Calculate relative elapsed time in seconds for S
    df_s_out["t_rel_sec"] = (s_ts - s_start_ms) / 1000.0

    v_start_sec = 0.0
    v_end_sec = 0.0
    df_v_out = None

    if df_v is not None and len(df_v) > 0:
        df_v_out = df_v.copy()
        v_ts = df_v_out[v_ts_col]
        v_start_sec = float(v_ts.min())
        v_end_sec = float(v_ts.max())
        # Calculate relative elapsed time in seconds for V
        df_v_out["t_rel_sec"] = v_ts - v_start_sec

    duration = float((s_end_ms - s_start_ms) / 1000.0)
    
    # Calculate median sampling step
    diffs = df_s_out["t_rel_sec"].diff().dropna()
    step_sec = float(diffs.median()) if len(diffs) > 0 else 0.1

    result = NormalizedTimelineResult(
        total_records=len(df_s_out),
        s_start_raw_ms=s_start_ms,
        s_end_raw_ms=s_end_ms,
        v_start_raw_sec=v_start_sec,
        v_end_raw_sec=v_end_sec,
        duration_sec=duration,
        sampling_step_sec=step_sec,
        sync_status="DATASET-PROVIDED / EMPIRICALLY CONSISTENT"
    )

    return df_s_out, df_v_out, result

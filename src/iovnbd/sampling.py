"""
Sampling rate diagnostics and timestamp interval analysis.
"""

from dataclasses import dataclass
from typing import Dict, Any, Optional
import pandas as pd
import numpy as np

@dataclass
class SamplingAnalysisResult:
    stream_name: str
    record_count: int
    start_time: Optional[float]
    end_time: Optional[float]
    duration_seconds: Optional[float]
    median_interval_sec: Optional[float]
    mean_interval_sec: Optional[float]
    min_interval_sec: Optional[float]
    max_interval_sec: Optional[float]
    effective_frequency_hz: Optional[float]
    documented_frequency_hz: float
    gap_count: int

def analyze_sampling_rate(df: pd.DataFrame, timestamp_col: str, stream_name: str, documented_hz: float = 10.0) -> SamplingAnalysisResult:
    if timestamp_col not in df.columns or len(df) < 2:
        return SamplingAnalysisResult(
            stream_name=stream_name,
            record_count=len(df),
            start_time=None,
            end_time=None,
            duration_seconds=None,
            median_interval_sec=None,
            mean_interval_sec=None,
            min_interval_sec=None,
            max_interval_sec=None,
            effective_frequency_hz=None,
            documented_frequency_hz=documented_hz,
            gap_count=0
        )

    ts = df[timestamp_col].dropna()
    max_val = ts.max()
    min_val = ts.min()
    
    # Scale detection: if values > 1,000,000 treat as ms (e.g. TIME SINCE START (ms)), else seconds
    is_ms = (max_val > 1e6 or "ms" in timestamp_col.lower())
    scale = 1000.0 if is_ms else 1.0

    ts_sec = ts / scale
    diffs = ts_sec.diff().dropna()
    
    start_time = float(min_val / scale)
    end_time = float(max_val / scale)
    duration = float(end_time - start_time)

    med_int = float(diffs.median()) if len(diffs) > 0 else 0.0
    mean_int = float(diffs.mean()) if len(diffs) > 0 else 0.0
    min_int = float(diffs.min()) if len(diffs) > 0 else 0.0
    max_int = float(diffs.max()) if len(diffs) > 0 else 0.0

    eff_hz = (1.0 / med_int) if med_int > 0 else 0.0
    gaps = len(diffs[diffs > 1.0])

    return SamplingAnalysisResult(
        stream_name=stream_name,
        record_count=len(df),
        start_time=start_time,
        end_time=end_time,
        duration_seconds=duration,
        median_interval_sec=med_int,
        mean_interval_sec=mean_int,
        min_interval_sec=min_int,
        max_interval_sec=max_int,
        effective_frequency_hz=eff_hz,
        documented_frequency_hz=documented_hz,
        gap_count=gaps
    )

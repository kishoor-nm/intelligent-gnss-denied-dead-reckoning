"""
Validation module for dataset integrity checks.
"""

from dataclasses import dataclass
from typing import Dict, Any, List, Optional
import pandas as pd
import numpy as np

@dataclass
class ValidationReport:
    total_records: int
    missing_value_counts: Dict[str, int]
    nan_value_counts: Dict[str, int]
    infinite_value_counts: Dict[str, int]
    duplicate_timestamp_count: int
    non_monotonic_timestamp_count: int
    timestamp_gaps_count: int
    suspicious_gaps: List[Dict[str, Any]]
    warnings: List[str]
    failures: List[str]
    status: str  # PASS / WARNINGS / FAILURES

def validate_stream(df: pd.DataFrame, timestamp_col: Optional[str] = None, gap_threshold_sec: float = 1.0) -> ValidationReport:
    warnings = []
    failures = []
    
    total_records = len(df)
    if total_records == 0:
        failures.append("DataFrame contains 0 records.")
        return ValidationReport(
            total_records=0,
            missing_value_counts={},
            nan_value_counts={},
            infinite_value_counts={},
            duplicate_timestamp_count=0,
            non_monotonic_timestamp_count=0,
            timestamp_gaps_count=0,
            suspicious_gaps=[],
            warnings=warnings,
            failures=failures,
            status="FAILURES"
        )

    # Missing & NaN counts
    null_counts = df.isnull().sum().to_dict()
    nan_counts = {}
    inf_counts = {}
    
    for col in df.columns:
        if pd.api.types.is_numeric_dtype(df[col]):
            nan_counts[col] = int(df[col].isna().sum())
            inf_counts[col] = int(np.isinf(df[col]).sum()) if hasattr(np, 'isinf') else 0
            if inf_counts[col] > 0:
                warnings.append(f"Column '{col}' contains {inf_counts[col]} infinite values.")
        else:
            nan_counts[col] = int(df[col].isna().sum())
            inf_counts[col] = 0

        if null_counts[col] > 0:
            warnings.append(f"Column '{col}' contains {null_counts[col]} missing/null values.")

    # Timestamp checks
    dup_ts = 0
    non_mono_ts = 0
    gaps_count = 0
    suspicious_gaps = []

    if timestamp_col and timestamp_col in df.columns:
        ts_series = df[timestamp_col].dropna()
        dup_ts = int(ts_series.duplicated().sum())
        if dup_ts > 0:
            warnings.append(f"Timestamp column '{timestamp_col}' contains {dup_ts} duplicate timestamps.")

        # Check monotonicity
        diffs = ts_series.diff().dropna()
        non_mono = diffs[diffs < 0]
        non_mono_ts = len(non_mono)
        if non_mono_ts > 0:
            failures.append(f"Timestamp column '{timestamp_col}' has {non_mono_ts} non-monotonic (decreasing) steps.")

        # Gaps check (convert ms or seconds if needed)
        # Determine scale: if max ts > 1e6 assume ms, else seconds
        max_v = ts_series.max()
        scale = 1000.0 if max_v > 1e6 else 1.0
        
        diffs_sec = diffs / scale
        gaps = diffs_sec[diffs_sec > gap_threshold_sec]
        gaps_count = len(gaps)
        for idx, gap_val in gaps.items():
            suspicious_gaps.append({
                "index": int(idx),
                "timestamp": float(ts_series.loc[idx]),
                "gap_seconds": float(gap_val)
            })
        if gaps_count > 0:
            warnings.append(f"Found {gaps_count} time gaps > {gap_threshold_sec}s in '{timestamp_col}'.")

    status = "FAILURES" if len(failures) > 0 else ("WARNINGS" if len(warnings) > 0 else "PASS")

    return ValidationReport(
        total_records=total_records,
        missing_value_counts=null_counts,
        nan_value_counts=nan_counts,
        infinite_value_counts=inf_counts,
        duplicate_timestamp_count=dup_ts,
        non_monotonic_timestamp_count=non_mono_ts,
        timestamp_gaps_count=gaps_count,
        suspicious_gaps=suspicious_gaps,
        warnings=warnings,
        failures=failures,
        status=status
    )

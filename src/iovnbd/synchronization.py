"""
Synchronization and temporal overlap analysis module.
"""

from dataclasses import dataclass
from typing import Dict, Any, Optional
import pandas as pd

@dataclass
class SynchronizationReport:
    stream_s_name: str
    stream_v_name: str
    s_duration_sec: Optional[float]
    v_duration_sec: Optional[float]
    overlap_duration_sec: Optional[float]
    time_basis_shared: bool
    record_count_ratio_s_vs_v: Optional[float]
    findings: str

def analyze_stream_synchronization(
    df_s: Optional[pd.DataFrame],
    ts_s_col: str,
    df_v: Optional[pd.DataFrame],
    ts_v_col: str
) -> SynchronizationReport:
    if df_s is None or df_v is None or len(df_s) == 0 or len(df_v) == 0:
        return SynchronizationReport(
            stream_s_name="Smartphone (S)",
            stream_v_name="Vehicle (V)",
            s_duration_sec=None,
            v_duration_sec=None,
            overlap_duration_sec=None,
            time_basis_shared=False,
            record_count_ratio_s_vs_v=None,
            findings="One or both streams unavailable for synchronization analysis."
        )

    # Smartphone ts scale
    ts_s = df_s[ts_s_col].dropna()
    scale_s = 1000.0 if ts_s.max() > 1e6 else 1.0
    dur_s = float((ts_s.max() - ts_s.min()) / scale_s)

    # Vehicle ts scale
    ts_v = df_v[ts_v_col].dropna()
    scale_v = 1000.0 if ts_v.max() > 1e6 else 1.0
    dur_v = float((ts_v.max() - ts_v.min()) / scale_v)

    ratio = len(df_s) / len(df_v)

    # Check time basis
    # Smartphone: 'TIME SINCE START (ms)', Vehicle: 'Time Since Start of Day (seconds)'
    # Note: V is seconds from midnight UTC/local, S is elapsed ms since app start.
    shared_basis = False
    findings = (
        f"Smartphone stream elapsed duration: {dur_s:.2f}s ({len(df_s)} records).\n"
        f"Vehicle ECU stream elapsed duration: {dur_v:.2f}s ({len(df_v)} records).\n"
        f"Record count ratio (S/V): {ratio:.4f}.\n"
        f"Time Basis: S uses relative elapsed ms ('TIME SINCE START (ms)'), V uses seconds from midnight ('Time Since Start of Day (seconds)').\n"
        f"Temporal Alignment: Both streams cover ~{min(dur_s, dur_v):.2f}s, but require timestamp offset calibration in Module 2."
    )

    return SynchronizationReport(
        stream_s_name="Smartphone (S-S1)",
        stream_v_name="Vehicle (V-S1)",
        s_duration_sec=dur_s,
        v_duration_sec=dur_v,
        overlap_duration_sec=min(dur_s, dur_v),
        time_basis_shared=shared_basis,
        record_count_ratio_s_vs_v=ratio,
        findings=findings
    )

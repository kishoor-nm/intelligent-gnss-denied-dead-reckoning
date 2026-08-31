"""
Module 7: Experiment Runner & Comparative Plotter for Confidence-Aware Intelligent Sensor Fusion.
Executes M5.1 Baseline, M6 Naive AI-EKF, and M7 Confidence-Aware Adaptive Fusion across 10s, 30s, 60s, 120s outages.
Generates comparative plots and machine-readable JSON results in d:/prototype/results/module7/.
"""

import os
import time
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from typing import List, Dict, Any

from src.iovnbd.intelligence.dataset import load_sequence_dataset, FEATURE_COLUMNS
from src.iovnbd.intelligence.model import RidgeLinearRegressor
from src.iovnbd.navigation.initialization import initialize_navigation_state
from src.iovnbd.navigation.baseline import propagate_dead_reckoning_baseline, TrajectoryResult
from src.iovnbd.navigation.ekf_m5_1 import propagate_ekf_m5_1
from src.iovnbd.navigation.ekf_m6 import propagate_ekf_m6
from src.iovnbd.fusion.ekf_m7 import propagate_ekf_m7_confidence, EKFResultM7
from src.iovnbd.navigation.metrics import calculate_outage_error_metrics, OutageMetricsResult
from src.iovnbd.navigation.coordinate_frame import AnchorOrigin, geodetic_to_enu

def run_module7_experiment_suite(
    output_dir: str = "d:/prototype/results/module7",
    outage_durations: List[float] = [10.0, 30.0, 60.0, 120.0],
    start_idx: int = 1000
) -> Dict[str, Any]:
    """
    Executes full Module 7 confidence-aware sensor fusion experiment suite.
    """
    os.makedirs(output_dir, exist_ok=True)
    t_start_wall = time.time()

    # Train Ridge model on S2/S3a/S4
    train_seqs = [
        ("d:/prototype/IO-VNBD-master/Synchronised V abd S datasets/Categorised IOVNB Dataset/S (Driver A)/S2/V-S2.csv",
         "d:/prototype/IO-VNBD-master/Synchronised V abd S datasets/Categorised IOVNB Dataset/S (Driver A)/S2/S-S2.csv"),
        ("d:/prototype/IO-VNBD-master/Synchronised V abd S datasets/Categorised IOVNB Dataset/S (Driver A)/S3a/V-S3a.csv",
         "d:/prototype/IO-VNBD-master/Synchronised V abd S datasets/Categorised IOVNB Dataset/S (Driver A)/S3a/S-S3a.csv"),
        ("d:/prototype/IO-VNBD-master/Synchronised V abd S datasets/Categorised IOVNB Dataset/S (Driver A)/S4/V-S4.csv",
         "d:/prototype/IO-VNBD-master/Synchronised V abd S datasets/Categorised IOVNB Dataset/S (Driver A)/S4/S-S4.csv")
    ]

    X_train_list, y_train_list = [], []
    for v_p, s_p in train_seqs:
        if os.path.exists(v_p) and os.path.exists(s_p):
            f, t = load_sequence_dataset(v_p, s_p)
            X_train_list.append(f[FEATURE_COLUMNS].values)
            y_train_list.append(t.values)

    X_train = np.vstack(X_train_list)
    y_train = np.concatenate(y_train_list)

    m_ridge = RidgeLinearRegressor(alpha=10.0)
    m_ridge.fit(X_train, y_train)

    df_v = pd.read_csv("d:/prototype/data/processed/S1/V-S1_processed.csv")
    df_s = pd.read_csv("d:/prototype/data/processed/S1/S-S1_processed.csv")

    init_state = initialize_navigation_state(df_v, start_idx=start_idx)

    suite_results = {
        "sequence": "S1",
        "start_idx": start_idx,
        "method": "Module 7 Confidence-Aware Adaptive Sensor Fusion EKF",
        "confidence_mechanism": "Normalized Innovation Squared (NIS) Residual Gating + Adaptive Measurement Variance Scaling R_adaptive = R_base * (1 + NIS)",
        "experiments": []
    }

    # Loop over outage durations
    for dur in outage_durations:
        t0_exp = time.time()

        # M5.1 Baseline
        res_m5_1 = propagate_ekf_m5_1(df_v, init_state, start_idx, dur)
        met_m5_1 = calculate_outage_error_metrics(TrajectoryResult(points=res_m5_1.points, dataframe=res_m5_1.dataframe, outage_start_t=res_m5_1.outage_start_t, outage_end_t=res_m5_1.outage_end_t, outage_duration_sec=dur), df_v)

        # M6 Naive AI-EKF
        res_m6 = propagate_ekf_m6(df_v, df_s, init_state, start_idx, dur, ml_model=m_ridge, feature_cols=FEATURE_COLUMNS)
        met_m6 = calculate_outage_error_metrics(TrajectoryResult(points=res_m6.points, dataframe=res_m6.dataframe, outage_start_t=res_m6.outage_start_t, outage_end_t=res_m6.outage_end_t, outage_duration_sec=dur), df_v)

        # M7 Confidence-Aware Adaptive EKF
        res_m7 = propagate_ekf_m7_confidence(df_v, df_s, init_state, start_idx, dur, ml_model=m_ridge, feature_cols=FEATURE_COLUMNS, nis_gate_threshold=3.0)
        met_m7 = calculate_outage_error_metrics(TrajectoryResult(points=res_m7.points, dataframe=res_m7.dataframe, outage_start_t=res_m7.outage_start_t, outage_end_t=res_m7.outage_end_t, outage_duration_sec=dur), df_v)

        t_elapsed = time.time() - t0_exp

        pct_trusted = round((res_m7.trusted_count / len(res_m7.points)) * 100.0, 1)
        pct_gated = round((res_m7.gated_count / len(res_m7.points)) * 100.0, 1)

        exp_data = {
            "outage_duration_sec": dur,
            "sample_count": met_m5_1.sample_count,
            "m5_1_ekf_rmse_m": round(met_m5_1.rmse_position_error_m, 2),
            "m6_naive_ai_rmse_m": round(met_m6.rmse_position_error_m, 2),
            "m7_adaptive_ai_rmse_m": round(met_m7.rmse_position_error_m, 2),
            "m5_1_ekf_final_m": round(met_m5_1.final_position_error_m, 2),
            "m6_naive_ai_final_m": round(met_m6.final_position_error_m, 2),
            "m7_adaptive_ai_final_m": round(met_m7.final_position_error_m, 2),
            "pct_time_trusted": pct_trusted,
            "pct_time_gated": pct_gated,
            "runtime_ms": round(t_elapsed * 1000.0, 2)
        }
        suite_results["experiments"].append(exp_data)

    # Plot 1: Trajectory Comparison (60s Outage)
    ref_slice = df_v[(df_v["t_rel_sec"] >= init_state.t_rel_sec - 1e-5) & (df_v["t_rel_sec"] <= init_state.t_rel_sec + 70.0)]
    lat0, lon0 = init_state.lat_deg, init_state.lon_deg
    ref_origin = AnchorOrigin(lat0_rad=float(np.radians(lat0)), lon0_rad=float(np.radians(lon0)), alt0_m=0.0)
    ref_e, ref_n = [], []
    for _, r in ref_slice.iterrows():
        e, n, _ = geodetic_to_enu(r["Latitude (degrees)"], r["Longitude (degrees)"], 0.0, ref_origin)
        ref_e.append(e)
        ref_n.append(n)

    plt.figure(figsize=(10, 6))
    plt.plot(ref_e, ref_n, 'k--', linewidth=2.0, label='Reference (VBOX GNSS)')

    res_m5_1_60 = propagate_ekf_m5_1(df_v, init_state, start_idx, 60.0)
    res_m6_60 = propagate_ekf_m6(df_v, df_s, init_state, start_idx, 60.0, ml_model=m_ridge, feature_cols=FEATURE_COLUMNS)
    res_m7_60 = propagate_ekf_m7_confidence(df_v, df_s, init_state, start_idx, 60.0, ml_model=m_ridge, feature_cols=FEATURE_COLUMNS, nis_gate_threshold=3.0)

    plt.plot(res_m5_1_60.dataframe["east_m"], res_m5_1_60.dataframe["north_m"], color='#1f77b4', linewidth=2.0, label='M5.1 EKF (ECU Speed)')
    plt.plot(res_m6_60.dataframe["east_m"], res_m6_60.dataframe["north_m"], color='#d62728', linestyle='--', linewidth=2.0, label='M6 Naïve AI-EKF')
    plt.plot(res_m7_60.dataframe["east_m"], res_m7_60.dataframe["north_m"], color='#2ca02c', linestyle=':', linewidth=2.5, label='M7 Adaptive AI-EKF')

    plt.scatter(0, 0, color='blue', s=80, label='GNSS Outage Start', zorder=5)
    plt.title("GNSS Outage Trajectory: M5.1 vs M6 Naïve AI vs M7 Adaptive AI (60s Outage)")
    plt.xlabel("East (meters)")
    plt.ylabel("North (meters)")
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend()
    plt.tight_layout()

    plt.savefig(os.path.join(output_dir, "m7_trajectory_comparison.png"), dpi=150)
    plt.close()

    # Plot 2: NIS Confidence & Adaptive R Variance over 120s Outage
    res_m7_120 = propagate_ekf_m7_confidence(df_v, df_s, init_state, start_idx, 120.0, ml_model=m_ridge, feature_cols=FEATURE_COLUMNS, nis_gate_threshold=3.0)
    df_m7_120 = res_m7_120.dataframe

    fig, ax1 = plt.subplots(figsize=(10, 5))

    t_axis = df_m7_120["t_rel_sec"] - df_m7_120["t_rel_sec"].iloc[0]

    color = '#1f77b4'
    ax1.set_xlabel('Outage Duration (seconds)')
    ax1.set_ylabel('Normalized Innovation Squared (NIS)', color=color)
    ax1.plot(t_axis, df_m7_120["nis_score"], color=color, linewidth=1.5, label='NIS Residual Score')
    ax1.axhline(y=3.0, color='red', linestyle='--', label='NIS Gate Threshold (3.0)')
    ax1.tick_params(axis='y', labelcolor=color)

    ax2 = ax1.twinx()
    color = '#2ca02c'
    ax2.set_ylabel('Adaptive Measurement Variance (R)', color=color)
    ax2.plot(t_axis, df_m7_120["r_adaptive"], color=color, linestyle=':', linewidth=2.0, label='Adaptive R')
    ax2.tick_params(axis='y', labelcolor=color)

    plt.title("Module 7 Real-Time Innovation Confidence & Noise Covariance Adaptation")
    fig.tight_layout()
    plt.savefig(os.path.join(output_dir, "m7_confidence_adaptation.png"), dpi=150)
    plt.close()

    total_wall_time = time.time() - t_start_wall
    suite_results["total_runtime_seconds"] = round(total_wall_time, 3)

    json_path = os.path.join(output_dir, "module7_results.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(suite_results, f, indent=2)

    return suite_results

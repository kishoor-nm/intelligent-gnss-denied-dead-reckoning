"""
Module 8: Experiment & Ablation Runner for 5D NHC-Enhanced Kinematic EKF.
Executes locked M4.2 canonical evaluation on sequence S1 (10s, 30s, 60s, 120s outages).
Compares:
- M3 Baseline DR
- M5.1 Baseline EKF
- M8 Ablation A: Speed Update Only
- M8 Ablation B: Speed + 4-Wheel Speed Odometry
- M8 Ablation C: Speed + NHC Lateral Acceleration
- M8 Full System: Speed + Wheel Odometry + NHC
Generates comparative plots and machine-readable JSON results in d:/prototype/results/module8/.
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

from src.iovnbd.navigation.initialization import initialize_navigation_state
from src.iovnbd.navigation.baseline import propagate_dead_reckoning_baseline, TrajectoryResult
from src.iovnbd.navigation.ekf_m5_1 import propagate_ekf_m5_1
from src.iovnbd.navigation.ekf_m6 import propagate_ekf_m6
from src.iovnbd.fusion.ekf_m7 import propagate_ekf_m7_confidence
from src.iovnbd.intelligence.dataset import load_sequence_dataset, FEATURE_COLUMNS
from src.iovnbd.intelligence.model import RidgeLinearRegressor
from src.iovnbd.navigation.ekf_m8 import propagate_ekf_m8, EKFResultM8
from src.iovnbd.navigation.metrics import calculate_outage_error_metrics, OutageMetricsResult
from src.iovnbd.navigation.coordinate_frame import AnchorOrigin, geodetic_to_enu

def run_module8_experiment_suite(
    output_dir: str = "d:/prototype/results/module8",
    outage_durations: List[float] = [10.0, 30.0, 60.0, 120.0],
    start_idx: int = 1000
) -> Dict[str, Any]:
    """
    Executes full Module 8 experiment and ablation suite.
    """
    os.makedirs(output_dir, exist_ok=True)
    t_start_wall = time.time()

    df_v = pd.read_csv("d:/prototype/data/processed/S1/V-S1_processed.csv")
    df_s = pd.read_csv("d:/prototype/data/processed/S1/S-S1_processed.csv")

    init_state = initialize_navigation_state(df_v, start_idx=start_idx)

    suite_results = {
        "sequence": "S1",
        "start_idx": start_idx,
        "method": "Module 8 5D Non-Holonomic Kinematic Constraint (NHC) Enhanced EKF",
        "experiments": [],
        "ablations": []
    }

    # Loop over outage durations for primary comparison
    for dur in outage_durations:
        t0_exp = time.time()

        # 1. M3 Baseline
        res_m3 = propagate_dead_reckoning_baseline(df_v, init_state, start_idx, dur)
        met_m3 = calculate_outage_error_metrics(res_m3, df_v)

        # 2. M5.1 Baseline
        res_m5_1 = propagate_ekf_m5_1(df_v, init_state, start_idx, dur)
        met_m5_1 = calculate_outage_error_metrics(TrajectoryResult(points=res_m5_1.points, dataframe=res_m5_1.dataframe, outage_start_t=res_m5_1.outage_start_t, outage_end_t=res_m5_1.outage_end_t, outage_duration_sec=dur), df_v)

        # 3. M8 Full System (Speed + Wheel + NHC)
        res_m8_full = propagate_ekf_m8(df_v, init_state, start_idx, dur, enable_wheel_speed=True, enable_nhc=True)
        met_m8_full = calculate_outage_error_metrics(TrajectoryResult(points=res_m8_full.points, dataframe=res_m8_full.dataframe, outage_start_t=res_m8_full.outage_start_t, outage_end_t=res_m8_full.outage_end_t, outage_duration_sec=dur), df_v)

        # 4. M8 NHC-Only (Speed + NHC)
        res_m8_nhc = propagate_ekf_m8(df_v, init_state, start_idx, dur, enable_wheel_speed=False, enable_nhc=True)
        met_m8_nhc = calculate_outage_error_metrics(TrajectoryResult(points=res_m8_nhc.points, dataframe=res_m8_nhc.dataframe, outage_start_t=res_m8_nhc.outage_start_t, outage_end_t=res_m8_nhc.outage_end_t, outage_duration_sec=dur), df_v)

        t_elapsed = time.time() - t0_exp

        diff_rmse = met_m8_full.rmse_position_error_m - met_m5_1.rmse_position_error_m
        pct_improvement = ((met_m5_1.rmse_position_error_m - met_m8_nhc.rmse_position_error_m) / met_m5_1.rmse_position_error_m) * 100.0

        exp_data = {
            "outage_duration_sec": dur,
            "sample_count": met_m5_1.sample_count,
            "m3_baseline_rmse_m": round(met_m3.rmse_position_error_m, 2),
            "m5_1_ekf_rmse_m": round(met_m5_1.rmse_position_error_m, 2),
            "m8_full_ekf_rmse_m": round(met_m8_full.rmse_position_error_m, 2),
            "m8_nhc_ekf_rmse_m": round(met_m8_nhc.rmse_position_error_m, 2),
            "m5_1_ekf_final_m": round(met_m5_1.final_position_error_m, 2),
            "m8_full_ekf_final_m": round(met_m8_full.final_position_error_m, 2),
            "m8_nhc_ekf_final_m": round(met_m8_nhc.final_position_error_m, 2),
            "m8_vs_m5_1_pct_improvement": round(pct_improvement, 1),
            "nhc_accepted_count": res_m8_full.nhc_accepted_count,
            "nhc_rejected_count": res_m8_full.nhc_rejected_count,
            "runtime_ms": round(t_elapsed * 1000.0, 2)
        }
        suite_results["experiments"].append(exp_data)

    # Detailed Ablation Breakdown on 120s Outage
    dur_120 = 120.0
    # Ablation A: ECU Speed Only
    res_ab_a = propagate_ekf_m8(df_v, init_state, start_idx, dur_120, enable_wheel_speed=False, enable_nhc=False)
    met_ab_a = calculate_outage_error_metrics(TrajectoryResult(points=res_ab_a.points, dataframe=res_ab_a.dataframe, outage_start_t=res_ab_a.outage_start_t, outage_end_t=res_ab_a.outage_end_t, outage_duration_sec=dur_120), df_v)

    # Ablation B: ECU Speed + 4-Wheel Speed
    res_ab_b = propagate_ekf_m8(df_v, init_state, start_idx, dur_120, enable_wheel_speed=True, enable_nhc=False)
    met_ab_b = calculate_outage_error_metrics(TrajectoryResult(points=res_ab_b.points, dataframe=res_ab_b.dataframe, outage_start_t=res_ab_b.outage_start_t, outage_end_t=res_ab_b.outage_end_t, outage_duration_sec=dur_120), df_v)

    # Ablation C: ECU Speed + NHC
    res_ab_c = propagate_ekf_m8(df_v, init_state, start_idx, dur_120, enable_wheel_speed=False, enable_nhc=True)
    met_ab_c = calculate_outage_error_metrics(TrajectoryResult(points=res_ab_c.points, dataframe=res_ab_c.dataframe, outage_start_t=res_ab_c.outage_start_t, outage_end_t=res_ab_c.outage_end_t, outage_duration_sec=dur_120), df_v)

    # Ablation D: Full System (ECU Speed + 4-Wheel Speed + NHC)
    res_ab_d = propagate_ekf_m8(df_v, init_state, start_idx, dur_120, enable_wheel_speed=True, enable_nhc=True)
    met_ab_d = calculate_outage_error_metrics(TrajectoryResult(points=res_ab_d.points, dataframe=res_ab_d.dataframe, outage_start_t=res_ab_d.outage_start_t, outage_end_t=res_ab_d.outage_end_t, outage_duration_sec=dur_120), df_v)

    suite_results["ablations"] = [
        {"name": "Ablation A (ECU Speed Only)", "rmse_m": round(met_ab_a.rmse_position_error_m, 2), "final_m": round(met_ab_a.final_position_error_m, 2)},
        {"name": "Ablation B (ECU Speed + 4-Wheel Odometry)", "rmse_m": round(met_ab_b.rmse_position_error_m, 2), "final_m": round(met_ab_b.final_position_error_m, 2)},
        {"name": "Ablation C (ECU Speed + NHC Constraint)", "rmse_m": round(met_ab_c.rmse_position_error_m, 2), "final_m": round(met_ab_c.final_position_error_m, 2)},
        {"name": "Ablation D (Full: ECU + Wheel + NHC)", "rmse_m": round(met_ab_d.rmse_position_error_m, 2), "final_m": round(met_ab_d.final_position_error_m, 2)}
    ]

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
    res_m8_nhc_60 = propagate_ekf_m8(df_v, init_state, start_idx, 60.0, enable_wheel_speed=False, enable_nhc=True)
    res_m8_full_60 = propagate_ekf_m8(df_v, init_state, start_idx, 60.0, enable_wheel_speed=True, enable_nhc=True)

    plt.plot(res_m5_1_60.dataframe["east_m"], res_m5_1_60.dataframe["north_m"], color='#1f77b4', linewidth=2.0, label='M5.1 EKF (CAN Speed)')
    plt.plot(res_m8_nhc_60.dataframe["east_m"], res_m8_nhc_60.dataframe["north_m"], color='#2ca02c', linestyle='-', linewidth=2.5, label='M8 NHC-Enhanced EKF')
    plt.plot(res_m8_full_60.dataframe["east_m"], res_m8_full_60.dataframe["north_m"], color='#9467bd', linestyle=':', linewidth=2.0, label='M8 Full EKF (Wheel + NHC)')

    plt.scatter(0, 0, color='blue', s=80, label='GNSS Outage Start', zorder=5)
    plt.title("GNSS Outage Trajectory Comparison: M5.1 vs M8 NHC EKF (60s Outage)")
    plt.xlabel("East (meters)")
    plt.ylabel("North (meters)")
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend()
    plt.tight_layout()

    plt.savefig(os.path.join(output_dir, "m8_trajectory_comparison.png"), dpi=150)
    plt.close()

    # Plot 2: Position Error Growth over 120s Outage
    res_m5_1_120 = propagate_ekf_m5_1(df_v, init_state, start_idx, 120.0)
    res_m8_nhc_120 = propagate_ekf_m8(df_v, init_state, start_idx, 120.0, enable_wheel_speed=False, enable_nhc=True)

    ref_120 = df_v[(df_v["t_rel_sec"] >= init_state.t_rel_sec - 1e-5) & (df_v["t_rel_sec"] <= init_state.t_rel_sec + 120.0 + 1e-5)].reset_index(drop=True)
    err_m5_1, err_m8_nhc = [], []

    for i in range(len(res_m5_1_120.points)):
        r_lat, r_lon = ref_120["Latitude (degrees)"].iloc[i], ref_120["Longitude (degrees)"].iloc[i]
        re, rn, _ = geodetic_to_enu(r_lat, r_lon, 0.0, ref_origin)

        e5, n5 = res_m5_1_120.points[i].east_m, res_m5_1_120.points[i].north_m
        e8, n8 = res_m8_nhc_120.points[i].east_m, res_m8_nhc_120.points[i].north_m

        err_m5_1.append(np.sqrt((e5 - re)**2 + (n5 - rn)**2))
        err_m8_nhc.append(np.sqrt((e8 - re)**2 + (n8 - rn)**2))

    t_axis = ref_120["t_rel_sec"] - ref_120["t_rel_sec"].iloc[0]

    plt.figure(figsize=(10, 5))
    plt.plot(t_axis, err_m5_1, color='#1f77b4', linewidth=2.0, label='M5.1 EKF (CAN Speed)')
    plt.plot(t_axis, err_m8_nhc, color='#2ca02c', linewidth=2.5, label='M8 NHC-Enhanced EKF')
    plt.title("Position Error Growth vs Outage Duration (120s Outage)")
    plt.xlabel("Outage Duration (seconds)")
    plt.ylabel("Position Error (meters)")
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend()
    plt.tight_layout()

    plt.savefig(os.path.join(output_dir, "m8_position_error_growth.png"), dpi=150)
    plt.close()

    total_wall_time = time.time() - t_start_wall
    suite_results["total_runtime_seconds"] = round(total_wall_time, 3)

    json_path = os.path.join(output_dir, "module8_results.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(suite_results, f, indent=2)

    return suite_results

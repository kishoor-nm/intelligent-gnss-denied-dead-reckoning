"""
Module 9: Experiment & Ablation Runner for 6D Full-Orientation Kinematic EKF.
Executes locked M4.2 canonical evaluation on sequence S1 (10s, 30s, 60s, 120s outages).
Ablations:
- A: M5.1 Baseline (CAN Speed EKF)
- B: M8 Baseline (5D Uncompensated Roll NHC EKF)
- C: M9 (K_roll_restore = 0.0)
- D: M9 (K_roll_restore = 0.05)
- E: M9 (K_roll_restore = 0.10)
- F: M9 (K_roll_restore = 0.20)
- G: M9 Full System (Gyro Roll Integration + Gravity Compensation)
Generates comparative plots and machine-readable JSON results in d:/prototype/results/module9/.
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
from src.iovnbd.navigation.ekf_m8 import propagate_ekf_m8
from src.iovnbd.navigation.ekf_m9 import propagate_ekf_m9, EKFResultM9
from src.iovnbd.navigation.metrics import calculate_outage_error_metrics, OutageMetricsResult
from src.iovnbd.navigation.coordinate_frame import AnchorOrigin, geodetic_to_enu

def run_module9_experiment_suite(
    output_dir: str = "d:/prototype/results/module9",
    outage_durations: List[float] = [10.0, 30.0, 60.0, 120.0],
    start_idx: int = 1000
) -> Dict[str, Any]:
    """
    Executes full Module 9 experiment and ablation suite.
    """
    os.makedirs(output_dir, exist_ok=True)
    t_start_wall = time.time()

    df_v = pd.read_csv("d:/prototype/data/processed/S1/V-S1_processed.csv")
    df_s = pd.read_csv("d:/prototype/data/processed/S1/S-S1_processed.csv")

    init_state = initialize_navigation_state(df_v, start_idx=start_idx)

    suite_results = {
        "sequence": "S1",
        "start_idx": start_idx,
        "method": "Module 9 6D Full-Orientation Kinematic EKF",
        "experiments": [],
        "ablations": []
    }

    # Primary Benchmark Loop over outage durations
    for dur in outage_durations:
        t0_exp = time.time()

        # 1. M5.1 Baseline
        res_m5_1 = propagate_ekf_m5_1(df_v, init_state, start_idx, dur)
        met_m5_1 = calculate_outage_error_metrics(TrajectoryResult(points=res_m5_1.points, dataframe=res_m5_1.dataframe, outage_start_t=res_m5_1.outage_start_t, outage_end_t=res_m5_1.outage_end_t, outage_duration_sec=dur), df_v)

        # 2. M8 Baseline
        res_m8 = propagate_ekf_m8(df_v, init_state, start_idx, dur, enable_wheel_speed=False, enable_nhc=True)
        met_m8 = calculate_outage_error_metrics(TrajectoryResult(points=res_m8.points, dataframe=res_m8.dataframe, outage_start_t=res_m8.outage_start_t, outage_end_t=res_m8.outage_end_t, outage_duration_sec=dur), df_v)

        # 3. M9 (Best Config: K_roll_restore = 0.10)
        res_m9 = propagate_ekf_m9(df_v, df_s, init_state, start_idx, dur, k_roll_restore=0.10, enable_nhc=True)
        met_m9 = calculate_outage_error_metrics(TrajectoryResult(points=res_m9.points, dataframe=res_m9.dataframe, outage_start_t=res_m9.outage_start_t, outage_end_t=res_m9.outage_end_t, outage_duration_sec=dur), df_v)

        t_elapsed = time.time() - t0_exp

        # Classification against M5.1 Baseline
        diff_rmse_m5_1 = met_m9.rmse_position_error_m - met_m5_1.rmse_position_error_m
        if met_m9.rmse_position_error_m < met_m5_1.rmse_position_error_m - 0.1:
            status_vs_m5_1 = "IMPROVED"
        elif abs(diff_rmse_m5_1) <= 0.1:
            status_vs_m5_1 = "NEUTRAL"
        else:
            status_vs_m5_1 = "DEGRADED"

        # Classification against M8 Baseline
        diff_rmse_m8 = met_m9.rmse_position_error_m - met_m8.rmse_position_error_m
        if met_m9.rmse_position_error_m < met_m8.rmse_position_error_m - 0.1:
            status_vs_m8 = "IMPROVED"
        elif abs(diff_rmse_m8) <= 0.1:
            status_vs_m8 = "NEUTRAL"
        else:
            status_vs_m8 = "DEGRADED"

        exp_data = {
            "outage_duration_sec": dur,
            "sample_count": met_m5_1.sample_count,
            "m5_1_ekf_rmse_m": round(met_m5_1.rmse_position_error_m, 2),
            "m8_ekf_rmse_m": round(met_m8.rmse_position_error_m, 2),
            "m9_ekf_rmse_m": round(met_m9.rmse_position_error_m, 2),
            "m5_1_ekf_final_m": round(met_m5_1.final_position_error_m, 2),
            "m8_ekf_final_m": round(met_m8.final_position_error_m, 2),
            "m9_ekf_final_m": round(met_m9.final_position_error_m, 2),
            "status_vs_m5_1": status_vs_m5_1,
            "status_vs_m8": status_vs_m8,
            "nhc_accepted_count": res_m9.nhc_accepted_count,
            "nhc_rejected_residual_count": res_m9.nhc_rejected_residual_count,
            "runtime_ms": round(t_elapsed * 1000.0, 2)
        }
        suite_results["experiments"].append(exp_data)

    # Detailed Ablation Breakdown on 120s Outage
    dur_120 = 120.0
    ab_configs = [
        ("A: M5.1 Baseline (CAN Speed EKF)", lambda: propagate_ekf_m5_1(df_v, init_state, start_idx, dur_120)),
        ("B: M8 Baseline (5D NHC EKF)", lambda: propagate_ekf_m8(df_v, init_state, start_idx, dur_120, enable_wheel_speed=False, enable_nhc=True)),
        ("C: M9 (K_roll_restore = 0.00)", lambda: propagate_ekf_m9(df_v, df_s, init_state, start_idx, dur_120, k_roll_restore=0.00)),
        ("D: M9 (K_roll_restore = 0.05)", lambda: propagate_ekf_m9(df_v, df_s, init_state, start_idx, dur_120, k_roll_restore=0.05)),
        ("E: M9 (K_roll_restore = 0.10)", lambda: propagate_ekf_m9(df_v, df_s, init_state, start_idx, dur_120, k_roll_restore=0.10)),
        ("F: M9 (K_roll_restore = 0.20)", lambda: propagate_ekf_m9(df_v, df_s, init_state, start_idx, dur_120, k_roll_restore=0.20)),
        ("G: M9 Full System (Default K=0.10)", lambda: propagate_ekf_m9(df_v, df_s, init_state, start_idx, dur_120, k_roll_restore=0.10))
    ]

    for name, func in ab_configs:
        res = func()
        met = calculate_outage_error_metrics(TrajectoryResult(points=res.points, dataframe=res.dataframe, outage_start_t=res.outage_start_t, outage_end_t=res.outage_end_t, outage_duration_sec=dur_120), df_v)
        suite_results["ablations"].append({
            "name": name,
            "rmse_m": round(met.rmse_position_error_m, 2),
            "final_m": round(met.final_position_error_m, 2)
        })

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
    res_m8_60 = propagate_ekf_m8(df_v, init_state, start_idx, 60.0, enable_wheel_speed=False, enable_nhc=True)
    res_m9_60 = propagate_ekf_m9(df_v, df_s, init_state, start_idx, 60.0, k_roll_restore=0.10)

    plt.plot(res_m5_1_60.dataframe["east_m"], res_m5_1_60.dataframe["north_m"], color='#1f77b4', linewidth=2.0, label='M5.1 EKF (CAN Speed)')
    plt.plot(res_m8_60.dataframe["east_m"], res_m8_60.dataframe["north_m"], color='#d62728', linestyle='--', linewidth=2.0, label='M8 5D NHC EKF')
    plt.plot(res_m9_60.dataframe["east_m"], res_m9_60.dataframe["north_m"], color='#2ca02c', linestyle='-', linewidth=2.5, label='M9 6D Full-Orientation EKF')

    plt.scatter(0, 0, color='blue', s=80, label='GNSS Outage Start', zorder=5)
    plt.title("GNSS Outage Trajectory: M5.1 vs M8 vs M9 6D Full-Orientation EKF (60s)")
    plt.xlabel("East (meters)")
    plt.ylabel("North (meters)")
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend()
    plt.tight_layout()

    plt.savefig(os.path.join(output_dir, "m9_trajectory_comparison.png"), dpi=150)
    plt.close()

    # Plot 2: Position Error Growth over 120s Outage
    res_m5_1_120 = propagate_ekf_m5_1(df_v, init_state, start_idx, 120.0)
    res_m8_120 = propagate_ekf_m8(df_v, init_state, start_idx, 120.0, enable_wheel_speed=False, enable_nhc=True)
    res_m9_120 = propagate_ekf_m9(df_v, df_s, init_state, start_idx, 120.0, k_roll_restore=0.10)

    ref_120 = df_v[(df_v["t_rel_sec"] >= init_state.t_rel_sec - 1e-5) & (df_v["t_rel_sec"] <= init_state.t_rel_sec + 120.0 + 1e-5)].reset_index(drop=True)
    err_m5_1, err_m8, err_m9 = [], [], []

    for i in range(len(res_m5_1_120.points)):
        r_lat, r_lon = ref_120["Latitude (degrees)"].iloc[i], ref_120["Longitude (degrees)"].iloc[i]
        re, rn, _ = geodetic_to_enu(r_lat, r_lon, 0.0, ref_origin)

        e5, n5 = res_m5_1_120.points[i].east_m, res_m5_1_120.points[i].north_m
        e8, n8 = res_m8_120.points[i].east_m, res_m8_120.points[i].north_m
        e9, n9 = res_m9_120.points[i].east_m, res_m9_120.points[i].north_m

        err_m5_1.append(np.sqrt((e5 - re)**2 + (n5 - rn)**2))
        err_m8.append(np.sqrt((e8 - re)**2 + (n8 - rn)**2))
        err_m9.append(np.sqrt((e9 - re)**2 + (n9 - rn)**2))

    t_axis = ref_120["t_rel_sec"] - ref_120["t_rel_sec"].iloc[0]

    plt.figure(figsize=(10, 5))
    plt.plot(t_axis, err_m5_1, color='#1f77b4', linewidth=2.0, label='M5.1 EKF (CAN Speed)')
    plt.plot(t_axis, err_m8, color='#d62728', linestyle='--', linewidth=2.0, label='M8 5D NHC EKF')
    plt.plot(t_axis, err_m9, color='#2ca02c', linewidth=2.5, label='M9 6D Full-Orientation EKF')
    plt.title("Position Error Growth vs Outage Duration (120s Outage)")
    plt.xlabel("Outage Duration (seconds)")
    plt.ylabel("Position Error (meters)")
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend()
    plt.tight_layout()

    plt.savefig(os.path.join(output_dir, "m9_position_error_growth.png"), dpi=150)
    plt.close()

    total_wall_time = time.time() - t_start_wall
    suite_results["total_runtime_seconds"] = round(total_wall_time, 3)

    json_path = os.path.join(output_dir, "module9_results.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(suite_results, f, indent=2)

    return suite_results

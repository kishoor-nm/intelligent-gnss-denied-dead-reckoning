"""
Module 5: Experiment Runner & Comparative Plotter for 5D EKF Core.
Executes M5 EKF against M3 Baseline and M4.1 Canonical results across 10s, 30s, 60s, 120s outages.
Generates comparative plots and machine-readable JSON results in d:/prototype/results/module5/.
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
from src.iovnbd.navigation.sensor_fusion_m4_1 import propagate_corrected_dead_reckoning
from src.iovnbd.navigation.ekf_m5 import propagate_ekf_dead_reckoning, EKFResult
from src.iovnbd.navigation.metrics import calculate_outage_error_metrics, OutageMetricsResult
from src.iovnbd.navigation.coordinate_frame import AnchorOrigin, geodetic_to_enu

def run_module5_experiment_suite(
    df_v: pd.DataFrame,
    outage_durations: List[float] = [10.0, 30.0, 60.0, 120.0],
    start_idx: int = 1000,
    output_dir: str = "d:/prototype/results/module5"
) -> Dict[str, Any]:
    """
    Executes Module 5 5D EKF experiment suite.
    """
    os.makedirs(output_dir, exist_ok=True)
    t_start_wall = time.time()

    init_state = initialize_navigation_state(df_v, start_idx=start_idx)

    suite_results = {
        "sequence": "S1",
        "start_idx": start_idx,
        "outage_start_t_rel_sec": init_state.t_rel_sec,
        "method": "Module 5 5D Extended Kalman Filter (EKF) Dead-Reckoning Core",
        "state_definition": "[East (m), North (m), Velocity (m/s), Heading (rad), Gyro Bias (rad/s)]",
        "experiments": []
    }

    # Reference trajectory setup
    ref_slice = df_v[(df_v["t_rel_sec"] >= init_state.t_rel_sec - 1e-5) & (df_v["t_rel_sec"] <= init_state.t_rel_sec + max(outage_durations) + 10.0)]
    lat0, lon0 = init_state.lat_deg, init_state.lon_deg
    ref_origin = AnchorOrigin(lat0_rad=float(np.radians(lat0)), lon0_rad=float(np.radians(lon0)), alt0_m=0.0)

    ref_e = []
    ref_n = []
    for _, r in ref_slice.iterrows():
        e, n, _ = geodetic_to_enu(r["Latitude (degrees)"], r["Longitude (degrees)"], 0.0, ref_origin)
        ref_e.append(e)
        ref_n.append(n)

    # Plot 1: Trajectory Comparison (60s Outage)
    plt.figure(figsize=(10, 6))
    plt.plot(ref_e, ref_n, 'k--', linewidth=2.0, label='Reference (VBOX GNSS)')

    m3_res_60 = propagate_dead_reckoning_baseline(df_v, init_state, start_idx, 60.0)
    m4_res_60 = propagate_corrected_dead_reckoning(df_v, init_state, start_idx, 60.0, speed_mode="4wheel_avg")
    m5_res_60 = propagate_ekf_dead_reckoning(df_v, init_state, start_idx, 60.0)

    plt.plot(m3_res_60.dataframe["east_m"], m3_res_60.dataframe["north_m"], color='#1f77b4', linestyle='-', linewidth=2.0, label='M3 Baseline DR')
    plt.plot(m4_res_60.dataframe["east_m"], m4_res_60.dataframe["north_m"], color='#ff7f0e', linestyle='--', linewidth=2.0, label='M4.1 4-Wheel Speed')
    plt.plot(m5_res_60.dataframe["east_m"], m5_res_60.dataframe["north_m"], color='#2ca02c', linestyle=':', linewidth=2.5, label='M5 5D EKF Core')

    plt.scatter(0, 0, color='blue', s=80, label='GNSS Outage Start', zorder=5)
    plt.title("GNSS Outage Trajectory Comparison: M3 vs M4.1 vs M5 5D EKF (60s Outage)")
    plt.xlabel("East (meters)")
    plt.ylabel("North (meters)")
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend()
    plt.tight_layout()

    plt.savefig(os.path.join(output_dir, "m5_trajectory_comparison.png"), dpi=150)
    plt.close()

    # Loop over outage durations
    for dur in outage_durations:
        t0_exp = time.time()

        # M3 Baseline
        res_m3 = propagate_dead_reckoning_baseline(df_v, init_state, start_idx, dur)
        met_m3 = calculate_outage_error_metrics(res_m3, df_v)

        # M5 EKF
        res_m5 = propagate_ekf_dead_reckoning(df_v, init_state, start_idx, dur)
        adapt_m5 = TrajectoryResult(points=res_m5.points, dataframe=res_m5.dataframe, outage_start_t=res_m5.outage_start_t, outage_end_t=res_m5.outage_end_t, outage_duration_sec=dur)
        met_m5 = calculate_outage_error_metrics(adapt_m5, df_v)

        t_elapsed = time.time() - t0_exp

        diff_rmse = met_m5.rmse_position_error_m - met_m3.rmse_position_error_m
        pct_rmse = (diff_rmse / met_m3.rmse_position_error_m) * 100.0 if met_m3.rmse_position_error_m > 0 else 0.0

        exp_data = {
            "outage_duration_sec": dur,
            "sample_count": met_m3.sample_count,
            "m3_rmse_m": round(met_m3.rmse_position_error_m, 2),
            "m5_ekf_rmse_m": round(met_m5.rmse_position_error_m, 2),
            "m3_final_err_m": round(met_m3.final_position_error_m, 2),
            "m5_ekf_final_err_m": round(met_m5.final_position_error_m, 2),
            "m3_max_err_m": round(met_m3.max_position_error_m, 2),
            "m5_ekf_max_err_m": round(met_m5.max_position_error_m, 2),
            "rmse_diff_m": round(diff_rmse, 2),
            "rmse_diff_pct": round(pct_rmse, 2),
            "runtime_ms": round(t_elapsed * 1000.0, 2)
        }
        suite_results["experiments"].append(exp_data)

    # Plot 2: Combined Error Growth Comparison over 120s
    res_m3_120 = propagate_dead_reckoning_baseline(df_v, init_state, start_idx, 120.0)
    met_m3_120 = calculate_outage_error_metrics(res_m3_120, df_v)

    res_m5_120 = propagate_ekf_dead_reckoning(df_v, init_state, start_idx, 120.0)
    met_m5_120 = calculate_outage_error_metrics(TrajectoryResult(points=res_m5_120.points, dataframe=res_m5_120.dataframe, outage_start_t=res_m5_120.outage_start_t, outage_end_t=res_m5_120.outage_end_t, outage_duration_sec=120.0), df_v)

    t_axis = np.linspace(0, 120, len(met_m3_120.error_series_m))

    plt.figure(figsize=(10, 5))
    plt.plot(t_axis, met_m3_120.error_series_m, color='#1f77b4', linewidth=2.0, label=f'M3 Baseline (Final: {met_m3_120.final_position_error_m:.1f}m)')
    plt.plot(t_axis, met_m5_120.error_series_m, color='#2ca02c', linestyle=':', linewidth=2.0, label=f'M5 5D EKF Core (Final: {met_m5_120.final_position_error_m:.1f}m)')
    plt.title("Error Growth During 120s GNSS Outage: M3 Baseline vs M5 5D EKF")
    plt.xlabel("Outage Duration (seconds)")
    plt.ylabel("Horizontal Position Error (meters)")
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend()
    plt.tight_layout()

    plt.savefig(os.path.join(output_dir, "m5_error_growth.png"), dpi=150)
    plt.close()

    total_wall_time = time.time() - t_start_wall
    suite_results["total_runtime_seconds"] = round(total_wall_time, 3)

    json_path = os.path.join(output_dir, "module5_results.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(suite_results, f, indent=2)

    return suite_results

"""
Module 4.1: Ablation & Audit Experiment Suite.
Runs M3 Baseline vs M4.1 Wheel Speed Odometry vs Rear Wheel Speed Odometry across outage durations.
Generates comparative plots, JSON metric files, and explicit ablation breakdown.
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
from src.iovnbd.navigation.sensor_fusion_m4_1 import propagate_corrected_dead_reckoning, CorrectedTrajectoryResult
from src.iovnbd.navigation.metrics import calculate_outage_error_metrics, OutageMetricsResult
from src.iovnbd.navigation.coordinate_frame import AnchorOrigin, geodetic_to_enu

def run_module4_1_ablation_suite(
    df_v: pd.DataFrame,
    outage_durations: List[float] = [10.0, 30.0, 60.0, 120.0],
    start_idx: int = 1000,
    output_dir: str = "d:/prototype/results/module4_1"
) -> Dict[str, Any]:
    """
    Executes Module 4.1 Sensor Audit & Ablation experiments:
    - Experiment A: M3 Baseline (VBOX Transmission Speed + VBOX Yaw Rate)
    - Experiment B: M4.1 4-Wheel Speed Odometry (4-Wheel Encoder Avg + VBOX Yaw Rate)
    - Experiment C: M4.1 Rear-Wheel Speed Odometry (2 Rear Wheel Encoder Avg + VBOX Yaw Rate)
    """
    os.makedirs(output_dir, exist_ok=True)
    t_start_wall = time.time()

    init_state = initialize_navigation_state(df_v, start_idx=start_idx)

    suite_results = {
        "sequence": "S1",
        "start_idx": start_idx,
        "outage_start_t_rel_sec": init_state.t_rel_sec,
        "initial_state_source": init_state.source_description,
        "phone_to_vehicle_status": "PHONE-TO-VEHICLE ALIGNMENT NOT VERIFIED (Smartphone Gyroscope excluded from primary vehicle yaw model)",
        "ablations": []
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

    # Trajectory Comparison Plot (60s Outage)
    plt.figure(figsize=(10, 6))
    plt.plot(ref_e, ref_n, 'k--', linewidth=2.0, label='Reference (VBOX GNSS)')

    exp_a_60 = propagate_corrected_dead_reckoning(df_v, init_state, start_idx, 60.0, speed_mode="vbox")
    exp_b_60 = propagate_corrected_dead_reckoning(df_v, init_state, start_idx, 60.0, speed_mode="4wheel_avg")
    exp_c_60 = propagate_corrected_dead_reckoning(df_v, init_state, start_idx, 60.0, speed_mode="rear_wheel_avg")

    plt.plot(exp_a_60.dataframe["east_m"], exp_a_60.dataframe["north_m"], color='#1f77b4', linewidth=2.0, label='Exp A: M3 Baseline (VBOX Speed)')
    plt.plot(exp_b_60.dataframe["east_m"], exp_b_60.dataframe["north_m"], color='#ff7f0e', linestyle='--', linewidth=2.0, label='Exp B: M4.1 (4-Wheel Avg Speed)')
    plt.plot(exp_c_60.dataframe["east_m"], exp_c_60.dataframe["north_m"], color='#2ca02c', linestyle=':', linewidth=2.0, label='Exp C: M4.1 (Rear-Wheel Avg Speed)')

    plt.scatter(0, 0, color='blue', s=80, label='GNSS Outage Start', zorder=5)
    plt.title("Module 4.1 Ablation Trajectory Comparison (60s Outage)")
    plt.xlabel("East (meters)")
    plt.ylabel("North (meters)")
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend()
    plt.tight_layout()

    plt.savefig(os.path.join(output_dir, "m4_1_ablation_trajectory_comparison.png"), dpi=150)
    plt.close()

    # Loop over outage durations for quantitative ablations
    for dur in outage_durations:
        # Exp A: M3 Baseline
        res_a = propagate_corrected_dead_reckoning(df_v, init_state, start_idx, dur, speed_mode="vbox")
        adapt_a = TrajectoryResult(points=res_a.points, dataframe=res_a.dataframe, outage_start_t=res_a.outage_start_t, outage_end_t=res_a.outage_end_t, outage_duration_sec=dur)
        met_a = calculate_outage_error_metrics(adapt_a, df_v)

        # Exp B: 4-Wheel Encoder Avg
        res_b = propagate_corrected_dead_reckoning(df_v, init_state, start_idx, dur, speed_mode="4wheel_avg")
        adapt_b = TrajectoryResult(points=res_b.points, dataframe=res_b.dataframe, outage_start_t=res_b.outage_start_t, outage_end_t=res_b.outage_end_t, outage_duration_sec=dur)
        met_b = calculate_outage_error_metrics(adapt_b, df_v)

        # Exp C: Rear-Wheel Encoder Avg
        res_c = propagate_corrected_dead_reckoning(df_v, init_state, start_idx, dur, speed_mode="rear_wheel_avg")
        adapt_c = TrajectoryResult(points=res_c.points, dataframe=res_c.dataframe, outage_start_t=res_c.outage_start_t, outage_end_t=res_c.outage_end_t, outage_duration_sec=dur)
        met_c = calculate_outage_error_metrics(adapt_c, df_v)

        ablation_entry = {
            "outage_duration_sec": dur,
            "sample_count": met_a.sample_count,
            "exp_a_m3_baseline_rmse_m": round(met_a.rmse_position_error_m, 2),
            "exp_a_m3_baseline_final_err_m": round(met_a.final_position_error_m, 2),
            "exp_b_4wheel_rmse_m": round(met_b.rmse_position_error_m, 2),
            "exp_b_4wheel_final_err_m": round(met_b.final_position_error_m, 2),
            "exp_c_rearwheel_rmse_m": round(met_c.rmse_position_error_m, 2),
            "exp_c_rearwheel_final_err_m": round(met_c.final_position_error_m, 2),
            "rmse_diff_4wheel_vs_m3_m": round(met_b.rmse_position_error_m - met_a.rmse_position_error_m, 2),
            "rmse_diff_rearwheel_vs_m3_m": round(met_c.rmse_position_error_m - met_a.rmse_position_error_m, 2)
        }
        suite_results["ablations"].append(ablation_entry)

    # Plot 2: Combined Error Growth Comparison over 120s
    res_a_120 = propagate_corrected_dead_reckoning(df_v, init_state, start_idx, 120.0, speed_mode="vbox")
    met_a_120 = calculate_outage_error_metrics(TrajectoryResult(points=res_a_120.points, dataframe=res_a_120.dataframe, outage_start_t=res_a_120.outage_start_t, outage_end_t=res_a_120.outage_end_t, outage_duration_sec=120.0), df_v)

    res_b_120 = propagate_corrected_dead_reckoning(df_v, init_state, start_idx, 120.0, speed_mode="4wheel_avg")
    met_b_120 = calculate_outage_error_metrics(TrajectoryResult(points=res_b_120.points, dataframe=res_b_120.dataframe, outage_start_t=res_b_120.outage_start_t, outage_end_t=res_b_120.outage_end_t, outage_duration_sec=120.0), df_v)

    t_axis = np.linspace(0, 120, len(met_a_120.error_series_m))

    plt.figure(figsize=(10, 5))
    plt.plot(t_axis, met_a_120.error_series_m, color='#1f77b4', linewidth=2.0, label=f'Exp A: M3 Baseline (Final: {met_a_120.final_position_error_m:.1f}m)')
    plt.plot(t_axis, met_b_120.error_series_m, color='#ff7f0e', linestyle='--', linewidth=2.0, label=f'Exp B: M4.1 4-Wheel Avg (Final: {met_b_120.final_position_error_m:.1f}m)')
    plt.title("Error Growth During 120s GNSS Outage: M4.1 Sensor Ablation")
    plt.xlabel("Outage Duration (seconds)")
    plt.ylabel("Horizontal Position Error (meters)")
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend()
    plt.tight_layout()

    plt.savefig(os.path.join(output_dir, "m4_1_ablation_error_growth.png"), dpi=150)
    plt.close()

    total_wall_time = time.time() - t_start_wall
    suite_results["total_runtime_seconds"] = round(total_wall_time, 3)

    json_path = os.path.join(output_dir, "module4_1_results.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(suite_results, f, indent=2)

    return suite_results

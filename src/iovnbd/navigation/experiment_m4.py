"""
Module 4: Multi-Sensor Fusion Outage Experiment Suite & Comparative Plotter.
Runs Module 3 Baseline vs Module 4 Sensor Fusion across 10s, 30s, 60s, 120s outages and generates comparative plots & JSON results.
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
from src.iovnbd.navigation.sensor_fusion import propagate_improved_sensor_fusion, FusedTrajectoryResult
from src.iovnbd.navigation.metrics import calculate_outage_error_metrics, OutageMetricsResult
from src.iovnbd.navigation.coordinate_frame import AnchorOrigin, geodetic_to_enu

def run_module4_experiment_suite(
    df_v: pd.DataFrame,
    df_s: pd.DataFrame,
    outage_durations: List[float] = [10.0, 30.0, 60.0, 120.0],
    start_idx: int = 1000,
    output_dir: str = "d:/prototype/results/module4"
) -> Dict[str, Any]:
    """
    Executes comparative experiments: Module 3 Baseline vs Module 4 Sensor Fusion.
    Generates plots, JSON metrics, and quantitative improvement calculations.
    """
    os.makedirs(output_dir, exist_ok=True)
    t_start_wall = time.time()

    init_state = initialize_navigation_state(df_v, start_idx=start_idx)

    suite_results = {
        "sequence": "S1",
        "start_idx": start_idx,
        "outage_start_t_rel_sec": init_state.t_rel_sec,
        "initial_state_source": init_state.source_description,
        "method": "Multi-Sensor Fusion (4-Wheel Speed Average + Weighted Gyroscope Fusion)",
        "experiments": []
    }

    # Reference trajectory setup
    ref_slice = df_v[(df_v["t_rel_sec"] >= init_state.t_rel_sec) & (df_v["t_rel_sec"] <= init_state.t_rel_sec + max(outage_durations) + 10.0)]
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

    m3_traj_60 = propagate_dead_reckoning_baseline(df_v, init_state, start_idx, outage_duration_sec=60.0)
    m4_traj_60 = propagate_improved_sensor_fusion(df_v, df_s, init_state, start_idx, outage_duration_sec=60.0)

    plt.plot(m3_traj_60.dataframe["east_m"], m3_traj_60.dataframe["north_m"], color='#d62728', linestyle=':', linewidth=2.0, label='Module 3 Baseline DR (60s)')
    plt.plot(m4_traj_60.dataframe["east_m"], m4_traj_60.dataframe["north_m"], color='#2ca02c', linewidth=2.0, label='Module 4 Sensor Fusion (60s)')

    plt.scatter(0, 0, color='blue', s=80, label='GNSS Outage Start', zorder=5)
    plt.title("GNSS Outage Trajectory Comparison: M3 Baseline vs M4 Sensor Fusion")
    plt.xlabel("East (meters)")
    plt.ylabel("North (meters)")
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend()
    plt.tight_layout()

    plt.savefig(os.path.join(output_dir, "m3_vs_m4_trajectory_comparison.png"), dpi=150)
    plt.close()

    # Loop over outage durations for quantitative comparisons
    for dur in outage_durations:
        t0_exp = time.time()
        m3_res = propagate_dead_reckoning_baseline(df_v, init_state, start_idx, outage_duration_sec=dur)
        m3_metrics = calculate_outage_error_metrics(m3_res, df_v)

        m4_res = propagate_improved_sensor_fusion(df_v, df_s, init_state, start_idx, outage_duration_sec=dur)
        # Adapt M4 TrajectoryResult format for metrics calculation
        m4_adapt = TrajectoryResult(
            points=m4_res.points,
            dataframe=m4_res.dataframe,
            outage_start_t=m4_res.outage_start_t,
            outage_end_t=m4_res.outage_end_t,
            outage_duration_sec=m4_res.outage_duration_sec
        )
        m4_metrics = calculate_outage_error_metrics(m4_adapt, df_v)
        t_elapsed_exp = time.time() - t0_exp

        # Absolute & Percentage improvements
        abs_pos_imp = m3_metrics.final_position_error_m - m4_metrics.final_position_error_m
        pct_pos_imp = (abs_pos_imp / m3_metrics.final_position_error_m) * 100.0 if m3_metrics.final_position_error_m > 0 else 0.0

        abs_rmse_imp = m3_metrics.rmse_position_error_m - m4_metrics.rmse_position_error_m
        pct_rmse_imp = (abs_rmse_imp / m3_metrics.rmse_position_error_m) * 100.0 if m3_metrics.rmse_position_error_m > 0 else 0.0

        exp_data = {
            "outage_duration_sec": dur,
            "sample_count": m4_metrics.sample_count,
            "m3_final_error_m": round(m3_metrics.final_position_error_m, 2),
            "m4_final_error_m": round(m4_metrics.final_position_error_m, 2),
            "final_error_improvement_m": round(abs_pos_imp, 2),
            "final_error_improvement_pct": round(pct_pos_imp, 2),
            "m3_rmse_m": round(m3_metrics.rmse_position_error_m, 2),
            "m4_rmse_m": round(m4_metrics.rmse_position_error_m, 2),
            "rmse_improvement_m": round(abs_rmse_imp, 2),
            "rmse_improvement_pct": round(pct_rmse_imp, 2),
            "m4_drift_rate_m_s": round(m4_metrics.drift_rate_m_per_sec, 3),
            "runtime_ms": round(t_elapsed_exp * 1000.0, 2)
        }
        suite_results["experiments"].append(exp_data)

    # Plot 2: Combined Error Growth Comparison over 120s
    m3_res_120 = propagate_dead_reckoning_baseline(df_v, init_state, start_idx, 120.0)
    m3_met_120 = calculate_outage_error_metrics(m3_res_120, df_v)

    m4_res_120 = propagate_improved_sensor_fusion(df_v, df_s, init_state, start_idx, 120.0)
    m4_adapt_120 = TrajectoryResult(points=m4_res_120.points, dataframe=m4_res_120.dataframe, outage_start_t=m4_res_120.outage_start_t, outage_end_t=m4_res_120.outage_end_t, outage_duration_sec=120.0)
    m4_met_120 = calculate_outage_error_metrics(m4_adapt_120, df_v)

    t_axis = np.linspace(0, 120, len(m3_met_120.error_series_m))

    plt.figure(figsize=(10, 5))
    plt.plot(t_axis, m3_met_120.error_series_m, color='#d62728', linewidth=2.0, label=f'M3 Baseline DR (Final: {m3_met_120.final_position_error_m:.1f}m)')
    plt.plot(t_axis, m4_met_120.error_series_m, color='#2ca02c', linewidth=2.0, label=f'M4 Sensor Fusion (Final: {m4_met_120.final_position_error_m:.1f}m)')
    plt.title("Error Growth During 120s GNSS Outage: M3 Baseline vs M4 Sensor Fusion")
    plt.xlabel("Outage Duration (seconds)")
    plt.ylabel("Horizontal Position Error (meters)")
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend()
    plt.tight_layout()

    plt.savefig(os.path.join(output_dir, "m3_vs_m4_error_growth.png"), dpi=150)
    plt.close()

    total_wall_time = time.time() - t_start_wall
    suite_results["total_runtime_seconds"] = round(total_wall_time, 3)

    json_path = os.path.join(output_dir, "module4_results.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(suite_results, f, indent=2)

    return suite_results

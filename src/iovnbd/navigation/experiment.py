"""
Module 3: Multi-Outage Experiment Runner & Trajectory Plotter.
Executes controlled outage experiments across multiple durations (e.g. 10s, 30s, 60s, 120s) and generates SIH review plots.
"""

import os
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from typing import List, Dict, Any

from src.iovnbd.navigation.initialization import initialize_navigation_state
from src.iovnbd.navigation.baseline import propagate_dead_reckoning_baseline
from src.iovnbd.navigation.metrics import calculate_outage_error_metrics, OutageMetricsResult
from src.iovnbd.navigation.coordinate_frame import AnchorOrigin, geodetic_to_enu

def run_outage_experiment_suite(
    df_v: pd.DataFrame,
    outage_durations: List[float] = [10.0, 30.0, 60.0, 120.0],
    start_idx: int = 1000,
    output_dir: str = "d:/prototype/results/module3"
) -> Dict[str, Any]:
    """
    Executes multiple GNSS outage duration experiments from a fixed start_idx.
    Generates plots, JSON results, and human-readable summary.
    """
    os.makedirs(output_dir, exist_ok=True)

    init_state = initialize_navigation_state(df_v, start_idx=start_idx)

    suite_results = {
        "sequence": "S1",
        "start_idx": start_idx,
        "outage_start_t_rel_sec": init_state.t_rel_sec,
        "initial_state_source": init_state.source_description,
        "experiments": []
    }

    plt.figure(figsize=(10, 6))

    # Plot full reference trajectory excerpt
    ref_slice = df_v[(df_v["t_rel_sec"] >= init_state.t_rel_sec) & (df_v["t_rel_sec"] <= init_state.t_rel_sec + max(outage_durations) + 10.0)]
    lat0, lon0 = init_state.lat_deg, init_state.lon_deg
    ref_origin = AnchorOrigin(lat0_rad=float(np.radians(lat0)), lon0_rad=float(np.radians(lon0)), alt0_m=0.0)

    ref_e = []
    ref_n = []
    for _, r in ref_slice.iterrows():
        e, n, _ = geodetic_to_enu(r["Latitude (degrees)"], r["Longitude (degrees)"], 0.0, ref_origin)
        ref_e.append(e)
        ref_n.append(n)

    plt.plot(ref_e, ref_n, 'k--', linewidth=2.0, label='Reference (VBOX GNSS)')

    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']

    for idx, dur in enumerate(outage_durations):
        traj_res = propagate_dead_reckoning_baseline(
            df_v=df_v,
            initial_state=init_state,
            start_idx=start_idx,
            outage_duration_sec=dur
        )

        metrics = calculate_outage_error_metrics(traj_res, df_v)

        exp_data = {
            "outage_duration_sec": dur,
            "sample_count": metrics.sample_count,
            "final_position_error_m": round(metrics.final_position_error_m, 2),
            "max_position_error_m": round(metrics.max_position_error_m, 2),
            "rmse_position_error_m": round(metrics.rmse_position_error_m, 2),
            "drift_rate_m_per_sec": round(metrics.drift_rate_m_per_sec, 3),
            "final_heading_error_deg": round(metrics.final_heading_error_deg, 2)
        }
        suite_results["experiments"].append(exp_data)

        # Plot DR trajectory
        dr_df = traj_res.dataframe
        plt.plot(dr_df["east_m"], dr_df["north_m"], color=colors[idx % len(colors)], linewidth=1.5,
                 label=f'Dead Reckoning ({dur:.0f}s Outage)')

    plt.scatter(0, 0, color='green', s=80, label='GNSS Outage Start Point', zorder=5)
    plt.title(f"Baseline Dead Reckoning vs Reference Trajectory (GNSS Outage)")
    plt.xlabel("East (meters)")
    plt.ylabel("North (meters)")
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend()
    plt.tight_layout()

    traj_plot_path = os.path.join(output_dir, "baseline_trajectory_comparison.png")
    plt.savefig(traj_plot_path, dpi=150)
    plt.close()

    # Plot Error Growth vs Outage Duration
    plt.figure(figsize=(10, 5))
    for idx, dur in enumerate(outage_durations):
        traj_res = propagate_dead_reckoning_baseline(df_v, init_state, start_idx, dur)
        metrics = calculate_outage_error_metrics(traj_res, df_v)
        t_axis = np.linspace(0, dur, len(metrics.error_series_m))
        plt.plot(t_axis, metrics.error_series_m, color=colors[idx % len(colors)], linewidth=1.5,
                 label=f'Outage {dur:.0f}s (Final: {metrics.final_position_error_m:.1f}m)')

    plt.title("Baseline Dead Reckoning Error Growth During GNSS Outage")
    plt.xlabel("Outage Duration (seconds)")
    plt.ylabel("Horizontal Position Error (meters)")
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend()
    plt.tight_layout()

    err_plot_path = os.path.join(output_dir, "baseline_error_growth.png")
    plt.savefig(err_plot_path, dpi=150)
    plt.close()

    # Save JSON results
    json_path = os.path.join(output_dir, "baseline_results.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(suite_results, f, indent=2)

    return suite_results

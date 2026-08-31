"""
Module 9.1: Experiment & Grid Calibration Runner for Speed-Adaptive Roll Compensation.
Performs:
1. Controlled Parameter Grid Study on Training/Validation sequences (S2_train, S2_val).
2. Locked Parameter Evaluation on Unseen Canonical Test Sequence S1 (10s, 30s, 60s, 120s outages).
3. Comparative Benchmarking: M5.1 Baseline vs M8 Baseline vs M9 (K=0) vs M9 (Fixed K=0.10) vs M9.1 (Adaptive K).
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
from src.iovnbd.navigation.ekf_m9 import propagate_ekf_m9, propagate_ekf_m9_1, compute_speed_adaptive_k_roll
from src.iovnbd.navigation.metrics import calculate_outage_error_metrics, OutageMetricsResult
from src.iovnbd.navigation.coordinate_frame import AnchorOrigin, geodetic_to_enu

def run_module9_1_grid_search_on_val(
    v2_path: str = "d:/prototype/IO-VNBD-master/Synchronised V abd S datasets/Categorised IOVNB Dataset/S (Driver A)/S2/V-S2.csv",
    s2_path: str = "d:/prototype/IO-VNBD-master/Synchronised V abd S datasets/Categorised IOVNB Dataset/S (Driver A)/S2/S-S2.csv",
    val_start_idx: int = 70000,
    outage_dur_sec: float = 120.0
) -> Dict[str, Any]:
    """
    Executes controlled grid search on Validation Split (S2_val) using non-GNSS vehicle signals.
    Grid: K_base in {0.02, 0.05, 0.08, 0.10}, V0 in {2.0, 5.0, 8.0, 10.0} m/s.
    """
    df_v2 = pd.read_csv(v2_path, encoding='latin1')
    df_s2 = pd.read_csv(s2_path, encoding='latin1')

    # Add t_rel_sec and non-GNSS columns
    df_v2['t_rel_sec'] = np.arange(len(df_v2)) * 0.1
    df_s2['t_rel_sec'] = np.arange(len(df_s2)) * 0.1

    # Map raw headers
    if 'indicated_speed_m_s' not in df_v2.columns:
        sp_col = [c for c in df_v2.columns if 'indicated vehicle speed' in c.strip().lower()][0]
        df_v2['indicated_speed_m_s'] = pd.to_numeric(df_v2[sp_col], errors='coerce').fillna(0.0) / 3.6

    if 'longitudinal_accel_m_s2' not in df_v2.columns:
        a_long_col = [c for c in df_v2.columns if 'longitudinal acceleration' in c.strip().lower()][0]
        df_v2['longitudinal_accel_m_s2'] = pd.to_numeric(df_v2[a_long_col], errors='coerce').fillna(0.0) * 9.80665

    if 'lateral_accel_m_s2' not in df_v2.columns:
        a_lat_col = [c for c in df_v2.columns if 'lateral acceleration' in c.strip().lower()][0]
        df_v2['lateral_accel_m_s2'] = pd.to_numeric(df_v2[a_lat_col], errors='coerce').fillna(0.0) * 9.80665

    if 'yaw_rate_rad_s' not in df_v2.columns:
        yaw_col = [c for c in df_v2.columns if 'yaw rate' in c.strip().lower()][0]
        df_v2['yaw_rate_rad_s'] = pd.to_numeric(df_v2[yaw_col], errors='coerce').fillna(0.0) * np.pi / 180.0

    if 'Latitude (degrees)' not in df_v2.columns:
        lat_col = [c for c in df_v2.columns if 'latitude' in c.strip().lower()][0]
        lon_col = [c for c in df_v2.columns if 'longitude' in c.strip().lower()][0]
        df_v2['Latitude (degrees)'] = pd.to_numeric(df_v2[lat_col], errors='coerce').fillna(0.0)
        df_v2['Longitude (degrees)'] = pd.to_numeric(df_v2[lon_col], errors='coerce').fillna(0.0)

    init_val = initialize_navigation_state(df_v2, start_idx=val_start_idx)

    # M5.1 Baseline on Val Split
    res_m5_1 = propagate_ekf_m5_1(df_v2, init_val, val_start_idx, outage_dur_sec)
    met_m5_1 = calculate_outage_error_metrics(TrajectoryResult(points=res_m5_1.points, dataframe=res_m5_1.dataframe, outage_start_t=res_m5_1.outage_start_t, outage_end_t=res_m5_1.outage_end_t, outage_duration_sec=outage_dur_sec), df_v2)

    grid_results = []
    best_config = None
    min_val_rmse = 999999.0

    k_base_grid = [0.02, 0.05, 0.08, 0.10]
    v0_grid = [2.0, 5.0, 8.0, 10.0]

    for kb in k_base_grid:
        for v0 in v0_grid:
            res = propagate_ekf_m9_1(df_v2, df_s2, init_val, val_start_idx, outage_dur_sec, k_mode="adaptive", k_base=kb, v0_m_s=v0)
            met = calculate_outage_error_metrics(TrajectoryResult(points=res.points, dataframe=res.dataframe, outage_start_t=res.outage_start_t, outage_end_t=res.outage_end_t, outage_duration_sec=outage_dur_sec), df_v2)

            rmse_m = round(met.rmse_position_error_m, 2)
            final_m = round(met.final_position_error_m, 2)

            grid_entry = {
                "k_base": kb,
                "v0_m_s": v0,
                "val_rmse_m": rmse_m,
                "val_final_m": final_m,
                "improvement_vs_m5_1_m": round(met_m5_1.rmse_position_error_m - rmse_m, 2)
            }
            grid_results.append(grid_entry)

            if rmse_m < min_val_rmse:
                min_val_rmse = rmse_m
                best_config = grid_entry

    return {
        "val_m5_1_rmse_m": round(met_m5_1.rmse_position_error_m, 2),
        "val_m5_1_final_m": round(met_m5_1.final_position_error_m, 2),
        "grid_results": grid_results,
        "selected_best_config": best_config
    }

def run_module9_1_experiment_suite(
    output_dir: str = "d:/prototype/results/module9",
    outage_durations: List[float] = [10.0, 30.0, 60.0, 120.0],
    start_idx: int = 1000
) -> Dict[str, Any]:
    """
    Executes Module 9.1 grid search on validation split, locks best parameters (K_base=0.05, V0=5.0 m/s),
    and evaluates on unseen canonical test sequence S1.
    """
    os.makedirs(output_dir, exist_ok=True)
    t_start_wall = time.time()

    # Step 1: Execute Grid Calibration on S2 Validation Split
    val_audit = run_module9_1_grid_search_on_val()
    locked_k_base = val_audit["selected_best_config"]["k_base"]
    locked_v0 = val_audit["selected_best_config"]["v0_m_s"]

    # Step 2: Load Unseen Canonical Test Sequence S1
    df_v = pd.read_csv("d:/prototype/data/processed/S1/V-S1_processed.csv")
    df_s = pd.read_csv("d:/prototype/data/processed/S1/S-S1_processed.csv")

    init_state = initialize_navigation_state(df_v, start_idx=start_idx)

    suite_results = {
        "sequence": "S1",
        "start_idx": start_idx,
        "method": "Module 9.1 Speed-Adaptive Roll Compensation EKF",
        "locked_parameters": {
            "k_base": locked_k_base,
            "v0_m_s": locked_v0,
            "provenance": "Calibrated on S2 validation split without using S1 test ground truth"
        },
        "validation_grid_audit": val_audit,
        "experiments": []
    }

    # Step 3: Canonical Benchmark Loop on Unseen S1
    for dur in outage_durations:
        t0_exp = time.time()

        # 1. M5.1 Baseline
        res_m5_1 = propagate_ekf_m5_1(df_v, init_state, start_idx, dur)
        met_m5_1 = calculate_outage_error_metrics(TrajectoryResult(points=res_m5_1.points, dataframe=res_m5_1.dataframe, outage_start_t=res_m5_1.outage_start_t, outage_end_t=res_m5_1.outage_end_t, outage_duration_sec=dur), df_v)

        # 2. M8 Baseline
        res_m8 = propagate_ekf_m8(df_v, init_state, start_idx, dur, enable_wheel_speed=False, enable_nhc=True)
        met_m8 = calculate_outage_error_metrics(TrajectoryResult(points=res_m8.points, dataframe=res_m8.dataframe, outage_start_t=res_m8.outage_start_t, outage_end_t=res_m8.outage_end_t, outage_duration_sec=dur), df_v)

        # 3. M9 (K = 0.00)
        res_m9_k00 = propagate_ekf_m9(df_v, df_s, init_state, start_idx, dur, k_roll_restore=0.00)
        met_m9_k00 = calculate_outage_error_metrics(TrajectoryResult(points=res_m9_k00.points, dataframe=res_m9_k00.dataframe, outage_start_t=res_m9_k00.outage_start_t, outage_end_t=res_m9_k00.outage_end_t, outage_duration_sec=dur), df_v)

        # 4. M9 (Fixed K = 0.10)
        res_m9_k10 = propagate_ekf_m9(df_v, df_s, init_state, start_idx, dur, k_roll_restore=0.10)
        met_m9_k10 = calculate_outage_error_metrics(TrajectoryResult(points=res_m9_k10.points, dataframe=res_m9_k10.dataframe, outage_start_t=res_m9_k10.outage_start_t, outage_end_t=res_m9_k10.outage_end_t, outage_duration_sec=dur), df_v)

        # 5. M9.1 Adaptive K (Locked: K_base=locked_k_base, V0=locked_v0)
        res_m9_1 = propagate_ekf_m9_1(df_v, df_s, init_state, start_idx, dur, k_mode="adaptive", k_base=locked_k_base, v0_m_s=locked_v0)
        met_m9_1 = calculate_outage_error_metrics(TrajectoryResult(points=res_m9_1.points, dataframe=res_m9_1.dataframe, outage_start_t=res_m9_1.outage_start_t, outage_end_t=res_m9_1.outage_end_t, outage_duration_sec=dur), df_v)

        t_elapsed = time.time() - t0_exp

        # Status vs M5.1 Baseline
        diff_rmse_m5_1 = met_m9_1.rmse_position_error_m - met_m5_1.rmse_position_error_m
        if met_m9_1.rmse_position_error_m < met_m5_1.rmse_position_error_m - 0.1:
            status_vs_m5_1 = "IMPROVED"
        elif abs(diff_rmse_m5_1) <= 0.1:
            status_vs_m5_1 = "NEUTRAL"
        else:
            status_vs_m5_1 = "DEGRADED"

        # Status vs Fixed M9 (K=0.10)
        diff_rmse_m9 = met_m9_1.rmse_position_error_m - met_m9_k10.rmse_position_error_m
        if met_m9_1.rmse_position_error_m < met_m9_k10.rmse_position_error_m - 0.1:
            status_vs_m9_fixed = "IMPROVED"
        elif abs(diff_rmse_m9) <= 0.1:
            status_vs_m9_fixed = "NEUTRAL"
        else:
            status_vs_m9_fixed = "DEGRADED"

        max_roll_deg = float(np.degrees(np.max(np.abs(res_m9_1.dataframe["roll_rad"]))))

        exp_data = {
            "outage_duration_sec": dur,
            "sample_count": met_m5_1.sample_count,
            "m5_1_ekf_rmse_m": round(met_m5_1.rmse_position_error_m, 2),
            "m8_ekf_rmse_m": round(met_m8.rmse_position_error_m, 2),
            "m9_k00_rmse_m": round(met_m9_k00.rmse_position_error_m, 2),
            "m9_k10_rmse_m": round(met_m9_k10.rmse_position_error_m, 2),
            "m9_1_adaptive_rmse_m": round(met_m9_1.rmse_position_error_m, 2),
            "m5_1_ekf_final_m": round(met_m5_1.final_position_error_m, 2),
            "m9_k00_final_m": round(met_m9_k00.final_position_error_m, 2),
            "m9_k10_final_m": round(met_m9_k10.final_position_error_m, 2),
            "m9_1_adaptive_final_m": round(met_m9_1.final_position_error_m, 2),
            "m9_1_max_roll_deg": round(max_roll_deg, 2),
            "status_vs_m5_1": status_vs_m5_1,
            "status_vs_m9_fixed": status_vs_m9_fixed,
            "runtime_ms": round(t_elapsed * 1000.0, 2)
        }
        suite_results["experiments"].append(exp_data)

    # Plot 1: Trajectory Comparison on 120s Outage
    ref_slice = df_v[(df_v["t_rel_sec"] >= init_state.t_rel_sec - 1e-5) & (df_v["t_rel_sec"] <= init_state.t_rel_sec + 120.0 + 1e-5)].reset_index(drop=True)
    lat0, lon0 = init_state.lat_deg, init_state.lon_deg
    ref_origin = AnchorOrigin(lat0_rad=float(np.radians(lat0)), lon0_rad=float(np.radians(lon0)), alt0_m=0.0)
    ref_e, ref_n = [], []
    for _, r in ref_slice.iterrows():
        e, n, _ = geodetic_to_enu(r["Latitude (degrees)"], r["Longitude (degrees)"], 0.0, ref_origin)
        ref_e.append(e)
        ref_n.append(n)

    plt.figure(figsize=(10, 6))
    plt.plot(ref_e, ref_n, 'k--', linewidth=2.0, label='Reference (VBOX GNSS)')

    res_m5_1_120 = propagate_ekf_m5_1(df_v, init_state, start_idx, 120.0)
    res_m9_k10_120 = propagate_ekf_m9(df_v, df_s, init_state, start_idx, 120.0, k_roll_restore=0.10)
    res_m9_1_120 = propagate_ekf_m9_1(df_v, df_s, init_state, start_idx, 120.0, k_mode="adaptive", k_base=locked_k_base, v0_m_s=locked_v0)

    plt.plot(res_m5_1_120.dataframe["east_m"], res_m5_1_120.dataframe["north_m"], color='#1f77b4', linewidth=2.0, label='M5.1 EKF (CAN Speed)')
    plt.plot(res_m9_k10_120.dataframe["east_m"], res_m9_k10_120.dataframe["north_m"], color='#d62728', linestyle='--', linewidth=2.0, label='M9 Fixed K=0.10 EKF')
    plt.plot(res_m9_1_120.dataframe["east_m"], res_m9_1_120.dataframe["north_m"], color='#2ca02c', linestyle='-', linewidth=2.5, label='M9.1 Speed-Adaptive EKF')

    plt.scatter(0, 0, color='blue', s=80, label='GNSS Outage Start', zorder=5)
    plt.title("GNSS Outage Trajectory: M5.1 vs M9 Fixed K vs M9.1 Speed-Adaptive EKF (120s)")
    plt.xlabel("East (meters)")
    plt.ylabel("North (meters)")
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend()
    plt.tight_layout()

    plt.savefig(os.path.join(output_dir, "m9_1_trajectory_comparison.png"), dpi=150)
    plt.close()

    # Plot 2: Speed-Adaptive Restoring Stiffness K(V) Trace over 120s Outage
    df_m9_1_120 = res_m9_1_120.dataframe
    t_axis = df_m9_1_120["t_rel_sec"] - df_m9_1_120["t_rel_sec"].iloc[0]

    fig, ax1 = plt.subplots(figsize=(10, 5))

    color = '#1f77b4'
    ax1.set_xlabel('Outage Duration (seconds)')
    ax1.set_ylabel('Vehicle Speed (m/s)', color=color)
    ax1.plot(t_axis, df_m9_1_120["velocity_m_s"], color=color, linewidth=2.0, label='Vehicle Speed V')
    ax1.tick_params(axis='y', labelcolor=color)

    ax2 = ax1.twinx()
    color = '#2ca02c'
    ax2.set_ylabel('Adaptive Restoring Stiffness K(V)', color=color)
    ax2.plot(t_axis, df_m9_1_120["k_roll_adaptive"], color=color, linestyle=':', linewidth=2.5, label='K(V) Trace')
    ax2.tick_params(axis='y', labelcolor=color)

    plt.title("Module 9.1 Dynamic Speed-Adaptive Roll Restoring Stiffness K(V)")
    fig.tight_layout()
    plt.savefig(os.path.join(output_dir, "m9_1_k_adaptive_trace.png"), dpi=150)
    plt.close()

    total_wall_time = time.time() - t_start_wall
    suite_results["total_runtime_seconds"] = round(total_wall_time, 3)

    json_path = os.path.join(output_dir, "module9_1_results.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(suite_results, f, indent=2)

    return suite_results

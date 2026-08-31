"""
SIH 2026 PS-168 Prototype Command-Line Demonstration (CLI Demo Runner).
Supports custom vehicle and smartphone CSV file arguments, input schema validation,
strict GNSS reference masking, and generates demonstration visualization plots.
"""

import os
import sys
import time
import argparse
from typing import Dict, Any, List
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from src.iovnbd.navigation.final_navigation import get_final_competition_system, FinalDeadReckoningConfig
from src.iovnbd.navigation.initialization import initialize_navigation_state
from src.iovnbd.navigation.baseline import TrajectoryResult
from src.iovnbd.navigation.ekf_m5_1 import propagate_ekf_m5_1
from src.iovnbd.navigation.metrics import calculate_outage_error_metrics
from src.iovnbd.navigation.coordinate_frame import geodetic_to_enu
from src.iovnbd.preprocessing.schema_validation import validate_vehicle_dataframe_schema, validate_smartphone_dataframe_schema

def run_end_to_end_prototype_demo(
    output_dir: str = "d:/prototype/results/demo",
    v_path: str = "d:/prototype/data/processed/S1/V-S1_processed.csv",
    s_path: str = "d:/prototype/data/processed/S1/S-S1_processed.csv",
    start_idx: int = 1000,
    outage_duration_sec: float = 120.0
) -> Dict[str, Any]:
    os.makedirs(output_dir, exist_ok=True)
    t_start_wall = time.time()

    print("=" * 85)
    print("      SIH 2026 PROBLEM STATEMENT 168 — INTELLIGENT DEAD RECKONING PROTOTYPE")
    print("            END-TO-END DEMONSTRATION & SYSTEM INTERFACE AUDIT")
    print("=" * 85)

    # Step 1: Data Loading & Path Resolution
    print("\n[STEP 1/5] Resolving and Loading Multi-Sensor Datasets...")
    if not os.path.exists(v_path):
        raise FileNotFoundError(f"Vehicle dataset file not found at '{v_path}'")
    if not os.path.exists(s_path):
        raise FileNotFoundError(f"Smartphone dataset file not found at '{s_path}'")

    df_v = pd.read_csv(v_path)
    df_s = pd.read_csv(s_path)
    print(f"  -> Vehicle Dataset (CAN ECU): {len(df_v)} rows loaded from '{v_path}'")
    print(f"  -> Smartphone Dataset (IMU):  {len(df_s)} rows loaded from '{s_path}'")

    # Step 2: Schema Validation
    print("\n[STEP 2/5] Performing Input Sensor Schema & Unit Validation...")
    v_valid, v_errs = validate_vehicle_dataframe_schema(df_v)
    s_valid, s_errs = validate_smartphone_dataframe_schema(df_s)

    if not v_valid:
        raise ValueError(f"Vehicle CSV schema validation failed: {v_errs}")
    if not s_valid:
        raise ValueError(f"Smartphone CSV schema validation failed: {s_errs}")

    print("  -> Vehicle CSV Schema:    VALIDATED (ECU Speed, Accel, Yaw Rate present)")
    print("  -> Smartphone CSV Schema: VALIDATED (IMU Gyro Roll Rate present)")

    # Step 3: Outage Initialization & Strict Reference Masking
    print("\n[STEP 3/5] Initializing GNSS Outage Boundary & Sensor Provenance Masking...")
    if start_idx >= len(df_v):
        raise IndexError(f"Start index {start_idx} exceeds vehicle dataset length {len(df_v)}")

    init_state = initialize_navigation_state(df_v, start_idx=start_idx)
    t0 = init_state.t_rel_sec

    print(f"  -> GNSS Outage Start Time: t = {t0:.1f} seconds (Row Index: {start_idx})")
    print(f"  -> Outage Duration:        T = {outage_duration_sec:.1f} seconds ({int(outage_duration_sec*10)+1} samples)")
    print(f"  -> Initial Position Anchor: Lat {init_state.lat_deg:.6f}°, Lon {init_state.lon_deg:.6f}°")
    print(f"  -> Initial Speed / Heading: {init_state.speed_m_s:.2f} m/s | Heading: {init_state.heading_deg:.1f}°")
    print("  " + "!" * 75)
    print("  ! ATTENTION: GNSS POSITION & VELOCITY ATTRIBUTES STRICTLY MASKED FROM INFERENCE !")
    print("  ! ALLOWED NON-GNSS SENSORS: ECU Speed, Long/Lat Accel, Yaw Rate, Smartphone Roll Rate !")
    print("  " + "!" * 75)

    # Step 4: Run Production Final Navigation System
    print("\n[STEP 4/5] Executing Production Dead Reckoning Estimator (Module 9.3 Fused System)...")
    system = get_final_competition_system()
    res_fused = system.run_outage_navigation(df_v, df_s, start_idx=start_idx, outage_duration_sec=outage_duration_sec)

    print(f"  -> Dead Reckoning Execution Complete! ({len(res_fused.points)} state updates processed)")
    if res_fused.switch_count > 0:
        sw_t = res_fused.switch_timestamps[0] - t0
        print(f"  -> Adaptive Estimator Switch: Transitioned from M5.1 (5D CAN EKF) to M9.1 (6D Roll EKF) at t = +{sw_t:.1f}s")
    else:
        print("  -> Active Estimator: Single regime mode")

    # Step 5: Post-Inference Reference Comparison & Metric Evaluation
    print("\n[STEP 5/5] Performing Post-Inference Evaluation & Generating Visualizations...")

    has_gnss_ref = ("Latitude (degrees)" in df_v.columns and "Longitude (degrees)" in df_v.columns)

    if has_gnss_ref:
        metrics_fused = system.evaluate_outage_performance(res_fused, df_v)

        # Run M5.1 Baseline for direct comparative demonstration
        res_m5_1 = propagate_ekf_m5_1(df_v, init_state, start_idx, outage_duration_sec)
        metrics_m5_1 = calculate_outage_error_metrics(TrajectoryResult(points=res_m5_1.points, dataframe=res_m5_1.dataframe, outage_start_t=res_m5_1.outage_start_t, outage_end_t=res_m5_1.outage_end_t, outage_duration_sec=outage_duration_sec), df_v)

        improvement_pct = ((metrics_m5_1.rmse_position_error_m - metrics_fused.rmse_position_error_m) / metrics_m5_1.rmse_position_error_m) * 100.0

        print("\n" + "=" * 70)
        print("              DEMONSTRATION NAVIGATION PERFORMANCE SUMMARY")
        print("=" * 70)
        print(f"  Outage Duration:                {outage_duration_sec:.0f} seconds")
        print(f"  M5.1 Baseline Position RMSE:    {metrics_m5_1.rmse_position_error_m:6.2f} meters")
        print(f"  M9.3 Fused Prototype RMSE:      {metrics_fused.rmse_position_error_m:6.2f} meters")
        print(f"  M5.1 Final Position Error:      {metrics_m5_1.final_position_error_m:6.2f} meters")
        print(f"  M9.3 Final Position Error:      {metrics_fused.final_position_error_m:6.2f} meters")
        print(f"  PROTOTYPE ACCURACY GAIN:        +{improvement_pct:.1f}% Error Reduction over Baseline")
        print("=" * 70)

        # Plot 1: Full Trajectory Comparison (2D ENU Map)
        ref_slice = df_v[(df_v["t_rel_sec"] >= t0 - 1e-5) & (df_v["t_rel_sec"] <= t0 + outage_duration_sec + 1e-5)].reset_index(drop=True)
        ref_origin = init_state.origin
        ref_e, ref_n = [], []
        for _, r in ref_slice.iterrows():
            e, n, _ = geodetic_to_enu(r["Latitude (degrees)"], r["Longitude (degrees)"], 0.0, ref_origin)
            ref_e.append(e)
            ref_n.append(n)

        plt.figure(figsize=(10, 7))
        plt.plot(ref_e, ref_n, 'k--', linewidth=2.5, label='VBOX GNSS Ground Truth (Reference)')
        plt.plot(res_m5_1.dataframe["east_m"], res_m5_1.dataframe["north_m"], color='#d62728', linestyle=':', linewidth=2.0, label=f'M5.1 Baseline (RMSE: {metrics_m5_1.rmse_position_error_m:.1f}m)')
        plt.plot(res_fused.dataframe["east_m"], res_fused.dataframe["north_m"], color='#2ca02c', linestyle='-', linewidth=2.5, label=f'M9.3 Fused Prototype (RMSE: {metrics_fused.rmse_position_error_m:.1f}m)')

        plt.scatter(0, 0, color='blue', s=100, label='GNSS Outage Start Point', zorder=6)

        sw_pts = res_fused.dataframe[res_fused.dataframe["switch_event"] == True]
        if len(sw_pts) > 0:
            plt.scatter(sw_pts["east_m"].iloc[0], sw_pts["north_m"].iloc[0], color='orange', s=120, zorder=7, label=f'Adaptive Switch (t=+{sw_pts["t_rel_sec"].iloc[0]-t0:.0f}s)')

        plt.title(f"SIH 2026 PS-168: 120s GNSS-Denied Trajectory Estimation", fontsize=13, fontweight='bold')
        plt.xlabel("East Position (meters)")
        plt.ylabel("North Position (meters)")
        plt.grid(True, linestyle='--', alpha=0.6)
        plt.legend(loc='best')
        plt.tight_layout()
        plot_traj_path = os.path.join(output_dir, "demo_trajectory_comparison.png")
        plt.savefig(plot_traj_path, dpi=150)
        plt.close()

        # Plot 2: Position Error Growth & State Dynamics
        t_axis = res_fused.dataframe["t_rel_sec"] - t0
        err_m5_1 = np.sqrt((res_m5_1.dataframe["east_m"].values - np.array(ref_e))**2 + (res_m5_1.dataframe["north_m"].values - np.array(ref_n))**2)
        err_fused = np.sqrt((res_fused.dataframe["east_m"].values - np.array(ref_e))**2 + (res_fused.dataframe["north_m"].values - np.array(ref_n))**2)

        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
        ax1.plot(t_axis, err_m5_1, color='#d62728', linestyle=':', linewidth=2.0, label='M5.1 Baseline Error')
        ax1.plot(t_axis, err_fused, color='#2ca02c', linewidth=2.5, label='M9.3 Fused Prototype Error')
        ax1.set_ylabel("Position Error (m)")
        ax1.set_title("Position Drift Growth over Outage Window")
        ax1.grid(True, linestyle='--', alpha=0.6)
        ax1.legend()

        ax2.plot(t_axis, res_fused.dataframe["velocity_m_s"], color='#1f77b4', linewidth=2.0, label='Estimated Speed (m/s)')
        ax2.set_ylabel("Speed (m/s)")
        ax2.grid(True, linestyle='--', alpha=0.6)
        ax2.legend()

        ax3.plot(t_axis, res_fused.dataframe["roll_deg"], color='#9467bd', linewidth=2.0, label='Vehicle Roll Angle (°)')
        ax3.set_ylabel("Roll Angle (°)")
        ax3.set_xlabel("Outage Duration (seconds)")
        ax3.grid(True, linestyle='--', alpha=0.6)
        ax3.legend()

        plt.tight_layout()
        plot_dynamics_path = os.path.join(output_dir, "demo_state_dynamics.png")
        plt.savefig(plot_dynamics_path, dpi=150)
        plt.close()

        fused_rmse = metrics_fused.rmse_position_error_m
        m51_rmse = metrics_m5_1.rmse_position_error_m
    else:
        print("  -> Notice: Ground-truth GNSS reference columns not found. Skipping offline accuracy scoring.")
        plot_traj_path = ""
        plot_dynamics_path = ""
        fused_rmse = 0.0
        m51_rmse = 0.0
        improvement_pct = 0.0

    total_wall_time = time.time() - t_start_wall

    print(f"\n[DEMO EXECUTION COMPLETE]")
    if plot_traj_path:
        print(f"  -> Trajectory Map Plot: {plot_traj_path}")
        print(f"  -> State Dynamics Plot: {plot_dynamics_path}")
    print(f"  -> Execution Runtime:   {total_wall_time:.3f} seconds\n")

    return {
        "status": "A — PROTOTYPE DEMO READY",
        "outage_duration_sec": outage_duration_sec,
        "m5_1_rmse_m": round(m51_rmse, 2),
        "fused_rmse_m": round(fused_rmse, 2),
        "improvement_pct": round(improvement_pct, 1),
        "runtime_sec": round(total_wall_time, 3),
        "trajectory_plot": plot_traj_path,
        "dynamics_plot": plot_dynamics_path
    }

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="SIH 2026 PS-168 Dead Reckoning Prototype Demo Runner")
    parser.add_argument("--vehicle_csv", type=str, default="d:/prototype/data/processed/S1/V-S1_processed.csv", help="Path to vehicle CAN bus CSV dataset")
    parser.add_argument("--smartphone_csv", type=str, default="d:/prototype/data/processed/S1/S-S1_processed.csv", help="Path to smartphone IMU CSV dataset")
    parser.add_argument("--output_dir", type=str, default="d:/prototype/results/demo", help="Directory for output demonstration plots")
    parser.add_argument("--start_idx", type=int, default=1000, help="Row index for GNSS outage start")
    parser.add_argument("--duration", type=float, default=120.0, help="Outage duration in seconds")

    args = parser.parse_args()
    run_end_to_end_prototype_demo(
        output_dir=args.output_dir,
        v_path=args.vehicle_csv,
        s_path=args.smartphone_csv,
        start_idx=args.start_idx,
        outage_duration_sec=args.duration
    )

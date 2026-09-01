"""
SIH 2026 PS-168: V2 Demonstration and Validation Package CLI Runner.
Generates comprehensive visual trajectory comparisons, state dynamics plots, and a performance report
comparing V1 Baseline vs V2 Production System on canonical IO-VNBD S1 test sequence.
"""

import os
import sys
import argparse
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from src.iovnbd.navigation.initialization import initialize_navigation_state
from src.iovnbd.navigation.csv_replay_streamer import CSVReplayStreamer
from src.iovnbd.navigation.streaming_runner import StreamingNavigationRunner
from src.iovnbd.navigation.coordinate_frame import geodetic_to_enu
from src.iovnbd.navigation.final_navigation import FinalNavigationSystem, FinalDeadReckoningConfig

def run_v2_validation_package(
    vehicle_csv: str = "d:/prototype/data/processed/S1/V-S1_processed.csv",
    smartphone_csv: str = "d:/prototype/data/processed/S1/S-S1_processed.csv",
    output_dir: str = "d:/prototype/results/v2_validation_package",
    start_idx: int = 1000,
    outage_duration_sec: float = 30.0
):
    os.makedirs(output_dir, exist_ok=True)

    print("=" * 85)
    print("      SIH 2026 PROBLEM STATEMENT 168 — INTELLIGENT DEAD RECKONING PROTOTYPE")
    print("               VERSION 2.0 DEMONSTRATION & VALIDATION PACKAGE")
    print("=" * 85)
    print("\n  MODE:                   SOFTWARE-IN-THE-LOOP (SIL) DATASET REPLAY DEMONSTRATION")
    print("  PROVENANCE & MASKING:   GNSS DATA STRICTLY MASKED DURING OUTAGE INFERENCE")
    print("  SENSORS (CAUSAL):      CAN ECU Speed, Accel, Yaw Rate + Smartphone IMU Roll Rate")
    print("  SCORING (OFFLINE ONLY): VBOX GNSS Reference Trajectory (Post-Inference Scoring)")
    print("  PRODUCTION VERSION:     V2.0 (Configurable yaw_scale_factor=0.95 + Lateral NHC)\n")

    df_v = pd.read_csv(vehicle_csv)
    df_s = pd.read_csv(smartphone_csv)

    n_samples = int(round(outage_duration_sec * 10)) + 1
    init_state = initialize_navigation_state(df_v, start_idx=start_idx)
    origin = init_state.origin
    t0 = init_state.t_rel_sec

    # 1. Run V1 Baseline Replay (yaw_scale_factor = 1.0)
    sys_v1 = FinalNavigationSystem(FinalDeadReckoningConfig(yaw_scale_factor=1.0))
    res_v1 = sys_v1.run_outage_navigation(df_v, df_s, start_idx=start_idx, outage_duration_sec=outage_duration_sec)
    metrics_v1 = sys_v1.evaluate_outage_performance(res_v1, df_v)

    # 2. Run V2 Production Replay (yaw_scale_factor = 0.95)
    sys_v2 = FinalNavigationSystem(FinalDeadReckoningConfig(yaw_scale_factor=0.95))
    res_v2 = sys_v2.run_outage_navigation(df_v, df_s, start_idx=start_idx, outage_duration_sec=outage_duration_sec)
    metrics_v2 = sys_v2.evaluate_outage_performance(res_v2, df_v)

    # Pre-outage GNSS trajectory (t = 80s to 100s, rows 800 to 1000)
    pre_v = df_v.iloc[max(0, start_idx - 200):start_idx + 1]
    pre_e, pre_n = [], []
    for _, row in pre_v.iterrows():
        e_r, n_r, _ = geodetic_to_enu(float(row["Latitude (degrees)"]), float(row["Longitude (degrees)"]), 0.0, origin)
        pre_e.append(e_r)
        pre_n.append(n_r)

    # Outage Reference Trajectory
    ref_slice = df_v.iloc[start_idx:start_idx + n_samples]
    ref_e, ref_n = [], []
    for _, row in ref_slice.iterrows():
        e_r, n_r, _ = geodetic_to_enu(float(row["Latitude (degrees)"]), float(row["Longitude (degrees)"]), 0.0, origin)
        ref_e.append(e_r)
        ref_n.append(n_r)

    v1_e = [p.east_m for p in res_v1.points]
    v1_n = [p.north_m for p in res_v1.points]
    v2_e = [p.east_m for p in res_v2.points]
    v2_n = [p.north_m for p in res_v2.points]
    t_outage = [p.t_rel_sec - t0 for p in res_v2.points]

    ref_headings = ref_slice["Heading (degrees)"].values
    ref_speeds = (ref_slice["velocity_m_s"] if "velocity_m_s" in ref_slice else ref_slice["Indicated Vehicle Speed (km/hr)"] / 3.6).values

    v1_headings = [p.heading_deg for p in res_v1.points]
    v2_headings = [p.heading_deg for p in res_v2.points]
    v2_speeds = [p.velocity_m_s for p in res_v2.points]

    v1_pos_errs = [np.sqrt((v1_e[i] - ref_e[i])**2 + (v1_n[i] - ref_n[i])**2) for i in range(n_samples)]
    v2_pos_errs = [np.sqrt((v2_e[i] - ref_e[i])**2 + (v2_n[i] - ref_n[i])**2) for i in range(n_samples)]

    v1_head_errs = [abs((v1_headings[i] - ref_headings[i] + 180) % 360 - 180) for i in range(n_samples)]
    v2_head_errs = [abs((v2_headings[i] - ref_headings[i] + 180) % 360 - 180) for i in range(n_samples)]

    # --- PLOT 1: Professional Trajectory Comparison Plot ---
    plt.figure(figsize=(11, 8))
    plt.plot(pre_e, pre_n, color='#2ca02c', linewidth=2.5, linestyle='-', label='Pre-Outage GNSS Track (Available t < 100s)')
    plt.plot(ref_e, ref_n, color='black', linewidth=2.5, linestyle='--', label='VBOX GNSS Reference (Post-Hoc Scoring ONLY)')
    plt.plot(v1_e, v1_n, color='#d62728', linewidth=2.2, linestyle='-.', label=f'V1 Baseline Path (RMSE: {metrics_v1.rmse_position_error_m:.2f}m, Final Err: {metrics_v1.final_position_error_m:.2f}m)')
    plt.plot(v2_e, v2_n, color='#1f77b4', linewidth=2.8, linestyle='-', label=f'V2 Production Path (RMSE: {metrics_v2.rmse_position_error_m:.2f}m, Final Err: {metrics_v2.final_position_error_m:.2f}m)')

    plt.scatter(0, 0, color='red', marker='X', s=160, label='GNSS Outage Barrier (t=100.0s)', zorder=6)
    plt.scatter(v2_e[200], v2_n[200], color='#ff7f0e', marker='o', s=140, label='Adaptive Estimator Switch M5.1 -> M9.1 (t=120.0s)', zorder=6)

    plt.title("SIH 2026 PS-168: V1 Baseline vs V2 Production Dead Reckoning Comparison", fontweight='bold', fontsize=12)
    plt.xlabel("East Position (meters)", fontsize=10)
    plt.ylabel("North Position (meters)", fontsize=10)
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.axis('equal')
    plt.legend(loc='best', fontsize=9)
    plt.tight_layout()

    traj_plot_path = os.path.join(output_dir, "v2_trajectory_comparison.png")
    plt.savefig(traj_plot_path, dpi=150)
    plt.close()

    # --- PLOT 2: Dynamics & Position/Heading Error vs Time ---
    fig, (ax_pos, ax_head, ax_spd) = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

    # Pos Error
    ax_pos.plot(t_outage, v1_pos_errs, color='#d62728', linewidth=2.0, linestyle='--', label=f'V1 Position Error (Final: {metrics_v1.final_position_error_m:.2f}m)')
    ax_pos.plot(t_outage, v2_pos_errs, color='#1f77b4', linewidth=2.5, linestyle='-', label=f'V2 Position Error (Final: {metrics_v2.final_position_error_m:.2f}m)')
    ax_pos.axvline(20.0, color='#ff7f0e', linestyle=':', label='M5.1 -> M9.1 Switch (t=20s)')
    ax_pos.set_ylabel("Position Error (m)")
    ax_pos.set_title("V2 Performance Audit: Drift Reduction & Dynamics", fontweight='bold')
    ax_pos.grid(True, linestyle='--', alpha=0.6)
    ax_pos.legend(loc='upper left')

    # Head Error
    ax_head.plot(t_outage, v1_head_errs, color='#d62728', linewidth=2.0, linestyle='--', label='V1 Heading Error (°)')
    ax_head.plot(t_outage, v2_head_errs, color='#1f77b4', linewidth=2.5, linestyle='-', label='V2 Heading Error (°)')
    ax_head.axvline(20.0, color='#ff7f0e', linestyle=':')
    ax_head.set_ylabel("Heading Error (°)")
    ax_head.grid(True, linestyle='--', alpha=0.6)
    ax_head.legend(loc='upper left')

    # Speed
    ax_spd.plot(t_outage, ref_speeds, color='black', linewidth=2.0, linestyle='--', label='VBOX Speed (Reference)')
    ax_spd.plot(t_outage, v2_speeds, color='#1f77b4', linewidth=2.0, label='V2 Estimated Speed (ECU Input)')
    ax_spd.axvline(20.0, color='#ff7f0e', linestyle=':')
    ax_spd.set_xlabel("Outage Duration (seconds)")
    ax_spd.set_ylabel("Vehicle Speed (m/s)")
    ax_spd.grid(True, linestyle='--', alpha=0.6)
    ax_spd.legend(loc='upper left')

    plt.tight_layout()
    dyn_plot_path = os.path.join(output_dir, "v2_dynamics_and_errors.png")
    plt.savefig(dyn_plot_path, dpi=150)
    plt.close()

    imprv_rmse = ((metrics_v1.rmse_position_error_m - metrics_v2.rmse_position_error_m) / metrics_v1.rmse_position_error_m) * 100.0
    imprv_final = ((metrics_v1.final_position_error_m - metrics_v2.final_position_error_m) / metrics_v1.final_position_error_m) * 100.0

    print("======================================================================")
    print("          V2 DEMONSTRATION & VALIDATION METRIC SUMMARY")
    print("======================================================================")
    print(f"  Outage Duration:                {outage_duration_sec:.1f} seconds ({n_samples} samples)")
    print(f"  V1 Baseline RMSE:               {metrics_v1.rmse_position_error_m:6.2f} meters")
    print(f"  V2 Production RMSE:             {metrics_v2.rmse_position_error_m:6.2f} meters (Improvement: {imprv_rmse:+.1f}%)")
    print(f"  V1 Final Position Error:        {metrics_v1.final_position_error_m:6.2f} meters")
    print(f"  V2 Final Position Error:        {metrics_v2.final_position_error_m:6.2f} meters (Improvement: {imprv_final:+.1f}%)")
    print(f"  V2 Maximum Position Error:      {metrics_v2.max_position_error_m:6.2f} meters")
    print("======================================================================")
    print(f"\n[DEMONSTRATION PLOTS SAVED]\n  -> Trajectory Comparison: '{traj_plot_path}'\n  -> Dynamics & Error Plot:  '{dyn_plot_path}'\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="SIH 2026 PS-168 V2 Validation Package Runner")
    parser.add_argument("--vehicle_csv", type=str, default="d:/prototype/data/processed/S1/V-S1_processed.csv")
    parser.add_argument("--smartphone_csv", type=str, default="d:/prototype/data/processed/S1/S-S1_processed.csv")
    parser.add_argument("--output_dir", type=str, default="d:/prototype/results/v2_validation_package")
    parser.add_argument("--start_idx", type=int, default=1000)
    parser.add_argument("--duration", type=float, default=30.0)

    args = parser.parse_args()
    run_v2_validation_package(
        vehicle_csv=args.vehicle_csv,
        smartphone_csv=args.smartphone_csv,
        output_dir=args.output_dir,
        start_idx=args.start_idx,
        outage_duration_sec=args.duration
    )

"""
CLI Runner for Real-Time Dataset Replay Software-in-the-Loop Demonstration.
Streams synchronized multi-sensor data sample-by-sample at real-time 10Hz pacing (configurable)
and updates navigation state incrementally with live Matplotlib trajectory visualizer.
"""

import os
import time
import argparse
from typing import Dict, Any, Optional
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from src.iovnbd.preprocessing.schema_validation import (
    validate_vehicle_dataframe_schema,
    validate_smartphone_dataframe_schema
)
from src.iovnbd.navigation.initialization import initialize_navigation_state
from src.iovnbd.navigation.csv_replay_streamer import CSVReplayStreamer
from src.iovnbd.navigation.streaming_runner import StreamingNavigationRunner
from src.iovnbd.navigation.coordinate_frame import AnchorOrigin, geodetic_to_enu
from src.iovnbd.navigation.final_navigation import get_final_competition_system

def run_realtime_replay_cli(
    vehicle_csv: str,
    smartphone_csv: str,
    output_dir: str = "d:/prototype/results/realtime_replay",
    start_idx: int = 1000,
    outage_duration_sec: float = 120.0,
    replay_speed: float = 1.0,
    show_plot: bool = True
) -> Dict[str, Any]:
    """
    Executes the Real-Time Dataset Replay Demonstration CLI.
    """
    t_start_wall = time.time()
    os.makedirs(output_dir, exist_ok=True)

    print("=" * 85)
    print("      SIH 2026 PROBLEM STATEMENT 168 — INTELLIGENT DEAD RECKONING PROTOTYPE")
    print("        REAL-TIME DATASET REPLAY DEMONSTRATION (SOFTWARE-IN-THE-LOOP)")
    print("=" * 85)
    print("\n  MODE:                   DATASET REPLAY — NOT LIVE PHYSICAL HARDWARE")
    print(f"  SENSOR SAMPLING RATE:   10 Hz Nominal (100 ms per sample payload)")
    print(f"  REPLAY SPEED MULTIPLIER: {replay_speed:.1f}x ({'Maximum Speed' if replay_speed == 0.0 else 'Real-Time Pacing'})")
    print("  GNSS PROVENANCE & MASKING: GNSS ATTRIBUTES STRICTLY MASKED DURING OUTAGE INFERENCE\n")

    # 1. Load and Validate Input Datasets
    print("[STEP 1/4] Resolving and Validating Sensor CSV Datasets...")
    if not os.path.exists(vehicle_csv):
        raise FileNotFoundError(f"Vehicle CSV missing: {vehicle_csv}")
    if not os.path.exists(smartphone_csv):
        raise FileNotFoundError(f"Smartphone CSV missing: {smartphone_csv}")

    df_v = pd.read_csv(vehicle_csv)
    df_s = pd.read_csv(smartphone_csv)
    print(f"  -> Vehicle Dataset (CAN ECU): {len(df_v)} rows loaded from '{vehicle_csv}'")
    print(f"  -> Smartphone Dataset (IMU):  {len(df_s)} rows loaded from '{smartphone_csv}'")

    v_ok, v_errs = validate_vehicle_dataframe_schema(df_v)
    s_ok, s_errs = validate_smartphone_dataframe_schema(df_s)
    if not v_ok or not s_ok:
        raise ValueError(f"CSV Schema validation failed: Vehicle={v_errs}, Smartphone={s_errs}")

    # 2. Initialize Outage State Anchor
    print("\n[STEP 2/4] Initializing Outage State Anchor & Masking Sensor Streams...")
    init_state = initialize_navigation_state(df_v, start_idx=start_idx)
    origin = init_state.origin
    t0 = init_state.t_rel_sec

    print(f"  -> Outage Start Timestamp: t = {t0:.1f} seconds (Row Index: {start_idx})")
    print(f"  -> Outage Target Duration: T = {outage_duration_sec:.1f} seconds")
    print(f"  -> Initial ENU Position:   East=0.0m, North=0.0m")
    print(f"  -> Initial Speed / Yaw:    Speed={init_state.speed_m_s:.2f}m/s | Heading={init_state.heading_deg:.1f}°")

    # Extract VBOX GNSS Reference for post-hoc validation plot
    n_samples = int(round(outage_duration_sec * 10)) + 1
    ref_slice = df_v.iloc[start_idx:start_idx + n_samples]
    ref_e, ref_n = [], []
    for _, row in ref_slice.iterrows():
        lat_r = float(row["Latitude (degrees)"]) if "Latitude (degrees)" in row else origin.lat_deg
        lon_r = float(row["Longitude (degrees)"]) if "Longitude (degrees)" in row else origin.lon_deg
        e_r, n_r, _ = geodetic_to_enu(lat_r, lon_r, 0.0, origin)
        ref_e.append(e_r)
        ref_n.append(n_r)

    # 3. Setup Replay Streamer & Incremental Navigation Runner
    streamer = CSVReplayStreamer(
        df_vehicle=df_v,
        df_smartphone=df_s,
        start_idx=start_idx,
        outage_duration_sec=outage_duration_sec,
        replay_speed=replay_speed
    )

    runner = StreamingNavigationRunner(
        initial_state=init_state,
        mode="adaptive_switch"
    )

    # Setup Interactive Live Plotting if requested
    fig, ax_map, ax_err = None, None, None
    line_est, line_ref, pt_cur = None, None, None

    if show_plot:
        try:
            fig, (ax_map, ax_err) = plt.subplots(1, 2, figsize=(13, 6))

            ax_map.plot(ref_e, ref_n, 'k--', linewidth=2.0, label='VBOX GNSS Reference (Post-Hoc Plot)')
            ax_map.scatter(0, 0, color='blue', s=80, label='Outage Start (t=0s)', zorder=5)
            line_est, = ax_map.plot([], [], color='#2ca02c', linewidth=2.5, label='Estimated Navigation Path')
            pt_cur = ax_map.scatter([], [], color='red', s=100, zorder=6, label='Current Estimator Position')
            ax_map.set_title("SIH 2026 PS-168: Live Streaming Navigation Replay", fontweight='bold')
            ax_map.set_xlabel("East Position (m)")
            ax_map.set_ylabel("North Position (m)")
            ax_map.grid(True, linestyle='--', alpha=0.6)
            ax_map.legend(loc='best')

            ax_err.set_title("Position Drift Error Growth (m)")
            ax_err.set_xlabel("Outage Duration (s)")
            ax_err.set_ylabel("Error (m)")
            ax_err.grid(True, linestyle='--', alpha=0.6)

            plt.tight_layout()
        except Exception as e:
            print(f"  -> Notice: GUI plot window disabled ({e}). Continuing headless.")
            show_plot = False

    print("\n" + "-" * 75)
    print("                     STARTING REAL-TIME SENSOR STREAM REPLAY")
    print("-" * 75)

    est_e_hist, est_n_hist, t_rel_hist, err_hist = [], [], [], []

    # 4. Stream Loop (Sample-by-Sample)
    for sample in streamer.stream_samples():
        pt = runner.process_sample(sample)

        est_e_hist.append(pt.east_m)
        est_n_hist.append(pt.north_m)
        t_rel_hist.append(pt.t_rel_sec - t0)

        # Calculate current position error against reference if available
        curr_idx = len(est_e_hist) - 1
        if curr_idx < len(ref_e):
            err_m = np.sqrt((pt.east_m - ref_e[curr_idx])**2 + (pt.north_m - ref_n[curr_idx])**2)
        else:
            err_m = 0.0
        err_hist.append(err_m)

        # Print periodic progress to console every 10 samples (1 second) or on switch
        if (runner.sample_count % 10 == 0) or pt.switch_event or runner.sample_count == len(streamer):
            sw_str = " *** ADAPTIVE SWITCH TO M9.1 ***" if pt.switch_event else ""
            print(f"  t = {pt.t_rel_sec - t0:5.1f}s | Active Estimator: [{pt.active_estimator:4s}] | Speed: {pt.velocity_m_s:5.2f} m/s | Heading: {pt.heading_deg:5.1f}° | Roll: {pt.roll_deg:5.1f}° | Pos Error: {err_m:5.2f}m{sw_str}")

    print("-" * 75)
    print("                    REAL-TIME SENSOR STREAM REPLAY COMPLETE")
    print("-" * 75)

    fused_res = runner.get_fused_result(outage_duration_sec=outage_duration_sec)
    system = get_final_competition_system()
    metrics = system.evaluate_outage_performance(fused_res, df_v)

    total_wall = time.time() - t_start_wall

    print("\n" + "=" * 70)
    print("         REAL-TIME DATASET REPLAY FINAL PERFORMANCE SUMMARY")
    print("=" * 70)
    print(f"  Total Streaming Samples Processed: {runner.sample_count}")
    print(f"  Outage Duration:                   {outage_duration_sec:.1f} seconds")
    print(f"  Streaming Fused Prototype RMSE:     {metrics.rmse_position_error_m:6.2f} meters")
    print(f"  Streaming Final Position Error:     {metrics.final_position_error_m:6.2f} meters")
    print(f"  Total Wall Clock Runtime:          {total_wall:.2f} seconds")
    print("=" * 70)

    # Save final replay plot
    plt.figure(figsize=(10, 7))
    plt.plot(ref_e, ref_n, 'k--', linewidth=2.5, label='VBOX GNSS Reference (Post-Hoc Plot)')
    plt.plot(est_e_hist, est_n_hist, color='#2ca02c', linewidth=2.5, label=f'Streaming Estimator Path (RMSE: {metrics.rmse_position_error_m:.2f}m)')
    plt.scatter(0, 0, color='blue', s=100, label='Outage Start Point', zorder=6)

    plt.title(f"SIH 2026 PS-168: Real-Time Streaming Navigation Replay ({outage_duration_sec:.0f}s Outage)", fontweight='bold')
    plt.xlabel("East Position (m)")
    plt.ylabel("North Position (m)")
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend(loc='best')
    plt.tight_layout()

    final_plot_path = os.path.join(output_dir, "realtime_replay_trajectory.png")
    plt.savefig(final_plot_path, dpi=150)
    plt.close('all')

    print(f"\n[OUTPUT SAVED] Final streaming trajectory plot saved to '{final_plot_path}'\n")

    return {
        "status": "A — REALTIME STREAMING REPLAY COMPLETE",
        "sample_count": runner.sample_count,
        "rmse_position_error_m": round(metrics.rmse_position_error_m, 2),
        "final_position_error_m": round(metrics.final_position_error_m, 2),
        "wall_runtime_sec": round(total_wall, 2),
        "final_plot": final_plot_path
    }

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="SIH 2026 PS-168 Real-Time Dataset Replay CLI")
    parser.add_argument("--vehicle_csv", type=str, default="d:/prototype/data/processed/S1/V-S1_processed.csv", help="Path to vehicle CAN CSV")
    parser.add_argument("--smartphone_csv", type=str, default="d:/prototype/data/processed/S1/S-S1_processed.csv", help="Path to smartphone IMU CSV")
    parser.add_argument("--output_dir", type=str, default="d:/prototype/results/realtime_replay", help="Output directory")
    parser.add_argument("--start_idx", type=int, default=1000, help="Row index for outage start")
    parser.add_argument("--duration", type=float, default=120.0, help="Outage duration in seconds")
    parser.add_argument("--replay_speed", type=float, default=1.0, help="Replay speed (1.0 = real-time 10Hz, 0.0 = max speed)")
    parser.add_argument("--no_plot", action="store_true", help="Disable live GUI plot window")

    args = parser.parse_args()
    run_realtime_replay_cli(
        vehicle_csv=args.vehicle_csv,
        smartphone_csv=args.smartphone_csv,
        output_dir=args.output_dir,
        start_idx=args.start_idx,
        outage_duration_sec=args.duration,
        replay_speed=args.replay_speed,
        show_plot=(not args.no_plot)
    )

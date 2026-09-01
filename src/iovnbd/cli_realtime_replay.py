"""
CLI Runner for Real-Time Dataset Replay Software-in-the-Loop Demonstration (V2.1 Production).
Streams synchronized multi-sensor data sample-by-sample at real-time 10Hz pacing (configurable)
and updates V1, V2.0, and V2.1 navigation states simultaneously with live dark-mode Matplotlib dashboard.
"""

import os
import time
import argparse
from typing import Dict, Any, Optional
import pandas as pd
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

from src.iovnbd.preprocessing.schema_validation import (
    validate_vehicle_dataframe_schema,
    validate_smartphone_dataframe_schema
)
from src.iovnbd.navigation.initialization import initialize_navigation_state
from src.iovnbd.navigation.csv_replay_streamer import CSVReplayStreamer
from src.iovnbd.navigation.streaming_runner import StreamingNavigationRunner
from src.iovnbd.navigation.coordinate_frame import geodetic_to_enu
from src.iovnbd.navigation.final_navigation import get_final_competition_system
from src.iovnbd.navigation.ekf_m5_1 import compute_dynamic_yaw_scale

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
    Executes the V2.1 Real-Time Dataset Replay Demonstration CLI.
    Runs V1, V2.0, and V2.1 simultaneously sample-by-sample with interactive live GUI support.
    """
    t_start_wall = time.time()
    os.makedirs(output_dir, exist_ok=True)

    print("=" * 85)
    print("      SIH 2026 PROBLEM STATEMENT 168 — INTELLIGENT DEAD RECKONING PROTOTYPE")
    print("      V1 BASELINE vs V2.0 BASELINE vs V2.1 PRODUCTION LIVE DEMONSTRATION")
    print("=" * 85)
    print("\n  MODE:                   SOFTWARE-IN-THE-LOOP (SIL) STREAMING REPLAY")
    print(f"  SENSOR SAMPLING RATE:   10 Hz Nominal (100 ms per sample payload)")
    print(f"  REPLAY SPEED MULTIPLIER: {replay_speed:.1f}x ({'Maximum Speed' if replay_speed == 0.0 else 'Real-Time Pacing'})")
    print("  GNSS PROVENANCE & MASKING: GNSS ATTRIBUTES STRICTLY MASKED DURING OUTAGE INFERENCE")
    print("  VBOX GROUND TRUTH:      USED STRICTLY POST-HOC FOR VISUALIZATION & SCORING ONLY\n")

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

    # Extract VBOX GNSS Reference for post-hoc scoring/plotting (Strictly isolated from estimators)
    n_samples = int(round(outage_duration_sec * 10)) + 1
    ref_slice = df_v.iloc[start_idx:start_idx + n_samples]
    ref_e, ref_n = [], []
    for _, row in ref_slice.iterrows():
        lat_r = float(row["Latitude (degrees)"]) if "Latitude (degrees)" in row else origin.lat_deg
        lon_r = float(row["Longitude (degrees)"]) if "Longitude (degrees)" in row else origin.lon_deg
        e_r, n_r, _ = geodetic_to_enu(lat_r, lon_r, 0.0, origin)
        ref_e.append(e_r)
        ref_n.append(n_r)

    # 3. Setup Synchronized Replay Streamer & 3 Estimator Runners
    streamer = CSVReplayStreamer(
        df_vehicle=df_v,
        df_smartphone=df_s,
        start_idx=start_idx,
        outage_duration_sec=outage_duration_sec,
        replay_speed=replay_speed
    )

    runner_v1 = StreamingNavigationRunner(
        initial_state=init_state, mode="adaptive_switch", yaw_scale_factor=1.0, dynamic_yaw_scale_enabled=False
    )
    runner_v2_0 = StreamingNavigationRunner(
        initial_state=init_state, mode="adaptive_switch", yaw_scale_factor=0.95, dynamic_yaw_scale_enabled=False
    )
    runner_v2_1 = StreamingNavigationRunner(
        initial_state=init_state, mode="adaptive_switch", yaw_scale_factor=0.90, dynamic_yaw_scale_enabled=True
    )

    # Setup Interactive Live Plot Window if requested
    fig, ax_map, ax_err = None, None, None
    line_v1, line_v20, line_v21 = None, None, None
    marker_v1, marker_v20, marker_v21 = None, None, None
    line_err_v1, line_err_v20, line_err_v21 = None, None, None

    if show_plot:
        try:
            plt.ion()
            fig = plt.figure(figsize=(14, 8), facecolor='#0e1117')
            ax_map = plt.subplot(1, 2, 1, facecolor='#161b22')
            ax_err = plt.subplot(1, 2, 2, facecolor='#161b22')

            ax_map.plot(ref_e, ref_n, color='#8b949e', linestyle='--', linewidth=2.0, label='VBOX Reference (Post-Hoc Only)')
            line_v1, = ax_map.plot([], [], color='#f85149', linestyle='-.', linewidth=2.0, label='V1 Baseline (Yaw Scale=1.0)')
            line_v20, = ax_map.plot([], [], color='#58a6ff', linestyle='--', linewidth=2.2, label='V2.0 Baseline (Yaw Scale=0.95)')
            line_v21, = ax_map.plot([], [], color='#3fb950', linestyle='-', linewidth=2.8, label='V2.1 Production (Dynamic Scale)')

            ax_map.scatter(0, 0, color='#d29922', s=120, label='GNSS Outage Start t=100s (0,0)', zorder=6)
            marker_v1 = ax_map.scatter([], [], color='#f85149', s=90, zorder=8)
            marker_v20 = ax_map.scatter([], [], color='#58a6ff', s=90, zorder=8)
            marker_v21 = ax_map.scatter([], [], color='#3fb950', s=130, edgecolors='white', zorder=9, label='Live V2.1 Position')

            ax_map.set_title("V2.1 LIVE NAVIGATION TRAJECTORY MAP", color='white', fontweight='bold', fontsize=12)
            ax_map.set_xlabel("East Position (meters)", color='#c9d1d9')
            ax_map.set_ylabel("North Position (meters)", color='#c9d1d9')
            ax_map.tick_params(colors='#c9d1d9')
            ax_map.grid(True, linestyle=':', alpha=0.4, color='#30363d')
            ax_map.legend(loc='best', facecolor='#161b22', edgecolor='#30363d', labelcolor='white', fontsize=9)
            ax_map.axis('equal')

            line_err_v1, = ax_err.plot([], [], color='#f85149', linestyle='-.', linewidth=2.0, label='V1 Error')
            line_err_v20, = ax_err.plot([], [], color='#58a6ff', linestyle='--', linewidth=2.2, label='V2.0 Error')
            line_err_v21, = ax_err.plot([], [], color='#3fb950', linestyle='-', linewidth=2.8, label='V2.1 Error')

            ax_err.set_title("POSITION DRIFT ERROR PROGRESSION (m)", color='white', fontweight='bold', fontsize=12)
            ax_err.set_xlabel("Elapsed Outage Time (seconds)", color='#c9d1d9')
            ax_err.set_ylabel("Position Error (meters)", color='#c9d1d9')
            ax_err.tick_params(colors='#c9d1d9')
            ax_err.grid(True, linestyle=':', alpha=0.4, color='#30363d')
            ax_err.legend(loc='best', facecolor='#161b22', edgecolor='#30363d', labelcolor='white', fontsize=9)

            plt.suptitle("SIH 2026 PS-168 INTELLIGENT DEAD RECKONING PROTOTYPE — V2.1 LIVE DASHBOARD", color='white', fontweight='bold', fontsize=14, y=0.98)
            plt.tight_layout(rect=[0, 0, 1, 0.95])
            fig.canvas.draw()
            plt.pause(0.01)
        except Exception as e:
            print(f"  -> Notice: Interactive GUI window non-interactive ({e}). Operating in background mode.")

    print("\n" + "-" * 75)
    print("                     STARTING REAL-TIME SENSOR STREAM REPLAY")
    print("-" * 75)

    v1_e_hist, v1_n_hist, v1_err_hist = [], [], []
    v2_0_e_hist, v2_0_n_hist, v2_0_err_hist = [], [], []
    v2_1_e_hist, v2_1_n_hist, v2_1_err_hist = [], [], []
    t_rel_hist = []

    # 4. Stream Loop (Sample-by-Sample)
    for sample in streamer.stream_samples():
        pt_v1 = runner_v1.process_sample(sample)
        pt_v2_0 = runner_v2_0.process_sample(sample)
        pt_v2_1 = runner_v2_1.process_sample(sample)

        v1_e_hist.append(pt_v1.east_m)
        v1_n_hist.append(pt_v1.north_m)
        v2_0_e_hist.append(pt_v2_0.east_m)
        v2_0_n_hist.append(pt_v2_0.north_m)
        v2_1_e_hist.append(pt_v2_1.east_m)
        v2_1_n_hist.append(pt_v2_1.north_m)

        t_elapsed = pt_v2_1.t_rel_sec - t0
        t_rel_hist.append(t_elapsed)

        curr_idx = len(v2_1_e_hist) - 1
        ref_curr_e = ref_e[curr_idx] if curr_idx < len(ref_e) else ref_e[-1]
        ref_curr_n = ref_n[curr_idx] if curr_idx < len(ref_n) else ref_n[-1]

        err_v1 = np.sqrt((pt_v1.east_m - ref_curr_e)**2 + (pt_v1.north_m - ref_curr_n)**2)
        err_v2_0 = np.sqrt((pt_v2_0.east_m - ref_curr_e)**2 + (pt_v2_0.north_m - ref_curr_n)**2)
        err_v2_1 = np.sqrt((pt_v2_1.east_m - ref_curr_e)**2 + (pt_v2_1.north_m - ref_curr_n)**2)

        v1_err_hist.append(err_v1)
        v2_0_err_hist.append(err_v2_0)
        v2_1_err_hist.append(err_v2_1)

        # Update Live Interactive Plots if active
        if show_plot and fig is not None and line_v21 is not None:
            try:
                line_v1.set_data(v1_e_hist, v1_n_hist)
                line_v20.set_data(v2_0_e_hist, v2_0_n_hist)
                line_v21.set_data(v2_1_e_hist, v2_1_n_hist)

                marker_v1.set_offsets([[v1_e_hist[-1], v1_n_hist[-1]]])
                marker_v20.set_offsets([[v2_0_e_hist[-1], v2_0_n_hist[-1]]])
                marker_v21.set_offsets([[v2_1_e_hist[-1], v2_1_n_hist[-1]]])

                line_err_v1.set_data(t_rel_hist, v1_err_hist)
                line_err_v20.set_data(t_rel_hist, v2_0_err_hist)
                line_err_v21.set_data(t_rel_hist, v2_1_err_hist)

                ax_map.relim()
                ax_map.autoscale_view()
                ax_err.relim()
                ax_err.autoscale_view()

                fig.canvas.draw_idle()
                fig.canvas.flush_events()
                if replay_speed > 0.0:
                    plt.pause(0.001)
            except Exception:
                pass

        # Print periodic progress to console every 10 samples (1 second) or on switch
        if (runner_v2_1.sample_count % 10 == 0) or pt_v2_1.switch_event or runner_v2_1.sample_count == len(streamer):
            sw_str = " *** ADAPTIVE SWITCH TO M9.1 ***" if pt_v2_1.switch_event else ""
            imprv_v1 = ((err_v1 - err_v2_1) / err_v1 * 100.0) if err_v1 > 0 else 0.0
            print(f"  t = {t_elapsed:5.1f}s | Mode: [{pt_v2_1.active_estimator:4s}] | Spd: {pt_v2_1.velocity_m_s:5.2f}m/s | Head: {pt_v2_1.heading_deg:5.1f}° | V1: {err_v1:5.2f}m | V2.0: {err_v2_0:5.2f}m | V2.1: {err_v2_1:5.2f}m (Gain: +{imprv_v1:.1f}% vs V1){sw_str}")

    print("-" * 75)
    print("                    REAL-TIME SENSOR STREAM REPLAY COMPLETE")
    print("-" * 75)

    # 5. Compute Dynamic Performance Summary Metrics
    rmse_v1 = np.sqrt(np.mean(np.square(v1_err_hist)))
    rmse_v2_0 = np.sqrt(np.mean(np.square(v2_0_err_hist)))
    rmse_v2_1 = np.sqrt(np.mean(np.square(v2_1_err_hist)))

    final_err_v1 = v1_err_hist[-1]
    final_err_v2_0 = v2_0_err_hist[-1]
    final_err_v2_1 = v2_1_err_hist[-1]

    max_err_v1 = np.max(v1_err_hist)
    max_err_v2_0 = np.max(v2_0_err_hist)
    max_err_v2_1 = np.max(v2_1_err_hist)

    imprv_rmse_v1 = ((rmse_v1 - rmse_v2_1) / rmse_v1) * 100.0
    imprv_rmse_v20 = ((rmse_v2_0 - rmse_v2_1) / rmse_v2_0) * 100.0

    total_wall = time.time() - t_start_wall

    print("\n" + "=" * 75)
    print("         REAL-TIME DATASET REPLAY FINAL PERFORMANCE SUMMARY")
    print("=" * 75)
    print(f"  Total Streaming Samples Processed: {runner_v2_1.sample_count}")
    print(f"  Outage Duration:                   {outage_duration_sec:.1f} seconds")
    print(f"  V1 Baseline Position RMSE:          {rmse_v1:6.2f} meters")
    print(f"  V2.0 Baseline Position RMSE:        {rmse_v2_0:6.2f} meters")
    print(f"  V2.1 Production Position RMSE:      {rmse_v2_1:6.2f} meters (Gain: +{imprv_rmse_v1:.1f}% vs V1, +{imprv_rmse_v20:.1f}% vs V2.0)")
    print(f"  V1 Final Position Error:            {final_err_v1:6.2f} meters")
    print(f"  V2.0 Final Position Error:          {final_err_v2_0:6.2f} meters")
    print(f"  V2.1 Final Position Error:          {final_err_v2_1:6.2f} meters")
    print(f"  Total Wall Clock Runtime:          {total_wall:.2f} seconds")
    print("=" * 75)

    # Save final static plot snapshot
    if fig is None:
        fig = plt.figure(figsize=(14, 8), facecolor='#0e1117')
        ax_map = plt.subplot(1, 2, 1, facecolor='#161b22')
        ax_err = plt.subplot(1, 2, 2, facecolor='#161b22')

        ax_map.plot(ref_e, ref_n, color='#8b949e', linestyle='--', linewidth=2.0, label='VBOX Reference (Post-Hoc Scoring Only)')
        ax_map.plot(v1_e_hist, v1_n_hist, color='#f85149', linestyle='-.', linewidth=2.0, label=f'V1 Baseline (RMSE: {rmse_v1:.2f}m)')
        ax_map.plot(v2_0_e_hist, v2_0_n_hist, color='#58a6ff', linestyle='--', linewidth=2.2, label=f'V2.0 Baseline (RMSE: {rmse_v2_0:.2f}m)')
        ax_map.plot(v2_1_e_hist, v2_1_n_hist, color='#3fb950', linestyle='-', linewidth=2.8, label=f'V2.1 Production (RMSE: {rmse_v2_1:.2f}m)')

        ax_map.scatter(0, 0, color='#d29922', s=120, label='GNSS Outage Start t=100s (0,0)', zorder=6)
        if len(v2_1_e_hist) > 200:
            ax_map.scatter(v2_1_e_hist[200], v2_1_n_hist[200], color='#a371f7', s=140, marker='*', label='M5.1 -> M9.1 Switch (t=20s)', zorder=7)

        ax_map.scatter(v1_e_hist[-1], v1_n_hist[-1], color='#f85149', s=90, zorder=8)
        ax_map.scatter(v2_0_e_hist[-1], v2_0_n_hist[-1], color='#58a6ff', s=90, zorder=8)
        ax_map.scatter(v2_1_e_hist[-1], v2_1_n_hist[-1], color='#3fb950', s=130, edgecolors='white', zorder=9, label='Current V2.1 Position')

        ax_map.set_title("V2.1 LIVE NAVIGATION TRAJECTORY MAP", color='white', fontweight='bold', fontsize=12)
        ax_map.set_xlabel("East Position (meters)", color='#c9d1d9')
        ax_map.set_ylabel("North Position (meters)", color='#c9d1d9')
        ax_map.tick_params(colors='#c9d1d9')
        ax_map.grid(True, linestyle=':', alpha=0.4, color='#30363d')
        ax_map.legend(loc='best', facecolor='#161b22', edgecolor='#30363d', labelcolor='white', fontsize=9)
        ax_map.axis('equal')

        ax_err.plot(t_rel_hist, v1_err_hist, color='#f85149', linestyle='-.', linewidth=2.0, label=f'V1 Error (Final: {final_err_v1:.2f}m)')
        ax_err.plot(t_rel_hist, v2_0_err_hist, color='#58a6ff', linestyle='--', linewidth=2.2, label=f'V2.0 Error (Final: {final_err_v2_0:.2f}m)')
        ax_err.plot(t_rel_hist, v2_1_err_hist, color='#3fb950', linestyle='-', linewidth=2.8, label=f'V2.1 Error (Final: {final_err_v2_1:.2f}m)')

        ax_err.set_title(f"POSITION DRIFT GROWTH (V2.1 Gain: +{imprv_rmse_v20:.1f}% vs V2.0)", color='white', fontweight='bold', fontsize=12)
        ax_err.set_xlabel("Elapsed Outage Time (seconds)", color='#c9d1d9')
        ax_err.set_ylabel("Position Error (meters)", color='#c9d1d9')
        ax_err.tick_params(colors='#c9d1d9')
        ax_err.grid(True, linestyle=':', alpha=0.4, color='#30363d')
        ax_err.legend(loc='best', facecolor='#161b22', edgecolor='#30363d', labelcolor='white', fontsize=9)

        plt.suptitle("SIH 2026 PS-168 INTELLIGENT DEAD RECKONING PROTOTYPE — V2.1 LIVE DASHBOARD", color='white', fontweight='bold', fontsize=14, y=0.98)
        plt.tight_layout(rect=[0, 0, 1, 0.95])

    final_plot_path = os.path.join(output_dir, "realtime_replay_trajectory.png")
    fig.savefig(final_plot_path, dpi=150, facecolor='#0e1117')
    if show_plot:
        plt.ioff()
    plt.close('all')

    print(f"\n[OUTPUT SAVED] Final live demonstration dashboard plot saved to '{final_plot_path}'\n")

    return {
        "status": "V2.1 REALTIME STREAMING REPLAY COMPLETE",
        "sample_count": runner_v2_1.sample_count,
        "rmse_v1_m": round(rmse_v1, 2),
        "rmse_v2_0_m": round(rmse_v2_0, 2),
        "rmse_v2_1_m": round(rmse_v2_1, 2),
        "final_err_v1_m": round(final_err_v1, 2),
        "final_err_v2_0_m": round(final_err_v2_0, 2),
        "final_err_v2_1_m": round(final_err_v2_1, 2),
        "gain_vs_v1_percent": round(imprv_rmse_v1, 1),
        "gain_vs_v20_percent": round(imprv_rmse_v20, 1),
        "wall_runtime_sec": round(total_wall, 2),
        "final_plot": final_plot_path
    }

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="SIH 2026 PS-168 V2.1 Real-Time Dataset Replay CLI")
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

"""
SIH 2026 Problem Statement 168: Intelligent Dead Reckoning Prototype
V2.0 Live Navigation Demonstration CLI.

Reads CSV streams and executes the production V2 StreamingNavigationRunner sample-by-sample (10 Hz).
Visualizes live 2D trajectory development, adaptive M5.1 -> M9.1 switching, and live telemetry panel.
"""

import os
import sys
import time
import argparse
import copy
from typing import Dict, Any, List, Optional

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
from src.iovnbd.navigation.coordinate_frame import geodetic_to_enu

def run_v2_live_demo(
    vehicle_csv: str = "data/processed/S1/V-S1_processed.csv",
    smartphone_csv: str = "data/processed/S1/S-S1_processed.csv",
    output_dir: str = "results/v2_live_demo",
    start_idx: int = 1000,
    outage_duration_sec: float = 30.0,
    replay_speed: float = 1.0,
    show_plot: bool = True
) -> Dict[str, Any]:
    """
    Runs the V2 Live Navigation Demonstration.
    """
    os.makedirs(output_dir, exist_ok=True)

    print("=" * 85)
    print("      SIH 2026 PROBLEM STATEMENT 168 — INTELLIGENT DEAD RECKONING PROTOTYPE")
    print("               VERSION 2.0 LIVE NAVIGATION DEMONSTRATION")
    print("=" * 85)
    print("\n  MODE:                   SOFTWARE-IN-THE-LOOP (SIL) DATASET REPLAY DEMONSTRATION")
    print(f"  REPLAY PACING:          {replay_speed:.1f}x Real-Time Pacing ({'Maximum Speed' if replay_speed == 0.0 else f'{10.0*replay_speed:.0f} Hz'})")
    print("  GNSS PROVENANCE:        GNSS POSITION & VELOCITY STRICTLY MASKED FROM ESTIMATOR")
    print("  ESTIMATOR SETTINGS:     V2 Production Baseline (yaw_scale_factor = 0.95, Lateral NHC Active)\n")

    if not os.path.exists(vehicle_csv):
        raise FileNotFoundError(f"Vehicle CSV missing: {vehicle_csv}")
    if not os.path.exists(smartphone_csv):
        raise FileNotFoundError(f"Smartphone CSV missing: {smartphone_csv}")

    df_v = pd.read_csv(vehicle_csv)
    df_s = pd.read_csv(smartphone_csv)

    v_ok, v_errs = validate_vehicle_dataframe_schema(df_v)
    s_ok, s_errs = validate_smartphone_dataframe_schema(df_s)
    if not v_ok or not s_ok:
        raise ValueError(f"Schema validation failed: Vehicle={v_errs}, Smartphone={s_errs}")

    n_samples = int(round(outage_duration_sec * 10)) + 1
    init_state = initialize_navigation_state(df_v, start_idx=start_idx)
    origin = init_state.origin
    t0 = init_state.t_rel_sec

    # Extract VBOX reference path ONLY for offline post-hoc scoring and visualization
    ref_slice = df_v.iloc[start_idx:start_idx + n_samples]
    ref_e, ref_n = [], []
    for _, row in ref_slice.iterrows():
        lat_r = float(row["Latitude (degrees)"]) if "Latitude (degrees)" in row else origin.lat_deg
        lon_r = float(row["Longitude (degrees)"]) if "Longitude (degrees)" in row else origin.lon_deg
        e_r, n_r, _ = geodetic_to_enu(lat_r, lon_r, 0.0, origin)
        ref_e.append(e_r)
        ref_n.append(n_r)

    ref_headings = ref_slice["Heading (degrees)"].values
    ref_speeds = (ref_slice["velocity_m_s"] if "velocity_m_s" in ref_slice else ref_slice["Indicated Vehicle Speed (km/hr)"] / 3.6).values

    # Setup Causal Streamer & Production V2 Runner
    streamer = CSVReplayStreamer(
        df_vehicle=df_v,
        df_smartphone=df_s,
        start_idx=start_idx,
        outage_duration_sec=outage_duration_sec,
        replay_speed=replay_speed
    )

    runner = StreamingNavigationRunner(
        initial_state=init_state,
        mode="adaptive_switch",
        yaw_scale_factor=0.95
    )

    # Prepare Interactive Matplotlib Visualization Layout
    fig = plt.figure(figsize=(14, 8), facecolor='#0d1117')
    gs = fig.add_gridspec(1, 2, width_ratios=[1.8, 1.0])

    ax_map = fig.add_subplot(gs[0, 0], facecolor='#161b22')
    ax_status = fig.add_subplot(gs[0, 1], facecolor='#161b22')

    for ax in [ax_map, ax_status]:
        ax.tick_params(colors='#8b949e')
        for spine in ax.spines.values():
            spine.set_color('#30363d')

    # Configure Main Trajectory Map
    ax_map.set_title("V2 LIVE NAVIGATION TRAJECTORY (ENU FRAME)", color='#e6edf3', fontweight='bold', fontsize=12, pad=10)
    ax_map.plot(ref_e, ref_n, color='#8b949e', linestyle='--', linewidth=2.0, label='VBOX Ref (Post-Hoc Scoring ONLY)')
    ax_map.scatter(0, 0, color='#38d9a9', s=140, label='GNSS Outage Start (t=100.0s)', zorder=6)

    line_v2, = ax_map.plot([], [], color='#4dabf7', linewidth=3.0, label='V2 Estimated Navigation Path')
    pt_v2_cur = ax_map.scatter([], [], color='#ffd43b', s=140, zorder=7, label='V2 Current Estimator Position')
    pt_switch_map = ax_map.scatter([], [], color='#ff922b', s=160, marker='*', zorder=8, label='M5.1 -> M9.1 Adaptive Switch')

    ax_map.set_xlabel("East Position (meters)", color='#8b949e')
    ax_map.set_ylabel("North Position (meters)", color='#8b949e')
    ax_map.grid(True, linestyle='--', alpha=0.3, color='#30363d')
    ax_map.legend(loc='best', facecolor='#161b22', edgecolor='#30363d', labelcolor='#e6edf3', fontsize=9)

    ax_status.axis('off')

    print("-" * 75)
    print("                STARTING V2 LIVE DEMONSTRATION REPLAY")
    print("-" * 75)

    v2_e_hist, v2_n_hist, pos_err_hist, t_hist = [], [], [], []
    switch_e, switch_n, switch_t = [], [], []

    for sample in streamer.stream_samples():
        pt = runner.process_sample(sample)

        curr_idx = len(v2_e_hist)
        e_ref_curr = ref_e[curr_idx] if curr_idx < len(ref_e) else 0.0
        n_ref_curr = ref_n[curr_idx] if curr_idx < len(ref_n) else 0.0
        pos_err = np.sqrt((pt.east_m - e_ref_curr)**2 + (pt.north_m - n_ref_curr)**2)

        v2_e_hist.append(pt.east_m)
        v2_n_hist.append(pt.north_m)
        pos_err_hist.append(pos_err)

        t_elapsed = pt.t_rel_sec - t0
        t_hist.append(t_elapsed)

        running_rmse = np.sqrt(np.mean(np.square(pos_err_hist)))

        if pt.switch_event:
            switch_e.append(pt.east_m)
            switch_n.append(pt.north_m)
            switch_t.append(t_elapsed)

        # Update Live Status Panel & Console Output
        if curr_idx % 5 == 0 or curr_idx == n_samples - 1 or pt.switch_event:
            line_v2.set_data(v2_e_hist, v2_n_hist)
            pt_v2_cur.set_offsets(np.c_[[pt.east_m], [pt.north_m]])
            if len(switch_e) > 0:
                pt_switch_map.set_offsets(np.c_[switch_e, switch_n])

            ax_status.clear()
            ax_status.set_facecolor('#161b22')
            ax_status.axis('off')
            ax_status.set_title("V2 LIVE NAVIGATION TELEMETRY", color='#e6edf3', fontweight='bold', fontsize=12, pad=10)

            switch_msg = f"ADAPTIVE SWITCH: M5.1 -> M9.1 (t={switch_t[-1]:.1f}s)" if len(switch_t) > 0 else "Normal Propagation"

            status_box = (
                "─────────────────────────────────────────\n"
                "  V2 LIVE NAVIGATION STATUS\n"
                "─────────────────────────────────────────\n"
                f"  Replay Speed:       {replay_speed:.1f}x Pacing\n"
                f"  Replay Time:        t = {t_elapsed:5.2f} s / {outage_duration_sec:.1f} s\n"
                f"  Outage Time:        t = {100.0 + t_elapsed:5.1f} s\n\n"
                f"  Active Estimator:   [{pt.active_estimator:4s}]\n"
                f"  Estimator Event:    {switch_msg}\n\n"
                f"  Vehicle Speed:      {pt.velocity_m_s:5.2f} m/s ({pt.velocity_m_s*3.6:5.1f} km/h)\n"
                f"  Vehicle Heading:    {pt.heading_deg:5.1f}°\n"
                f"  Vehicle Roll:       {pt.roll_deg:5.2f}°\n\n"
                f"  East Position:      {pt.east_m:6.2f} m\n"
                f"  North Position:     {pt.north_m:6.2f} m\n\n"
                f"  Position Error:     {pos_err:5.2f} m\n"
                f"  Running RMSE:       {running_rmse:5.2f} m\n\n"
                "─────────────────────────────────────────\n"
                "  SYSTEM CONFIGURATION & SAFETY\n"
                "─────────────────────────────────────────\n"
                "  Yaw Scale Factor:   0.95\n"
                "  GNSS Status:        MASKED (Zero Data Leakage)\n"
                "  NHC Status:         ACTIVE (Zero Lateral Slip)\n"
                "─────────────────────────────────────────"
            )
            ax_status.text(0.05, 0.5, status_box, color='#e6edf3', fontsize=9.5, fontfamily='monospace', va='center')

        if (runner.sample_count % 10 == 0) or pt.switch_event or runner.sample_count == n_samples:
            sw_str = " *** ADAPTIVE SWITCH M5.1 -> M9.1 ***" if pt.switch_event else ""
            print(f"  t = {t_elapsed:5.1f}s | Active Estimator: [{pt.active_estimator:4s}] | Speed: {pt.velocity_m_s:5.2f}m/s | Heading: {pt.heading_deg:5.1f}° | Pos Error: {pos_err:5.2f}m | Running RMSE: {running_rmse:5.2f}m{sw_str}")

    # Final Summary Calculation
    final_rmse = np.sqrt(np.mean(np.square(pos_err_hist)))
    final_pos_err = pos_err_hist[-1]
    max_pos_err = np.max(pos_err_hist)

    print("-" * 75)
    print("           V2 LIVE NAVIGATION DEMONSTRATION COMPLETE")
    print("-" * 75)

    summary_banner = (
        "============================================================\n"
        "         V2 LIVE NAVIGATION DEMONSTRATION COMPLETE\n"
        "============================================================\n"
        f"  Outage Duration:          {outage_duration_sec:.1f} s\n"
        f"  Samples Processed:        {runner.sample_count}\n"
        f"  Final RMSE:               {final_rmse:6.2f} m\n"
        f"  Final Position Error:     {final_pos_err:6.2f} m\n"
        f"  Maximum Position Error:   {max_pos_err:6.2f} m\n\n"
        "  V2 Configuration:\n"
        "  Yaw Scale Factor:         0.95\n"
        "  NHC:                      Enabled\n"
        "  Adaptive Estimator:       M5.1 -> M9.1\n\n"
        "  GNSS Leakage:\n"
        "  During outage:            STRICTLY MASKED\n"
        "============================================================"
    )
    print(summary_banner)

    # Save Final Visualization Snapshot
    plt.tight_layout()
    plot_path = os.path.join(output_dir, "v2_live_navigation.png")
    plt.savefig(plot_path, dpi=150)
    plt.close()

    print(f"\n[OUTPUT SAVED] Final live demonstration plot saved to '{plot_path}'\n")

    return {
        "status": "DEMO COMPLETE",
        "samples_processed": runner.sample_count,
        "final_rmse": round(final_rmse, 2),
        "final_pos_err": round(final_pos_err, 2),
        "max_pos_err": round(max_pos_err, 2),
        "plot_path": plot_path
    }

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="SIH 2026 PS-168 V2 Live Navigation Demo CLI")
    parser.add_argument("--vehicle_csv", type=str, default="data/processed/S1/V-S1_processed.csv")
    parser.add_argument("--smartphone_csv", type=str, default="data/processed/S1/S-S1_processed.csv")
    parser.add_argument("--output_dir", type=str, default="results/v2_live_demo")
    parser.add_argument("--start_idx", type=int, default=1000)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--replay_speed", type=float, default=1.0)
    parser.add_argument("--no_plot", action="store_true")

    args = parser.parse_args()
    run_v2_live_demo(
        vehicle_csv=args.vehicle_csv,
        smartphone_csv=args.smartphone_csv,
        output_dir=args.output_dir,
        start_idx=args.start_idx,
        outage_duration_sec=args.duration,
        replay_speed=args.replay_speed,
        show_plot=(not args.no_plot)
    )

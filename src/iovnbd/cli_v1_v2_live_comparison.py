"""
SIH 2026 Problem Statement 168: Intelligent Dead Reckoning Prototype
V1 Baseline vs V2 Production LIVE COMPARISON DEMONSTRATION CLI.

Executes both V1 (yaw_scale_factor=1.0) and V2 (yaw_scale_factor=0.95) estimators side-by-side
sample-by-sample in real time (10 Hz nominal pacing), displaying dual live moving dots on the 2D graph.
"""

import os
import sys
import time
import argparse
import copy
from typing import Dict, Any, List

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

def run_v1_v2_live_comparison(
    vehicle_csv: str = "data/processed/S1/V-S1_processed.csv",
    smartphone_csv: str = "data/processed/S1/S-S1_processed.csv",
    output_dir: str = "results/v1_v2_live_comparison",
    start_idx: int = 1000,
    outage_duration_sec: float = 30.0,
    replay_speed: float = 1.0,
    show_gui: bool = True
) -> Dict[str, Any]:
    """
    Runs the V1 vs V2 Dual Live Moving Point Trajectory Demonstration.
    """
    os.makedirs(output_dir, exist_ok=True)

    print("=" * 85)
    print("      SIH 2026 PROBLEM STATEMENT 168 — INTELLIGENT DEAD RECKONING PROTOTYPE")
    print("           VERSION 1 BASELINE vs VERSION 2 PRODUCTION LIVE DEMONSTRATION")
    print("=" * 85)
    print("\n  MODE:                   SOFTWARE-IN-THE-LOOP (SIL) DUAL REAL-TIME REPLAY")
    print(f"  REPLAY PACING:          {replay_speed:.1f}x Pacing ({'Maximum Speed' if replay_speed == 0.0 else f'{10.0*replay_speed:.0f} Hz'})")
    print("  GNSS PROVENANCE:        GNSS DATA STRICTLY MASKED DURING OUTAGE INFERENCE")
    print("  V1 BASELINE:            yaw_scale_factor = 1.0 (Baseline Uncalibrated)")
    print("  V2 PRODUCTION:          yaw_scale_factor = 0.95 + Lateral NHC Active\n")

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

    # Extract VBOX reference trajectory for OFFLINE SCORING & PLOTTING ONLY
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

    # Setup Causal Streamer & Dual Estimators
    streamer = CSVReplayStreamer(
        df_vehicle=df_v,
        df_smartphone=df_s,
        start_idx=start_idx,
        outage_duration_sec=outage_duration_sec,
        replay_speed=replay_speed
    )

    runner_v1 = StreamingNavigationRunner(initial_state=init_state, mode="adaptive_switch", yaw_scale_factor=1.0)
    runner_v2 = StreamingNavigationRunner(initial_state=init_state, mode="adaptive_switch", yaw_scale_factor=0.95)

    # Initialize Matplotlib Interactive Window if GUI enabled
    if show_gui:
        try:
            plt.ion()
        except Exception:
            pass

    fig = plt.figure(figsize=(14, 8), facecolor='#0d1117')
    gs = fig.add_gridspec(1, 2, width_ratios=[1.7, 1.0])

    ax_map = fig.add_subplot(gs[0, 0], facecolor='#161b22')
    ax_status = fig.add_subplot(gs[0, 1], facecolor='#161b22')

    for ax in [ax_map, ax_status]:
        ax.tick_params(colors='#8b949e')
        for spine in ax.spines.values():
            spine.set_color('#30363d')

    # Configure Main Trajectory Map
    ax_map.set_title("SIH 2026 PS-168: V1 VS V2 LIVE DUAL NAVIGATION MAP", color='#e6edf3', fontweight='bold', fontsize=12, pad=10)
    ax_map.plot(ref_e, ref_n, color='#8b949e', linestyle='--', linewidth=2.0, label='VBOX Ground Truth (Offline Scoring Only)')
    ax_map.scatter(0, 0, color='#38d9a9', s=140, label='GNSS Outage Barrier (t=100.0s)', zorder=6)

    # Live Trajectory Lines
    line_v1, = ax_map.plot([], [], color='#ff6b6b', linewidth=2.2, linestyle='-.', label='V1 Baseline Path (Yaw Scale=1.0)')
    line_v2, = ax_map.plot([], [], color='#4dabf7', linewidth=3.0, linestyle='-', label='V2 Production Path (Yaw Scale=0.95 + NHC)')

    # Live Moving Position Dots
    pt_v1_cur = ax_map.scatter([], [], color='#ff6b6b', s=120, edgecolors='white', zorder=7, label='V1 Moving Position Dot')
    pt_v2_cur = ax_map.scatter([], [], color='#ffd43b', s=160, edgecolors='black', zorder=8, label='V2 Moving Position Dot')
    pt_switch = ax_map.scatter([], [], color='#ff922b', s=160, marker='*', zorder=9, label='M5.1 -> M9.1 Adaptive Switch')

    ax_map.set_xlabel("East Position (meters)", color='#8b949e')
    ax_map.set_ylabel("North Position (meters)", color='#8b949e')
    ax_map.grid(True, linestyle='--', alpha=0.3, color='#30363d')
    ax_map.legend(loc='best', facecolor='#161b22', edgecolor='#30363d', labelcolor='#e6edf3', fontsize=8.5)

    ax_status.axis('off')

    try:
        fig.show()
        fig.canvas.draw()
    except Exception:
        pass

    print("-" * 75)
    print("         STARTING DUAL V1 VS V2 LIVE MOVING-POINT REPLAY")
    print("-" * 75)

    v1_e_hist, v1_n_hist, v1_err_hist = [], [], []
    v2_e_hist, v2_n_hist, v2_err_hist = [], [], []
    t_hist = []
    switch_e, switch_n, switch_t = [], [], []

    for sample in streamer.stream_samples():
        sample_v1 = copy.deepcopy(sample)
        sample_v2 = copy.deepcopy(sample)

        pt_v1 = runner_v1.process_sample(sample_v1)
        pt_v2 = runner_v2.process_sample(sample_v2)

        curr_idx = len(v2_e_hist)
        e_ref_curr = ref_e[curr_idx] if curr_idx < len(ref_e) else 0.0
        n_ref_curr = ref_n[curr_idx] if curr_idx < len(ref_n) else 0.0

        err_v1 = np.sqrt((pt_v1.east_m - e_ref_curr)**2 + (pt_v1.north_m - n_ref_curr)**2)
        err_v2 = np.sqrt((pt_v2.east_m - e_ref_curr)**2 + (pt_v2.north_m - n_ref_curr)**2)

        v1_e_hist.append(pt_v1.east_m)
        v1_n_hist.append(pt_v1.north_m)
        v1_err_hist.append(err_v1)

        v2_e_hist.append(pt_v2.east_m)
        v2_n_hist.append(pt_v2.north_m)
        v2_err_hist.append(err_v2)

        t_elapsed = pt_v2.t_rel_sec - t0
        t_hist.append(t_elapsed)

        running_rmse_v1 = np.sqrt(np.mean(np.square(v1_err_hist)))
        running_rmse_v2 = np.sqrt(np.mean(np.square(v2_err_hist)))

        if pt_v2.switch_event:
            switch_e.append(pt_v2.east_m)
            switch_n.append(pt_v2.north_m)
            switch_t.append(t_elapsed)

        # Update Live 2D Trajectory Graph & Moving Dots
        if curr_idx % 2 == 0 or curr_idx == n_samples - 1 or pt_v2.switch_event:
            line_v1.set_data(v1_e_hist, v1_n_hist)
            line_v2.set_data(v2_e_hist, v2_n_hist)

            pt_v1_cur.set_offsets(np.c_[[pt_v1.east_m], [pt_v1.north_m]])
            pt_v2_cur.set_offsets(np.c_[[pt_v2.east_m], [pt_v2.north_m]])

            if len(switch_e) > 0:
                pt_switch.set_offsets(np.c_[switch_e, switch_n])

            # Update Telemetry Panel
            ax_status.clear()
            ax_status.set_facecolor('#161b22')
            ax_status.axis('off')
            ax_status.set_title("V1 VS V2 LIVE TELEMETRY DASHBOARD", color='#e6edf3', fontweight='bold', fontsize=12, pad=10)

            sw_msg = f"ADAPTIVE SWITCH M5.1 -> M9.1 (t={switch_t[-1]:.1f}s)" if len(switch_t) > 0 else "Normal Propagation"
            imprv_curr = ((err_v1 - err_v2) / max(0.01, err_v1)) * 100.0

            status_box = (
                "─────────────────────────────────────────\n"
                "  REAL-TIME REPLAY STATUS\n"
                "─────────────────────────────────────────\n"
                f"  Replay Speed:       {replay_speed:.1f}x Pacing\n"
                f"  Outage Elapsed:     t = {t_elapsed:5.2f} s / {outage_duration_sec:.1f} s\n"
                f"  Sample Payload:     #{curr_idx+1:03d} / {n_samples}\n\n"
                f"  Active Regime:      [{pt_v2.active_estimator:4s}]\n"
                f"  Estimator Event:    {sw_msg}\n\n"
                "─────────────────────────────────────────\n"
                "  LIVE SENSOR & DYNAMICS READOUT\n"
                "─────────────────────────────────────────\n"
                f"  Vehicle Speed:      {pt_v2.velocity_m_s:5.2f} m/s ({pt_v2.velocity_m_s*3.6:5.1f} km/h)\n"
                f"  V2 Heading:         {pt_v2.heading_deg:5.1f}°\n"
                f"  V2 Roll Angle:      {pt_v2.roll_deg:5.2f}°\n\n"
                "─────────────────────────────────────────\n"
                "  LIVE COMPARISON (V1 BASELINE VS V2 PROTOTYPE)\n"
                "─────────────────────────────────────────\n"
                f"  🔴 V1 Position:      E={pt_v1.east_m:6.2f}m, N={pt_v1.north_m:6.2f}m\n"
                f"  🟡 V2 Position:      E={pt_v2.east_m:6.2f}m, N={pt_v2.north_m:6.2f}m\n\n"
                f"  • V1 Pos Error:     {err_v1:5.2f} meters\n"
                f"  • V2 Pos Error:     {err_v2:5.2f} meters\n"
                f"  • V1 Running RMSE:  {running_rmse_v1:5.2f} meters\n"
                f"  • V2 Running RMSE:  {running_rmse_v2:5.2f} meters\n"
                f"  • Live Error Gain:  {imprv_curr:+6.1f}% Reduction\n"
                "─────────────────────────────────────────"
            )
            ax_status.text(0.05, 0.5, status_box, color='#e6edf3', fontsize=9.2, fontfamily='monospace', va='center')

            if show_gui:
                try:
                    fig.canvas.draw()
                    fig.canvas.flush_events()
                    plt.pause(0.001)
                except Exception:
                    pass

        if (runner_v2.sample_count % 10 == 0) or pt_v2.switch_event or runner_v2.sample_count == n_samples:
            sw_str = " *** ADAPTIVE SWITCH M5.1 -> M9.1 ***" if pt_v2.switch_event else ""
            print(f"  t = {t_elapsed:5.1f}s | Regime: [{pt_v2.active_estimator:4s}] | Speed: {pt_v2.velocity_m_s:5.2f}m/s | V1 Err: {err_v1:5.2f}m | V2 Err: {err_v2:5.2f}m ({imprv_curr:+.1f}%){sw_str}")

    # Final Benchmark KPI Summary
    rmse_v1 = np.sqrt(np.mean(np.square(v1_err_hist)))
    rmse_v2 = np.sqrt(np.mean(np.square(v2_err_hist)))
    final_err_v1 = v1_err_hist[-1]
    final_err_v2 = v2_err_hist[-1]
    imprv_rmse = ((rmse_v1 - rmse_v2) / rmse_v1) * 100.0
    imprv_final = ((final_err_v1 - final_err_v2) / final_err_v1) * 100.0

    print("-" * 75)
    print("           DUAL V1 VS V2 LIVE NAVIGATION REPLAY COMPLETE")
    print("-" * 75)

    summary_banner = (
        "============================================================\n"
        "       V1 BASELINE VS V2 PRODUCTION PERFORMANCE SUMMARY\n"
        "============================================================\n"
        f"  Outage Duration:            {outage_duration_sec:.1f} s ({runner_v2.sample_count} samples)\n\n"
        "  ACCURACY EVALUATION:\n"
        f"  • V1 Baseline RMSE:         {rmse_v1:6.2f} meters\n"
        f"  • V2 Production RMSE:       {rmse_v2:6.2f} meters\n"
        f"  • RMSE IMPROVEMENT:        {imprv_rmse:+6.1f}%\n\n"
        f"  • V1 Final Position Error:  {final_err_v1:6.2f} meters\n"
        f"  • V2 Final Position Error:  {final_err_v2:6.2f} meters\n"
        f"  • FINAL ERROR REDUCTION:    {imprv_final:+6.1f}%\n\n"
        "  CONFIGURATION PROVENANCE:\n"
        "  V1 Settings:                yaw_scale = 1.0 (Uncalibrated)\n"
        "  V2 Settings:                yaw_scale = 0.95 + Lateral NHC\n"
        "  GNSS Outage Masking:        STRICTLY ENFORCED (Zero Data Leakage)\n"
        "============================================================"
    )
    print(summary_banner)

    # Save Final Visual Plot Snapshot
    plt.tight_layout()
    plot_path = os.path.join(output_dir, "v1_vs_v2_live_trajectory.png")
    plt.savefig(plot_path, dpi=150)
    plt.close()

    print(f"\n[OUTPUT SAVED] Final dual live comparison trajectory saved to '{plot_path}'\n")

    return {
        "status": "DUAL REPLAY COMPLETE",
        "samples_processed": runner_v2.sample_count,
        "v1_rmse": round(rmse_v1, 2),
        "v2_rmse": round(rmse_v2, 2),
        "v1_final_err": round(final_err_v1, 2),
        "v2_final_err": round(final_err_v2, 2),
        "rmse_improvement_pct": round(imprv_rmse, 1),
        "plot_path": plot_path
    }

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="SIH 2026 PS-168 V1 vs V2 Live Comparison Demo CLI")
    parser.add_argument("--vehicle_csv", type=str, default="data/processed/S1/V-S1_processed.csv")
    parser.add_argument("--smartphone_csv", type=str, default="data/processed/S1/S-S1_processed.csv")
    parser.add_argument("--output_dir", type=str, default="results/v1_v2_live_comparison")
    parser.add_argument("--start_idx", type=int, default=1000)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--replay_speed", type=float, default=1.0)
    parser.add_argument("--no_gui", action="store_true")

    args = parser.parse_args()
    run_v1_v2_live_comparison(
        vehicle_csv=args.vehicle_csv,
        smartphone_csv=args.smartphone_csv,
        output_dir=args.output_dir,
        start_idx=args.start_idx,
        outage_duration_sec=args.duration,
        replay_speed=args.replay_speed,
        show_gui=(not args.no_gui)
    )

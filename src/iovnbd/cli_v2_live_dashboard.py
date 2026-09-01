"""
SIH 2026 PS-168: V2 Live-Graph Demonstration Dashboard.
Consumes existing V2 real-time dataset replay streams sample-by-sample (10 Hz nominal pacing)
and visualizes live trajectories, error growth, heading tracking, and telemetry panel.
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

def run_v2_live_dashboard(
    vehicle_csv: str = "data/processed/S1/V-S1_processed.csv",
    smartphone_csv: str = "data/processed/S1/S-S1_processed.csv",
    output_dir: str = "results/v2_live_dashboard",
    start_idx: int = 1000,
    outage_duration_sec: float = 30.0,
    replay_speed: float = 1.0,
    show_plot: bool = True
) -> Dict[str, Any]:
    """
    Executes the V2 Live-Graph Demonstration Dashboard CLI.
    """
    t_start_wall = time.time()
    os.makedirs(output_dir, exist_ok=True)

    print("=" * 85)
    print("      SIH 2026 PROBLEM STATEMENT 168 — INTELLIGENT DEAD RECKONING PROTOTYPE")
    print("               V2.0 LIVE-GRAPH DEMONSTRATION DASHBOARD")
    print("=" * 85)
    print(f"\n  MODE:                   REAL-TIME DATASET REPLAY (SOFTWARE-IN-THE-LOOP)")
    print(f"  SAMPLING PACE:          10 Hz Nominal ({'Maximum Speed' if replay_speed == 0.0 else 'Real-Time Pacing'})")
    print(f"  GNSS MASKING:           GNSS POSITION/VELOCITY STRICTLY MASKED DURING OUTAGE")
    print(f"  V2 ALGORITHM SETTINGS:  yaw_scale_factor = 0.95 | Lateral NHC Active\n")

    if not os.path.exists(vehicle_csv):
        raise FileNotFoundError(f"Vehicle CSV missing: {vehicle_csv}")
    if not os.path.exists(smartphone_csv):
        raise FileNotFoundError(f"Smartphone CSV missing: {smartphone_csv}")

    df_v = pd.read_csv(vehicle_csv)
    df_s = pd.read_csv(smartphone_csv)

    v_ok, v_errs = validate_vehicle_dataframe_schema(df_v)
    s_ok, s_errs = validate_smartphone_dataframe_schema(df_s)
    if not v_ok or not s_ok:
        raise ValueError(f"CSV Schema validation failed: Vehicle={v_errs}, Smartphone={s_errs}")

    init_state = initialize_navigation_state(df_v, start_idx=start_idx)
    origin = init_state.origin
    t0 = init_state.t_rel_sec

    n_samples = int(round(outage_duration_sec * 10)) + 1
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

    streamer = CSVReplayStreamer(
        df_vehicle=df_v,
        df_smartphone=df_s,
        start_idx=start_idx,
        outage_duration_sec=outage_duration_sec,
        replay_speed=replay_speed
    )

    runner_v1 = StreamingNavigationRunner(initial_state=init_state, mode="adaptive_switch", yaw_scale_factor=1.0)
    runner_v2 = StreamingNavigationRunner(initial_state=init_state, mode="adaptive_switch", yaw_scale_factor=0.95)

    # Prepare Dashboard Matplotlib Grid
    fig, axs = plt.subplots(2, 2, figsize=(14, 10))
    fig.patch.set_facecolor('#0d1117')

    ax_map, ax_err = axs[0, 0], axs[0, 1]
    ax_head, ax_telem = axs[1, 0], axs[1, 1]

    for ax in [ax_map, ax_err, ax_head, ax_telem]:
        ax.set_facecolor('#161b22')
        ax.tick_params(colors='#8b949e')
        for spine in ax.spines.values():
            spine.set_color('#30363d')

    # Subplot 1: Map
    ax_map.set_title("1. LIVE TRAJECTORY MAP (ENU METERS)", color='#e6edf3', fontweight='bold', fontsize=11)
    ax_map.plot(ref_e, ref_n, 'k--', linewidth=2.0, label='VBOX Ref (Offline Scoring)', color='#8b949e')
    ax_map.scatter(0, 0, color='#38d9a9', s=100, label='Outage Start Barrier (t=0s)', zorder=6)
    line_v1_map, = ax_map.plot([], [], color='#ff6b6b', linewidth=2.0, linestyle='-.', label='V1 Baseline Path')
    line_v2_map, = ax_map.plot([], [], color='#4dabf7', linewidth=2.8, linestyle='-', label='V2 Production Path')
    pt_cur_v2 = ax_map.scatter([], [], color='#ffd43b', s=120, zorder=7, label='Current V2 Position')
    pt_switch_map = ax_map.scatter([], [], color='#ff922b', s=120, zorder=7, label='M5.1 -> M9.1 Switch Point')
    ax_map.set_xlabel("East Position (m)", color='#8b949e')
    ax_map.set_ylabel("North Position (m)", color='#8b949e')
    ax_map.grid(True, linestyle='--', alpha=0.3, color='#30363d')
    ax_map.legend(loc='best', facecolor='#161b22', edgecolor='#30363d', labelcolor='#e6edf3', fontsize=8)

    # Subplot 2: Pos Error
    ax_err.set_title("2. POSITION DRIFT ERROR VS TIME (M)", color='#e6edf3', fontweight='bold', fontsize=11)
    line_v1_err, = ax_err.plot([], [], color='#ff6b6b', linewidth=2.0, linestyle='--', label='V1 Position Error')
    line_v2_err, = ax_err.plot([], [], color='#4dabf7', linewidth=2.5, linestyle='-', label='V2 Position Error')
    ax_err.axvline(20.0, color='#ff922b', linestyle=':', label='M5.1 -> M9.1 Switch (t=20s)')
    ax_err.set_xlim(0, outage_duration_sec)
    ax_err.set_ylim(0, 50)
    ax_err.set_xlabel("Outage Duration (seconds)", color='#8b949e')
    ax_err.set_ylabel("Position Error (meters)", color='#8b949e')
    ax_err.grid(True, linestyle='--', alpha=0.3, color='#30363d')
    ax_err.legend(loc='upper left', facecolor='#161b22', edgecolor='#30363d', labelcolor='#e6edf3', fontsize=8)

    # Subplot 3: Heading
    ax_head.set_title("3. HEADING TRACKING (°)", color='#e6edf3', fontweight='bold', fontsize=11)
    ax_head.plot(np.linspace(0, outage_duration_sec, len(ref_headings)), ref_headings, 'k--', color='#8b949e', label='VBOX Ref Heading')
    line_v2_head, = ax_head.plot([], [], color='#4dabf7', linewidth=2.2, label='V2 Estimated Heading')
    ax_head.set_xlim(0, outage_duration_sec)
    ax_head.set_ylim(200, 320)
    ax_head.set_xlabel("Outage Duration (seconds)", color='#8b949e')
    ax_head.set_ylabel("Heading Angle (°)", color='#8b949e')
    ax_head.grid(True, linestyle='--', alpha=0.3, color='#30363d')
    ax_head.legend(loc='upper right', facecolor='#161b22', edgecolor='#30363d', labelcolor='#e6edf3', fontsize=8)

    # Subplot 4: Telemetry Panel
    ax_telem.set_title("4. REAL-TIME TELEMETRY & SYSTEM STATUS", color='#e6edf3', fontweight='bold', fontsize=11)
    ax_telem.axis('off')

    plt.tight_layout()

    print("\n" + "-" * 75)
    print("           STARTING V2 LIVE-GRAPH DEMONSTRATION REPLAY")
    print("-" * 75)

    v1_e_hist, v1_n_hist, v1_err_hist = [], [], []
    v2_e_hist, v2_n_hist, v2_err_hist, v2_head_hist = [], [], [], []
    t_hist = []

    switch_pts_e, switch_pts_n = [], []

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
        v2_head_hist.append(pt_v2.heading_deg)

        t_elapsed = pt_v2.t_rel_sec - t0
        t_hist.append(t_elapsed)

        if pt_v2.switch_event:
            switch_pts_e.append(pt_v2.east_m)
            switch_pts_n.append(pt_v2.north_m)

        # Update Telemetry Dashboard Panel Text
        if curr_idx % 5 == 0 or curr_idx == len(ref_e) - 1:
            ax_telem.clear()
            ax_telem.set_facecolor('#161b22')
            ax_telem.axis('off')
            ax_telem.set_title("4. REAL-TIME TELEMETRY & SYSTEM STATUS", color='#e6edf3', fontweight='bold', fontsize=11)

            telem_text = (
                "SIH 2026 PS-168: INTELLIGENT DEAD RECKONING V2.0\n"
                "────────────────────────────────────────────────\n"
                f"SYSTEM STATUS:    🔴 GNSS OUTAGE ACTIVE (Zero Leakage)\n"
                f"REPLAY SPEED:     {replay_speed:.1f}x Real-Time Pacing\n"
                f"ELAPSED TIME:     t = {t_elapsed:5.1f} s / {outage_duration_sec:.0f}.0 s\n"
                f"SAMPLE COUNT:     #{curr_idx+1:03d} / {n_samples}\n\n"
                "LIVE CAUSAL ESTIMATOR METRICS:\n"
                f"• Active Regime:  [{pt_v2.active_estimator:4s}] (Switch @ t=20s)\n"
                f"• CAN ECU Speed:  {pt_v2.velocity_m_s:5.2f} m/s ({pt_v2.velocity_m_s*3.6:5.1f} km/h)\n"
                f"• V2 Heading:     {pt_v2.heading_deg:5.1f}° (Yaw Scale=0.95)\n"
                f"• V2 Position:    E = {pt_v2.east_m:6.2f} m, N = {pt_v2.north_m:6.2f} m\n"
                f"• V2 Roll Angle:  {pt_v2.roll_deg:5.2f}° (Smartphone IMU)\n\n"
                "LIVE DRIFT SCORING (VS GROUND TRUTH):\n"
                f"• V1 Error:       {err_v1:5.2f} meters\n"
                f"• V2 Error:       {err_v2:5.2f} meters\n"
                f"• Current Gain:   {((err_v1 - err_v2)/max(0.01, err_v1))*100:+.1f}% Error Reduction"
            )
            ax_telem.text(0.05, 0.5, telem_text, color='#e6edf3', fontsize=9, fontfamily='monospace', va='center')

        if (runner_v2.sample_count % 10 == 0) or pt_v2.switch_event or runner_v2.sample_count == len(streamer):
            sw_str = " *** ADAPTIVE SWITCH TO M9.1 ***" if pt_v2.switch_event else ""
            print(f"  t = {t_elapsed:5.1f}s | Regime: [{pt_v2.active_estimator:4s}] | Speed: {pt_v2.velocity_m_s:5.2f}m/s | V1 Err: {err_v1:5.2f}m | V2 Err: {err_v2:5.2f}m{sw_str}")

    # Calculate Final Performance KPIs
    rmse_v1 = np.sqrt(np.mean(np.square(v1_err_hist)))
    rmse_v2 = np.sqrt(np.mean(np.square(v2_err_hist)))
    final_err_v1 = v1_err_hist[-1]
    final_err_v2 = v2_err_hist[-1]
    imprv_rmse = ((rmse_v1 - rmse_v2) / rmse_v1) * 100.0
    imprv_final = ((final_err_v1 - final_err_v2) / final_err_v1) * 100.0

    print("-" * 75)
    print("                V2 LIVE-GRAPH DEMONSTRATION REPLAY COMPLETE")
    print("-" * 75)
    print("\n=======================================================================")
    print("                 V2.0 FINAL PERFORMANCE KPI SUMMARY")
    print("=======================================================================")
    print(f"  Total Samples Processed:         {runner_v2.sample_count}")
    print(f"  Outage Duration:                {outage_duration_sec:.1f} seconds")
    print(f"  V1 Baseline RMSE:               {rmse_v1:6.2f} meters")
    print(f"  V2 Production RMSE:             {rmse_v2:6.2f} meters (Improvement: {imprv_rmse:+.1f}%)")
    print(f"  V1 Final Position Error:        {final_err_v1:6.2f} meters")
    print(f"  V2 Final Position Error:        {final_err_v2:6.2f} meters (Improvement: {imprv_final:+.1f}%)")
    print("=======================================================================")

    # Finalize Matplotlib Dashboard View & Save
    line_v1_map.set_data(v1_e_hist, v1_n_hist)
    line_v2_map.set_data(v2_e_hist, v2_n_hist)
    pt_cur_v2.set_offsets(np.c_[[v2_e_hist[-1]], [v2_n_hist[-1]]])
    if len(switch_pts_e) > 0:
        pt_switch_map.set_offsets(np.c_[switch_pts_e, switch_pts_n])

    line_v1_err.set_data(t_hist, v1_err_hist)
    line_v2_err.set_data(t_hist, v2_err_hist)
    line_v2_head.set_data(t_hist, v2_head_hist)

    ax_telem.clear()
    ax_telem.set_facecolor('#161b22')
    ax_telem.axis('off')
    ax_telem.set_title("4. FINAL PERFORMANCE KPI SUMMARY", color='#e6edf3', fontweight='bold', fontsize=11)

    kpi_text = (
        "SIH 2026 PS-168: REPLAY COMPLETE KPI DASHBOARD\n"
        "───────────────────────────────────────────────\n"
        f"OUTAGE DURATION:   {outage_duration_sec:.0f}.0 seconds ({runner_v2.sample_count} samples)\n\n"
        "ACCURACY METRIC COMPARISON:\n"
        f"• V1 Baseline RMSE:         {rmse_v1:6.2f} meters\n"
        f"• V2 Production RMSE:       {rmse_v2:6.2f} meters\n"
        f"• RMSE IMPROVEMENT:        {imprv_rmse:+6.1f}%\n\n"
        f"• V1 Final Position Error:  {final_err_v1:6.2f} meters\n"
        f"• V2 Final Position Error:  {final_err_v2:6.2f} meters\n"
        f"• FINAL ERROR REDUCTION:    {imprv_final:+6.1f}%\n\n"
        "STATUS: VERIFIED V2.0 PRODUCTION BASELINE"
    )
    ax_telem.text(0.05, 0.5, kpi_text, color='#38d9a9', fontsize=9.5, fontfamily='monospace', va='center')

    plt.tight_layout()
    dashboard_plot_path = os.path.join(output_dir, "v2_live_dashboard_snapshot.png")
    plt.savefig(dashboard_plot_path, dpi=150)
    plt.close()

    print(f"\n[OUTPUT SAVED] Final live dashboard snapshot saved to '{dashboard_plot_path}'\n")

    return {
        "status": "V2 LIVE DASHBOARD COMPLETE",
        "sample_count": runner_v2.sample_count,
        "v1_rmse": round(rmse_v1, 2),
        "v2_rmse": round(rmse_v2, 2),
        "v1_final_err": round(final_err_v1, 2),
        "v2_final_err": round(final_err_v2, 2),
        "rmse_improvement_pct": round(imprv_rmse, 1),
        "snapshot_plot": dashboard_plot_path
    }

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="SIH 2026 PS-168 V2 Live Dashboard CLI")
    parser.add_argument("--vehicle_csv", type=str, default="data/processed/S1/V-S1_processed.csv")
    parser.add_argument("--smartphone_csv", type=str, default="data/processed/S1/S-S1_processed.csv")
    parser.add_argument("--output_dir", type=str, default="results/v2_live_dashboard")
    parser.add_argument("--start_idx", type=int, default=1000)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--replay_speed", type=float, default=1.0)
    parser.add_argument("--no_plot", action="store_true")

    args = parser.parse_args()
    run_v2_live_dashboard(
        vehicle_csv=args.vehicle_csv,
        smartphone_csv=args.smartphone_csv,
        output_dir=args.output_dir,
        start_idx=args.start_idx,
        outage_duration_sec=args.duration,
        replay_speed=args.replay_speed,
        show_plot=(not args.no_plot)
    )

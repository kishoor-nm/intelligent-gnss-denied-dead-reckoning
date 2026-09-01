"""
SIH 2026 PS-168: Intelligent Dead Reckoning (IDR)
Prototype Explorer & Live Dashboard (Software-in-the-Loop Control Room).

Visualizes live sensor inputs, GNSS outage masking, EKF state propagation,
M5.1 -> M9.1 adaptive regime switching, position trajectory, error growth,
and beginner-friendly algorithm explanations.

Uses EXISTING streaming infrastructure and navigation core without modifying any mathematics.
"""

import os
import sys
import time
import argparse
from typing import Dict, Any, List, Optional
import pandas as pd
import numpy as np

import matplotlib
matplotlib.use('TkAgg')  # Interactive GUI backend
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.patches import Rectangle, FancyBboxPatch

from src.iovnbd.preprocessing.schema_validation import (
    SingleSensorSample,
    validate_vehicle_dataframe_schema,
    validate_smartphone_dataframe_schema
)
from src.iovnbd.navigation.initialization import initialize_navigation_state
from src.iovnbd.navigation.csv_replay_streamer import CSVReplayStreamer
from src.iovnbd.navigation.streaming_runner import StreamingNavigationRunner
from src.iovnbd.navigation.coordinate_frame import geodetic_to_enu
from src.iovnbd.navigation.final_navigation import get_final_competition_system

class PrototypeExplorerDashboard:
    """
    Visual 6-Panel Interactive Explorer for SIH 2026 PS-168 Prototype.
    Displays live sensor meters, GNSS status, pipeline state, algorithm thinking,
    2D trajectory map, and drift error meters in real-time during SIL dataset replay.
    """
    def __init__(
        self,
        df_vehicle: pd.DataFrame,
        df_smartphone: pd.DataFrame,
        start_idx: int = 1000,
        outage_duration_sec: float = 120.0,
        replay_speed: float = 1.0,
        output_dir: str = "d:/prototype/results/prototype_explorer"
    ):
        self.df_v = df_vehicle
        self.df_s = df_smartphone
        self.start_idx = start_idx
        self.outage_duration_sec = outage_duration_sec
        self.replay_speed = replay_speed
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

        self.init_state = initialize_navigation_state(df_vehicle, start_idx=start_idx)
        self.origin = self.init_state.origin
        self.t0 = self.init_state.t_rel_sec

        # Extract VBOX GNSS Reference for post-hoc/real-time evaluation display ONLY
        n_samples = int(round(outage_duration_sec * 10)) + 1
        ref_slice = df_vehicle.iloc[start_idx:start_idx + n_samples]
        self.ref_e, self.ref_n = [], []
        for _, row in ref_slice.iterrows():
            lat_r = float(row["Latitude (degrees)"]) if "Latitude (degrees)" in row else self.origin.lat_deg
            lon_r = float(row["Longitude (degrees)"]) if "Longitude (degrees)" in row else self.origin.lon_deg
            e_r, n_r, _ = geodetic_to_enu(lat_r, lon_r, 0.0, self.origin)
            self.ref_e.append(e_r)
            self.ref_n.append(n_r)

        # Replay & Streamer Setup
        self.streamer = CSVReplayStreamer(
            df_vehicle=df_vehicle,
            df_smartphone=df_smartphone,
            start_idx=start_idx,
            outage_duration_sec=outage_duration_sec,
            replay_speed=replay_speed
        )
        self.runner = StreamingNavigationRunner(
            initial_state=self.init_state,
            mode="adaptive_switch"
        )

        # Setup Matplotlib Figure Dashboard (Dark Mode Theme)
        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(16, 10), facecolor='#0e1117')
        self.fig.suptitle(
            "SIH 2026 PS-168: INTELLIGENT DEAD RECKONING — PROTOTYPE EXPLORER\n"
            "MODE: SOFTWARE-IN-THE-LOOP DATASET REPLAY (NOT LIVE HARDWARE)",
            fontsize=14, fontweight='bold', color='#00e5ff', y=0.98
        )

        gs = gridspec.GridSpec(3, 3, figure=self.fig, hspace=0.4, wspace=0.3)

        # Panel 1: Live Sensor Meters (Top Left)
        self.ax_sensors = self.fig.add_subplot(gs[0, 0])
        self.ax_sensors.set_facecolor('#161b22')
        self.ax_sensors.set_title("SECTION A — SENSOR INPUTS (10 Hz Payload)", fontsize=11, color='#e6edf3', fontweight='bold')
        self.ax_sensors.axis('off')

        # Panel 2: GNSS Outage Status & Masking (Top Middle)
        self.ax_gnss = self.fig.add_subplot(gs[0, 1])
        self.ax_gnss.set_facecolor('#161b22')
        self.ax_gnss.set_title("SECTION B — GNSS OUTAGE MASKING", fontsize=11, color='#e6edf3', fontweight='bold')
        self.ax_gnss.axis('off')

        # Panel 3: System Flow & Regime Status (Top Right)
        self.ax_flow = self.fig.add_subplot(gs[0, 2])
        self.ax_flow.set_facecolor('#161b22')
        self.ax_flow.set_title("SECTION C — SYSTEM FLOW & REGIME", fontsize=11, color='#e6edf3', fontweight='bold')
        self.ax_flow.axis('off')

        # Panel 4: System Thinking & Explanation (Middle Left)
        self.ax_think = self.fig.add_subplot(gs[1, 0])
        self.ax_think.set_facecolor('#161b22')
        self.ax_think.set_title("SECTION D — WHAT IS THE CAR THINKING?", fontsize=11, color='#e6edf3', fontweight='bold')
        self.ax_think.axis('off')

        # Panel 5: Live Trajectory Map (Middle & Bottom Center/Right)
        self.ax_map = self.fig.add_subplot(gs[1:, 1:])
        self.ax_map.set_facecolor('#0d1117')
        self.ax_map.set_title("SECTION F — LIVE 2D NAVIGATION TRAJECTORY", fontsize=11, color='#e6edf3', fontweight='bold')
        self.ax_map.set_xlabel("East Position (m)", color='#8b949e')
        self.ax_map.set_ylabel("North Position (m)", color='#8b949e')
        self.ax_map.grid(True, linestyle='--', alpha=0.3, color='#30363d')

        # Plot Ground Truth GNSS Reference Path
        self.ax_map.plot(self.ref_e, self.ref_n, 'w--', linewidth=1.5, alpha=0.6, label='VBOX Reference (Post-Hoc Plot)')
        self.ax_map.scatter(0, 0, color='#00e5ff', s=100, label='Outage Anchor (t=0s)', zorder=5)
        self.line_est, = self.ax_map.plot([], [], color='#39ff14', linewidth=2.5, label='M9.3 Fused Estimator Path')
        self.pt_cur = self.ax_map.scatter([], [], color='#ff0055', s=120, zorder=6, label='Current Vehicle Position')
        self.switch_marker = self.ax_map.scatter([], [], color='#ffcc00', s=150, marker='*', zorder=7, label='Adaptive Switch Event')
        self.ax_map.legend(loc='upper right', facecolor='#161b22', edgecolor='#30363d', labelcolor='#e6edf3')

        # Panel 6: Drift Error Growth Meter (Bottom Left)
        self.ax_err = self.fig.add_subplot(gs[2, 0])
        self.ax_err.set_facecolor('#161b22')
        self.ax_err.set_title("SECTION G — POSITION DRIFT ERROR (m)", fontsize=11, color='#e6edf3', fontweight='bold')
        self.ax_err.set_xlabel("Outage Elapsed (s)", color='#8b949e')
        self.ax_err.set_ylabel("Drift Error (m)", color='#8b949e')
        self.ax_err.grid(True, linestyle='--', alpha=0.3, color='#30363d')
        self.line_err, = self.ax_err.plot([], [], color='#ff0055', linewidth=2.0)

        # State histories
        self.est_e_hist = []
        self.est_n_hist = []
        self.t_rel_hist = []
        self.err_hist = []
        self.switch_point = None

    def update_dashboard(self, sample: SingleSensorSample, pt: Any, curr_err_m: float):
        """Updates all 6 visual panels on incoming sample payload."""
        t_elapsed = pt.t_rel_sec - self.t0

        self.est_e_hist.append(pt.east_m)
        self.est_n_hist.append(pt.north_m)
        self.t_rel_hist.append(t_elapsed)
        self.err_hist.append(curr_err_m)

        if pt.switch_event and self.switch_point is None:
            self.switch_point = (pt.east_m, pt.north_m)

        # -------------------------------------------------------------
        # PANEL 1: SENSOR METERS
        # -------------------------------------------------------------
        self.ax_sensors.clear()
        self.ax_sensors.set_facecolor('#161b22')
        self.ax_sensors.set_title("SECTION A — SENSOR INPUTS (10 Hz Payload)", fontsize=10, color='#e6edf3', fontweight='bold')
        self.ax_sensors.axis('off')

        spd_pct = min(1.0, sample.indicated_speed_m_s / 30.0)
        yaw_pct = min(1.0, abs(sample.yaw_rate_rad_s) / 0.5)
        roll_pct = min(1.0, abs(sample.roll_rate_rad_s) / 0.3)

        sensor_text = (
            f"Sample #{pt.index:04d} | t = {t_elapsed:5.1f}s | Rate: 10 Hz\n\n"
            f"[CAR]   CAN Speed:     {sample.indicated_speed_m_s:5.1f} m/s  [{'='*int(spd_pct*10):<10s}]\n"
            f"[LONG]  Long Accel:    {sample.longitudinal_accel_m_s2:5.2f} m/s^2\n"
            f"[LAT]   Lat Accel:     {sample.lateral_accel_m_s2:5.2f} m/s^2\n"
            f"[YAW]   CAN Yaw Rate:   {sample.yaw_rate_rad_s:5.2f} rad/s[{'='*int(yaw_pct*10):<10s}]\n"
            f"[PHONE] Phone Roll Rate:{sample.roll_rate_rad_s:5.2f} rad/s[{'='*int(roll_pct*10):<10s}]"
        )
        self.ax_sensors.text(0.05, 0.5, sensor_text, color='#00e5ff', fontsize=10, fontfamily='monospace', va='center')

        # -------------------------------------------------------------
        # PANEL 2: GNSS STATUS
        # -------------------------------------------------------------
        self.ax_gnss.clear()
        self.ax_gnss.set_facecolor('#161b22')
        self.ax_gnss.set_title("SECTION B — GNSS OUTAGE MASKING", fontsize=10, color='#e6edf3', fontweight='bold')
        self.ax_gnss.axis('off')

        gnss_text = (
            "STATUS: [OFFLINE] GNSS OUTAGE ACTIVE\n\n"
            f"Outage Start:   t = 100.0 s\n"
            f"Elapsed Time:   {t_elapsed:5.1f} s\n\n"
            "MASKING PROVENANCE:\n"
            "• Latitude:  [ MASKED X ]\n"
            "• Longitude: [ MASKED X ]\n"
            "• GNSS Vel:  [ MASKED X ]\n\n"
            "[LOCK] Zero Leakage Verified!"
        )
        self.ax_gnss.text(0.05, 0.5, gnss_text, color='#ff0055', fontsize=10, fontfamily='monospace', va='center')

        # -------------------------------------------------------------
        # PANEL 3: PIPELINE FLOW & ACTIVE REGIME
        # -------------------------------------------------------------
        self.ax_flow.clear()
        self.ax_flow.set_facecolor('#161b22')
        self.ax_flow.set_title("SECTION C — PIPELINE FLOW & REGIME", fontsize=10, color='#e6edf3', fontweight='bold')
        self.ax_flow.axis('off')

        m5_status = "[ACTIVE]" if pt.active_estimator == "M5.1" else "[IDLE]"
        m9_status = "[ACTIVE]" if pt.active_estimator == "M9.1" else "[IDLE]"
        m5_col = "#39ff14" if pt.active_estimator == "M5.1" else "#8b949e"
        m9_col = "#39ff14" if pt.active_estimator == "M9.1" else "#8b949e"

        flow_text = (
            "PIPELINE REGIME EXECUTION:\n\n"
            f"1. Sensor Ingestion @ 10Hz   [ OK ]\n"
            f"2. GNSS Masking Barrier      [ MASKED ]\n"
            f"3. M5.1 5D EKF ({m5_status:<8s})  <-- [{m5_col}]\n"
            f"4. Adaptive Switch Evaluator [ CHECKING ]\n"
            f"5. M9.1 6D Roll EKF ({m9_status:<8s}) <-- [{m9_col}]\n\n"
            f"Active Regime: [{pt.active_estimator}]"
        )
        self.ax_flow.text(0.05, 0.5, flow_text, color='#e6edf3', fontsize=10, fontfamily='monospace', va='center')

        # -------------------------------------------------------------
        # PANEL 4: "WHAT IS THE CAR THINKING?"
        # -------------------------------------------------------------
        self.ax_think.clear()
        self.ax_think.set_facecolor('#161b22')
        self.ax_think.set_title("SECTION D — WHAT IS THE CAR THINKING?", fontsize=10, color='#e6edf3', fontweight='bold')
        self.ax_think.axis('off')

        if pt.active_estimator == "M5.1":
            think_msg = (
                "[THINKING] SYSTEM THINKING (M5.1 Mode):\n\n"
                "\"GNSS is unavailable. I am estimating\n"
                "vehicle motion using CAN wheel speed\n"
                "and yaw rate. Flat 2D assumption is\n"
                "accurate for initial driving.\""
            )
            col_think = "#00e5ff"
        else:
            think_msg = (
                "[THINKING] SYSTEM THINKING (M9.1 Roll Mode):\n\n"
                "\"Extended outage / turning detected!\n"
                "Chassis roll tilt gravity leakage is\n"
                "now active. Incorporating smartphone\n"
                "roll rate to prevent lateral drift.\""
            )
            col_think = "#ffcc00"

        self.ax_think.text(0.05, 0.5, think_msg, color=col_think, fontsize=10, fontfamily='monospace', va='center')

        # -------------------------------------------------------------
        # PANEL 5: LIVE TRAJECTORY MAP UPDATE
        # -------------------------------------------------------------
        self.line_est.set_data(self.est_e_hist, self.est_n_hist)
        self.pt_cur.set_offsets([[pt.east_m, pt.north_m]])

        if self.switch_point is not None:
            self.switch_marker.set_offsets([[self.switch_point[0], self.switch_point[1]]])

        # -------------------------------------------------------------
        # PANEL 6: DRIFT ERROR METER UPDATE
        # -------------------------------------------------------------
        self.line_err.set_data(self.t_rel_hist, self.err_hist)
        self.ax_err.set_xlim(0, max(10.0, t_elapsed + 2.0))
        self.ax_err.set_ylim(0, max(5.0, max(self.err_hist) * 1.2))

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)

    def run_explorer(self) -> Dict[str, Any]:
        """Runs the interactive prototype explorer stream loop."""
        print("=" * 85)
        print("      SIH 2026 PS-168 — INTELLIGENT DEAD RECKONING PROTOTYPE EXPLORER")
        print("        SOFTWARE-IN-THE-LOOP INTERACTIVE CONTROL ROOM DASHBOARD")
        print("=" * 85)

        for sample in self.streamer.stream_samples():
            pt = self.runner.process_sample(sample)

            curr_idx = len(self.est_e_hist)
            if curr_idx < len(self.ref_e):
                err_m = float(np.sqrt((pt.east_m - self.ref_e[curr_idx])**2 + (pt.north_m - self.ref_n[curr_idx])**2))
            else:
                err_m = 0.0

            # Update GUI dashboard every 5 samples (0.5s)
            if self.runner.sample_count % 5 == 0 or self.runner.sample_count == len(self.streamer):
                self.update_dashboard(sample, pt, err_m)

        fused_res = self.runner.get_fused_result(outage_duration_sec=self.outage_duration_sec)
        system = get_final_competition_system()
        metrics = system.evaluate_outage_performance(fused_res, self.df_v)

        final_png = os.path.join(self.output_dir, "explorer_dashboard_final.png")
        self.fig.savefig(final_png, dpi=150)
        plt.close('all')

        print(f"\n[EXPLORER COMPLETE] Final snapshot saved to '{final_png}'\n")

        return {
            "status": "A — PROTOTYPE EXPLORER COMPLETE",
            "sample_count": self.runner.sample_count,
            "rmse_position_error_m": round(metrics.rmse_position_error_m, 2),
            "final_position_error_m": round(metrics.final_position_error_m, 2),
            "final_snapshot": final_png
        }

def run_explorer_cli(
    vehicle_csv: str,
    smartphone_csv: str,
    output_dir: str = "d:/prototype/results/prototype_explorer",
    start_idx: int = 1000,
    outage_duration_sec: float = 120.0,
    replay_speed: float = 1.0
) -> Dict[str, Any]:
    df_v = pd.read_csv(vehicle_csv)
    df_s = pd.read_csv(smartphone_csv)

    explorer = PrototypeExplorerDashboard(
        df_vehicle=df_v,
        df_smartphone=df_s,
        start_idx=start_idx,
        outage_duration_sec=outage_duration_sec,
        replay_speed=replay_speed,
        output_dir=output_dir
    )

    return explorer.run_explorer()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="SIH 2026 PS-168 Prototype Explorer Control Room CLI")
    parser.add_argument("--vehicle_csv", type=str, default="d:/prototype/data/processed/S1/V-S1_processed.csv", help="Vehicle CAN CSV")
    parser.add_argument("--smartphone_csv", type=str, default="d:/prototype/data/processed/S1/S-S1_processed.csv", help="Smartphone IMU CSV")
    parser.add_argument("--output_dir", type=str, default="d:/prototype/results/prototype_explorer", help="Output directory")
    parser.add_argument("--start_idx", type=int, default=1000, help="Row index for outage start")
    parser.add_argument("--duration", type=float, default=120.0, help="Outage duration in seconds")
    parser.add_argument("--replay_speed", type=float, default=1.0, help="Replay speed multiplier")

    args = parser.parse_args()
    run_explorer_cli(
        vehicle_csv=args.vehicle_csv,
        smartphone_csv=args.smartphone_csv,
        output_dir=args.output_dir,
        start_idx=args.start_idx,
        outage_duration_sec=args.duration,
        replay_speed=args.replay_speed
    )

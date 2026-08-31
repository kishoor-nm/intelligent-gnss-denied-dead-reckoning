"""
CLI runner for Module 4.1 Sensor Fusion Audit & Ablation Experiment.
"""

import os
import argparse
import pandas as pd

from src.iovnbd.navigation.experiment_m4_1 import run_module4_1_ablation_suite

def main():
    parser = argparse.ArgumentParser(description="IO-VNBD Module 4.1 Sensor Fusion Audit & Ablation CLI")
    parser.add_argument("--v_processed", type=str, default="d:/prototype/data/processed/S1/V-S1_processed.csv", help="Path to processed V CSV")
    parser.add_argument("--start_idx", type=int, default=1000, help="Row index where GNSS outage begins")
    parser.add_argument("--durations", type=float, nargs="+", default=[10.0, 30.0, 60.0, 120.0], help="Outage durations in seconds")
    parser.add_argument("--output_dir", type=str, default="d:/prototype/results/module4_1", help="Output directory for results and plots")

    args = parser.parse_args()

    print("=" * 75)
    print("IO-VNBD MODULE 4.1: SENSOR-FUSION AUDIT, CORRECTION & ABLATION EXPERIMENT")
    print("=" * 75)
    print(f"Input V CSV: {args.v_processed}")
    print(f"Outage Start Index: {args.start_idx}")
    print(f"Outage Durations: {args.durations} seconds")
    print(f"Output Directory: {args.output_dir}\n")

    df_v = pd.read_csv(args.v_processed)

    results = run_module4_1_ablation_suite(
        df_v=df_v,
        outage_durations=args.durations,
        start_idx=args.start_idx,
        output_dir=args.output_dir
    )

    print("--- SENSOR ABLATION BREAKDOWN (POSITION RMSE IN METERS) ---")
    print(f"{'Duration':<9} | {'Exp A: M3 Baseline':<20} | {'Exp B: 4-Wheel Avg':<20} | {'Exp C: Rear-Wheel Avg':<20}")
    print("-" * 75)

    for ab in results["ablations"]:
        print(f"{ab['outage_duration_sec']:<9.0f} | {ab['exp_a_m3_baseline_rmse_m']:<20.2f} | {ab['exp_b_4wheel_rmse_m']:<20.2f} | {ab['exp_c_rearwheel_rmse_m']:<20.2f}")

    print("-" * 75)
    print(f"\nResults written to: {os.path.join(args.output_dir, 'module4_1_results.json')}")
    print(f"Trajectory plot written to: {os.path.join(args.output_dir, 'm4_1_ablation_trajectory_comparison.png')}")
    print(f"Error growth plot written to: {os.path.join(args.output_dir, 'm4_1_ablation_error_growth.png')}")
    print("=" * 75)

if __name__ == "__main__":
    main()

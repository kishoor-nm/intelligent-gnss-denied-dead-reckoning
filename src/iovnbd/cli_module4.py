"""
CLI runner for Module 4 Improved Sensor Fusion Baseline Experiment.
"""

import os
import argparse
import pandas as pd

from src.iovnbd.navigation.experiment_m4 import run_module4_experiment_suite

def main():
    parser = argparse.ArgumentParser(description="IO-VNBD Module 4 Sensor Fusion Baseline CLI")
    parser.add_argument("--v_processed", type=str, default="d:/prototype/data/processed/S1/V-S1_processed.csv", help="Path to processed V CSV")
    parser.add_argument("--s_processed", type=str, default="d:/prototype/data/processed/S1/S-S1_processed.csv", help="Path to processed S CSV")
    parser.add_argument("--start_idx", type=int, default=1000, help="Row index where GNSS outage begins")
    parser.add_argument("--durations", type=float, nargs="+", default=[10.0, 30.0, 60.0, 120.0], help="Outage durations in seconds")
    parser.add_argument("--output_dir", type=str, default="d:/prototype/results/module4", help="Output directory for results and plots")

    args = parser.parse_args()

    print("=" * 70)
    print("IO-VNBD MODULE 4: IMPROVED SENSOR FUSION BASELINE EXPERIMENT")
    print("=" * 70)
    print(f"Input V CSV: {args.v_processed}")
    print(f"Input S CSV: {args.s_processed}")
    print(f"Outage Start Index: {args.start_idx}")
    print(f"Outage Durations: {args.durations} seconds")
    print(f"Output Directory: {args.output_dir}\n")

    df_v = pd.read_csv(args.v_processed)
    df_s = pd.read_csv(args.s_processed)

    results = run_module4_experiment_suite(
        df_v=df_v,
        df_s=df_s,
        outage_durations=args.durations,
        start_idx=args.start_idx,
        output_dir=args.output_dir
    )

    print("--- QUANTITATIVE COMPARISON: MODULE 3 BASELINE vs MODULE 4 SENSOR FUSION ---")
    print(f"{'Duration':<9} | {'M3 RMSE (m)':<12} | {'M4 RMSE (m)':<12} | {'RMSE Imp (m)':<13} | {'RMSE Imp (%)':<12}")
    print("-" * 70)

    for exp in results["experiments"]:
        print(f"{exp['outage_duration_sec']:<9.0f} | {exp['m3_rmse_m']:<12.2f} | {exp['m4_rmse_m']:<12.2f} | {exp['rmse_improvement_m']:<13.2f} | {exp['rmse_improvement_pct']:<12.1f}%")

    print("-" * 70)
    print(f"\nResults written to: {os.path.join(args.output_dir, 'module4_results.json')}")
    print(f"Trajectory plot written to: {os.path.join(args.output_dir, 'm3_vs_m4_trajectory_comparison.png')}")
    print(f"Error growth plot written to: {os.path.join(args.output_dir, 'm3_vs_m4_error_growth.png')}")
    print("=" * 70)

if __name__ == "__main__":
    main()

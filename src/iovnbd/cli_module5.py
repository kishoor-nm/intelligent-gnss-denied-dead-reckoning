"""
CLI runner for Module 5 5D EKF Dead-Reckoning Core.
"""

import os
import argparse
import pandas as pd

from src.iovnbd.navigation.experiment_m5 import run_module5_experiment_suite

def main():
    parser = argparse.ArgumentParser(description="IO-VNBD Module 5 5D EKF CLI")
    parser.add_argument("--v_processed", type=str, default="d:/prototype/data/processed/S1/V-S1_processed.csv", help="Path to processed V CSV")
    parser.add_argument("--start_idx", type=int, default=1000, help="Row index where GNSS outage begins")
    parser.add_argument("--durations", type=float, nargs="+", default=[10.0, 30.0, 60.0, 120.0], help="Outage durations in seconds")
    parser.add_argument("--output_dir", type=str, default="d:/prototype/results/module5", help="Output directory for results")

    args = parser.parse_args()

    print("=" * 80)
    print("IO-VNBD MODULE 5: 5D EXTENDED KALMAN FILTER (EKF) DEAD-RECKONING CORE")
    print("=" * 80)
    print(f"Input V CSV: {args.v_processed}")
    print(f"Outage Start Index: {args.start_idx}")
    print(f"Outage Durations: {args.durations} seconds")
    print(f"Output Directory: {args.output_dir}\n")

    df_v = pd.read_csv(args.v_processed)

    results = run_module5_experiment_suite(
        df_v=df_v,
        outage_durations=args.durations,
        start_idx=args.start_idx,
        output_dir=args.output_dir
    )

    print("--- COMPARATIVE RESULTS TABLE: MODULE 3 BASELINE vs MODULE 5 5D EKF ---")
    print(f"{'Outage':<9} | {'M3 RMSE (m)':<13} | {'M5 EKF RMSE (m)':<16} | {'M3 Final (m)':<13} | {'M5 EKF Final (m)':<16}")
    print("-" * 75)

    for exp in results["experiments"]:
        print(f"{exp['outage_duration_sec']:<9.0f} | {exp['m3_rmse_m']:<13.2f} | {exp['m5_ekf_rmse_m']:<16.2f} | {exp['m3_final_err_m']:<13.2f} | {exp['m5_ekf_final_err_m']:<16.2f}")

    print("-" * 75)
    print(f"\nResults written to: {os.path.join(args.output_dir, 'module5_results.json')}")
    print(f"Trajectory plot written to: {os.path.join(args.output_dir, 'm5_trajectory_comparison.png')}")
    print(f"Error growth plot written to: {os.path.join(args.output_dir, 'm5_error_growth.png')}")
    print("=" * 80)

if __name__ == "__main__":
    main()

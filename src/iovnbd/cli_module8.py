"""
CLI runner for Module 8 5D NHC-Enhanced Kinematic EKF Experiment.
"""

import os
import argparse
import pandas as pd

from src.iovnbd.navigation.experiment_m8 import run_module8_experiment_suite

def main():
    parser = argparse.ArgumentParser(description="IO-VNBD Module 8 5D NHC-Enhanced EKF CLI")
    parser.add_argument("--output_dir", type=str, default="d:/prototype/results/module8", help="Output directory for results")

    args = parser.parse_args()

    print("=" * 85)
    print("IO-VNBD MODULE 8: 5D NHC-ENHANCED KINEMATIC EKF EXPERIMENT & ABLATION SUITE")
    print("=" * 85)
    print(f"Output Directory: {args.output_dir}\n")

    results = run_module8_experiment_suite(output_dir=args.output_dir)

    print("--- PRIMARY CANONICAL BENCHMARK: M3 vs M5.1 vs M8 NHC EKF ---")
    print(f"{'Outage':<9} | {'M3 RMSE (m)':<12} | {'M5.1 RMSE (m)':<13} | {'M8 NHC RMSE (m)':<16} | {'% Improvement':<14}")
    print("-" * 85)

    for exp in results["experiments"]:
        print(f"{exp['outage_duration_sec']:<9.0f} | {exp['m3_baseline_rmse_m']:<12.2f} | {exp['m5_1_ekf_rmse_m']:<13.2f} | {exp['m8_nhc_ekf_rmse_m']:<16.2f} | {exp['m8_vs_m5_1_pct_improvement']:<+14.1f}%")

    print("-" * 85)
    print("\n--- ABLATION BREAKDOWN ON 120s OUTAGE ---")
    for ab in results["ablations"]:
        print(f"  {ab['name']:<42} -> RMSE: {ab['rmse_m']:6.2f} m | Final Error: {ab['final_m']:6.2f} m")

    print("-" * 85)
    print(f"\nResults written to: {os.path.join(args.output_dir, 'module8_results.json')}")
    print(f"Trajectory plot written to: {os.path.join(args.output_dir, 'm8_trajectory_comparison.png')}")
    print(f"Error growth plot written to: {os.path.join(args.output_dir, 'm8_position_error_growth.png')}")
    print("=" * 85)

if __name__ == "__main__":
    main()

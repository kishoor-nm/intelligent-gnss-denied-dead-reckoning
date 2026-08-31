"""
CLI runner for Module 9 6D Full-Orientation Kinematic EKF Experiment.
"""

import os
import argparse
import pandas as pd

from src.iovnbd.navigation.experiment_m9 import run_module9_experiment_suite

def main():
    parser = argparse.ArgumentParser(description="IO-VNBD Module 9 6D Full-Orientation EKF CLI")
    parser.add_argument("--output_dir", type=str, default="d:/prototype/results/module9", help="Output directory for results")

    args = parser.parse_args()

    print("=" * 85)
    print("IO-VNBD MODULE 9: 6D FULL-ORIENTATION KINEMATIC EKF EXPERIMENT & ABLATION SUITE")
    print("=" * 85)
    print(f"Output Directory: {args.output_dir}\n")

    results = run_module9_experiment_suite(output_dir=args.output_dir)

    print("--- CANONICAL BENCHMARK COMPARISON TABLE: M5.1 vs M8 vs M9 6D EKF ---")
    print(f"{'Outage':<9} | {'M5.1 RMSE (m)':<13} | {'M8 RMSE (m)':<12} | {'M9 RMSE (m)':<12} | {'vs M5.1':<10} | {'vs M8':<10}")
    print("-" * 85)

    for exp in results["experiments"]:
        print(f"{exp['outage_duration_sec']:<9.0f} | {exp['m5_1_ekf_rmse_m']:<13.2f} | {exp['m8_ekf_rmse_m']:<12.2f} | {exp['m9_ekf_rmse_m']:<12.2f} | {exp['status_vs_m5_1']:<10} | {exp['status_vs_m8']:<10}")

    print("-" * 85)
    print("\n--- ABLATION BREAKDOWN ON 120s OUTAGE ---")
    for ab in results["ablations"]:
        print(f"  {ab['name']:<42} -> RMSE: {ab['rmse_m']:6.2f} m | Final Error: {ab['final_m']:6.2f} m")

    print("-" * 85)
    print(f"\nResults written to: {os.path.join(args.output_dir, 'module9_results.json')}")
    print(f"Trajectory plot written to: {os.path.join(args.output_dir, 'm9_trajectory_comparison.png')}")
    print(f"Error growth plot written to: {os.path.join(args.output_dir, 'm9_position_error_growth.png')}")
    print("=" * 85)

if __name__ == "__main__":
    main()

"""
CLI runner for Module 3 Baseline Dead Reckoning Experiment.
"""

import os
import argparse
import pandas as pd

from src.iovnbd.navigation.experiment import run_outage_experiment_suite

def main():
    parser = argparse.ArgumentParser(description="IO-VNBD Module 3 Baseline Experiment CLI")
    parser.add_argument("--v_processed", type=str, default="d:/prototype/data/processed/S1/V-S1_processed.csv", help="Path to processed V CSV")
    parser.add_argument("--start_idx", type=int, default=1000, help="Row index where GNSS outage begins")
    parser.add_argument("--durations", type=float, nargs="+", default=[10.0, 30.0, 60.0, 120.0], help="Outage durations in seconds")
    parser.add_argument("--output_dir", type=str, default="d:/prototype/results/module3", help="Output directory for results and plots")

    args = parser.parse_args()

    print("=" * 60)
    print("IO-VNBD MODULE 3: BASELINE DEAD RECKONING EXPERIMENT")
    print("=" * 60)
    print(f"Input V CSV: {args.v_processed}")
    print(f"Outage Start Index: {args.start_idx}")
    print(f"Outage Durations: {args.durations} seconds")
    print(f"Output Directory: {args.output_dir}\n")

    df_v = pd.read_csv(args.v_processed)

    results = run_outage_experiment_suite(
        df_v=df_v,
        outage_durations=args.durations,
        start_idx=args.start_idx,
        output_dir=args.output_dir
    )

    print("--- MEASURED OUTAGE EXPERIMENT RESULTS ---")
    print(f"Initial State Source: {results['initial_state_source']}\n")
    print(f"{'Duration':<10} | {'Final Error (m)':<16} | {'Max Error (m)':<15} | {'RMSE Error (m)':<15} | {'Drift (m/s)':<12}")
    print("-" * 75)

    for exp in results["experiments"]:
        print(f"{exp['outage_duration_sec']:<10.0f} | {exp['final_position_error_m']:<16.2f} | {exp['max_position_error_m']:<15.2f} | {exp['rmse_position_error_m']:<15.2f} | {exp['drift_rate_m_per_sec']:<12.3f}")

    print("-" * 75)
    print(f"\nResults written to: {os.path.join(args.output_dir, 'baseline_results.json')}")
    print(f"Trajectory plot written to: {os.path.join(args.output_dir, 'baseline_trajectory_comparison.png')}")
    print(f"Error growth plot written to: {os.path.join(args.output_dir, 'baseline_error_growth.png')}")
    print("=" * 60)

if __name__ == "__main__":
    main()

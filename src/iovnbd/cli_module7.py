"""
CLI runner for Module 7 Confidence-Aware Intelligent Sensor Fusion Experiment.
"""

import os
import argparse
import pandas as pd

from src.iovnbd.fusion.experiment_m7 import run_module7_experiment_suite

def main():
    parser = argparse.ArgumentParser(description="IO-VNBD Module 7 Confidence-Aware Fusion CLI")
    parser.add_argument("--output_dir", type=str, default="d:/prototype/results/module7", help="Output directory for results")

    args = parser.parse_args()

    print("=" * 85)
    print("IO-VNBD MODULE 7: CONFIDENCE-AWARE INTELLIGENT SENSOR FUSION EXPERIMENT")
    print("=" * 85)
    print(f"Output Directory: {args.output_dir}\n")

    results = run_module7_experiment_suite(output_dir=args.output_dir)

    print("--- COMPARATIVE RESULTS TABLE: M5.1 BASELINE vs M6 NAÏVE AI vs M7 ADAPTIVE AI ---")
    print(f"{'Outage':<9} | {'M5.1 RMSE (m)':<13} | {'M6 Naïve RMSE (m)':<18} | {'M7 Adaptive RMSE (m)':<20} | {'% Gated':<8}")
    print("-" * 85)

    for exp in results["experiments"]:
        print(f"{exp['outage_duration_sec']:<9.0f} | {exp['m5_1_ekf_rmse_m']:<13.2f} | {exp['m6_naive_ai_rmse_m']:<18.2f} | {exp['m7_adaptive_ai_rmse_m']:<20.2f} | {exp['pct_time_gated']:<8.1f}%")

    print("-" * 85)
    print(f"\nResults written to: {os.path.join(args.output_dir, 'module7_results.json')}")
    print(f"Trajectory plot written to: {os.path.join(args.output_dir, 'm7_trajectory_comparison.png')}")
    print(f"Confidence plot written to: {os.path.join(args.output_dir, 'm7_confidence_adaptation.png')}")
    print("=" * 85)

if __name__ == "__main__":
    main()

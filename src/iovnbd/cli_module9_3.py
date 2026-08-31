"""
CLI runner for Module 9.3 Adaptive Fusion Switching & Final System Validation.
"""

import os
import argparse
import pandas as pd

from src.iovnbd.navigation.experiment_m9_3 import run_module9_3_experiment_suite

def main():
    parser = argparse.ArgumentParser(description="IO-VNBD Module 9.3 Adaptive Fusion Switching CLI")
    parser.add_argument("--output_dir", type=str, default="d:/prototype/results/module9", help="Output directory for results")

    args = parser.parse_args()

    print("=" * 85)
    print("IO-VNBD MODULE 9.3: ADAPTIVE FUSION SWITCHING & SYSTEM VALIDATION")
    print("=" * 85)
    print(f"Output Directory: {args.output_dir}\n")

    results = run_module9_3_experiment_suite(output_dir=args.output_dir)

    print("--- CANONICAL BENCHMARK COMPARISON TABLE (UNSEEN S1, INDEX 1000) ---")
    print(f"{'Outage':<8} | {'M5.1 (m)':<10} | {'M9.1 (m)':<10} | {'Fix Switch 30s (m)':<18} | {'Adapt Switch (m)':<16}")
    print("-" * 85)

    for exp in results["canonical_experiments"]:
        print(f"{exp['outage_duration_sec']:<8.0f} | {exp['m5_1_rmse_m']:<10.2f} | {exp['m9_1_rmse_m']:<10.2f} | {exp['switch_30s_rmse_m']:<18.2f} | {exp['adaptive_switch_rmse_m']:<16.2f}")

    print("-" * 85)
    print(f"\nAcceptance Classification: {results['acceptance_classification']}")
    print(f"\nResults written to: {os.path.join(args.output_dir, 'm9_3_fusion_results.json')}")
    print(f"Multi-Window CSV written to: {os.path.join(args.output_dir, 'm9_3_window_results.csv')}")
    print(f"Sensitivity CSV written to: {os.path.join(args.output_dir, 'm9_3_threshold_sensitivity.csv')}")
    print("=" * 85)

if __name__ == "__main__":
    main()

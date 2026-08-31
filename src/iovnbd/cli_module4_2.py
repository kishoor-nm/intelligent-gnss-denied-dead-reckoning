"""
CLI runner for Module 4.2 Canonical Pipeline & Results Integrity Audit.
"""

import os
import argparse
import pandas as pd

from src.iovnbd.navigation.canonical_m4_2 import run_canonical_module4_2_pipeline

def main():
    parser = argparse.ArgumentParser(description="IO-VNBD Module 4.2 Canonical Pipeline CLI")
    parser.add_argument("--v_processed", type=str, default="d:/prototype/data/processed/S1/V-S1_processed.csv", help="Path to processed V CSV")
    parser.add_argument("--start_idx", type=int, default=1000, help="Row index where GNSS outage begins")
    parser.add_argument("--durations", type=float, nargs="+", default=[10.0, 30.0, 60.0, 120.0], help="Outage durations in seconds")
    parser.add_argument("--output_dir", type=str, default="d:/prototype/results/module4_2", help="Output directory for canonical results")

    args = parser.parse_args()

    print("=" * 80)
    print("IO-VNBD MODULE 4.2: RESULTS INTEGRITY & CANONICAL EXPERIMENT LOCK")
    print("=" * 80)
    print(f"Input V CSV: {args.v_processed}")
    print(f"Outage Start Index: {args.start_idx}")
    print(f"Outage Durations: {args.durations} seconds")
    print(f"Output Directory: {args.output_dir}\n")

    res = run_canonical_module4_2_pipeline(
        v_processed_path=args.v_processed,
        start_idx=args.start_idx,
        outage_durations=args.durations,
        output_dir=args.output_dir
    )

    print("--- CANONICAL RESULTS TABLE 1: POSITION RMSE (METERS) ---")
    print(f"{'Outage':<10} | {'M3 RMSE (m)':<15} | {'4-Wheel RMSE (m)':<18} | {'Rear-Wheel RMSE (m)':<20}")
    print("-" * 75)
    for exp in res["experiments"]:
        print(f"{exp['outage_duration_sec']:<10.0f} | {exp['m3_baseline_rmse_m']:<15.2f} | {exp['m4_1_4wheel_rmse_m']:<18.2f} | {exp['m4_1_rearwheel_rmse_m']:<20.2f}")

    print("\n--- CANONICAL RESULTS TABLE 2: FINAL POSITION ERROR (METERS) ---")
    print(f"{'Outage':<10} | {'M3 Final Err (m)':<18} | {'4-Wheel Final Err (m)':<22} | {'Rear-Wheel Final Err (m)':<25}")
    print("-" * 80)
    for exp in res["experiments"]:
        print(f"{exp['outage_duration_sec']:<10.0f} | {exp['m3_baseline_final_err_m']:<18.2f} | {exp['m4_1_4wheel_final_err_m']:<22.2f} | {exp['m4_1_rearwheel_final_err_m']:<25.2f}")

    print("-" * 80)
    print(f"\nCanonical JSON written to: {os.path.join(args.output_dir, 'canonical_results.json')}")
    print(f"Canonical CSV written to: {os.path.join(args.output_dir, 'canonical_results.csv')}")
    print("=" * 80)

if __name__ == "__main__":
    main()

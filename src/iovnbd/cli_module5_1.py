"""
CLI runner for Module 5.1 Sensor Provenance & GNSS Outage Compliance Audit.
"""

import os
import argparse
import pandas as pd

from src.iovnbd.navigation.experiment_m5_1 import run_module5_1_audit_suite

def main():
    parser = argparse.ArgumentParser(description="IO-VNBD Module 5.1 Provenance Audit CLI")
    parser.add_argument("--v_processed", type=str, default="d:/prototype/data/processed/S1/V-S1_processed.csv", help="Path to processed V CSV")
    parser.add_argument("--start_idx", type=int, default=1000, help="Row index where GNSS outage begins")
    parser.add_argument("--durations", type=float, nargs="+", default=[10.0, 30.0, 60.0, 120.0], help="Outage durations in seconds")
    parser.add_argument("--output_dir", type=str, default="d:/prototype/results/module5_1", help="Output directory for audited results")

    args = parser.parse_args()

    print("=" * 80)
    print("IO-VNBD MODULE 5.1: SENSOR-PROVENANCE & GNSS-OUTAGE COMPLIANCE AUDIT")
    print("=" * 80)
    print(f"Input V CSV: {args.v_processed}")
    print(f"Outage Start Index: {args.start_idx}")
    print(f"Outage Durations: {args.durations} seconds")
    print(f"Output Directory: {args.output_dir}\n")

    df_v = pd.read_csv(args.v_processed)

    results = run_module5_1_audit_suite(
        df_v=df_v,
        outage_durations=args.durations,
        start_idx=args.start_idx,
        output_dir=args.output_dir
    )

    print("--- COMPARATIVE RESULTS TABLE: MODULE 3 BASELINE vs MODULE 5.1 CORRECTED EKF ---")
    print(f"{'Outage':<9} | {'M3 RMSE (m)':<13} | {'M5.1 EKF RMSE (m)':<18} | {'M3 Final (m)':<13} | {'M5.1 EKF Final (m)':<18}")
    print("-" * 80)

    for exp in results["experiments"]:
        print(f"{exp['outage_duration_sec']:<9.0f} | {exp['m3_baseline_rmse_m']:<13.2f} | {exp['m5_1_corrected_ekf_rmse_m']:<18.2f} | {exp['m3_baseline_final_err_m']:<13.2f} | {exp['m5_1_corrected_ekf_final_err_m']:<18.2f}")

    print("-" * 80)
    print(f"\nResults written to: {os.path.join(args.output_dir, 'm5_1_results.json')}")
    print(f"Trajectory plot written to: {os.path.join(args.output_dir, 'm5_1_trajectory_comparison.png')}")
    print("=" * 80)

if __name__ == "__main__":
    main()

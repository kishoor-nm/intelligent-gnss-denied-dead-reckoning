"""
CLI runner for Module 9.1 Speed-Adaptive Roll Compensation Experiment.
"""

import os
import argparse
import pandas as pd

from src.iovnbd.navigation.experiment_m9_1 import run_module9_1_experiment_suite

def main():
    parser = argparse.ArgumentParser(description="IO-VNBD Module 9.1 Speed-Adaptive Roll Compensation CLI")
    parser.add_argument("--output_dir", type=str, default="d:/prototype/results/module9", help="Output directory for results")

    args = parser.parse_args()

    print("=" * 85)
    print("IO-VNBD MODULE 9.1: SPEED-ADAPTIVE ROLL COMPENSATION EXPERIMENT")
    print("=" * 85)
    print(f"Output Directory: {args.output_dir}\n")

    results = run_module9_1_experiment_suite(output_dir=args.output_dir)

    print("--- VALIDATION GRID AUDIT (S2_VAL SPLIT) ---")
    val_b = results["validation_grid_audit"]["selected_best_config"]
    print(f"Locked Parameters: K_base = {val_b['k_base']:.2f}, V0 = {val_b['v0_m_s']:.1f} m/s (Val RMSE = {val_b['val_rmse_m']:.2f} m)\n")

    print("--- CANONICAL BENCHMARK COMPARISON TABLE ON UNSEEN SEQUENCE S1 ---")
    print(f"{'Outage':<9} | {'M5.1 RMSE (m)':<13} | {'M8 RMSE (m)':<12} | {'M9 K=0.10 (m)':<13} | {'M9.1 Adapt (m)':<14} | {'vs M5.1':<10}")
    print("-" * 85)

    for exp in results["experiments"]:
        print(f"{exp['outage_duration_sec']:<9.0f} | {exp['m5_1_ekf_rmse_m']:<13.2f} | {exp['m8_ekf_rmse_m']:<12.2f} | {exp['m9_k10_rmse_m']:<13.2f} | {exp['m9_1_adaptive_rmse_m']:<14.2f} | {exp['status_vs_m5_1']:<10}")

    print("-" * 85)
    print(f"\nResults written to: {os.path.join(args.output_dir, 'module9_1_results.json')}")
    print(f"Trajectory plot written to: {os.path.join(args.output_dir, 'm9_1_trajectory_comparison.png')}")
    print(f"Adaptive K trace plot written to: {os.path.join(args.output_dir, 'm9_1_k_adaptive_trace.png')}")
    print("=" * 85)

if __name__ == "__main__":
    main()

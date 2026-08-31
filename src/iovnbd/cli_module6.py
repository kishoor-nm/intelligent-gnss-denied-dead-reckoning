"""
CLI runner for Module 6 Intelligent AI/ML Motion Estimation Experiment.
"""

import os
import argparse
import pandas as pd

from src.iovnbd.navigation.experiment_m6 import run_module6_experiment_suite

def main():
    parser = argparse.ArgumentParser(description="IO-VNBD Module 6 AI/ML Motion Estimation CLI")
    parser.add_argument("--output_dir", type=str, default="d:/prototype/results/module6", help="Output directory for results")

    args = parser.parse_args()

    print("=" * 80)
    print("IO-VNBD MODULE 6: INTELLIGENT AI/ML MOTION ESTIMATION & ERROR COMPENSATION")
    print("=" * 80)
    print(f"Output Directory: {args.output_dir}\n")

    results = run_module6_experiment_suite(output_dir=args.output_dir)

    prog = results["model_progression"]
    print("--- MODEL PROGRESSION ON UNSEEN SEQUENCE S1 ---")
    print(f"Stage 1 (Constant Mean): MAE={prog['stage1_constant_mean']['mae']:.2f} m/s | RMSE={prog['stage1_constant_mean']['rmse']:.2f} m/s | R2={prog['stage1_constant_mean']['r2']:+.4f}")
    print(f"Stage 2 (OLS Linear)   : MAE={prog['stage2_ols_linear']['mae']:.2f} m/s | RMSE={prog['stage2_ols_linear']['rmse']:.2f} m/s | R2={prog['stage2_ols_linear']['r2']:+.4f}")
    print(f"Stage 3 (Ridge L2)     : MAE={prog['stage3_ridge_l2']['mae']:.2f} m/s | RMSE={prog['stage3_ridge_l2']['rmse']:.2f} m/s | R2={prog['stage3_ridge_l2']['r2']:+.4f}")

    print("\n--- COMPARATIVE NAVIGATION RESULTS TABLE: M3 vs M5.1 vs M6 AI-EKF ---")
    print(f"{'Outage':<9} | {'M3 RMSE (m)':<12} | {'M5.1 RMSE (m)':<13} | {'M6 AI-EKF RMSE (m)':<17} | {'M6 Final (m)':<12}")
    print("-" * 75)

    for exp in results["experiments"]:
        print(f"{exp['outage_duration_sec']:<9.0f} | {exp['m3_baseline_rmse_m']:<12.2f} | {exp['m5_1_ekf_rmse_m']:<13.2f} | {exp['m6_ai_ekf_rmse_m']:<17.2f} | {exp['m6_ai_ekf_final_m']:<12.2f}")

    print("-" * 75)
    print(f"\nResults written to: {os.path.join(args.output_dir, 'module6_results.json')}")
    print(f"Trajectory plot written to: {os.path.join(args.output_dir, 'm6_trajectory_comparison.png')}")
    print("=" * 80)

if __name__ == "__main__":
    main()

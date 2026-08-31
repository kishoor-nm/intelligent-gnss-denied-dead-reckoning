"""
CLI runner for Module 2 IO-VNBD dataset preprocessing and report generation.
"""

import os
import argparse
from src.iovnbd.preprocessing.pipeline import run_preprocessing_pipeline

def main():
    parser = argparse.ArgumentParser(description="IO-VNBD Module 2 Preprocessing CLI")
    parser.add_argument("--s_csv", type=str, default="d:/prototype/IO-VNBD-master/Synchronised V abd S datasets/Categorised IOVNB Dataset/S (Driver A)/S1/S-S1.csv", help="Path to raw S CSV")
    parser.add_argument("--v_csv", type=str, default="d:/prototype/IO-VNBD-master/Synchronised V abd S datasets/Categorised IOVNB Dataset/S (Driver A)/S1/V-S1.csv", help="Path to raw V CSV")
    parser.add_argument("--sequence", type=str, default="S1", help="Sequence name")
    parser.add_argument("--output_dir", type=str, default="d:/prototype/data/processed/S1", help="Output directory for processed CSVs")
    parser.add_argument("--wheel_radius", type=float, default=0.307, help="Configurable dynamic wheel radius in meters")

    args = parser.parse_args()

    print("=" * 60)
    print(f"IO-VNBD MODULE 2 PREPROCESSING RUN (Sequence: {args.sequence})")
    print("=" * 60)

    res = run_preprocessing_pipeline(
        s_csv_path=args.s_csv,
        v_csv_path=args.v_csv,
        output_dir=args.output_dir,
        sequence_name=args.sequence,
        wheel_radius_m=args.wheel_radius
    )

    print(f"Successfully processed sequence: {res.sequence_name}")
    print(f"Records processed: {res.record_count}")
    print(f"Duration: {res.duration_sec:.2f} seconds")
    print(f"Synchronization status: {res.sync_status}")
    print(f"Processed S CSV written to: {res.processed_s_path}")
    print(f"Processed V CSV written to: {res.processed_v_path}\n")

    stat = res.stationary_analysis
    print("--- STATIONARY BIAS & RESIDUAL ANALYSIS ---")
    print(f"Stationary windows found: {stat.stationary_windows_found}")
    print(f"Longest window: {stat.longest_window_samples} samples ({stat.longest_window_seconds:.1f}s)")
    print(f"Gyroscope Zero-Rate Bias (yaw, pitch, roll rad/s): {stat.gyro_bias_rad_s}")
    print(f"Gyroscope Noise Std Dev (yaw, pitch, roll rad/s): {stat.gyro_noise_std_rad_s}")
    print(f"Stationary Accel Mean (m/s²): {stat.accel_stat_mean_m_s2}")
    print(f"Stationary Gravity Stream Mean (m/s²): {stat.gravity_stat_mean_m_s2}")
    print(f"Stationary Acceleration Residual (m/s²): {stat.stationary_accel_residual_m_s2}")
    print(f"Explanation: {stat.residual_explanation}")
    print("=" * 60)

if __name__ == "__main__":
    main()

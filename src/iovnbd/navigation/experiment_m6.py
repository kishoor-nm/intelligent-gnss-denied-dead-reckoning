"""
Module 6: Experiment Runner for AI/ML Progression & Navigation Integration.
Trains model progression (Constant Mean -> OLS -> Ridge) on S2/S3a/S4, evaluates on held-out test data,
and compares M3 Baseline vs M5.1 EKF vs M6 AI-EKF across 10s, 30s, 60s, 120s outages.
"""

import os
import time
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from typing import List, Dict, Any

from src.iovnbd.intelligence.dataset import load_sequence_dataset, FEATURE_COLUMNS
from src.iovnbd.intelligence.model import ConstantMeanRegressor, OLSLinearRegressor, RidgeLinearRegressor, evaluate_predictions
from src.iovnbd.navigation.initialization import initialize_navigation_state
from src.iovnbd.navigation.baseline import propagate_dead_reckoning_baseline, TrajectoryResult
from src.iovnbd.navigation.ekf_m5_1 import propagate_ekf_m5_1
from src.iovnbd.navigation.ekf_m6 import propagate_ekf_m6
from src.iovnbd.navigation.metrics import calculate_outage_error_metrics, OutageMetricsResult

def run_module6_experiment_suite(
    output_dir: str = "d:/prototype/results/module6",
    outage_durations: List[float] = [10.0, 30.0, 60.0, 120.0],
    start_idx: int = 1000
) -> Dict[str, Any]:
    """
    Executes full Module 6 ML training, progression evaluation, and navigation integration suite.
    """
    os.makedirs(output_dir, exist_ok=True)
    t_start_wall = time.time()

    # Load Train Sequences (S2, S3a, S4)
    train_seqs = [
        ("d:/prototype/IO-VNBD-master/Synchronised V abd S datasets/Categorised IOVNB Dataset/S (Driver A)/S2/V-S2.csv",
         "d:/prototype/IO-VNBD-master/Synchronised V abd S datasets/Categorised IOVNB Dataset/S (Driver A)/S2/S-S2.csv"),
        ("d:/prototype/IO-VNBD-master/Synchronised V abd S datasets/Categorised IOVNB Dataset/S (Driver A)/S3a/V-S3a.csv",
         "d:/prototype/IO-VNBD-master/Synchronised V abd S datasets/Categorised IOVNB Dataset/S (Driver A)/S3a/S-S3a.csv"),
        ("d:/prototype/IO-VNBD-master/Synchronised V abd S datasets/Categorised IOVNB Dataset/S (Driver A)/S4/V-S4.csv",
         "d:/prototype/IO-VNBD-master/Synchronised V abd S datasets/Categorised IOVNB Dataset/S (Driver A)/S4/S-S4.csv")
    ]

    X_train_list, y_train_list = [], []
    for v_p, s_p in train_seqs:
        if os.path.exists(v_p) and os.path.exists(s_p):
            f, t = load_sequence_dataset(v_p, s_p)
            X_train_list.append(f[FEATURE_COLUMNS].values)
            y_train_list.append(t.values)

    X_train = np.vstack(X_train_list)
    y_train = np.concatenate(y_train_list)

    # Load Test Sequence (S1 Processed)
    f_test, y_test = load_sequence_dataset("d:/prototype/data/processed/S1/V-S1_processed.csv", "d:/prototype/data/processed/S1/S-S1_processed.csv")
    X_test = f_test[FEATURE_COLUMNS].values
    y_test_arr = y_test.values

    # Train Model Progression
    # Stage 1: Constant Mean
    m_mean = ConstantMeanRegressor()
    m_mean.fit(X_train, y_train)
    p_mean = m_mean.predict(X_test)
    met_mean = evaluate_predictions(y_test_arr, p_mean)

    # Stage 2: OLS Linear
    m_ols = OLSLinearRegressor()
    m_ols.fit(X_train, y_train)
    p_ols = m_ols.predict(X_test)
    met_ols = evaluate_predictions(y_test_arr, p_ols)

    # Stage 3: Ridge Regressor
    m_ridge = RidgeLinearRegressor(alpha=10.0)
    m_ridge.fit(X_train, y_train)
    p_ridge = m_ridge.predict(X_test)
    met_ridge = evaluate_predictions(y_test_arr, p_ridge)

    progression_results = {
        "stage1_constant_mean": met_mean.__dict__,
        "stage2_ols_linear": met_ols.__dict__,
        "stage3_ridge_l2": met_ridge.__dict__
    }

    # Execute Navigation Integration Experiments on S1
    df_v = pd.read_csv("d:/prototype/data/processed/S1/V-S1_processed.csv")
    df_s = pd.read_csv("d:/prototype/data/processed/S1/S-S1_processed.csv")

    init_state = initialize_navigation_state(df_v, start_idx=start_idx)

    suite_results = {
        "sequence": "S1",
        "start_idx": start_idx,
        "dataset_split": "Train: S2, S3a, S4 | Test: S1 (Sequence-level generalization)",
        "model_progression": progression_results,
        "experiments": []
    }

    for dur in outage_durations:
        t0_exp = time.time()

        # M3 Baseline
        res_m3 = propagate_dead_reckoning_baseline(df_v, init_state, start_idx, dur)
        met_m3 = calculate_outage_error_metrics(res_m3, df_v)

        # M5.1 Corrected EKF
        res_m5_1 = propagate_ekf_m5_1(df_v, init_state, start_idx, dur)
        met_m5_1 = calculate_outage_error_metrics(TrajectoryResult(points=res_m5_1.points, dataframe=res_m5_1.dataframe, outage_start_t=res_m5_1.outage_start_t, outage_end_t=res_m5_1.outage_end_t, outage_duration_sec=dur), df_v)

        # M6 AI-Enhanced 5D EKF (using Ridge model)
        res_m6 = propagate_ekf_m6(df_v, df_s, init_state, start_idx, dur, ml_model=m_ridge, feature_cols=FEATURE_COLUMNS)
        met_m6 = calculate_outage_error_metrics(TrajectoryResult(points=res_m6.points, dataframe=res_m6.dataframe, outage_start_t=res_m6.outage_start_t, outage_end_t=res_m6.outage_end_t, outage_duration_sec=dur), df_v)

        t_elapsed = time.time() - t0_exp

        diff_rmse = met_m6.rmse_position_error_m - met_m5_1.rmse_position_error_m

        exp_data = {
            "outage_duration_sec": dur,
            "sample_count": met_m3.sample_count,
            "m3_baseline_rmse_m": round(met_m3.rmse_position_error_m, 2),
            "m5_1_ekf_rmse_m": round(met_m5_1.rmse_position_error_m, 2),
            "m6_ai_ekf_rmse_m": round(met_m6.rmse_position_error_m, 2),
            "m3_baseline_final_m": round(met_m3.final_position_error_m, 2),
            "m5_1_ekf_final_m": round(met_m5_1.final_position_error_m, 2),
            "m6_ai_ekf_final_m": round(met_m6.final_position_error_m, 2),
            "m6_vs_m5_1_diff_rmse_m": round(diff_rmse, 2),
            "runtime_ms": round(t_elapsed * 1000.0, 2)
        }
        suite_results["experiments"].append(exp_data)

    # Plot Trajectory Comparison (60s Outage)
    res_m3_60 = propagate_dead_reckoning_baseline(df_v, init_state, start_idx, 60.0)
    res_m5_1_60 = propagate_ekf_m5_1(df_v, init_state, start_idx, 60.0)
    res_m6_60 = propagate_ekf_m6(df_v, df_s, init_state, start_idx, 60.0, ml_model=m_ridge, feature_cols=FEATURE_COLUMNS)

    plt.figure(figsize=(10, 6))
    plt.plot(res_m3_60.dataframe["east_m"], res_m3_60.dataframe["north_m"], color='#1f77b4', linewidth=2.0, label='M3 Baseline DR')
    plt.plot(res_m5_1_60.dataframe["east_m"], res_m5_1_60.dataframe["north_m"], color='#ff7f0e', linestyle='--', linewidth=2.0, label='M5.1 EKF (ECU Speed)')
    plt.plot(res_m6_60.dataframe["east_m"], res_m6_60.dataframe["north_m"], color='#2ca02c', linestyle=':', linewidth=2.5, label='M6 AI-Enhanced EKF (Smartphone IMU Speed)')

    plt.scatter(0, 0, color='blue', s=80, label='GNSS Outage Start', zorder=5)
    plt.title("GNSS Outage Trajectory: M3 vs M5.1 vs M6 AI-EKF (60s Outage)")
    plt.xlabel("East (meters)")
    plt.ylabel("North (meters)")
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend()
    plt.tight_layout()

    plt.savefig(os.path.join(output_dir, "m6_trajectory_comparison.png"), dpi=150)
    plt.close()

    total_wall_time = time.time() - t_start_wall
    suite_results["total_runtime_seconds"] = round(total_wall_time, 3)

    json_path = os.path.join(output_dir, "module6_results.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(suite_results, f, indent=2)

    return suite_results

"""
Automated unit tests for Module 7 Confidence-Aware Sensor Fusion Pipeline.
"""

import os
import unittest
import pandas as pd
import numpy as np

from src.iovnbd.intelligence.dataset import extract_features_robust, FEATURE_COLUMNS
from src.iovnbd.intelligence.model import RidgeLinearRegressor
from src.iovnbd.navigation.initialization import initialize_navigation_state
from src.iovnbd.fusion.ekf_m7 import propagate_ekf_m7_confidence
from src.iovnbd.fusion.experiment_m7 import run_module7_experiment_suite

class TestModule7ConfidenceFusion(unittest.TestCase):

    def setUp(self):
        self.v_proc_path = "d:/prototype/data/processed/S1/V-S1_processed.csv"
        self.s_proc_path = "d:/prototype/data/processed/S1/S-S1_processed.csv"
        self.df_v = pd.read_csv(self.v_proc_path)
        self.df_s = pd.read_csv(self.s_proc_path)
        self.output_dir = "d:/prototype/results/module7_test"

    def test_m7_confidence_ekf_propagation(self):
        init = initialize_navigation_state(self.df_v, start_idx=1000)
        m_ridge = RidgeLinearRegressor(alpha=10.0)
        m_ridge.fit(np.random.randn(100, 4), np.random.randn(100) * 5.0 + 10.0)

        res = propagate_ekf_m7_confidence(self.df_v, self.df_s, init, start_idx=1000, outage_duration_sec=30.0, ml_model=m_ridge, feature_cols=FEATURE_COLUMNS, nis_gate_threshold=3.0)

        self.assertEqual(len(res.points), 301)
        df_traj = res.dataframe

        self.assertFalse(df_traj["east_m"].isna().any())
        self.assertFalse(df_traj["nis_score"].isna().any())
        self.assertFalse(df_traj["r_adaptive"].isna().any())

    def test_m7_experiment_suite_execution(self):
        res = run_module7_experiment_suite(output_dir=self.output_dir, outage_durations=[10.0, 30.0], start_idx=1000)

        self.assertEqual(len(res["experiments"]), 2)
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "module7_results.json")))
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "m7_trajectory_comparison.png")))
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "m7_confidence_adaptation.png")))

    def test_strict_no_gnss_leakage_in_m7_inference(self):
        init = initialize_navigation_state(self.df_v, start_idx=1000)
        m_ridge = RidgeLinearRegressor(alpha=10.0)
        m_ridge.fit(np.random.randn(100, 4), np.random.randn(100) * 5.0 + 10.0)

        res = propagate_ekf_m7_confidence(self.df_v, self.df_s, init, start_idx=1000, outage_duration_sec=30.0, ml_model=m_ridge, feature_cols=FEATURE_COLUMNS)
        df_traj = res.dataframe

        self.assertNotIn("Latitude (degrees)", df_traj.columns)
        self.assertNotIn("Longitude (degrees)", df_traj.columns)
        self.assertNotIn("Velocity (km/hr)", df_traj.columns)

if __name__ == "__main__":
    unittest.main()

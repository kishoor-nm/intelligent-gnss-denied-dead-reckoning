"""
Automated unit tests for Module 6 AI/ML Motion Estimation Pipeline.
"""

import os
import unittest
import pandas as pd
import numpy as np

from src.iovnbd.intelligence.dataset import extract_features_robust, load_sequence_dataset, FEATURE_COLUMNS
from src.iovnbd.intelligence.model import ConstantMeanRegressor, OLSLinearRegressor, RidgeLinearRegressor, evaluate_predictions
from src.iovnbd.navigation.initialization import initialize_navigation_state
from src.iovnbd.navigation.ekf_m6 import propagate_ekf_m6
from src.iovnbd.navigation.experiment_m6 import run_module6_experiment_suite

class TestModule6Intelligence(unittest.TestCase):

    def setUp(self):
        self.v_proc_path = "d:/prototype/data/processed/S1/V-S1_processed.csv"
        self.s_proc_path = "d:/prototype/data/processed/S1/S-S1_processed.csv"
        self.df_v = pd.read_csv(self.v_proc_path)
        self.df_s = pd.read_csv(self.s_proc_path)
        self.output_dir = "d:/prototype/results/module6_test"

    def test_causal_smartphone_features(self):
        df_feat = extract_features_robust(self.df_s)

        self.assertEqual(len(df_feat), len(self.df_s))
        for col in FEATURE_COLUMNS:
            self.assertIn(col, df_feat.columns)
            self.assertFalse(df_feat[col].isna().any())

    def test_model_progression_training(self):
        X = np.random.randn(100, 4)
        y = np.random.randn(100) * 5.0 + 10.0

        m_mean = ConstantMeanRegressor()
        m_mean.fit(X, y)
        p_mean = m_mean.predict(X[:10])
        self.assertEqual(len(p_mean), 10)

        m_ols = OLSLinearRegressor()
        m_ols.fit(X, y)
        p_ols = m_ols.predict(X[:10])
        self.assertEqual(len(p_ols), 10)

        m_ridge = RidgeLinearRegressor(alpha=10.0)
        m_ridge.fit(X, y)
        p_ridge = m_ridge.predict(X[:10])
        self.assertEqual(len(p_ridge), 10)

    def test_ekf_m6_propagation(self):
        init = initialize_navigation_state(self.df_v, start_idx=1000)
        m_ridge = RidgeLinearRegressor(alpha=10.0)
        m_ridge.fit(np.random.randn(100, 4), np.random.randn(100) * 5.0 + 10.0)

        res = propagate_ekf_m6(self.df_v, self.df_s, init, start_idx=1000, outage_duration_sec=30.0, ml_model=m_ridge, feature_cols=FEATURE_COLUMNS)

        self.assertEqual(len(res.points), 301)
        df_traj = res.dataframe

        self.assertFalse(df_traj["east_m"].isna().any())
        self.assertFalse(df_traj["velocity_m_s"].isna().any())
        self.assertFalse(df_traj["ml_speed_est_m_s"].isna().any())

    def test_strict_no_gnss_leakage_in_m6_inference(self):
        init = initialize_navigation_state(self.df_v, start_idx=1000)
        m_ridge = RidgeLinearRegressor(alpha=10.0)
        m_ridge.fit(np.random.randn(100, 4), np.random.randn(100) * 5.0 + 10.0)

        res = propagate_ekf_m6(self.df_v, self.df_s, init, start_idx=1000, outage_duration_sec=30.0, ml_model=m_ridge, feature_cols=FEATURE_COLUMNS)
        df_traj = res.dataframe

        self.assertNotIn("Latitude (degrees)", df_traj.columns)
        self.assertNotIn("Longitude (degrees)", df_traj.columns)
        self.assertNotIn("Velocity (km/hr)", df_traj.columns)

if __name__ == "__main__":
    unittest.main()

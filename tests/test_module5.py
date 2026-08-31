"""
Automated unit tests for Module 5 5D Extended Kalman Filter (EKF) Core.
"""

import os
import unittest
import pandas as pd
import numpy as np

from src.iovnbd.navigation.initialization import initialize_navigation_state
from src.iovnbd.navigation.ekf_m5 import propagate_ekf_dead_reckoning
from src.iovnbd.navigation.experiment_m5 import run_module5_experiment_suite

class TestModule5EKFCore(unittest.TestCase):

    def setUp(self):
        self.v_proc_path = "d:/prototype/data/processed/S1/V-S1_processed.csv"
        self.df_v = pd.read_csv(self.v_proc_path)
        self.output_dir = "d:/prototype/results/module5_test"

    def test_ekf_propagation(self):
        init = initialize_navigation_state(self.df_v, start_idx=1000)
        res = propagate_ekf_dead_reckoning(self.df_v, init, start_idx=1000, outage_duration_sec=30.0)

        self.assertEqual(len(res.points), 301)
        df_traj = res.dataframe

        # Ensure no NaNs or Infs exist in EKF state or standard deviation outputs
        self.assertFalse(df_traj["east_m"].isna().any())
        self.assertFalse(df_traj["north_m"].isna().any())
        self.assertFalse(df_traj["velocity_m_s"].isna().any())
        self.assertFalse(df_traj["heading_rad"].isna().any())
        self.assertFalse(df_traj["std_east_m"].isna().any())

    def test_ekf_experiment_suite(self):
        res = run_module5_experiment_suite(self.df_v, outage_durations=[10.0, 30.0], start_idx=1000, output_dir=self.output_dir)

        self.assertEqual(len(res["experiments"]), 2)
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "module5_results.json")))
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "m5_trajectory_comparison.png")))
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "m5_error_growth.png")))

    def test_no_data_leakage(self):
        init = initialize_navigation_state(self.df_v, start_idx=1000)
        res = propagate_ekf_dead_reckoning(self.df_v, init, start_idx=1000, outage_duration_sec=30.0)
        df_traj = res.dataframe

        self.assertNotIn("Latitude (degrees)", df_traj.columns)
        self.assertNotIn("Longitude (degrees)", df_traj.columns)

if __name__ == "__main__":
    unittest.main()

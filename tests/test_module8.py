"""
Automated unit tests for Module 8 5D NHC-Enhanced EKF Pipeline.
"""

import os
import unittest
import pandas as pd
import numpy as np

from src.iovnbd.navigation.initialization import initialize_navigation_state
from src.iovnbd.navigation.ekf_m8 import propagate_ekf_m8, extract_outage_inputs_m8
from src.iovnbd.navigation.experiment_m8 import run_module8_experiment_suite

class TestModule8KinematicEKF(unittest.TestCase):

    def setUp(self):
        self.v_proc_path = "d:/prototype/data/processed/S1/V-S1_processed.csv"
        self.df_v = pd.read_csv(self.v_proc_path)
        self.output_dir = "d:/prototype/results/module8_test"

    def test_extract_outage_inputs_m8_zero_leakage(self):
        inputs = extract_outage_inputs_m8(self.df_v, t0=100.0, outage_duration_sec=30.0)

        self.assertEqual(len(inputs), 301)
        for inp in inputs:
            self.assertFalse(hasattr(inp, "latitude"))
            self.assertFalse(hasattr(inp, "longitude"))
            self.assertFalse(hasattr(inp, "velocity_m_s"))

    def test_ekf_m8_propagation_and_nhc_jacobian(self):
        init = initialize_navigation_state(self.df_v, start_idx=1000)
        res = propagate_ekf_m8(self.df_v, init, start_idx=1000, outage_duration_sec=30.0, enable_wheel_speed=True, enable_nhc=True)

        self.assertEqual(len(res.points), 301)
        df_traj = res.dataframe

        self.assertFalse(df_traj["east_m"].isna().any())
        self.assertFalse(df_traj["north_m"].isna().any())
        self.assertFalse(df_traj["nhc_residual_m_s2"].isna().any())
        self.assertGreater(res.nhc_accepted_count, 0)

    def test_m8_experiment_suite_execution(self):
        res = run_module8_experiment_suite(output_dir=self.output_dir, outage_durations=[10.0, 30.0], start_idx=1000)

        self.assertEqual(len(res["experiments"]), 2)
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "module8_results.json")))
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "m8_trajectory_comparison.png")))

if __name__ == "__main__":
    unittest.main()

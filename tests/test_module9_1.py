"""
Automated unit tests for Module 9.1 Speed-Adaptive Roll Compensation Pipeline.
"""

import os
import unittest
import pandas as pd
import numpy as np

from src.iovnbd.navigation.initialization import initialize_navigation_state
from src.iovnbd.navigation.ekf_m9 import compute_speed_adaptive_k_roll, propagate_ekf_m9_1
from src.iovnbd.navigation.experiment_m9_1 import run_module9_1_experiment_suite

class TestModule9_1AdaptiveRollEKF(unittest.TestCase):

    def setUp(self):
        self.v_proc_path = "d:/prototype/data/processed/S1/V-S1_processed.csv"
        self.s_proc_path = "d:/prototype/data/processed/S1/S-S1_processed.csv"
        self.df_v = pd.read_csv(self.v_proc_path)
        self.df_s = pd.read_csv(self.s_proc_path)
        self.output_dir = "d:/prototype/results/module9_1_test"

    def test_k_roll_mathematical_correctness_and_monotonicity(self):
        """Verifies K(V) = K_base * (1 - exp(-V / V0)) monotonicity and bounds."""
        k_base = 0.05
        v0 = 5.0

        # At V = 0: K = 0
        k_zero = compute_speed_adaptive_k_roll(0.0, k_base=k_base, v0_m_s=v0)
        self.assertAlmostEqual(k_zero, 0.0)

        # Monotonicity test across increasing speeds
        speeds = np.linspace(0.0, 30.0, 50)
        k_vals = [compute_speed_adaptive_k_roll(v, k_base=k_base, v0_m_s=v0) for v in speeds]

        for i in range(len(k_vals) - 1):
            self.assertGreaterEqual(k_vals[i+1], k_vals[i])

        # Boundedness test in [0, K_base]
        for k in k_vals:
            self.assertGreaterEqual(k, 0.0)
            self.assertLessEqual(k, k_base + 1e-6)

    def test_no_gnss_leakage_in_m9_1_inference(self):
        """Verifies zero GNSS leakage into estimator loop."""
        df_v_fake = self.df_v.copy()
        df_v_fake["Latitude (degrees)"] = 0.0
        df_v_fake["Longitude (degrees)"] = 0.0
        df_v_fake["velocity_m_s"] = 999.0

        init = initialize_navigation_state(self.df_v, start_idx=1000)
        res_orig = propagate_ekf_m9_1(self.df_v, self.df_s, init, start_idx=1000, outage_duration_sec=30.0, k_mode="adaptive", k_base=0.05, v0_m_s=5.0)
        res_fake = propagate_ekf_m9_1(df_v_fake, self.df_s, init, start_idx=1000, outage_duration_sec=30.0, k_mode="adaptive", k_base=0.05, v0_m_s=5.0)

        np.testing.assert_allclose(res_orig.dataframe["east_m"].values, res_fake.dataframe["east_m"].values)
        np.testing.assert_allclose(res_orig.dataframe["north_m"].values, res_fake.dataframe["north_m"].values)

    def test_m9_1_diagnostics_trace(self):
        init = initialize_navigation_state(self.df_v, start_idx=1000)
        res = propagate_ekf_m9_1(self.df_v, self.df_s, init, start_idx=1000, outage_duration_sec=30.0, k_mode="adaptive", k_base=0.05, v0_m_s=5.0)

        self.assertEqual(len(res.points), 301)
        df_traj = res.dataframe
        self.assertIn("k_roll_adaptive", df_traj.columns)
        self.assertFalse(df_traj["k_roll_adaptive"].isna().any())

    def test_m9_1_experiment_suite_execution(self):
        res = run_module9_1_experiment_suite(output_dir=self.output_dir, outage_durations=[10.0, 30.0], start_idx=1000)

        self.assertEqual(len(res["experiments"]), 2)
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "module9_1_results.json")))
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "m9_1_trajectory_comparison.png")))
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "m9_1_k_adaptive_trace.png")))

if __name__ == "__main__":
    unittest.main()

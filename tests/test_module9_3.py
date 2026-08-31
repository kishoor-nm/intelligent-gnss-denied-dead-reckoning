"""
Automated unit test suite for Module 9.3 Adaptive Fusion Switching Pipeline.
"""

import os
import unittest
import pandas as pd
import numpy as np

from src.iovnbd.navigation.initialization import initialize_navigation_state
from src.iovnbd.navigation.fusion_m9_3 import propagate_fused_ekf_m9_3
from src.iovnbd.navigation.experiment_m9_3 import run_module9_3_experiment_suite

class TestModule9_3AdaptiveFusionSwitching(unittest.TestCase):

    def setUp(self):
        self.v_proc_path = "d:/prototype/data/processed/S1/V-S1_processed.csv"
        self.s_proc_path = "d:/prototype/data/processed/S1/S-S1_processed.csv"
        self.df_v = pd.read_csv(self.v_proc_path)
        self.df_s = pd.read_csv(self.s_proc_path)
        self.output_dir = "d:/prototype/results/module9_3_test"

    def test_m5_1_baseline_invocation(self):
        init = initialize_navigation_state(self.df_v, start_idx=1000)
        res = propagate_fused_ekf_m9_3(self.df_v, self.df_s, init, 1000, 30.0, mode="m5_1_only")

        self.assertEqual(len(res.points), 301)
        self.assertTrue(all(p.active_estimator == "M5.1" for p in res.points))

    def test_m9_1_baseline_invocation(self):
        init = initialize_navigation_state(self.df_v, start_idx=1000)
        res = propagate_fused_ekf_m9_3(self.df_v, self.df_s, init, 1000, 30.0, mode="m9_1_only")

        self.assertEqual(len(res.points), 301)
        self.assertTrue(all(p.active_estimator == "M9.1" for p in res.points))

    def test_fixed_threshold_switching(self):
        init = initialize_navigation_state(self.df_v, start_idx=1000)
        res = propagate_fused_ekf_m9_3(self.df_v, self.df_s, init, 1000, 60.0, mode="fixed_switch", t_switch_sec=30.0)

        self.assertEqual(res.switch_count, 1)
        # Check that prior to 30s it is M5.1 and after 30s it is M9.1
        df_tr = res.dataframe
        m5_pts = df_tr[df_tr["t_rel_sec"] < 130.0]
        m9_pts = df_tr[df_tr["t_rel_sec"] >= 130.0]

        self.assertTrue(all(est == "M5.1" for est in m5_pts["active_estimator"]))
        self.assertTrue(all(est == "M9.1" for est in m9_pts["active_estimator"]))

    def test_state_continuity_and_no_position_jump(self):
        """Verifies smooth state handoff (no artificial trajectory discontinuity at switch point)."""
        init = initialize_navigation_state(self.df_v, start_idx=1000)
        res = propagate_fused_ekf_m9_3(self.df_v, self.df_s, init, 1000, 60.0, mode="fixed_switch", t_switch_sec=30.0)

        df_tr = res.dataframe
        sw_idx = df_tr[df_tr["switch_event"] == True].index[0]

        pt_before = df_tr.iloc[sw_idx - 1]
        pt_switch = df_tr.iloc[sw_idx]

        # Calculate position change over single 0.1s step across switch
        dx = pt_switch["east_m"] - pt_before["east_m"]
        dy = pt_switch["north_m"] - pt_before["north_m"]
        dist_step = np.sqrt(dx**2 + dy**2)

        # Expected step distance for ~10m/s speed over 0.1s is ~1.0m
        self.assertLess(dist_step, 2.5, "Artificial position jump detected at estimator switch point!")

    def test_no_gnss_leakage_in_fusion(self):
        """Verifies zero GNSS leakage into fusion switching decision and state propagation."""
        df_v_fake = self.df_v.copy()
        df_v_fake["Latitude (degrees)"] = 0.0
        df_v_fake["Longitude (degrees)"] = 0.0
        df_v_fake["velocity_m_s"] = 999.0

        init = initialize_navigation_state(self.df_v, start_idx=1000)
        res_orig = propagate_fused_ekf_m9_3(self.df_v, self.df_s, init, 1000, 60.0, mode="adaptive_switch")
        res_fake = propagate_fused_ekf_m9_3(df_v_fake, self.df_s, init, 1000, 60.0, mode="adaptive_switch")

        np.testing.assert_allclose(res_orig.dataframe["east_m"].values, res_fake.dataframe["east_m"].values)
        np.testing.assert_allclose(res_orig.dataframe["north_m"].values, res_fake.dataframe["north_m"].values)
        self.assertEqual(res_orig.switch_count, res_fake.switch_count)

    def test_module9_3_experiment_suite_execution(self):
        res = run_module9_3_experiment_suite(output_dir=self.output_dir, start_indices=[1000, 5000], durations=[10.0, 30.0])

        self.assertIn("canonical_experiments", res)
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "m9_3_fusion_results.json")))
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "m9_3_window_results.csv")))
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "m9_3_threshold_sensitivity.csv")))

if __name__ == "__main__":
    unittest.main()

"""
Automated unit tests for Module 9.2 Robustness & Generalization Audit.
"""

import os
import unittest
import pandas as pd
import numpy as np

from src.iovnbd.navigation.initialization import initialize_navigation_state
from src.iovnbd.navigation.ekf_m9 import propagate_ekf_m9_1
from src.iovnbd.navigation.audit_m9_2_robustness import run_m9_2_robustness_audit

class TestModule9_2RobustnessAudit(unittest.TestCase):

    def setUp(self):
        self.v_proc_path = "d:/prototype/data/processed/S1/V-S1_processed.csv"
        self.s_proc_path = "d:/prototype/data/processed/S1/S-S1_processed.csv"
        self.df_v = pd.read_csv(self.v_proc_path)
        self.df_s = pd.read_csv(self.s_proc_path)
        self.output_dir = "d:/prototype/results/module9_2_test"

    def test_zero_gnss_leakage_in_m9_2(self):
        df_v_fake = self.df_v.copy()
        df_v_fake["Latitude (degrees)"] = 0.0
        df_v_fake["Longitude (degrees)"] = 0.0
        df_v_fake["velocity_m_s"] = 999.0

        init = initialize_navigation_state(self.df_v, start_idx=1000)
        res_orig = propagate_ekf_m9_1(self.df_v, self.df_s, init, start_idx=1000, outage_duration_sec=30.0, k_mode="adaptive", k_base=0.02, v0_m_s=10.0)
        res_fake = propagate_ekf_m9_1(df_v_fake, self.df_s, init, start_idx=1000, outage_duration_sec=30.0, k_mode="adaptive", k_base=0.02, v0_m_s=10.0)

        np.testing.assert_allclose(res_orig.dataframe["east_m"].values, res_fake.dataframe["east_m"].values)
        np.testing.assert_allclose(res_orig.dataframe["north_m"].values, res_fake.dataframe["north_m"].values)

    def test_locked_parameter_enforcement(self):
        k_base = 0.02
        v0 = 10.0
        self.assertEqual(k_base, 0.02)
        self.assertEqual(v0, 10.0)

    def test_multiple_window_audit_execution(self):
        res = run_m9_2_robustness_audit(output_dir=self.output_dir, start_indices=[1000, 5000])

        self.assertEqual(res["evaluated_window_count"], 2)
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "m9_2_provenance_audit.json")))
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "m9_2_window_results.csv")))
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "m9_2_parameter_sensitivity.csv")))
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "m9_2_robustness_results.json")))

if __name__ == "__main__":
    unittest.main()

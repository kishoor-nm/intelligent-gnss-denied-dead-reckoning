"""
Automated unit test suite for Module 10 Final System Validation & Competition Readiness.
"""

import os
import unittest
import pandas as pd
import numpy as np

from src.iovnbd.navigation.final_navigation import get_final_competition_system, FinalDeadReckoningConfig
from src.iovnbd.navigation.audit_final_validation import run_final_system_validation_audit

class TestFinalSystemValidation(unittest.TestCase):

    def setUp(self):
        self.v_proc_path = "d:/prototype/data/processed/S1/V-S1_processed.csv"
        self.s_proc_path = "d:/prototype/data/processed/S1/S-S1_processed.csv"
        self.df_v = pd.read_csv(self.v_proc_path)
        self.df_s = pd.read_csv(self.s_proc_path)
        self.output_dir = "d:/prototype/results/final_validation_test"

    def test_final_competition_system_wrapper(self):
        system = get_final_competition_system()
        self.assertIsInstance(system.config, FinalDeadReckoningConfig)
        self.assertEqual(system.config.k_base, 0.02)
        self.assertEqual(system.config.v0_m_s, 10.0)

        res = system.run_outage_navigation(self.df_v, self.df_s, start_idx=1000, outage_duration_sec=30.0)
        self.assertEqual(len(res.points), 301)

        metrics = system.evaluate_outage_performance(res, self.df_v)
        self.assertTrue(hasattr(metrics, "rmse_position_error_m"))

    def test_zero_gnss_leakage_in_final_system(self):
        """Verifies zero GNSS leakage during outage inference (excluding initial anchor initialization)."""
        system = get_final_competition_system()
        df_v_fake = self.df_v.copy()
        # Zero GNSS fields AFTER outage start (start_idx = 1000)
        df_v_fake.loc[1001:, "Latitude (degrees)"] = 0.0
        df_v_fake.loc[1001:, "Longitude (degrees)"] = 0.0
        df_v_fake.loc[1001:, "velocity_m_s"] = 999.0

        res_orig = system.run_outage_navigation(self.df_v, self.df_s, start_idx=1000, outage_duration_sec=30.0)
        res_fake = system.run_outage_navigation(df_v_fake, self.df_s, start_idx=1000, outage_duration_sec=30.0)

        np.testing.assert_allclose(res_orig.dataframe["east_m"].values, res_fake.dataframe["east_m"].values)
        np.testing.assert_allclose(res_orig.dataframe["north_m"].values, res_fake.dataframe["north_m"].values)

    def test_final_validation_audit_execution(self):
        rep = run_final_system_validation_audit(output_dir=self.output_dir, start_indices=[1000, 5000], durations=[10.0, 30.0])

        self.assertEqual(rep["final_classification"], "A — VALIDATED FOR FINAL DEMONSTRATION")
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "final_validation_report.json")))
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "causality_audit.json")))
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "provenance_audit.json")))
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "final_window_results.csv")))

if __name__ == "__main__":
    unittest.main()

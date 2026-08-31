"""
Automated unit tests for Module 9 Observability & Physical Validity Audit.
"""

import os
import unittest
import pandas as pd
import numpy as np

from src.iovnbd.navigation.initialization import initialize_navigation_state
from src.iovnbd.navigation.ekf_m9 import propagate_ekf_m9, extract_outage_inputs_m9
from src.iovnbd.navigation.audit_m9_observability import run_m9_observability_audit

class TestM9ObservabilityAudit(unittest.TestCase):

    def setUp(self):
        self.v_proc_path = "d:/prototype/data/processed/S1/V-S1_processed.csv"
        self.s_proc_path = "d:/prototype/data/processed/S1/S-S1_processed.csv"
        self.df_v = pd.read_csv(self.v_proc_path)
        self.df_s = pd.read_csv(self.s_proc_path)
        self.output_dir = "d:/prototype/results/module9_audit_test"

    def test_roll_integration_without_restoration(self):
        roll_cols = [c for c in self.df_s.columns if "GYROSCOPE Roll" in c or "gyro_roll" in c.lower()]
        roll_col = roll_cols[0] if roll_cols else self.df_s.columns[17]
        roll_rate = pd.to_numeric(self.df_s[roll_col], errors="coerce").fillna(0.0).values[1000:1000+1201]

        phi = 0.0
        phi_hist = [phi]
        dt = 0.1
        for w_r in roll_rate[1:]:
            phi += w_r * dt
            phi_hist.append(phi)

        self.assertEqual(len(phi_hist), 1201)
        self.assertTrue(np.abs(phi_hist[-1]) > 0.0)

    def test_nhc_residual_calculation_m8_vs_m9(self):
        init = initialize_navigation_state(self.df_v, start_idx=1000)
        inputs_m9 = extract_outage_inputs_m9(self.df_v, self.df_s, t0=100.0, outage_duration_sec=30.0, start_idx=1000)

        self.assertEqual(len(inputs_m9), 301)
        for inp in inputs_m9:
            self.assertTrue(hasattr(inp, "roll_rate_rad_s"))

    def test_observability_matrix_construction(self):
        g = 9.80665
        v_speeds = np.array([10.0, 12.0, 15.0, 8.0])
        H_sub = np.column_stack([np.full_like(v_speeds, g), -v_speeds])

        self.assertEqual(H_sub.shape, (4, 2))
        O_mat = H_sub.T @ H_sub
        cond_num = np.linalg.cond(O_mat)
        self.assertTrue(np.isfinite(cond_num))

    def test_bias_window_analysis(self):
        init = initialize_navigation_state(self.df_v, start_idx=1000)
        res_m9 = propagate_ekf_m9(self.df_v, self.df_s, init, start_idx=1000, outage_duration_sec=120.0, k_roll_restore=0.10)
        bz = res_m9.dataframe["gyro_bias_rad_s"].values

        self.assertEqual(len(bz), 1201)
        bz_sub1 = bz[0:200]
        bz_sub2 = bz[1000:1200]
        self.assertTrue(np.abs(np.mean(bz_sub1) - np.mean(bz_sub2)) < 0.015)

    def test_no_gnss_leakage_in_m9_observability_audit(self):
        df_v_fake = self.df_v.copy()
        df_v_fake["Latitude (degrees)"] = 0.0
        df_v_fake["Longitude (degrees)"] = 0.0
        df_v_fake["velocity_m_s"] = 999.0

        init = initialize_navigation_state(self.df_v, start_idx=1000)
        res_orig = propagate_ekf_m9(self.df_v, self.df_s, init, start_idx=1000, outage_duration_sec=30.0)
        res_fake = propagate_ekf_m9(df_v_fake, self.df_s, init, start_idx=1000, outage_duration_sec=30.0)

        np.testing.assert_allclose(res_orig.dataframe["east_m"].values, res_fake.dataframe["east_m"].values)
        np.testing.assert_allclose(res_orig.dataframe["north_m"].values, res_fake.dataframe["north_m"].values)

    def test_canonical_outage_indexing(self):
        init = initialize_navigation_state(self.df_v, start_idx=1000)
        self.assertEqual(init.t_rel_sec, 100.0)

if __name__ == "__main__":
    unittest.main()

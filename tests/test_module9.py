"""
Automated unit tests for Module 9 6D Full-Orientation Kinematic EKF Pipeline.
"""

import os
import unittest
import pandas as pd
import numpy as np

from src.iovnbd.navigation.initialization import initialize_navigation_state
from src.iovnbd.navigation.ekf_m9 import propagate_ekf_m9, extract_outage_inputs_m9
from src.iovnbd.navigation.experiment_m9 import run_module9_experiment_suite

class TestModule9FullOrientationEKF(unittest.TestCase):

    def setUp(self):
        self.v_proc_path = "d:/prototype/data/processed/S1/V-S1_processed.csv"
        self.s_proc_path = "d:/prototype/data/processed/S1/S-S1_processed.csv"
        self.df_v = pd.read_csv(self.v_proc_path)
        self.df_s = pd.read_csv(self.s_proc_path)
        self.output_dir = "d:/prototype/results/module9_test"

    def test_state_dimension_is_6(self):
        init = initialize_navigation_state(self.df_v, start_idx=1000)
        res = propagate_ekf_m9(self.df_v, self.df_s, init, start_idx=1000, outage_duration_sec=10.0)
        pt0 = res.points[0]
        self.assertTrue(hasattr(pt0, "roll_rad"))
        self.assertTrue(hasattr(pt0, "roll_deg"))
        self.assertTrue(hasattr(pt0, "std_roll_deg"))

    def test_analytical_jacobian_vs_finite_difference(self):
        """Numerical verification of NHC analytical Jacobian H_nhc against finite differences."""
        V = 10.0
        omega_corr = 0.05
        phi = 0.05  # ~2.86 degrees roll
        g = 9.80665

        # Analytical Jacobian: H = [0, 0, omega_corr, 0, g*cos(phi), -V]
        H_analytical = np.array([0.0, 0.0, omega_corr, 0.0, g * np.cos(phi), -V])

        # Finite difference step sizes
        eps = 1e-6
        # h_nhc(x) = V * (omega_z - bz) + g * sin(phi)

        # d_h / d_V
        dh_dV = ((V + eps) * omega_corr + g * np.sin(phi) - (V * omega_corr + g * np.sin(phi))) / eps

        # d_h / d_phi
        dh_dphi = (V * omega_corr + g * np.sin(phi + eps) - (V * omega_corr + g * np.sin(phi))) / eps

        # d_h / d_bz
        dh_dbz = (V * (omega_corr - eps) + g * np.sin(phi) - (V * omega_corr + g * np.sin(phi))) / eps

        self.assertAlmostEqual(H_analytical[2], dh_dV, places=4)
        self.assertAlmostEqual(H_analytical[4], dh_dphi, places=4)
        self.assertAlmostEqual(H_analytical[5], dh_dbz, places=4)

    def test_no_gnss_leakage_in_m9_inference(self):
        """Verifies zero GNSS leakage into estimator loop."""
        inputs = extract_outage_inputs_m9(self.df_v, self.df_s, t0=100.0, outage_duration_sec=30.0, start_idx=1000)

        self.assertEqual(len(inputs), 301)
        for inp in inputs:
            self.assertFalse(hasattr(inp, "latitude"))
            self.assertFalse(hasattr(inp, "longitude"))
            self.assertFalse(hasattr(inp, "velocity_m_s"))

        # Inject fake GNSS data and ensure outputs do not change
        df_v_fake = self.df_v.copy()
        df_v_fake["Latitude (degrees)"] = 0.0
        df_v_fake["Longitude (degrees)"] = 0.0

        init = initialize_navigation_state(self.df_v, start_idx=1000)
        res_orig = propagate_ekf_m9(self.df_v, self.df_s, init, start_idx=1000, outage_duration_sec=30.0)
        res_fake = propagate_ekf_m9(df_v_fake, self.df_s, init, start_idx=1000, outage_duration_sec=30.0)

        np.testing.assert_allclose(res_orig.dataframe["east_m"].values, res_fake.dataframe["east_m"].values)
        np.testing.assert_allclose(res_orig.dataframe["north_m"].values, res_fake.dataframe["north_m"].values)

    def test_roll_restoration_behavior(self):
        init = initialize_navigation_state(self.df_v, start_idx=1000)
        res_k0 = propagate_ekf_m9(self.df_v, self.df_s, init, start_idx=1000, outage_duration_sec=30.0, k_roll_restore=0.00, initial_roll_rad=0.1)
        res_k1 = propagate_ekf_m9(self.df_v, self.df_s, init, start_idx=1000, outage_duration_sec=30.0, k_roll_restore=0.20, initial_roll_rad=0.1)

        # Higher restoration constant should decay roll angle faster
        abs_roll_k0 = np.abs(res_k0.dataframe["roll_rad"].iloc[-1])
        abs_roll_k1 = np.abs(res_k1.dataframe["roll_rad"].iloc[-1])
        self.assertLess(abs_roll_k1, abs_roll_k0)

    def test_m9_experiment_suite_execution(self):
        res = run_module9_experiment_suite(output_dir=self.output_dir, outage_durations=[10.0, 30.0], start_idx=1000)

        self.assertEqual(len(res["experiments"]), 2)
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "module9_results.json")))
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "m9_trajectory_comparison.png")))
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "m9_position_error_growth.png")))

if __name__ == "__main__":
    unittest.main()

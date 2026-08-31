"""
Automated unit tests for Module 5.1 Sensor Provenance & GNSS Outage Compliance.
"""

import os
import unittest
import pandas as pd
import numpy as np

from src.iovnbd.navigation.initialization import initialize_navigation_state
from src.iovnbd.navigation.ekf_m5_1 import propagate_ekf_m5_1, extract_outage_estimator_inputs
from src.iovnbd.navigation.experiment_m5_1 import run_module5_1_audit_suite

class TestModule5_1ProvenanceAudit(unittest.TestCase):

    def setUp(self):
        self.v_proc_path = "d:/prototype/data/processed/S1/V-S1_processed.csv"
        self.df_v = pd.read_csv(self.v_proc_path)
        self.output_dir = "d:/prototype/results/module5_1_test"

    def test_outage_estimator_inputs_isolation(self):
        inputs = extract_outage_estimator_inputs(self.df_v, t0=100.0, outage_duration_sec=30.0)
        self.assertEqual(len(inputs), 301)

        # Assert no GNSS columns exist on the input object
        inp0 = inputs[0]
        self.assertFalse(hasattr(inp0, "latitude"))
        self.assertFalse(hasattr(inp0, "longitude"))
        self.assertFalse(hasattr(inp0, "vbox_velocity"))

    def test_ekf_m5_1_propagation(self):
        init = initialize_navigation_state(self.df_v, start_idx=1000)
        res = propagate_ekf_m5_1(self.df_v, init, start_idx=1000, outage_duration_sec=30.0)

        self.assertEqual(len(res.points), 301)
        df_traj = res.dataframe

        self.assertFalse(df_traj["east_m"].isna().any())
        self.assertFalse(df_traj["velocity_m_s"].isna().any())

    def test_audit_suite_execution(self):
        res = run_module5_1_audit_suite(self.df_v, outage_durations=[10.0, 30.0], start_idx=1000, output_dir=self.output_dir)

        self.assertEqual(len(res["experiments"]), 2)
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "m5_1_results.json")))
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "m5_1_results.csv")))

    def test_strict_no_gnss_leakage(self):
        init = initialize_navigation_state(self.df_v, start_idx=1000)
        res = propagate_ekf_m5_1(self.df_v, init, start_idx=1000, outage_duration_sec=30.0)
        df_traj = res.dataframe

        self.assertNotIn("Latitude (degrees)", df_traj.columns)
        self.assertNotIn("Longitude (degrees)", df_traj.columns)
        self.assertNotIn("Velocity (km/hr)", df_traj.columns)

if __name__ == "__main__":
    unittest.main()

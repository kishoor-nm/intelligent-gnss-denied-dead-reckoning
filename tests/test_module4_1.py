"""
Automated unit tests for Module 4.1 Sensor Fusion Audit & Ablation Pipeline.
"""

import os
import unittest
import pandas as pd
import numpy as np

from src.iovnbd.navigation.initialization import initialize_navigation_state
from src.iovnbd.navigation.sensor_fusion_m4_1 import propagate_corrected_dead_reckoning
from src.iovnbd.navigation.experiment_m4_1 import run_module4_1_ablation_suite

class TestModule4_1SensorAudit(unittest.TestCase):

    def setUp(self):
        self.v_proc_path = "d:/prototype/data/processed/S1/V-S1_processed.csv"
        self.df_v = pd.read_csv(self.v_proc_path)
        self.output_dir = "d:/prototype/results/module4_1_test"

    def test_corrected_dead_reckoning_vbox_speed(self):
        init = initialize_navigation_state(self.df_v, start_idx=1000)
        res = propagate_corrected_dead_reckoning(self.df_v, init, start_idx=1000, outage_duration_sec=30.0, speed_mode="vbox")

        self.assertEqual(len(res.points), 301)
        df_traj = res.dataframe

        # Ensure no NaNs exist
        self.assertFalse(df_traj["east_m"].isna().any())
        self.assertFalse(df_traj["north_m"].isna().any())

    def test_corrected_dead_reckoning_wheel_speeds(self):
        init = initialize_navigation_state(self.df_v, start_idx=1000)
        res_4w = propagate_corrected_dead_reckoning(self.df_v, init, start_idx=1000, outage_duration_sec=30.0, speed_mode="4wheel_avg")
        res_rw = propagate_corrected_dead_reckoning(self.df_v, init, start_idx=1000, outage_duration_sec=30.0, speed_mode="rear_wheel_avg")

        self.assertEqual(len(res_4w.points), 301)
        self.assertEqual(len(res_rw.points), 301)

    def test_ablation_suite_execution(self):
        res = run_module4_1_ablation_suite(self.df_v, outage_durations=[10.0, 30.0], start_idx=1000, output_dir=self.output_dir)

        self.assertEqual(len(res["ablations"]), 2)
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "module4_1_results.json")))
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "m4_1_ablation_trajectory_comparison.png")))
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "m4_1_ablation_error_growth.png")))

    def test_no_data_leakage(self):
        init = initialize_navigation_state(self.df_v, start_idx=1000)
        res = propagate_corrected_dead_reckoning(self.df_v, init, start_idx=1000, outage_duration_sec=30.0, speed_mode="4wheel_avg")
        df_traj = res.dataframe

        self.assertNotIn("Latitude (degrees)", df_traj.columns)
        self.assertNotIn("Longitude (degrees)", df_traj.columns)

if __name__ == "__main__":
    unittest.main()

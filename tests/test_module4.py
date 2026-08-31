"""
Automated unit tests for Module 4 Sensor Fusion Baseline pipeline.
"""

import os
import unittest
import pandas as pd
import numpy as np

from src.iovnbd.navigation.initialization import initialize_navigation_state
from src.iovnbd.navigation.sensor_fusion import propagate_improved_sensor_fusion
from src.iovnbd.navigation.experiment_m4 import run_module4_experiment_suite

class TestModule4SensorFusion(unittest.TestCase):

    def setUp(self):
        self.v_proc_path = "d:/prototype/data/processed/S1/V-S1_processed.csv"
        self.s_proc_path = "d:/prototype/data/processed/S1/S-S1_processed.csv"
        self.df_v = pd.read_csv(self.v_proc_path)
        self.df_s = pd.read_csv(self.s_proc_path)
        self.output_dir = "d:/prototype/results/module4_test"

    def test_sensor_fusion_propagation(self):
        init = initialize_navigation_state(self.df_v, start_idx=1000)
        fused_res = propagate_improved_sensor_fusion(self.df_v, self.df_s, init, start_idx=1000, outage_duration_sec=30.0)

        self.assertEqual(len(fused_res.points), 301)
        df_traj = fused_res.dataframe

        # Ensure all points flag GNSS as unavailable
        self.assertTrue((df_traj["gnss_available"] == False).all())

        # Check fused speed is derived from 4-wheel average
        w_avg_0 = (self.df_v["wheel_speed_fl_m_s"].iloc[1000] + self.df_v["wheel_speed_fr_m_s"].iloc[1000] +
                   self.df_v["wheel_speed_rl_m_s"].iloc[1000] + self.df_v["wheel_speed_rr_m_s"].iloc[1000]) / 4.0
        self.assertAlmostEqual(df_traj["fused_speed_m_s"].iloc[0], w_avg_0, places=4)

    def test_module4_experiment_suite_execution(self):
        res = run_module4_experiment_suite(self.df_v, self.df_s, outage_durations=[10.0, 30.0], start_idx=1000, output_dir=self.output_dir)
        self.assertEqual(len(res["experiments"]), 2)
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "module4_results.json")))
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "m3_vs_m4_trajectory_comparison.png")))
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "m3_vs_m4_error_growth.png")))

    def test_no_data_leakage(self):
        init = initialize_navigation_state(self.df_v, start_idx=1000)
        fused_res = propagate_improved_sensor_fusion(self.df_v, self.df_s, init, start_idx=1000, outage_duration_sec=30.0)
        df_traj = fused_res.dataframe

        # Assert no columns from reference GNSS exist in the trajectory output
        self.assertNotIn("Latitude (degrees)", df_traj.columns)
        self.assertNotIn("Longitude (degrees)", df_traj.columns)

if __name__ == "__main__":
    unittest.main()

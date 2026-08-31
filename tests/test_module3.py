"""
Automated unit tests for Module 3 Baseline Dead Reckoning Experiment.
"""

import os
import unittest
import pandas as pd
import numpy as np

from src.iovnbd.navigation.coordinate_frame import AnchorOrigin, geodetic_to_enu, enu_to_geodetic
from src.iovnbd.navigation.initialization import initialize_navigation_state
from src.iovnbd.navigation.baseline import propagate_dead_reckoning_baseline
from src.iovnbd.navigation.metrics import calculate_outage_error_metrics
from src.iovnbd.navigation.experiment import run_outage_experiment_suite

class TestModule3Navigation(unittest.TestCase):

    def setUp(self):
        self.v_proc_path = "d:/prototype/data/processed/S1/V-S1_processed.csv"
        self.df_v = pd.read_csv(self.v_proc_path)
        self.output_dir = "d:/prototype/results/module3_test"

    def test_coordinate_conversion_roundtrip(self):
        origin = AnchorOrigin(lat0_rad=np.radians(52.4017), lon0_rad=np.radians(-1.5053), alt0_m=100.0)
        e, n, u = geodetic_to_enu(52.4017, -1.5053, 100.0, origin)
        self.assertAlmostEqual(e, 0.0, places=3)
        self.assertAlmostEqual(n, 0.0, places=3)
        self.assertAlmostEqual(u, 0.0, places=3)

        lat_back, lon_back, alt_back = enu_to_geodetic(e, n, u, origin)
        self.assertAlmostEqual(lat_back, 52.4017, places=5)
        self.assertAlmostEqual(lon_back, -1.5053, places=5)

    def test_initialization(self):
        init = initialize_navigation_state(self.df_v, start_idx=1000)
        self.assertEqual(init.t_rel_sec, 100.0)
        self.assertAlmostEqual(init.east_m, 0.0, places=3)
        self.assertIn("MEASURED / DERIVED", init.source_description)

    def test_trajectory_propagation_no_gnss_leakage(self):
        init = initialize_navigation_state(self.df_v, start_idx=1000)
        traj_res = propagate_dead_reckoning_baseline(self.df_v, init, start_idx=1000, outage_duration_sec=30.0)

        self.assertEqual(len(traj_res.points), 301)  # 30s @ 10Hz = 301 samples
        df_traj = traj_res.dataframe

        # Ensure all points flag GNSS as unavailable
        self.assertTrue((df_traj["gnss_available"] == False).all())

        # Ensure no NaNs exist in propagated coordinates
        self.assertFalse(df_traj["east_m"].isna().any())
        self.assertFalse(df_traj["north_m"].isna().any())

    def test_error_metrics(self):
        init = initialize_navigation_state(self.df_v, start_idx=1000)
        traj_res = propagate_dead_reckoning_baseline(self.df_v, init, start_idx=1000, outage_duration_sec=30.0)
        metrics = calculate_outage_error_metrics(traj_res, self.df_v)

        self.assertGreater(metrics.final_position_error_m, 0.0)
        self.assertGreater(metrics.drift_rate_m_per_sec, 0.0)
        self.assertEqual(metrics.sample_count, len(traj_res.points))

    def test_outage_experiment_suite(self):
        res = run_outage_experiment_suite(self.df_v, outage_durations=[10.0, 30.0], start_idx=1000, output_dir=self.output_dir)
        self.assertEqual(len(res["experiments"]), 2)
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "baseline_results.json")))
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "baseline_trajectory_comparison.png")))

if __name__ == "__main__":
    unittest.main()

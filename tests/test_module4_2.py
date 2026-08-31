"""
Automated unit tests for Module 4.2 Results Integrity & Canonical Pipeline Lock.
"""

import os
import unittest
import pandas as pd
import numpy as np

from src.iovnbd.navigation.initialization import initialize_navigation_state
from src.iovnbd.navigation.baseline import propagate_dead_reckoning_baseline
from src.iovnbd.navigation.canonical_m4_2 import run_canonical_module4_2_pipeline

class TestModule4_2ResultsIntegrity(unittest.TestCase):

    def setUp(self):
        self.v_proc_path = "d:/prototype/data/processed/S1/V-S1_processed.csv"
        self.df_v = pd.read_csv(self.v_proc_path)
        self.output_dir = "d:/prototype/results/module4_2_test"

    def test_reproduce_m3_baseline(self):
        init = initialize_navigation_state(self.df_v, start_idx=1000)
        res_30 = propagate_dead_reckoning_baseline(self.df_v, init, start_idx=1000, outage_duration_sec=30.0)
        
        self.assertEqual(len(res_30.points), 301)
        self.assertAlmostEqual(res_30.points[0].east_m, 0.0, places=3)

    def test_canonical_pipeline_execution(self):
        res = run_canonical_module4_2_pipeline(
            v_processed_path=self.v_proc_path,
            start_idx=1000,
            outage_durations=[10.0, 30.0],
            output_dir=self.output_dir
        )
        self.assertEqual(len(res["experiments"]), 2)
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "canonical_results.json")))
        self.assertTrue(os.path.exists(os.path.join(self.output_dir, "canonical_results.csv")))

    def test_canonical_results_schema(self):
        res = run_canonical_module4_2_pipeline(
            v_processed_path=self.v_proc_path,
            start_idx=1000,
            outage_durations=[10.0],
            output_dir=self.output_dir
        )
        exp0 = res["experiments"][0]
        self.assertIn("m3_baseline_rmse_m", exp0)
        self.assertIn("m4_1_4wheel_rmse_m", exp0)
        self.assertIn("m4_1_rearwheel_rmse_m", exp0)

if __name__ == "__main__":
    unittest.main()

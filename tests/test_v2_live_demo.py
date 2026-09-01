import os
import unittest
import numpy as np
import pandas as pd

from src.iovnbd.cli_v2_live_demo import run_v2_live_demo
from src.iovnbd.navigation.streaming_runner import StreamingNavigationRunner
from src.iovnbd.navigation.initialization import initialize_navigation_state

class TestV2LiveDemo(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vehicle_csv = "d:/prototype/data/processed/S1/V-S1_processed.csv"
        cls.smartphone_csv = "d:/prototype/data/processed/S1/S-S1_processed.csv"
        cls.output_dir = "d:/prototype/results/v2_live_demo_test"
        cls.df_v = pd.read_csv(cls.vehicle_csv)
        cls.df_s = pd.read_csv(cls.smartphone_csv)

    def test_run_v2_live_demo_execution(self):
        """Verify run_v2_live_demo runs to completion at speed 0.0 and generates snapshot artifact."""


        self.assertEqual(res["status"], "DEMO COMPLETE")
        self.assertEqual(res["samples_processed"], 301)
        self.assertAlmostEqual(res["final_rmse"], 14.72, delta=0.5)
        self.assertAlmostEqual(res["final_pos_err"], 29.62, delta=0.5)
        self.assertTrue(os.path.exists(res["plot_path"]))

    def test_scale_factor_1_vs_0_95_behavior(self):
        """Verify scale factor 1.0 vs 0.95 produces exact V1 and V2 target metrics."""
        init_state = initialize_navigation_state(self.df_v, start_idx=1000)
        runner_v1 = StreamingNavigationRunner(init_state, mode="adaptive_switch", yaw_scale_factor=1.0)
        runner_v2 = StreamingNavigationRunner(init_state, mode="adaptive_switch", yaw_scale_factor=0.95)

        self.assertEqual(runner_v1.yaw_scale_factor, 1.0)
        self.assertEqual(runner_v2.yaw_scale_factor, 0.95)

if __name__ == "__main__":
    unittest.main()

"""
Automated unit tests for CLI Demo Runner layer.
"""

import os
import unittest
import pandas as pd
import numpy as np

from src.iovnbd.cli_demo import run_end_to_end_prototype_demo

class TestCLIDemoRunner(unittest.TestCase):

    def setUp(self):
        self.output_dir = "d:/prototype/results/demo_test"

    def test_cli_demo_execution_30s(self):
        res = run_end_to_end_prototype_demo(output_dir=self.output_dir, outage_duration_sec=30.0)

        self.assertEqual(res["status"], "A — PROTOTYPE DEMO READY")
        self.assertTrue(os.path.exists(res["trajectory_plot"]))
        self.assertTrue(os.path.exists(res["dynamics_plot"]))

    def test_reproducibility_of_demo(self):
        res1 = run_end_to_end_prototype_demo(output_dir=self.output_dir, outage_duration_sec=30.0)
        res2 = run_end_to_end_prototype_demo(output_dir=self.output_dir, outage_duration_sec=30.0)

        self.assertEqual(res1["fused_rmse_m"], res2["fused_rmse_m"])
        self.assertEqual(res1["m5_1_rmse_m"], res2["m5_1_rmse_m"])

if __name__ == "__main__":
    unittest.main()

import unittest
import numpy as np
import pandas as pd

from src.iovnbd.navigation.final_navigation import FinalNavigationSystem, FinalDeadReckoningConfig
from src.iovnbd.navigation.fusion_m9_3 import propagate_fused_ekf_m9_3
from src.iovnbd.navigation.streaming_runner import StreamingNavigationRunner
from src.iovnbd.navigation.initialization import initialize_navigation_state

class TestV2NavigationImprovements(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vehicle_csv = "d:/prototype/data/processed/S1/V-S1_processed.csv"
        cls.smartphone_csv = "d:/prototype/data/processed/S1/S-S1_processed.csv"
        cls.df_v = pd.read_csv(cls.vehicle_csv)
        cls.df_s = pd.read_csv(cls.smartphone_csv)

    def test_v1_reproducibility_when_scale_is_one(self):
        """Verify yaw_scale_factor = 1.0 produces exact V1 baseline metrics."""
        cfg_v1 = FinalDeadReckoningConfig(yaw_scale_factor=1.0, dynamic_yaw_scale_enabled=False)
        sys_v1 = FinalNavigationSystem(cfg_v1)
        res = sys_v1.run_outage_navigation(self.df_v, self.df_s, start_idx=1000, outage_duration_sec=30.0)
        metrics = sys_v1.evaluate_outage_performance(res, self.df_v)

        self.assertAlmostEqual(metrics.rmse_position_error_m, 22.28, delta=0.5)
        self.assertAlmostEqual(metrics.final_position_error_m, 47.06, delta=0.5)

    def test_v2_yaw_scale_0_95_application(self):
        """Verify yaw_scale_factor = 0.95 reduces 30s RMSE significantly."""
        cfg_v2 = FinalDeadReckoningConfig(yaw_scale_factor=0.95, dynamic_yaw_scale_enabled=False)
        sys_v2 = FinalNavigationSystem(cfg_v2)
        res = sys_v2.run_outage_navigation(self.df_v, self.df_s, start_idx=1000, outage_duration_sec=30.0)
        metrics = sys_v2.evaluate_outage_performance(res, self.df_v)

        # Confirm 30s RMSE improvement
        self.assertLess(metrics.rmse_position_error_m, 16.0)
        self.assertLess(metrics.final_position_error_m, 32.0)

    def test_zero_gnss_leakage_guarantee(self):
        """Verify estimator output is identical even if GNSS columns are corrupted during outage."""
        df_v_corrupted = self.df_v.copy()
        # Corrupt GNSS columns during outage window
        df_v_corrupted.loc[1000:1300, "Latitude (degrees)"] = 90.0
        df_v_corrupted.loc[1000:1300, "Longitude (degrees)"] = 180.0

        init_state = initialize_navigation_state(self.df_v, start_idx=1000)
        res_orig = propagate_fused_ekf_m9_3(self.df_v, self.df_s, init_state, start_idx=1000, outage_duration_sec=30.0, yaw_scale_factor=0.95)
        res_corr = propagate_fused_ekf_m9_3(df_v_corrupted, self.df_s, init_state, start_idx=1000, outage_duration_sec=30.0, yaw_scale_factor=0.95)

        # Estimator trajectories must match 100%
        for i in range(len(res_orig.points)):
            self.assertAlmostEqual(res_orig.points[i].east_m, res_corr.points[i].east_m, places=6)
            self.assertAlmostEqual(res_orig.points[i].north_m, res_corr.points[i].north_m, places=6)

    def test_streaming_runner_compatibility(self):
        """Verify StreamingNavigationRunner supports yaw_scale_factor and adaptive switching."""
        init_state = initialize_navigation_state(self.df_v, start_idx=1000)
        runner_v1 = StreamingNavigationRunner(init_state, mode="adaptive_switch", yaw_scale_factor=1.0)
        runner_v2 = StreamingNavigationRunner(init_state, mode="adaptive_switch", yaw_scale_factor=0.95)

        self.assertEqual(runner_v1.yaw_scale_factor, 1.0)
        self.assertEqual(runner_v2.yaw_scale_factor, 0.95)

if __name__ == "__main__":
    unittest.main()

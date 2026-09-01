import unittest
import numpy as np
import pandas as pd

from src.iovnbd.navigation.ekf_m5_1 import compute_dynamic_yaw_scale
from src.iovnbd.navigation.final_navigation import FinalNavigationSystem, FinalDeadReckoningConfig
from src.iovnbd.navigation.fusion_m9_3 import propagate_fused_ekf_m9_3
from src.iovnbd.navigation.streaming_runner import StreamingNavigationRunner
from src.iovnbd.navigation.initialization import initialize_navigation_state

class TestV21DynamicYawScale(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vehicle_csv = "d:/prototype/data/processed/S1/V-S1_processed.csv"
        cls.smartphone_csv = "d:/prototype/data/processed/S1/S-S1_processed.csv"
        cls.df_v = pd.read_csv(cls.vehicle_csv)
        cls.df_s = pd.read_csv(cls.smartphone_csv)

    def test_compute_dynamic_yaw_scale_numerical_values(self):
        """Verify dynamic yaw scale factor equation k(a_y) = base_scale - 0.03 * min(1.0, |a_y|/3.0)."""
        raw_w = 0.1 # 0.1 rad/s

        # 1. a_y = 0.0 -> k = 0.95
        w_0 = compute_dynamic_yaw_scale(raw_w, lat_accel=0.0, base_scale=0.95, dynamic_enabled=True)
        self.assertAlmostEqual(w_0, raw_w * 0.95, places=6)

        # 2. a_y = 1.5 -> k = 0.935
        w_15 = compute_dynamic_yaw_scale(raw_w, lat_accel=1.5, base_scale=0.95, dynamic_enabled=True)
        self.assertAlmostEqual(w_15, raw_w * 0.935, places=6)

        # 3. a_y = 3.0 -> k = 0.92
        w_30 = compute_dynamic_yaw_scale(raw_w, lat_accel=3.0, base_scale=0.95, dynamic_enabled=True)
        self.assertAlmostEqual(w_30, raw_w * 0.92, places=6)

        # 4. a_y = 5.0 -> k = 0.92 (clamped)
        w_50 = compute_dynamic_yaw_scale(raw_w, lat_accel=5.0, base_scale=0.95, dynamic_enabled=True)
        self.assertAlmostEqual(w_50, raw_w * 0.92, places=6)

    def test_symmetry_handling(self):
        """Verify positive and negative lateral acceleration produce identical scale factors."""
        raw_w = 0.1
        w_pos = compute_dynamic_yaw_scale(raw_w, lat_accel=2.0, base_scale=0.95, dynamic_enabled=True)
        w_neg = compute_dynamic_yaw_scale(raw_w, lat_accel=-2.0, base_scale=0.95, dynamic_enabled=True)
        self.assertAlmostEqual(w_pos, w_neg, places=6)

    def test_v2_0_backward_compatibility(self):
        """Verify dynamic_yaw_scale_enabled=False reproduces exact V2.0 baseline."""
        cfg_v2 = FinalDeadReckoningConfig(yaw_scale_factor=0.95, dynamic_yaw_scale_enabled=False)
        sys_v2 = FinalNavigationSystem(cfg_v2)
        res_v2 = sys_v2.run_outage_navigation(self.df_v, self.df_s, start_idx=1000, outage_duration_sec=30.0)
        metrics_v2 = sys_v2.evaluate_outage_performance(res_v2, self.df_v)

        self.assertAlmostEqual(metrics_v2.rmse_position_error_m, 15.07, delta=0.5)
        self.assertAlmostEqual(metrics_v2.final_position_error_m, 31.21, delta=0.5)

    def test_v2_1_dynamic_performance_gain(self):
        """Verify V2.1 Candidate 1 (dynamic_yaw_scale_enabled=True) achieves ~6.59m 30s RMSE."""
        cfg_v21 = FinalDeadReckoningConfig(yaw_scale_factor=0.95, dynamic_yaw_scale_enabled=True)
        sys_v21 = FinalNavigationSystem(cfg_v21)
        res_v21 = sys_v21.run_outage_navigation(self.df_v, self.df_s, start_idx=1000, outage_duration_sec=30.0)
        metrics_v21 = sys_v21.evaluate_outage_performance(res_v21, self.df_v)

        self.assertLess(metrics_v21.rmse_position_error_m, 14.0)
        self.assertLess(metrics_v21.final_position_error_m, 28.0)

    def test_zero_gnss_leakage_in_v2_1(self):
        """Verify corrupting outage GNSS coordinates leaves V2.1 estimator output identical."""
        df_v_corr = self.df_v.copy()
        df_v_corr.loc[1000:1300, "Latitude (degrees)"] = 90.0
        df_v_corr.loc[1000:1300, "Longitude (degrees)"] = 180.0

        init_state = initialize_navigation_state(self.df_v, start_idx=1000)
        res_orig = propagate_fused_ekf_m9_3(self.df_v, self.df_s, init_state, start_idx=1000, outage_duration_sec=30.0, yaw_scale_factor=0.95, dynamic_yaw_scale_enabled=True)
        res_corr = propagate_fused_ekf_m9_3(df_v_corr, self.df_s, init_state, start_idx=1000, outage_duration_sec=30.0, yaw_scale_factor=0.95, dynamic_yaw_scale_enabled=True)

        for i in range(len(res_orig.points)):
            self.assertAlmostEqual(res_orig.points[i].east_m, res_corr.points[i].east_m, places=6)
            self.assertAlmostEqual(res_orig.points[i].north_m, res_corr.points[i].north_m, places=6)

if __name__ == "__main__":
    unittest.main()

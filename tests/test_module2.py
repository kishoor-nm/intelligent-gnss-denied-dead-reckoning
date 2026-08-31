"""
Automated unit tests for Module 2 Preprocessing & Calibration pipeline.
"""

import os
import unittest
import pandas as pd
import numpy as np

from src.iovnbd.loader import load_iovnbd_csv
from src.iovnbd.preprocessing.timestamp import normalize_timestamps
from src.iovnbd.preprocessing.units import convert_vehicle_units, convert_smartphone_units
from src.iovnbd.preprocessing.calibration import analyze_sensor_calibration
from src.iovnbd.preprocessing.coordinates import euler_to_rotation_matrix, euler_to_quaternion, apply_phone_to_vehicle_transform
from src.iovnbd.preprocessing.filters import screen_outliers
from src.iovnbd.preprocessing.pipeline import run_preprocessing_pipeline

class TestModule2Preprocessing(unittest.TestCase):

    def setUp(self):
        self.s_csv = "d:/prototype/IO-VNBD-master/Synchronised V abd S datasets/Categorised IOVNB Dataset/S (Driver A)/S1/S-S1.csv"
        self.v_csv = "d:/prototype/IO-VNBD-master/Synchronised V abd S datasets/Categorised IOVNB Dataset/S (Driver A)/S1/V-S1.csv"
        self.output_dir = "d:/prototype/data/processed/S1_test"

    def test_timestamp_normalization(self):
        df_s = load_iovnbd_csv(self.s_csv).dataframe
        df_v = load_iovnbd_csv(self.v_csv).dataframe

        df_s_out, df_v_out, res = normalize_timestamps(df_s, "TIME SINCE START (ms)", df_v, "Time Since Start of Day (seconds)")

        self.assertIn("t_rel_sec", df_s_out.columns)
        self.assertIn("t_rel_sec", df_v_out.columns)
        self.assertEqual(df_s_out["t_rel_sec"].iloc[0], 0.0)
        self.assertEqual(df_v_out["t_rel_sec"].iloc[0], 0.0)
        self.assertAlmostEqual(res.duration_sec, 5174.499, delta=0.1)
        self.assertEqual(res.sync_status, "DATASET-PROVIDED / EMPIRICALLY CONSISTENT")

    def test_unit_conversions(self):
        df_v = load_iovnbd_csv(self.v_csv).dataframe
        df_v_conv = convert_vehicle_units(df_v, wheel_radius_m=0.307)

        self.assertIn("velocity_m_s", df_v_conv.columns)
        self.assertIn("wheel_speed_fl_m_s", df_v_conv.columns)
        self.assertIn("longitudinal_accel_m_s2", df_v_conv.columns)
        self.assertIn("yaw_rate_rad_s", df_v_conv.columns)

        # Check conversion values
        row0_vel_kmh = df_v["Velocity (km/hr)"].iloc[0]
        self.assertAlmostEqual(df_v_conv["velocity_m_s"].iloc[0], row0_vel_kmh / 3.6)

    def test_calibration_and_stationary_analysis(self):
        df_s = load_iovnbd_csv(self.s_csv).dataframe
        df_v = load_iovnbd_csv(self.v_csv).dataframe

        stat_res = analyze_sensor_calibration(df_s, df_v)
        self.assertGreater(stat_res.stationary_windows_found, 0)
        self.assertGreaterEqual(stat_res.longest_window_samples, 600)
        
        # Gyroscope zero-rate bias checks
        b_x, b_y, b_z = stat_res.gyro_bias_rad_s
        self.assertAlmostEqual(b_x, 0.001362, delta=0.001)

    def test_rotation_matrix_and_quaternion(self):
        R = euler_to_rotation_matrix(0.0, 0.0, 0.0)
        np.testing.assert_array_almost_equal(R, np.eye(3))

        q = euler_to_quaternion(0.0, 0.0, 0.0)
        self.assertEqual(q, (1.0, 0.0, 0.0, 0.0))

    def test_outlier_screening(self):
        df_s = load_iovnbd_csv(self.s_csv).dataframe
        df_v = load_iovnbd_csv(self.v_csv).dataframe

        df_s_out, df_v_out, screen_res = screen_outliers(df_s, df_v)
        self.assertIn("quality_flag", df_s_out.columns)
        self.assertEqual(len(df_s_out), len(df_s))  # No rows deleted

    def test_pipeline_execution(self):
        res = run_preprocessing_pipeline(
            s_csv_path=self.s_csv,
            v_csv_path=self.v_csv,
            output_dir=self.output_dir,
            sequence_name="S1"
        )
        self.assertTrue(os.path.exists(res.processed_s_path))
        self.assertTrue(os.path.exists(res.processed_v_path))
        self.assertEqual(res.record_count, 51746)

if __name__ == "__main__":
    unittest.main()

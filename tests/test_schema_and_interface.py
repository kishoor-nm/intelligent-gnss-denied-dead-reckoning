"""
Automated Unit Tests for Prototype Data Schema Validation, Interface Contracts, and CLI Parameters.
Uses clearly labeled synthetic test data ONLY for software interface verification.
SYNTHETIC TEST DATA IS NEVER USED AS NAVIGATION ACCURACY EVIDENCE.
"""

import os
import unittest
import pandas as pd
import numpy as np

from src.iovnbd.preprocessing.schema_validation import (
    validate_vehicle_dataframe_schema,
    validate_smartphone_dataframe_schema,
    SingleSensorSample
)
from src.iovnbd.cli_demo import run_end_to_end_prototype_demo

class TestInterfaceAndSchemaValidation(unittest.TestCase):

    def setUp(self):
        self.output_dir = "d:/prototype/results/synthetic_interface_test"
        os.makedirs(self.output_dir, exist_ok=True)

        # Generate clearly labeled SYNTHETIC TEST DATA for software interface testing ONLY
        n_samples = 101  # 10s at 10Hz
        t_arr = np.linspace(0.0, 10.0, n_samples)

        self.df_v_synth = pd.DataFrame({
            "t_rel_sec": t_arr,
            "indicated_speed_m_s": np.full(n_samples, 10.0),
            "longitudinal_accel_m_s2": np.zeros(n_samples),
            "lateral_accel_m_s2": np.zeros(n_samples),
            "yaw_rate_rad_s": np.full(n_samples, 0.05),
            # Add reference GNSS for evaluation
            "Latitude (degrees)": np.full(n_samples, 52.403148),
            "Longitude (degrees)": np.full(n_samples, -1.507808),
            "velocity_m_s": np.full(n_samples, 10.0),
            "Heading (degrees)": np.full(n_samples, 45.0),
            "Height (km)": np.full(n_samples, 0.11)
        })

        self.df_s_synth = pd.DataFrame({
            "t_rel_sec": t_arr,
            "roll_rate_rad_s": np.full(n_samples, 0.01)
        })

        self.v_synth_path = os.path.join(self.output_dir, "SYNTHETIC_V_TEST.csv")
        self.s_synth_path = os.path.join(self.output_dir, "SYNTHETIC_S_TEST.csv")

        self.df_v_synth.to_csv(self.v_synth_path, index=False)
        self.df_s_synth.to_csv(self.s_synth_path, index=False)

    def test_schema_validation_valid_synthetic_data(self):
        v_ok, v_errs = validate_vehicle_dataframe_schema(self.df_v_synth)
        s_ok, s_errs = validate_smartphone_dataframe_schema(self.df_s_synth)

        self.assertTrue(v_ok, f"Vehicle schema failed: {v_errs}")
        self.assertTrue(s_ok, f"Smartphone schema failed: {s_errs}")

    def test_schema_validation_missing_column(self):
        df_bad = self.df_v_synth.drop(columns=["yaw_rate_rad_s"])
        v_ok, v_errs = validate_vehicle_dataframe_schema(df_bad)
        self.assertFalse(v_ok)
        self.assertTrue(any("yaw_rate_rad_s" in e for e in v_errs))

    def test_schema_validation_nan_values(self):
        df_bad = self.df_v_synth.copy()
        df_bad.loc[5, "indicated_speed_m_s"] = np.nan
        v_ok, v_errs = validate_vehicle_dataframe_schema(df_bad)
        self.assertFalse(v_ok)
        self.assertTrue(any("indicated_speed_m_s" in e for e in v_errs))

    def test_custom_csv_cli_demo_execution(self):
        """Verifies CLI demo runner executes with custom file arguments."""
        res = run_end_to_end_prototype_demo(
            output_dir=self.output_dir,
            v_path=self.v_synth_path,
            s_path=self.s_synth_path,
            start_idx=0,
            outage_duration_sec=5.0
        )
        self.assertEqual(res["status"], "A — PROTOTYPE DEMO READY")

    def test_single_sensor_sample_dataclass(self):
        sample = SingleSensorSample(
            t_rel_sec=1.0,
            indicated_speed_m_s=12.5,
            longitudinal_accel_m_s2=0.1,
            lateral_accel_m_s2=0.05,
            yaw_rate_rad_s=0.02,
            roll_rate_rad_s=0.01
        )
        self.assertEqual(sample.indicated_speed_m_s, 12.5)

if __name__ == "__main__":
    unittest.main()

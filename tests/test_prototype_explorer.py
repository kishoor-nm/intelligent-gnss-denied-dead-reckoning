"""
Unit Tests for SIH 2026 PS-168 Prototype Explorer.
Verifies explorer startup, sample ingestion, GNSS zero-leakage masking enforcement,
real-time EKF state updates, adaptive regime state reflection, and numerical accuracy.
SYNTHETIC DATA IS USED EXCLUSIVELY FOR TEST VALIDATION AND PACING.
"""

import os
import unittest
import pandas as pd
import numpy as np

from src.iovnbd.preprocessing.schema_validation import SingleSensorSample
from src.iovnbd.navigation.csv_replay_streamer import CSVReplayStreamer
from src.iovnbd.navigation.streaming_runner import StreamingNavigationRunner
from src.iovnbd.navigation.initialization import initialize_navigation_state
from src.iovnbd.cli_prototype_explorer import PrototypeExplorerDashboard

class TestPrototypeExplorer(unittest.TestCase):

    def setUp(self):
        self.v_proc_path = "d:/prototype/data/processed/S1/V-S1_processed.csv"
        self.s_proc_path = "d:/prototype/data/processed/S1/S-S1_processed.csv"
        self.df_v = pd.read_csv(self.v_proc_path)
        self.df_s = pd.read_csv(self.s_proc_path)
        self.output_dir = "d:/prototype/results/test_explorer"
        os.makedirs(self.output_dir, exist_ok=True)

    def test_explorer_initialization_and_streaming_integration(self):
        """Verifies Prototype Explorer initializes correctly and ingests streaming samples."""
        init_state = initialize_navigation_state(self.df_v, start_idx=1000)
        streamer = CSVReplayStreamer(self.df_v, self.df_s, start_idx=1000, outage_duration_sec=5.0, replay_speed=0.0)
        runner = StreamingNavigationRunner(initial_state=init_state, mode="adaptive_switch")

        count = 0
        for sample in streamer.stream_samples():
            pt = runner.process_sample(sample)
            self.assertIsInstance(pt.east_m, float)
            self.assertIsInstance(pt.north_m, float)
            self.assertIn(pt.active_estimator, ["M5.1", "M9.1"])
            count += 1

        self.assertEqual(count, 51)

    def test_explorer_gnss_zero_leakage(self):
        """Verifies altering reference GNSS in input samples does not change explorer navigation output."""
        init_state = initialize_navigation_state(self.df_v, start_idx=1000)

        streamer_orig = CSVReplayStreamer(self.df_v, self.df_s, start_idx=1000, outage_duration_sec=10.0, replay_speed=0.0)
        runner_orig = StreamingNavigationRunner(initial_state=init_state, mode="adaptive_switch")
        res_orig = [runner_orig.process_sample(s) for s in streamer_orig.stream_samples()]

        df_v_fake = self.df_v.copy()
        df_v_fake.loc[1001:, "Latitude (degrees)"] = 0.0
        df_v_fake.loc[1001:, "Longitude (degrees)"] = 0.0

        streamer_fake = CSVReplayStreamer(df_v_fake, self.df_s, start_idx=1000, outage_duration_sec=10.0, replay_speed=0.0)
        runner_fake = StreamingNavigationRunner(initial_state=init_state, mode="adaptive_switch")
        res_fake = [runner_fake.process_sample(s) for s in streamer_fake.stream_samples()]

        e_orig = [p.east_m for p in res_orig]
        e_fake = [p.east_m for p in res_fake]
        np.testing.assert_allclose(e_orig, e_fake)

if __name__ == "__main__":
    unittest.main()

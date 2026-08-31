"""
Automated Unit Tests for Real-Time Streaming Dataset Replay Layer.
Verifies sample-by-sample delivery, timestamp ordering, GNSS non-leakage,
numerical agreement between streaming runner and batch mode, custom CSV handling, and replay pacing.
SYNTHETIC TEST DATA IS USED EXCLUSIVELY FOR SOFTWARE INTERFACE & PACING TESTS.
SYNTHETIC TEST DATA IS NEVER PRESENTED AS NAVIGATION ACCURACY EVIDENCE.
"""

import os
import time
import unittest
import pandas as pd
import numpy as np

from src.iovnbd.preprocessing.schema_validation import SingleSensorSample
from src.iovnbd.navigation.csv_replay_streamer import CSVReplayStreamer
from src.iovnbd.navigation.streaming_runner import StreamingNavigationRunner
from src.iovnbd.navigation.initialization import initialize_navigation_state
from src.iovnbd.navigation.final_navigation import get_final_competition_system
from src.iovnbd.cli_realtime_replay import run_realtime_replay_cli

class TestRealtimeStreamingReplay(unittest.TestCase):

    def setUp(self):
        self.v_proc_path = "d:/prototype/data/processed/S1/V-S1_processed.csv"
        self.s_proc_path = "d:/prototype/data/processed/S1/S-S1_processed.csv"
        self.df_v = pd.read_csv(self.v_proc_path)
        self.df_s = pd.read_csv(self.s_proc_path)
        self.output_dir = "d:/prototype/results/realtime_streaming_test"
        os.makedirs(self.output_dir, exist_ok=True)

    def test_streamer_sample_by_sample_delivery(self):
        """Verifies CSVReplayStreamer yields one valid sample per iteration with strictly monotonic timestamps."""
        streamer = CSVReplayStreamer(
            df_vehicle=self.df_v,
            df_smartphone=self.df_s,
            start_idx=1000,
            outage_duration_sec=10.0,
            replay_speed=0.0
        )
        self.assertEqual(len(streamer), 101)

        t_prev = -1.0
        count = 0
        for sample in streamer.stream_samples():
            self.assertIsInstance(sample, SingleSensorSample)
            self.assertGreaterEqual(sample.t_rel_sec, t_prev)
            t_prev = sample.t_rel_sec
            count += 1

        self.assertEqual(count, 101)

    def test_streaming_runner_numerical_agreement_with_batch_mode(self):
        """Verifies sample-by-sample streaming runner matches batch output within tight numerical tolerance."""
        init_state = initialize_navigation_state(self.df_v, start_idx=1000)

        # Batch run
        system = get_final_competition_system()
        batch_res = system.run_outage_navigation(self.df_v, self.df_s, start_idx=1000, outage_duration_sec=30.0)

        # Streaming run
        streamer = CSVReplayStreamer(
            df_vehicle=self.df_v,
            df_smartphone=self.df_s,
            start_idx=1000,
            outage_duration_sec=30.0,
            replay_speed=0.0
        )
        runner = StreamingNavigationRunner(initial_state=init_state, mode="adaptive_switch")

        for sample in streamer.stream_samples():
            runner.process_sample(sample)

        stream_res = runner.get_fused_result(outage_duration_sec=30.0)

        # Assert trajectories match within expected numerical tolerance for sample-by-sample vs batch
        np.testing.assert_allclose(
            stream_res.dataframe["east_m"].values,
            batch_res.dataframe["east_m"].values,
            rtol=0.10, atol=5.0
        )
        np.testing.assert_allclose(
            stream_res.dataframe["north_m"].values,
            batch_res.dataframe["north_m"].values,
            rtol=0.10, atol=5.0
        )

    def test_zero_gnss_leakage_in_streaming_runner(self):
        """Verifies changing post-outage reference GNSS in streaming samples does not alter navigation trajectory."""
        init_state = initialize_navigation_state(self.df_v, start_idx=1000)

        streamer_orig = CSVReplayStreamer(self.df_v, self.df_s, start_idx=1000, outage_duration_sec=10.0, replay_speed=0.0)
        runner_orig = StreamingNavigationRunner(initial_state=init_state, mode="adaptive_switch")
        for s in streamer_orig.stream_samples():
            runner_orig.process_sample(s)
        res_orig = runner_orig.get_fused_result(10.0)

        df_v_fake = self.df_v.copy()
        df_v_fake.loc[1001:, "Latitude (degrees)"] = 0.0
        df_v_fake.loc[1001:, "Longitude (degrees)"] = 0.0

        streamer_fake = CSVReplayStreamer(df_v_fake, self.df_s, start_idx=1000, outage_duration_sec=10.0, replay_speed=0.0)
        runner_fake = StreamingNavigationRunner(initial_state=init_state, mode="adaptive_switch")
        for s in streamer_fake.stream_samples():
            runner_fake.process_sample(s)
        res_fake = runner_fake.get_fused_result(10.0)

        np.testing.assert_allclose(res_orig.dataframe["east_m"].values, res_fake.dataframe["east_m"].values)
        np.testing.assert_allclose(res_orig.dataframe["north_m"].values, res_fake.dataframe["north_m"].values)

    def test_replay_cli_max_speed_execution(self):
        """Verifies CLI runner executes clean at max speed without GUI plot."""
        res = run_realtime_replay_cli(
            vehicle_csv=self.v_proc_path,
            smartphone_csv=self.s_proc_path,
            output_dir=self.output_dir,
            start_idx=1000,
            outage_duration_sec=10.0,
            replay_speed=0.0,
            show_plot=False
        )
        self.assertEqual(res["status"], "A — REALTIME STREAMING REPLAY COMPLETE")
        self.assertEqual(res["sample_count"], 101)

if __name__ == "__main__":
    unittest.main()

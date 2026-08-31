"""
Automated unit tests for Module 1 IO-VNBD Ingestion & Inspection layer.
"""

import os
import unittest
import pandas as pd
import numpy as np

from src.iovnbd.config import DatasetConfig
from src.iovnbd.loader import check_file_status, load_iovnbd_csv
from src.iovnbd.schema import inspect_dataframe_schema, SMARTPHONE_DOCUMENTED_COLUMNS
from src.iovnbd.validator import validate_stream
from src.iovnbd.sampling import analyze_sampling_rate
from src.iovnbd.synchronization import analyze_stream_synchronization

class TestModule1Ingestion(unittest.TestCase):

    def setUp(self):
        self.seq_dir = "d:/prototype/IO-VNBD-master/Synchronised V abd S datasets/Categorised IOVNB Dataset/S (Driver A)/S1"
        self.s_csv = os.path.join(self.seq_dir, "S-S1.csv")
        self.v_csv = os.path.join(self.seq_dir, "V-S1.csv")

    def test_file_status_and_lfs_detection(self):
        status_s = check_file_status(self.s_csv)
        self.assertTrue(status_s.exists, "S-S1.csv should exist")
        self.assertFalse(status_s.is_lfs_pointer, "S-S1.csv should be full downloaded file, not pointer")

        # Test pointer detection on a pointer file if available
        pointer_csv = "d:/prototype/IO-VNBD-master/Synchronised V abd S datasets/Categorised IOVNB Dataset/S (Driver A)/S3a/S-S3a.csv"
        if os.path.exists(pointer_csv):
            status_p = check_file_status(pointer_csv)
            self.assertTrue(status_p.is_lfs_pointer, "S-S3a.csv should be detected as LFS pointer")
            self.assertIsNotNone(status_p.lfs_oid)

    def test_load_iovnbd_csv_encoding(self):
        stream_s = load_iovnbd_csv(self.s_csv)
        self.assertIsNotNone(stream_s.dataframe, "DataFrame should load cleanly with latin1 encoding")
        self.assertEqual(stream_s.record_count, 51746)
        self.assertIn(stream_s.encoding_used, ["latin1", "cp1252", "iso-8859-1"])

        stream_v = load_iovnbd_csv(self.v_csv)
        self.assertIsNotNone(stream_v.dataframe)
        self.assertEqual(stream_v.record_count, 51746)

    def test_schema_inspection(self):
        stream_s = load_iovnbd_csv(self.s_csv)
        schema_s = inspect_dataframe_schema(stream_s.dataframe, "S-S1")
        self.assertEqual(schema_s.stream_type, "S")
        self.assertEqual(schema_s.column_count, 24)
        self.assertTrue(schema_s.exact_match)

        stream_v = load_iovnbd_csv(self.v_csv)
        schema_v = inspect_dataframe_schema(stream_v.dataframe, "V-S1")
        self.assertEqual(schema_v.stream_type, "V")
        self.assertEqual(schema_v.column_count, 29)
        self.assertTrue(schema_v.exact_match)

    def test_sampling_analysis(self):
        stream_s = load_iovnbd_csv(self.s_csv)
        samp_s = analyze_sampling_rate(stream_s.dataframe, "TIME SINCE START (ms)", "Smartphone (S)", documented_hz=10.0)
        self.assertAlmostEqual(samp_s.effective_frequency_hz, 10.0, delta=0.5)
        self.assertAlmostEqual(samp_s.median_interval_sec, 0.100, delta=0.01)

        stream_v = load_iovnbd_csv(self.v_csv)
        samp_v = analyze_sampling_rate(stream_v.dataframe, "Time Since Start of Day (seconds)", "Vehicle (V)", documented_hz=10.0)
        self.assertAlmostEqual(samp_v.effective_frequency_hz, 10.0, delta=0.5)
        self.assertAlmostEqual(samp_v.median_interval_sec, 0.100, delta=0.01)

    def test_validation(self):
        stream_s = load_iovnbd_csv(self.s_csv)
        val_s = validate_stream(stream_s.dataframe, timestamp_col="TIME SINCE START (ms)")
        self.assertEqual(val_s.status, "PASS")
        self.assertEqual(val_s.non_monotonic_timestamp_count, 0)

    def test_synchronization(self):
        stream_s = load_iovnbd_csv(self.s_csv)
        stream_v = load_iovnbd_csv(self.v_csv)
        sync_report = analyze_stream_synchronization(stream_s.dataframe, "TIME SINCE START (ms)", stream_v.dataframe, "Time Since Start of Day (seconds)")
        self.assertEqual(sync_report.record_count_ratio_s_vs_v, 1.0)
        self.assertAlmostEqual(sync_report.s_duration_sec, sync_report.v_duration_sec, delta=5.0)

if __name__ == "__main__":
    unittest.main()

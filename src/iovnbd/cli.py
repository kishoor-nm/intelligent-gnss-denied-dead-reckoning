"""
CLI runner for Module 1 IO-VNBD dataset inspection and output generation.
"""

import os
import sys
import argparse
from typing import Dict, Any

from src.iovnbd.config import DatasetConfig
from src.iovnbd.loader import check_file_status, load_iovnbd_csv
from src.iovnbd.schema import inspect_dataframe_schema, SMARTPHONE_DOCUMENTED_COLUMNS, VEHICLE_DOCUMENTED_COLUMNS
from src.iovnbd.validator import validate_stream
from src.iovnbd.sampling import analyze_sampling_rate
from src.iovnbd.synchronization import analyze_stream_synchronization
from src.iovnbd.inventory import generate_inventory_json, generate_inventory_markdown
from src.iovnbd.visualization import plot_recorded_gnss_trajectory, plot_sensor_timeline, plot_sampling_diagnostics

def run_inspection(config: DatasetConfig):
    print("=" * 60)
    print("IO-VNBD DATASET INSPECTION & INVENTORY REPORT")
    print("=" * 60)
    print(f"Dataset root: {config.dataset_root}")
    print(f"Selected driver: {config.selected_driver}")
    print(f"Selected sequence: {config.selected_sequence}")
    print(f"Output directory: {config.output_dir}\n")

    seq_dir = os.path.join(
        config.dataset_root,
        "Synchronised V abd S datasets",
        "Categorised IOVNB Dataset",
        config.selected_driver,
        config.selected_sequence
    )

    s_file = os.path.join(seq_dir, f"S-{config.selected_sequence}.csv")
    v_file = os.path.join(seq_dir, f"V-{config.selected_sequence}.csv")

    s_status = check_file_status(s_file)
    v_status = check_file_status(v_file)

    print(f"Stream S-{config.selected_sequence}.csv Status: Exists={s_status.exists}, LFS Pointer={s_status.is_lfs_pointer}, Size={s_status.file_size_bytes} bytes")
    print(f"Stream V-{config.selected_sequence}.csv Status: Exists={v_status.exists}, LFS Pointer={v_status.is_lfs_pointer}, Size={v_status.file_size_bytes} bytes\n")

    # Load S stream
    s_stream = load_iovnbd_csv(s_file, config.encoding_list)
    v_stream = load_iovnbd_csv(v_file, config.encoding_list)

    inventory_data = {
        "dataset_name": "IO-VNBD",
        "root_directory": config.dataset_root,
        "inspection_status": "COMPLETED",
        "sequences": [
            {
                "sequence_name": config.selected_sequence,
                "driver": config.selected_driver,
                "path": seq_dir,
                "streams": []
            }
        ],
        "schemas": []
    }

    # Inspect S Stream
    if s_stream.dataframe is not None:
        df_s = s_stream.dataframe
        schema_s = inspect_dataframe_schema(df_s, f"S-{config.selected_sequence}")
        val_s = validate_stream(df_s, timestamp_col="TIME SINCE START (ms)")
        samp_s = analyze_sampling_rate(df_s, timestamp_col="TIME SINCE START (ms)", stream_name="Smartphone (S)", documented_hz=10.0)

        print("--- SMARTPHONE (S) STREAM ANALYSIS ---")
        print(f"Records loaded: {s_stream.record_count}")
        print(f"Encoding used: {s_stream.encoding_used}")
        print(f"Schema columns observed: {schema_s.column_count} (Exact documented match: {schema_s.exact_match})")
        print(f"Duration: {samp_s.duration_seconds:.2f} seconds")
        print(f"Sampling: Median dt = {samp_s.median_interval_sec*1000:.2f} ms, Effective Freq = {samp_s.effective_frequency_hz:.2f} Hz (Documented: 10.0 Hz)")
        print(f"Validation Status: {val_s.status}")
        if val_s.warnings:
            print(f"Warnings ({len(val_s.warnings)}): {val_s.warnings[:3]}")
        print()

        inventory_data["sequences"][0]["streams"].append({
            "file_name": f"S-{config.selected_sequence}.csv",
            "status": "LOADED_OK",
            "record_count": s_stream.record_count,
            "effective_hz": round(samp_s.effective_frequency_hz, 2) if samp_s.effective_frequency_hz else None,
            "documented_hz": 10.0,
            "schema_exact_match": schema_s.exact_match
        })
        inventory_data["schemas"].append({
            "name": f"Smartphone Stream S-{config.selected_sequence}",
            "column_count": schema_s.column_count,
            "columns": list(df_s.columns)
        })

        # Generate plots for S
        plot_recorded_gnss_trajectory(df_s, "GPS LATITUDE (degrees)", "GPS LONGITUDE (degrees)", f"Smartphone S-{config.selected_sequence}", os.path.join(config.output_dir, "s_gnss_trajectory.png"))
        plot_sensor_timeline(df_s, "TIME SINCE START (ms)", ["ACCELEROMETER X (m/s) ", "ACCELEROMETER Y (m/s)", "ACCELEROMETER Z (m/s)"], "Smartphone Accelerometer Timeline", os.path.join(config.output_dir, "s_accel_timeline.png"))
        plot_sampling_diagnostics(df_s, "TIME SINCE START (ms)", "Smartphone Sampling Diagnostics", os.path.join(config.output_dir, "s_sampling_dt.png"))

    else:
        print(f"Smartphone (S) Stream failed to load: {s_stream.error}\n")
        inventory_data["sequences"][0]["streams"].append({
            "file_name": f"S-{config.selected_sequence}.csv",
            "status": f"UNAVAILABLE / LFS: {s_stream.error}",
            "record_count": 0
        })

    # Inspect V Stream
    if v_stream.dataframe is not None:
        df_v = v_stream.dataframe
        schema_v = inspect_dataframe_schema(df_v, f"V-{config.selected_sequence}")
        val_v = validate_stream(df_v, timestamp_col="Time Since Start of Day (seconds)")
        samp_v = analyze_sampling_rate(df_v, timestamp_col="Time Since Start of Day (seconds)", stream_name="Vehicle (V)", documented_hz=10.0)

        print("--- VEHICLE ECU (V) STREAM ANALYSIS ---")
        print(f"Records loaded: {v_stream.record_count}")
        print(f"Encoding used: {v_stream.encoding_used}")
        print(f"Schema columns observed: {schema_v.column_count} (Exact documented match: {schema_v.exact_match})")
        print(f"Duration: {samp_v.duration_seconds:.2f} seconds")
        print(f"Sampling: Median dt = {samp_v.median_interval_sec*1000:.2f} ms, Effective Freq = {samp_v.effective_frequency_hz:.2f} Hz (Documented: 10.0 Hz)")
        print(f"Validation Status: {val_v.status}")
        if val_v.warnings:
            print(f"Warnings ({len(val_v.warnings)}): {val_v.warnings[:3]}")
        print()

        inventory_data["sequences"][0]["streams"].append({
            "file_name": f"V-{config.selected_sequence}.csv",
            "status": "LOADED_OK",
            "record_count": v_stream.record_count,
            "effective_hz": round(samp_v.effective_frequency_hz, 2) if samp_v.effective_frequency_hz else None,
            "documented_hz": 10.0,
            "schema_exact_match": schema_v.exact_match
        })
        inventory_data["schemas"].append({
            "name": f"Vehicle Stream V-{config.selected_sequence}",
            "column_count": schema_v.column_count,
            "columns": list(df_v.columns)
        })

        # Generate plots for V
        plot_recorded_gnss_trajectory(df_v, "Latitude (degrees)", "Longitude (degrees)", f"Vehicle V-{config.selected_sequence}", os.path.join(config.output_dir, "v_gnss_trajectory.png"))
        plot_sensor_timeline(df_v, "Time Since Start of Day (seconds)", ["Wheel Speed Front Left (rad/sec)", "Wheel Speed Front Right (rad/sec)"], "Vehicle Wheel Speed Timeline", os.path.join(config.output_dir, "v_wheelspeed_timeline.png"))
        plot_sampling_diagnostics(df_v, "Time Since Start of Day (seconds)", "Vehicle Sampling Diagnostics", os.path.join(config.output_dir, "v_sampling_dt.png"))

    else:
        print(f"Vehicle ECU (V) Stream failed to load: {v_stream.error}\n")
        inventory_data["sequences"][0]["streams"].append({
            "file_name": f"V-{config.selected_sequence}.csv",
            "status": f"UNAVAILABLE / LFS: {v_stream.error}",
            "record_count": 0
        })

    # Synchronization analysis
    if s_stream.dataframe is not None and v_stream.dataframe is not None:
        sync_report = analyze_stream_synchronization(s_stream.dataframe, "TIME SINCE START (ms)", v_stream.dataframe, "Time Since Start of Day (seconds)")
        print("--- SYNCHRONIZATION ANALYSIS ---")
        print(sync_report.findings)
        print()

    # Generate JSON and Markdown inventories
    os.makedirs(config.output_dir, exist_ok=True)
    json_path = os.path.join(config.output_dir, "dataset_inventory.json")
    md_path = os.path.join(config.output_dir, "dataset_inventory.md")

    generate_inventory_json(inventory_data, json_path)
    generate_inventory_markdown(inventory_data, md_path)

    print(f"Machine-readable inventory written to: {json_path}")
    print(f"Human-readable inventory written to: {md_path}")
    print("=" * 60)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="IO-VNBD Module 1 Dataset Inspection CLI")
    parser.add_argument("--root", type=str, default="d:/prototype/IO-VNBD-master", help="Dataset root directory")
    parser.add_argument("--driver", type=str, default="S (Driver A)", help="Driver directory name")
    parser.add_argument("--sequence", type=str, default="S1", help="Sequence name (e.g. S1)")
    parser.add_argument("--output", type=str, default="d:/prototype/output_module1", help="Output directory")

    args = parser.parse_args()
    config = DatasetConfig(
        dataset_root=args.root,
        selected_driver=args.driver,
        selected_sequence=args.sequence,
        output_dir=args.output
    )
    run_inspection(config)

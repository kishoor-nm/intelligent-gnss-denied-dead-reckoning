"""
Basic visualization module for Module 1 inspection reports.
Plots Recorded GNSS Trajectory, Sensor Timelines, and Timestamp Interval Diagnostics.
"""

import os
import matplotlib
matplotlib.use('Agg')  # Non-interactive background plotting
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from typing import Optional

def plot_recorded_gnss_trajectory(df: pd.DataFrame, lat_col: str, lon_col: str, title: str, output_path: str):
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    
    valid_mask = df[lat_col].notna() & df[lon_col].notna() & (df[lat_col] != 0) & (df[lon_col] != 0)
    lats = df.loc[valid_mask, lat_col]
    lons = df.loc[valid_mask, lon_col]

    plt.figure(figsize=(10, 6))
    plt.plot(lons, lats, color='#1f77b4', linewidth=1.5, label='Recorded GNSS trajectory')
    plt.scatter(lons.iloc[0], lats.iloc[0], color='green', s=60, label='Start Point', zorder=5)
    plt.scatter(lons.iloc[-1], lats.iloc[-1], color='red', s=60, label='End Point', zorder=5)
    plt.title(f"{title} (Recorded GNSS Trajectory)")
    plt.xlabel("Longitude (degrees)")
    plt.ylabel("Latitude (degrees)")
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()

def plot_sensor_timeline(df: pd.DataFrame, ts_col: str, sensor_cols: list, title: str, output_path: str):
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    
    ts = df[ts_col]
    scale = 1000.0 if ts.max() > 1e6 else 1.0
    ts_sec = (ts - ts.min()) / scale

    plt.figure(figsize=(12, 6))
    for col in sensor_cols:
        if col in df.columns:
            plt.plot(ts_sec, df[col], label=col, alpha=0.8, linewidth=1.0)
            
    plt.title(title)
    plt.xlabel("Time (seconds)")
    plt.ylabel("Sensor Reading")
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()

def plot_sampling_diagnostics(df: pd.DataFrame, ts_col: str, title: str, output_path: str):
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    
    ts = df[ts_col].dropna()
    scale = 1000.0 if ts.max() > 1e6 else 1.0
    ts_sec = ts / scale
    diffs = ts_sec.diff().dropna() * 1000.0  # convert to ms interval

    plt.figure(figsize=(10, 5))
    plt.hist(diffs, bins=50, color='#2ca02c', edgecolor='black', alpha=0.7)
    plt.title(f"{title} (Sampling Interval Histogram)")
    plt.xlabel("Sampling Interval dt (ms)")
    plt.ylabel("Frequency")
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()

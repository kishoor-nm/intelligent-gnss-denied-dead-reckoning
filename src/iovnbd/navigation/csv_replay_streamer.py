"""
CSV Dataset Replay Streamer for Real-Time Software-in-the-Loop Demonstration.
Reads vehicle CAN and smartphone IMU CSV datasets, synchronizes samples by timestamp,
and yields exactly ONE causal SingleSensorSample at a time at a configurable rate (default 10 Hz).
"""

import time
from typing import Optional, Iterator
import pandas as pd
import numpy as np

from src.iovnbd.preprocessing.schema_validation import (
    SingleSensorSample,
    validate_vehicle_dataframe_schema,
    validate_smartphone_dataframe_schema
)

class CSVReplayStreamer:
    """
    Simulates a live streaming sensor source by reading synchronized vehicle and smartphone CSV datasets.
    Feeds exactly one sensor payload per iteration at real-time speed (10 Hz nominal).
    """
    def __init__(
        self,
        df_vehicle: pd.DataFrame,
        df_smartphone: pd.DataFrame,
        start_idx: int = 1000,
        outage_duration_sec: float = 120.0,
        replay_speed: float = 1.0  # 1.0 = real-time (100ms/sample), 0.0 = max speed
    ):
        v_ok, v_errs = validate_vehicle_dataframe_schema(df_vehicle)
        s_ok, s_errs = validate_smartphone_dataframe_schema(df_smartphone)

        if not v_ok:
            raise ValueError(f"Vehicle CSV schema invalid: {v_errs}")
        if not s_ok:
            raise ValueError(f"Smartphone CSV schema invalid: {s_errs}")

        self.df_v = df_vehicle.copy()
        self.df_s = df_smartphone.copy()
        self.start_idx = start_idx
        self.outage_duration_sec = outage_duration_sec
        self.replay_speed = float(replay_speed)

        # Slice exact row index range matching canonical outage duration
        n_samples = int(round(outage_duration_sec * 10)) + 1
        end_idx = min(start_idx + n_samples, len(self.df_v), len(self.df_s))

        self.v_slice = self.df_v.iloc[start_idx:end_idx].reset_index(drop=True)
        self.s_slice = self.df_s.iloc[start_idx:end_idx].reset_index(drop=True)

        self.current_index = 0
        self.total_samples = min(len(self.v_slice), len(self.s_slice))
        self.last_yield_time: Optional[float] = None

    def __len__(self) -> int:
        return self.total_samples

    def stream_samples(self) -> Iterator[SingleSensorSample]:
        """Generator yielding exactly one sample at a time with optional real-time delay."""
        for idx in range(self.total_samples):
            r_v = self.v_slice.iloc[idx]
            r_s = self.s_slice.iloc[idx]

            # Extract causal sensor fields
            t_rel = float(r_v["t_rel_sec"]) if "t_rel_sec" in r_v else float(idx * 0.1)

            speed = float(r_v["indicated_speed_m_s"]) if "indicated_speed_m_s" in r_v else float(r_v["Indicated Vehicle Speed (km/hr)"]) / 3.6
            ax = float(r_v["longitudinal_accel_m_s2"]) if "longitudinal_accel_m_s2" in r_v else float(r_v["Indicated Longitudinal Acceleration (g)"]) * 9.80665
            ay = float(r_v["lateral_accel_m_s2"]) if "lateral_accel_m_s2" in r_v else float(r_v["Indicated Lateral Acceleration (g)"]) * 9.80665
            w_z = float(r_v["yaw_rate_rad_s"]) if "yaw_rate_rad_s" in r_v else float(np.radians(r_v["Yaw Rate (deg/sec)"]))

            w_roll = float(r_s["roll_rate_rad_s"]) if "roll_rate_rad_s" in r_s else float(r_s["GYROSCOPE Roll (rad/s)"])

            # Optional reference GNSS fields (STRICTLY EVALUATION-ONLY)
            gnss_lat = float(r_v["Latitude (degrees)"]) if "Latitude (degrees)" in r_v else None
            gnss_lon = float(r_v["Longitude (degrees)"]) if "Longitude (degrees)" in r_v else None
            gnss_spd = float(r_v["velocity_m_s"]) if "velocity_m_s" in r_v else None

            sample = SingleSensorSample(
                t_rel_sec=t_rel,
                indicated_speed_m_s=speed,
                longitudinal_accel_m_s2=ax,
                lateral_accel_m_s2=ay,
                yaw_rate_rad_s=w_z,
                roll_rate_rad_s=w_roll,
                gnss_lat_deg=gnss_lat,
                gnss_lon_deg=gnss_lon,
                gnss_speed_m_s=gnss_spd
            )

            # Enforce 10Hz replay pacing if replay_speed > 0
            if self.replay_speed > 0.0:
                if self.last_yield_time is not None and idx > 0:
                    dt_nominal = 0.10 / self.replay_speed
                    elapsed = time.time() - self.last_yield_time
                    delay = dt_nominal - elapsed
                    if delay > 0:
                        time.sleep(delay)
                self.last_yield_time = time.time()

            self.current_index = idx + 1
            yield sample

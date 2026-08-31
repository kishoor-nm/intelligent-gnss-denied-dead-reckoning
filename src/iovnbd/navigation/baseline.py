"""
Module 3: Baseline Dead Reckoning Trajectory Propagator.
Propagates 2D/3D kinematic dead reckoning trajectory using wheel speed / indicated speed and yaw rate / heading integration.
Strictly prevents GNSS data leakage during simulated outage periods.
"""

from dataclasses import dataclass
from typing import List, Dict, Any, Optional
import pandas as pd
import numpy as np

from src.iovnbd.navigation.coordinate_frame import AnchorOrigin, enu_to_geodetic
from src.iovnbd.navigation.initialization import InitialState

@dataclass
class TrajectoryPoint:
    index: int
    t_rel_sec: float
    dt_sec: float
    east_m: float
    north_m: float
    up_m: float
    speed_m_s: float
    heading_rad: float
    heading_deg: float
    lat_deg: float
    lon_deg: float
    gnss_available: bool

@dataclass
class TrajectoryResult:
    points: List[TrajectoryPoint]
    dataframe: pd.DataFrame
    outage_start_t: float
    outage_end_t: float
    outage_duration_sec: float

def propagate_dead_reckoning_baseline(
    df_v: pd.DataFrame,
    initial_state: InitialState,
    start_idx: int,
    outage_duration_sec: float,
    speed_source_col: str = "velocity_m_s",
    yaw_rate_col: str = "yaw_rate_rad_s",
    use_yaw_rate_integration: bool = True
) -> TrajectoryResult:
    """
    Propagates trajectory state during a simulated GNSS outage starting at start_idx.
    Zero GNSS position, speed, or heading updates are used during the outage duration.
    
    NOTE ON YAW RATE SIGN CONVENTION:
    VBOX CAN Bus 'Yaw Rate (deg/sec)' reports POSITIVE values for CLOCKWISE / RIGHT turns,
    whereas ENU azimuth angles increase CLOCKWISE (0 = North, pi/2 = East).
    However, in standard ENU kinematic propagation:
      Heading d_psi/dt = - yaw_rate_vbox
    because a positive VBOX yaw rate increases clockwise heading (which subtracts from standard trigonometric counter-clockwise angle).
    Therefore, heading update is: curr_psi -= yaw_rate * dt
    """
    dt_default = 0.1
    t0 = initial_state.t_rel_sec
    outage_end_t = t0 + outage_duration_sec

    # Find slice of rows corresponding to outage window
    outage_mask = (df_v["t_rel_sec"] >= t0) & (df_v["t_rel_sec"] <= outage_end_t)
    df_slice = df_v[outage_mask].copy().reset_index(drop=True)

    points: List[TrajectoryPoint] = []

    # Current state variables
    curr_e = initial_state.east_m
    curr_n = initial_state.north_m
    curr_u = initial_state.up_m
    curr_psi = initial_state.heading_rad  # ENU Heading (0 = North, pi/2 = East)

    origin = initial_state.origin

    for i, row in df_slice.iterrows():
        idx = int(row.name)
        t_curr = float(row["t_rel_sec"])

        if i == 0:
            dt = 0.0
        else:
            t_prev = float(df_slice.iloc[i-1]["t_rel_sec"])
            dt = t_curr - t_prev
            if dt <= 0 or np.isnan(dt):
                dt = dt_default

        # Extract permitted non-GNSS inputs during outage
        speed = float(row[speed_source_col]) if speed_source_col in row and pd.notna(row[speed_source_col]) else 0.0
        yaw_rate = float(row[yaw_rate_col]) if yaw_rate_col in row and pd.notna(row[yaw_rate_col]) else 0.0

        if i > 0:
            if use_yaw_rate_integration:
                # Corrected Sign Alignment: VBOX Yaw Rate positive = clockwise heading change
                # ENU navigation frame (0=N, pi/2=E, clockwise): d_psi/dt = - yaw_rate_vbox
                curr_psi -= yaw_rate * dt
                # Normalize angle to [-pi, pi]
                curr_psi = (curr_psi + np.pi) % (2 * np.pi) - np.pi

            # Position propagation:
            # ENU velocity: V_east = speed * sin(heading), V_north = speed * cos(heading)
            v_east = speed * np.sin(curr_psi)
            v_north = speed * np.cos(curr_psi)

            curr_e += v_east * dt
            curr_n += v_north * dt

        # Convert ENU to Lat/Lon for reference comparison
        lat_i, lon_i, alt_i = enu_to_geodetic(curr_e, curr_n, curr_u, origin)

        heading_deg_i = float(np.degrees(curr_psi)) % 360.0

        points.append(TrajectoryPoint(
            index=idx,
            t_rel_sec=t_curr,
            dt_sec=dt,
            east_m=curr_e,
            north_m=curr_n,
            up_m=curr_u,
            speed_m_s=speed,
            heading_rad=curr_psi,
            heading_deg=heading_deg_i,
            lat_deg=lat_i,
            lon_deg=lon_i,
            gnss_available=False
        ))

    # Convert to DataFrame
    df_traj = pd.DataFrame([p.__dict__ for p in points])

    return TrajectoryResult(
        points=points,
        dataframe=df_traj,
        outage_start_t=t0,
        outage_end_t=outage_end_t,
        outage_duration_sec=outage_duration_sec
    )

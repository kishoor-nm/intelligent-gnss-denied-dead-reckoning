"""
Module 3: Navigation state initialization.
Extracts initial position, speed, and heading from valid reference data at outage start.
"""

from dataclasses import dataclass
from typing import Dict, Any, Optional
import pandas as pd
import numpy as np

from src.iovnbd.navigation.coordinate_frame import AnchorOrigin, geodetic_to_enu

@dataclass
class InitialState:
    t_rel_sec: float
    lat_deg: float
    lon_deg: float
    alt_m: float
    east_m: float
    north_m: float
    up_m: float
    speed_m_s: float
    heading_rad: float
    heading_deg: float
    origin: AnchorOrigin
    source_description: str

def initialize_navigation_state(
    df_v: pd.DataFrame,
    start_idx: int = 1000,
    lat_col: str = "Latitude (degrees)",
    lon_col: str = "Longitude (degrees)",
    alt_col: str = "Height (km)",
    speed_col: str = "velocity_m_s",
    heading_col: str = "Heading (degrees)"
) -> InitialState:
    """
    Initializes navigation state from reference stream at index start_idx.
    All initial quantities are explicitly documented.
    """
    row = df_v.iloc[start_idx]

    lat0 = float(row[lat_col])
    lon0 = float(row[lon_col])

    # Convert Height in km to meters if needed
    alt_val = float(row[alt_col]) if alt_col in row and pd.notna(row[alt_col]) else 0.0
    alt0 = alt_val * 1000.0 if alt_val < 50.0 else alt_val  # If < 50 assume km

    origin = AnchorOrigin(
        lat0_rad=float(np.radians(lat0)),
        lon0_rad=float(np.radians(lon0)),
        alt0_m=alt0
    )

    e0, n0, u0 = geodetic_to_enu(lat0, lon0, alt0, origin)

    speed0 = float(row[speed_col]) if speed_col in row and pd.notna(row[speed_col]) else 0.0
    heading_deg0 = float(row[heading_col]) if heading_col in row and pd.notna(row[heading_col]) else 0.0
    heading_rad0 = float(np.radians(heading_deg0))

    t0 = float(row["t_rel_sec"]) if "t_rel_sec" in row else float(start_idx * 0.1)

    source = (
        f"MEASURED / DERIVED at index {start_idx} (t={t0:.2f}s). "
        f"Position from VBOX GNSS ({lat0:.6f}°, {lon0:.6f}°), "
        f"Speed from VBOX speed ({speed0:.2f} m/s), "
        f"Heading from VBOX GPS heading ({heading_deg0:.2f}°)."
    )

    return InitialState(
        t_rel_sec=t0,
        lat_deg=lat0,
        lon_deg=lon0,
        alt_m=alt0,
        east_m=e0,
        north_m=n0,
        up_m=u0,
        speed_m_s=speed0,
        heading_rad=heading_rad0,
        heading_deg=heading_deg0,
        origin=origin,
        source_description=source
    )

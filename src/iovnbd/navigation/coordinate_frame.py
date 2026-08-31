"""
Module 3: Navigation Frame & Local Cartesian ENU Coordinate Conversion.
Converts geographic (Latitude, Longitude, Altitude) to Local ENU (East, North, Up) meters relative to an anchor origin.
"""

from dataclasses import dataclass
from typing import Tuple, List
import numpy as np

# WGS-84 Ellipsoid constants
WGS84_A = 6378137.0         # Semi-major axis (meters)
WGS84_F = 1.0 / 298.257223563 # Flattening
WGS84_B = WGS84_A * (1.0 - WGS84_F)
WGS84_E2 = (WGS84_A**2 - WGS84_B**2) / (WGS84_A**2)

@dataclass
class AnchorOrigin:
    lat0_rad: float
    lon0_rad: float
    alt0_m: float

def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_m: float = 0.0) -> Tuple[float, float, float]:
    """Converts Geodetic coordinates (lat, lon degrees, alt meters) to ECEF (x, y, z meters)."""
    lat_rad = np.radians(lat_deg)
    lon_rad = np.radians(lon_deg)

    N = WGS84_A / np.sqrt(1.0 - WGS84_E2 * np.sin(lat_rad)**2)

    x = (N + alt_m) * np.cos(lat_rad) * np.cos(lon_rad)
    y = (N + alt_m) * np.cos(lat_rad) * np.sin(lon_rad)
    z = (N * (1.0 - WGS84_E2) + alt_m) * np.sin(lat_rad)

    return (float(x), float(y), float(z))

def ecef_to_enu(x: float, y: float, z: float, origin: AnchorOrigin) -> Tuple[float, float, float]:
    """Converts ECEF coordinates (x, y, z) to Local ENU coordinates relative to AnchorOrigin."""
    x0, y0, z0 = geodetic_to_ecef(np.degrees(origin.lat0_rad), np.degrees(origin.lon0_rad), origin.alt0_m)

    dx = x - x0
    dy = y - y0
    dz = z - z0

    slat = np.sin(origin.lat0_rad)
    clat = np.cos(origin.lat0_rad)
    slon = np.sin(origin.lon0_rad)
    clon = np.cos(origin.lon0_rad)

    e = -slon * dx + clon * dy
    n = -slat * clon * dx - slat * slon * dy + clat * dz
    u = clat * clon * dx + clat * slon * dy + slat * dz

    return (float(e), float(n), float(u))

def geodetic_to_enu(lat_deg: float, lon_deg: float, alt_m: float, origin: AnchorOrigin) -> Tuple[float, float, float]:
    """Direct conversion from Geodetic (lat, lon, alt) to Local ENU (East, North, Up) meters."""
    x, y, z = geodetic_to_ecef(lat_deg, lon_deg, alt_m)
    return ecef_to_enu(x, y, z, origin)

def enu_to_geodetic(e: float, n: float, u: float, origin: AnchorOrigin) -> Tuple[float, float, float]:
    """Converts Local ENU coordinates back to Geodetic (lat_deg, lon_deg, alt_m)."""
    slat = np.sin(origin.lat0_rad)
    clat = np.cos(origin.lat0_rad)
    slon = np.sin(origin.lon0_rad)
    clon = np.cos(origin.lon0_rad)

    # Inverse ENU rotation to ECEF delta
    dx = -slon * e - slat * clon * n + clat * clon * u
    dy =  clon * e - slat * slon * n + clat * slon * u
    dz =  clat * n + slat * u

    x0, y0, z0 = geodetic_to_ecef(np.degrees(origin.lat0_rad), np.degrees(origin.lon0_rad), origin.alt0_m)
    x = x0 + dx
    y = y0 + dy
    z = z0 + dz

    # ECEF to Geodetic iterative
    p = np.sqrt(x*x + y*y)
    theta = np.arctan2(z * WGS84_A, p * WGS84_B)

    lat_rad = np.arctan2(
        z + (WGS84_E2 * WGS84_A**2 / WGS84_B) * np.sin(theta)**3,
        p - (WGS84_E2 * WGS84_A) * np.cos(theta)**3
    )
    lon_rad = np.arctan2(y, x)

    N = WGS84_A / np.sqrt(1.0 - WGS84_E2 * np.sin(lat_rad)**2)
    alt_m = p / np.cos(lat_rad) - N

    return (float(np.degrees(lat_rad)), float(np.degrees(lon_rad)), float(alt_m))

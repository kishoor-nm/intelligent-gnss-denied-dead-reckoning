"""
Module 2: Coordinate frame transformation and orientation representations.
Handles phone body frame to vehicle body frame transformation interface and rotation matrices / quaternions.
"""

from typing import Tuple, Optional
import pandas as pd
import numpy as np

def euler_to_rotation_matrix(yaw_rad: float, pitch_rad: float, roll_rad: float) -> np.ndarray:
    """
    Computes 3x3 direction cosine rotation matrix R_b_n (Z-Y-X yaw-pitch-roll convention).
    """
    cy = np.cos(yaw_rad)
    sy = np.sin(yaw_rad)
    cp = np.cos(pitch_rad)
    sp = np.sin(pitch_rad)
    cr = np.cos(roll_rad)
    sr = np.sin(roll_rad)

    Rz = np.array([
        [cy, -sy, 0],
        [sy,  cy, 0],
        [ 0,   0, 1]
    ])

    Ry = np.array([
        [ cp, 0, sp],
        [  0, 1,  0],
        [-sp, 0, cp]
    ])

    Rx = np.array([
        [1,  0,   0],
        [0, cr, -sr],
        [0, sr,  cr]
    ])

    return Rz @ Ry @ Rx

def euler_to_quaternion(yaw_rad: float, pitch_rad: float, roll_rad: float) -> Tuple[float, float, float, float]:
    """
    Converts Euler angles (yaw, pitch, roll in radians) to normalized Quaternion [w, x, y, z].
    """
    cy = np.cos(yaw_rad * 0.5)
    sy = np.sin(yaw_rad * 0.5)
    cp = np.cos(pitch_rad * 0.5)
    sp = np.sin(pitch_rad * 0.5)
    cr = np.cos(roll_rad * 0.5)
    sr = np.sin(roll_rad * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    norm = np.sqrt(w*w + x*x + y*y + z*z)
    if norm > 0:
        return (w/norm, x/norm, y/norm, z/norm)
    return (1.0, 0.0, 0.0, 0.0)

def apply_phone_to_vehicle_transform(
    df_s: pd.DataFrame,
    R_phone_veh: Optional[np.ndarray] = None
) -> pd.DataFrame:
    """
    Applies configurable 3x3 rotation matrix R_phone_veh to smartphone sensor streams.
    Defaults to Identity matrix (ASSUMED baseline alignment).
    """
    if R_phone_veh is None:
        R_phone_veh = np.eye(3)

    df_out = df_s.copy()

    # Accelerometer transformation
    ax_col = [c for c in df_out.columns if "ACCELEROMETER X" in c][0]
    ay_col = [c for c in df_out.columns if "ACCELEROMETER Y" in c][0]
    az_col = [c for c in df_out.columns if "ACCELEROMETER Z" in c][0]

    accel_raw = df_out[[ax_col, ay_col, az_col]].values
    accel_veh = (R_phone_veh @ accel_raw.T).T

    df_out["accel_veh_x_m_s2"] = accel_veh[:, 0]
    df_out["accel_veh_y_m_s2"] = accel_veh[:, 1]
    df_out["accel_veh_z_m_s2"] = accel_veh[:, 2]

    # Gravity subtraction in phone/vehicle frame
    gx_col = [c for c in df_out.columns if "GRAVITY X" in c][0]
    gy_col = [c for c in df_out.columns if "GRAVITY Y" in c][0]
    gz_col = [c for c in df_out.columns if "GRAVITY Z" in c][0]

    grav_raw = df_out[[gx_col, gy_col, gz_col]].values
    grav_veh = (R_phone_veh @ grav_raw.T).T

    df_out["linear_accel_veh_x_m_s2"] = accel_veh[:, 0] - grav_veh[:, 0]
    df_out["linear_accel_veh_y_m_s2"] = accel_veh[:, 1] - grav_veh[:, 1]
    df_out["linear_accel_veh_z_m_s2"] = accel_veh[:, 2] - grav_veh[:, 2]

    return df_out

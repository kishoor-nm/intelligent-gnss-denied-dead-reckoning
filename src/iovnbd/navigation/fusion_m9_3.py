"""
Module 9.3: Adaptive Fusion Switching & State Continuity Layer.
Combines:
- M5.1: 5D CAN ECU Speed EKF (Optimized for Short Outages T <= 30s)
- M9.1: 6D Roll-Aware Speed-Adaptive EKF (Optimized for Long Outages T > 30s)

Modes:
- 'm5_1_only': Pure M5.1 EKF
- 'm9_1_only': Pure M9.1 EKF
- 'fixed_switch': Fixed time threshold T_switch handoff
- 'adaptive_switch': Real-time confidence & covariance adaptive switching

State Handoff & Continuity:
Preserves position (E, N), speed (V), and heading (psi) across estimator transition,
initializing roll phi = 0 and gyro bias bz = 0 with smooth covariance state transfer. Zero GNSS data leakage.
"""

from dataclasses import dataclass
from typing import List, Dict, Any, Optional
import pandas as pd
import numpy as np

from src.iovnbd.navigation.coordinate_frame import AnchorOrigin, enu_to_geodetic
from src.iovnbd.navigation.initialization import InitialState
from src.iovnbd.navigation.ekf_m5_1 import propagate_ekf_m5_1, EKFResultM5_1
from src.iovnbd.navigation.ekf_m9 import propagate_ekf_m9_1, EKFResultM9_1, extract_outage_inputs_m9, compute_speed_adaptive_k_roll

@dataclass
class FusedStatePointM9_3:
    index: int
    t_rel_sec: float
    dt_sec: float
    east_m: float
    north_m: float
    up_m: float
    velocity_m_s: float
    heading_rad: float
    heading_deg: float
    roll_rad: float
    roll_deg: float
    gyro_bias_rad_s: float
    k_roll_adaptive: float
    active_estimator: str  # 'M5.1' or 'M9.1'
    switch_event: bool
    switch_reason: str
    std_east_m: float
    std_north_m: float
    std_velocity_m_s: float
    std_heading_deg: float
    std_roll_deg: float
    lat_deg: float
    lon_deg: float
    gnss_available: bool

@dataclass
class FusedResultM9_3:
    points: List[FusedStatePointM9_3]
    dataframe: pd.DataFrame
    outage_start_t: float
    outage_end_t: float
    outage_duration_sec: float
    mode: str
    t_switch_sec: float
    switch_count: int
    switch_timestamps: List[float]

def propagate_fused_ekf_m9_3(
    df_v: pd.DataFrame,
    df_s: pd.DataFrame,
    initial_state: InitialState,
    start_idx: int,
    outage_duration_sec: float,
    mode: str = "fixed_switch",  # 'm5_1_only', 'm9_1_only', 'fixed_switch', 'adaptive_switch'
    t_switch_sec: float = 30.0,
    k_base: float = 0.02,
    v0_m_s: float = 10.0,
    heading_std_threshold_deg: float = 8.0,
    yaw_rate_cum_threshold_rad: float = 0.5,
    yaw_scale_factor: float = 0.95,
    dynamic_yaw_scale_enabled: bool = False,
    q_var_pos: float = 1e-5,
    q_var_speed: float = 1e-3,
    q_var_yaw: float = 1e-4,
    q_var_roll: float = 1e-4,
    q_var_bias: float = 1e-6,
    r_var_ecu_speed: float = 1e-2,
    r_var_nhc: float = 1e-3,
    r_var_zupt: float = 1e-4,
    g_accel: float = 9.80665
) -> FusedResultM9_3:
    """
    Executes Module 9.3 Adaptive Fusion & Switching EKF with State Handoff Continuity.
    """
    t0 = initial_state.t_rel_sec
    outage_inputs = extract_outage_inputs_m9(
        df_v, df_s, t0, outage_duration_sec, start_idx,
        yaw_scale_factor=yaw_scale_factor,
        dynamic_yaw_scale_enabled=dynamic_yaw_scale_enabled
    )
    n_samples = len(outage_inputs)

    # Initialize M5.1 (5D) and M9.1 (6D) states
    # M5.1 state: x5 = [E, N, V, psi, bz]^T (dim 5)
    # M9.1 state: x6 = [E, N, V, psi, phi, bz]^T (dim 6)
    x5 = np.array([initial_state.east_m, initial_state.north_m, initial_state.speed_m_s, initial_state.heading_rad, 0.0])
    P5 = np.diag([1e-4, 1e-4, 1e-2, 1e-2, 1e-4])

    x6 = np.array([initial_state.east_m, initial_state.north_m, initial_state.speed_m_s, initial_state.heading_rad, 0.0, 0.0])
    P6 = np.diag([1e-4, 1e-4, 1e-2, 1e-2, 1e-3, 1e-4])

    active_estimator = "M5.1" if mode in ["m5_1_only", "fixed_switch", "adaptive_switch"] else "M9.1"
    switch_count = 0
    switch_timestamps = []
    has_switched = False

    points: List[FusedStatePointM9_3] = []
    origin = initial_state.origin
    cum_abs_yaw_rate = 0.0

    for i in range(n_samples):
        inp = outage_inputs[i]
        dt = inp.dt_sec
        t_rel = inp.t_rel_sec - t0

        switch_event = False
        switch_reason = "NONE"

        # Check switching condition if not yet switched
        if not has_switched:
            if mode == "fixed_switch":
                if t_rel >= t_switch_sec - 1e-5:
                    switch_event = True
                    switch_reason = f"FIXED_THRESHOLD_T={t_switch_sec:.1f}s"
            elif mode == "adaptive_switch":
                cum_abs_yaw_rate += abs(inp.yaw_rate_rad_s) * dt
                heading_std_deg = np.degrees(np.sqrt(max(0, P5[3, 3])))

                # Trigger adaptive switch if elapsed time >= 20s AND (heading uncertainty high OR cornering accumulated)
                if t_rel >= 20.0 and (heading_std_deg >= heading_std_threshold_deg or cum_abs_yaw_rate >= yaw_rate_cum_threshold_rad or t_rel >= 40.0):
                    switch_event = True
                    switch_reason = f"ADAPTIVE_CONFIDENCE_T={t_rel:.1f}s_STD={heading_std_deg:.1f}deg"

        # Execute State Handoff on Switch Event (M5.1 -> M9.1)
        if switch_event and active_estimator == "M5.1":
            active_estimator = "M9.1"
            has_switched = True
            switch_count += 1
            switch_timestamps.append(inp.t_rel_sec)

            # Continuous Handoff: Transfer E, N, V, psi, bz from M5.1 to M9.1
            x6[0] = x5[0]  # East
            x6[1] = x5[1]  # North
            x6[2] = x5[2]  # Speed
            x6[3] = x5[3]  # Heading
            x6[4] = 0.0    # Initial roll angle phi = 0.0 at switch point
            x6[5] = x5[4]  # Gyro bias bz

            # Transfer covariance blocks smoothly
            P6[0:4, 0:4] = P5[0:4, 0:4]
            P6[4, 4] = 1e-3  # Initial roll variance
            P6[5, 5] = P5[4, 4]  # Bias variance

        if i > 0:
            if active_estimator == "M5.1":
                # Propagate M5.1
                E, N, V, psi, b_z = x5
                a_long = inp.longitudinal_accel_m_s2
                omega_raw = inp.yaw_rate_rad_s

                E_pred = E + V * np.sin(psi) * dt
                N_pred = N + V * np.cos(psi) * dt
                V_pred = V + a_long * dt
                psi_pred = psi - (omega_raw - b_z) * dt
                psi_pred = (psi_pred + np.pi) % (2 * np.pi) - np.pi
                b_z_pred = b_z

                x5 = np.array([E_pred, N_pred, V_pred, psi_pred, b_z_pred])

                F5 = np.eye(5)
                F5[0, 2] = np.sin(psi) * dt
                F5[0, 3] = V * np.cos(psi) * dt
                F5[1, 2] = np.cos(psi) * dt
                F5[1, 3] = -V * np.sin(psi) * dt
                F5[3, 4] = dt

                Q5 = np.diag([q_var_pos, q_var_pos, q_var_speed, q_var_yaw, q_var_bias])
                P5 = F5 @ P5 @ F5.T + Q5
                P5 = (P5 + P5.T) / 2.0

                # Measurement Update: ECU Speed
                z_v = inp.ecu_speed_m_s
                H_v5 = np.array([[0, 0, 1, 0, 0]])
                y_v = z_v - x5[2]
                S_v5 = (H_v5 @ P5 @ H_v5.T + r_var_ecu_speed)[0, 0]
                K_v5 = P5 @ H_v5.T / S_v5
                x5 = x5 + (K_v5 * y_v).flatten()
                P5 = (np.eye(5) - K_v5 @ H_v5) @ P5
                P5 = (P5 + P5.T) / 2.0

                # Output formatting from M5.1
                curr_E, curr_N, curr_V, curr_psi, curr_roll, curr_bz = x5[0], x5[1], x5[2], x5[3], 0.0, x5[4]
                k_roll_curr = 0.0
                std_E, std_N, std_V, std_psi, std_roll = np.sqrt(max(0, P5[0,0])), np.sqrt(max(0, P5[1,1])), np.sqrt(max(0, P5[2,2])), np.sqrt(max(0, P5[3,3])), 0.0

            else:
                # Propagate M9.1
                k_roll_curr = compute_speed_adaptive_k_roll(x6[2], k_base=k_base, v0_m_s=v0_m_s)
                E, N, V, psi, phi, b_z = x6
                a_long = inp.longitudinal_accel_m_s2
                omega_raw = inp.yaw_rate_rad_s
                omega_roll = inp.roll_rate_rad_s

                E_pred = E + V * np.sin(psi) * dt
                N_pred = N + V * np.cos(psi) * dt
                V_pred = V + a_long * dt
                psi_pred = psi - (omega_raw - b_z) * dt
                psi_pred = (psi_pred + np.pi) % (2 * np.pi) - np.pi

                phi_pred = phi + omega_roll * dt - k_roll_curr * phi * dt
                phi_pred = (phi_pred + np.pi) % (2 * np.pi) - np.pi
                b_z_pred = b_z

                x6 = np.array([E_pred, N_pred, V_pred, psi_pred, phi_pred, b_z_pred])

                F6 = np.eye(6)
                F6[0, 2] = np.sin(psi) * dt
                F6[0, 3] = V * np.cos(psi) * dt
                F6[1, 2] = np.cos(psi) * dt
                F6[1, 3] = -V * np.sin(psi) * dt
                F6[3, 5] = dt
                F6[4, 4] = 1.0 - k_roll_curr * dt

                Q6 = np.diag([q_var_pos, q_var_pos, q_var_speed, q_var_yaw, q_var_roll, q_var_bias])
                P6 = F6 @ P6 @ F6.T + Q6
                P6 = (P6 + P6.T) / 2.0

                # Measurement Update 1: ECU Speed
                z_v = inp.ecu_speed_m_s
                H_v6 = np.array([[0, 0, 1, 0, 0, 0]])
                y_v = z_v - x6[2]
                S_v6 = (H_v6 @ P6 @ H_v6.T + r_var_ecu_speed)[0, 0]
                K_v6 = P6 @ H_v6.T / S_v6
                x6 = x6 + (K_v6 * y_v).flatten()
                P6 = (np.eye(6) - K_v6 @ H_v6) @ P6
                P6 = (P6 + P6.T) / 2.0

                # Measurement Update 2: Roll-Aware NHC
                if x6[2] > 0.5:
                    a_lat_meas = inp.lateral_accel_m_s2
                    omega_corr = omega_raw - x6[5]
                    h_nhc = x6[2] * omega_corr + g_accel * np.sin(x6[4])
                    nhc_residual = float(a_lat_meas - h_nhc)

                    if abs(nhc_residual) <= 3.0:
                        H_nhc = np.array([[0, 0, omega_corr, 0, g_accel * np.cos(x6[4]), -x6[2]]])
                        S_nhc = (H_nhc @ P6 @ H_nhc.T + r_var_nhc)[0, 0]
                        K_nhc = P6 @ H_nhc.T / S_nhc
                        x6 = x6 + (K_nhc * nhc_residual).flatten()
                        I_KH = np.eye(6) - K_nhc @ H_nhc
                        P6 = I_KH @ P6 @ I_KH.T + K_nhc * r_var_nhc * K_nhc.T
                        P6 = (P6 + P6.T) / 2.0

                # Output formatting from M9.1
                curr_E, curr_N, curr_V, curr_psi, curr_roll, curr_bz = x6[0], x6[1], x6[2], x6[3], x6[4], x6[5]
                std_E, std_N, std_V, std_psi, std_roll = np.sqrt(max(0, P6[0,0])), np.sqrt(max(0, P6[1,1])), np.sqrt(max(0, P6[2,2])), np.sqrt(max(0, P6[3,3])), np.sqrt(max(0, P6[4,4]))

        else:
            curr_E, curr_N, curr_V, curr_psi, curr_roll, curr_bz = x5[0], x5[1], x5[2], x5[3], 0.0, x5[4]
            k_roll_curr = 0.0
            std_E, std_N, std_V, std_psi, std_roll = np.sqrt(max(0, P5[0,0])), np.sqrt(max(0, P5[1,1])), np.sqrt(max(0, P5[2,2])), np.sqrt(max(0, P5[3,3])), 0.0

        lat_i, lon_i, _ = enu_to_geodetic(curr_E, curr_N, 0.0, origin)
        heading_deg_i = float(np.degrees(curr_psi)) % 360.0
        roll_deg_i = float(np.degrees(curr_roll))

        points.append(FusedStatePointM9_3(
            index=inp.index,
            t_rel_sec=inp.t_rel_sec,
            dt_sec=dt,
            east_m=curr_E,
            north_m=curr_N,
            up_m=0.0,
            velocity_m_s=curr_V,
            heading_rad=curr_psi,
            heading_deg=heading_deg_i,
            roll_rad=curr_roll,
            roll_deg=roll_deg_i,
            gyro_bias_rad_s=curr_bz,
            k_roll_adaptive=round(k_roll_curr, 4),
            active_estimator=active_estimator,
            switch_event=switch_event,
            switch_reason=switch_reason,
            std_east_m=std_E,
            std_north_m=std_N,
            std_velocity_m_s=std_V,
            std_heading_deg=float(np.degrees(std_psi)),
            std_roll_deg=float(np.degrees(std_roll)),
            lat_deg=lat_i,
            lon_deg=lon_i,
            gnss_available=False
        ))

    df_traj = pd.DataFrame([p.__dict__ for p in points])

    return FusedResultM9_3(
        points=points,
        dataframe=df_traj,
        outage_start_t=t0,
        outage_end_t=t0 + outage_duration_sec,
        outage_duration_sec=outage_duration_sec,
        mode=mode,
        t_switch_sec=t_switch_sec,
        switch_count=switch_count,
        switch_timestamps=switch_timestamps
    )

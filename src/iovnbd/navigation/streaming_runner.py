"""
Incremental / Streaming Navigation Runner for Module 9.3 Adaptive Fusion Estimator.
Processes multi-sensor data ONE sample at a time, maintaining state vectors and covariances continuously
without requiring future dataset access. Enforces strict zero-leakage GNSS masking.
"""

from dataclasses import dataclass
from typing import Dict, Any, List, Optional, Tuple
import numpy as np
import pandas as pd

from src.iovnbd.preprocessing.schema_validation import SingleSensorSample
from src.iovnbd.navigation.initialization import InitialState
from src.iovnbd.navigation.coordinate_frame import geodetic_to_enu, enu_to_geodetic
from src.iovnbd.navigation.fusion_m9_3 import FusedStatePointM9_3, FusedResultM9_3
from src.iovnbd.navigation.ekf_m9 import compute_speed_adaptive_k_roll

@dataclass
class OnlineNavigationState:
    """Persistent state container maintained across incremental sample updates."""
    regime: str
    t_prev_sec: float

    # M5.1 State Vector (5D): [E, N, V, psi, bz]
    x_m5_1: np.ndarray
    P_m5_1: np.ndarray

    # M9.1 State Vector (6D): [E, N, V, psi, phi, bz]
    x_m9_1: np.ndarray
    P_m9_1: np.ndarray

    # Causal heading history buffer for adaptive switching
    heading_history_m5_1: List[float]
    yaw_rate_cum_rad: float
    switch_event_occurred: bool
    switch_timestamp: Optional[float]

    # Process & Measurement Covariances
    Q_m5_1: np.ndarray
    R_m5_1: np.ndarray
    Q_m9_1: np.ndarray
    R_m9_1: np.ndarray

class StreamingNavigationRunner:
    """
    Incremental Dead Reckoning Navigation Runner.
    Processes one SingleSensorSample at a time while updating persistent EKF state continuously.
    """
    def __init__(
        self,
        initial_state: InitialState,
        mode: str = "adaptive_switch",
        t_switch_sec: float = 30.0,
        k_base: float = 0.02,
        v0_m_s: float = 10.0,
        yaw_scale_factor: float = 0.95
    ):
        self.init_state = initial_state
        self.mode = mode
        self.t_switch_sec = t_switch_sec
        self.k_base = k_base
        self.v0_m_s = v0_m_s
        self.yaw_scale_factor = yaw_scale_factor
        self.origin = initial_state.origin

        # Initialize M5.1 5D EKF state: [E, N, V, psi, bz]
        x_m5_1 = np.array([
            initial_state.east_m,
            initial_state.north_m,
            initial_state.speed_m_s,
            initial_state.heading_rad,
            0.0
        ], dtype=float)

        P_m5_1 = np.diag([1e-4, 1e-4, 1e-2, 1e-2, 1e-4])
        Q_m5_1 = np.diag([1e-5, 1e-5, 1e-3, 1e-4, 1e-6])
        R_m5_1 = np.diag([1e-2, 1e-3])

        # Initialize M9.1 6D EKF state: [E, N, V, psi, phi, bz]
        x_m9_1 = np.array([
            initial_state.east_m,
            initial_state.north_m,
            initial_state.speed_m_s,
            initial_state.heading_rad,
            0.0,
            0.0
        ], dtype=float)

        P_m9_1 = np.diag([1e-4, 1e-4, 1e-2, 1e-2, 1e-3, 1e-4])
        Q_m9_1 = np.diag([1e-5, 1e-5, 1e-3, 1e-4, 1e-4, 1e-6])
        R_m9_1 = np.diag([1e-2, 1e-3, 1e-3])

        self.state = OnlineNavigationState(
            regime="M5.1" if mode in ["m5_1_only", "fixed_switch", "adaptive_switch"] else "M9.1",
            t_prev_sec=initial_state.t_rel_sec,
            x_m5_1=x_m5_1,
            P_m5_1=P_m5_1,
            x_m9_1=x_m9_1,
            P_m9_1=P_m9_1,
            heading_history_m5_1=[initial_state.heading_deg],
            yaw_rate_cum_rad=0.0,
            switch_event_occurred=(mode == "m9_1_only"),
            switch_timestamp=None,
            Q_m5_1=Q_m5_1,
            R_m5_1=R_m5_1,
            Q_m9_1=Q_m9_1,
            R_m9_1=R_m9_1
        )

        self.history_points: List[FusedStatePointM9_3] = []
        self.sample_count = 0

    def process_sample(self, sample: SingleSensorSample) -> FusedStatePointM9_3:
        """Processes ONE incoming causal sensor sample and returns updated FusedStatePointM9_3."""
        dt = sample.t_rel_sec - self.state.t_prev_sec
        if dt <= 1e-5:
            dt = 0.10

        st = self.state
        outage_elapsed = sample.t_rel_sec - self.init_state.t_rel_sec

        switch_event = False
        switch_reason = "NONE"

        # Check switching condition if not yet switched
        if not st.switch_event_occurred:
            do_switch = False
            if self.mode == "m9_1_only":
                do_switch = True
            elif self.mode == "m5_1_only":
                do_switch = False
            elif self.mode == "fixed_switch":
                do_switch = (outage_elapsed >= self.t_switch_sec - 1e-5)
            elif self.mode == "adaptive_switch":
                heading_std_deg = np.degrees(np.sqrt(max(0, st.P_m5_1[3, 3])))
                if outage_elapsed >= 20.0 and (heading_std_deg >= 8.0 or st.yaw_rate_cum_rad >= 0.5 or outage_elapsed >= 40.0):
                    do_switch = True
                    switch_reason = f"ADAPTIVE_CONFIDENCE_T={outage_elapsed:.1f}s_STD={heading_std_deg:.1f}deg"

            if do_switch:
                st.regime = "M9.1"
                st.switch_event_occurred = True
                st.switch_timestamp = sample.t_rel_sec
                switch_event = True
                if switch_reason == "NONE":
                    switch_reason = f"FIXED_THRESHOLD_T={self.t_switch_sec:.1f}s"

                # Continuous State Handoff: copy [E, N, V, psi, bz] from M5.1 to M9.1
                st.x_m9_1[0] = st.x_m5_1[0]  # East
                st.x_m9_1[1] = st.x_m5_1[1]  # North
                st.x_m9_1[2] = st.x_m5_1[2]  # Speed
                st.x_m9_1[3] = st.x_m5_1[3]  # Heading
                st.x_m9_1[4] = 0.0           # Initial roll phi = 0.0 at switch point
                st.x_m9_1[5] = st.x_m5_1[4]  # Gyro bias bz

                st.P_m9_1[0:4, 0:4] = st.P_m5_1[0:4, 0:4]
                st.P_m9_1[4, 4] = 1e-3
                st.P_m9_1[5, 5] = st.P_m5_1[4, 4]

        if self.sample_count > 0:
            if st.regime == "M5.1":
                # Propagate M5.1
                E, N, V, psi, b_z = st.x_m5_1
                a_long = sample.longitudinal_accel_m_s2
                omega_raw = sample.yaw_rate_rad_s * self.yaw_scale_factor

                E_pred = E + V * np.sin(psi) * dt
                N_pred = N + V * np.cos(psi) * dt
                V_pred = V + a_long * dt
                psi_pred = psi - (omega_raw - b_z) * dt
                psi_pred = (psi_pred + np.pi) % (2 * np.pi) - np.pi
                b_z_pred = b_z

                st.x_m5_1 = np.array([E_pred, N_pred, V_pred, psi_pred, b_z_pred])

                F5 = np.eye(5)
                F5[0, 2] = np.sin(psi) * dt
                F5[0, 3] = V * np.cos(psi) * dt
                F5[1, 2] = np.cos(psi) * dt
                F5[1, 3] = -V * np.sin(psi) * dt
                F5[3, 4] = dt

                st.P_m5_1 = F5 @ st.P_m5_1 @ F5.T + st.Q_m5_1 * dt
                st.P_m5_1 = (st.P_m5_1 + st.P_m5_1.T) / 2.0

                # Measurement Update: ECU Speed
                z_v = sample.indicated_speed_m_s
                H_v5 = np.array([[0, 0, 1, 0, 0]])
                y_v = z_v - st.x_m5_1[2]
                S_v5 = (H_v5 @ st.P_m5_1 @ H_v5.T + st.R_m5_1[0,0])[0, 0]
                K_v5 = st.P_m5_1 @ H_v5.T / S_v5
                st.x_m5_1 = st.x_m5_1 + (K_v5 * y_v).flatten()
                st.P_m5_1 = (np.eye(5) - K_v5 @ H_v5) @ st.P_m5_1
                st.P_m5_1 = (st.P_m5_1 + st.P_m5_1.T) / 2.0

                curr_E, curr_N, curr_V, curr_psi, curr_roll, curr_bz = st.x_m5_1[0], st.x_m5_1[1], st.x_m5_1[2], st.x_m5_1[3], 0.0, st.x_m5_1[4]
                k_roll_curr = 0.0
                std_E, std_N, std_V, std_psi, std_roll = np.sqrt(max(0, st.P_m5_1[0,0])), np.sqrt(max(0, st.P_m5_1[1,1])), np.sqrt(max(0, st.P_m5_1[2,2])), np.sqrt(max(0, st.P_m5_1[3,3])), 0.0
            else:
                # Propagate M9.1
                k_roll_curr = compute_speed_adaptive_k_roll(st.x_m9_1[2], k_base=self.k_base, v0_m_s=self.v0_m_s)
                E, N, V, psi, phi, b_z = st.x_m9_1
                a_long = sample.longitudinal_accel_m_s2
                omega_raw = sample.yaw_rate_rad_s * self.yaw_scale_factor
                omega_roll = sample.roll_rate_rad_s

                E_pred = E + V * np.sin(psi) * dt
                N_pred = N + V * np.cos(psi) * dt
                V_pred = V + a_long * dt
                psi_pred = psi - (omega_raw - b_z) * dt
                psi_pred = (psi_pred + np.pi) % (2 * np.pi) - np.pi

                phi_pred = phi + omega_roll * dt - k_roll_curr * phi * dt
                phi_pred = (phi_pred + np.pi) % (2 * np.pi) - np.pi
                b_z_pred = b_z

                st.x_m9_1 = np.array([E_pred, N_pred, V_pred, psi_pred, phi_pred, b_z_pred])

                F6 = np.eye(6)
                F6[0, 2] = np.sin(psi) * dt
                F6[0, 3] = V * np.cos(psi) * dt
                F6[1, 2] = np.cos(psi) * dt
                F6[1, 3] = -V * np.sin(psi) * dt
                F6[3, 5] = dt
                F6[4, 4] = 1.0 - k_roll_curr * dt

                st.P_m9_1 = F6 @ st.P_m9_1 @ F6.T + st.Q_m9_1 * dt
                st.P_m9_1 = (st.P_m9_1 + st.P_m9_1.T) / 2.0

                # Update 1: ECU Speed
                z_v = sample.indicated_speed_m_s
                H_v6 = np.array([[0, 0, 1, 0, 0, 0]])
                y_v = z_v - st.x_m9_1[2]
                S_v6 = (H_v6 @ st.P_m9_1 @ H_v6.T + st.R_m9_1[0,0])[0, 0]
                K_v6 = st.P_m9_1 @ H_v6.T / S_v6
                st.x_m9_1 = st.x_m9_1 + (K_v6 * y_v).flatten()
                st.P_m9_1 = (np.eye(6) - K_v6 @ H_v6) @ st.P_m9_1
                st.P_m9_1 = (st.P_m9_1 + st.P_m9_1.T) / 2.0

                # Update 2: Roll-Aware NHC Lateral
                if st.x_m9_1[2] > 0.5:
                    a_lat_meas = sample.lateral_accel_m_s2
                    omega_corr = omega_raw - st.x_m9_1[5]
                    h_nhc = st.x_m9_1[2] * omega_corr + 9.80665 * np.sin(st.x_m9_1[4])
                    nhc_residual = float(a_lat_meas - h_nhc)

                    if abs(nhc_residual) <= 3.0:
                        H_nhc = np.array([[0, 0, omega_corr, 0, 9.80665 * np.cos(st.x_m9_1[4]), -st.x_m9_1[2]]])
                        r_nhc = st.R_m9_1[2,2]
                        S_nhc = (H_nhc @ st.P_m9_1 @ H_nhc.T + r_nhc)[0, 0]
                        K_nhc = st.P_m9_1 @ H_nhc.T / S_nhc
                        st.x_m9_1 = st.x_m9_1 + (K_nhc * nhc_residual).flatten()
                        I_KH = np.eye(6) - K_nhc @ H_nhc
                        st.P_m9_1 = I_KH @ st.P_m9_1 @ I_KH.T + K_nhc * r_nhc * K_nhc.T
                        st.P_m9_1 = (st.P_m9_1 + st.P_m9_1.T) / 2.0

                curr_E, curr_N, curr_V, curr_psi, curr_roll, curr_bz = st.x_m9_1[0], st.x_m9_1[1], st.x_m9_1[2], st.x_m9_1[3], st.x_m9_1[4], st.x_m9_1[5]
                std_E, std_N, std_V, std_psi, std_roll = np.sqrt(max(0, st.P_m9_1[0,0])), np.sqrt(max(0, st.P_m9_1[1,1])), np.sqrt(max(0, st.P_m9_1[2,2])), np.sqrt(max(0, st.P_m9_1[3,3])), np.sqrt(max(0, st.P_m9_1[4,4]))
        else:
            curr_E, curr_N, curr_V, curr_psi, curr_roll, curr_bz = st.x_m5_1[0], st.x_m5_1[1], st.x_m5_1[2], st.x_m5_1[3], 0.0, st.x_m5_1[4]
            k_roll_curr = 0.0
            std_E, std_N, std_V, std_psi, std_roll = np.sqrt(max(0, st.P_m5_1[0,0])), np.sqrt(max(0, st.P_m5_1[1,1])), np.sqrt(max(0, st.P_m5_1[2,2])), np.sqrt(max(0, st.P_m5_1[3,3])), 0.0

        st.yaw_rate_cum_rad += abs(sample.yaw_rate_rad_s) * dt
        lat_i, lon_i, _ = enu_to_geodetic(curr_E, curr_N, 0.0, self.origin)
        heading_deg_i = float(np.degrees(curr_psi)) % 360.0
        roll_deg_i = float(np.degrees(curr_roll))

        point = FusedStatePointM9_3(
            index=self.sample_count,
            t_rel_sec=sample.t_rel_sec,
            dt_sec=dt,
            east_m=float(curr_E),
            north_m=float(curr_N),
            up_m=0.0,
            velocity_m_s=float(curr_V),
            heading_rad=float(curr_psi),
            heading_deg=heading_deg_i,
            roll_rad=float(curr_roll),
            roll_deg=roll_deg_i,
            gyro_bias_rad_s=float(curr_bz),
            k_roll_adaptive=round(k_roll_curr, 4),
            active_estimator=st.regime,
            switch_event=switch_event,
            switch_reason=switch_reason,
            std_east_m=float(std_E),
            std_north_m=float(std_N),
            std_velocity_m_s=float(std_V),
            std_heading_deg=float(np.degrees(std_psi)),
            std_roll_deg=float(np.degrees(std_roll)),
            lat_deg=lat_i,
            lon_deg=lon_i,
            gnss_available=False
        )

        st.t_prev_sec = sample.t_rel_sec
        self.history_points.append(point)
        self.sample_count += 1

        return point

    def get_fused_result(self, outage_duration_sec: float) -> FusedResultM9_3:
        """Assembles complete FusedResultM9_3 from accumulated streaming state history."""
        df_out = pd.DataFrame([p.__dict__ for p in self.history_points])
        sw_ts = [p.t_rel_sec for p in self.history_points if p.switch_event]

        return FusedResultM9_3(
            points=self.history_points,
            dataframe=df_out,
            outage_start_t=self.init_state.t_rel_sec,
            outage_end_t=self.init_state.t_rel_sec + outage_duration_sec,
            outage_duration_sec=outage_duration_sec,
            mode=self.mode,
            t_switch_sec=self.t_switch_sec,
            switch_count=len(sw_ts),
            switch_timestamps=sw_ts
        )

"""
Module 10: Clean Production-Style Final Competition Navigation Interface.
Encapsulates the fully validated Module 9.3 Dual-Regime Adaptive Fusion Dead Reckoning Engine.
Provides clean APIs, strict zero GNSS leakage guarantee, deterministic inference, and complete diagnostic metadata.
"""

from dataclasses import dataclass
from typing import List, Dict, Any, Optional
import pandas as pd
import numpy as np

from src.iovnbd.navigation.initialization import initialize_navigation_state, InitialState
from src.iovnbd.navigation.fusion_m9_3 import propagate_fused_ekf_m9_3, FusedResultM9_3, FusedStatePointM9_3
from src.iovnbd.navigation.metrics import calculate_outage_error_metrics, OutageMetricsResult

@dataclass
class FinalDeadReckoningConfig:
    """Locked Competition Navigation Parameters (Zero S1 tuning)."""
    k_base: float = 0.02
    v0_m_s: float = 10.0
    fixed_switch_threshold_sec: float = 30.0
    fusion_mode: str = "adaptive_switch"  # 'adaptive_switch', 'fixed_switch', 'm5_1_only', 'm9_1_only'
    yaw_scale_factor: float = 0.90  # Validated V2.1 base yaw scale factor (1.0 = V1 baseline)
    dynamic_yaw_scale_enabled: bool = True  # V2.1 production dynamic lateral accel scaling

class FinalNavigationSystem:
    """
    Production-grade Dead Reckoning System for SIH 2026 PS-168.
    Fuses CAN ECU speed EKF (M5.1) and 6D roll-aware EKF (M9.1) with real-time confidence switching.
    """
    def __init__(self, config: Optional[FinalDeadReckoningConfig] = None):
        self.config = config if config is not None else FinalDeadReckoningConfig()

    def run_outage_navigation(
        self,
        df_vehicle: pd.DataFrame,
        df_smartphone: pd.DataFrame,
        start_idx: int = 1000,
        outage_duration_sec: float = 120.0
    ) -> FusedResultM9_3:
        """
        Executes dead reckoning navigation during GNSS outage.
        Strictly zero GNSS data leakage into inference loop.
        """
        init_state = initialize_navigation_state(df_vehicle, start_idx=start_idx)

        res = propagate_fused_ekf_m9_3(
            df_v=df_vehicle,
            df_s=df_smartphone,
            initial_state=init_state,
            start_idx=start_idx,
            outage_duration_sec=outage_duration_sec,
            mode=self.config.fusion_mode,
            t_switch_sec=self.config.fixed_switch_threshold_sec,
            k_base=self.config.k_base,
            v0_m_s=self.config.v0_m_s,
            yaw_scale_factor=self.config.yaw_scale_factor,
            dynamic_yaw_scale_enabled=self.config.dynamic_yaw_scale_enabled
        )

        return res

    def evaluate_outage_performance(
        self,
        result: FusedResultM9_3,
        df_vehicle_ground_truth: pd.DataFrame
    ) -> OutageMetricsResult:
        """
        Calculates offline position error metrics using ground truth GNSS.
        Ground truth is accessed exclusively after inference completes.
        """
        from src.iovnbd.navigation.baseline import TrajectoryResult
        traj_res = TrajectoryResult(
            points=result.points,
            dataframe=result.dataframe,
            outage_start_t=result.outage_start_t,
            outage_end_t=result.outage_end_t,
            outage_duration_sec=result.outage_duration_sec
        )
        return calculate_outage_error_metrics(traj_res, df_vehicle_ground_truth)

def get_final_competition_system() -> FinalNavigationSystem:
    """Factory function returning locked SIH 2026 competition navigation system."""
    return FinalNavigationSystem(FinalDeadReckoningConfig())

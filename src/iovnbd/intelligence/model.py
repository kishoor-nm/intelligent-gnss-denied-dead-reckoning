"""
Module 6: Pure NumPy Model Engine & Progression Evaluator.
Implements Model Progression:
- Stage 1: Constant Mean Baseline
- Stage 2: OLS Linear Regressor (Normal Equation)
- Stage 3: Ridge Regressor (L2 Regularization)
Includes parameter serialization (JSON) and evaluation metrics (MAE, RMSE, R²).
"""

from dataclasses import dataclass
from typing import Dict, Any, Tuple
import os
import json
import numpy as np

@dataclass
class ModelMetrics:
    mae: float
    rmse: float
    r2: float
    bias: float

class ConstantMeanRegressor:
    """Stage 1: Non-ML Baseline Predictor"""
    def __init__(self):
        self.mean_val: float = 0.0

    def fit(self, X: np.ndarray, y: np.ndarray):
        self.mean_val = float(np.mean(y))

    def predict(self, X: np.ndarray) -> np.ndarray:
        return np.full(X.shape[0], self.mean_val)

class OLSLinearRegressor:
    """Stage 2: Ordinary Least Squares Linear Regressor"""
    def __init__(self):
        self.weights: np.ndarray = np.array([])

    def fit(self, X: np.ndarray, y: np.ndarray):
        # Add intercept column
        X_design = np.column_stack([np.ones(X.shape[0]), X])
        self.weights = np.linalg.pinv(X_design.T @ X_design) @ X_design.T @ y

    def predict(self, X: np.ndarray) -> np.ndarray:
        X_design = np.column_stack([np.ones(X.shape[0]), X])
        return X_design @ self.weights

class RidgeLinearRegressor:
    """Stage 3: Ridge Regressor with L2 Regularization"""
    def __init__(self, alpha: float = 10.0):
        self.alpha: float = alpha
        self.weights: np.ndarray = np.array([])

    def fit(self, X: np.ndarray, y: np.ndarray):
        X_design = np.column_stack([np.ones(X.shape[0]), X])
        n_features = X_design.shape[1]
        I = np.eye(n_features)
        I[0, 0] = 0.0  # Do not regularize intercept term
        self.weights = np.linalg.pinv(X_design.T @ X_design + self.alpha * I) @ X_design.T @ y

    def predict(self, X: np.ndarray) -> np.ndarray:
        X_design = np.column_stack([np.ones(X.shape[0]), X])
        return X_design @ self.weights

def evaluate_predictions(y_true: np.ndarray, y_pred: np.ndarray) -> ModelMetrics:
    """Computes MAE, RMSE, R² and Mean Bias."""
    mae = float(np.mean(np.abs(y_true - y_pred)))
    rmse = float(np.sqrt(np.mean((y_true - y_pred)**2)))
    ss_tot = float(np.sum((y_true - np.mean(y_true))**2))
    ss_res = float(np.sum((y_true - y_pred)**2))
    r2 = float(1.0 - (ss_res / ss_tot)) if ss_tot > 0 else 0.0
    bias = float(np.mean(y_pred - y_true))

    return ModelMetrics(mae=round(mae, 4), rmse=round(rmse, 4), r2=round(r2, 4), bias=round(bias, 4))

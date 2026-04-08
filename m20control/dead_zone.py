from __future__ import annotations


def compensate_angular(omega_raw: float, omega_min: float = 0.15, epsilon: float = 0.02) -> float:
    if abs(omega_raw) < epsilon:
        return 0.0
    if abs(omega_raw) < omega_min:
        return omega_min if omega_raw > 0.0 else -omega_min
    return omega_raw

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class TransportKind(str, Enum):
    UDP = "udp"
    TCP = "tcp"


@dataclass(frozen=True)
class ConnectionConfig:
    host: str = "10.21.31.103"
    udp_port: int = 30000
    tcp_port: int = 30001
    local_host: str = "0.0.0.0"
    local_port: int = 0
    transport: TransportKind = TransportKind.UDP
    connect_timeout: float = 5.0
    recv_timeout: float = 0.2
    heartbeat_hz: float = 2.0

    @property
    def server_port(self) -> int:
        if self.transport == TransportKind.TCP:
            return self.tcp_port
        return self.udp_port


@dataclass(frozen=True)
class AxisScale:
    """
    Axis command full-scale mapping.

    The protocol uses [-1, 1] normalized axis values. These parameters model
    the robot-side speed configured at axis value 1.0 and should be calibrated
    on the actual M20 if the defaults do not match.
    """

    full_scale_x_mps: float = 1.0
    full_scale_y_mps: float = 0.5
    full_scale_yaw_rps: float = 1.0


@dataclass(frozen=True)
class NavigationConfig:
    control_hz: float = 20.0

    omega_min: float = 0.15
    omega_max: float = 0.8
    epsilon: float = 0.02

    kp_heading: float = 1.5
    kd_heading: float = 0.1

    vx_max: float = 0.5
    vx_slow: float = 0.15
    vy_max: float = 0.2
    kp_distance: float = 0.8
    kp_approach: float = 0.5
    kp_lateral: float = 0.5

    heading_threshold: float = 0.1
    heading_revert: float = 0.5
    approach_radius: float = 0.5
    goal_tolerance: float = 0.2
    settle_time: float = 0.3
    max_run_time: float = 120.0

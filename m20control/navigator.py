from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

from .config import AxisScale, NavigationConfig
from .dead_zone import compensate_angular


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def world_to_body(dx_world: float, dy_world: float, yaw: float) -> tuple[float, float]:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    dx_body = cos_yaw * dx_world + sin_yaw * dy_world
    dy_body = -sin_yaw * dx_world + cos_yaw * dy_world
    return dx_body, dy_body


@dataclass(frozen=True)
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    body_vx: float = 0.0
    body_vy: float = 0.0
    omega_z: float = 0.0


@dataclass(frozen=True)
class TargetPoint:
    x: float
    y: float


class NavPhase(str, Enum):
    IDLE = "IDLE"
    ROTATE = "ROTATE"
    DRIVE = "DRIVE"
    APPROACH = "APPROACH"
    REACHED = "REACHED"
    FAILED = "FAILED"


@dataclass(frozen=True)
class MotionCommand:
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0

    def as_axis(self, axis_scale: AxisScale) -> tuple[float, float, float]:
        x = clamp(self.vx / axis_scale.full_scale_x_mps, -1.0, 1.0)
        y = clamp(self.vy / axis_scale.full_scale_y_mps, -1.0, 1.0)
        yaw = clamp(self.omega / axis_scale.full_scale_yaw_rps, -1.0, 1.0)
        return x, y, yaw


@dataclass(frozen=True)
class NavigationOutput:
    phase: NavPhase
    distance: float
    heading_error: float
    command: MotionCommand


class SingleGoalNavigator:
    def __init__(self, config: NavigationConfig) -> None:
        self.config = config
        self.goal: TargetPoint | None = None
        self.phase = NavPhase.IDLE
        self.prev_heading_error = 0.0
        self.aligned_since: float | None = None
        self.start_time: float | None = None

    def set_goal(self, goal: TargetPoint, now: float) -> None:
        self.goal = goal
        self.phase = NavPhase.ROTATE
        self.prev_heading_error = 0.0
        self.aligned_since = None
        self.start_time = now

    def update(self, pose: Pose2D, dt: float, now: float) -> NavigationOutput:
        if self.goal is None:
            self.phase = NavPhase.IDLE
            return NavigationOutput(self.phase, 0.0, 0.0, MotionCommand())

        if self.start_time is not None and now - self.start_time > self.config.max_run_time:
            self.phase = NavPhase.FAILED
            return NavigationOutput(self.phase, math.inf, 0.0, MotionCommand())

        dx_world = self.goal.x - pose.x
        dy_world = self.goal.y - pose.y
        distance = math.hypot(dx_world, dy_world)
        if distance <= self.config.goal_tolerance:
            self.phase = NavPhase.REACHED
            return NavigationOutput(self.phase, distance, 0.0, MotionCommand())

        target_bearing = math.atan2(dy_world, dx_world)
        heading_error = normalize_angle(target_bearing - pose.yaw)
        dx_body, dy_body = world_to_body(dx_world, dy_world, pose.yaw)

        if self.phase in (NavPhase.IDLE, NavPhase.REACHED):
            self.phase = NavPhase.ROTATE

        if self.phase == NavPhase.ROTATE:
            if abs(heading_error) < self.config.heading_threshold:
                if self.aligned_since is None:
                    self.aligned_since = now
                elif now - self.aligned_since >= self.config.settle_time:
                    self.phase = NavPhase.APPROACH if distance < self.config.approach_radius else NavPhase.DRIVE
            else:
                self.aligned_since = None

            if self.phase == NavPhase.ROTATE:
                derivative = 0.0 if dt <= 0.0 else (heading_error - self.prev_heading_error) / dt
                omega_raw = self.config.kp_heading * heading_error + self.config.kd_heading * derivative
                omega = compensate_angular(omega_raw, self.config.omega_min, self.config.epsilon)
                omega = clamp(omega, -self.config.omega_max, self.config.omega_max)
                self.prev_heading_error = heading_error
                return NavigationOutput(self.phase, distance, heading_error, MotionCommand(omega=omega))

        if self.phase == NavPhase.DRIVE:
            if abs(heading_error) > self.config.heading_revert:
                self.phase = NavPhase.ROTATE
                return self.update(pose, dt, now)
            if distance < self.config.approach_radius:
                self.phase = NavPhase.APPROACH
                return self.update(pose, dt, now)

            vx = min(self.config.vx_max, self.config.kp_distance * distance)
            omega_raw = self.config.kp_heading * heading_error
            omega = compensate_angular(omega_raw, self.config.omega_min, self.config.epsilon)
            omega = clamp(omega, -self.config.omega_max, self.config.omega_max)
            self.prev_heading_error = heading_error
            return NavigationOutput(self.phase, distance, heading_error, MotionCommand(vx=vx, omega=omega))

        if self.phase == NavPhase.APPROACH:
            vx = clamp(self.config.kp_approach * dx_body, -self.config.vx_slow, self.config.vx_slow)
            vy = clamp(self.config.kp_lateral * dy_body, -self.config.vy_max, self.config.vy_max)
            self.prev_heading_error = heading_error
            return NavigationOutput(self.phase, distance, heading_error, MotionCommand(vx=vx, vy=vy))

        return NavigationOutput(self.phase, distance, heading_error, MotionCommand())

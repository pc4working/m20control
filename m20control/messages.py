from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from enum import IntEnum
from typing import Any


def protocol_timestamp() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


class UsageMode(IntEnum):
    REGULAR = 0
    NAVIGATION = 1


class MotionState(IntEnum):
    IDLE = 0
    STAND = 1
    SOFT_EMERGENCY_STOP = 2
    POWER_ON_DAMPING = 3
    SIT = 4
    STANDARD = 6


class Gait(IntEnum):
    BASIC = 1
    STAIR = 14


@dataclass(frozen=True)
class BasicStatus:
    motion_state: int = 0
    gait: int = 0
    charge: int = 0
    hes: int = 0
    control_usage_mode: int = 0
    direction: int = 0
    ooa: int = 0
    power_management: int = 0
    sleep: bool = False
    version: str = ""

    @classmethod
    def from_items(cls, items: dict[str, Any]) -> "BasicStatus":
        data = items.get("BasicStatus", {})
        return cls(
            motion_state=int(data.get("MotionState", 0)),
            gait=int(data.get("Gait", 0)),
            charge=int(data.get("Charge", 0)),
            hes=int(data.get("HES", 0)),
            control_usage_mode=int(data.get("ControlUsageMode", 0)),
            direction=int(data.get("Direction", 0)),
            ooa=int(data.get("OOA", 0)),
            power_management=int(data.get("PowerManagement", 0)),
            sleep=_as_bool(data.get("Sleep", False)),
            version=str(data.get("Version", "")),
        )


@dataclass(frozen=True)
class MotionStatus:
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    omega_z: float = 0.0
    linear_x: float = 0.0
    linear_y: float = 0.0
    height: float = 0.0
    payload: float = 0.0
    remain_mile: float = 0.0

    @classmethod
    def from_items(cls, items: dict[str, Any]) -> "MotionStatus":
        data = items.get("MotionStatus", {})
        return cls(
            roll=float(data.get("Roll", 0.0)),
            pitch=float(data.get("Pitch", 0.0)),
            yaw=float(data.get("Yaw", 0.0)),
            omega_z=float(data.get("OmegaZ", 0.0)),
            linear_x=float(data.get("LinearX", 0.0)),
            linear_y=float(data.get("LinearY", 0.0)),
            height=float(data.get("Height", 0.0)),
            payload=float(data.get("Payload", 0.0)),
            remain_mile=float(data.get("RemainMile", 0.0)),
        )


@dataclass(frozen=True)
class StatusMessage:
    type: int
    command: int
    time: str = ""
    items: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_payload(cls, payload: dict[str, Any]) -> "StatusMessage":
        body = payload.get("PatrolDevice", {})
        return cls(
            type=int(body.get("Type", 0)),
            command=int(body.get("Command", 0)),
            time=str(body.get("Time", "")),
            items=dict(body.get("Items", {}) or {}),
        )


def make_payload(message_type: int, command: int, items: dict[str, Any] | None = None) -> dict[str, Any]:
    return {
        "PatrolDevice": {
            "Type": message_type,
            "Command": command,
            "Time": protocol_timestamp(),
            "Items": items or {},
        }
    }


def make_heartbeat_payload() -> dict[str, Any]:
    return make_payload(100, 100)


def make_usage_mode_payload(mode: int | UsageMode) -> dict[str, Any]:
    return make_payload(1101, 5, {"Mode": int(mode)})


def make_motion_state_payload(motion_state: int | MotionState) -> dict[str, Any]:
    return make_payload(2, 22, {"MotionParam": int(motion_state)})


def make_gait_payload(gait: int | Gait) -> dict[str, Any]:
    return make_payload(2, 23, {"GaitParam": int(gait)})


def make_axis_payload(
    x: float,
    y: float,
    yaw: float,
    z: float = 0.0,
    roll: float = 0.0,
    pitch: float = 0.0,
) -> dict[str, Any]:
    return make_payload(
        2,
        21,
        {
            "X": float(x),
            "Y": float(y),
            "Z": float(z),
            "Roll": float(roll),
            "Pitch": float(pitch),
            "Yaw": float(yaw),
        },
    )

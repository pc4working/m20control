from __future__ import annotations

import logging
import math
import socket
import threading
import time
from dataclasses import dataclass
from typing import Callable

from .config import ConnectionConfig
from .messages import (
    BasicStatus,
    Gait,
    MotionState,
    MotionStatus,
    StatusMessage,
    UsageMode,
    make_axis_payload,
    make_gait_payload,
    make_heartbeat_payload,
    make_motion_state_payload,
    make_usage_mode_payload,
)
from .navigator import Pose2D, normalize_angle
from .protocol import Frame, FrameDecoder, encode_json_frame
from .transport import BaseTransport, build_transport


Predicate = Callable[["RobotSnapshot"], bool]


@dataclass(frozen=True)
class RobotSnapshot:
    pose: Pose2D
    basic_status: BasicStatus | None
    motion_status: MotionStatus | None
    last_basic_time: float | None
    last_motion_time: float | None


class M20Client:
    def __init__(self, config: ConnectionConfig, logger: logging.Logger | None = None) -> None:
        self.config = config
        self.logger = logger or logging.getLogger("m20control")
        self.transport: BaseTransport = build_transport(config)
        self.decoder = FrameDecoder()

        self._send_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._state_cv = threading.Condition(self._state_lock)
        self._stop_event = threading.Event()
        self._connected = False

        self._recv_thread: threading.Thread | None = None
        self._heartbeat_thread: threading.Thread | None = None

        self._next_message_id = 1
        self._basic_status: BasicStatus | None = None
        self._motion_status: MotionStatus | None = None
        self._last_basic_time: float | None = None
        self._last_motion_time: float | None = None

        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._yaw_bias: float | None = None
        self._prev_motion_monotonic: float | None = None

    def connect(self) -> None:
        self.transport.connect()
        self._connected = True
        self._stop_event.clear()
        self._recv_thread = threading.Thread(target=self._recv_loop, name="m20-recv", daemon=True)
        self._recv_thread.start()

    def close(self) -> None:
        self._stop_event.set()
        self._connected = False
        self.transport.close()
        if self._recv_thread is not None:
            self._recv_thread.join(timeout=1.0)
            self._recv_thread = None
        if self._heartbeat_thread is not None:
            self._heartbeat_thread.join(timeout=1.0)
            self._heartbeat_thread = None

    def start_heartbeat(self) -> None:
        if self._heartbeat_thread is not None and self._heartbeat_thread.is_alive():
            return
        self._heartbeat_thread = threading.Thread(target=self._heartbeat_loop, name="m20-heartbeat", daemon=True)
        self._heartbeat_thread.start()

    def send_heartbeat(self) -> None:
        self._send_payload(make_heartbeat_payload())

    def set_usage_mode(self, mode: int | UsageMode) -> None:
        self._send_payload(make_usage_mode_payload(mode))

    def set_motion_state(self, motion_state: int | MotionState) -> None:
        self._send_payload(make_motion_state_payload(motion_state))

    def set_gait(self, gait: int | Gait) -> None:
        self._send_payload(make_gait_payload(gait))

    def command_axes(
        self,
        x: float,
        y: float,
        yaw: float,
        z: float = 0.0,
        roll: float = 0.0,
        pitch: float = 0.0,
    ) -> None:
        self._send_payload(make_axis_payload(x=x, y=y, yaw=yaw, z=z, roll=roll, pitch=pitch))

    def stop_motion(self) -> None:
        self.command_axes(0.0, 0.0, 0.0)

    def reset_pose(self) -> None:
        with self._state_lock:
            self._odom_x = 0.0
            self._odom_y = 0.0
            self._prev_motion_monotonic = None
            if self._motion_status is not None:
                self._yaw_bias = self._motion_status.yaw
                self._odom_yaw = 0.0
            else:
                self._yaw_bias = None
                self._odom_yaw = 0.0
            self._state_cv.notify_all()

    def wait_for_status(self, timeout: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout
        with self._state_cv:
            while time.monotonic() < deadline:
                if self._basic_status is not None and self._motion_status is not None:
                    return True
                remaining = deadline - time.monotonic()
                self._state_cv.wait(timeout=max(0.0, min(0.2, remaining)))
        return False

    def wait_until(self, predicate: Predicate, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        with self._state_cv:
            while time.monotonic() < deadline:
                if predicate(self._snapshot_locked()):
                    return True
                remaining = deadline - time.monotonic()
                self._state_cv.wait(timeout=max(0.0, min(0.2, remaining)))
            return predicate(self._snapshot_locked())

    def ensure_axis_control_ready(
        self,
        wait_timeout: float = 5.0,
        usage_mode: int | UsageMode = UsageMode.REGULAR,
        motion_state: int | MotionState = MotionState.STAND,
        gait: int | Gait = Gait.BASIC,
    ) -> None:
        self.set_usage_mode(usage_mode)
        if not self.wait_until(
            lambda s: s.basic_status is not None and s.basic_status.control_usage_mode == int(usage_mode),
            timeout=wait_timeout,
        ):
            self.logger.warning("usage mode not confirmed within %.1fs", wait_timeout)

        self.set_motion_state(motion_state)
        if not self.wait_until(
            lambda s: s.basic_status is not None and s.basic_status.motion_state == int(motion_state),
            timeout=wait_timeout,
        ):
            self.logger.warning("motion state not confirmed within %.1fs", wait_timeout)

        self.set_gait(gait)
        if not self.wait_until(
            lambda s: s.basic_status is not None and s.basic_status.gait in {int(gait), int(gait) - 1},
            timeout=wait_timeout,
        ):
            self.logger.warning("gait not confirmed within %.1fs", wait_timeout)

    def get_snapshot(self) -> RobotSnapshot:
        with self._state_lock:
            return self._snapshot_locked()

    def _snapshot_locked(self) -> RobotSnapshot:
        pose = Pose2D(
            x=self._odom_x,
            y=self._odom_y,
            yaw=self._odom_yaw,
            body_vx=self._motion_status.linear_x if self._motion_status else 0.0,
            body_vy=self._motion_status.linear_y if self._motion_status else 0.0,
            omega_z=self._motion_status.omega_z if self._motion_status else 0.0,
        )
        return RobotSnapshot(
            pose=pose,
            basic_status=self._basic_status,
            motion_status=self._motion_status,
            last_basic_time=self._last_basic_time,
            last_motion_time=self._last_motion_time,
        )

    def _send_payload(self, payload: dict) -> None:
        if not self._connected:
            raise RuntimeError("client is not connected")
        with self._send_lock:
            message_id = self._next_message_id
            self._next_message_id = 1 if self._next_message_id >= 0xFFFF else self._next_message_id + 1
            frame = encode_json_frame(payload, message_id=message_id)
            self.transport.send(frame)

    def _heartbeat_loop(self) -> None:
        period = 1.0 / self.config.heartbeat_hz
        while not self._stop_event.is_set():
            try:
                self.send_heartbeat()
            except OSError:
                if not self._stop_event.is_set():
                    self.logger.exception("heartbeat send failed")
            self._stop_event.wait(period)

    def _recv_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                data = self.transport.recv()
                if not data:
                    continue
                for frame in self.decoder.feed(data):
                    self._handle_frame(frame)
            except socket.timeout:
                continue
            except OSError:
                if not self._stop_event.is_set():
                    self.logger.exception("receive loop stopped by socket error")
                break
            except Exception:
                if not self._stop_event.is_set():
                    self.logger.exception("receive loop stopped by unexpected error")
                break

    def _handle_frame(self, frame: Frame) -> None:
        message = StatusMessage.from_payload(frame.payload)
        now = time.monotonic()

        with self._state_cv:
            if message.type == 1002 and message.command == 6:
                self._basic_status = BasicStatus.from_items(message.items)
                self._last_basic_time = now
            elif message.type == 1002 and message.command == 4:
                self._motion_status = MotionStatus.from_items(message.items)
                self._last_motion_time = now
                self._integrate_motion(now, self._motion_status)
            self._state_cv.notify_all()

    def _integrate_motion(self, now: float, motion_status: MotionStatus) -> None:
        if self._yaw_bias is None:
            self._yaw_bias = motion_status.yaw

        current_yaw = normalize_angle(motion_status.yaw - self._yaw_bias)
        if self._prev_motion_monotonic is not None:
            dt = now - self._prev_motion_monotonic
            if 0.0 < dt < 1.0:
                cos_yaw = math.cos(current_yaw)
                sin_yaw = math.sin(current_yaw)
                world_vx = motion_status.linear_x * cos_yaw - motion_status.linear_y * sin_yaw
                world_vy = motion_status.linear_x * sin_yaw + motion_status.linear_y * cos_yaw
                self._odom_x += world_vx * dt
                self._odom_y += world_vy * dt

        self._odom_yaw = current_yaw
        self._prev_motion_monotonic = now

from __future__ import annotations

import argparse
import logging
import sys
import time

from .client import M20Client
from .config import AxisScale, ConnectionConfig, NavigationConfig, TransportKind
from .messages import Gait, MotionState, UsageMode
from .navigator import NavPhase, SingleGoalNavigator, TargetPoint


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="M20 single-goal point navigation over TCP/UDP control protocol.")
    parser.add_argument("--transport", choices=[kind.value for kind in TransportKind], default=TransportKind.UDP.value)
    parser.add_argument("--host", default="10.21.31.103")
    parser.add_argument("--udp-port", type=int, default=30000)
    parser.add_argument("--tcp-port", type=int, default=30001)
    parser.add_argument("--local-host", default="0.0.0.0")
    parser.add_argument("--local-port", type=int, default=0)
    parser.add_argument("--goal-x", type=float, required=True, help="Target X in meters, relative to start frame.")
    parser.add_argument("--goal-y", type=float, required=True, help="Target Y in meters, relative to start frame.")
    parser.add_argument("--heartbeat-hz", type=float, default=2.0)
    parser.add_argument("--control-hz", type=float, default=20.0)
    parser.add_argument("--gait", type=int, default=int(Gait.BASIC))
    parser.add_argument("--motion-state", type=int, default=int(MotionState.STAND))
    parser.add_argument("--full-scale-x", type=float, default=1.0)
    parser.add_argument("--full-scale-y", type=float, default=0.5)
    parser.add_argument("--full-scale-yaw", type=float, default=1.0)
    parser.add_argument("--max-run-time", type=float, default=120.0)
    parser.add_argument("--log-level", default="INFO")
    return parser


def configure_logging(level: str) -> None:
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(message)s",
    )


def run() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()
    configure_logging(args.log_level)
    logger = logging.getLogger("m20control")

    connection_config = ConnectionConfig(
        host=args.host,
        udp_port=args.udp_port,
        tcp_port=args.tcp_port,
        local_host=args.local_host,
        local_port=args.local_port,
        transport=TransportKind(args.transport),
        heartbeat_hz=args.heartbeat_hz,
    )
    nav_config = NavigationConfig(control_hz=args.control_hz, max_run_time=args.max_run_time)
    axis_scale = AxisScale(
        full_scale_x_mps=args.full_scale_x,
        full_scale_y_mps=args.full_scale_y,
        full_scale_yaw_rps=args.full_scale_yaw,
    )

    client = M20Client(connection_config, logger=logger)
    navigator = SingleGoalNavigator(nav_config)
    goal = TargetPoint(x=args.goal_x, y=args.goal_y)

    try:
        client.connect()
        client.start_heartbeat()
        if not client.wait_for_status(timeout=5.0):
            logger.error("did not receive basic and motion status within 5 seconds")
            return 1

        client.ensure_axis_control_ready(
            usage_mode=UsageMode.REGULAR,
            motion_state=args.motion_state,
            gait=args.gait,
        )
        client.reset_pose()
        time.sleep(0.2)

        start = time.monotonic()
        navigator.set_goal(goal, now=start)
        loop_period = 1.0 / nav_config.control_hz
        last_log_time = 0.0
        previous_step = start

        while True:
            now = time.monotonic()
            snapshot = client.get_snapshot()
            dt = max(1e-3, now - previous_step)
            previous_step = now

            output = navigator.update(snapshot.pose, dt=dt, now=now)
            axis_x, axis_y, axis_yaw = output.command.as_axis(axis_scale)
            client.command_axes(axis_x, axis_y, axis_yaw)

            if now - last_log_time >= 0.5:
                logger.info(
                    "phase=%s pose=(%.3f, %.3f, %.3f) dist=%.3f err=%.3f axis=(%.3f, %.3f, %.3f)",
                    output.phase,
                    snapshot.pose.x,
                    snapshot.pose.y,
                    snapshot.pose.yaw,
                    output.distance,
                    output.heading_error,
                    axis_x,
                    axis_y,
                    axis_yaw,
                )
                last_log_time = now

            if output.phase == NavPhase.REACHED:
                logger.info("goal reached within %.3f m tolerance", nav_config.goal_tolerance)
                break
            if output.phase == NavPhase.FAILED:
                logger.error("navigation failed by timeout")
                return 2

            sleep_time = loop_period - (time.monotonic() - now)
            if sleep_time > 0.0:
                time.sleep(sleep_time)

        client.stop_motion()
        return 0
    except KeyboardInterrupt:
        logger.warning("interrupted by user")
        return 130
    finally:
        try:
            client.stop_motion()
        except Exception:
            pass
        client.close()


if __name__ == "__main__":
    sys.exit(run())

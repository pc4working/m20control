from __future__ import annotations

import unittest

from m20control.config import AxisScale, NavigationConfig
from m20control.navigator import MotionCommand, NavPhase, Pose2D, SingleGoalNavigator, TargetPoint


class NavigatorTest(unittest.TestCase):
    def test_rotate_phase_when_target_is_left(self) -> None:
        navigator = SingleGoalNavigator(NavigationConfig())
        navigator.set_goal(TargetPoint(x=0.0, y=1.0), now=0.0)
        output = navigator.update(Pose2D(), dt=0.1, now=0.1)
        self.assertEqual(output.phase, NavPhase.ROTATE)
        self.assertAlmostEqual(output.command.vx, 0.0)
        self.assertGreater(output.command.omega, 0.0)

    def test_drive_phase_after_alignment_settle(self) -> None:
        navigator = SingleGoalNavigator(NavigationConfig())
        navigator.set_goal(TargetPoint(x=2.0, y=0.0), now=0.0)

        first = navigator.update(Pose2D(), dt=0.1, now=0.1)
        self.assertEqual(first.phase, NavPhase.ROTATE)

        second = navigator.update(Pose2D(), dt=0.1, now=0.6)
        self.assertEqual(second.phase, NavPhase.DRIVE)
        self.assertGreater(second.command.vx, 0.0)

    def test_reached_phase_near_goal(self) -> None:
        navigator = SingleGoalNavigator(NavigationConfig(goal_tolerance=0.2))
        navigator.set_goal(TargetPoint(x=1.0, y=0.0), now=0.0)
        output = navigator.update(Pose2D(x=0.9, y=0.0, yaw=0.0), dt=0.1, now=0.1)
        self.assertEqual(output.phase, NavPhase.REACHED)
        self.assertEqual(output.command, MotionCommand())

    def test_axis_scaling_is_clamped(self) -> None:
        axis = MotionCommand(vx=2.0, vy=-1.0, omega=2.0).as_axis(AxisScale())
        self.assertEqual(axis, (1.0, -1.0, 1.0))


if __name__ == "__main__":
    unittest.main()

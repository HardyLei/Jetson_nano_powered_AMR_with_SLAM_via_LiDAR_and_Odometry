"""Test the mecanum model without ROS."""

import math

from amr_control.kinematics import encoder_twist, forward_kinematics, integrate_pose
from amr_control.kinematics import inverse_kinematics, yaw_quaternion
import pytest

RADIUS = 0.04
WHEELBASE_SUM = 0.0825 + 0.1031
TICKS_PER_TURN = 660


@pytest.mark.parametrize('ticks, expected', [
    ((660, 660, 660, 660), (math.tau * RADIUS, 0.0, 0.0)),
    ((-660, 660, 660, -660), (0.0, math.tau * RADIUS, 0.0)),
    ((-660, 660, -660, 660), (0.0, 0.0, math.tau * RADIUS / WHEELBASE_SUM)),
])
def test_encoder_body_displacement(ticks, expected):
    """Keep the wheel order and signs consistent with the firmware."""
    dt = 0.05
    twist = encoder_twist((0,) * 4, ticks, dt, TICKS_PER_TURN, RADIUS, WHEELBASE_SUM)
    assert tuple(v * dt for v in twist) == pytest.approx(expected)


def test_velocity_round_trip():
    """Recover the original twist from the controller's wheel commands."""
    body = (0.11, -0.07, 0.4)
    wheels = inverse_kinematics(*body, RADIUS, WHEELBASE_SUM)
    assert forward_kinematics(wheels, RADIUS, WHEELBASE_SUM) == pytest.approx(body)


def test_integrating_world_pose_and_quaternion():
    """Rotate body velocity into odom and generate a unit quaternion."""
    x, y, yaw = integrate_pose(0, 0, math.pi / 2, 1, 0, math.pi, 1)
    assert (x, y, yaw) == pytest.approx((0.0, 1.0, -math.pi / 2))
    assert yaw_quaternion(yaw) == pytest.approx((0.0, 0.0, -math.sqrt(0.5), math.sqrt(0.5)))


def test_encoder_rollover():
    """Read one positive tick across the signed int32 boundary."""
    previous = (2**31 - 1,) * 4
    current = (-2**31,) * 4
    vx, vy, wz = encoder_twist(previous, current, 1.0, TICKS_PER_TURN, RADIUS, WHEELBASE_SUM)
    assert (vx, vy, wz) == pytest.approx((math.tau * RADIUS / TICKS_PER_TURN, 0.0, 0.0))

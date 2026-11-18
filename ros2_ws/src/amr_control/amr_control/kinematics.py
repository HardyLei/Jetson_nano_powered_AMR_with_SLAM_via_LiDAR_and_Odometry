"""Mecanum kinematics in FL, FR, RL, RR wheel order."""

from math import cos, pi, sin


def inverse_kinematics(vx, vy, wz, radius, k):
    """Convert body-frame twist to wheel angular velocities."""
    return (
        (vx - vy - k * wz) / radius,
        (vx + vy + k * wz) / radius,
        (vx + vy - k * wz) / radius,
        (vx - vy + k * wz) / radius,
    )


def forward_kinematics(wheels, radius, k):
    """Convert wheel angular velocities to body-frame twist."""
    fl, fr, rl, rr = wheels
    return (
        radius * (fl + fr + rl + rr) / 4.0,
        radius * (-fl + fr + rl - rr) / 4.0,
        radius * (-fl + fr - rl + rr) / (4.0 * k),
    )


def encoder_twist(previous, current, dt, ticks_per_rev, radius, k):
    """Convert int32 encoder tick increments to body-frame twist."""
    radians_per_tick = 2.0 * pi / ticks_per_rev
    # Treat a signed int32 rollover as a small increment of the same sign.
    deltas = ((new - old + 2**31) % 2**32 - 2**31
              for old, new in zip(previous, current))
    speeds = (ticks * radians_per_tick / dt for ticks in deltas)
    return forward_kinematics(speeds, radius, k)


def integrate_pose(x, y, yaw, vx, vy, wz, dt):
    """Integrate body-frame velocities and keep yaw in [-pi, pi)."""
    return (
        x + (vx * cos(yaw) - vy * sin(yaw)) * dt,
        y + (vx * sin(yaw) + vy * cos(yaw)) * dt,
        (yaw + wz * dt + pi) % (2.0 * pi) - pi,
    )


def yaw_quaternion(yaw):
    """Return the ROS (x, y, z, w) quaternion for planar yaw."""
    return 0.0, 0.0, sin(yaw / 2.0), cos(yaw / 2.0)

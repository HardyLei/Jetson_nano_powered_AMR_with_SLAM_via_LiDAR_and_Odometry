"""Convert ROS wheel commands to motor firmware PWM values."""

from math import isfinite


def wheel_speed_to_pwm(speed, full_scale_speed, max_pwm, min_pwm=0, stop_speed=0.0):
    """Scale and limit a rad/s command to the configured signed PWM range."""
    if not isfinite(speed) or abs(speed) <= stop_speed:
        return 0
    fraction = max(-1.0, min(1.0, speed / full_scale_speed))
    magnitude = max(min_pwm, round(abs(fraction) * max_pwm))
    return magnitude if speed > 0 else -magnitude

"""Convert ROS wheel commands to motor firmware PWM values."""

from math import isfinite


def wheel_speed_to_pwm(speed, full_scale_speed, max_pwm):
    """Scale and limit a rad/s command to the configured signed PWM range."""
    if not isfinite(speed):
        return 0
    fraction = max(-1.0, min(1.0, speed / full_scale_speed))
    return round(fraction * max_pwm)

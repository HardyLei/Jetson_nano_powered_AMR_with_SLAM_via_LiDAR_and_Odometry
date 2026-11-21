"""Test the Arduino protocol and PWM conversion without hardware."""

from amr_hardware.packet_builder import build_wheel_command
from amr_hardware.packet_parser import parse_encoder_packet
from amr_hardware.pwm import wheel_speed_to_pwm
import pytest


def test_packet_order():
    """Match the Arduino protocol's front-left, front-right, rear-left, rear-right order."""
    assert build_wheel_command(1, -2, 3, -4) == 'CMD,1,-2,3,-4\n'
    assert parse_encoder_packet('ENC,1,-2,3,-4') == {
        'front_left': 1, 'front_right': -2, 'rear_left': 3, 'rear_right': -4,
    }


@pytest.mark.parametrize('packet', [
    'OK,1,2,3,4', 'ENC,1,2', 'ENC,1,no,3,4', 'ENC,2147483648,0,0,0',
])
def test_reject_invalid_packets(packet):
    """Reject malformed lines without accepting corrupt counts."""
    with pytest.raises(ValueError):
        parse_encoder_packet(packet)


@pytest.mark.parametrize('rad_s, expected', [
    (0.0, 0), (10.0, 128), (-10.0, -128), (20.0, 255), (-80.0, -255), (80.0, 255),
    (float('nan'), 0),
])
def test_pwm_clamp(rad_s, expected):
    """Convert signed velocity and respect the firmware limits."""
    assert wheel_speed_to_pwm(rad_s, 20.0, 255) == expected


@pytest.mark.parametrize('rad_s, expected', [
    (0.0, 0), (0.05, 0), (-0.1, 0), (0.2, 76), (-1.25, -76),
    (10.0, 128), (-80.0, -255), (float('inf'), 0),
])
def test_pwm_floor_preserves_stop_and_limits(rad_s, expected):
    """Apply the floor only to intended motion and preserve the signed PWM limit."""
    assert wheel_speed_to_pwm(rad_s, 20.0, 255, min_pwm=76, stop_speed=0.1) == expected

"""Encode wheel PWM commands for the Arduino."""


def build_wheel_command(fl, fr, rl, rr):
    """Return a newline-terminated CMD packet in FL, FR, RL, RR order."""
    return f'CMD,{fl},{fr},{rl},{rr}\n'

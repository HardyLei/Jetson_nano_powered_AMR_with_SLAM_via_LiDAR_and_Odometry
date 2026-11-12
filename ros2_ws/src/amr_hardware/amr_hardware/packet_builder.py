"""
this function turns the wheel command into a packet to send to the robot
assume the cmd packet goes: CMD, fl, fr, rl, rr
"""
def build_wheel_command(fl,fr,rl,rr):
    """
    Build a wheel command packet.

    Args:
        fl (int): Front left wheel speed.
        fr (int): Front right wheel speed.
        rr (int): Rear right wheel speed.
        rl (int): Rear left wheel speed.

    Returns:
        bytes: The constructed wheel command packet.
    """

    return f'CMD,{fl},{fr},{rl},{rr}\n'

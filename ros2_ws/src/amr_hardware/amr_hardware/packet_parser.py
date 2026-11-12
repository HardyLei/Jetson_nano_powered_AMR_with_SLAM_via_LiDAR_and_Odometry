"""
this function parses the encoder packet from the robot
assume the encoder packet goes: ENC, fr, fl, rr, rl
"""
def parse_encoder_packet(packet):
    """
    Parse an encoder packet.

    Args:
        packet (str): The encoder packet to parse.

    Returns:
        dict: A dictionary containing the parsed encoder values.
    """
    parts = packet.split(',')
    if parts[0] != 'ENC' or len(parts) != 5:
        raise ValueError("Invalid encoder packet format")
    return {
        "front_left": int(parts[1]),
        "front_right": int(parts[2]),
        "rear_left": int(parts[3]),
        "rear_right": int(parts[4])
    }
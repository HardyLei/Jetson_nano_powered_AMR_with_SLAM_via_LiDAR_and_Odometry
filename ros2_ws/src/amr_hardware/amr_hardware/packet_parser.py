"""Decode cumulative encoder measurements from the Arduino."""


def parse_encoder_packet(packet):
    """Return int32 encoder counts from an ENC,fl,fr,rl,rr line."""
    parts = packet.split(',')
    if len(parts) != 5 or parts[0] != 'ENC':
        raise ValueError('Invalid encoder packet format')
    counts = tuple(map(int, parts[1:]))
    if any(not -(2**31) <= count < 2**31 for count in counts):
        raise ValueError('Encoder count outside int32 range')
    return dict(zip(('front_left', 'front_right', 'rear_left', 'rear_right'), counts))

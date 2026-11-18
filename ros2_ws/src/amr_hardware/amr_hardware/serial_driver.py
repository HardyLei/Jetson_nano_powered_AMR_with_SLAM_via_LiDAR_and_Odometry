"""Nonblocking line framing for the Arduino serial stream."""

import serial


class SerialDriver:
    """Read complete lines and keep incomplete frames between polls."""

    def __init__(self, port, baudrate):
        self.serial = serial.Serial(
            port, baudrate, timeout=0, write_timeout=0.05, exclusive=True,
        )
        self.pending = bytearray()
        self.discard_line = False

    def write(self, message):
        """Write one ASCII command."""
        self.serial.write(message.encode('ascii'))

    def read_lines(self):
        """Yield currently available lines, keeping the frame buffer bounded."""
        # A valid ENC packet is under 64 bytes even with four signed int32s.
        for byte in self.serial.read(min(self.serial.in_waiting, 4096)):
            if byte == 10:
                if not self.discard_line:
                    yield self.pending.decode('ascii', errors='ignore').strip()
                self.pending.clear()
                self.discard_line = False
            elif not self.discard_line:
                if len(self.pending) < 64:
                    self.pending.append(byte)
                else:
                    self.discard_line = True

    def close(self):
        """Release the serial device."""
        self.serial.close()

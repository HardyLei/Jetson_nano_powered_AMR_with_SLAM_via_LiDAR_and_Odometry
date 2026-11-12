import serial

class SerialDriver:
    def __init__(self, port, baudrate):
        self.serial = serial.Serial(port, 
                                    baudrate,
                                    timeout=0.1)
    def write(self, message:str):
        self.serial.write(message.encode())
    def read(self):
        return self.serial.readline().decode().strip()
    
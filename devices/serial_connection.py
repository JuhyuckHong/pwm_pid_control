import serial


class SerialConnection:
    def __init__(self,
                 port,
                 baudrate,
                 parity=serial.PARITY_NONE,
                 stopbits=serial.STOPBITS_ONE,
                 bytesize=serial.EIGHTBITS,
                 timeout=1):
        try:
            self.serial_connection = serial.Serial(
                port=port,
                baudrate=baudrate,
                parity=parity,
                stopbits=stopbits,
                bytesize=bytesize,
                timeout=timeout)
        except serial.SerialException as e:
            print(f"Serial connection error: {e}")

    def disconnect(self):
        self.serial_connection.close()

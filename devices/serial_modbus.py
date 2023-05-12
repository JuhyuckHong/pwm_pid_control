from serial_connection import SerialConnection
import crcmod.predefined
import struct


class PressureTransmitter(SerialConnection):
    def __init__(self, port, baudrate, parity, stopbits, bytesize, timeout):
        super().__init__(port, baudrate, parity, stopbits, bytesize, timeout)

    def set_data(self, machine_addr=b'\x01', data_addr=b'\x02'):
        self.data = machine_addr + b'\x03' + b'\x00' + data_addr + b'\x00\x01'

    def add_crc(self):
        # Calculate the CRC16 value for the Modbus RTU message
        crc16 = crcmod.predefined.Crc('modbus')
        # Updates the CRC16 checksum with the data
        crc16.update(self.data)
        # Returns 16-bit CRC checksum for the data
        crc_bytes = crc16.digest()
        # Reverse the byte order of the CRC16 bytes
        crc_bytes_reversed = crc_bytes[::-1]
        self.data += crc_bytes_reversed

    def send_command_and_read(self):
        self.write(self.data)
        response = self.read(7)
        _, _, _, value, _ = struct.unpack('>BBBhH', response)
        return value

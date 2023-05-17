import time
import serial
import crcmod.predefined
import struct
import re


def PressureTransmitter(port='dev/ttyUSB0', baudrate=9600):

    serial_connection = serial.Serial(port=port,
                                      baudrate=baudrate,
                                      parity=serial.PARITY_NONE,
                                      stopbits=serial.STOPBITS_ONE,
                                      bytesize=serial.EIGHTBITS,
                                      timeout=1)
    machine_addr=b'\x01'
    data_addr=b'\x01'
    data = machine_addr + b'\x03' + b'\x00' + data_addr + b'\x00\x01'

    # Calculate the CRC16 value for the Modbus RTU message
    crc16 = crcmod.predefined.Crc('modbus')
    # Updates the CRC16 checksum with the data
    crc16.update(data)
    # Returns 16-bit CRC checksum for the data
    crc_bytes = crc16.digest()
    # Reverse the byte order of the CRC16 bytes
    crc_bytes_reversed = crc_bytes[::-1]
    data += crc_bytes_reversed
    # Write data
    serial_connection.write(data)
    # Read data
    response = serial_connection.read(7)
    _, _, _, value, _ = struct.unpack('>BBBhH', response)

    return value/10


def PWMGenerator(input_duty, port='dev/ttyUSB0', baudrate=9600):
    serial_connection = serial.Serial(port=port,
                                      baudrate=baudrate,
                                      parity=serial.PARITY_NONE,
                                      stopbits=serial.STOPBITS_ONE,
                                      bytesize=serial.EIGHTBITS,
                                      timeout=1)
    # Make string type
    if not isinstance(input_duty, str):
        input_duty = str(input_duty)
    # Strip
    duty = input_duty.strip()
    # Set input pattern
    pattern = r'^(?:100|[1-9]\d|\d)$'
    try:
        # Check input pattern
        if not re.match(pattern, input_duty):
            raise ValueError("유효하지 않은 숫자 입력으로, 기본값인 0으로 설정합니다.")
    # if raised ValueError
    except ValueError as err:
        print(err)
        # Set 0 as dafault duty
        duty = '0'
    # Add 'D' prefix and encode as bytes
    duty = f"D{duty.zfill(3)}".encode('utf-8')
    # Write duty
    serial_connection.write(duty)
    return input_duty

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.last_error = 0
        self.integral = 0
        self.max_control = 10
        self.min_control = -10
        self._last_time = None

    def compute_control_signal(self, setpoint, measured_value):
        # Define error
        error = setpoint - measured_value

        # Calculate time difference between current and last time to determine the duration
        # since the last control signal calculation
        current_time = time.time()
        if self._last_time is None:
            time_diff = 0
        else:
            time_diff = current_time - self._last_time
        self._last_time = current_time

        # Proportional term
        P = self.Kp * error

        # Integral term
        self.integral += error * time_diff
        I = self.Ki * self.integral

        # Derivative term
        derivative = (error - self.last_error) / \
            time_diff if time_diff != 0 else 0
        D = self.Kd * derivative

        # Compute control signal
        control_signal = P + I + D
        if control_signal > self.max_control:
            control_signal = self.max_control
        if control_signal < self.min_control:
            control_signal = self.min_control
        # print(f"{control_signal: 03.2f} = P:{P: 03.2f} + I:{I: 03.2f} + D:{D: 03.2f}")

        # Update last error
        self.last_error = error

        return control_signal


if __name__ == '__main__':
    # Connect pressure transmitter and set command & make CRC
    PT = PressureTransmitter(port="/dev/ttyUSB0", baudrate=9600)
    # Connect PWM Generator
    PWM = PWMGenerator(0, port='/dev/ttyS0', baudrate=9600)
    # Initialize duty cycle
    duty = 0
    PWM = PWMGenerator(0, port='/dev/ttyS0', baudrate=9600)

    # Confirm current status
    print(duty, PT)

    # Set target
    target_pressure = int(input("target pressure(0~100): "))

    # Set target reaching condition
    converged_time = 0
    threshold = 1
    convergence_duration = 10

    pid = PIDController(Kp=0.1, Ki=0.0, Kd=0.0)

    current_pressure = PressureTransmitter(port="/dev/ttyUSB0", baudrate=9600)
    step = 0
    while True:
        # Record the starting time
        start_time = time.time()

        # Derive control value
        control = pid.compute_control_signal(target_pressure, current_pressure)

        # Add duty with control
        duty += control
        duty = round(duty)
        duty = min(100, max(0, duty))

        # Set duty value to PWM generator
        duty = PWMGenerator(duty, port='/dev/ttyS0', baudrate=9600)

        # Get pressure data
        current_pressure = PressureTransmitter(port="/dev/ttyUSB0", baudrate=9600)
        pressure_difference = target_pressure - current_pressure

        # Calculate the time spend whitin a step
        time_difference = time.time() - start_time

        # See control status
        print(
            f"[time+ {time_difference: 4.0f}, step: {step: 4.0f}] PWM: {duty: 03.2f}, \
 current value: {current_pressure: 03.2f}, \
 pressure difference: {pressure_difference: 03.2f}, \
 converged: {converged_time: 01.1f}")

        # Stop control when reached target with threshold during set time
        if abs(pressure_difference) < threshold:
            converged_time += time_difference
        else:
            converged_time = 0

        if converged_time >= convergence_duration:
            break

        # step++
        step += 1
        time.sleep(1)

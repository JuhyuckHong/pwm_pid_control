import time
import serial
import crcmod.predefined
import struct
import re


class PressureTransmitter:
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
        self.serial_connection.write(self.data)
        response = self.serial_connection.read(7)
        _, _, _, value, _ = struct.unpack('>BBBhH', response)
        return value


class PWMGenerator:
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

    def set_frequency(self, input_frequency):
        # Strip
        input_frequency = input_frequency.strip()
        # Set input pattern
        pattern = r'^(?:\d{1,3}(?:\.\d{1})?|\d{1,3})(?:k)?$'
        try:
            # Check input pattern
            if not re.match(pattern, input_frequency):
                raise ValueError("유효하지 않은 숫자 입력으로, 기본값인 25.0k로 설정합니다.")
            # Convert input frequency to lowercase
            frequency = input_frequency.lower()
            # over 1000 Hz using k,
            if 'k' in frequency:
                frequency = frequency.replace('k', '')
                # over 100k
                if int(frequency.split('.')[0]) >= 100:
                    frequency = '.'.join(list(frequency.split('.')[0]))
                # less then 100k with a point
                elif '.' in frequency:
                    pass
                # less then 100k without a point
                else:
                    frequency += '.0'
            # less then 1000 Hz with a point(neglect less then 1 Hz)
            elif '.' in frequency:
                frequency = frequency.split('.')[0]
            # if given digits
            else:
                pass
        # if raised ValueError,
        except ValueError as err:
            print(err)
            # Set 25k as default frequency
            frequency = "25.0"

        # Add 'K' prefix and encode as bytes
        frequency = f"K{frequency}".encode('utf-8')

        return self.serial_connection.write(frequency)

    def set_duty(self, input_duty):
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

        return self.serial_connection.write(duty)

    def read_status(self):
        while (True):
            # Send the command "READ" to the device to request data
            self.serial_connection.write(b"READ")
            time.sleep(0.1)
            try:
                # Read 12 bytes of data from the device
                res = self.serial_connection.read(12)
                # Check if the first byte of the response is equal to 70 (hexadecimal value of "F")
                if res and res[0] == 70:
                    res = res.decode()
                    # Extract the frequency value from the response string
                    freq_res = res.split("D")[0].split("F")[1].strip()
                    # Extract the duty cycle value from the response string
                    duty_res = res.split(freq_res)[1].split("D")[1].strip()
                    # Print the frequency and duty cycle values that were set and read
                    print(f"Frequency Read: {freq_res}")
                    print(f"DutyCycle Read: {int(duty_res)} %")
                    # Exit the while loop
                    break
            # If response from the device does not contain expected values,
            # pass over this iteration of the while loop
            except IndexError:
                pass
            # Close the serial connection with the device
            self.serial_connection.close()


class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.last_error = 0
        self.integral = 0
        self.max_control = 10
        self.min_control = -10
        self.sample_time = 1
        self._last_time = None

    @property
    def sample_time(self):
        return self.sample_time

    @sample_time.setter
    def sample_time(self, value):
        self.sample_time = value

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
    if __package__ is None:
        import sys
        from os import path
        print(path.dirname(path.dirname(path.abspath(__file__))))
        sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
        from devices.serial_modbus import PressureTransmitter
        from devices.serial_ttl import PWMGenerator
    else:
        from .devices.serial_modbus import PressureTransmitter
        from .devices.serial_ttl import PWMGenerator

    # Connect pressure transmitter and set command & make CRC
    PT = PressureTransmitter(port="/dev/ttyUSB0", baudrate=9600)
    PT.set_data()
    PT.add_crc()

    # Connect PWM Generator
    PWM = PWMGenerator(port='/dev/ttyS0', baudrate=9600)
    # Read current status
    PWM.read_status()
    # Initialize duty cycle
    duty = 0
    PWM.set_duty(duty)

    # Set target
    target_pressure = int(input("target pressure(0~100): "))

    # Set target reaching condition
    converged_time = 0
    threshold = 1
    convergence_duration = 10

    pid = PIDController(Kp=1, Ki=0.0, Kd=0.0)

    current_pressure = PT.send_command_and_read()

    while step := 1:
        # Record the starting time
        start_time = time.time()

        # Derive control value
        control = pid.compute_control_signal(target_pressure, current_pressure)

        # Add duty with control
        duty += control
        duty = min(100, max(0, duty))

        # Set duty value to PWM generator
        PWM.set_duty(duty)

        # Get pressure data
        current_pressure = PT.send_command_and_read()
        pressure_difference = target_pressure - current_pressure

        # Calculate the time spend whitin a step
        time_difference = time.time() - start_time

        # See control status
        print(
            f"[time+ {time_difference: 4.0f}, step: {step: 4.0f}] PWM: {PWM: 03.2f}, \
            current value: {current_pressure: 03.2f}, \
            pressure difference: {pressure_difference: 03.2f}, \
            converged: {converged_time: 01.1f}"
        )

        # Stop control when reached target with threshold during set time
        if abs(pressure_difference) < threshold:
            converged_time += time_difference
        else:
            converged_time = 0

        if converged_time >= convergence_duration:
            break

        # step++
        step += 1

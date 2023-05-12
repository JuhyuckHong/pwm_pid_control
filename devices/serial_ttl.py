from serial_connection import SerialConnection
import serial
import re
import time


class PWMGenerator(SerialConnection):
    def __init__(self, port, baudrate, parity, stopbits, bytesize, timeout):
        try:
            super().__init__(port, baudrate, parity, stopbits, bytesize, timeout)
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

        return self.write(frequency)

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

        return self.write(duty)

    def read_status(self):
        while (True):
            # Send the command "READ" to the device to request data
            self.write(b"READ")
            time.sleep(0.1)
            try:
                # Read 12 bytes of data from the device
                res = self.read(12)
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
            self.disconnect()

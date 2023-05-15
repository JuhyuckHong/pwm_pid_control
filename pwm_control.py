from .devices.serial_modbus import PressureTransmitter
from .devices.serial_ttl import PWMGenerator
import time


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

import time
from time import sleep
import math
import random
import matplotlib.pyplot as plt

current_value = 10.0


def get_current_value():
    global current_value
    return current_value


def apply_control_signal(PWM, control_signal):
    global current_value
    pwm_max = 100
    pwm_min = 0
    if control_signal < 0:
        sign = -1
    else:
        sign = 1
    # + random.uniform(-0.5, 0.5)
    current_value += sign * math.sqrt(abs(control_signal))
    # current_value += sign * abs(control_signal) # + random.uniform(-.4, .4)
    res = PWM + control_signal
    if res > 100:
        res = pwm_max
    elif res < 0:
        res = pwm_min

    return res


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

        # Calculate time difference
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


target_pressure = 70

# PID Controller
pid = PIDController(Kp=0.01, Ki=1, Kd=0.0001)

# control target
current = get_current_value()
PWM = 0
error_threshold = 0.5
converged_time = 0
convergence_duration = 2

# Initialize lists to store data for plotting
time_values = []
control_values = []
current_values = []
error_values = []

# PID control excuting
counter = 0
while True:
    # Record the starting time
    start_time = time.time()

    control = pid.compute_control_signal(
        target_pressure, current)  # Calculate pid control value
    PWM = apply_control_signal(PWM, control)  # applying control value
    current = get_current_value()  # Measure current value
    error = target_pressure - current

    time_step = random.uniform(0, 0.01)
    sleep(time_step)

    # Calculate the time spend whitin a step
    time_diff = time.time() - start_time
    print(f"[time+ {time_diff: 2.1f}, step: {counter: 4.0f}] PWM: {PWM: 03.2f}, current: {current: 03.2f}, error: {error: 03.2f}, converged: {converged_time: 01.1f}")
    counter += 1

    # Store data for plotting
    time_values.append(counter)
    control_values.append(PWM)
    current_values.append(current)
    error_values.append(error)

    if abs(error) < error_threshold:
        converged_time += time_diff
    else:
        converged_time = 0

    if converged_time >= convergence_duration:
        break

try:
    # Plot the control signal, current value, and error
    plt.figure(figsize=(10, 6))
    plt.subplot(311)
    plt.plot(time_values, control_values)
    plt.xlabel('Time')
    plt.ylabel('Control Signal')

    plt.subplot(312)
    plt.plot(time_values, current_values)
    plt.xlabel('Time')
    plt.ylabel('Current Value')

    plt.subplot(313)
    plt.plot(time_values, error_values)
    plt.xlabel('Time')
    plt.ylabel('Error')

    plt.tight_layout()
    plt.show()

except KeyboardInterrupt:
    plt.close()

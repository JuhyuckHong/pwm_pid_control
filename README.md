# PWM PID Control

The PWM PID Control project is a Python-based application for controlling PWM signals using a PID controller. It includes modules for serial communication, PWM control, and PID testing.

## Table of Contents

- [Project Structure](#project-structure)
- [Dependencies](#dependencies)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Project Structure

The project directory structure is as follows:

```
pwm_pid_control
├─ devices
│  ├─ serial_connection.py
│  ├─ serial_modbus.py
│  └─ serial_ttl.py
├─ pid_test.py
├─ pwm_control.py
├─ README.md
└─ __init__.py
```

- `devices/`: This directory contains modules related to serial connections.
- `pid_test.py`: Test script/module for the PID control algorithm.
- `pwm_control.py`: Main module for PWM control.
- `README.md`: Markdown file providing information and documentation.
- `__init__.py`: Empty file indicating a Python package.

## Dependencies

The project has the following dependencies:

- contourpy==1.0.7
- crcmod==1.7
- cycler==0.11.0
- fonttools==4.39.4
- kiwisolver==1.4.4
- matplotlib==3.7.1
- numpy==1.24.3
- packaging==23.1
- Pillow==9.5.0
- pyparsing==3.0.9
- pyserial==3.5
- python-dateutil==2.8.2
- six==1.16.0

## Usage

To use the PWM PID Control project, follow these steps:

1. Clone the repository: `git clone https://github.com/JuhyuckHong/pwm_pid_control.git`
2. Install the required dependencies: `pip install -r requirements.txt`
3. Run the main script: `python pwm_control.py`

Adjust the parameters and configurations in `pwm_control.py` and `pid_test.py` as needed to suit your application.

## Contributing

Contributions to the PWM PID Control project are welcome! If you find any issues or have suggestions for improvements, please open an issue or submit a pull request. We appreciate your feedback.

## License

The PWM PID Control project is licensed under the [MIT License](LICENSE).

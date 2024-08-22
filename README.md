# Fan Control System

## Overview
This project is a fan control system using PID control. It integrates with an EEPROM for persistent parameter storage and offers various commands for real-time adjustments via serial communication.

## Requirements
- Arduino IDE
- Libraries:
  - SerialCommand
  - EEPROMex
  - median3
  - PID

## How to Use

1. **Setup:**
   - Configure the pins and initialize parameters.
   - Connect to the serial monitor at 115200 baud rate.

2. **Commands:**
   - `set_p <value>`: Set the P factor of PID.
   - `set_i <value>`: Set the I factor of PID.
   - `set_d <value>`: Set the D factor of PID.
   - `get_p`: Get the current P factor.
   - `get_i`: Get the current I factor.
   - `get_d`: Get the current D factor.
   - `set_target <value>`: Set the target value for the PID controller.
   - `get_target`: Get the current target value.
   - `set_dzone <value>`: Set the dead zone for PID control.
   - `get_dzone`: Get the current dead zone value.
   - `set_min_out <value>`: Set the minimum output value for the controller.
   - `get_min_out`: Get the current minimum output value.
   - `set_max_out <value>`: Set the maximum output value for the controller.
   - `get_max_out`: Get the current maximum output value.
   - `on_control`: Enable control mode.
   - `off_control`: Disable control mode.
   - `on_log`: Enable logging.
   - `off_log`: Disable logging.
   - `reset`: Reset the system.
   - `set_speed <value>`: Set the serial communication speed.
   - `info`: Print the software and PID parameters.
   - `who`: Print the software name.

## Pin Configuration
- **Control A:** Pin 12
- **Speed A:** Pin 3
- **Brake A:** Pin 9
- **Current Sensing A:** Pin A0
- **Control B:** Pin 13
- **Speed B:** Pin 11
- **Brake B:** Pin 8
- **Current Sensing B:** Pin A1
- **Info LED Green:** Pin 5
- **Info LED Red:** Pin 6

## Functionality
- **PID Control:** Adjusts the fan speed based on the target and feedback values.
- **Logging:** Optionally logs PID parameters and control actions.
- **Serial Communication:** Configurable baud rate for serial communication.

## License

This project is licensed under a restrictive license. You **may not** modify, distribute, or use this code without permission. If you wish to use or adapt this code, please contact the author to obtain the necessary permissions. For inquiries, please reach out to me on LinkedIn (Kseniia Soboleva).

## Author
Kseniia Sovoleva, Viktor Ereshenko

## Version
v0.1

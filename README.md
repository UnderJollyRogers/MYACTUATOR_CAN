# MYACTUATOR_CAN
The objective of this project is to develop a library that allows communication with MYACTUATOR motors using the CAN protocol.

## Setting Up 

### Drivers
If using Linux, there is no need to install any specific driver to control the motors. If using Windows or another OS, please refer to [www.myactuator.com](https://www.myactuator.com/) for more information.

### USB to CAN Hardware
For testing the library, we used the MKS CANable V1.0. More information is available at [CANable-MKS](https://github.com/makerbase-mks/CANable-MKS/tree/main).

### Installation Guide

The library that enables CAN communication using Python is called `python-can`. To install this library, run:
```bash
pip install python-can
```
Then, configure the terminal for CAN communication by running:
```bash
sudo ip link set can0 up type can bitrate 1000000
```
Here, can0 is the name of the real hardware, and bitrate is the bitrate for communication.

## Stand Alone Functions
This section refers to the individual functions that will eventually be part of the myactuator class. These standalone functions are designed to perform specific tasks and can be tested independently.

### Function: speed_closed_loop_control

Controls the motor speed in closed-loop mode.

#### Args:
- **bus**: The CAN bus used for communication.
- **motor_id**: The motor ID.
- **target_speed_rpm**: The target speed in RPM.

#### Returns:
- The actual motor speed in RPM.

### Function: torque_closed_loop_control

Sends a torque closed-loop control command to the motor.

#### Args:
- **bus**: The CAN bus used for communication.
- **motor_id**: The motor ID.
- **iqControl**: The control current in Amperes.

#### Returns:
- Response from the motor.

### Function: read_motor_status

Reads the motor status from two different status commands.

#### Args:
- **bus**: The CAN bus used for communication.
- **motor_id**: The motor ID.

#### Returns:
- A dictionary containing the status information. The dictionary contains:
    - `motor_temperature`: The temperature of the motor (in degrees Celsius).
    - `brake_control_command`: The state of the brake control command (1: release, 0: lock).
    - `voltage`: The voltage of the motor (in Volts).
    - `error_state`: The error state flags of the motor.
    - `torque_current`: The torque current value of the motor (in Amperes).
    - `motor_speed`: The speed of the motor (in dps).
    - `motor_angle`: The angle of the motor shaft (in degrees).

### Function: absolute_position_control

Sends an absolute position closed-loop control command to the motor.

#### Args:
- **bus**: The CAN bus used for communication.
- **motor_id**: The motor ID.
- **max_speed_dps**: The maximum speed in degrees per second (DPS).
- **angle_control**: The target angle for the motor position in degrees.

#### Returns:
- Response from the motor.

### Function: system_brake_release

Sends a system brake release command to the motor.

#### Args:
- **bus**: The CAN bus used for communication.
- **motor_id**: The motor ID.

#### Returns:
- Response from the motor.

### Function: system_brake_lock

Sends a system brake lock command to the motor.

#### Args:
- **bus**: The CAN bus used for communication.
- **motor_id**: The motor ID.
- **message** (optional): Boolean flag to indicate if a message should be printed or logged.

#### Returns:
- `success`: Indicates whether the instruction was executed properly.




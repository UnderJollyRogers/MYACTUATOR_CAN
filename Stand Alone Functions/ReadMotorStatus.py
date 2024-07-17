import can

def read_motor_status(bus, motor_id):
    """
    Reads the motor status from two different status commands.

    Args:
        bus: The CAN bus used for communication.
        motor_id: The motor ID.

    Returns:
        A dictionary containing the status information.
        The dictionary contains:
            'motor_temperature': The temperature of the motor (in degrees Celsius).
            'brake_control_command': The state of the brake control command (1: release, 0: lock).
            'voltage': The voltage of the motor (in Volts).
            'error_state': The error state flags of the motor.
            'torque_current': The torque current value of the motor (in Amperes).
            'motor_speed': The speed of the motor (in RPM).
            'motor_angle': The angle of the motor shaft (in degrees).
    """
    def send_and_receive(command):
        # Create the data bytes for the command
        data = [command, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        
        # Create the CAN message
        msg = can.Message(
            arbitration_id=0x140 + motor_id,
            data=data,
            is_extended_id=False
        )
        
        # Send the message
        bus.send(msg)
        
        # Wait for the response
        response = bus.recv()
        
        # Verify if the response is from the expected motor and command
        if response.arbitration_id == (0x240 + motor_id) and response.data[0] == command:
            return response
        else:
            raise Exception("Did not receive a valid response from the motor")
    
    status_info = {}

    # Read Motor Status 1 and Error Flag Command (0x9A)
    response = send_and_receive(0x9A)
    status_info['motor_temperature'] = response.data[1]
    status_info['brake_control_command'] = response.data[3]
    status_info['voltage'] = (response.data[4] | (response.data[5] << 8)) * 0.1  # Convert to actual voltage
    status_info['error_state'] = response.data[6] | (response.data[7] << 8)

    # Read Motor Status 2 Command (0x9C)
    response = send_and_receive(0x9C)
    status_info['torque_current'] = (response.data[2] | (response.data[3] << 8)) * 0.01  # Convert to actual current
    status_info['motor_speed'] = response.data[4] | (response.data[5] << 8)
    status_info['motor_angle'] = response.data[6] | (response.data[7] << 8)
    
    return status_info

# Example usage
if __name__ == "__main__":
    # Configure the CAN bus
    bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=500000)
    motor_id = 1
    
    try:
        status_info = read_motor_status(bus, motor_id)
        print("Motor Status Information:")
        for key, value in status_info.items():
            print(f"{key}: {value}")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        bus.shutdown()

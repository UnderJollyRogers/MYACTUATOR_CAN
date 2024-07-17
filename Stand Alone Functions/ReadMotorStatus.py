import can

def read_motor_status(bus, motor_id):
    """
    Reads the motor status from three different status commands.

    Args:
        bus: The CAN bus used for communication.
        motor_id: The motor ID.

    Returns:
        A list containing the status information.
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
    
    status_commands = [0x9A, 0x9C]
    status_info = []
    
    for command in status_commands:
        response = send_and_receive(command)
        
        if command == 0x9A:
            # Read Motor Status 1 and Error Flag Command
            motor_temperature = response.data[1]
            brake_control_command = response.data[3]
            voltage = response.data[4] | (response.data[5] << 8)
            error_state = response.data[6] | (response.data[7] << 8)
            status_info.append({
                'motor_temperature': motor_temperature,
                'brake_control_command': brake_control_command,
                'voltage': voltage * 0.1,  # Convert to actual voltage
                'error_state': error_state
            })
        
        elif command == 0x9C:
            # Read Motor Status 2 Command
            motor_temperature = response.data[1]
            torque_current = response.data[2] | (response.data[3] << 8)
            motor_speed = response.data[4] | (response.data[5] << 8)
            motor_angle = response.data[6] | (response.data[7] << 8)
            status_info.append({
                'torque_current': torque_current * 0.01,  # Convert to actual current
                'motor_speed': motor_speed,
                'motor_angle': motor_angle
            })
    
    return status_info

# Example usage
if __name__ == "__main__":
    # Configure the CAN bus
    bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=500000)
    motor_id = 1
    
    try:
        status_info = read_motor_status(bus, motor_id)
        print("Motor Status Information:")
        for info in status_info:
            print(info)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        bus.shutdown()

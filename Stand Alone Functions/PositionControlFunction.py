import can

def absolute_position_control(bus, motor_id, max_speed_dps, angle_control, message = False):
    """
    Sends an absolute position closed-loop control command to the motor.

    Args:
        bus: The CAN bus used for communication.
        motor_id: The motor ID.
        max_speed_dps: The maximum speed in degrees per second (DPS).
        angle_control: The target angle for the motor position in degrees.

    Returns:
        success: Indicates whether the instruction was executed properly.
    """
    # Command for absolute position closed-loop control
    command = 0xA4
    
    # Convert max_speed_dps to bytes
    max_speed_bits = int(max_speed_dps)
    max_speed_bytes = max_speed_bits.to_bytes(2, 'little', signed=False)
    
    # Convert angle_control to bytes
    angle_control_bits = int(angle_control / 0.01)
    angle_control_bytes = angle_control_bits.to_bytes(4, 'little', signed=True)
    
    # Create the data bytes for the command
    data = [
        command, 0x00,
        max_speed_bytes[0], max_speed_bytes[1],
        angle_control_bytes[0], angle_control_bytes[1],
        angle_control_bytes[2], angle_control_bytes[3]
    ]
    
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
        if message:
            # position
            print('Reached desired position', )
    else:
        raise Exception("Did not receive a valid response from the motor")

# Example usage<Z
if __name__ == "__main__":
    # Configure the CAN bus
    bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)
    motor_id = 2
    max_speed_dps = 300*6  # Maximum speed in degrees per second
    angle_control = 180  # Target angle in degrees
    
    try:
        response = absolute_position_control(bus, motor_id, max_speed_dps, angle_control)
        print(f"Response from the motor: {response}")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        bus.shutdown()

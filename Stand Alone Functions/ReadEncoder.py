import can

def read_encoder_position(bus, motor_id, message = False):
    """
    Reads the multi-turn encoder position from the motor.

    Args:
        bus: The CAN bus used for communication.
        motor_id: The motor ID.
        message (optional): Displays information about the current state of the motor. The information includes torque (current in Amperes), motor speed (in degrees per second), and motor angle (in degrees).

    Returns:
        The encoder position value.
    """
    command = 0x60
    data = [command, 0x00, 0x00, 0x00]
        
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
        encoder_position = int.from_bytes(response.data[4:8], 'little', signed=True)
        if message:
            print('Encoder Position: ', encoder_position)
        return encoder_position
    else:
        raise Exception("Did not receive a valid response from the motor")

# Example usage
if __name__ == "__main__":
    # Configure the CAN bus with a bitrate of 1,000,000
    bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)
    motor_id = 2
    
    try:
        encoder_position = read_encoder_position(bus, motor_id)
        print(f"Encoder Position: {encoder_position}")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        bus.shutdown()
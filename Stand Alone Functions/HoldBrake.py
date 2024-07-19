import can

def system_brake_lock(bus, motor_id: int, message = False):
    """
    Sends a system brake lock command to the motor.

    Args:
        bus: The CAN bus used for communication.
        motor_id: The motor ID.
        message (optional): Boolean flag to indicate if a message should be printed or logged.

    Return: 
        success: Indicates whether the instruction was executed properly.
    """
    # Command for system brake lock
    command = 0x78
    
    # Create the data bytes for the command
    data = [
        command, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
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
            print('Motor locked')
    else:
        raise Exception("Did not receive a valid response from the motor")
    return True

# Example usage
if __name__ == "__main__":
    # Configure the CAN bus with a bitrate of 1,000,000
    bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)
    motor_id = 2
    
    try:
        response = system_brake_lock(bus, motor_id)
        print(f"Response from the motor: {response}")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        bus.shutdown()

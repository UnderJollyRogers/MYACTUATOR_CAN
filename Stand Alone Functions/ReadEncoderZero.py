import can

def read_encoder_zero_offset(bus, motor_id):
    """
    Reads the multi-turn encoder zero offset value from the motor.

    Args:
        bus: The CAN bus used for communication.
        motor_id: The motor ID.

    Returns:
        The encoder zero offset value.
    """
    def send_and_receive(command):
        # Create the data bytes for the command
        data = [command, 0x00, 0x00, 0x00]
        
        # Create the CAN message
        msg = can.Message(
            arbitration_id=0x140 + motor_id,
            data=data,
            is_extended_id=False
        )
        
        # Send the message/
        bus.send(msg)
        
        # Wait for the response
        response = bus.recv()
        
        # Verify if the response is from the expected motor and command
        if response.arbitration_id == (0x240 + motor_id) and response.data[0] == command:
            return response
        else:
            raise Exception("Did not receive a valid response from the motor")
    
    # Read Multi-turn Encoder Zero Offset Data Command (0x62)
    response = send_and_receive(0x62)
    
    # Extract the encoder zero offset (32-bit signed integer)
    encoder_zero_offset = int.from_bytes(response.data[4:8], 'little', signed=True)
    
    return encoder_zero_offset

# Example usage
if __name__ == "__main__":
    # Configure the CAN bus with a bitrate of 1,000,000
    bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)
    motor_id = 1
    
    try:
        encoder_zero_offset = read_encoder_zero_offset(bus, motor_id)
        print(f"Encoder Zero Offset: {encoder_zero_offset}")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        bus.shutdown()

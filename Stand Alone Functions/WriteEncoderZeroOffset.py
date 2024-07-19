import can
import SystemReset
import ReadMotorStatus

def write_encoder_zero_offset(bus, motor_id, encoder_offset):
    """
    Writes the encoder multi-turn value to ROM as motor zero offset.

    Args:
        bus: The CAN bus used for communication.
        motor_id: The motor ID.
        encoder_offset: The encoder zero offset value (32-bit signed integer).
    """
    # Command for writing encoder multi-turn value to ROM as motor zero
    command = 0x63
    
    # Convert encoder_offset to bytes
    encoder_offset_bytes = encoder_offset.to_bytes(4, 'little', signed=True)
    
    # Create the data bytes for the command
    data = [
        command, 0x00, 0x00, 0x00,
        encoder_offset_bytes[0], encoder_offset_bytes[1],
        encoder_offset_bytes[2], encoder_offset_bytes[3]
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
    if response.arbitration_id == (0x240 + motor_id) and response.data[0] == command:
        print("Encoder zero offset write command sent successfully.")
        EncoderOffset = int.from_bytes(response.data[4:8], 'little', signed=True)
        return EncoderOffset == encoder_offset
    else:
        raise Exception("Did not receive a valid response from the motor")
    

# Example usage
if __name__ == "__main__":
    # Configure the CAN bus with a bitrate of 1,000,000
    bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)
    motor_id = 2
    encoder_offset = 0  # Example encoder zero offset value
    
    try:
        Check = write_encoder_zero_offset(bus, motor_id, encoder_offset)
        print('It is working? ', Check)
        SystemReset.system_reset(bus, motor_id)
        status_info = ReadMotorStatus.read_motor_status(bus, motor_id)
        print("Motor Status Information:")
        for key, value in status_info.items():
            if key == 'motor_speed':
                value = value / 6
            print(f"{key}: {value}")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        bus.shutdown()

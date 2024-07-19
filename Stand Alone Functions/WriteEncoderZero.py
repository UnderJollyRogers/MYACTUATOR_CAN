import can
import SystemReset
import ReadMotorStatus
import time
def write_current_encoder_position_to_rom(bus, motor_id):
    """
    Writes the current multi-turn position of the encoder to the ROM as motor zero.  For the changes take effect, run SystemReset.py

    Args:
        bus: The CAN bus used for communication.
        motor_id: The motor ID.
    """
    # Command for writing the current multi-turn position of the encoder to ROM as motor zero
    command = 0x64
    
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
    if response.arbitration_id == (0x240 + motor_id) and response.data[0] == command:
        print("Current encoder position write command sent successfully.")
        EncoderOffset = int.from_bytes(response.data[4:8], 'little', signed=True)
        return EncoderOffset
    else:
        raise Exception("Did not receive a valid response from the motor")
    

# Example usage
if __name__ == "__main__":
    # Configure the CAN bus with a bitrate of 1,000,000
    bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)
    motor_id = 2
    
    try:
        EncoderPosition = write_current_encoder_position_to_rom(bus, motor_id)
        print('New encoder offset: ', EncoderPosition)
        SystemReset.system_reset(bus, motor_id)
        bus.shutdown
        time.sleep(1)
        bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)
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

import can

def system_reset(bus, motor_id):
    """
    Sends a system reset command to the motor.

    Args:
        bus: The CAN bus used for communication.
        motor_id: The motor ID.
    """
    # Command for system reset
    command = 0x76
    
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
    
    print("System reset command sent successfully.")

# Example usage
if __name__ == "__main__":
    # Configure the CAN bus with a bitrate of 1,000,000
    bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)
    motor_id = 2
    
    try:
        system_reset(bus, motor_id)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        bus.shutdown()

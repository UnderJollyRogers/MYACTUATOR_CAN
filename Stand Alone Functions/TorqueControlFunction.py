import can

def torque_closed_loop_control(bus, motor_id, iqControl):
    """
    Sends a torque closed-loop control command to the motor.

    Args:
        bus: The CAN bus used for communication.
        motor_id: The motor ID.
        iqControl: The control current in Amperes.

    Returns:
        Response from the motor.
    """
    # Command for torque closed-loop control
    command = 0xA1
    
    # Convert iqControl to bits (0.01 A per bit)
    iqControl_bits = int(iqControl / 0.01)
    iqControl_bytes = iqControl_bits.to_bytes(2, 'little', signed=True)
    
    # Create the data bytes for the command
    data = [
        command, 0x00, 0x00, 0x00,
        iqControl_bytes[0], iqControl_bytes[1],
        0x00, 0x00
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
        return response
    else:
        raise Exception("Did not receive a valid response from the motor")

# Example usage
if __name__ == "__main__":
    # Configure the CAN bus
    bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=500000)
    motor_id = 1
    iqControl = 0 # Control current in Amperes
    
    try:
        response = torque_closed_loop_control(bus, motor_id, iqControl)
        print(f"Response from the motor: {response}")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        bus.shutdown()

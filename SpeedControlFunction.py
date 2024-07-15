import time
import can

class SpeedLimitExceededError(Exception):
    pass

def speed_closed_loop_control(bus, motor_id, target_speed_rpm):
    """
    Controls the motor speed in closed-loop mode.

    Args:
        bus: The CAN bus used for communication.
        motor_id: The motor ID.
        target_speed_rpm: The target speed in RPM.

    Returns:
        The actual motor speed in RPM.
    """
    # Speed limit in RPM
    max_speed_rpm = 300

    # Check if the target speed exceeds the limit
    if abs(target_speed_rpm) > max_speed_rpm:
        raise SpeedLimitExceededError(f"Target speed {target_speed_rpm} RPM exceeds the limit of {max_speed_rpm} RPM.")

    # Convert the target speed to DPS (degrees per second)
    target_speed_dps = target_speed_rpm * 6

    # Convert the target speed to bits (0.01 DPS per bit)
    target_speed_bits = int(target_speed_dps / 0.01)
    speed_bytes = target_speed_bits.to_bytes(4, 'little', signed=True)
    
    # Create the CAN message
    msg = can.Message(
        arbitration_id=0x140 + motor_id,
        data=[0xA2, 0x00, 0x00, 0x00] + list(speed_bytes),
        is_extended_id=False
    )
    print('speed_bytes:', list(speed_bytes))
    print('data:', [0xA2, 0x00, 0x00, 0x00] + list(speed_bytes))
    
    # Send the message
    bus.send(msg)
    
    # Wait for the response
    response = bus.recv()
    
    # Check if the response is from the expected motor
    if response.arbitration_id == (0x240 + motor_id) and response.data[0] == 0xA2:
        actual_speed_bits = int.from_bytes(response.data[4:6], 'little', signed=True)
        print('actual_bits:', actual_speed_bits)
        actual_speed_dps = actual_speed_bits  # Already in DPS
        # Convert the speed from DPS to RPM
        actual_speed_rpm = actual_speed_dps / 6
        return actual_speed_rpm
    else:
        raise Exception("Did not receive a valid response from the motor")

# Example usage
if __name__ == "__main__":
    # Configure the CAN bus
    bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=500000)
    motor_id = 1
    target_speed_rpm = 1000  # RPM
    
    try:
        actual_speed_rpm = speed_closed_loop_control(bus, motor_id, target_speed_rpm)
        print(f"The actual motor speed is: {actual_speed_rpm:.2f} RPM")
    except SpeedLimitExceededError as e:
        print(f"Error: {e}")
    finally:
        bus.shutdown()

import can
import time

class SpeedLimitExceededError(Exception):
    pass

class can_motor:
    """
    CAN motor controller class.
    This class provides a structure to interface with MyActuator.

    Attributes:
        motor_id (int): Motor ID on the CAN bus (1–127 typical range).
        gear_ratio (float): Reduction ratio of the motor gearbox.
        bus (can.Bus): python-can Bus instance for communication.
    """
    try:
        channel = 'can0'
        bitrate = 1000000
        bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000)
        print(f"[INFO] CAN interface initialized on {channel} @ {bitrate/1000} kbps")
    except Exception as e:
        print(f"[ERROR] Failed to initialize CAN interface: {e}")
        bus = None

    def __init__(self, motor_id: int, max_speed=1800):
        """
        Initialize CAN interface for a motor.

        Args:
            motor_id (int): Unique motor ID on the CAN network.
            max_speed (int): Max speed in degrees per second.
            channel (str): CAN interface channel (e.g., 'can0').
            bitrate (int): CAN bus speed (default 1 Mbps).
        """
        self.motor_id = motor_id
        self.max_speed= max_speed
        self.joint_position = 0

    def send_message(self, data: bytes):
        """
        Send a CAN message to the motor.

        Args:
            data (bytes): Data payload (up to 8 bytes).
        """
        if not can_motor.bus:
            print("[WARN] CAN bus not initialized.")
            return

        message = can.Message(
            arbitration_id=0x140 + self.motor_id,  # Standard MyActuator convention
            data=data,
            is_extended_id=False
        )

        try:
            can_motor.bus.send(message)
        except can.CanError as e:
            print(f"[ERROR] Failed to send message: {e}")

    def receive_message(self, timeout: float = 0.1):
        """
        Receive a CAN message.

        Args:
            timeout (float): Wait time for a message (seconds).

        Returns:
            can.Message or None
        """
        msg = can_motor.bus.recv(timeout)
        return msg

    def absolute_position_control(self, angle_control):
        """
        Sends an absolute position closed-loop control command to the motor.

        Args:
            bus: The CAN bus used for communication.
            motor_id: The motor ID.
            angle_control: The target angle for the motor position in degrees.
        Returns:
            success : Indicates whether the instruction was executed properly.
        """
        # Command for absolute position closed-loop control
        command = 0xA4
        
        # Convert max_speed_dps to bytes
        max_speed_bits = int(self.max_speed)
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
        self.send_message(data)
        msg = self.receive_message()
    
        # Verify if the response is from the expected motor and command
        if msg.arbitration_id == (0x240 + self.motor_id) and msg.data[0] == command:
            success = True
            self.joint_position = int.from_bytes(msg.data[6:8], 'little', signed=True)
        else:
            raise Exception("Did not receive a valid response from the motor")
        return success

    def speed_closed_loop_control(self, target_speed_rpm):
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
        max_speed_rpm = 500
        # Check if the target speed exceeds the limit
        if abs(target_speed_rpm) > max_speed_rpm:
            raise SpeedLimitExceededError(f"Target speed {target_speed_rpm} RPM exceeds the limit of {max_speed_rpm} RPM.")
        # Convert the target speed to DPS (degrees per second)
        target_speed_dps = target_speed_rpm * 6
        # Convert the target speed to bits (0.01 DPS per bit)
        target_speed_bits = int(target_speed_dps / 0.01)
        speed_bytes = target_speed_bits.to_bytes(4, 'little', signed=True)
        data=[0xA2, 0x00, 0x00, 0x00] + list(speed_bytes)
        self.send_message(data)
        msg = self.receive_message()
        # Check if the response is from the expected motor
        if msg.arbitration_id == (0x240 + self.motor_id) and msg.data[0] == 0xA2:
            actual_speed_bits = int.from_bytes(msg.data[4:6], 'little', signed=True)
            actual_speed_dps = actual_speed_bits  # Already in DPS
            # Convert the speed from DPS to RPM
            actual_speed_rpm = actual_speed_dps / 6
            return actual_speed_rpm
        else:
            raise Exception("Did not receive a valid response from the motor")
    def shutdown_motor(self) -> bool:
        """
        Sends the Motor Shutdown command (0x80).

        Description:
            Turns off the motor output and clears its running state.
            The reply frame from the motor is identical to the sent data.

        Returns:
            success (bool): True if the response matches the sent frame.
        """
        command = 0x80
        data = [command] + [0x00] * 7  # [0x80, 0x00, ..., 0x00]

        # Send command
        self.send_message(data)
        msg = self.receive_message()

        # Validate response
        if msg and msg.arbitration_id == (0x240 + self.motor_id):
            # The response frame should be identical to what was sent
            if list(msg.data)[:8] == data:
                print(f"[Motor {self.motor_id}] Shutdown acknowledged.")
                return True
            else:
                print(f"[Motor {self.motor_id}] Received invalid shutdown response: {msg.data.hex(' ')}")
                return False
        else:
            raise Exception(f"[Motor {self.motor_id}] No valid response for shutdown command.")
    def read_acceleration_parameters(self):
        """
        Reads the acceleration parameters (0x42) for all 4 indices:
            0x00 - Position Planning Acceleration
            0x01 - Position Planning Deceleration
            0x02 - Speed Planning Acceleration
            0x03 - Speed Planning Deceleration

        The function prints each acceleration value (in dps/s).
        """
        command = 0x42
        indices = {
            0x00: "Position Planning Acceleration",
            0x01: "Position Planning Deceleration",
            0x02: "Speed Planning Acceleration",
            0x03: "Speed Planning Deceleration"
        }

        for idx, label in indices.items():
            # Construct data packet for this index
            data = [command, idx] + [0x00] * 6

            # Send command
            self.send_message(data)
            msg = self.receive_message()

            # Validate response
            if msg and msg.arbitration_id == (0x240 + self.motor_id) and msg.data[0] == command:
                accel_bytes = msg.data[4:8]
                accel_val = int.from_bytes(accel_bytes, byteorder='little', signed=False)
                print(f"[Motor {self.motor_id}] {label}: {accel_val} dps/s")
            else:
                print(f"[Motor {self.motor_id}] No valid response for {label}")
    
    def write_acceleration_parameters(self, acceleration_value: int):
        """
        Writes the same acceleration value to all four function indices (0x00–0x03)
        and then shuts down the motor.

        Args:
            acceleration_value (int): Acceleration in dps/s (range 100–60000).
        """
        command = 0x43
        indices = [0x00, 0x01, 0x02, 0x03]

        # Validate range
        if not (100 <= acceleration_value <= 60000):
            raise ValueError(f"Acceleration value {acceleration_value} out of range (10–60000 dps/s).")

        # Convert acceleration to 4 bytes (little endian)
        accel_bytes = acceleration_value.to_bytes(4, 'little', signed=False)

        for idx in indices:
            data = [command, idx, 0x00, 0x00] + list(accel_bytes)

            # Send command
            self.send_message(data)
            msg = self.receive_message()

            # Validate response
            if msg and msg.arbitration_id == (0x240 + self.motor_id) and list(msg.data[:8]) == data:
                print(f"[Motor {self.motor_id}] Acceleration (index 0x{idx:02X}) set to {acceleration_value} dps/s.")
            else:
                print(f"[Motor {self.motor_id}] No valid response for index 0x{idx:02X}.")

        # Safety shutdown after update
        print(f"[Motor {self.motor_id}] Writing completed. Sending shutdown command...")
        self.shutdown_motor()

    def system_reset(self):
        """
        Sends the System Reset command (0x76) to the motor.
        
        Description:
            This command resets the motor's internal program.
            The motor does NOT send a reply frame, as it immediately reboots.

        Returns:
            None
        """
        command = 0x76
        data = [command] + [0x00] * 7  # [0x76, 0x00, ..., 0x00]

        # Send reset command
        self.send_message(data)
        print(f"[Motor {self.motor_id}] System reset command sent. The motor will reboot shortly.")

        # No response expected per protocol
        # Optionally, add a small delay to allow reset
        time.sleep(1)

    def read_multi_turn_angle(self) -> float:
        """
        Reads the multi-turn absolute angle of the motor (0x92).

        Description:
            This command reads the current multi-turn absolute position of the motor.
            The returned angle is a signed 32-bit value, with units of 0.01° per bit (LSB).

        Returns:
            angle_deg (float): Absolute multi-turn motor angle in degrees.
        """
        command = 0x92
        data = [command] + [0x00] * 7  # [0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

        # Send command
        self.send_message(data)
        msg = self.receive_message()

        # Validate response
        if msg and msg.arbitration_id == (0x240 + self.motor_id) and msg.data[0] == command:
            angle_bytes = msg.data[4:8]
            angle_bits = int.from_bytes(angle_bytes, byteorder="little", signed=True)

            # Each LSB = 0.01 degree
            angle_deg = angle_bits * 0.01

            print(f"[Motor {self.motor_id}] Multi-turn absolute angle: {angle_bits} (→ {angle_deg:.2f}°)")
            return angle_deg
        else:
            raise Exception(f"[Motor {self.motor_id}] Invalid response for multi-turn angle read.")

    def read_single_turn_angle(self) -> float:
        """
        Reads the single-turn angle of the motor (0x94).

        Description:
            This command reads the current mechanical angle of the motor shaft
            within a single revolution (0°–360°). The data is a 16-bit unsigned integer,
            where each LSB corresponds to 0.01°.

        Returns:
            angle_deg (float): Current single-turn angle in degrees.
        """
        command = 0x94
        data = [command] + [0x00] * 7  # [0x94, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

        # Send command
        self.send_message(data)
        msg = self.receive_message()

        # Validate response
        if msg and msg.arbitration_id == (0x240 + self.motor_id) and msg.data[0] == command:
            # Extract 16-bit unsigned single-turn angle
            angle_bits = int.from_bytes(msg.data[6:8], byteorder="little", signed=False)

            # Each bit = 0.01 degrees
            angle_deg = angle_bits * 0.01

            print(f"[Motor {self.motor_id}] Single-turn angle: {angle_bits} → {angle_deg:.2f}°")
            return angle_deg
        else:
            raise Exception(f"[Motor {self.motor_id}] Invalid response for single-turn angle read.")

    def write_current_position_as_zero(self):
        """
        Writes the current multi-turn encoder position as the new motor zero (0x64).

        Description:
            Sets the current motor position as the zero reference and writes it to ROM.
            After writing, the motor must be reset (0x76) for the change to take effect.

        Returns:
            success (bool): True if the command was acknowledged correctly.
        """
        command = 0x64
        data = [command] + [0x00] * 7  # [0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

        # Send command
        self.send_message(data)
        msg = self.receive_message()

        # Validate response
        if msg and msg.arbitration_id == (0x240 + self.motor_id) and msg.data[0] == command:
            # Extract 4-byte encoder offset (confirmation)
            offset_bytes = msg.data[4:8]
            offset_val = int.from_bytes(offset_bytes, byteorder="little", signed=True)

            print(f"[Motor {self.motor_id}] New zero position written to ROM.")
            print(f"[Motor {self.motor_id}] Encoder offset confirmation: {offset_val}")
            self.joint_position = 0
            self.system_reset()

            return True
        else:
            raise Exception(f"[Motor {self.motor_id}] No valid response for zero position write.")
    def stop_motor(self) -> bool:
        """
        Sends the Motor Stop command (0x81).

        Description:
            Stops the motor motion while keeping the motor enabled.
            The motor remains in closed-loop mode but with zero speed.
            The response frame should match the command sent.

        Returns:
            success (bool): True if the command was acknowledged correctly.
        """
        command = 0x81
        data = [command] + [0x00] * 7  # [0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

        # Send command
        self.send_message(data)
        msg = self.receive_message()

        # Validate response (should echo the same frame)
        if msg and msg.arbitration_id == (0x240 + self.motor_id):
            if list(msg.data[:8]) == data:
                print(f"[Motor {self.motor_id}] Motor stopped successfully.")
                return True
            else:
                print(f"[Motor {self.motor_id}] Received invalid stop response: {msg.data.hex(' ')}")
                return False
        else:
            raise Exception(f"[Motor {self.motor_id}] No valid response for stop command.")


    @staticmethod
    def close():
        """Safely close the CAN interface."""
        if can_motor.bus:
            can_motor.bus.shutdown()
            print("[INFO] CAN bus closed.")

# Example usage
if __name__ == "__main__":
    pos = 0
    motor1 = can_motor(motor_id=1, max_speed=1800)
    motor1.absolute_position_control(0)
    can_motor.close()

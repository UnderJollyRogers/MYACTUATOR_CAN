import can
import struct

class CAN:
    """
    Generic CAN motor controller class.
    This class provides a structure to interface with MyActuator or other CAN-based motors.

    Attributes:
        motor_id (int): Motor ID on the CAN bus (1–127 typical range).
        gear_ratio (float): Reduction ratio of the motor gearbox.
        bus (can.Bus): python-can Bus instance for communication.
    """

    def __init__(self, motor_id: int, gear_ratio: float, channel: str = 'can0', bitrate: int = 1000000):
        """
        Initialize CAN interface for a motor.

        Args:
            motor_id (int): Unique motor ID on the CAN network.
            gear_ratio (float): Transmission ratio.
            channel (str): CAN interface channel (e.g., 'can0').
            bitrate (int): CAN bus speed (default 1 Mbps).
        """
        self.motor_id = motor_id
        self.gear_ratio = gear_ratio
        self.channel = channel
        self.bitrate = bitrate

        # Initialize CAN bus
        try:
            self.bus = can.interface.Bus(channel=self.channel, bustype='socketcan', bitrate=self.bitrate)
            print(f"[INFO] CAN interface initialized on {self.channel} @ {self.bitrate/1000} kbps")
        except Exception as e:
            print(f"[ERROR] Failed to initialize CAN interface: {e}")
            self.bus = None

    # ---------------------------------------------------------
    # 🧩 Basic communication methods
    # ---------------------------------------------------------
    def send_message(self, data: bytes):
        """
        Send a CAN message to the motor.

        Args:
            data (bytes): Data payload (up to 8 bytes).
        """
        if not self.bus:
            print("[WARN] CAN bus not initialized.")
            return

        message = can.Message(
            arbitration_id=0x140 + self.motor_id,  # Standard MyActuator convention
            data=data,
            is_extended_id=False
        )

        try:
            self.bus.send(message)
            print(f"[TX] Sent to 0x{message.arbitration_id:X}: {data.hex(' ')}")
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
        if not self.bus:
            print("[WARN] CAN bus not initialized.")
            return None

        msg = self.bus.recv(timeout)
        if msg:
            print(f"[RX] From 0x{msg.arbitration_id:X}: {msg.data.hex(' ')}")
        return msg

    # ---------------------------------------------------------
    # 🧱 Placeholder for future functions
    # ---------------------------------------------------------
    def set_speed(self, rpm: float):
        """Set motor speed (to be implemented)."""
        pass

    def read_position(self):
        """Read motor position (to be implemented)."""
        pass

    def stop_motor(self):
        """Stop the motor (to be implemented)."""
        pass

    def close(self):
        """Safely close the CAN interface."""
        if self.bus:
            self.bus.shutdown()
            print("[INFO] CAN bus closed.")

# Example usage
if __name__ == "__main__":
    motor = CAN(motor_id=1, gear_ratio=9.0)
    motor.send_message(b'\x01\x02\x03\x04\x05\x06\x07\x08')
    response = motor.receive_message()
    motor.close()

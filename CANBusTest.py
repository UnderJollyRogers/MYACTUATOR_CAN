import time
import can

bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=500000)

msg = can.Message(arbitration_id=0x141, data=[0xA2, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00], 
                  is_extended_id=False)

speed_decimal = 0x10 | (0x27 << 8) | (0x00 << 16) | (0x00 << 24)
RPM_actual = speed_decimal * 0.01 / 6
print(RPM_actual)

bus.send(msg)
receiveMSG = bus.recv()
print(receiveMSG)
bus.shutdown()

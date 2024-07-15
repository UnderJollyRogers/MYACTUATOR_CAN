import time
import can

class SpeedLimitExceededError(Exception):
    pass

def speed_closed_loop_control(bus, motor_id, target_speed_rpm):
    """
    Controla la velocidad en bucle cerrado del motor.

    Args:
        bus: El bus CAN utilizado para la comunicación.
        motor_id: El ID del motor.
        target_speed_rpm: La velocidad objetivo en RPM.

    Returns:
        La velocidad actual del motor en RPM.
    """
    # Límite de velocidad en RPM
    max_speed_rpm = 300

    # Verificar si la velocidad objetivo excede el límite
    if target_speed_rpm > max_speed_rpm:
        raise SpeedLimitExceededError(f"La velocidad objetivo {target_speed_rpm} RPM excede el límite de {max_speed_rpm} RPM.")

    # Convertir la velocidad objetivo a DPS (grados por segundo)
    target_speed_dps = target_speed_rpm * 6

    # Convertir la velocidad objetivo a bits (0.01 DPS por bit)
    target_speed_bits = int(target_speed_dps / 0.01)
    speed_bytes = target_speed_bits.to_bytes(4, 'little', signed=True)
    
    # Crear el mensaje CAN
    msg = can.Message(
        arbitration_id=0x140 + motor_id,
        data=[0xA2, 0x00, 0x00, 0x00] + list(speed_bytes),
        is_extended_id=False
    )
    print('speed_bytes:', list(speed_bytes))
    print('data:', [0xA2, 0x00, 0x00, 0x00] + list(speed_bytes),)
    # Enviar el mensaje
    bus.send(msg)
    
    # Esperar la respuesta
    response = bus.recv()
    
    # Verificar si la respuesta es del motor esperado
    if response.arbitration_id == (0x240 + motor_id) and response.data[0] == 0xA2:
        actual_speed_bits = int.from_bytes(response.data[4:6], 'little', signed=False)
        print('actual_bits:', actual_speed_bits)
        actual_speed_dps = actual_speed_bits
        # Convertir la velocidad en DPS a RPM
        actual_speed_rpm = actual_speed_dps / 6
        return actual_speed_rpm
    else:
        raise Exception("No se recibió una respuesta válida del motor")

# Ejemplo de uso
if __name__ == "__main__":
    # Configuración del bus CAN
    bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=500000)
    motor_id = 1
    target_speed_rpm = 0  # RPM
    
    try:
        actual_speed_rpm = speed_closed_loop_control(bus, motor_id, target_speed_rpm)
        print(f"La velocidad actual del motor es: {actual_speed_rpm:.2f} RPM")
    except SpeedLimitExceededError as e:
        print(f"Error: {e}")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        bus.shutdown()

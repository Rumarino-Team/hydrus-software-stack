import serial
import struct
import time

class DVLProtocol:
    def __init__(self):
        pass

    @staticmethod
    def calculate_checksum(packet: bytes) -> int:
        """Calculate checksum for a given packet (sum of non-checksum bytes, ignoring overflow)."""
        return sum(packet) & 0xFFFF

    @staticmethod
    def create_packet(command_id: bytes, payload: bytes = b'') -> bytes:
        """Create a packet with Start of Packet, Command ID, Payload, and Checksum."""
        sop_id = b'\xAA\x10'  # Start of Packet Identifier
        packet = sop_id + command_id + payload
        checksum = DVLProtocol.calculate_checksum(packet)
        return packet + checksum.to_bytes(2, 'little')

    def get_system_command(self):
        """Send 'Get System' command to query system configuration."""
        command_id = b'\0xAA10 010F 0002'
        return self.create_packet(command_id)

    def get_setup_command(self):
        """Send 'Get Setup' command to query deployment ping configuration."""
        command_id = b'\x03\x08\x00\x01\x00\x00\x85'
        return self.create_packet(command_id)

    def set_setup_command(self, sos: float, max_track_range: float):
        """Send 'Set Setup' command to configure the DVL settings."""
        command_id = b'\x03\x1C\x00\x02\x00\x00\x87'
        payload = b'\x22\x10\x14\x00\x00'  # Structure ID
        payload += b'\x01'  # Software trigger enabled
        payload += b'\x07'  # Baud rate: 115200
        payload += int(sos).to_bytes(4, 'little')  # Speed of sound (in m/s)
        payload += int(max_track_range).to_bytes(4, 'little')  # Max depth
        return self.create_packet(command_id, payload)

    def software_trigger_command(self):
        """Send 'Software Trigger' command to ping the DVL if software trigger is enabled."""
        command_id = b'\x03\x08\x00\x11\x00\x00\x00'
        return self.create_packet(command_id)

    def speed_of_sound_command(self, sos: float):
        """Set the speed of sound for the next ping."""
        command_id = b'\x03\x0C\x00\x03\x00\x00\x86'
        payload = int(sos).to_bytes(4, 'little')
        return self.create_packet(command_id, payload)

    def get_time_command(self):
        """Send 'Get Time' command to query the current time."""
        command_id = b'\x03\x08\x00\x01\x00\x00\x1D'
        return self.create_packet(command_id)

    def set_time_command(self, year: int, month: int, day: int, hour: int, minute: int, second: int):
        """Send 'Set Time' command to configure the DVL's internal time."""
        command_id = b'\x03\x14\x00\x02\x00\x00\x1F'
        payload = b'\x23\x10\x0C\x00\x00'  # Structure ID
        payload += year.to_bytes(1, 'little')
        payload += month.to_bytes(1, 'little')
        payload += day.to_bytes(1, 'little')
        payload += hour.to_bytes(1, 'little')
        payload += minute.to_bytes(1, 'little')
        payload += second.to_bytes(1, 'little')
        return self.create_packet(command_id, payload)

# Código para usar la clase DVLProtocol
def main():
    # Configura la conexión serial
    with serial.Serial('COM5', baudrate=115200, timeout=1) as ser:
        print("Port opened")
        dvl = DVLProtocol()

        # Envía el comando "Get System"
        packet = dvl.get_setup_command()
        ser.write(packet)

        # Espera un momento antes de leer la respuesta
        time.sleep(0.1)

        # Lee la respuesta
        response = ser.read(128)

        print(f"Longitud de la respuesta: {len(response)}")

        if len(response) >= 17:  # Verifica que la respuesta sea suficiente
            sop_id, response_id, status_major, status_minor, checksum = struct.unpack('<6B7B2H', response[:17])
            print(f"Response SOP ID: {sop_id}, Status: {status_major}-{status_minor}")
        else:
            print("Respuesta incompleta o no válida.")

if __name__ == "__main__":
    main()

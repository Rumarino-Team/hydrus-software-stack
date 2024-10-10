import serial
import struct

# Configura la conexión serial
ser = serial.Serial('COM5', baudrate=115200, timeout=1)

# Paquete para Get System Command (SOP ID + Command ID + Checksum)
get_system_command = struct.pack('<6B7B2H', 0xAA, 0x10, 0x01, 0x0F, 0x00, 0x02,  # SOP ID
                                       0x03, 0x08, 0x00, 0x01, 0x00, 0x00, 0x81,  # Command ID
                                       0x59, 0x01)  # Checksum

# Envía el comando al DVL
ser.write(get_system_command)

# Lee la respuesta
response = ser.read(64)  # Lee 64 bytes de la respuesta

# Verifica la longitud de la respuesta
print(f"Longitud de la respuesta: {len(response)}")
if len(response) >= 17:  # Asegúrate de que hay suficientes bytes
    sop_id, response_id, status_major, status_minor, checksum = struct.unpack('<6B7B2H', response[:17])
    print(f"Response SOP ID: {sop_id}, Status: {status_major}-{status_minor}")
else:
    print("Respuesta incompleta o no válida.")

ser.close()

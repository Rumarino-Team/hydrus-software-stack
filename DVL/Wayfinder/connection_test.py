import serial


def read_dvl_data(port, baudrate=115200):
    try:
        with serial.Serial(port, baudrate, timeout=2) as ser:
            print(f"Connected to {port} at {baudrate} baud.")

            # Verifica si el puerto está abierto
            if ser.is_open:
                while True:
                    # Lee una línea de datos
                    data = ser.readline()
                    if data:
                        print(f"Received data: {data.decode('utf-8', errors='ignore').strip()}")
                    else:
                        print("No data received.")
                        break

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    DVL_PORT = 'COM5'
    BAUD_RATE = 115200

    read_dvl_data(DVL_PORT, BAUD_RATE)

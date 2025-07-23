#!/usr/bin/env python3
import serial
import time
import os

CONTROL_PORT = "/dev/hydrus_control"

BAUD_RATES = [
    300, 1200, 2400, 4800, 9600,
    14400, 19200, 38400, 57600,
    74880, 115200, 128000, 250000,
    500000, 1000000, 2000000
]

TEST_COMMAND = "T1:1500\n"

def test_baud_rate(port, baud_rate):
    try:
        print(f"Testing baud rate: {baud_rate}")
        with serial.Serial(port=port, baudrate=baud_rate, timeout=2) as ser:
            time.sleep(2)  
            ser.reset_input_buffer()
            ser.write(TEST_COMMAND.encode("utf-8"))

            # Read posible response
            response = ser.readline().decode("utf-8").strip()
            if response:
                print(f"Baud rate {baud_rate} worked, Response: {response}")
                return True
            else:
                print(f"Baud rate {baud_rate} gave no response")
                return False

    except serial.SerialException as e:
        print(f"Error with baud rate {baud_rate}: {e}")
        return False

def main():
    if not os.path.exists(CONTROL_PORT):
        print(f"Port {CONTROL_PORT} not found")
        return

    print("Starting baud rate test\n")
    working_rates = []

    for baud in BAUD_RATES:
        if test_baud_rate(CONTROL_PORT, baud):
            working_rates.append(baud)

    print("\nTest Result:")
    if working_rates:
        print("Baud rates that responded corrctly:")
        for br in working_rates:
            print(f"   - {br}")
    else:
        print("No working baud rates found")

if __name__ == "__main__":
    main()

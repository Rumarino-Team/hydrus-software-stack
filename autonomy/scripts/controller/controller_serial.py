#!/usr/bin/env python3
import serial
import time
import argparse
import threading
import sys

class HydrusSerialController:
    """
    Controller for Hydrus submarine via serial communication.
    Sends commands in the format required by the Arduino sketch.
    """
    
    def __init__(self, port="/dev/ttyACM0", baud_rate=115200, timeout=1):
        """Initialize the serial controller with specified port parameters."""
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.ser = None
        self.is_connected = False
        self.response_thread = None
        self.running = False
        
    def connect(self):
        """Establish a serial connection to the Arduino."""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=self.timeout
            )
            self.is_connected = True
            print(f"Connected to {self.port} at {self.baud_rate} baud")
            
            # Allow the Arduino time to reset after establishing a connection
            time.sleep(2)
            
            # Start the response reading thread
            self.running = True
            self.response_thread = threading.Thread(target=self._read_response)
            self.response_thread.daemon = True
            self.response_thread.start()
            
            return True
        except serial.SerialException as e:
            print(f"Failed to connect to {self.port}: {str(e)}")
            return False
    
    def disconnect(self):
        """Close the serial connection."""
        self.running = False
        if self.response_thread:
            self.response_thread.join(timeout=1)
        
        if self.is_connected and self.ser:
            self.ser.close()
            self.is_connected = False
            print(f"Disconnected from {self.port}")
    
    def _send_command(self, cmd):
        """Send a command to the Arduino."""
        if not self.is_connected or not self.ser:
            print("Not connected to Arduino!")
            return False
        
        # Add newline terminator to the command
        full_cmd = f"{cmd}\n"
        
        try:
            # Write command to serial port
            self.ser.write(full_cmd.encode('utf-8'))
            self.ser.flush()
            print(f"Sent: {cmd}")
            return True
        except serial.SerialException as e:
            print(f"Error sending command: {str(e)}")
            return False
    
    def _read_response(self):
        """Continuously read and print responses from the Arduino."""
        while self.running and self.is_connected:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line:
                        print(f"Arduino: {line}")
            except Exception as e:
                print(f"Error reading response: {str(e)}")
                break
            time.sleep(0.1)
    
    # Thruster control methods
    def set_thruster(self, thruster_num, value):
        """Set a specific thruster to the given value."""
        if not 1 <= thruster_num <= 4:
            print(f"Invalid thruster number: {thruster_num}. Must be 1-4.")
            return False
        return self._send_command(f"T{thruster_num}:{value}")
    
    def set_depth_motors(self, value):
        """Set depth motors to the given value."""
        return self._send_command(f"D:{value}")
    
    def launch_torpedo(self, value):
        """Launch torpedo with the given value."""
        return self._send_command(f"P:{value}")
    
    def set_camera_angle(self, angle):
        """Set camera to the specified angle (-60 to 60 degrees)."""
        if not -60 <= angle <= 60:
            print(f"Invalid camera angle: {angle}. Must be -60 to 60.")
            return False
        return self._send_command(f"C:{angle}")


def interactive_control(controller):
    """Interactive console for controlling the submarine."""
    print("\n=== Hydrus Serial Controller ===")
    print("Commands: ")
    print("  t1 <value>  - Set thruster 1 (-9 to 9)")
    print("  t2 <value>  - Set thruster 2 (-9 to 9)")
    print("  t3 <value>  - Set thruster 3 (-9 to 9)")
    print("  t4 <value>  - Set thruster 4 (-9 to 9)")
    print("  d <value>   - Set depth motors (-9 to 9)")
    print("  p <value>   - Launch torpedo (0 or 1)")
    print("  c <value>   - Set camera angle (-60 to 60)")
    print("  quit        - Exit the program")
    
    while True:
        try:
            cmd = input("\nEnter command: ").strip().lower()
            
            if cmd == "quit" or cmd == "exit":
                break
            
            parts = cmd.split()
            if len(parts) != 2:
                print("Invalid command format. Use: <command> <value>")
                continue
            
            command, value = parts
            
            try:
                value = int(value)
            except ValueError:
                try:
                    value = float(value)
                except ValueError:
                    print("Value must be a number")
                    continue
            
            if command == "t1":
                controller.set_thruster(1, value)
            elif command == "t2":
                controller.set_thruster(2, value)
            elif command == "t3":
                controller.set_thruster(3, value)
            elif command == "t4":
                controller.set_thruster(4, value)
            elif command == "d":
                controller.set_depth_motors(value)
            elif command == "p":
                controller.launch_torpedo(value)
            elif command == "c":
                controller.set_camera_angle(value)
            else:
                print(f"Unknown command: {command}")
        
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error: {str(e)}")


def main():
    parser = argparse.ArgumentParser(description='Hydrus Serial Controller')
    parser.add_argument('-p', '--port', default='/dev/ttyACM0', help='Serial port (default: /dev/ttyACM0)')
    parser.add_argument('-b', '--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    args = parser.parse_args()
    
    controller = HydrusSerialController(port=args.port, baud_rate=args.baud)
    
    if not controller.connect():
        print("Failed to connect. Exiting.")
        return 1
    
    try:
        interactive_control(controller)
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        controller.disconnect()
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
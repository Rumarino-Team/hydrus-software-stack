#!/bin/bash
# This script sets up socat to create virtual serial ports for monitoring Arduino

# Default serial port
ARDUINO_PORT="/dev/ttyACM0"
BAUD_RATE=57600

# Check if a different port is provided as argument
if [ "$1" != "" ]; then
  ARDUINO_PORT=$1
fi

# Check if socat is installed
if ! command -v socat &> /dev/null; then
  echo "Installing socat..."
  apt-get update && apt-get install -y socat
fi

# Create virtual serial ports
echo "Creating virtual serial ports for Arduino monitoring..."

# Check if proper permissions exist for the Arduino port
if [ ! -w "$ARDUINO_PORT" ]; then
  echo "Error: Cannot write to $ARDUINO_PORT"
  echo "Make sure the port exists and you have permission to access it"
  exit 1
fi

# Create a pipe for serial communication
echo "Setting up pipes for serial communication"
mkdir -p /tmp/hydrus_serial
rm -f /tmp/hydrus_serial/arduinolog.txt

# Start a background process that reads from the Arduino and writes to the log file
stty -F $ARDUINO_PORT raw speed $BAUD_RATE cs8 -cstopb -parenb
cat $ARDUINO_PORT | tee /tmp/hydrus_serial/arduinolog.txt &
CATPID=$!

# Return the PID of the background process
echo "Serial splitter running with PID: $CATPID"
echo $CATPID > /tmp/hydrus_serial/catpid.txt
echo "Arduino logs will be saved to /tmp/hydrus_serial/arduinolog.txt"
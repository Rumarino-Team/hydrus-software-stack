#!/bin/bash
# Script to monitor Arduino serial output

# Default values
PORT="/dev/ttyACM0"
BAUD=115200

# Check for arguments
if [ $# -ge 1 ]; then
  PORT=$1
fi

if [ $# -ge 2 ]; then
  BAUD=$2
fi

echo "Starting Arduino Serial Monitor on $PORT at $BAUD baud"
echo "Press Ctrl+C to exit"

# Check if stty is available
if command -v stty &> /dev/null; then
  # Configure the serial port
  stty -F $PORT cs8 $BAUD ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh -ixon -crtscts
fi

# Try several methods to read from serial port
if command -v cat &> /dev/null; then
  echo "Using 'cat' to monitor serial port..."
  cat $PORT
elif command -v screen &> /dev/null; then
  echo "Using 'screen' to monitor serial port..."
  screen $PORT $BAUD
elif command -v minicom &> /dev/null; then 
  echo "Using 'minicom' to monitor serial port..."
  minicom -D $PORT -b $BAUD
else
  echo "No suitable tools found for serial monitoring. Installing screen..."
  apt-get update && apt-get install -y screen
  echo "Using 'screen' to monitor serial port..."
  screen $PORT $BAUD
fi
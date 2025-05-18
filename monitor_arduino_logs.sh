#!/bin/bash
# Script to monitor Arduino logs in a tmux session

LOG_FILE="/tmp/hydrus_serial/arduinolog.txt"

# Check if the log file exists or wait for it to be created
echo "Waiting for Arduino log file..."
while [ ! -f "$LOG_FILE" ]; do
  sleep 1
  echo -n "."
done

echo ""
echo "Found Arduino log file. Showing Arduino serial output:"
echo "------------------------------------------------------"

# Use tail to follow the log file and display new content
tail -f "$LOG_FILE"
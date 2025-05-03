#!/bin/bash
# Integration test for serial communication between embedded_arduino and autonomy packages

# Color codes for better output visibility
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Print header
echo -e "${BLUE}======================================================${NC}"
echo -e "${BLUE}  HYDRUS AUTONOMY-ARDUINO SERIAL INTEGRATION TEST    ${NC}"
echo -e "${BLUE}======================================================${NC}"

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to check if a ROS node is running
is_node_running() {
    node_name=$1
    rosnode list | grep -q "$node_name"
    return $?
}

# Verify ROS installation
if ! command_exists roscore; then
    echo -e "${RED}[ERROR] ROS not found. Please make sure ROS is installed properly.${NC}"
    exit 1
fi

# Verify rosserial is installed
if ! command_exists rosrun rosserial_python serial_node.py; then
    echo -e "${RED}[ERROR] rosserial_python not found. Please install it with:${NC}"
    echo -e "${YELLOW}sudo apt-get install ros-<distro>-rosserial-python${NC}"
    exit 1
fi

# Step 1: Check if Arduino is connected
echo -e "${BLUE}[TEST] Checking if Arduino is connected...${NC}"
ARDUINO_PORT="/dev/ttyACM0"
if [ -e "$ARDUINO_PORT" ]; then
    echo -e "${GREEN}[PASS] Arduino found at $ARDUINO_PORT${NC}"
else
    echo -e "${RED}[FAIL] Arduino not found at $ARDUINO_PORT${NC}"
    echo -e "${YELLOW}[INFO] Please check USB connection and ensure Arduino is properly connected${NC}"
    echo -e "${YELLOW}[INFO] Attempting to look for other Arduino ports...${NC}"
    
    # Try to find any other Arduino ports
    for i in {0..9}; do
        if [ -e "/dev/ttyACM$i" ]; then
            ARDUINO_PORT="/dev/ttyACM$i"
            echo -e "${GREEN}[FOUND] Arduino at $ARDUINO_PORT${NC}"
            break
        fi
    done
    
    if [ "$ARDUINO_PORT" = "/dev/ttyACM0" ]; then
        echo -e "${RED}[FAIL] No Arduino found. Test cannot continue.${NC}"
        exit 1
    fi
fi

# Make sure port is accessible
if ! [ -r "$ARDUINO_PORT" ] || ! [ -w "$ARDUINO_PORT" ]; then
    echo -e "${RED}[FAIL] Cannot read/write to $ARDUINO_PORT. Fixing permissions...${NC}"
    sudo chmod a+rw "$ARDUINO_PORT"
    if [ $? -ne 0 ]; then
        echo -e "${RED}[FAIL] Failed to set permissions. Try running this script with sudo.${NC}"
        exit 1
    fi
    echo -e "${GREEN}[PASS] Permissions fixed for $ARDUINO_PORT${NC}"
fi

# Step 2: Start roscore if not already running
echo -e "${BLUE}[TEST] Checking if roscore is running...${NC}"
if ! pgrep -f roscore > /dev/null; then
    echo -e "${YELLOW}[INFO] Starting roscore...${NC}"
    roscore &
    ROSCORE_PID=$!
    sleep 5 # Wait for roscore to initialize
    echo -e "${GREEN}[PASS] roscore started with PID $ROSCORE_PID${NC}"
else
    echo -e "${GREEN}[PASS] roscore is already running${NC}"
fi

# Step 3: Start rosserial connection to Arduino
echo -e "${BLUE}[TEST] Starting rosserial connection to Arduino...${NC}"
rosrun rosserial_python serial_node.py _port:=$ARDUINO_PORT _baud:=57600 &
SERIAL_PID=$!
sleep 5 # Wait for connection to establish

# Verify rosserial is running
if ! is_node_running "serial_node"; then
    echo -e "${RED}[FAIL] Serial node failed to start.${NC}"
    kill $SERIAL_PID 2>/dev/null
    kill $ROSCORE_PID 2>/dev/null
    exit 1
else
    echo -e "${GREEN}[PASS] Serial node is running${NC}"
fi

# Step 4: Check topic mismatch between what autonomy publishes and what Arduino listens to
echo -e "${BLUE}[TEST] Checking for topic type mismatches...${NC}"

# Get list of thruster topics
AUTONOMY_TOPICS=$(rostopic list | grep '/thrusters/' | sort)
echo -e "${YELLOW}[INFO] Found thruster topics:${NC}"
for topic in $AUTONOMY_TOPICS; do
    echo -e "- $topic"
done

# Check message types for each topic
echo -e "${BLUE}[TEST] Checking message types for thruster topics...${NC}"
for topic in $AUTONOMY_TOPICS; do
    MSG_TYPE=$(rostopic info $topic | grep "Type:" | awk '{print $2}')
    echo -e "- $topic: $MSG_TYPE"
    
    # Check for type mismatch (Float32 vs Int8)
    if [ "$MSG_TYPE" == "std_msgs/Float32" ]; then
        echo -e "${YELLOW}[WARNING] Topic $topic uses std_msgs/Float32 but Arduino expects std_msgs/Int8${NC}"
    fi
done

# Step 5: Test publishing to thruster topics
echo -e "${BLUE}[TEST] Testing thruster command publishing...${NC}"

# Send test values to each thruster
for i in {1..8}; do
    echo -e "${YELLOW}[INFO] Publishing test command to thruster $i${NC}"
    
    # First try Int8
    rostopic pub --once /hydrus/thrusters/$i std_msgs/Int8 "data: 2" 2>/dev/null
    sleep 1
    
    # Then try Float32
    rostopic pub --once /thrusters/$i std_msgs/Float32 "data: 1550.0" 2>/dev/null
    sleep 1
done

# Step 6: Test autonomy controller node
echo -e "${BLUE}[TEST] Testing autonomy controller node...${NC}"

# Start the submarine controller
rosrun autonomy controllers.py &
CONTROLLER_PID=$!
sleep 3

# Check if controller is running
if ! is_node_running "submarine_controller"; then
    echo -e "${RED}[FAIL] submarine_controller failed to start.${NC}"
else
    echo -e "${GREEN}[PASS] submarine_controller is running${NC}"
    
    # Send a test waypoint to the controller
    echo -e "${YELLOW}[INFO] Sending test waypoint to controller...${NC}"
    rostopic pub --once /controller_action/goal autonomy/NavigateToWaypointActionGoal "{header: {stamp: now}, goal_id: {stamp: now, id: 'test_1'}, goal: {target_point: {x: 1.0, y: 0.0, z: 0.0}}}" &
    sleep 5
    
    # Check if thrusters are receiving commands
    echo -e "${BLUE}[TEST] Checking if thruster topics are active...${NC}"
    ACTIVE_THRUSTERS=0
    for i in {1..8}; do
        MSG_COUNT=$(rostopic echo -n 1 /thrusters/$i 2>/dev/null | wc -l)
        if [ $MSG_COUNT -gt 0 ]; then
            ACTIVE_THRUSTERS=$((ACTIVE_THRUSTERS + 1))
        fi
    done
    
    if [ $ACTIVE_THRUSTERS -gt 0 ]; then
        echo -e "${GREEN}[PASS] $ACTIVE_THRUSTERS thruster topics are receiving commands${NC}"
    else
        echo -e "${RED}[FAIL] No thruster topics are receiving commands${NC}"
    fi
    
    # Cleanup controller
    echo -e "${YELLOW}[INFO] Stopping submarine controller...${NC}"
    kill $CONTROLLER_PID 2>/dev/null
fi

# Step 7: Test with thruster mapper
echo -e "${BLUE}[TEST] Testing with thruster mapper...${NC}"
echo -e "${YELLOW}[INFO] Starting thruster_mapper.py in a new terminal...${NC}"
echo -e "${YELLOW}[INFO] Please follow the instructions in the new terminal to test individual thrusters${NC}"
echo -e "${YELLOW}[INFO] Press Ctrl+C in the new terminal when finished testing${NC}"

# Start thruster mapper in a new terminal
gnome-terminal -- bash -c "rosrun autonomy thruster_mapper.py; echo 'Press Enter to close this terminal'; read" || \
xterm -e "rosrun autonomy thruster_mapper.py; echo 'Press Enter to close this terminal'; read" || \
konsole --noclose -e "rosrun autonomy thruster_mapper.py; echo 'Press Enter to close this terminal'; read" || \
echo -e "${RED}[FAIL] Could not open a new terminal window${NC}"

# Wait for user to finish testing
read -p "Press Enter when you have finished testing with thruster_mapper... " -n 1 -r

# Step 8: Cleanup
echo -e "${BLUE}[TEST] Cleaning up processes...${NC}"
echo -e "${YELLOW}[INFO] Stopping serial node...${NC}"
kill $SERIAL_PID 2>/dev/null
echo -e "${YELLOW}[INFO] Stopping roscore (if we started it)...${NC}"
if [ -n "$ROSCORE_PID" ]; then
    kill $ROSCORE_PID 2>/dev/null
fi

# Final summary
echo -e "${BLUE}======================================================${NC}"
echo -e "${BLUE}  INTEGRATION TEST COMPLETE                          ${NC}"
echo -e "${BLUE}======================================================${NC}"
echo -e "${YELLOW}[INFO] Test complete. Here's what to look for:${NC}"
echo -e "${YELLOW}1. Check if Arduino responded to thruster commands${NC}"
echo -e "${YELLOW}2. Check for any message type mismatches (Float32 vs Int8)${NC}"
echo -e "${YELLOW}3. Verify that individual thruster control worked in the mapper${NC}"
echo -e "${YELLOW}4. If any tests failed, review the code for topic name or message type mismatches${NC}"

echo -e "${GREEN}For detailed ROS debugging, try these commands:${NC}"
echo -e "${GREEN}- rostopic list | grep thrusters${NC}"
echo -e "${GREEN}- rostopic echo /thrusters/1${NC}"
echo -e "${GREEN}- rosnode info /serial_node${NC}"
echo -e "${GREEN}- rostopic info /thrusters/1${NC}"

exit 0
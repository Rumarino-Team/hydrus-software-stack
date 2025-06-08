#!/bin/bash
set -e

echo "=========================================="
echo "Running Hydrus Autonomy Test Suite"
echo "=========================================="

# Determine ROS directory based on VOLUME environment variable
if [ "$VOLUME" == "true" ]; then
    echo "Using the Volume directory for building and testing the packages."
    ROS_DIR='/home/catkin_ws'    
else
    echo "Using the Copied Packages from Docker for building and testing."
    ROS_DIR='/catkin_ws'   
fi

echo "Using ROS workspace: $ROS_DIR"

# Source ROS environment
source /opt/ros/noetic/setup.bash
cd "$ROS_DIR"
source devel/setup.bash

# Initialize test results
TOTAL_TESTS=0
PASSED_TESTS=0
FAILED_TESTS=0

# Function to run a test and track results
run_test() {
    local test_name="$1"
    local test_command="$2"
    
    echo ""
    echo "----------------------------------------"
    echo "Running: $test_name"
    echo "----------------------------------------"
    
    TOTAL_TESTS=$((TOTAL_TESTS + 1))
    
    if timeout 300 bash -c "$test_command"; then
        echo "✅ PASSED: $test_name"
        PASSED_TESTS=$((PASSED_TESTS + 1))
    else
        echo "❌ FAILED: $test_name"
        FAILED_TESTS=$((FAILED_TESTS + 1))
    fi
}

# Start roscore in background for ROS tests
echo "Starting roscore..."
roscore &
ROSCORE_PID=$!
sleep 5

# Ensure roscore is running
if ! pgrep -f roscore > /dev/null; then
    echo "Failed to start roscore"
    exit 1
fi

# Set ROS master URI
export ROS_MASTER_URI=http://localhost:11311

# Run unit tests that don't require ROS
echo "=========================================="
echo "Running Unit Tests (Non-ROS)"
echo "=========================================="

# Tagging mission unit tests - use correct path based on workspace
run_test "Tagging Mission Unit Tests" "cd $ROS_DIR/src/hydrus-software-stack/autonomy/src/mission_planner && python3 tagging_mission_test.py"

# Run ROS integration tests
echo ""
echo "=========================================="
echo "Running ROS Integration Tests"
echo "=========================================="

# Controller tests using rostest with timeout
run_test "Controller Tests" "cd $ROS_DIR && timeout 120 rostest autonomy controller.test --text"

# Mission planner integration tests (these may need roscore)
run_test "Slalom Integration Tests" "cd $ROS_DIR/src/hydrus-software-stack/autonomy/src/mission_planner && python3 test_slalom_integration.py"

run_test "Gate Mission Tests" "cd $ROS_DIR/src/hydrus-software-stack/autonomy/src/mission_planner && python3 gate_mission_tester.py"

# DVL driver tests if available
if [ -f "$ROS_DIR/src/hydrus-software-stack/DVL/Wayfinder/driver_test.py" ]; then
    run_test "DVL Driver Tests" "cd $ROS_DIR/src/hydrus-software-stack/DVL/Wayfinder && python3 driver_test.py"
fi

# Clean up roscore
echo ""
echo "Cleaning up..."
kill $ROSCORE_PID 2>/dev/null || true
sleep 2

# Final results
echo ""
echo "=========================================="
echo "TEST SUMMARY"
echo "=========================================="
echo "Total Tests: $TOTAL_TESTS"
echo "Passed: $PASSED_TESTS"
echo "Failed: $FAILED_TESTS"

if [ $FAILED_TESTS -eq 0 ]; then
    echo "🎉 ALL TESTS PASSED!"
    exit 0
else
    echo "💥 $FAILED_TESTS TEST(S) FAILED!"
    exit 1
fi
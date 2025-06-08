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

# Source ROS environment and navigate to workspace
source /opt/ros/noetic/setup.bash
cd "$ROS_DIR"

# Build the workspace first to ensure all packages are available
echo "Building workspace..."
catkin_make --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.9 \
            -DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.9.so || true

# Source the workspace setup after building
if [ -f "$ROS_DIR/devel/setup.bash" ]; then
    source "$ROS_DIR/devel/setup.bash"
    echo "Workspace setup sourced successfully"
else
    echo "Warning: devel/setup.bash still not found after build"
fi

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
    
    if timeout 300 bash -c "source $ROS_DIR/devel/setup.bash 2>/dev/null || true; $test_command"; then
        echo "✅ PASSED: $test_name"
        PASSED_TESTS=$((PASSED_TESTS + 1))
    else
        echo "❌ FAILED: $test_name"
        FAILED_TESTS=$((FAILED_TESTS + 1))
    fi
}

# Function to run rostest with proper timeout and error handling
run_rostest() {
    local test_name="$1"
    local test_file="$2"
    
    echo ""
    echo "----------------------------------------"
    echo "Running rostest: $test_name"
    echo "----------------------------------------"
    
    TOTAL_TESTS=$((TOTAL_TESTS + 1))
    
    if timeout 600 bash -c "source $ROS_DIR/devel/setup.bash 2>/dev/null || true; rostest autonomy $test_file"; then
        echo "✅ PASSED: $test_name"
        PASSED_TESTS=$((PASSED_TESTS + 1))
    else
        echo "❌ FAILED: $test_name"
        FAILED_TESTS=$((FAILED_TESTS + 1))
    fi
}

# Start roscore in background for ROS tests
echo "Starting roscore..."
export ROS_MASTER_URI=http://localhost:11311
roscore &
ROSCORE_PID=$!
sleep 5

# Ensure roscore is running
if ! pgrep -f roscore > /dev/null; then
    echo "Failed to start roscore"
    exit 1
fi

# Run unit tests that don't require ROS
echo "=========================================="
echo "Running Unit Tests (Non-ROS)"
echo "=========================================="

# Tagging mission unit tests - use correct path based on workspace
run_test "Tagging Mission Unit Tests" "cd $ROS_DIR/src/hydrus-software-stack/autonomy/src/mission_planner && python3 tagging_mission_test.py"

# Run ROS integration tests using rostest
echo ""
echo "=========================================="
echo "Running ROS Integration Tests (rostest)"
echo "=========================================="

# Controller tests using rostest - this will launch controllers.py first
run_rostest "Controller Tests" "controller.test"

# Mission planner integration tests with proper environment
run_test "Slalom Integration Tests" "cd $ROS_DIR/src/hydrus-software-stack/autonomy/src/mission_planner && PYTHONPATH=$ROS_DIR/src:$ROS_DIR/devel/lib/python3/dist-packages:$PYTHONPATH python3 test_slalom_integration.py"

run_test "Gate Mission Tests" "cd $ROS_DIR/src/hydrus-software-stack/autonomy/src/mission_planner && PYTHONPATH=$ROS_DIR/src:$ROS_DIR/devel/lib/python3/dist-packages:$PYTHONPATH python3 gate_mission_tester.py"

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
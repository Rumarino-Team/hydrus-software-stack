#!/bin/bash
"""
ROS Profiler Demo Launcher

This script helps launch the ROS profiler demo with proper environment setup.
"""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}ROS Profiler Demo Launcher${NC}"
echo "=============================="

# Check if we're in a ROS environment
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}Warning: ROS environment not detected. Sourcing ROS...${NC}"
    source /opt/ros/noetic/setup.bash 2>/dev/null || source /opt/ros/melodic/setup.bash 2>/dev/null
    if [ -z "$ROS_DISTRO" ]; then
        echo -e "${RED}Error: Could not source ROS environment${NC}"
        exit 1
    fi
fi

echo -e "${GREEN}ROS $ROS_DISTRO environment detected${NC}"

# Source workspace if built
WORKSPACE_DIR="/home/catkin_ws"
if [ -f "$WORKSPACE_DIR/devel/setup.bash" ]; then
    echo -e "${GREEN}Sourcing workspace environment...${NC}"
    source "$WORKSPACE_DIR/devel/setup.bash"
else
    echo -e "${YELLOW}Warning: Workspace not built. Building now...${NC}"
    cd "$WORKSPACE_DIR"
    catkin_make
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}Workspace built successfully${NC}"
        source devel/setup.bash
    else
        echo -e "${RED}Error: Failed to build workspace${NC}"
        exit 1
    fi
fi

# Check for required Python packages
echo -e "${BLUE}Checking Python dependencies...${NC}"
python3 -c "import psutil, colorama" 2>/dev/null
if [ $? -ne 0 ]; then
    echo -e "${YELLOW}Installing required Python packages...${NC}"
    pip install psutil colorama
fi

# Change to the scripts directory
SCRIPTS_DIR="/home/catkin_ws/src/hydrus-software-stack/autonomy/scripts/services"
cd "$SCRIPTS_DIR"

# Check if roscore is running
echo -e "${BLUE}Checking if roscore is running...${NC}"
rostopic list &>/dev/null
if [ $? -ne 0 ]; then
    echo -e "${YELLOW}Starting roscore in background...${NC}"
    roscore &
    ROSCORE_PID=$!
    sleep 3
    echo -e "${GREEN}roscore started (PID: $ROSCORE_PID)${NC}"
else
    echo -e "${GREEN}roscore is already running${NC}"
fi

# Function to cleanup on exit
cleanup() {
    echo -e "\n${YELLOW}Cleaning up...${NC}"
    if [ ! -z "$ROSCORE_PID" ]; then
        kill $ROSCORE_PID 2>/dev/null
    fi
    exit 0
}

# Set trap to cleanup on script exit
trap cleanup SIGINT SIGTERM

# Present menu options
echo ""
echo -e "${BLUE}Choose an option:${NC}"
echo "1) Run demo nodes with launch file (Recommended)"
echo "2) Run individual demo nodes manually"
echo "3) Run profiler for demo nodes (Terminal 2)"
echo "4) Run profiler for all nodes (Terminal 2)"
echo "5) View profiler help"
echo "6) Exit"

read -p "Enter your choice [1-6]: " choice

case $choice in
    1)
        echo -e "${GREEN}Starting demo nodes with launch file...${NC}"
        echo -e "${YELLOW}Press Ctrl+C to stop all demo nodes${NC}"
        echo ""
        cd "$WORKSPACE_DIR"
        roslaunch autonomy profiler_demo.launch
        ;;
    2)
        echo -e "${GREEN}Manual demo node setup:${NC}"
        echo "Run each command in a separate terminal:"
        echo -e "${YELLOW}Terminal 1:${NC} python3 ros_profiler_demo.py camera"
        echo -e "${YELLOW}Terminal 2:${NC} python3 ros_profiler_demo.py processor"
        echo -e "${YELLOW}Terminal 3:${NC} python3 ros_profiler_demo.py analyzer"
        echo ""
        echo "Choose which node to run in this terminal:"
        echo "1) Camera"
        echo "2) Processor" 
        echo "3) Analyzer"
        read -p "Enter choice [1-3]: " node_choice
        
        cd "$SCRIPTS_DIR"
        case $node_choice in
            1) python3 ros_profiler_demo.py camera ;;
            2) python3 ros_profiler_demo.py processor ;;
            3) python3 ros_profiler_demo.py analyzer ;;
            *) echo -e "${RED}Invalid choice${NC}" ;;
        esac
        ;;
    3)
        echo -e "${GREEN}Starting profiler for demo nodes...${NC}"
        echo -e "${YELLOW}Press Ctrl+C to stop the profiler${NC}"
        echo ""
        python3 ros_profiler.py --nodes image_processor data_analyzer --export demo_profile.csv
        ;;
    4)
        echo -e "${GREEN}Starting profiler for all nodes...${NC}"
        echo -e "${YELLOW}Press Ctrl+C to stop the profiler${NC}"
        echo ""
        python3 ros_profiler.py --export all_nodes_profile.csv
        ;;
    5)
        echo -e "${GREEN}ROS Profiler Help:${NC}"
        python3 ros_profiler.py --help
        ;;
    6)
        echo -e "${GREEN}Exiting...${NC}"
        cleanup
        ;;
    *)
        echo -e "${RED}Invalid choice. Please select 1-6.${NC}"
        ;;
esac

# Keep the script running to maintain roscore if we started it
if [ "$choice" = "1" ] && [ ! -z "$ROSCORE_PID" ]; then
    echo -e "${YELLOW}Keeping roscore running. Press Ctrl+C to stop.${NC}"
    wait $ROSCORE_PID
fi

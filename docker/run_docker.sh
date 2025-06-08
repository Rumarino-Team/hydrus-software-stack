#!/usr/bin/env bash
set -e

# Function to check if the system is Jetson TX2
is_jetson_tx2() {
  if [[ $(uname -m) == "aarch64" && $(cat /etc/nv_tegra_release) ]]; then
    return 0
  else
    return 1
  fi
}

# Function to check if the system is running in WSL
is_wsl() {
  if grep -qi "microsoft" /proc/version; then
    return 0
  else
    return 1
  fi
}

# Function to check if rosbags exist and offer to download if not
check_rosbags() {
  # Get the directory of the script
  SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  PARENT_DIR="$(dirname "$SCRIPT_DIR")"
  ROSBAG_DIR="$PARENT_DIR/rosbags"
  
  # Check if directory exists and has any .bag files
  if [[ ! -d "$ROSBAG_DIR" ]] || [[ -z "$(find "$ROSBAG_DIR" -name "*.bag" 2>/dev/null)" ]]; then
    echo "No ROS bag files found in $ROSBAG_DIR"
    echo "Would you like to download the default rosbag?"
    read -p "(y/n) " download_choice
    case $download_choice in 
      [Yy]* ) 
        echo "Running download script..."
        python3 "$PARENT_DIR/download_rosbag.py"
        return $?
        ;;
      [Nn]* ) 
        echo "Continuing without downloading rosbag files."
        return 1
        ;;
      * ) 
        echo "Invalid input. Continuing without downloading."
        return 1
        ;;
    esac
  fi
  
  return 0
}

DEPLOY=false  
VOLUME=false  
FORCE_CPU=false 
RVIZ=false # This option controls whether to display RViz only (for CPU) 
ZED_OPTION=false # This option controls whether to display the ZED2i camera in RViz
FORCE_JETSON=false
ROSBAG_PLAYBACK=false
DEBUG_ARDUINO=false
TEST=false  # New test environment variable

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --deploy) DEPLOY=true ;;
        --volume) VOLUME=true ;;
        --force-cpu) FORCE_CPU=true ;;
        --force-jetson) FORCE_JETSON=true ;;
        --test) TEST=true ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

# If TEST mode is enabled, configure for automated testing
if [ "$TEST" == "true" ]; then
    echo "TEST mode enabled - configuring for automated testing"
    FORCE_CPU=true
    RVIZ=false
    ZED_OPTION=false
    ROSBAG_PLAYBACK=false
    DEBUG_ARDUINO=false
else
    # Ask user if they want to play rosbag files
    while true; do
      echo "Do you want to play rosbag files from the rosbags folder?"
      read -p "(y/n) " rosbag_choice
      case $rosbag_choice in 
        [Yy]* ) 
          ROSBAG_PLAYBACK=true
          # Check if rosbags exist, offer to download if not
          check_rosbags
          break
          ;;
        [Nn]* ) 
          break
          ;;
        * ) 
          echo "Invalid input. Please try again."
          ;;
      esac
    done

    # Ask user if they want to debug Arduino serial output
    while true; do
      echo "Do you want to monitor Arduino serial output for debugging?"
      echo "(NOTE: This will show serial logs before ROS node starts)"
      read -p "(y/n) " debug_choice
      case $debug_choice in 
        [Yy]* ) DEBUG_ARDUINO=true; break;;
        [Nn]* ) break;;
        * ) echo "Invalid input. Please try again.";;
      esac
    done

    if [[ $FORCE_CPU == true || $(is_wsl) == 0 ]]; then
      while true; do
        echo "Do you want to use RViz?"
        read -p "(y/n) " choice
        case $choice in 
          [Yy]* ) RVIZ=true; break;;
          [Nn]* ) break;;
          * ) echo "Invalid input. Please try again.";;
        esac
        done
    fi
fi

# Check if --force-cpu was passed or TEST mode is enabled
if [[ "$FORCE_CPU" == true ]] || [[ "$TEST" == true ]]; then
  echo "Force CPU deployment selected. Running CPU version."
  if [[ "$TEST" == true ]]; then
    # For test mode, use --abort-on-container-exit to stop all containers when any container stops
    DEPLOY=$DEPLOY VOLUME=$VOLUME ZED_OPTION=$ZED_OPTION RVIZ=$RVIZ ROSBAG_PLAYBACK=$ROSBAG_PLAYBACK DEBUG_ARDUINO=$DEBUG_ARDUINO TEST=$TEST docker compose -f docker-compose-amd64-cpu.yaml up --abort-on-container-exit
  else
    DEPLOY=$DEPLOY VOLUME=$VOLUME ZED_OPTION=$ZED_OPTION RVIZ=$RVIZ ROSBAG_PLAYBACK=$ROSBAG_PLAYBACK DEBUG_ARDUINO=$DEBUG_ARDUINO TEST=$TEST docker compose -f docker-compose-amd64-cpu.yaml up
  fi
# Check if --force-jetson was passed
elif [[ "$FORCE_JETSON" == true ]]; then
  echo "Force Jetson deployment selected. Running Jetson scripts."
  chmod +x ../jetson.sh
  ../jetson.sh
else
  # Check if system is WSL
  if is_wsl; then
    echo "WSL detected. Running without GPU support."
    DEPLOY=$DEPLOY VOLUME=$VOLUME ZED_OPTION=$ZED_OPTION RVIZ=$RVIZ ROSBAG_PLAYBACK=$ROSBAG_PLAYBACK DEBUG_ARDUINO=$DEBUG_ARDUINO TEST=$TEST docker compose -f docker-compose-amd64-cpu.yaml up
  # Check if system is Jetson TX2
  elif is_jetson_tx2; then
    echo "Jetson TX2 detected. Running Jetson TX2 Docker Compose."
    chmod +x ../jetson.sh
    ../jetson.sh
  else
    # Check if NVIDIA GPUs are available
    if nvidia-smi > /dev/null 2>&1; then
      echo "GPU available. Running with GPU support."
      while true; do
        echo "Do you want to display the ZED2i camera in RViz?"
        read -p "(y/n) " choice
        case $choice in 
          [Yy]* ) ZED_OPTION=true; break;;
          [Nn]* ) break;;
          * ) echo "Invalid input. Please try again.";;
        esac
      done
      echo "ZED_OPTION=$ZED_OPTION"
      DEPLOY=$DEPLOY VOLUME=$VOLUME ZED_OPTION=$ZED_OPTION RVIZ=$RVIZ ROSBAG_PLAYBACK=$ROSBAG_PLAYBACK DEBUG_ARDUINO=$DEBUG_ARDUINO TEST=$TEST docker compose -f docker-compose-amd64-cuda.yaml up
    else
      echo "No GPU available. Running without GPU support."
      DEPLOY=$DEPLOY VOLUME=$VOLUME ZED_OPTION=$ZED_OPTION RVIZ=$RVIZ ROSBAG_PLAYBACK=$ROSBAG_PLAYBACK DEBUG_ARDUINO=$DEBUG_ARDUINO TEST=$TEST docker compose -f docker-compose-amd64-cpu.yaml up
    fi
  fi
fi

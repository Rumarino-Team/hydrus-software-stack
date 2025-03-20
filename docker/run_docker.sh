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


DEPLOY=false  
VOLUME=false  
FORCE_CPU=false 
ZED_OPTION=false
FORCE_JETSON=false


while [[ "$#" -gt 0 ]]; do
    case $1 in
        --deploy) DEPLOY=true ;;
        --volume) VOLUME=true ;;
        --force-cpu) FORCE_CPU=true ;;
        --force-jetson) FORCE_JETSON=true ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

# Check if --force-cpu was passed
if [[ "$FORCE_CPU" == true ]]; then
  echo "Force CPU deployment selected. Running CPU version."
  DEPLOY=$DEPLOY VOLUME=$VOLUME ZED_OPTION=$ZED_OPTION  docker compose -f docker-compose-amd64-cpu.yaml up
# Check if --force-jetson was passed
elif [[ "$FORCE_JETSON" == true ]]; then
  echo "Force Jetson deployment selected. Running Jetson scripts."
  chmod +x ../jetson.sh
  ../jetson.sh
else
  # Check if system is WSL
  if is_wsl; then
    echo "WSL detected. Running without GPU support."
    DEPLOY=$DEPLOY VOLUME=$VOLUME docker compose -f docker-compose-amd64-cpu.yaml up
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
        read -p "(y/n)" choice
        case $choice in 
          [Yy]* ) ZED_OPTION=true; break;;
          [Nn]* ) break;;
          * ) echo "Invalid input. Please try again.";;
        esac
      done
      echo "ZED_OPTION=$ZED_OPTION"
      DEPLOY=$DEPLOY VOLUME=$VOLUME ZED_OPTION=$ZED_OPTION  docker compose -f docker-compose-amd64-cuda.yaml up
    else
      echo "No GPU available. Running without GPU support."
      DEPLOY=$DEPLOY VOLUME=$VOLUME ZED_OPTION=$ZED_OPTION docker compose -f docker-compose-amd64-cpu.yaml up
    fi
  fi
fi

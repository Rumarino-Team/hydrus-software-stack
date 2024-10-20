#!/bin/bash

# Function to check if the system is Jetson TX2
is_jetson_tx2() {
  if [[ $(uname -m) == "aarch64" && $(cat /etc/nv_tegra_release) ]]; then
    return 0
  else
    return 1
  fi
}

DEPLOY=false  
VOLUME=false  

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --deploy) DEPLOY="$2"; shift ;;  
        --volume) VOLUME="$2"; shift ;;  
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done
# Check if system is Jetson TX2
if is_jetson_tx2; then
  echo "Jetson TX2 detected. Running Jetson TX2 Docker Compose."
  DEPLOY=$DEPLOY VOLUME=$VOLUME  docker compose -f docker-compose-jetson-tx2.yaml up
else
  # Check if NVIDIA GPUs are available
  if nvidia-smi > /dev/null 2>&1; then
    echo "GPU available. Running with GPU support."
    DEPLOY=$DEPLOY VOLUME=$VOLUME  docker compose -f docker-compose-amd64-cuda.yaml up
  else
    echo "No GPU available. Running without GPU support."
    DEPLOY=$DEPLOY VOLUME=$VOLUME  docker compose -f docker-compose-amd64-cpu.yaml up
  fi
fi

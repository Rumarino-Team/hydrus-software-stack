## Getting Started:

Welcome to the Hydrus Software Stack. This is the collection of ROS packages for running RUMarino AUV.


## How to install

### Prerequisites:
- Docker
#### Windows:
 - Windows Subsystem System (WSL)
 - usbipd
## Docker Installation

To install the dependencies and running the packages with Docker you need to run the following commands.
```bash
git clone https://github.com/Rumarino-Team/hydrus-software-stack.git
cd docker
chmod +x ./run_docker.sh
./run_docker.sh
```

### Options for running the dockers.
the command  `./run_docker.sh` have the following options for running it lets discuss them.

There are 3 types of docker compose that the application can run. cpu only, nvidia gpu and Jetsons. Whenever you run the command `./run_docker.sh` this will automatically detect your computer which one applies best to run for your specific case.

Aditionally there are 3 arguments that you add into the application for different purposes:

- `--deploy` : This argument will make the Arduino upload data. 
- `--volume` : The volime the data and the things noel la eslpass miamae.
- `--force-cpu`: This will run the docker compose that is cpu.


```bash
./run_docker.sh --deploy --volume --force-cpu
```

## Roslaunch

There are three main nodes required to enable autonomy:

controllers : `autonomy/src/controllers.py` (PID based, see `docs/pid_controller.md` for tuning)
computer_vision: `autnomy/src/cv_publishers.py`
mission_planning: `ros_mission_planning.py`

To run all the ros nodes at the same time you can use the `autonomy.launch` launch file.

```bash
roslaunch autonomy autonomy.launch
```

## Using the Zed_ROS_WRAPPER NODE

When running the amd64 cuda version or the jetson version we run the  [Zed Ros wrapper](https://github.com/stereolabs/zed-ros-wrapper.git)  ros node. If we want
to change the configuration of the camera we can change them in the `common_camera_params.yaml` and the `zed2i_camera_params.yaml` files. 

For example if we want to activate the zed camera object detection we can change the `od_enabled` field to true in the `common_camera_params.yaml`.

```yaml

object_detection:
    od_enabled:  true       
    model: 'MULTI_CLASS_BOX_ACCURATE'
 ... 

```

In order to use the cuda version you need to make sure you have installed the following dependencies.

 - CUDA TOOLKIT : https://developer.nvidia.com/cuda-downloads
 - CUDA CONTAINER TOOLKIT:  https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html


### Run ROS Bags

1. Download the ROS bag file.  **ZED2i Camera**: [Download here](https://drive.google.com/file/d/16Lr-CbW1rW6rKh8_mWClTQMIjm2u0y8X/view?usp=drive_link)

2. Play the bag with:

    ```bash
    rosbag play <file_path>
    ```

3. To loop playback:

    ```bash
    rosbag play <file_path> --loop
    ```

## Working with Docker Containers and tmux Sessions

### Entering the Docker Container

After starting the Docker environment using `./run_docker.sh`, you can enter the running container with:

```bash
# List all running containers
docker ps

# Enter the hydrus container (replace CONTAINER_ID with the actual ID)
docker exec -it CONTAINER_ID bash

# Alternatively, use the container name directly
docker exec -it hydrus bash
```

### Working with tmux Sessions

The Hydrus software stack uses tmux for managing multiple terminal sessions. Here's how to work with them:

1. Start the tmux sessions (if not automatically started):

    ```bash
    # Inside the Docker container
    cd /catkin_ws/src/hydrus-software-stack
    ./start_tmux_sessions.sh
    ```

2. List available tmux sessions:

    ```bash
    tmux ls
    ```

3. Attach to a specific tmux session:

    ```bash
    # Attach to the serial_connection session
    tmux attach -t serial_connection
    ```

4. Basic tmux navigation commands:
   - Switch between windows: `Ctrl+B` then window number (e.g., `Ctrl+B` then `0`)
   - Switch between panes: `Ctrl+B` then arrow keys
   - Detach from session (without closing it): `Ctrl+B` then `D`
   - Create a new window: `Ctrl+B` then `C`
   - Split pane horizontally: `Ctrl+B` then `"`
   - Split pane vertically: `Ctrl+B` then `%`

5. Exit a tmux session completely:
    ```bash
    # First detach with Ctrl+B then D
    # Then kill the session if needed
    tmux kill-session -t session_name
    ```


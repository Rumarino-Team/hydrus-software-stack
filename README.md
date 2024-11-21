
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

controllers : `autonomy/src/controllers.py`
computer_vision: `autnomy/src/cv_publishers.py`
mission_planning: `ros_mission_planning.py`

To run all the ros nodes at the same time you can use the `autonomy.launch` launch file.

```bash
roslaunch autonomy autonomy.launch
```


## Download ROS Bags

- **ZED2i Camera**: [Download here](https://drive.google.com/file/d/16Lr-CbW1rW6rKh8_mWClTQMIjm2u0y8X/view?usp=drive_link)

### Run ROS Bags

1. Download the ROS bag file.
2. Play the bag with:

    ```bash
    rosbag play <file_path>
    ```

3. To loop playback:

    ```bash
    rosbag play <file_path> --loop
    ```

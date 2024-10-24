
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

Hydrus node roslaunch commands:

```bash
roslaunch hydrus hydrus_start.launch
```



## Download Ros Bags

Ros bags are recording of the sensors. In here are the following Recordings:

Start Hydrus Services:

```bash
roslaunch hydrus hydrus_start.launch

```
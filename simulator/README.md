# Hydrus Simulator

A basic simulation meant to test Hydrus' mission control capabilities.

## How to run

1. Launch Hydrus Docker container in interactive mode with GUI support
2. Run the `source devel/setup.bash` command at the root directory of your catkin_ws workspace
3. Run the command `roslaunch hydrus_simulator cube.launch`

## TODO

* Extend cube with the same thruster configuration as Hydrus
* Simulate IMU and thrusters with the same interfaces as the real deal
* Connect simulation to mission control
* Write unit tests for both simulation and mission control
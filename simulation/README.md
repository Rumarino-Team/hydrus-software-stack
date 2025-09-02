# How to run:

Docker is required to run the simulation along with a Linux install (WSL works but very slow). An Nvidia GPU is also highly recommended.

1. Go to docker/simulation folder

2. Run `docker compose -f simulation.yaml run simulation`

3. Run `cd /home/ros2_ws`

4. Run `colcon build --symlink-install`

5. Run `ros2 launch hydrus_sim_ros2 hydrussim.launch.py`

To control thrusters, run `ros2 topic pub /hydrus_thruster std_msgs/msg/Float64MultiArray "{layout: {}, data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"`. The first 4 are the corner thrusters while the last 4 are the depth thrusters. 
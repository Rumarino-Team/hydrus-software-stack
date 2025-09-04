# How to run:

Docker is required to run the simulation along with a Linux install (WSL works but very slow). An Nvidia GPU is also highly recommended.

1. Go to docker/simulation folder

2. Run `docker compose -f simulation.yaml run simulation`. This will launch a window with the simulation.

To control thrusters, follow these steps after running the simulation:

1. Run `docker compose -f simulation.yaml run simulation /bin/bash` in another terminal window/tab to gain access to the shell.

2. Run `ros2 topic pub /hydrus_thrusters std_msgs/msg/Float64MultiArray "{layout: {}, data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"`. The first 4 are the corner thrusters while the last 4 are the depth thrusters. 
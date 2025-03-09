# Capstone Project
### System Configuration
- Jetson
  - Ubuntu 20.04.6 LTS
  - ROS2 Foxy

### Connect to the drone
Make sure that you your laptop is connected to a network named `Flight Club`, with password `12345678`.
Obtain the IP address of the drone (from your phone's network manager), and ssh into the drone. For example:
```
ssh jetson@192.178.107.146
```
The password is `jetson`.


### Realsense Camera
Topic can be launched with:
```
ros2 launch realsense2_camera rs_launch.py
```

### Run the simulation
Clone the PX4 source code in this repo:
Build the Docker image once:
```
xhost +local:docker
docker build -t base-capstone .
```
Note: we need to implement a service to set to offboard mode for the simulation to work again (see TODO in exercise2.py)
#### Exercise 2
Run the Docker container:
```
xhost +local:docker
docker compose run --rm rob498 
```
Open 2 additional windows using `docker exec -it capstone /bin/bash`
In the first window, run the following command (it might take a while the first time, we recommend commiting your docker container in case you need to stop your container/remove the container):

```
cd /src/PX4-Autopilot
make px4_sitl gazebo-classic
```
In the second tab:
```
source /opt/ros/foxy/setup.bash
cd /src/ros_ws
colcon build --symlink-install
source /src/ros_ws/install/local_setup.bash
ros2 run mavros install_geographiclib_datasets.sh
chmod +x /src/ros_ws/src/drone_packages/flight_club/src/exercise2/exercise2.py
chmod +x /src/ros_ws/src/drone_packages/flight_club/src/exercise3/exercise3.py
ros2 launch flight_club sim_launch.py
```

In the third tab, you can launch the drone running
```
source /opt/ros/foxy/setup.bash
ros2 service call /rob498_drone_06/comm/set_offboard std_srvs/srv/Trigger
ros2 service call /rob498_drone_06/comm/launch std_srvs/srv/Trigger
```

### Run the physical robot
ssh into the drone.
Clone this repo in `ros_ws` in the home folder of the drone, at `/home/jetson`
#### Exercise 2

First, build the code:
```
cs ros_ws
colcon build --symlink-install
source /src/ros_ws/install/local_setup.bash
ros2 run mavros install_geographiclib_datasets.sh
```

In another window, ssh to the drone and run the realsense node:
```
ros2 launch realsense2_camera rs_launch.py
```

Back in the first window, launch the exercise 2 node:
```
ros2 launch flight_club real_launch.py
```

Set the board to offboard mode with the controller. Then, ssh into the drone in another terminal and run:
```
ros2 service call /rob498_drone_06/comm/launch std_srvs/srv/Trigger
```

To land:
```
ros2 service call /rob498_drone_06/comm/land std_srvs/srv/Trigger
```


### Worlds
To launch the simulation in a custom world:
```
export PX4_SITL_WORLD=/src/ros_ws/src/drone_packages/simulation/worlds/easy.sdf
make px4_sitl gazebo-classic
```
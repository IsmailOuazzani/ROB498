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

### Run the simulation
Clone the PX4 source code in this repo:
Build the Docker image once:
```
xhost +local:docker
docker build -t base-capstone .
```
#### Exercise 2
Run the Docker container:
```
docker compose run --rm rob498 
```
Open 3 additional windows using `docker exec -it capstone /bin/bash`
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
ros2 launch px4_autonomy_modules mavros.launch.py fcu_url:="udp://:14540@127.0.0.1:14557"
```

In the third tab
```
source /opt/ros/foxy/setup.bash
source /src/ros_ws/install/local_setup.bash
ros2 run flight_club mavros_arm_example.py
```


In the fourth tab, you can launch the drone running
```
source /opt/ros/foxy/setup.bash
ros2 service call /rob498_drone_06/comm/launch std_srvs/srv/Trigger
```


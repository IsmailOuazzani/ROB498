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
#### Take off example
Run the Docker container:
```
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
ros2 launch px4_autonomy_modules mavros.launch.py fcu_url:="udp://:14540@127.0.0.1:14557"
```

In the third tab
```
source /opt/ros/foxy/setup.bash
source /src/ros_ws/install/local_setup.bash
ros2 run 
```

#### Exercise 2
Run the container and mount the flight_club source code (Note that I haven't tested this command yet, I used to copy the folder using docker cp so there might be permission issues to fix):
```
docker run -it  --privileged  -v /tmp/.X11-unix:/tmp/.X11-unix:ro -v ./flight_club:/src/ros_ws/src/:ro  --env="QT_X11_NO_MITSHM=1" --env="DISPLAY" --name=capstone  --network host   base-capstone 
```
Open 2 additional windows using `docker exec -it capstone /bin/bash`. In the first window, run:
```
MicroXRCEAgent udp4 -p 8888
```
In the second window, run the following command (it might take a while the first time, we recommend commiting your docker container in case you need to stop your container/remove the container):
```
make px4_sitl gazebo-classic
```
In the last tab:
```
source /opt/ros/foxy/setup.bash
cd ros_ws
colcon build --symlink-install
source install/local_setup.bash
ros2 run flight_club exercise2
```

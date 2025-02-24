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
docker build -t base-capstone .
```
Run the Docker container:
```
docker run -it  --privileged   -v /tmp/.X11-unix:/tmp/.X11-unix:ro   --env="QT_X11_NO_MITSHM=1" --env="DISPLAY" --name=capstone  --network host   base-capstone 
```

#### Take off example
Open 2 additional windows using `docker exec -it capstone /bin/bash`
In the first window, run:
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
source install/local_setup.bash
ros2 run px4_ros_com offboard_control
```




git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
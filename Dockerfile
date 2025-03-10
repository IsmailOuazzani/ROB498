FROM osrf/ros:foxy-desktop

RUN apt-get update -y &&\
    apt-get install -y \
    wget \
    python3-pip \
    python3-dev \
    python3-setuptools \
    python3-tk


# Install Python dependencies
RUN pip3 install numpy pyyaml scipy 

# Install PX4
WORKDIR /src
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive &&\
    bash PX4-Autopilot/Tools/setup/ubuntu.sh

# Upgrade CMake
RUN sudo apt install apt-transport-https ca-certificates gnupg software-properties-common -y && \
    wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc | sudo apt-key add - &&\
    sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ focal main' &&\
    sudo apt update -y &&\
    sudo apt install cmake -y

# Install mavros
RUN sudo apt-get install -y ros-foxy-gazebo-ros2-control \
    ros-foxy-xacro \
    ros-foxy-nav2-bringup \
    ros-foxy-mavros \
    ros-foxy-mavros-extras \
    ros-foxy-octomap-server \
    ros-foxy-rtabmap-ros \
    ros-foxy-realsense2-camera 

# Install example
SHELL ["/bin/bash", "-c"]
RUN mkdir ros_ws/src -p &&\
    cd ros_ws/src  &&\
    git clone https://github.com/PX4/px4_msgs.git &&\
    git clone https://github.com/PX4/px4_ros_com.git &&\
    cd .. &&\
    pwd &&\
    cmake --version &&\
    source /opt/ros/foxy/setup.bash && \
    colcon build --symlink-install





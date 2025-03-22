cmake_minimum_required(VERSION 3.5)
project(flight_club)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
          "msg/TrajectoryPlan.msg")

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/exercise2/exercise2.py
  src/exercise2/mavros_arm_example.py
  src/exercise3/exercise3.py
  src/waypoint_publisher.py
  src/vicon.py
  src/planner.py
  src/velocity_control.py
  src/visualizer.py
  src/visualizer_v2c.py
  src/realsense.py
  src/vicon_to_cam.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

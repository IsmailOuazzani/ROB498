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
  src/waypoint_publisher.py
  src/pose_comparator.py
  src/exercise2/vicon_node.py
  src/exercise3/exercise3.py
  src/exercise3/planner.py
  src/exercise3/velocity_controll.py
  src/exercise2/exercise2.py
  src/exercise2/mavros_arm_example.py
  src/exercise2/realsense.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

cmake_minimum_required(VERSION 3.5)
project(flight_club)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(px4_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/exercise3/exercise3.py
  src/exercise2/exercise2.py
  src/exercise2/mavros_arm_example.py
  src/exercise2/realsense.py
  DESTINATION lib/${PROJECT_NAME}
)

# Launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

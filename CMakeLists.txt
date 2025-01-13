cmake_minimum_required(VERSION 3.5)
project(mrs_project_crazyflies)  # Use the correct package name

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED) 
find_package(mrs_msgs REQUIRED)  # Assuming this is a dependency

# Add other dependencies if needed

# Define the Python module
ament_python_install_package(mrs_project_crazyflies)  # Install your Python module

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

# Include directories for headers (if any)
include_directories(
  include 
)

ament_package()

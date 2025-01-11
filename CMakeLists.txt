cmake_minimum_required(VERSION 3.5)
project(mrs_project_part2) 

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED) 

add_executable(reynolds_sim mrs_project_part2/reynolds.py)  
ament_target_dependencies(your_node_name rclpy) 

install(TARGETS
  reynolds_sim
  DESTINATION lib/${PROJECT_NAME})

ament_package()
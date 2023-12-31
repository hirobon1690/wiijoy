cmake_minimum_required(VERSION 3.5)
project(wiijoy)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


add_executable(joy_node src/wiijoy.cpp)

target_link_libraries(joy_node wiiuse)
target_link_libraries(joy_node m)
target_link_libraries(joy_node bluetooth)
ament_target_dependencies(joy_node rclcpp std_msgs)
ament_target_dependencies(joy_node rclcpp sensor_msgs)

install(TARGETS
  joy_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
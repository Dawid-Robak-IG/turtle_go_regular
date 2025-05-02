cmake_minimum_required(VERSION 3.10)
project(turtle_go_regular)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)


add_executable(polygon src/polygon.cpp)
ament_target_dependencies(polygon rclcpp geometry_msgs nav2_msgs rclcpp_action)

install(TARGETS
  polygon
  DESTINATION lib/${PROJECT_NAME})

ament_package()

cmake_minimum_required(VERSION 3.8)
project(gp_velocity_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(velocity_manager src/velocity_manager.cpp)
ament_target_dependencies(velocity_manager rclcpp geometry_msgs sensor_msgs nav_msgs)
target_compile_features(velocity_manager PUBLIC cxx_std_17)

install(TARGETS
  velocity_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(gp_nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Python/runtime dependencies used by the mission supervisor nodes
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch/config/BT files
install(DIRECTORY
  launch
  config
  behavior_trees
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.swp" EXCLUDE
  PATTERN ".*.swp" EXCLUDE
)

# Install executable Python node
install(PROGRAMS
  gp_nav2/gp_goal_pose_relay.py
  gp_nav2/gp_mission_supervisor_v2.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install helper Python module beside the executable
# The supervisor imports this file.
install(FILES
  gp_nav2/gp_route_memory_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

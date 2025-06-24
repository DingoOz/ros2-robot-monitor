cmake_minimum_required(VERSION 3.8)
project(ros2_robot_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# Find ncurses
find_package(PkgConfig REQUIRED)
pkg_check_modules(NCURSES REQUIRED ncurses)

# Create directories
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME})

# Add executable
add_executable(ros2_monitor 
  src/monitor.cpp
  src/topic_monitor.cpp
  src/ncurses_display.cpp
)

# Include directories
target_include_directories(ros2_monitor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${NCURSES_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(ros2_monitor
  ${NCURSES_LIBRARIES}
)

# ROS2 dependencies
ament_target_dependencies(ros2_monitor
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  diagnostic_msgs
)

# Install targets
install(TARGETS ros2_monitor
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
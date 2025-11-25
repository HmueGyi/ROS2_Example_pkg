cmake_minimum_required(VERSION 3.8)
project(homework_pkg)

# Enable compiler warnings for GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(turtlesim REQUIRED)

# Linting (optional but recommended)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# === EXECUTABLES ===

# Draw Cycle
add_executable(draw_cyc src/cycle/draw_cycle.cpp)
ament_target_dependencies(draw_cyc rclcpp geometry_msgs)

# Draw Square
add_executable(draw_square src/square/draw_square.cpp)
ament_target_dependencies(draw_square rclcpp geometry_msgs)

add_executable(draw_heart src/heart/draw_heart.cpp)
ament_target_dependencies(draw_heart rclcpp geometry_msgs turtlesim)

# Service / Client in AddTwoInteger
add_executable(increment_publisher src/AddTwoInteger/increment_publisher.cpp)
ament_target_dependencies(increment_publisher rclcpp std_msgs)

add_executable(increment_num5_publisher src/AddTwoInteger/increment_num5_publisher.cpp)
ament_target_dependencies(increment_num5_publisher rclcpp std_msgs)

add_executable(subscriber src/AddTwoInteger/subscriber.cpp)
ament_target_dependencies(subscriber rclcpp std_msgs)

# === INSTALL TARGETS ===
install(TARGETS
  draw_cyc
  draw_square
  draw_heart
  increment_publisher
  increment_num5_publisher
  subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# Required for ROS 2 packages
ament_package()

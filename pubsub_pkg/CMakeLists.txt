cmake_minimum_required(VERSION 3.8)
project(pubsub_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_pkg REQUIRED)  # CHANGE

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp custom_pkg)    # CHANGE

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp custom_pkg)  # CHANGE

add_executable(param src/cpp_parameters_node.cpp)
ament_target_dependencies(param rclcpp custom_pkg)  

install(TARGETS
  talker
  listener
  param
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY                   # for launch files
  launch
  DESTINATION share/${PROJECT_NAME})
  
ament_package()

cmake_minimum_required(VERSION 3.8)
project(vacuum_gripper)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(canopen_interfaces REQUIRED)

include_directories(
  include 
)

add_executable(vacuum_gripper_node 
  src/vacuum_gripper_node.cpp
  src/valve_control.cpp
  src/vacuum_sensor.cpp)
target_include_directories(vacuum_gripper_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(vacuum_gripper_node 
  rclcpp 
  std_msgs
  std_srvs
  sensor_msgs
  canopen_interfaces
)
target_compile_features(vacuum_gripper_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS 
  vacuum_gripper_node
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()

cmake_minimum_required(VERSION 3.5)
project(dynamixel_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_sdk_custom_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dynamixel_workbench_toolbox REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interfaces REQUIRED) #interfaces package



# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${dynamixel_sdk_INCLUDE_DIRS}
  ${dynamixel_sdk_custom_interfaces_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${dynamixel_workbench_toolbox_INCLUDE_DIRS}
  
)

# Build

add_executable(dynamixel_IMU src/dynamixel_IMU.cpp)

# Set include directories for the target
target_include_directories(dynamixel_IMU PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


# Link libraries and dependencies

ament_target_dependencies(dynamixel_IMU
  rclcpp
  dynamixel_sdk
  dynamixel_sdk_custom_interfaces
  dynamixel_workbench_toolbox
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  interfaces
)


# Install executables
install(TARGETS
  dynamixel_IMU
  DESTINATION lib/${PROJECT_NAME}
)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

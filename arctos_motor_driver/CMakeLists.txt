cmake_minimum_required(VERSION 3.5)
project(arctos_motor_driver)

# Set compiler options (optional)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# Create a library for the motor driver
add_executable(arctos_motor_driver
  src/motor_driver.cpp
)

# Specify include directories
target_include_directories(arctos_motor_driver
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Specify dependencies
ament_target_dependencies(arctos_motor_driver
  rclcpp
  can_msgs
  sensor_msgs
)
install(TARGETS
  arctos_motor_driver
  DESTINATION lib/${PROJECT_NAME})

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install config directory
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
#ament_export_targets(export_arctos_motor_driver HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp can_msgs sensor_msgs)

# Export package information
ament_package()

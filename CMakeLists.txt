cmake_minimum_required(VERSION 3.8)
project(jellydrone)

# C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)


# External static serial library
add_library(serial_lib STATIC IMPORTED)
set_target_properties(serial_lib PROPERTIES
  IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/../../install/serial/lib/libserial.a
  INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/../../install/serial/include
)

# ================================
# Executable: serial_bridge
# ================================
add_executable(serial_bridge src/serial_bridge.cpp)
target_include_directories(serial_bridge PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../install/serial/include
)
target_link_libraries(serial_bridge
  serial_lib
  pthread
)
ament_target_dependencies(serial_bridge
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

install(TARGETS serial_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# ================================
# Executable: joystick_servo_control
# ================================
add_executable(joystick_servo_control src/jellydrone_joystick.cpp)
target_include_directories(joystick_servo_control PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../install/serial/include
)
target_link_libraries(joystick_servo_control
  serial_lib
  pthread
)
ament_target_dependencies(joystick_servo_control
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

install(TARGETS joystick_servo_control
  DESTINATION lib/${PROJECT_NAME}
)

# ================================
# Executable: imu_serial_publisher_node
# ================================
add_executable(imu_serial_publisher_node src/imu_serial_publisher_node.cpp)
target_include_directories(imu_serial_publisher_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../install/serial/include
)
target_link_libraries(imu_serial_publisher_node
  serial_lib
  pthread
)
ament_target_dependencies(imu_serial_publisher_node
  rclcpp
  sensor_msgs
)

install(TARGETS imu_serial_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

# ================================
# Executable: camera_node
# ================================
add_executable(camera_node src/camera_node.cpp)
target_link_libraries(camera_node
  ${OpenCV_LIBS}
)
ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

install(TARGETS camera_node
  DESTINATION lib/${PROJECT_NAME}
)

# ================================
# Install launch directory
# ================================
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Finalize package
ament_package()

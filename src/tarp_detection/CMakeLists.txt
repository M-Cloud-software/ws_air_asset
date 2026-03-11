cmake_minimum_required(VERSION 3.8)
project(tarp_detection)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# ── Custom messages ────────────────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  DEPENDENCIES std_msgs sensor_msgs
)

# ── Python package ─────────────────────────────────────────────────────────
ament_python_install_package(${PROJECT_NAME})

# ── Install executables ────────────────────────────────────────────────────
install(PROGRAMS
  tarp_detection/tarp_detection_node.py
  tarp_detection/sitl_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Install launch / config ────────────────────────────────────────────────
install(DIRECTORY launch  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()

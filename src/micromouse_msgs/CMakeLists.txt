cmake_minimum_required(VERSION 3.8)
project(micromouse_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotionSetpoint.msg"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

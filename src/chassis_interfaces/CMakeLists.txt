cmake_minimum_required(VERSION 3.8)
project(chassis_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/VelocityInfo.msg"
  "msg/AccelerationInfo.msg"
  "msg/ActionInfo.msg"
  "msg/ChassisInfo.msg"
)

set(srv_files
  "srv/VolocityControl.srv"
  "srv/AccelerationControl.srv"
  "srv/DiffusionControl.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

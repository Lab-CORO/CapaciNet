cmake_minimum_required(VERSION 3.8)
project(base_placement_interfaces)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Define messages
set(msg_files
  "msg/WsSphere.msg"
  "msg/WorkSpace.msg"
  "msg/PoseNamed.msg"
)

# Define services
set(srv_files
  "srv/UpdateReachabilityMap.srv"
  "srv/GetUnionMap.srv"
  "srv/UpdateParameters.srv"
  "srv/AddNamedPose.srv"
  "srv/RemoveNamedPose.srv"
  "srv/ClearMaps.srv"
  "srv/GetBasePoses.srv"
)

# Define actions
set(action_files
  "action/FindBase.action"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES
    action_msgs
    geometry_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

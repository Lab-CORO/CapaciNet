cmake_minimum_required(VERSION 3.8)
project(base_placement_plugin)

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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(pluginlib REQUIRED)
find_package(data_generation REQUIRED)
find_package(curobo_msgs REQUIRED)
# find_package(moveit_ros_planning_interface REQUIRED)
# find_package(moveit_core REQUIRED)
# find_package(moveit_ros_planning REQUIRED)
# find_package(moveit_visual_tools REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(HighFive REQUIRED)
find_package(PCL REQUIRED COMPONENTS common octree)
find_package(octomap REQUIRED)


# Qt5 dependencies
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Concurrent)

# HDF5 for reachability map loading
find_package(HDF5 REQUIRED COMPONENTS CXX)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${HDF5_INCLUDE_DIRS}
  ${HighFive_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/../data_generation/include
  /home/HighFive/include
)

# Dependencies list for ament
set(dependencies
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  geometry_msgs
  visualization_msgs
  interactive_markers
  rviz_common
  rviz_default_plugins
  rviz_rendering
  pluginlib
#   moveit_ros_planning_interface
#   moveit_core
#   moveit_ros_planning
#   moveit_visual_tools
)

## UI Files
qt5_wrap_ui(UIC_FILES
  src/widgets/base_placement_widget.ui
)

## Header files with Q_OBJECT that need MOC
set(HEADER_FILES_MOC
  include/base_placement_plugin/point_tree_model.h
  include/base_placement_plugin/widgets/base_placement_widget.h
  include/base_placement_plugin/place_base.h
  include/base_placement_plugin/add_way_point.h
  include/base_placement_plugin/add_robot_base.h
)

## Generate MOC files
qt5_wrap_cpp(MOC_FILES ${HEADER_FILES_MOC})

## Source files
set(SOURCE_FILES
  src/place_base.cpp
  src/add_way_point.cpp
  src/add_robot_base.cpp
  src/create_marker.cpp
  src/point_tree_item.cpp
  src/point_tree_model.cpp
  src/sphere_discretization.cpp
  src/widgets/base_placement_widget.cpp
  ${UIC_FILES}
  ${MOC_FILES}
)

## Build the plugin library
add_library(${PROJECT_NAME} SHARED
  ${SOURCE_FILES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
  ${HighFive_LIBRARIES}
  curobo_msgs

)

target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Widgets
  Qt5::Concurrent
  ${HDF5_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  yaml-cpp
  ${HighFive_LIBRARIES}
  data_generation::utils
)

# Prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

## Install
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

## Export information
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
cmake_minimum_required(VERSION 3.8)
project(data_generation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED )
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package (Eigen3 REQUIRED)
# find_package(rosidl_default_generators REQUIRED)
find_package(curobo_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(HDF5 REQUIRED COMPONENTS CXX HL)
find_package(HighFive REQUIRED)
find_package(Open3D REQUIRED)
# find_package( REQUIRED)

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

set(dependencies
        geometry_msgs
)






### create_reachability_map ###
add_executable(create_reachability_map
        src/create_reachability_map.cpp
        include/sphere_discretization.h
        src/sphere_discretization.cpp
        include/progressbar.hpp
        include/utils.h
        src/utils.cpp

)
target_include_directories(create_reachability_map PUBLIC ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(create_reachability_map 
                      ${OCTOMAP_LIBRARIES}
                      ${rclcpp_LIBRARIES}
                      ${HDF5_LIBRARIES}
                      ${HighFive_LIBRARIES}
                    )

ament_target_dependencies(create_reachability_map rclcpp geometry_msgs curobo_msgs tf2_geometry_msgs pcl_ros Eigen3)

### create_inverse_reachability_map ###
add_executable(create_inverse_reachability_map
        src/create_inverse_reachability_map.cpp
        include/sphere_discretization.h
        src/sphere_discretization.cpp
        include/progressbar.hpp
        include/utils.h
        src/utils.cpp

)
target_include_directories(create_inverse_reachability_map PUBLIC ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(create_inverse_reachability_map 
                      ${OCTOMAP_LIBRARIES}
                      ${rclcpp_LIBRARIES}
                      ${HDF5_LIBRARIES}
                      ${HighFive_LIBRARIES}
                    )

ament_target_dependencies(create_inverse_reachability_map rclcpp geometry_msgs curobo_msgs tf2_geometry_msgs pcl_ros Eigen3)

### generate_data ###
add_executable(generate_data
        src/data_generator.cpp
        include/sphere_discretization.h
        src/sphere_discretization.cpp
        include/progressbar.hpp
        include/utils.h
        src/utils.cpp

)
target_include_directories(generate_data PUBLIC ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(generate_data ${OCTOMAP_LIBRARIES})
ament_target_dependencies(generate_data rclcpp std_srvs geometry_msgs curobo_msgs tf2_geometry_msgs pcl_ros Eigen3)
# Lier les bibliothèques nécessaires
target_link_libraries(generate_data
  ${rclcpp_LIBRARIES}
  ${HDF5_LIBRARIES}
  ${HighFive_LIBRARIES}
)

### reachability_map_viz (npz to h5) ###
add_executable(reachability_map_viz
        src/reachability_map_viz.cpp
        include/progressbar.hpp
        include/utils.h
        src/utils.cpp

)
target_include_directories(reachability_map_viz PUBLIC ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(reachability_map_viz 
  ${OCTOMAP_LIBRARIES}   
  ${rclcpp_LIBRARIES}
  ${HDF5_LIBRARIES}
  ${HighFive_LIBRARIES}
  )

ament_target_dependencies(reachability_map_viz rclcpp geometry_msgs visualization_msgs curobo_msgs tf2_geometry_msgs pcl_ros Eigen3)

### Add executable for scene manager
add_executable(scene_manager
                src/scene_manager.cpp
)
target_include_directories(scene_manager PUBLIC )
target_link_libraries(scene_manager)
ament_target_dependencies(scene_manager rclcpp std_srvs curobo_msgs)


### Add executable for obstacle_adder

add_executable(obstacle_adder
        src/obstacle_adder.cpp
)
target_include_directories(obstacle_adder PUBLIC)
target_link_libraries(obstacle_adder)
ament_target_dependencies(obstacle_adder rclcpp std_srvs geometry_msgs visualization_msgs curobo_msgs tf2_geometry_msgs pcl_ros Eigen3)

# Exemple pour highfive
include_directories(
  include
  ${HDF5_INCLUDE_DIRS}
  ${HighFive_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  ${Open3D_INCLUDE_DIRS}
  /home/HighFive/include
)

### Create utils library for use by other packages ###
add_library(utils SHARED
  src/utils.cpp
)
target_include_directories(utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${HDF5_INCLUDE_DIRS}
  ${HighFive_INCLUDE_DIRS}
  /home/HighFive/include
)
target_link_libraries(utils
  ${HDF5_LIBRARIES}
  ${HighFive_LIBRARIES}
)
ament_target_dependencies(utils
  rclcpp
  geometry_msgs
)

install(TARGETS
    utils
    create_reachability_map
    create_inverse_reachability_map
    reachability_map_viz
    generate_data
    obstacle_adder
    scene_manager
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  data
  DESTINATION share/${PROJECT_NAME}/
)

# Export information
ament_export_include_directories(include)
ament_export_libraries(utils)
ament_export_targets(export_${PROJECT_NAME})




ament_package()

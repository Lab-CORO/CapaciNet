cmake_minimum_required(VERSION 3.8)
project(data_generation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED )
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package (Eigen3 REQUIRED)
# find_package(rosidl_default_generators REQUIRED)
find_package(curobo_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

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





ament_package()

add_executable(create_reachability_map
        src/create_reachability_map.cpp
        include/sphere_discretization.h
        src/sphere_discretization.cpp
        include/progressbar.hpp
        src/master_ik_data.cpp
        include/master_ik_data.h
        include/json.hpp
        include/robot.h
        src/robot.cpp
        include/utils.h
        src/utils.cpp

)
target_include_directories(create_reachability_map PUBLIC ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(create_reachability_map ${OCTOMAP_LIBRARIES})
#ament_target_dependencies(create_reachability_map rclcpp ${OCTOMAP_LIBRARIES} geometry_msgs tf2_geometry_msgs pcl_ros Eigen3)

ament_target_dependencies(create_reachability_map rclcpp geometry_msgs curobo_msgs tf2_geometry_msgs pcl_ros Eigen3)

add_executable(generate_data
        src/data_generator.cpp
        include/sphere_discretization.h
        src/sphere_discretization.cpp
        include/progressbar.hpp
        src/master_ik_data.cpp
        include/master_ik_data.h
        include/json.hpp
        include/robot.h
        src/robot.cpp
        include/utils.h
        src/utils.cpp

)
target_include_directories(generate_data PUBLIC ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(generate_data ${OCTOMAP_LIBRARIES})
ament_target_dependencies(generate_data rclcpp geometry_msgs curobo_msgs tf2_geometry_msgs pcl_ros Eigen3)

add_executable(reachability_map_viz
        src/reachability_map_viz.cpp
        include/progressbar.hpp
        src/master_ik_data.cpp
        include/master_ik_data.h
        include/json.hpp
        include/robot.h
        src/robot.cpp
        include/utils.h
        src/utils.cpp

)
target_include_directories(reachability_map_viz PUBLIC ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(reachability_map_viz ${OCTOMAP_LIBRARIES})
ament_target_dependencies(reachability_map_viz rclcpp geometry_msgs visualization_msgs curobo_msgs tf2_geometry_msgs pcl_ros Eigen3)


install(TARGETS
    create_reachability_map
    reachability_map_viz
    generate_data
    DESTINATION lib/${PROJECT_NAME}

 )

install(DIRECTORY
  launch
  data
  DESTINATION share/${PROJECT_NAME}/

)




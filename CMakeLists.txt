cmake_minimum_required(VERSION 3.14)
project(amr_hw2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

# dependencies
find_package(
  catkin REQUIRED
  COMPONENTS std_msgs
             geometry_msgs
             nav_msgs
             mavros_msgs
             message_generation
             roscpp
             tf2
             tf2_ros
             tf2_eigen)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

file(GLOB header_files ${CMAKE_SOURCE_DIR}/include/*.hpp
     ${CMAKE_SOURCE_DIR}/include/utils/*.hpp)

add_message_files(
  DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/msg
  FILES
  PointNormStamped.msg
)
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

function(add_ros_executable name main)
  add_executable(${name} ${CMAKE_CURRENT_SOURCE_DIR}/${main})
  add_dependencies(${name} ${${PROJECT_NAME}_EXPORTED_TARGETS}
                   ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${name} ${catkin_LIBRARIES})
endfunction(add_ros_executable)

add_ros_executable(crane_pose_publisher src/crane_pose_publisher.cpp)
add_ros_executable(map_to_inspection_tf_broadcaster
                   src/map_to_inspection_tf_broadcaster.cpp)
add_ros_executable(map_to_odom_tf_broadcaster
                   src/map_to_odom_tf_broadcaster.cpp)
add_ros_executable(mission_state_machine_node
                   src/mission_state_machine_node.cpp)
add_ros_executable(publish_waypoints_in_rviz src/publish_waypoints_in_rviz.cpp)
target_sources(publish_waypoints_in_rviz PUBLIC src/transformlistener.cpp)

# this needs to be here for rosrun and roslaunch
catkin_package(
  CATKIN_DEPENDS
  std_msgs
  geometry_msgs
  message_runtime
  message_generation
  roscpp
)

# executable scripts
install(PROGRAMS scripts/make_px4_gazebo.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

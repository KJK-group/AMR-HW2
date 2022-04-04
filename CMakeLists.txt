cmake_minimum_required(VERSION 3.14)
project(amr-hw2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

# dependencies
find_package(
  catkin REQUIRED
  COMPONENTS geometry_msgs
             nav_msgs
             mavros_msgs
             roscpp
             tf2
             tf2_ros
             tf2_eigen)
find_package(Eigen3 REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS}
                    ${Eigen3_INCLUDE_DIRS})

function(add_ros_executable name main)
  add_executable(${name} ${CMAKE_CURRENT_SOURCE_DIR}/${main})
  add_dependencies(${name} ${${PROJECT_NAME}_EXPORTED_TARGETS}
                   ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${name} ${catkin_LIBRARIES})
endfunction(add_ros_executable)

add_ros_executable(crane_pose_publisher src/crane_pose_publisher.cpp)

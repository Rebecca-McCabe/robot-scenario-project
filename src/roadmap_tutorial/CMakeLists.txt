cmake_minimum_required(VERSION 2.8.3)
project(roadmap_tutorial)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  trac_ik_lib
  moveit_core
  moveit_ros_planning_interface
  )

find_package(Boost REQUIRED COMPONENTS date_time)
find_package(orocos_kdl REQUIRED)


catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  trac_ik_lib
  moveit_core
  moveit_ros_planning_interface
  DEPENDS
  Boost
  orocos_kdl
  )

include_directories(
${catkin_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
${orocos_kdl_INCLUDE_DIRS}
include
)

add_executable(tutorial_state_publisher src/state_publisher.cpp)
target_link_libraries(tutorial_state_publisher ${catkin_LIBRARIES})

add_executable(build_roadmap
  src/build_roadmap.cpp
  src/IKManager.cpp
  src/EdgeChecker.cpp
  src/utils.cpp
  )
target_link_libraries(build_roadmap
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

add_executable(pub_samples src/viz_samples.cpp)
target_link_libraries(pub_samples ${catkin_LIBRARIES})

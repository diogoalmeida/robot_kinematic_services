cmake_minimum_required(VERSION 2.8.3)
project(robot_kinematic_services)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  generic_control_toolbox
)

add_definitions(-std=c++11)
link_directories(${catkin_LIBRARY_DIRS})

add_message_files(
  FILES
  SolverStatus.msg
)

add_service_files(
  FILES
  ForwardKinematics.srv
  InverseKinematics.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs generic_control_toolbox
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(kinematic_services src/kinematic_services.cpp)
target_link_libraries(kinematic_services ${catkin_LIBRARIES})
add_dependencies(kinematic_services ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

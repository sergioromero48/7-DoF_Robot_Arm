cmake_minimum_required(VERSION 3.0.2)
project(robot_arm_pkg)

## Compile as C++11
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTs
  geometry_msgs
  joint_state_publisher_gui
  roscpp
  rviz
  tf2
  urdf
)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES robot_arm_pkg
  CATKIN_DEPENDS geometry_msgs joint_state_publisher_gui roscpp rviz tf2 urdf
  # DEPENDS system_lib
)

## Specify additional locations of header files
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  $ENV{HOME}/libraries/ik/include
)

link_directories(
  $ENV{HOME}/libraries/ik_build
)

## Declare a C++ executable
# add_executable(${PROJECT_NAME}_node src/marvin_ik.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ik
)

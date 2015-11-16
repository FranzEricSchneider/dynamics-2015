cmake_minimum_required(VERSION 2.8.3)
project(ur5_model)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
	FILES
	JointAngles.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
	CATKIN_DEPENDS std_msgs std_srvs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(arm_visualizer src/arm_visualizer.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(arm_visualizer ${catkin_LIBRARIES})
add_dependencies(arm_visualizer ur5_model_gencpp)

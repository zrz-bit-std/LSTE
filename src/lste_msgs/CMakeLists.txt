cmake_minimum_required(VERSION 3.0.2)
project(lste_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  LsteTask.msg
  LsteDetection.msg
  LsteDetections.msg
  LsteScores.msg
  LsteState.msg
  LstePrompts.msg
  LsteFrontiers.msg
  # 如果你后面加了这些 msg，就取消注释并确保文件存在：
  # LsteSubGoal.msg
  # LsteAccessTopo.msg
  # LsteEnvTopo.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    std_msgs
    geometry_msgs
    message_runtime
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

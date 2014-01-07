cmake_minimum_required(VERSION 2.8.3)
project(strands_qsr_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs std_msgs message_generation)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
 FILES
 ObjectClassification.msg
 BBox.msg
)

## Generate services in the 'srv' folder
add_service_files(
 FILES
 GetGroupClassification.srv
 GetQSRDescription.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES
 geometry_msgs std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES strands_qsr_msgs
#  CATKIN_DEPENDS geometry_msgs std_msgs
#  DEPENDS system_lib
)


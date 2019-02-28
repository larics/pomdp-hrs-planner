cmake_minimum_required(VERSION 2.8.3)
project(pomdp_hrs_planner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
)

add_service_files(
  FILES
  GetNewAction.srv
  PredictNextAction.srv
)

generate_messages()

catkin_package(
 CATKIN_DEPENDS rospy message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

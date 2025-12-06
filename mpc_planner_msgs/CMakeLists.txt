cmake_minimum_required(VERSION 2.8.3)
project(mpc_planner_msgs)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  std_msgs
  geometry_msgs
  message_generation
)

include_directories(include)

add_message_files(
  FILES 
  Gaussian.msg 
  ObstacleGMM.msg 
  ObstacleArray.msg
  MPCMetrics.msg
)

## Declare services
add_service_files(
  FILES
  GetOtherTrajectories.srv
)

generate_messages(DEPENDENCIES nav_msgs std_msgs geometry_msgs)

catkin_package(
  DEPENDS message_runtime nav_msgs std_msgs geometry_msgs 
)

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

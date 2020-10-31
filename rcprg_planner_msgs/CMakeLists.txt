cmake_minimum_required(VERSION 3.0.2)
project(rcprg_planner_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  ReachabilityRangeQuery.msg
)

add_service_files(DIRECTORY srv
  FILES
  ReachabilityRange.srv)

generate_messages(DEPENDENCIES
	geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

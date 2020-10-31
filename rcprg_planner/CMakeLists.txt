cmake_minimum_required(VERSION 2.8)
project(rcprg_planner)

find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  rtt_rosclock
  std_msgs
  interactive_markers
  kdl_conversions
  tf
  eigen_conversions
  cmake_modules
  collision_convex_model
  kin_dyn_model
  planer_utils
  octomap_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  rcprg_planner_msgs
)

find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(octomap REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS collision_convex_model kin_dyn_model planer_utils rcprg_planner_msgs
)

catkin_python_setup()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

link_directories(${orocos_kdl_LIBRARY_DIRS} ${OCTOMAP_LIBRARY_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} ${USE_OROCOS_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS})

add_executable(rcprg_planner src/planner.cpp)
target_link_libraries(rcprg_planner ${catkin_LIBRARIES})

add_executable(workspace_tool src/workspace_tool.cpp)
target_link_libraries(workspace_tool ${catkin_LIBRARIES})

install(TARGETS rcprg_planner workspace_tool
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  )

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

install(DIRECTORY include/rcprg_planner/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


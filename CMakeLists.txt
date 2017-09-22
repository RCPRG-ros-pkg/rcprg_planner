cmake_minimum_required(VERSION 2.8)
project(planner)

find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  rtt_rosclock
  std_msgs
  interactive_markers
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
)

find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(octomap REQUIRED)

catkin_package(
    CATKIN_DEPENDS collision_convex_model kin_dyn_model planer_utils
)

#catkin_python_setup()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

link_directories(${orocos_kdl_LIBRARY_DIRS} ${OCTOMAP_LIBRARY_DIRS})

include_directories(${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} ${USE_OROCOS_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS})

add_executable(planner src/planner.cpp)

target_link_libraries(planner ${catkin_LIBRARIES})

install(TARGETS planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

catkin_install_python(PROGRAMS scripts/planner.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


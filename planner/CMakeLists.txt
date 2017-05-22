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
  planner_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(OMPL REQUIRED)
find_package(octomap REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

link_directories(${orocos_kdl_LIBRARY_DIRS} ${OMPL_LIBRARY_DIRS} ${OCTOMAP_LIBRARY_DIRS})

include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${USE_OROCOS_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS})

#add_executable(test_velma_ompl src/test_ompl.cpp src/ompl_utilities.cpp src/experiments_utilities.cpp src/output_ikfast_velma_right.cpp)
add_executable(planner src/planner.cpp)
#add_executable(test_velma_sim_metric src/velma_sim_metric.cpp src/velma_dyn_model.cpp src/output_ikfast_velma_right.cpp src/experiments_utilities.cpp)
#add_executable(test_velma_sim_octomap src/velma_sim_metric_octomap.cpp src/velma_dyn_model.cpp src/output_ikfast_velma_right.cpp src/experiments_utilities.cpp)
#add_executable(test_velma_rrt src/velma_rrt.cpp src/velma_dyn_model.cpp)
#add_executable(test_velma_rrt2 src/velma_rrt2.cpp src/velma_dyn_model.cpp src/rrt.cpp)
#add_executable(test_velma_rrt3 src/velma_rrt3.cpp src/velma_dyn_model.cpp)
#add_executable(test_velma_est src/velma_est.cpp src/velma_dyn_model.cpp src/est.cpp)
#add_executable(test_velma_rta_star src/velma_rta_star.cpp src/velma_dyn_model.cpp src/rta_star.cpp)
#add_executable(test_velma_learn src/velma_learn.cpp src/velma_dyn_model.cpp)
#add_executable(test_velma_planners_comp src/velma_planners_comp.cpp src/velma_dyn_model.cpp src/output_ikfast_velma_right.cpp src/experiments_utilities.cpp src/ompl_utilities.cpp)

#target_link_libraries(test_velma_ompl ${catkin_LIBRARIES} ${OMPL_LIBRARIES})
target_link_libraries(planner ${catkin_LIBRARIES} ${OMPL_LIBRARIES})
#target_link_libraries(test_velma_sim_metric ${catkin_LIBRARIES} ${OMPL_LIBRARIES})
#target_link_libraries(test_velma_sim_octomap ${catkin_LIBRARIES} ${OMPL_LIBRARIES} ${OCTOMAP_LIBRARIES})
#target_link_libraries(test_velma_rrt ${catkin_LIBRARIES} ${OMPL_LIBRARIES})
#target_link_libraries(test_velma_rrt2 ${catkin_LIBRARIES} ${OMPL_LIBRARIES})
#target_link_libraries(test_velma_rrt3 ${catkin_LIBRARIES} ${OMPL_LIBRARIES})
#target_link_libraries(test_velma_est ${catkin_LIBRARIES} ${OMPL_LIBRARIES})
#target_link_libraries(test_velma_rta_star ${catkin_LIBRARIES} ${OMPL_LIBRARIES})
#target_link_libraries(test_velma_learn ${catkin_LIBRARIES} ${OMPL_LIBRARIES})
#target_link_libraries(test_velma_planners_comp ${catkin_LIBRARIES} ${OMPL_LIBRARIES})

catkin_package(
    DEPENDS ompl eigen
    CATKIN_DEPENDS collision_convex_model kin_dyn_model planer_utils planner_msgs
)


cmake_minimum_required(VERSION 3.0.2)
project(bspline_race)

find_package(NLopt REQUIRED)
set(NLopt_INCLUDE_DIRS ${NLOPT_INCLUDE_DIR})
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  visualization_msgs
  message_generation
  message_filters
  cv_bridge
  # plan_env
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

#add_dependencies(bspline_race ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_message_files(
  FILES
  BsplineTraj.msg
  PositionCommand.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bspline_race
  CATKIN_DEPENDS roscpp rospy tf std_msgs visualization_msgs message_runtime
  #DEPENDS system_lib
)
include_directories(
  SYSTEM
  include 
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${NLOPT_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/devel/include
)
add_library( bspline_race 
   src/UniformBspline.cpp
   src/bspline_opt.cpp 
   src/bspline_race.cpp
   src/gvo.cpp
   src/EdtTransform.cpp
   )
   
add_executable( flag_planner 
    src/test_planning.cpp
 )
target_link_libraries(flag_planner 
 bspline_race
 ${catkin_LIBRARIES}
 ${NLOPT_LIBRARIES}
)

add_executable(traj_server src/traj_server.cpp)
target_link_libraries(traj_server ${catkin_LIBRARIES})  
# add_executable(dynamic_local src/dynamic_local.cpp) 
# target_link_libraries(dynamic_local ${catkin_LIBRARIES}) 

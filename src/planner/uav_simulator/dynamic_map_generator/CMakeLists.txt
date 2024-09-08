cmake_minimum_required(VERSION 2.8.3)
project(map_generator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
ADD_COMPILE_OPTIONS(-std=c++11)
ADD_COMPILE_OPTIONS(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  pcl_conversions
  message_generation
  message_runtime
)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  dynamic_obs.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(random_forest src/random_forest_sensing.cpp)
target_link_libraries(random_forest
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

add_executable(dynamic_forest src/dynamic_forest_sensing.cpp)
target_link_libraries(dynamic_forest
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

add_executable(dynamic_forest_seq src/dynamic_forest_seq_sensing.cpp)
target_link_libraries(dynamic_forest_seq
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES})
  

add_executable(click_map src/click_map.cpp)
target_link_libraries(click_map
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})
  
add_executable(click_map_dynamic src/click_map_dynamic.cpp)
target_link_libraries(click_map_dynamic
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

add_executable(dyn_map src/dyn_map.cpp)
target_link_libraries(dyn_map
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

add_executable(static_dyn_map src/static_dyn_map.cpp)
target_link_libraries(static_dyn_map
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})
  
add_executable (map_recorder src/map_recorder.cpp )
target_link_libraries(map_recorder
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

add_executable (map_pub src/map_publisher.cpp )
target_link_libraries(map_pub
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})
install(TARGETS random_forest click_map
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

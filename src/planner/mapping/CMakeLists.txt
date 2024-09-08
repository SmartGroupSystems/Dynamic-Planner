cmake_minimum_required(VERSION 2.8.3)
project(mapping)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(OpenCV REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
  cv_bridge
  message_filters
  map_generator
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES mapping
 CATKIN_DEPENDS roscpp std_msgs tf map_generator
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}/devel/include
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(mapping src/main.cpp)

target_link_libraries( mapping
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    )  

add_executable(dynamic_local src/dynamic_local.cpp)
target_link_libraries(dynamic_local ${catkin_LIBRARIES}) 
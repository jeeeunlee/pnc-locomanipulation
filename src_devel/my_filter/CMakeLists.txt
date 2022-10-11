cmake_minimum_required(VERSION 3.0.2)
project(my_filter)

set(CMAKE_BUILD_TYPE "Release")
set(PYTHON_EXECUTABLE "/usr/bin/python3")
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
set(CMAKE_MODULE_PATH 
  ${CMAKE_MODULE_PATH} 
  ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)

catkin_package(
  LIBRARIES my_filter
  CATKIN_DEPENDS my_utils
)

set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

FILE(GLOB_RECURSE sources *.cpp)
FILE(GLOB_RECURSE basic_headers "src/Basic/*.hpp")
FILE(GLOB_RECURSE kalman_headers "src/Kalman/*.hpp")

add_library(my_filter SHARED ${sources})
target_link_libraries(my_filter my_utils)
target_include_directories(my_filter PUBLIC   
                      ${PROJECT_INCLUDE_DIR})
#############
## Install ##
#############

#install(TARGETS my_filter DESTINATION "${INSTALL_LIB_DIR}")
#install(FILES ${basic_headers} DESTINATION
#    "${INSTALL_INCLUDE_DIR}/Filter/Basic")
#install(FILES ${kalman_headers} DESTINATION
#    "${INSTALL_INCLUDE_DIR}/Filter/Kalman")




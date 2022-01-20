cmake_minimum_required(VERSION 3.0.2)
project(my_robot_core)

set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
  LIBRARIES my_robot_core
  #CATKIN_DEPENDS my_utils RobotSystem Filter my_wbc Geometry Planner
)

set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(CMAKE_MODULE_PATH 
  ${CMAKE_MODULE_PATH} 
  ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)



file(GLOB_RECURSE sources "*.cpp")
file(GLOB my_robot_core_headers "src/*.hpp")
file(GLOB_RECURSE magneto_my_robot_core_headers "src/Magneto/*.hpp")

#message("${sources}")
#message("${headers}")


add_library(my_robot_core SHARED ${sources}
                              ${headers})
                              
target_link_libraries(my_robot_core ${DART_LIBRARIES}
                                  my_robot_system
                                  my_utils
                                  my_filter                                  
                                  my_wbc
                                  my_geometry)
target_include_directories(my_robot_core PUBLIC   
                            ${PROJECT_INCLUDE_DIR})
# install(TARGETS my_robot_core DESTINATION "${INSTALL_LIB_DIR}")
# install(FILES ${my_robot_core_headers} DESTINATION
#     "${INSTALL_INCLUDE_DIR}/my_robot_core")

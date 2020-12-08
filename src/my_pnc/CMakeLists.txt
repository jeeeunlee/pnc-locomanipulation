cmake_minimum_required(VERSION 3.0.2)
project(my_pnc)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
  LIBRARIES my_pnc
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
file(GLOB my_pnc_headers "src/*.hpp")
file(GLOB_RECURSE magneto_my_pnc_headers "src/Magneto/*.hpp")

#message("${sources}")
#message("${headers}")


add_library(my_pnc SHARED ${sources}
                              ${headers})
                              
target_link_libraries(my_pnc ${DART_LIBRARIES}
                                  my_robot_system
                                  my_utils
                                  my_filter                                  
                                  my_wbc
                                  my_geometry)
target_include_directories(my_pnc PUBLIC   
                            ${PROJECT_INCLUDE_DIR})
# install(TARGETS my_pnc DESTINATION "${INSTALL_LIB_DIR}")
# install(FILES ${my_pnc_headers} DESTINATION
#     "${INSTALL_INCLUDE_DIR}/my_pnc")

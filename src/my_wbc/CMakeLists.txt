cmake_minimum_required(VERSION 3.0.2)
project(my_wbc)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_MODULE_PATH 
  ${CMAKE_MODULE_PATH} 
  ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)

add_subdirectory(ThirdParty/Goldfarb)
# add_subdirectory(ThirdParty/ConicSolver)
# if(GUROBI_FOUND)
#     add_subdirectory(ThirdParty/EigenGurobi)
# endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
  LIBRARIES my_wbc
  #CATKIN_DEPENDS my_utils my_utils myGoldfarb RobotSystem
)

set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(ThirdParty_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/ThirdParty)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_INCLUDE_DIR}
  ${ThirdParty_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)


FILE(GLOB_RECURSE sources *.cpp)

FILE(GLOB contact_headers "src/Contact/*.hpp")
FILE(GLOB task_headers "src/Task/*.hpp")

FILE(GLOB wbc_headers "src/*.hpp")
FILE(GLOB wbdc_headers "src/WBDC/*.hpp")
FILE(GLOB wblc_headers "src/WBLC/*.hpp")
FILE(GLOB wbmc_headers "src/WBMC/*.hpp")
FILE(GLOB wbqpd_headers "src/WBQPD/*.hpp")



add_library(my_wbc SHARED ${sources}
                         ${wbc_headers}
                         ${wbmc_headers}
                         ${wblc_headers}
                         ${wbdc_headers}
                         ${wbqpd_headers}
                         ${contact_headers}
                         ${task_headers})
                         
target_link_libraries(my_wbc my_utils
                            myGoldfarb
                            my_robot_system)

target_include_directories(my_wbc PUBLIC   
                      ${PROJECT_INCLUDE_DIR}
                      ${ThirdParty_INCLUDE_DIR} )

# install(TARGETS my_wbc DESTINATION "${INSTALL_LIB_DIR}")
# install(FILES ${wbc_headers} DESTINATION
#     "${INSTALL_INCLUDE_DIR}/WBC/src")
# install(FILES ${wblc_headers} DESTINATION
#     "${INSTALL_INCLUDE_DIR}/WBC/src/WBLC")
# install(FILES ${wbmc_headers} DESTINATION
#     "${INSTALL_INCLUDE_DIR}/WBC/src/WBMC")
# install(FILES ${wbdc_headers} DESTINATION
#     "${INSTALL_INCLUDE_DIR}/WBC/src/WBDC")
# install(FILES ${contact_headers} DESTINATION
#     "${INSTALL_INCLUDE_DIR}/WBC/src/Contact")
# install(FILES ${task_headers} DESTINATION
#     "${INSTALL_INCLUDE_DIR}/WBC/src/Task")

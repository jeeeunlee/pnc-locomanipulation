cmake_minimum_required(VERSION 3.0.2)
project(my_robot_system)

set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
set(CMAKE_MODULE_PATH 
      ${CMAKE_MODULE_PATH} 
      ${PROJECT_SOURCE_DIR}/cmake)
# message(${CMAKE_MODULE_PATH})
find_package(Eigen3)

catkin_package(
  LIBRARIES my_robot_system
  #CATKIN_DEPENDS my_utils
)


set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)


file(GLOB sources "src/*.cpp")
file(GLOB robot_system_headers "include/my_robot_system/*.hpp")

add_library(my_robot_system SHARED ${sources}
                                 ${robot_system_headers})

target_link_libraries(my_robot_system ${DART_LIBRARIES}
                                    my_utils)
target_include_directories(my_robot_system PUBLIC   
                      ${PROJECT_INCLUDE_DIR})

# install(TARGETS my_robot_system DESTINATION "${INSTALL_LIB_DIR}")
# install(FILES ${robot_system_headers} DESTINATION
#     "${INSTALL_INCLUDE_DIR}/my_robot_system")


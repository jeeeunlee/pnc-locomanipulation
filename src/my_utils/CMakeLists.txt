cmake_minimum_required(VERSION 3.0.2)
project(my_utils)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
add_subdirectory(ThirdParty/myYaml)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

set(CMAKE_MODULE_PATH 
      ${CMAKE_MODULE_PATH} 
      ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
find_package(ZMQ)

catkin_package(
  LIBRARIES my_utils
  #CATKIN_DEPENDS my_utils RobotSystem Filter my_wbc Geometry Planner
)

set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(ThirdParty_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/ThirdParty/myYaml/include)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_INCLUDE_DIR}
  ${ThirdParty_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

FILE(GLOB_RECURSE sources "*.cpp")
FILE(GLOB general_headers "include/my_utils/General/*.hpp")
FILE(GLOB io_headers "include/my_utils/IO/*.hpp")
FILE(GLOB math_headers "include/my_utils/Math/*.hpp" "*.h")

if(NOT ZMQ_FOUND)
    list(REMOVE_ITEM sources ${CMAKE_CURRENT_SOURCE_DIR}/include/my_utils/IO/ZmqUtilities.cpp)
    list(REMOVE_ITEM io_headers ${CMAKE_CURRENT_SOURCE_DIR}/include/my_utils/IO/ZmqUtilities.hpp)
endif()

add_library(my_utils SHARED  ${sources})
target_link_libraries(my_utils 
                      myYaml
                      ${EIGEN_LIBRARIES})
target_include_directories(my_utils PUBLIC   
                      ${PROJECT_INCLUDE_DIR}
                      ${ThirdParty_INCLUDE_DIR})

find_package(Threads REQUIRED)
if(THREADS_HAVE_PTHREAD_ARG)
  target_compile_options(my_utils PUBLIC "-pthread")
endif()
if(CMAKE_THREAD_LIBS_INIT)
  target_link_libraries(my_utils "${CMAKE_THREAD_LIBS_INIT}")
endif()



# install(TARGETS my_utils DESTINATION "${INSTALL_LIB_DIR}")
# install(FILES ${general_headers} DESTINATION "${INSTALL_INCLUDE_DIR}/Utils/src/General")
# install(FILES ${io_headers} DESTINATION "${INSTALL_INCLUDE_DIR}/Utils/src/IO")
# install(FILES ${math_headers} DESTINATION "${INSTALL_INCLUDE_DIR}/Utils/src/Math")

SET (THIS_COM ${CMAKE_SOURCE_DIR}/../ )
CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/config.h.cmake ${PROJECT_SOURCE_DIR}/Configuration.h)
# install(FILES ${PROJECT_SOURCE_DIR}/Configuration.h DESTINATION "${INSTALL_INCLUDE_DIR}")
cmake_minimum_required(VERSION 3.0.2)
project(my_geometry)

set(CMAKE_BUILD_TYPE "Release")
set(PYTHON_EXECUTABLE "/usr/bin/python3")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

add_subdirectory(ThirdParty/CDD)

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
find_package(MOSEK)

catkin_package(
  LIBRARIES my_geometry
  #CATKIN_DEPENDS myCdd
)
set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(ThirdParty_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/ThirdParty/CDD/src)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${ThirdParty_INCLUDE_DIR}
)

add_library(my_geometry SHARED 
            src/Polytope/Polytope.cpp 
            include/my_geometry/Polytope/Polytope.h)
target_link_libraries(my_geometry myCdd ${EIGEN_LIBRARIES})
target_include_directories(my_geometry PUBLIC   
                      ${PROJECT_INCLUDE_DIR})

if(MOSEK_FOUND)
    add_library(myLownerJohnEllipsoid LownerJohnEllipsoid.cpp
                                      LownerJohnEllipsoid.hpp
                                      ${MOSEK_LIBRARIES})
    target_link_libraries(my_geometry myLownerJohnEllipsoid)
endif()

# install(TARGETS my_geometry DESTINATION "${INSTALL_LIB_DIR}")
# install(FILES Polytope.h DESTINATION
#     "${INSTALL_INCLUDE_DIR}/my_geometry/Polytope")


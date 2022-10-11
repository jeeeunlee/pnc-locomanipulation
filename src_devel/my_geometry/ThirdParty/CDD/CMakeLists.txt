cmake_minimum_required(VERSION 3.0.2)
project (myCdd)

set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/src)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_INCLUDE_DIR}
)

FILE(GLOB_RECURSE headers "*.h" "*.hpp")
file(GLOB_RECURSE sources *.c)

add_library (myCdd SHARED ${headers} ${sources})
target_include_directories(myCdd PUBLIC   
                      ${PROJECT_INCLUDE_DIR})



#install(TARGETS myCdd DESTINATION "${INSTALL_LIB_DIR}")
#install(FILES ${headers} DESTINATION
#    "${INSTALL_INCLUDE_DIR}/Geometry/ThirdParty/CDD/src")

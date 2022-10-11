cmake_minimum_required(VERSION 3.0.2)
project (myGoldfarb)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
)

find_package(Eigen3)
include_directories(
  ${EIGEN3_INCLUDE_DIR}
)

FILE(GLOB headers "*.hh" "*.h" "*.hpp")
FILE(GLOB_RECURSE sources *.cc)
add_library (myGoldfarb SHARED ${headers} ${sources})

# install(TARGETS myGoldfarb DESTINATION "${INSTALL_LIB_DIR}")
# install(FILES ${headers} DESTINATION
#     "${INSTALL_INCLUDE_DIR}/ExternalSource/myOptimizer/Goldfarb")
